# Detect Bounding-Box Modal — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render a modal over the live camera feed that shows the frozen detection frame with bboxes drawn on it.

**Spec:** `docs/superpowers/specs/2026-04-13-detect-bbox-modal-design.md`

**Baseline:** `07746f2` (last known good — 6/6 detect triggers stable)

---

### Task 1: Firmware — cache last detection result in detect_task

**Files:**
- Modify: `main/detect_task.cpp`
- Modify: `main/detect_task.h`

- [ ] **Step 1: Add result-cache types and extern getter to `detect_task.h`**

Add a C-linkage API to read the cached result. Define a plain-C struct with the bbox fields so `http_server.c` (C) can consume it.

```c
typedef struct {
    int32_t category;
    float   score;
    int32_t x1, y1, x2, y2;
} detect_item_t;

typedef struct {
    uint64_t      frame_ts_us;
    uint32_t      seq;
    int16_t       count;        // 0..10
    detect_item_t items[10];
} detect_result_snapshot_t;

#ifdef __cplusplus
extern "C" {
#endif

/** Fill *out with the latest detection result if seq > since, return ESP_OK.
 *  Return ESP_ERR_NOT_FOUND if no newer result.
 *  Thread-safe; takes an internal mutex. */
esp_err_t detect_get_result_snapshot(uint32_t since, detect_result_snapshot_t *out);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Add static cache + mutex in `detect_task.cpp`**

File-scope:

```c
static SemaphoreHandle_t s_detect_mu = NULL;
static detect_result_snapshot_t s_last_detect = { 0 };
```

Create the mutex in `detect_init()` before the task is created: `s_detect_mu = xSemaphoreCreateMutex();` with an `if (!s_detect_mu) return ESP_ERR_NO_MEM;` guard.

- [ ] **Step 3: Capture `frame_ts_us` at Step 0 of detect_task_fn**

At the top of the inference branch (right after `ulTaskNotifyTake`), save `uint64_t frame_ts_us = esp_timer_get_time();` so the cache entry reflects when the user's click was received, not when publishing happens.

- [ ] **Step 4: Write the cache after inference, before `pipeline_publish_sensors`**

Where the code currently fills `snap.detections[]`, also populate `s_last_detect` under mutex. Copy category/score/x1/y1/x2/y2 verbatim (coords are in post-rotation 800×640 space — do NOT transform here; dashboard handles rotation).

```c
xSemaphoreTake(s_detect_mu, portMAX_DELAY);
s_last_detect.frame_ts_us = frame_ts_us;
s_last_detect.seq++;
s_last_detect.count = (int16_t)snap.detections_count;
for (size_t i = 0; i < snap.detections_count && i < 10; i++) {
    s_last_detect.items[i].category = snap.detections[i].category;
    s_last_detect.items[i].score    = snap.detections[i].score;
    s_last_detect.items[i].x1       = snap.detections[i].x1;
    s_last_detect.items[i].y1       = snap.detections[i].y1;
    s_last_detect.items[i].x2       = snap.detections[i].x2;
    s_last_detect.items[i].y2       = snap.detections[i].y2;
}
xSemaphoreGive(s_detect_mu);
```

- [ ] **Step 5: Implement `detect_get_result_snapshot`**

```c
extern "C" esp_err_t detect_get_result_snapshot(uint32_t since, detect_result_snapshot_t *out)
{
    if (!out || !s_detect_mu) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_detect_mu, portMAX_DELAY);
    esp_err_t ret = ESP_ERR_NOT_FOUND;
    if (s_last_detect.seq > since) {
        *out = s_last_detect;  // struct copy under mutex
        ret = ESP_OK;
    }
    xSemaphoreGive(s_detect_mu);
    return ret;
}
```

- [ ] **Step 6: Build — verify clean compile**

`idf.py build` should succeed with no warnings. If C/C++ linkage errors around detect_item_t, double-check `extern "C"` wrapping.

---

### Task 2: Firmware — HTTP /detect_result endpoint

**Files:**
- Modify: `main/http_server.c`

- [ ] **Step 1: Include `detect_task.h` at top of `http_server.c`**

- [ ] **Step 2: Add `detect_result_handler`**

Parse `?since=N` from query string (use `httpd_req_get_url_query_len` + `httpd_req_get_url_query_str` + `httpd_query_key_value`). Default to 0 if missing/malformed.

Call `detect_get_result_snapshot(since, &snap)`. On `ESP_ERR_NOT_FOUND` → HTTP 204, empty body, return.

On success, build JSON into a bounded `char buf[768]` (10 detections × ~60 bytes each + envelope). Use `snprintf` with cursor tracking. No cJSON.

```c
static esp_err_t detect_result_handler(httpd_req_t *req)
{
    // parse since
    uint32_t since = 0;
    size_t qlen = httpd_req_get_url_query_len(req);
    if (qlen > 0 && qlen < 64) {
        char query[64];
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
            char val[16];
            if (httpd_query_key_value(query, "since", val, sizeof(val)) == ESP_OK) {
                since = (uint32_t)strtoul(val, NULL, 10);
            }
        }
    }

    detect_result_snapshot_t snap;
    if (detect_get_result_snapshot(since, &snap) != ESP_OK) {
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, NULL, 0);
    }

    char buf[768];
    int n = snprintf(buf, sizeof(buf),
        "{\"seq\":%u,\"frame_ts_us\":%llu,\"detections\":[",
        (unsigned)snap.seq, (unsigned long long)snap.frame_ts_us);
    for (int i = 0; i < snap.count && n < (int)sizeof(buf) - 96; i++) {
        n += snprintf(buf + n, sizeof(buf) - n,
            "%s{\"category\":%d,\"score\":%.3f,\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d}",
            i == 0 ? "" : ",",
            (int)snap.items[i].category, snap.items[i].score,
            (int)snap.items[i].x1, (int)snap.items[i].y1,
            (int)snap.items[i].x2, (int)snap.items[i].y2);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}
```

- [ ] **Step 3: Register the URI in `http_server_start`**

```c
const httpd_uri_t detect_result_uri = {
    .uri = "/detect_result", .method = HTTP_GET,
    .handler = detect_result_handler, .user_ctx = NULL,
};
httpd_register_uri_handler(server, &detect_result_uri);
```

Also bump `cfg.max_uri_handlers` from 4 to 5 (dashboard `/`, ws `/ws`, snapshot `/snapshot`, detect result `/detect_result`, + 1 reserved).

- [ ] **Step 4: Build + flash + smoke test**

After flashing, from host: `curl http://192.168.1.201/detect_result?since=0` should return HTTP 204 on a fresh boot (no inference yet). After clicking Detect once, same curl should return HTTP 200 with JSON body including `seq: 1`.

---

### Task 3: Dashboard — HTML structure + crossorigin attribute

**Files:**
- Modify: `main/dashboard.html`

- [ ] **Step 1: Read current dashboard.html and locate the live feed `<img>` element**

Identify the MJPEG `<img>` tag (likely id `cam-img` or similar). Confirm whether `crossorigin` attribute is present.

- [ ] **Step 2: Add `crossorigin="anonymous"` to the MJPEG `<img>`**

If the dashboard currently has the attribute, leave it alone. If missing, add it. This is load-bearing — without it, `drawImage` taints the canvas.

- [ ] **Step 3: Add hidden snapshot helper canvas**

After the live feed container, add:

```html
<canvas id="detect-snap-src" width="800" height="640" hidden></canvas>
```

This is the "copy from stream" scratch area. Width/height are fixed to the camera's native 800×640.

- [ ] **Step 4: Add modal HTML**

Place near end of `<body>`:

```html
<div id="detect-modal" class="detect-modal" hidden role="dialog" aria-modal="true" aria-label="Detection result">
  <div class="detect-modal-backdrop"></div>
  <div class="detect-modal-body">
    <button type="button" class="detect-modal-close" aria-label="Close">×</button>
    <div class="detect-modal-canvas-wrap">
      <canvas id="detect-display" width="800" height="640"></canvas>
    </div>
    <div class="detect-modal-stats"></div>
  </div>
</div>
```

- [ ] **Step 5: Add CSS**

- `.detect-modal` is fixed positioning, full-viewport, z-index above the live feed, initially `hidden`
- `.detect-modal-backdrop` is `rgba(0,0,0,0.7)`, full cover
- `.detect-modal-body` is centered, max-width ~90vw, dark panel with border
- `#detect-display` has `transform: rotate(180deg)` so the captured pixels match the user's visual view
- `.detect-modal-close` top-right absolute
- `.hidden` utility (or keep using the `hidden` attribute) for show/hide

Reuse existing dashboard style tokens/colors if present.

---

### Task 4: Dashboard — JavaScript controller

**Files:**
- Modify: `main/dashboard.html`

- [ ] **Step 1: Add `rotate180Bbox` helper + load-time assertion**

```js
function rotate180Bbox(x1, y1, x2, y2, W, H) {
  return { x1: W - x2, y1: H - y2, x2: W - x1, y2: H - y1 };
}
(function assertRotation() {
  const t = rotate180Bbox(0, 0, 100, 100, 800, 640);
  console.assert(t.x1 === 700 && t.y1 === 540 && t.x2 === 800 && t.y2 === 640,
                 "rotation transform broken");
})();
```

- [ ] **Step 2: Implement `snapshotCurrentFrame`**

```js
function snapshotCurrentFrame() {
  const img = document.getElementById('cam-img');
  const src = document.getElementById('detect-snap-src');
  const dst = document.getElementById('detect-display');
  if (!img || !img.naturalWidth) throw new Error('no live frame');

  const sctx = src.getContext('2d');
  sctx.drawImage(img, 0, 0, 800, 640);

  const dctx = dst.getContext('2d');
  dctx.drawImage(src, 0, 0, 800, 640);
  return performance.now();  // snapshotTs
}
```

- [ ] **Step 3: Implement `drawBboxes`**

```js
function drawBboxes(detections) {
  const canvas = document.getElementById('detect-display');
  const ctx = canvas.getContext('2d');
  ctx.lineWidth = 4;
  ctx.font = '20px sans-serif';
  ctx.textBaseline = 'top';

  for (const d of detections) {
    const r = rotate180Bbox(d.x1, d.y1, d.x2, d.y2, 800, 640);
    // Simple color scheme for v1 — class-aware colors in Phase 2
    const color = d.category === 0 ? '#FF3838' : '#00E676';
    ctx.strokeStyle = color;
    ctx.strokeRect(r.x1, r.y1, r.x2 - r.x1, r.y2 - r.y1);

    const label = `cat ${d.category} ${d.score.toFixed(2)}`;
    const tw = ctx.measureText(label).width;
    ctx.fillStyle = color;
    ctx.fillRect(r.x1, r.y1, tw + 8, 24);
    ctx.fillStyle = '#000';
    ctx.fillText(label, r.x1 + 4, r.y1 + 2);
  }
}
```

- [ ] **Step 4: Implement `DetectController` class**

Fields: `state` ('IDLE'|'ARMING'|'DISPLAY'), `clickTs`, `lastSeenSeq`, `pollTimer`, `deadlineTs`.

Methods:
- `onClickDetect()` — guard state === IDLE; `clickTs = snapshotCurrentFrame()`; state='ARMING'; send `DetectCommand` over WS (reuse existing WS helper); disable button; `deadlineTs = clickTs + 10000`; start polling.
- `startPolling()` — `setInterval(() => this.pollOnce(), 1000);` in pollTimer.
- `pollOnce()` — if `performance.now() > deadlineTs`, stop + show timeout toast; if WS not OPEN, skip this tick; otherwise `fetch('/detect_result?since=' + this.lastSeenSeq)` and handle response.
- `handlePollResponse(res)` — 204 → return; 200 → parse JSON; if `json.seq > lastSeenSeq && json.frame_ts_us >= clickTsUs` → stop polling, show modal.
- `showModal(detections)` — `drawBboxes(detections)`; show stats line; reveal modal; focus close button.
- `dismissModal()` — hide modal, clear canvas, state = IDLE, re-enable button.

*Note on timestamp units:* `clickTs` is `performance.now()` (ms, client clock). `frame_ts_us` is `esp_timer_get_time()` (µs, device clock). Since the clocks are independent, the `frame_ts_us >= clickTs` guard cannot be a direct comparison. Fix: at page load, `fetch /detect_result?since=0` once to capture the current device seq (the "baseline seq"), store as `lastSeenSeq`. On click, we already have `lastSeenSeq`; the guard becomes simply `json.seq > lastSeenSeq` since the cache only bumps seq when a new inference completes AFTER our click (because the button was disabled between our click and the response). The `frame_ts_us` field is included in the response for diagnostics/logging but not used as a correlation guard.

- [ ] **Step 5: Wire up event listeners**

On DOMContentLoaded:

```js
const controller = new DetectController();
// One-time fetch at load to sync baseline seq (without opening modal)
controller.syncBaselineSeq();

document.getElementById('detect-btn').addEventListener('click', () => controller.onClickDetect());
document.querySelector('.detect-modal-close').addEventListener('click', () => controller.dismissModal());
document.querySelector('.detect-modal-backdrop').addEventListener('click', () => controller.dismissModal());
document.addEventListener('keydown', e => {
  if (e.key === 'Escape' && controller.state === 'DISPLAY') controller.dismissModal();
});
```

`syncBaselineSeq` does a single `GET /detect_result?since=0`, stores `lastSeenSeq = response.seq` if 200, leaves at 0 if 204. Never opens the modal from this call.

- [ ] **Step 6: Smoke test in browser**

Load dashboard. Verify in console: no rotation assertion failure. Click Detect — modal should open with boxes (or empty canvas if 0 detections). Press Escape — modal closes. Click Detect again — new modal opens. No DevTools red lines during the WS-down window (polling paused correctly).

---

### Task 5: On-device visual alignment test

**Files:**
- No code changes; hardware procedure

- [ ] **Step 1: Place a high-contrast object in a known corner of the camera view**

For example, a red marker in the top-left of what you see in the LIVE dashboard feed.

- [ ] **Step 2: Click Detect**

- [ ] **Step 3: Verify the bbox appears in the same corner you see live**

If the bbox appears in the **opposite** corner (bottom-right), the rotation transform is inverted. Double-check `rotate180Bbox` formula or CSS rotation on `#detect-display`.

- [ ] **Step 4: Multi-trigger stability**

Click Detect 5 times in a row (dismissing each modal). All 5 should render correctly, each with its own fresh frame. Verifies `since=`/`seq` correlation.

---

### Task 6: Commit

- [ ] **Step 1: Commit firmware changes**

```bash
git add main/detect_task.cpp main/detect_task.h main/http_server.c
git commit -m "$(cat <<'EOF'
feat(detect): cache last detection + GET /detect_result endpoint

Detect task now writes the latest inference result (bbox coords + score
+ frame timestamp) to a mutex-guarded static cache. Exposes a C getter
detect_get_result_snapshot(since, out) that returns the cached result
only if seq > since.

New HTTP endpoint GET /detect_result?since=<N> on port 80 serves the
cache as JSON (or HTTP 204 if nothing newer). Reconnect-tolerant
delivery path for the dashboard bbox modal — survives the ~5-6 s WS
reconnect window after esp_wifi_disconnect().

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

- [ ] **Step 2: Commit dashboard changes**

```bash
git add main/dashboard.html
git commit -m "$(cat <<'EOF'
feat(dashboard): detect bbox modal with client-side snapshot

Click Detect → dashboard drawImage-snapshots the current MJPEG frame to
a canvas immediately, sends WS DetectCommand, then polls
GET /detect_result?since=<seq> every 1s. First response with new seq
opens a modal over the live feed showing the frozen frame with bbox
overlays. Live MJPEG keeps streaming behind the modal; dismiss via
X / backdrop / Escape.

Rotation: canvas CSS-rotated 180° to match physical camera orientation;
bbox coords flipped client-side via rotate180Bbox(). Load-time
assertion verifies the transform.

Polling pauses while WS is closed (avoids spamming requests during
WiFi reconnect). Baseline seq fetched at page load to seed the
lastSeenSeq correlation — no auto-open on page load.

Crossorigin="anonymous" added to stream img so drawImage does not
taint the canvas.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

- [ ] **Step 3: Verify working tree clean and push-ready**

`git status` → clean. `git log --oneline -5` → both new commits present on top of `07746f2`.

---

## Rollback

Each task is in its own commit. If any step breaks the build or the runtime behavior, `git revert <sha>` brings us back to the `07746f2` baseline with 6/6 detect triggers still working. The firmware cache and HTTP endpoint are purely additive; removing them does not affect inference correctness.
