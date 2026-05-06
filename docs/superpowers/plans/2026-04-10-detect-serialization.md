# /detect Serialization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `GET /detect` on port 82 safe against rapid-fire client requests by serializing the handler with a non-blocking FreeRTOS mutex (429 on contention), and update the dashboard to remove its "max 5 rounds" workaround and retry transparently on 429.

**Architecture:** Pure handler-layer hardening. One static `SemaphoreHandle_t` in `detect_server.c` guards `detect_handler()` (not `detect_image_handler`, which is a rodata byte dump). Non-blocking `xSemaphoreTake(..., 0)` returns `pdFALSE` → handler writes 429 + static JSON literal. The handler body is extracted into a `do_detect_locked()` helper so the give-side is a single line in the caller — no "forgot to release" paths. Dashboard's `runDetect()` gains a 429-aware retry loop (50 ms backoff, 20-retry budget per round, ~1 s max backoff) and loses the `max="5"` input cap + tooltip + JS `Math.min(5, …)` hardcode.

**Tech Stack:** ESP-IDF 5.4.3, C, FreeRTOS `semphr.h`, `esp_http_server`, embedded HTML/JS (dashboard delivered via `EMBED_TXTFILES`).

**Spec:** [docs/superpowers/specs/2026-04-10-detect-serialization-design.md](../specs/2026-04-10-detect-serialization-design.md)

---

## Preamble: Environment setup (once, before Task 1)

- [ ] **Step 0.1: Set the boat IP variable for this session**

All subsequent curl commands reference `$BOAT_IP`. Set it once:

```bash
export BOAT_IP=<boat ip here, e.g. 192.168.1.42>
```

If you don't know the boat's IP, check the serial log during boot for the `WIFI got ip:` line, or use `idf.py monitor` briefly and then detach.

- [ ] **Step 0.2: Verify you are on the `feat/detect-phase1` branch**

Run:

```bash
git status
```

Expected: `On branch feat/detect-phase1`. If not, stop and check with the user before continuing — this plan builds on the Phase 1 detect infrastructure and must not land on `main`.

- [ ] **Step 0.3: Confirm the current state builds cleanly**

Run:

```bash
idf.py build 2>&1 | tail -5
```

Expected: `Project build complete.` with no errors. If this fails, the baseline is broken — stop and fix the baseline before starting this plan.

---

## Task 1: Server-side mutex + 429 busy response

**Rationale:** The one change that actually makes rapid-fire safe. Adds a non-blocking FreeRTOS mutex around the handler body and a static 429 JSON literal. Refactors the existing handler into a helper so the mutex give is guaranteed on every exit path from one place.

**Files:**
- Modify: `main/detect_server.c` — add semphr include, static mutex, kBusyJson literal, refactor handler into helper, wrap with take/give, create mutex in `detect_server_start()`

- [ ] **Step 1.1: Read the current `main/detect_server.c` to confirm its shape**

Run:

```bash
wc -l main/detect_server.c
```

Expected: `134 main/detect_server.c` (or very close). If the file is significantly different in size, stop and reconcile with the user — this plan was written against the post-`4a2f539` version.

- [ ] **Step 1.2: Replace `main/detect_server.c` with the serialized version**

The refactor is extensive enough that a full rewrite is clearer than a pile of small edits. Overwrite `main/detect_server.c` with the content below:

```c
// SPDX-License-Identifier: MIT
//
// detect_server.c — dedicated httpd instance for /detect inference.
//
// Why a separate server:
//
// The port-80 httpd is single-worker. It serves the dashboard (/),
// a long-lived WebSocket (/ws) broadcasting protobuf sensor frames at
// 20 Hz, and used to also serve /detect. When an HTTP client hit
// /detect, the single httpd worker blocked inside s_detect->run() for
// ~400ms (first call, model load) or ~56ms (steady state). During that
// blocking window, WS keep-alives and incoming frames could not be
// serviced; under real usage that opened a stale-fd race that
// eventually corrupted the heap and panic'd Core 1 inside esp-dl's
// next allocation (observed MEPC=0x4ff0f6a0 / MEPC=0x00222318).
//
// camera_stream.c already solved this exact problem for the MJPEG
// stream by running on its own httpd instance on CONFIG_HTTP_STREAM_PORT
// (81). This file applies the same pattern to /detect:
//
//   Port 80  →  dashboard / WS            (existing http_server.c)
//   Port 81  →  MJPEG stream              (existing camera_stream.c)
//   Port 82  →  /detect, /detect/image.jpg (THIS FILE)
//
// CORS: the dashboard loads from origin http://<boat>:80 and fetches
// /detect from http://<boat>:82 — that's a cross-origin request, so
// every response here sets Access-Control-Allow-Origin: *.
//
// Rapid-fire serialization (2026-04-10):
//
// Isolating /detect on its own httpd did not make it safe against
// back-to-back requests — esp-dl panics under overlapping calls
// regardless of which httpd hosts the handler. s_detect_mutex below
// serializes the handler so overlapping calls return HTTP 429 instead
// of crashing the device. See docs/superpowers/specs/
// 2026-04-10-detect-serialization-design.md.

#include "detect_server.h"
#include "detection/detection.h"

#include "esp_log.h"
#include "esp_http_server.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "DETECT_SRV";

/* Serializes detect_handler() against rapid-fire clients. Created once
 * in detect_server_start() before httpd_start(). Non-blocking take:
 * if another request is in flight, we return 429 immediately.
 *
 * Does NOT guard detect_image_handler() — that handler is a rodata
 * byte dump with no inference state and no heap pressure. */
static SemaphoreHandle_t s_detect_mutex = NULL;

/* Static 429 body. Shape matches build_error_json()'s {ok,stage,error}
 * contract so clients don't need a special parser. "busy" is a
 * transport-layer state, NOT a detection pipeline stage, so it does
 * not get added to detection_stage_t in detection.h. */
static const char kBusyJson[] =
    "{\"ok\":false,\"stage\":\"busy\",\"error\":\"detect already in flight\"}";
static const size_t kBusyJsonLen = sizeof(kBusyJson) - 1;  /* exclude NUL */

/* Body of detect_handler() extracted so the mutex give in the caller
 * is a single line. All existing 200/500 behavior is preserved here
 * exactly — only the top-level take/give wrapping is new. */
static esp_err_t do_detect_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded(&out);
    if (ret != ESP_OK) {
        /* detection_run_on_embedded always returns ESP_OK in phase 1,
         * but be defensive in case that changes. */
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send_500(req);
        return ret;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_status(req,
        out.http_status == 200 ? "200 OK" : "500 Internal Server Error");

    if (out.json && out.json_len > 0) {
        /* detection.cpp guarantees non-NULL json via static fallback on
         * double OOM, but keep the guard for robustness. */
        esp_err_t send_ret = httpd_resp_send(req, out.json, out.json_len);
        detection_response_free(&out);
        return send_ret;
    }

    detection_response_free(&out);
    httpd_resp_send_500(req);
    return ESP_OK;
}

static esp_err_t detect_handler(httpd_req_t *req)
{
    /* Non-blocking take. If another request is in flight, we do NOT
     * wait — return 429 immediately so the client can retry. */
    if (xSemaphoreTake(s_detect_mutex, 0) != pdTRUE) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_status(req, "429 Too Many Requests");
        /* httpd_resp_send accepts a const char * + length, safe for a
         * static literal in rodata. */
        return httpd_resp_send(req, kBusyJson, kBusyJsonLen);
    }

    esp_err_t ret = do_detect_locked(req);
    xSemaphoreGive(s_detect_mutex);
    return ret;
}

static esp_err_t detect_image_handler(httpd_req_t *req)
{
    size_t len = 0;
    const unsigned char *bytes = detection_get_embedded_image(&len);
    if (!bytes || len == 0) {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    /* Embedded JPEG never changes during a firmware session. */
    httpd_resp_set_hdr(req, "Cache-Control",
                       "public, max-age=3600, immutable");
    return httpd_resp_send(req, (const char *)bytes, len);
}

static const httpd_uri_t s_detect_uri = {
    .uri      = "/detect",
    .method   = HTTP_GET,
    .handler  = detect_handler,
    .user_ctx = NULL,
};

static const httpd_uri_t s_detect_image_uri = {
    .uri      = "/detect/image.jpg",
    .method   = HTTP_GET,
    .handler  = detect_image_handler,
    .user_ctx = NULL,
};

esp_err_t detect_server_start(void)
{
    /* Create the serialization mutex BEFORE starting httpd, so the
     * first accepted request always sees a live mutex. One-shot; no
     * destroy path — this server lives for the entire process. */
    if (s_detect_mutex == NULL) {
        s_detect_mutex = xSemaphoreCreateMutex();
        if (s_detect_mutex == NULL) {
            ESP_LOGE(TAG, "xSemaphoreCreateMutex failed");
            return ESP_ERR_NO_MEM;
        }
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = CONFIG_HTTP_DETECT_PORT;
    cfg.ctrl_port        = 32770;  /* distinct from :80 (32768) and :81 (32769) */
    cfg.stack_size       = 16384;  /* C++ inference call path needs headroom */
    cfg.max_open_sockets = 3;
    cfg.max_uri_handlers = 4;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &s_detect_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register /detect failed: %s", esp_err_to_name(ret));
        httpd_stop(server);
        return ret;
    }

    ret = httpd_register_uri_handler(server, &s_detect_image_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register /detect/image.jpg failed: %s",
                 esp_err_to_name(ret));
        httpd_stop(server);
        return ret;
    }

    ESP_LOGI(TAG, "Detect server ready on port %d: /detect, /detect/image.jpg",
             CONFIG_HTTP_DETECT_PORT);
    return ESP_OK;
}
```

- [ ] **Step 1.3: Build the firmware**

Run:

```bash
idf.py build 2>&1 | tee /tmp/task1_build.log | tail -20
```

Expected: `Project build complete.` with no warnings from `detect_server.c`. If there are warnings about unused functions or missing headers, re-check that the `freertos/semphr.h` include is present and that `do_detect_locked` is `static`.

- [ ] **Step 1.4: Flash and monitor briefly**

Run:

```bash
idf.py flash 2>&1 | tee /tmp/task1_flash.log | tail -10
```

Expected: `Hash of data verified.` followed by `Leaving...`. If flashing fails, check the USB connection and re-run.

Then briefly monitor to confirm the server starts without crashing at boot:

```bash
timeout 40 idf.py monitor 2>&1 | grep -E "DETECT_SRV|xSemaphore|MAIN: System running|abort|assert"
```

Expected lines:
```
I (....) DETECT_SRV: Detect server ready on port 82: /detect, /detect/image.jpg
I (....) MAIN: System running.
```

Expected **non-lines** (must NOT appear): any `abort`, `assert failed`, `xSemaphoreCreateMutex failed`, or Guru Meditation output. If any appear, capture the full serial log and stop.

- [ ] **Step 1.5: Single-call smoke test (baseline unchanged)**

Run:

```bash
curl -sS -w '\nHTTP %{http_code} in %{time_total}s\n' http://$BOAT_IP:82/detect
```

Expected: HTTP 200 with a JSON body containing `"ok":true` and at least one detection. First-call latency ~500–700 ms (cold model load), subsequent calls ~56 ms. If 429, something is holding the mutex from a previous session — reboot the boat (power cycle) and retry. If 500 or reboot, stop and check the serial log.

- [ ] **Step 1.6: Concurrent-call contention test (the whole point of this task)**

Run TWO curls in parallel:

```bash
curl -sS -w 'A: HTTP %{http_code}\n' http://$BOAT_IP:82/detect -o /tmp/a.json &
curl -sS -w 'B: HTTP %{http_code}\n' http://$BOAT_IP:82/detect -o /tmp/b.json &
wait
echo "--- /tmp/a.json ---"; cat /tmp/a.json; echo
echo "--- /tmp/b.json ---"; cat /tmp/b.json; echo
```

Expected: exactly one `HTTP 200` and one `HTTP 429`. The 200 body is a normal detection result; the 429 body is exactly:

```
{"ok":false,"stage":"busy","error":"detect already in flight"}
```

The device MUST NOT reboot. Run `curl -sS http://$BOAT_IP:82/detect` a third time after the parallel pair returns — expected: HTTP 200, proving the mutex was released cleanly.

If both returned 200, the two calls did not actually overlap (they were spaced out by shell startup cost). Repeat the parallel test 3–5 times; at least one pair should overlap. If none do, add a small pre-curl loop warmup (`curl -sS http://$BOAT_IP:82/detect >/dev/null` first) and retry — the first call is the slow one and the parallel pair is more likely to overlap when both are hitting a warm cache.

If both returned 429, the mutex is leaking — see step 1.4 serial log and investigate.

If one returned 500 or the device rebooted, stop and diagnose.

- [ ] **Step 1.7: Commit**

```bash
git add main/detect_server.c
git commit -m "$(cat <<'EOF'
feat(detect): serialize /detect with non-blocking mutex + 429 on contention

Rapid-fire /detect calls panic esp-dl regardless of which httpd hosts
the handler. Isolating to port 82 in 4a2f539 fixed WS coexistence but
not the rapid-fire crash itself.

This commit adds a single FreeRTOS binary mutex in detect_server.c that
guards the handler body. Overlapping clients see HTTP 429 with a static
busy JSON literal; the device never reboots. The handler body is
extracted into do_detect_locked() so the give-side is a single line in
detect_handler(), eliminating forgot-to-release paths.

detection.cpp/h untouched — the LOAD-BEARING INVARIANT is strengthened
by explicit serialization, not weakened. detect_image_handler is
intentionally NOT guarded; it's a rodata byte dump with no inference
state.

See docs/superpowers/specs/2026-04-10-detect-serialization-design.md.
EOF
)"
```

Run `git status` after committing and verify:

- `nothing to commit, working tree clean` (other than the pre-existing unstaged `sdkconfig.defaults` and `.devcontainer/Dockerfile` changes, which are NOT part of this task).

---

## Task 2: Dashboard — remove cap and add 429 retry

**Rationale:** With the server returning 429 instead of crashing, the dashboard's `max="5"` rounds cap is a lie that needs to go. `runDetect()` must also retry 429 transparently so the auto-run-on-mode-switch race against a manual click doesn't surface a failure to the user.

**Files:**
- Modify: `main/dashboard.html:331` — remove `max="5"` and the tooltip from `#detect-rounds` input
- Modify: `main/dashboard.html:1390` — remove the `Math.min(5, …)` hardcode from `runDetect()`
- Modify: `main/dashboard.html:1402-1427` — add 429 retry loop around the fetch

- [ ] **Step 2.1: Remove the cap on the `#detect-rounds` input (line ~331)**

Find the current line:

```html
        <input id="detect-rounds" type="number" min="1" max="5" value="1" title="capped at 5 — esp-dl crashes under rapid repeat calls">
```

Replace with:

```html
        <input id="detect-rounds" type="number" min="1" value="1">
```

Use the Edit tool to avoid touching surrounding lines.

- [ ] **Step 2.2: Remove the JS `Math.min(5, …)` hardcode in `runDetect()` (line ~1390)**

Find the current line:

```javascript
    const n = Math.max(1, Math.min(5, parseInt(roundsIn.value, 10) || 1));
```

Replace with:

```javascript
    const n = Math.max(1, parseInt(roundsIn.value, 10) || 1);
```

- [ ] **Step 2.3: Add 429 retry loop inside `runDetect()`'s fetch block**

Find the current fetch block (lines ~1402–1427):

```javascript
    for (let i = 0; i < n; i++) {
      statusEl.textContent = `running ${i+1}/${n}…`;
      statusEl.className = '';
      try {
        const r = await fetch(`${DETECT_BASE}/detect`, { cache: 'no-store' });
        let body;
        try { body = await r.json(); }
        catch (_) {
          firstFail = `HTTP ${r.status} · non-JSON`;
          break;
        }
        if (!r.ok || !body.ok) {
          firstFail = `FAIL · ${body.stage || '?'} · ${body.error || `HTTP ${r.status}`}`;
          break;
        }
        lastRuns.push({
          inference_ms: body.inference_ms,
          detections: body.detections || [],
          body,
        });
        if (i < n - 1) await new Promise(res => setTimeout(res, INTER_CALL_MS));
      } catch (e) {
        firstFail = `ERROR · ${e.message}`;
        break;
      }
    }
```

Replace with:

```javascript
    // 429 retry budget: 20 attempts × 50 ms ≈ 1 s max backoff per round.
    // A 429 means "another caller got here first" — transparently retry
    // without incrementing the round counter. If the budget is spent,
    // treat it as a genuine failure for that round.
    const BUSY_BACKOFF_MS = 50;
    const BUSY_MAX_RETRIES = 20;

    outer: for (let i = 0; i < n; i++) {
      statusEl.textContent = `running ${i+1}/${n}…`;
      statusEl.className = '';

      let body = null;
      let lastStatus = 0;
      let busyRetries = 0;

      while (true) {
        let r;
        try {
          r = await fetch(`${DETECT_BASE}/detect`, { cache: 'no-store' });
        } catch (e) {
          firstFail = `ERROR · ${e.message}`;
          break outer;
        }
        lastStatus = r.status;

        if (r.status === 429) {
          if (++busyRetries > BUSY_MAX_RETRIES) {
            firstFail = `BUSY · no slot after ${BUSY_MAX_RETRIES} retries`;
            break outer;
          }
          await new Promise(res => setTimeout(res, BUSY_BACKOFF_MS));
          continue;
        }

        try { body = await r.json(); }
        catch (_) {
          firstFail = `HTTP ${r.status} · non-JSON`;
          break outer;
        }
        if (!r.ok || !body.ok) {
          firstFail = `FAIL · ${body.stage || '?'} · ${body.error || `HTTP ${r.status}`}`;
          break outer;
        }
        break;  // success, exit retry loop
      }

      lastRuns.push({
        inference_ms: body.inference_ms,
        detections: body.detections || [],
        body,
      });
      if (i < n - 1) await new Promise(res => setTimeout(res, INTER_CALL_MS));
    }
```

Notes on the diff:
- The outer `for` gains a label `outer:` so inner error paths can bail the whole loop without flag variables.
- The single-request logic moves into a `while (true)` retry loop, broken on success, non-429 error, or retry budget exhaustion.
- `break outer;` replaces the plain `break;` in every error path.
- The 500 ms `INTER_CALL_MS` between successful rounds is preserved (matches the existing "proven-stable spacing" comment at line 1394).

- [ ] **Step 2.4: Build the firmware**

Run:

```bash
idf.py build 2>&1 | tee /tmp/task2_build.log | tail -10
```

Expected: `Project build complete.` No JS linting happens in the ESP-IDF build (the HTML is embedded as raw bytes), so a bad JS syntax will only surface at browser load time. Visually re-read the JS block for syntax sanity before flashing.

- [ ] **Step 2.5: Flash**

Run:

```bash
idf.py flash 2>&1 | tail -5
```

Expected: `Leaving...`.

- [ ] **Step 2.6: Browser smoke test — cap removed**

Open `http://$BOAT_IP/` in a browser, switch to the Detect tab. In the rounds input, type `10`. It should accept the value (no browser-enforced cap). Click Run.

Expected: 10 rounds run sequentially, each ~56 ms inference (after the first cold round at ~500 ms), ~5.5 s wall time (10 × 500 ms inter-call + inference). Status reads `ok · 10× mean=...ms inf · ...ms wall`. Zero reboots; `idf.py monitor` during this shows no panic.

If only 5 rounds run, Step 2.2 was not applied — the JS `Math.min(5, …)` is still capping. Re-check the diff.

If any round fails with `BUSY · no slot after 20 retries`, a concurrent WS or other task is hammering the mutex — unlikely in the normal dashboard but possible. Investigate which caller took the mutex by adding a temporary `ESP_LOGI` in the take/give branches and re-run. This is not expected in passing conditions.

- [ ] **Step 2.7: Auto-run + manual click race test**

This is the specific scenario the 429 retry is designed to handle: the dashboard auto-runs `runDetect()` when you switch to the Detect tab (`window._detectHasRun` check on line ~1276), and a fast user click on the Run button can race with it.

Procedure:
1. Reload the dashboard (`Cmd+R` / `F5`).
2. Switch to Detect tab.
3. **Immediately** (within 200 ms) click the Run button once.

Expected: No visible error. Status ends with `ok · 1×` (or whatever the rounds input is set to). Under the hood, one of the two triggers got the mutex and the other saw a 429 and retried — invisible to the user.

Failure mode: the button click triggers a second batch of `n` rounds while the first is still running — this is a pre-existing dashboard behavior (no mutex in JS), so the second batch will serialize against the first on the server. Both should eventually complete. Status may flicker between the two batches; that's acceptable for Phase 2 and not in scope to fix here.

- [ ] **Step 2.8: Commit**

```bash
git add main/dashboard.html
git commit -m "$(cat <<'EOF'
feat(dashboard): remove detect-rounds cap, add transparent 429 retry

Now that detect_server.c returns HTTP 429 under contention instead of
crashing the device, the "max 5 rounds" cap in the rounds input and
the Math.min(5, ...) hardcode in runDetect() are no longer needed.
Remove them.

Add a 50 ms / 20-retry budget busy loop around each fetch so the
auto-run-on-tab-switch race against a manual Run click is invisible
to the user. Preserves the 500 ms INTER_CALL_MS spacing between
successful rounds.

See docs/superpowers/specs/2026-04-10-detect-serialization-design.md.
EOF
)"
```

---

## Task 3: Full acceptance test battery

**Rationale:** The spec lists seven acceptance tests. Walk through all seven in order, record results, and only sign off when all pass. No code changes in this task.

**Files:** None.

- [ ] **Step 3.1: Test 1 — Single call baseline**

```bash
curl -sS -w '\nHTTP %{http_code} in %{time_total}s\n' http://$BOAT_IP:82/detect
```

Pass criteria: HTTP 200, JSON with `"ok":true` and ≥1 detection, latency within 5% of pre-serialization baseline (first call ~500–700 ms, steady ~56 ms).

- [ ] **Step 3.2: Test 2 — Simultaneous double call (429 contention)**

```bash
curl -sS -o /tmp/a.json -w 'A: HTTP %{http_code}\n' http://$BOAT_IP:82/detect &
curl -sS -o /tmp/b.json -w 'B: HTTP %{http_code}\n' http://$BOAT_IP:82/detect &
wait
cat /tmp/a.json; echo
cat /tmp/b.json; echo
```

Pass criteria: exactly one `HTTP 200` and one `HTTP 429`. The 429 body is the exact `kBusyJson` literal. Device does not reboot.

- [ ] **Step 3.3: Test 3 — Curl rapid-fire loop**

```bash
while true; do
  curl -sS -o /dev/null -w "%{http_code}\n" http://$BOAT_IP:82/detect
done | head -100
```

Pass criteria: mix of `200` and `429` codes. Zero reboots. Run at least 100 requests. Also keep `idf.py monitor` tailing during this — there must be zero panic / abort / Guru Meditation entries.

- [ ] **Step 3.4: Test 4 — Dashboard auto-run + manual click race**

Procedure:
1. Reload dashboard.
2. Switch to Detect tab.
3. Click Run within 200 ms.

Pass criteria: No visible error. At least one round completes successfully. Device does not reboot.

- [ ] **Step 3.5: Test 5 — Sequential sustained load (memory leak check)**

```bash
for i in $(seq 1 100); do
  curl -sS -o /dev/null -w "%{http_code}\n" http://$BOAT_IP:82/detect
  sleep 0.2
done
```

Pass criteria: all 100 lines print `200`. Device does not reboot. Serial log (`idf.py monitor` in a second terminal) shows no abort, no panic, no "CORRUPT HEAP" from heap poisoning. This is the empirical test for whether the rapid-fire crash is purely a concurrency issue or has a sequential-state component too.

**If this fails** (reboots mid-sequence, or any HTTP response is 500 with `"stage":"inference"` or `"stage":"marshal"`): the rapid-fire fix is necessary but not sufficient — there's a deeper esp-dl state-reuse bug. Stop, capture the serial log, and open a new spec. This does NOT block the commit from Task 1 or Task 2 — it's a known limitation, not a regression.

- [ ] **Step 3.6: Test 6 — CORS preservation on all status codes**

200 path (warm state):

```bash
curl -sSi -H "Origin: http://localhost" http://$BOAT_IP:82/detect | grep -i "access-control"
```

Expected: `Access-Control-Allow-Origin: *`.

429 path (fire a second curl during the first):

```bash
(curl -sS http://$BOAT_IP:82/detect -o /dev/null &)
sleep 0.01
curl -sSi -H "Origin: http://localhost" http://$BOAT_IP:82/detect | grep -iE "HTTP/|access-control"
wait
```

Expected: `HTTP/1.1 429 Too Many Requests` and `Access-Control-Allow-Origin: *`. Timing is fiddly; retry 2–3 times if the first attempt doesn't actually hit 429. Pass criteria: when a 429 is observed, it carries the CORS header.

500 path: triggering a real 500 requires the model to fail, which doesn't happen in normal operation. **Skip** this sub-test — the 500 path setting CORS has not changed and its behavior is identical to before.

- [ ] **Step 3.7: Test 7 — Co-residency during contention (the Phase 1 re-run under load)**

Setup:
1. Open the dashboard in a browser (`http://$BOAT_IP/`) and confirm:
   - Camera MJPEG is live (no stall).
   - IMU horizon is moving / heading is updating.
   - ToF grids are updating.
   - WebSocket is green (`ok` status dot).
2. In a terminal, run the rapid-fire loop from Test 3 for at least 60 seconds.
3. Observe the browser during the loop.

Pass criteria:
- Camera stream does NOT freeze.
- IMU heading does NOT freeze (watch for values changing if the boat is moved, or at minimum the horizon tilt animating).
- ToF grids do NOT freeze.
- WebSocket stays connected (status dot stays green).
- Device does not reboot.

This is the key acceptance test: it proves the serialization does not regress the Phase 1 co-residency guarantee. If any subsystem stalls, investigate whether the detect server is somehow starving the port-80 httpd task (shouldn't be possible, but worth verifying). Capture both the serial log and browser DevTools network tab for triage.

- [ ] **Step 3.8: Record results**

Create a short results file to attach to the PR description (or just paste into the commit message of a follow-up doc update):

```bash
cat > /tmp/task3_results.txt <<'EOF'
Test 1 — Single call:                 [PASS/FAIL] ______
Test 2 — Simultaneous double:         [PASS/FAIL] ______
Test 3 — Rapid-fire curl loop (100):  [PASS/FAIL] ______
Test 4 — Dashboard auto-run + click:  [PASS/FAIL] ______
Test 5 — Sequential 100 × 200ms:      [PASS/FAIL] ______
Test 6 — CORS on 429:                 [PASS/FAIL] ______
Test 7 — Co-residency under load:     [PASS/FAIL] ______
EOF
cat /tmp/task3_results.txt
```

Fill in PASS/FAIL and any notes. There is no commit for this task — these are verification steps.

---

## Post-plan — what's left for this branch

After Task 3 passes, the `feat/detect-phase1` branch has:

1. The Phase 1 cat detect endpoint (from prior commits).
2. A serialized, rapid-fire-safe handler (Task 1 commit).
3. A dashboard that retries 429s transparently and has no artificial rounds cap (Task 2 commit).

This is the "simple working concept" milestone. It is **safe** for:
- Dropping in the maritime Pico `.espdl` when it's ready (only `models/p4/` swap + class labels).
- Phase 3 live-camera inference (a second caller path for `detection_run_on_embedded()` will automatically serialize against the HTTP caller through the same mutex).

It does **not** address:
- Root cause of why esp-dl panics on overlapping calls (out of scope — symptom is unreachable now).
- Long-session heap fragmentation on sequential calls (out of scope — if Test 5 surfaces it, open a new spec).
- Any maritime-specific or camera-specific work.

If all Task 3 tests pass, the branch is ready for code review and merge to `main`. Follow the project's existing review workflow.
