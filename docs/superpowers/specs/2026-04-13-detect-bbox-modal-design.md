# Detect Bounding-Box Modal on Dashboard — Design

**Date:** 2026-04-13
**Status:** Approved
**Baseline:** `07746f2` (last known good — 6/6 detect triggers stable with live MJPEG)

## Goal

When the user clicks **Detect**, pop a modal over the live camera feed that shows the *exact* still frame the inference ran on, with bounding boxes drawn on it. The live MJPEG stream keeps flowing underneath and is restored as soon as the user dismisses the modal.

## Non-Goals (Phase 2 parking lot)

- Per-class colors and class-name mapping
- "Save detection to gallery" / localStorage history
- Score-threshold slider in the UI
- Device-side frame echo via protobuf (current approach is client-side snapshot; device echo is a future option if alignment proves too loose)
- Burst mode / auto-trigger by ToF proximity
- Per-frame MJPEG timestamps (needed for a real staleness warning — not in v1)

## Architecture

All work is in `main/dashboard.html` + two small firmware additions in `main/detect_task.cpp` and `main/http_server.c`. No protobuf changes, no Kconfig changes, no new components.

### Data flow

```
USER ─click Detect──▶ JS
                      │
                      ├─► drawImage(streamImg) → hidden canvas (800×640)
                      ├─► save clickTs = performance.now()
                      ├─► send WS DetectCommand
                      ├─► disable button + show spinner
                      └─► begin HTTP polling of /detect_result?since=<lastSeenSeq>  (1 s interval)

DEVICE ────────────── detect_task runs (capture, decode, rotate, WiFi disconnect, inference,
                      WiFi connect, cache result, publish on WS for any live client) ──────────
                      ▲                                                                        │
                      │                                                                        │
                      └── updates static s_last_detect (mutex-guarded): seq++,                 │
                          frame_ts_us, N detections, bbox coords in rotated 800×640 space      │
                                                                                                │
JS ◀── HTTP GET /detect_result?since=<lastSeenSeq> ─────────────────────────────────────────────┘
         • HTTP 204 → keep polling
         • HTTP 200 with seq>lastSeen AND frame_ts_us>=clickTs → stop polling,
           draw bboxes on canvas (with rotation coord flip), open modal
         • 10 s total elapsed → timeout toast → IDLE
```

### Coordinate transform (180° rotation)

The MJPEG `<img>` has `transform: rotate(180deg)` via CSS — pixels on the wire are upside-down, browser shows them upside-right.

`drawImage(streamImg, ...)` captures **raw pre-rotation pixels** (canvas ignores CSS transforms). So the snapshot canvas holds upside-down pixels.

Device bbox coords are in **post-rotation** 800×640 space (detect_task.cpp:78–90 rotates RGB888 before inference).

**Fix:** apply the same `transform: rotate(180deg)` CSS to the snapshot canvas, then flip bbox coords before drawing so the boxes land on the rotated pixels:

```js
// Bbox arrives as (x1, y1, x2, y2) in rotated space.
// Canvas stores pre-rotation pixels but is CSS-rotated for display.
// Draw coords are on the pre-rotation pixel grid. Apply 180° inverse:
const W = 800, H = 640;
const tx1 = W - x2;
const ty1 = H - y2;
const tx2 = W - x1;
const ty2 = H - y1;
ctx.strokeRect(tx1, ty1, tx2 - tx1, ty2 - ty1);
```

**Load-time assertion** (catches formula errors at page load):

```js
const test = rotate180Bbox(0, 0, 100, 100, 800, 640);
console.assert(test.x1 === 700 && test.y1 === 540 &&
               test.x2 === 800 && test.y2 === 640,
               "rotation transform broken");
```

### State machine

```
IDLE
  │
  │ user clicks Detect
  ▼
ARMING ─── snapshot stream → hidden canvas
  │        save clickTs
  │        send WS DetectCommand
  │        disable button + show spinner
  │        start HTTP poll loop (1 s interval)
  │
  ├── poll response: seq>lastSeen AND frame_ts_us>=clickTs ──► DISPLAY (modal open)
  │                                                              │
  │                                                              │ dismiss (X | backdrop | Esc)
  │                                                              ▼
  │                                                            IDLE
  │
  └── 10 s elapsed with no matching response ──► ERROR (toast) ──► IDLE
```

**Guards:**
- Button is disabled from ARMING until IDLE re-entry (no double-fires)
- Polling is **paused while WS is closed** (detect `ws.readyState !== OPEN`) — avoids spamming requests during the 5-6 s WiFi reconnect. Resume on `ws.onopen` OR on the next 1 s tick after WS re-opens.
- Do **NOT** auto-poll on page load — only poll after an explicit Detect click. The cache is for surviving WS reconnect, not for showing history.

## Firmware changes

### `detect_task.cpp`

Add a static result cache guarded by a mutex:

```c
typedef struct {
    int32_t category;
    float   score;
    int32_t x1, y1, x2, y2;
} detect_item_t;

static struct {
    SemaphoreHandle_t mu;
    uint64_t      frame_ts_us;
    uint32_t      seq;          // monotonically increasing
    int16_t       count;        // 0..10
    detect_item_t items[10];
} s_last_detect = { 0 };
```

Created in `detect_init()`, written under mutex at the end of `detect_task_fn` right before (or alongside) `pipeline_publish_sensors`. Publish via WS stays — it's a free push for any connected client; HTTP poll is the reliable path.

Expose a reader:

```c
// Returns count of items copied. 0 if no result yet or seq <= since.
esp_err_t detect_get_result(uint32_t since, detect_result_snapshot_t *out);
```

`frame_ts_us` = `esp_timer_get_time()` at Step 0 (trigger received), so the snapshot corresponds to the physical frame the model ran on.

### `http_server.c`

New handler on port 80:

```
GET /detect_result?since=<N>
  → HTTP 204 (no newer result)
  → HTTP 200 application/json
     {
       "seq": 42,
       "frame_ts_us": 12345678901,
       "detections": [
         { "category": 0, "score": 0.73, "x1": 120, "y1": 200, "x2": 350, "y2": 420 },
         ...
       ]
     }
```

Bespoke JSON builder (no cJSON dep) — bounded output, ~400 bytes max with 10 detections. Reuses the `s_last_detect.mu` mutex for a short read-and-copy critical section.

`since` parses to `uint32_t`; malformed or missing → treat as 0.

## Dashboard changes (`main/dashboard.html`)

Work on top of whatever scaffolding commits `45f8563` + `6846b62` left in place — reuse class-aware coloring if it's present; otherwise single-color stroke for v1.

### HTML structure (new)

```html
<!-- add crossorigin to the existing MJPEG <img> -->
<img id="cam-img" crossorigin="anonymous" src="http://<ip>:81/stream">

<!-- hidden offscreen snapshot canvas -->
<canvas id="detect-snap" width="800" height="640" hidden></canvas>

<!-- modal overlay -->
<div id="detect-modal" class="detect-modal hidden" role="dialog" aria-modal="true">
  <div class="detect-modal-backdrop"></div>
  <div class="detect-modal-body">
    <button class="detect-modal-close" aria-label="Close">×</button>
    <div class="detect-modal-canvas-wrap">
      <!-- display canvas: CSS-rotated so it matches what the user saw live -->
      <canvas id="detect-display" width="800" height="640"></canvas>
    </div>
    <div class="detect-modal-stats"><!-- "N detections, inference 57ms" --></div>
  </div>
</div>
```

### Critical attribute

**`crossorigin="anonymous"`** on the stream `<img>` is load-bearing. Without it, `drawImage` taints the canvas even though `camera_stream.c` already returns `Access-Control-Allow-Origin: *`. Commit `bdb113c` had this; the c7cd5b5 reset likely lost it. Verify and re-add if missing.

### JS structure (outline)

- `class DetectController` with methods: `arm()`, `stopPoll()`, `showModal(result)`, `dismissModal()`, keyboard listener for Escape
- `snapshotCurrentFrame()` — `drawImage(streamImg, 0, 0, 800, 640)` into the display canvas, copies pixels; stores `snapshotTs = performance.now()`
- `pollOnce(lastSeenSeq)` — `fetch('/detect_result?since='+lastSeenSeq)`; handles 204/200/network-error
- `drawBboxes(ctx, detections)` — loops detections, applies `rotate180Bbox`, draws rect + label
- `rotate180Bbox(x1, y1, x2, y2, W, H) → {x1, y1, x2, y2}` — pure function with load-time assertion

### Dismissal

- Click backdrop → dismiss
- Click `×` button → dismiss
- `keydown` Escape when modal is open → dismiss
- Dismiss animates out, canvas is cleared, state returns to IDLE

## Error handling

| Case | Behavior |
|---|---|
| WiFi never reconnects within 10 s | Toast "Detection timed out", button re-enabled |
| WS closed, only HTTP poll active | Works — HTTP path doesn't depend on WS |
| `drawImage` throws (image broken) | Toast "No live frame to capture", skip detect |
| Poll returns malformed JSON | Log to console, keep polling until timeout |
| Multi-tab: other tab's click arrives first | `frame_ts_us >= clickTs` guard rejects it; keep polling for this tab's own result |
| Page refresh during inference | New session starts IDLE; no auto-poll at load; user re-clicks if needed |
| `s_last_detect.mu` contended with detect_task writing | httpd handler blocks ~100 µs, negligible |

## Testing

**Smoke:** Manual click-Detect on dashboard, confirm modal opens with boxes roughly aligned on whatever is in frame.

**Rotation alignment:** Point camera at a known object in one corner (e.g., a colored sticker in top-left of the physical view). Click Detect. Confirm the bbox appears in the **same visible corner** in the modal — not mirrored to the opposite corner. If wrong: rotation coord flip is inverted.

**Multi-trigger stability:** Click Detect 5× in a row, dismissing each modal. Each should show its own fresh result. Verifies `since=`/`frame_ts_us` correlation works across cycles.

**Reconnect resilience:** Open DevTools Network tab. Click Detect. Observe polling during 5-6 s WiFi-down window — should pause while WS is closed (not flood with failed requests).

**Multi-tab:** Open two dashboard tabs. Click Detect in tab A. Tab A gets its result, tab B does not pop a modal (no active click in tab B).

**Load-time assertion:** Open browser console on page load. Confirm no `AssertionError: rotation transform broken` in the log.

## Rollout & risk

Single commit, reversible. Worst case: modal is visually broken; live feed and sensor telemetry are unaffected. Firmware changes are additive (new HTTP handler, new cache struct) — if they regress, revert the commit, no data loss.
