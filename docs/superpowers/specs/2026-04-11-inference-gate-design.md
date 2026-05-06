# Inference Gate — Design Spec

**Date:** 2026-04-11
**Status:** Approved by user, ready for implementation plan
**Scope:** Debug-mode firmware change to let esp-dl `/detect` inference coexist with WS sensor streaming and MJPEG camera streaming without crashing

## Context

The BoatEspP4 firmware runs three independent httpd instances concurrently:
- `:80` — dashboard HTML + `/ws` WebSocket streaming 20 Hz protobuf sensor frames
- `:81` — MJPEG camera stream at `/stream`
- `:82` — `/detect` esp-dl inference endpoint (embedded JPEG input, will become live camera later)

When the dashboard is fully connected (WS + MJPEG both live), a single `/detect` call reliably crashes the device. Three crash fingerprints observed during investigation:

1. **Core 0 Instruction access fault** (`MEPC=0`, `RA=0`) in `sdio_read_task` — the saved task context was trampled.
2. **Core 0 Load access fault** (`MEPC=0x4ff130dc`) in `ws_tx_task` inside TLSF `remove_free_block` called from `poll()` cleanup — the heap free-list had a NULL-pointer link.
3. **Core 1 `heap_caps_free` assertion** (`heap != NULL && "free() target pointer is outside heap areas"`) inside `dl::TensorBase::TensorBase` during model load — a bogus (non-heap) pointer was passed to free.

All three are consistent with **concurrent PSRAM allocator contention** between esp-dl tensor allocations and streamer task allocations. With `CONFIG_HEAP_POISONING_COMPREHENSIVE=y`, the detector caught concrete corruption at `0x487e1a20` (PSRAM) where a freed block's fill pattern had been overwritten with tensor-like byte data.

Tests that do **not** crash:
- `curl /detect` with no dashboard
- `curl /detect/burst` (five back-to-back inferences with 1 s gaps) with no dashboard
- The reference `cat_detect/main/app_main.cpp` example: 500 rounds in a tight loop from `main_task`, no concurrent tasks, never crashes

Tests that **do** crash:
- `curl /detect` with dashboard fully connected
- `/detect/burst` with dashboard fully connected

## Goal

Make `/detect` (and `/detect/800`, `/detect/burst`) runnable without crashing while the dashboard is open. This is a **debug-mode** mechanism — the target production architecture moves inference to a local task with periodic ESP-NOW reporting and doesn't need this gate at all. The gate lives as long as the WS + MJPEG streaming infrastructure lives.

## Non-goals

- Fixing the underlying esp-dl ↔ streamer allocator race. The streamers will continue to have an allocator-contention bug with esp-dl; we work around it by not letting them run simultaneously, not by fixing the collision.
- Eliminating `/detect/burst`. The debug endpoint stays.
- Architectural redesign toward ESP-NOW or cache-reader `/detect`. Tracked separately; this spec does not block it.
- Covering multi-tab dashboard load (fd budget concern — orthogonal, see ultraplan S2).
- A long-soak stability test. Good to do eventually, not a prerequisite for accepting this change.

## Approach summary

When `/detect` handler acquires `s_detect_mutex`, it calls `vTaskSuspend()` on the two tasks whose allocator activity is known to race with esp-dl — `ws_tx_task` (WS frame sender) and the port-81 httpd worker (MJPEG handler) — runs inference with those tasks frozen, then `vTaskResume()`s them and releases the mutex. A 3-second one-shot `esp_timer` watchdog force-resumes the suspended tasks if inference runs absurdly long, providing deadlock recovery.

This is **Option B** from the brainstorming session: raw external `vTaskSuspend`, not cooperative self-suspension. Deadlock risk (suspending a task that holds a shared lock esp-dl wants) is accepted and mitigated by the watchdog.

## Architecture

The inference gate is a short critical section wrapped around esp-dl inference calls in `detect_server.c`. The order of operations is:

1. Take `s_detect_mutex` (non-blocking; on failure, return 429 — unchanged behavior)
2. `ws_transport_pause()`
3. `camera_stream_pause()`
4. `esp_timer_start_once(s_gate_watchdog, 3_000_000)` — 3-second one-shot
5. `detection_run_on_embedded(&out)` — esp-dl runs with the chip to itself
6. `esp_timer_stop(s_gate_watchdog)`
7. `camera_stream_resume()`
8. `ws_transport_resume()`
9. `send_detect_response(...)`
10. Release `s_detect_mutex`

Tasks **not** suspended and the reason each stays live during inference:
- `sensor_task` — writes only to a static double-buffer (`s_slot_buf`), no heap activity
- IMU task, ToF task — I²C reads, no heap activity
- H_SDIO_DRV read/write/process tasks — mandatory for WiFi (host ↔ C6 slave transport); suspending them would break `/detect`'s own HTTP response path
- Port-80 httpd worker — serves dashboard HTML and WS handshakes, doesn't do allocator-heavy work in the hot path
- Port-82 httpd worker — that's the task running `/detect` itself

The gate is expected to be **debug-only**. When the production architecture pivots to local inference + periodic report (no streaming), the gate and its three touch points get deleted wholesale.

## Components

### `main/transports/ws_transport.c` + `ws_transport.h`

Two new public functions operating on the existing static `s_tx_task` handle:

```c
// ws_transport.h
void ws_transport_pause(void);
void ws_transport_resume(void);
```

```c
// ws_transport.c
void ws_transport_pause(void)
{
    if (s_tx_task) vTaskSuspend(s_tx_task);
}

void ws_transport_resume(void)
{
    if (s_tx_task) vTaskResume(s_tx_task);
}
```

Both check for NULL to handle the pre-initialization window (called before `ws_transport_init()` has run). `vTaskResume` is safe on a running task (FreeRTOS no-op per docs).

### `main/camera_stream.c` + `camera_stream.h`

Same shape. The port-81 httpd worker task handle is **captured lazily** inside the MJPEG stream handler on the first client connection:

```c
// camera_stream.c — inside the stream URI handler
static TaskHandle_t s_mjpeg_task = NULL;

static esp_err_t mjpeg_stream_handler(httpd_req_t *req)
{
    if (!s_mjpeg_task) s_mjpeg_task = xTaskGetCurrentTaskHandle();
    // ... existing stream logic ...
}
```

Pause/resume:

```c
// camera_stream.h
void camera_stream_pause(void);
void camera_stream_resume(void);
```

```c
// camera_stream.c
void camera_stream_pause(void)
{
    if (s_mjpeg_task) vTaskSuspend(s_mjpeg_task);
}

void camera_stream_resume(void)
{
    if (s_mjpeg_task) vTaskResume(s_mjpeg_task);
}
```

If no MJPEG client has ever connected, `s_mjpeg_task` is NULL and both functions are no-ops — correct because an idle port-81 worker parked in `accept()` holds no PSRAM allocations and isn't a collider.

### `main/detect_server.c`

Four additions:

1. **Includes** — `#include "transports/ws_transport.h"` and `#include "camera_stream.h"` (plus `esp_timer.h` if not already present).

2. **Watchdog timer state** — created once in `detect_server_start()`:
   ```c
   static esp_timer_handle_t s_gate_watchdog = NULL;
   static void gate_watchdog_cb(void *arg);
   #define GATE_WATCHDOG_US (3 * 1000 * 1000)  // 3 seconds
   ```

   The timer is created in `detect_server_start()` alongside `s_detect_mutex` via `esp_timer_create()` with `ESP_TIMER_TASK` dispatch.

3. **Handler body rewrites** — `do_detect_locked`, `do_detect_800_locked`, and `do_detect_burst_locked` are bracketed with the pause/watchdog/resume sequence (burst is retained per the Non-goals section):

   ```c
   static esp_err_t do_detect_locked(httpd_req_t *req)
   {
       detection_response_t out = {0};
       esp_err_t ret;

       ws_transport_pause();
       camera_stream_pause();
       esp_timer_start_once(s_gate_watchdog, GATE_WATCHDOG_US);

       ret = detection_run_on_embedded(&out);

       esp_timer_stop(s_gate_watchdog);
       camera_stream_resume();
       ws_transport_resume();

       return send_detect_response(req, &out, ret);
   }
   ```

   `do_detect_burst_locked` wraps the **entire burst** (not per-iteration) in one gate acquire/release — simpler and the burst is a debug tool so a 5-second streamer pause is acceptable.

4. **Watchdog callback**:
   ```c
   static void gate_watchdog_cb(void *arg)
   {
       ESP_LOGE(TAG,
                "INFERENCE GATE WATCHDOG: inference exceeded %d ms, force-resuming streamers",
                GATE_WATCHDOG_US / 1000);
       camera_stream_resume();
       ws_transport_resume();
   }
   ```

   Force-resumes both subsystems and logs loudly. Does not abort or interrupt inference — esp-dl finishes on its own timeline. Subsequent `do_detect_locked` cleanup calls `_resume()` again, which is a no-op on running tasks.

## Data flow (single /detect request, dashboard live)

```
Client                :82 httpd          ws_tx_task     :81 httpd worker    esp-dl
  │                       │                  │                │                │
  │──GET /detect─────────▶│                  │                │                │
  │                       │ xSemaphoreTake   │                │                │
  │                       │  s_detect_mutex  │                │                │
  │                       │                  │                │                │
  │                       │ ws_transport_pause()              │                │
  │                       │──vTaskSuspend───▶│                │                │
  │                       │                  │ (Suspended)    │                │
  │                       │                  │                │                │
  │                       │ camera_stream_pause()             │                │
  │                       │──vTaskSuspend────│───────────────▶│                │
  │                       │                  │                │ (Suspended)    │
  │                       │                  │                │                │
  │                       │ esp_timer_start_once(watchdog, 3s)│                │
  │                       │                  │                │                │
  │                       │ detection_run_on_embedded(&out)   │                │
  │                       │────────────────────────────────────────────▶│      │
  │                       │                  │                │       (load+run)
  │                       │                  │                │            ~500 ms
  │                       │◀──────────────────────────────────────────│        │
  │                       │                  │                │                │
  │                       │ esp_timer_stop(watchdog)          │                │
  │                       │ camera_stream_resume()            │                │
  │                       │──vTaskResume─────│───────────────▶│                │
  │                       │                  │                │ (Running)      │
  │                       │ ws_transport_resume()             │                │
  │                       │──vTaskResume────▶│                │                │
  │                       │                  │ (Running)      │                │
  │                       │                  │                │                │
  │                       │ send_detect_response              │                │
  │                       │ xSemaphoreGive   │                │                │
  │                       │  s_detect_mutex  │                │                │
  │◀─200 + JSON──────────│                  │                │                │
```

### Ordering invariants

1. Tasks are suspended **after** `s_detect_mutex` is taken and **before** any esp-dl allocation begins. There is no window where inference runs concurrently with the streamers.
2. Resume happens **before** `send_detect_response` so that the response path runs with the streamers alive.
3. If `detection_run_on_embedded` aborts early, `esp_timer_stop` and both `_resume` calls still run — guaranteed by the single-exit control flow in `do_detect_locked`.
4. The 3-second watchdog is longer than the worst-case inference (first-call ~500 ms) but short enough to recover from a true deadlock before the user notices.

### Concurrent request during gate interval

A second client hitting `/detect`, `/detect/800`, or `/detect/burst` races for `s_detect_mutex`. The mutex is non-blocking (`xSemaphoreTake(..., 0)`), so the second request gets HTTP 429 immediately and does not touch the gate. Unchanged from today.

`/detect/image.jpg` and `/detect/800/image.jpg` do not take the mutex and serve rodata byte dumps; they work fine while the gate is engaged.

## Error handling

### Inference returns an error mid-way

`detection_run_on_embedded()` can fail (JPEG decode error, alloc failure, marshal OOM). In all cases the gate must release and the watchdog must be stopped. Implemented via single-exit control flow in `do_detect_locked` — no early returns between pause and resume.

### Watchdog fires

The 3-second one-shot timer callback runs on the `esp_timer` task. It force-resumes both subsystems and logs an error. It does **not** abort the inference call — esp-dl keeps running on the /detect handler's task. When inference eventually finishes, the handler's cleanup calls `_resume()` again, which is a FreeRTOS no-op on a running task.

The watchdog's job is **recovery**, not **enforcement**. A fired watchdog means either (a) inference is genuinely slow or (b) we hit the deadlock case Option B permits. Either way, streamers are unparked and the dashboard recovers.

### Task handle is NULL when pause is called

Pre-init or never-used case. Handled by the explicit NULL check in each pause/resume function. No error, no log spam — NULL handle is a normal state that means "nothing to pause".

### Double-suspend / double-resume

FreeRTOS semantics: `vTaskSuspend` on an already-suspended task and `vTaskResume` on a running task are both safe no-ops. The watchdog's force-resume followed by the handler's cleanup-resume is therefore safe.

### Suspended task holds a lock esp-dl wants

This is the deadlock risk that Option B accepts by design. If `ws_tx_task` gets suspended while holding the TLSF heap mutex or an lwIP socket lock, the next esp-dl allocation blocks forever — until the 3-second watchdog fires and force-resumes the suspended task, releasing the lock.

**Worst-case `/detect` latency when this happens: 3 s watchdog + ~500 ms inference = ~3.5 s.** Normal case remains ~60 ms steady-state, ~500 ms cold-start.

**Monitoring signal:** the serial log line
```
INFERENCE GATE WATCHDOG: inference exceeded 3000 ms, force-resuming streamers
```
If this fires repeatedly during normal use, the gate design is insufficient and the conversation moves to cooperative self-suspension (Option A from brainstorming) or a different approach.

### Inference call nested inside another inference call

Not possible. `s_detect_mutex` is non-recursive and serializes all `/detect*` requests. A second request gets 429 and doesn't touch the gate.

## Testing

### T1 — Primary success test (the thing we're fixing)

**Setup:** Dashboard open at `http://192.168.1.201/`, WS streaming sensor data actively updating, MJPEG stream visibly showing camera frames.

**Action:** `curl http://192.168.1.201:82/detect`

**Pass criteria:**
- Serial log shows a full gate cycle (pause → inference → resume) with no panic, no assertion, no `CORRUPT HEAP`
- Curl returns HTTP 200 with valid detection JSON
- MJPEG stream briefly freezes for ~500 ms (first call) or ~60 ms (subsequent) then resumes
- WS sensor data shows a matching gap then resumes
- Device does not crash or reboot

### T2 — Steady-state back-to-back

**Action:** 10 sequential `curl /detect` calls, ~1 second apart, dashboard still open.

**Pass criteria:** First call 500 ms, remaining nine ~60 ms each. All return 200. Dashboard shows ten micro-freezes but recovers between each. No crash.

### T3 — Stress with /detect/burst

**Action:** `curl /detect/burst` with dashboard open.

**Pass criteria:** Burst wraps all 5 iterations in one gate acquire/release (not per-iteration). Dashboard MJPEG freezes for ~5 s, WS gap of ~5 s, then both recover. All 5 iterations complete without crash. Burst returns 200.

### T4 — Negative test: no-dashboard regression

**Action:** `curl /detect` with no dashboard open (no WS client, no MJPEG client).

**Pass criteria:** `s_tx_task` exists but is idle in `ulTaskNotifyTake`; `s_mjpeg_task` is NULL because no client has ever hit `/stream`. Both pause calls are safe no-ops or NULL-check skips. Inference runs as it did before the gate existed. Returns 200. No regression.

### T5 — Watchdog recovery verification

**Action:** Temporarily inject `vTaskDelay(pdMS_TO_TICKS(5000))` into `do_detect_locked` between pause and `detection_run_on_embedded`. Rebuild, reflash, run T1.

**Pass criteria:**
- After 3 seconds, serial log shows `INFERENCE GATE WATCHDOG: inference exceeded 3000 ms, force-resuming streamers`
- Dashboard streamers resume mid-delay (~2 seconds before the injected delay finishes)
- Delay finishes, real inference runs, response sent, curl returns 200
- Device does not crash

Revert the injected delay after verification; rebuild; reflash; re-run T1 to confirm the revert.

### Test execution order

1. T4 first (baseline, no dashboard needed)
2. T1, T2, T3 in order (dashboard needed; ask user to confirm dashboard is live before running)
3. T5 last (requires code injection + rebuild cycle)

### Verification commands

```bash
idf.py build          # after code changes
idf.py flash          # device should reset cleanly to new firmware
# user confirms dashboard is live for T1-T3
curl -v http://192.168.1.201:82/detect          # T1
for i in 1 2 3 4 5 6 7 8 9 10; do curl -s http://192.168.1.201:82/detect; sleep 1; done  # T2
curl -v --max-time 15 http://192.168.1.201:82/detect/burst   # T3
# no dashboard for T4
curl -v http://192.168.1.201:82/detect          # T4 (dashboard closed)
```

Serial log is captured in the background during each test with `timeout 30 cat /dev/ttyACM0 > /tmp/gate_test_N.log`.

## Not in scope

- **Production architecture pivot** (local inference task + periodic ESP-NOW report). Planned separately; this gate is debug-only and disappears with the streaming infrastructure it protects.
- **Phase-0 diagnostic cleanup** (revert comprehensive heap poisoning back to LIGHT, revert main.c camera_stream disable). Part of the implementation plan, not this design.
- **`/detect/burst` endpoint removal**. The burst is a useful debug tool; keep it. Gate-wrap it like the other detect handlers.
- **fd budget audit** (ultraplan S2). Orthogonal, not gated by this change.
- **Co-residency regression harness rewrite** (ultraplan Phase A). The broken harness needs fixing for the new gate assertions, but that's tracked separately and lands after this change.
- **Cooperative self-suspension (Option A from brainstorming)**. Falls back here only if the watchdog fires repeatedly in practice, indicating B's race window is problematic.

## Risks

1. **Deadlock at pause point** — primary risk of Option B. Mitigated by the 3-second watchdog. Worst-case `/detect` latency becomes 3.5 s when it fires. Monitored via the dedicated log line.
2. **Watchdog fires on first-call cold-start** — if model load somehow takes >3 s. Current measurement: ~500 ms. 6x safety margin. Not expected but the watchdog bound is tunable via `GATE_WATCHDOG_US`.
3. **Suspended task holding hardware resource** — e.g., mid-SDIO DMA setup. Per the existing comment in `ws_transport.c`, concurrent close+send on the same fd corrupts esp_hosted's internal spinlock. The gate doesn't create new fd-close races because it doesn't close fds, but suspending mid-DMA remains theoretical. Empirically we'll see if it manifests; the watchdog is the escape.
4. **Browser MJPEG client behavior during long pause** — a 5-second MJPEG freeze (T3 burst case) may trigger some browsers' connection timeout heuristics. Acceptable: if it does, the browser reconnects and fetches a fresh stream. The device doesn't care.
5. **Gate outlives its usefulness** — the gate is debug-only but there's no forcing function to remove it when the production pivot lands. Documented in code comments and in this spec's Non-goals section. Removal is a 3-file diff and should be easy when the time comes.
