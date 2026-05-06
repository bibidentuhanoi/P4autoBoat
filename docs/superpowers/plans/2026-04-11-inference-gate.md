# Inference Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a FreeRTOS-level inference gate that suspends the WS frame-sender and MJPEG stream worker during esp-dl inference calls, eliminating the PSRAM allocator-contention crashes observed when `/detect` runs with the dashboard open.

**Architecture:** Three-file surgical change — ws_transport.c and camera_stream.c each expose `pause()`/`resume()` functions that wrap `vTaskSuspend`/`vTaskResume` on their respective task handles. detect_server.c brackets its inference calls with both pause functions plus a 3-second `esp_timer` watchdog for deadlock recovery. Mutex-protected, single-exit control flow guarantees the gate always releases.

**Tech Stack:** ESP-IDF v5.4, FreeRTOS (RISC-V port), esp_http_server, esp_timer, esp-dl v3.x.

**Spec reference:** `docs/superpowers/specs/2026-04-11-inference-gate-design.md`

---

## Project-specific constraints

- **Do NOT create a git worktree.** `main/idf_component.yml` has `override_path: "../docs/esp-dl/models/cat_detect"` pointing into the gitignored `docs/` tree, and `main/CMakeLists.txt` references `tools/esp-detection/espdet.jpg` via `configure_file`. Neither path is copied into worktrees, so the CMake configure step fails. Work directly in the main checkout at `/workspaces/BoatEspP4`.
- **Subagents can build but cannot flash.** `idf.py build` runs fine headless, but `idf.py flash` needs USB access to `/dev/ttyACM0` which subagents don't have. Any task that requires hardware (flash, curl, serial capture) must run in the main session, not a subagent.
- **`docs/` is gitignored** (`.gitignore` line 83). The spec and this plan cannot be `git add`ed — they live on disk only. Do not attempt to commit them.
- **Watchdogs are disabled** in `sdkconfig.defaults` (`CONFIG_ESP_INT_WDT=n`, `CONFIG_ESP_TASK_WDT_EN=n`). Our gate's own `esp_timer`-based watchdog is the only recovery mechanism for the deadlock case.

## File Structure

Files touched (no new files):

| File | Responsibility | Changes |
|---|---|---|
| `main/transports/ws_transport.h` | Public WS transport API | Add 2 function declarations |
| `main/transports/ws_transport.c` | WS frame sender (ws_tx_task) | Add 2 function definitions |
| `main/camera_stream.h` | Public MJPEG server API | Add 2 function declarations |
| `main/camera_stream.c` | MJPEG stream handler | Add 2 function definitions + lazy task handle capture |
| `main/detect_server.c` | /detect HTTP endpoints | Add watchdog timer state + callback; bracket 3 handler bodies with gate; remove Phase-0 debug logging |
| `main/main.c` | App init | Revert Phase-0 diagnostic: re-enable `camera_stream_server_start()` |
| `sdkconfig.defaults` | Default build config | Revert Phase-0 diagnostic: `HEAP_POISONING_LIGHT` instead of `COMPREHENSIVE` |
| `sdkconfig` | Active build config | Revert Phase-0 diagnostic: match sdkconfig.defaults |

---

## Task 1 — Revert Phase-0 diagnostic noise

Clean baseline first. The gate goes on top of a code tree that doesn't have leftover diagnostic mutations from the debugging session. **Do not test after this task** — the crash is still reproducible without the gate, so running T1 here would just crash the device. Next task immediately.

**Files:**
- Modify: `main/main.c` (re-enable camera_stream_server_start)
- Modify: `sdkconfig.defaults` (HEAP_POISONING LIGHT)
- Modify: `sdkconfig` (HEAP_POISONING LIGHT)
- Modify: `main/detect_server.c` (remove HWM debug logs + simplify burst handler; keep burst endpoint itself)

- [ ] **Step 1.1: Re-enable camera_stream_server_start in main.c**

The Phase-0 session commented out this line to test isolation. Restore it.

Read the current state of `main/main.c` around line 154, then apply this Edit:

```c
// Old:
    } else {
        // 11. Start servers — three independent httpd instances so one
        //     slow handler can't starve another:
        //       :80  dashboard / WS (http_server.c)
        //       :81  MJPEG stream   (camera_stream.c)
        //       :82  /detect inference (detect_server.c)
        // PHASE-0 DEBUG: camera_stream temporarily disabled to isolate
        // whether the MJPEG path is the task colliding with /detect.
        // Revert by re-enabling this line once the root cause is found.
        // ESP_ERROR_CHECK(camera_stream_server_start());
        ESP_LOGW(TAG, "PHASE-0 DEBUG: camera_stream_server_start() SKIPPED");
        ESP_ERROR_CHECK(http_server_start());
        ESP_ERROR_CHECK(detect_server_start());
    }

// New:
    } else {
        // 11. Start servers — three independent httpd instances so one
        //     slow handler can't starve another:
        //       :80  dashboard / WS (http_server.c)
        //       :81  MJPEG stream   (camera_stream.c)
        //       :82  /detect inference (detect_server.c)
        ESP_ERROR_CHECK(camera_stream_server_start());
        ESP_ERROR_CHECK(http_server_start());
        ESP_ERROR_CHECK(detect_server_start());
    }
```

- [ ] **Step 1.2: Revert heap poisoning in sdkconfig.defaults**

```
// Old:
# --- Heap poisoning (COMPREHENSIVE) — catches use-after-free, overruns,
#     and free-list corruption. Temporarily raised from LIGHT while
#     debugging the ws_tx_task TLSF remove_free_block NULL-deref crash.
#     Overhead is ~5-10% vs LIGHT's ~2%; revert after root cause is fixed. ---
# CONFIG_HEAP_POISONING_LIGHT is not set
CONFIG_HEAP_POISONING_COMPREHENSIVE=y

// New:
# --- Heap poisoning (light) — catches corruption early, ~2% overhead ---
CONFIG_HEAP_POISONING_LIGHT=y
```

- [ ] **Step 1.3: Revert heap poisoning in sdkconfig**

The existing `sdkconfig` file has the COMPREHENSIVE setting from Phase-0. `idf.py build` honors `sdkconfig` over `sdkconfig.defaults` when both exist, so we must update both.

Read `sdkconfig` around line 1707 to confirm current state, then Edit:

```
// Old:
# CONFIG_HEAP_POISONING_DISABLED is not set
# CONFIG_HEAP_POISONING_LIGHT is not set
CONFIG_HEAP_POISONING_COMPREHENSIVE=y

// New:
# CONFIG_HEAP_POISONING_DISABLED is not set
CONFIG_HEAP_POISONING_LIGHT=y
# CONFIG_HEAP_POISONING_COMPREHENSIVE is not set
```

- [ ] **Step 1.4: Remove Phase-0 HWM debug logs from detect_server.c**

The HWM logs in `do_detect_locked` and `do_detect_800_locked` served their purpose (proved stack overflow isn't the cause). Remove them. The `#include "freertos/task.h"` stays — it's needed for vTaskDelay in the burst handler.

Read `main/detect_server.c` around lines 107-135, then Edit:

```c
// Old:
/* Body of detect_handler() extracted so the mutex give in the caller
 * is a single line. Runs inference on the 224×224 embedded image. */
static esp_err_t do_detect_locked(httpd_req_t *req)
{
    /* Phase-0 debug: bracket the esp-dl call with stack HWM checks so we
     * can see how close the httpd worker is to its stack limit during the
     * first (model-load) and subsequent (steady-state) calls. HWM is the
     * minimum free words seen since task start — we want this nowhere
     * near zero. */
    ESP_LOGW(TAG, "[hwm] /detect pre-run free=%u words",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded(&out);
    ESP_LOGW(TAG, "[hwm] /detect post-run free=%u words",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    return send_detect_response(req, &out, ret);
}

/* Body of detect_800_handler() — runs inference on the 800×640 embedded
 * image variant (camera resolution). Exercises the preprocessor letterbox
 * path that /detect bypasses. */
static esp_err_t do_detect_800_locked(httpd_req_t *req)
{
    ESP_LOGW(TAG, "[hwm] /detect/800 pre-run free=%u words",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded_800(&out);
    ESP_LOGW(TAG, "[hwm] /detect/800 post-run free=%u words",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    return send_detect_response(req, &out, ret);
}

// New:
/* Body of detect_handler() extracted so the mutex give in the caller
 * is a single line. Runs inference on the 224×224 embedded image. */
static esp_err_t do_detect_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded(&out);
    return send_detect_response(req, &out, ret);
}

/* Body of detect_800_handler() — runs inference on the 800×640 embedded
 * image variant (camera resolution). Exercises the preprocessor letterbox
 * path that /detect bypasses. */
static esp_err_t do_detect_800_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded_800(&out);
    return send_detect_response(req, &out, ret);
}
```

Task 4 will re-add gate brackets. We're just removing debug scaffolding in this step.

- [ ] **Step 1.5: Simplify /detect/burst — remove per-iteration debug probes**

The burst handler's per-iteration `heap_caps_check_integrity_all` / `heap_caps_get_free_size` / HWM probes were Phase-0 debug. Remove them and keep only the loop that calls `detection_run_on_embedded` N times. The burst endpoint stays registered (per spec Non-goals: "Eliminating /detect/burst — The debug endpoint stays"); only its internals simplify.

Read `main/detect_server.c` around `do_detect_burst_locked` (it's the ~80-line function added in the Phase-0 session with `#define DETECT_BURST_ROUNDS 5`), then Edit to replace with:

```c
static esp_err_t do_detect_burst_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = ESP_OK;

    ESP_LOGW(TAG, "[burst] start: %d rounds, %d ms gap",
             DETECT_BURST_ROUNDS, DETECT_BURST_GAP_MS);

    for (int i = 0; i < DETECT_BURST_ROUNDS; i++) {
        /* Release the previous round's JSON body before running the next
         * round, or the final send_detect_response() would leak every
         * intermediate allocation. */
        if (i > 0) {
            detection_response_free(&out);
        }
        memset(&out, 0, sizeof(out));
        ret = detection_run_on_embedded(&out);

        ESP_LOGW(TAG, "[burst] iter=%d/%d ret=%d",
                 i + 1, DETECT_BURST_ROUNDS, (int)ret);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[burst] iter=%d inference failed — aborting", i + 1);
            break;
        }

        if (i < DETECT_BURST_ROUNDS - 1) {
            vTaskDelay(pdMS_TO_TICKS(DETECT_BURST_GAP_MS));
        }
    }

    ESP_LOGW(TAG, "[burst] done");
    return send_detect_response(req, &out, ret);
}
```

The `#define DETECT_BURST_ROUNDS 5` and `#define DETECT_BURST_GAP_MS 1000` stay where they are above the function. Task 4 will re-wrap this function with the gate.

Also remove the `#include "esp_heap_caps.h"` line if it was added in Phase-0 — `heap_caps_check_integrity_all` is no longer called from this file. Check the top of `detect_server.c` and remove the Phase-0 debug-only include if present.

- [ ] **Step 1.6: Build to confirm clean baseline compiles**

```bash
idf.py build
```

Expected: successful build. Binary size approximately `0x325c50` bytes (pre-Phase-0 size). If build fails, check that the `#include "esp_heap_caps.h"` removal didn't break another file that relied on it (it shouldn't — only detect_server.c used it for Phase-0).

- [ ] **Step 1.7: Commit**

```bash
git add main/main.c main/detect_server.c sdkconfig.defaults sdkconfig
git commit -m "$(cat <<'EOF'
revert(phase-0): restore clean baseline before inference-gate work

- Re-enable camera_stream_server_start() in main.c
- HEAP_POISONING_LIGHT instead of COMPREHENSIVE in sdkconfig{,.defaults}
- Remove HWM debug logs from do_detect_locked / do_detect_800_locked
- Simplify /detect/burst to just the loop + vTaskDelay (drop per-iter
  heap integrity / free-size probes that were Phase-0 diagnostics)

Build verified. The coexistence crash this was debugging still exists
without these diagnostics; the next commits add the inference gate that
fixes it.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2 — Add ws_transport pause/resume

Add two public functions to the WS transport module that wrap `vTaskSuspend`/`vTaskResume` on its existing static `s_tx_task` handle. NULL-check to handle the pre-init window.

**Files:**
- Modify: `main/transports/ws_transport.h`
- Modify: `main/transports/ws_transport.c`

- [ ] **Step 2.1: Declare pause/resume in ws_transport.h**

Read `main/transports/ws_transport.h` first (it's short — 14 lines), then Edit:

```c
// Old:
#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Initialize the WebSocket transport and register it with the pipeline.
 *        Must be called after pipeline_init() and after httpd is started.
 * @param server  The httpd handle to register the /ws URI on.
 */
esp_err_t ws_transport_init(httpd_handle_t server);

/** httpd close callback — removes stale WS clients on any socket closure */
void ws_transport_close_fd(httpd_handle_t hd, int fd);

// New:
#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Initialize the WebSocket transport and register it with the pipeline.
 *        Must be called after pipeline_init() and after httpd is started.
 * @param server  The httpd handle to register the /ws URI on.
 */
esp_err_t ws_transport_init(httpd_handle_t server);

/** httpd close callback — removes stale WS clients on any socket closure */
void ws_transport_close_fd(httpd_handle_t hd, int fd);

/**
 * @brief Suspend the WS TX task so esp-dl inference can run without
 *        the WS send path allocating against it in PSRAM.
 *
 * Safe to call before init (no-op if s_tx_task is NULL). Idempotent:
 * vTaskSuspend on an already-suspended task is a FreeRTOS no-op.
 *
 * Intended to be called from detect_server.c's inference-gate critical
 * section. Must be balanced with ws_transport_resume() to avoid leaving
 * the task permanently stopped.
 */
void ws_transport_pause(void);

/**
 * @brief Resume the WS TX task after inference completes.
 *
 * Safe to call before init (no-op if s_tx_task is NULL). Idempotent:
 * vTaskResume on a running task is a FreeRTOS no-op, so the inference
 * gate's cleanup path can call this even if the gate watchdog already
 * force-resumed.
 */
void ws_transport_resume(void);
```

- [ ] **Step 2.2: Implement pause/resume in ws_transport.c**

Add the two function definitions. They can go at the end of the file, after `ws_transport_close_fd` and before any trailing comment. Read the file to find a stable insertion point (around the existing `ws_transport_close_fd` at ~line 249), then add:

```c
void ws_transport_pause(void)
{
    if (s_tx_task) {
        vTaskSuspend(s_tx_task);
    }
}

void ws_transport_resume(void)
{
    if (s_tx_task) {
        vTaskResume(s_tx_task);
    }
}
```

Place them immediately after the closing `}` of `ws_transport_close_fd` (or at the end of the file — pick whichever keeps related functions together).

- [ ] **Step 2.3: Build to verify**

```bash
idf.py build
```

Expected: successful build. The new functions aren't called yet (detect_server.c wiring happens in Task 4), so this only validates the header/implementation pair compiles.

- [ ] **Step 2.4: Commit**

```bash
git add main/transports/ws_transport.h main/transports/ws_transport.c
git commit -m "$(cat <<'EOF'
feat(ws): add ws_transport_pause / ws_transport_resume

Public pause/resume API that wraps vTaskSuspend/vTaskResume on the
existing s_tx_task handle. NULL-safe for the pre-init window and
idempotent (FreeRTOS no-ops on already-suspended/running tasks).

Part of the inference-gate work — detect_server.c will call these
around esp-dl inference to keep ws_tx_task from allocating against
esp-dl's tensor pool in PSRAM.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3 — Add camera_stream pause/resume + lazy task handle capture

camera_stream.c currently has no stored handle for its httpd worker task — the esp_http_server API doesn't expose it. Solution: capture it lazily inside `stream_handler` on the first client connection via `xTaskGetCurrentTaskHandle()`. Then pause/resume operate on the captured handle with NULL-check for the "no MJPEG client has ever connected" case.

**Files:**
- Modify: `main/camera_stream.h`
- Modify: `main/camera_stream.c`

- [ ] **Step 3.1: Declare pause/resume in camera_stream.h**

Read `main/camera_stream.h` (12 lines), then Edit to append:

```c
// Old:
#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Start the MJPEG HTTP stream server on CONFIG_HTTP_STREAM_PORT (default 81).
 *
 * Self-contained — does not share its httpd handle. The API/dashboard
 * server runs separately on CONFIG_HTTP_API_PORT (default 80).
 */
esp_err_t camera_stream_server_start(void);

// New:
#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Start the MJPEG HTTP stream server on CONFIG_HTTP_STREAM_PORT (default 81).
 *
 * Self-contained — does not share its httpd handle. The API/dashboard
 * server runs separately on CONFIG_HTTP_API_PORT (default 80).
 */
esp_err_t camera_stream_server_start(void);

/**
 * @brief Suspend the MJPEG stream worker task so esp-dl inference can
 *        run without the stream's JPEG encode + httpd send path
 *        allocating against it in PSRAM.
 *
 * The task handle is captured lazily inside stream_handler() on the
 * first /stream request. Before that, camera_stream_pause() is a no-op
 * — which is correct, because an idle port-81 httpd worker parked in
 * accept() holds no PSRAM allocations.
 *
 * Intended to be called from detect_server.c's inference-gate critical
 * section. Must be balanced with camera_stream_resume().
 */
void camera_stream_pause(void);

/**
 * @brief Resume the MJPEG stream worker task after inference completes.
 *
 * No-op if no client has ever connected. Idempotent: FreeRTOS no-ops
 * vTaskResume on a running task.
 */
void camera_stream_resume(void);
```

- [ ] **Step 3.2: Add static task handle + lazy capture in camera_stream.c**

Read `main/camera_stream.c` around the top of the file (includes + static variables) and around line 40 (the `stream_handler` entry point).

Add the static variable near the top of the file, among the existing static state. If the file has an `#include "freertos/task.h"` already, good; if not, add it along with the static.

First, check top-of-file includes: add `#include "freertos/task.h"` if missing (it's needed for `TaskHandle_t`, `xTaskGetCurrentTaskHandle`, `vTaskSuspend`, `vTaskResume`).

Then add the static variable near the other file-scope statics:

```c
/* Captured lazily on the first /stream request — see the comment in
 * stream_handler() below. Initialized to NULL so ..._pause() and
 * ..._resume() are safe no-ops before any MJPEG client has connected. */
static TaskHandle_t s_mjpeg_task = NULL;
```

- [ ] **Step 3.3: Lazy-capture the handle at the top of stream_handler**

Read `stream_handler` starting at line 40. Add the capture as the very first statement in the function body, before any existing logic:

```c
// New opening of stream_handler (before any existing logic):
static esp_err_t stream_handler(httpd_req_t *req)
{
    /* First request captures this httpd worker task's handle so that
     * the inference gate in detect_server.c can vTaskSuspend us during
     * esp-dl inference. Subsequent requests pass through. The esp_http_server
     * API doesn't expose the worker task handle directly, which is why
     * we grab it from inside the handler — xTaskGetCurrentTaskHandle()
     * inside the handler returns the worker task we care about. */
    if (s_mjpeg_task == NULL) {
        s_mjpeg_task = xTaskGetCurrentTaskHandle();
    }

    // ... existing stream_handler body unchanged ...
```

Use Edit to insert the capture block immediately after the opening brace of `stream_handler`, preserving the rest of the function verbatim. Do not touch any of the existing stream logic.

- [ ] **Step 3.4: Implement pause/resume at the end of camera_stream.c**

Find a stable insertion point — probably right before `camera_stream_server_start()` or at the very end of the file. Add:

```c
void camera_stream_pause(void)
{
    if (s_mjpeg_task) {
        vTaskSuspend(s_mjpeg_task);
    }
}

void camera_stream_resume(void)
{
    if (s_mjpeg_task) {
        vTaskResume(s_mjpeg_task);
    }
}
```

- [ ] **Step 3.5: Build to verify**

```bash
idf.py build
```

Expected: successful build. Not called yet, just validates the pair compiles.

- [ ] **Step 3.6: Commit**

```bash
git add main/camera_stream.h main/camera_stream.c
git commit -m "$(cat <<'EOF'
feat(camera): add camera_stream_pause / camera_stream_resume

Public pause/resume API for the MJPEG stream worker task. The port-81
httpd worker handle isn't exposed by esp_http_server, so stream_handler
captures xTaskGetCurrentTaskHandle() on the first request into a file-
scope static. Pre-first-request the handle is NULL and both functions
are safe no-ops — correct, because an idle port-81 worker parked in
accept() holds no PSRAM allocations worth suspending for.

Part of the inference-gate work — detect_server.c will call these
around esp-dl inference to keep the MJPEG encoder+send path from
allocating against esp-dl's tensor pool in PSRAM.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4 — Wire the gate into detect_server.c

The heart of the change. Add watchdog state and callback. Bracket all three detect handlers (`do_detect_locked`, `do_detect_800_locked`, `do_detect_burst_locked`) with pause/watchdog-start/inference/watchdog-stop/resume. Burst wraps the whole 5-iteration loop, not per-iteration.

**Files:**
- Modify: `main/detect_server.c`

- [ ] **Step 4.1: Add includes**

Read the include block at the top of `main/detect_server.c`, then Edit to add the three new includes alongside the existing ones:

```c
// Old (partial, showing the include block):
#include "detect_server.h"
#include "detection/detection.h"

#include <string.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"  /* Phase-0 debug: uxTaskGetStackHighWaterMark, vTaskDelay */

// New:
#include "detect_server.h"
#include "detection/detection.h"
#include "transports/ws_transport.h"
#include "camera_stream.h"

#include <string.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
```

The `freertos/task.h` comment is updated because we no longer call `uxTaskGetStackHighWaterMark` from this file — `vTaskDelay` is still needed by the burst handler.

- [ ] **Step 4.2: Add watchdog state and callback above the handler functions**

Read `main/detect_server.c` to find the insertion point — right after the existing `s_detect_mutex` declaration, before `kBusyJson`, or before `send_detect_response`. Edit to add:

```c
/* Inference-gate watchdog. 3-second one-shot esp_timer that fires if
 * inference takes absurdly long (normal worst case is ~500ms cold-start,
 * ~60ms steady state — 3s leaves 6x safety margin). When it fires, it
 * force-resumes both streamer subsystems so the dashboard recovers
 * from the deadlock case where vTaskSuspend caught a task holding a
 * shared lock esp-dl needs. Does NOT abort inference — esp-dl continues
 * on the handler task and eventually returns, at which point the
 * handler's cleanup path calls resume() again (FreeRTOS no-op on a
 * running task).
 *
 * Created once in detect_server_start() alongside s_detect_mutex. */
static esp_timer_handle_t s_gate_watchdog = NULL;
#define GATE_WATCHDOG_US   (3 * 1000 * 1000)   /* 3 seconds */

static void gate_watchdog_cb(void *arg)
{
    (void)arg;
    ESP_LOGE(TAG,
             "INFERENCE GATE WATCHDOG: inference exceeded %d ms, force-resuming streamers",
             GATE_WATCHDOG_US / 1000);
    camera_stream_resume();
    ws_transport_resume();
}
```

Place this block near the top of the file — right after the `static SemaphoreHandle_t s_detect_mutex = NULL;` declaration is a natural home.

- [ ] **Step 4.3: Rewrite do_detect_locked with the gate**

Read the current (Task-1-simplified) `do_detect_locked` and replace:

```c
// Old:
static esp_err_t do_detect_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded(&out);
    return send_detect_response(req, &out, ret);
}

// New:
static esp_err_t do_detect_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret;

    /* --- Inference gate: freeze the two streamer tasks we've proven
     * collide with esp-dl's PSRAM tensor pool, then run inference with
     * the chip effectively to ourselves. Watchdog is deadlock recovery
     * (see gate_watchdog_cb above). --- */
    ws_transport_pause();
    camera_stream_pause();
    esp_timer_start_once(s_gate_watchdog, GATE_WATCHDOG_US);

    ret = detection_run_on_embedded(&out);

    esp_timer_stop(s_gate_watchdog);
    camera_stream_resume();
    ws_transport_resume();
    /* --- end inference gate --- */

    return send_detect_response(req, &out, ret);
}
```

- [ ] **Step 4.4: Rewrite do_detect_800_locked with the gate**

```c
// Old:
static esp_err_t do_detect_800_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = detection_run_on_embedded_800(&out);
    return send_detect_response(req, &out, ret);
}

// New:
static esp_err_t do_detect_800_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret;

    ws_transport_pause();
    camera_stream_pause();
    esp_timer_start_once(s_gate_watchdog, GATE_WATCHDOG_US);

    ret = detection_run_on_embedded_800(&out);

    esp_timer_stop(s_gate_watchdog);
    camera_stream_resume();
    ws_transport_resume();

    return send_detect_response(req, &out, ret);
}
```

- [ ] **Step 4.5: Rewrite do_detect_burst_locked to gate the whole burst**

The burst handler from Task 1 is a plain loop without gate. Wrap the entire loop (not per-iteration) so the streamers are suspended for the whole ~5 second burst duration. The 5 s pause is longer than T1/T2 but still < 3 s * 2 = 6 s watchdog bound... wait, the watchdog is 3 s, and the burst takes ~5 s. **This matters.**

Burst duration breakdown: 500 ms cold-start + 4 × 56 ms steady = ~724 ms of inference + 4 × 1000 ms gap = 4000 ms gaps = **4.7 s total**. That's > 3 s watchdog, so the watchdog would fire mid-burst and force-resume the streamers half way through.

**Fix:** burst uses a LONGER watchdog, or the burst doesn't use the shared 3 s watchdog at all. Simplest: burst sets the watchdog to 10 s (plenty of headroom for 5 iterations) just for its own scope, and restores/doesn't use the one-shot timer's prior value (esp_timer one-shot is stateless — `esp_timer_start_once` just schedules a new fire time, so calling it with 10 s in burst and 3 s in the non-burst handlers is fine because the gate is serialized by `s_detect_mutex` — only one handler's watchdog is armed at a time).

So burst can simply call `esp_timer_start_once(s_gate_watchdog, 10 * 1000 * 1000)` instead of `GATE_WATCHDOG_US`. Add a `#define GATE_WATCHDOG_BURST_US (10 * 1000 * 1000)` next to the regular one.

First, extend the `#define`s (Edit to replace the existing single define):

```c
// Old:
#define GATE_WATCHDOG_US   (3 * 1000 * 1000)   /* 3 seconds */

// New:
#define GATE_WATCHDOG_US         (3  * 1000 * 1000)   /* 3 seconds — single-call detect */
#define GATE_WATCHDOG_BURST_US   (10 * 1000 * 1000)   /* 10 seconds — 5-round burst
                                                        * (~5 s total: 500 ms model load
                                                        *  + 4 × 56 ms inference
                                                        *  + 4 × 1000 ms gap)
                                                        * with 2x safety margin */
```

Then rewrite `do_detect_burst_locked`:

```c
// Old (from Task 1.5):
static esp_err_t do_detect_burst_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = ESP_OK;

    ESP_LOGW(TAG, "[burst] start: %d rounds, %d ms gap",
             DETECT_BURST_ROUNDS, DETECT_BURST_GAP_MS);

    for (int i = 0; i < DETECT_BURST_ROUNDS; i++) {
        if (i > 0) {
            detection_response_free(&out);
        }
        memset(&out, 0, sizeof(out));
        ret = detection_run_on_embedded(&out);

        ESP_LOGW(TAG, "[burst] iter=%d/%d ret=%d",
                 i + 1, DETECT_BURST_ROUNDS, (int)ret);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[burst] iter=%d inference failed — aborting", i + 1);
            break;
        }

        if (i < DETECT_BURST_ROUNDS - 1) {
            vTaskDelay(pdMS_TO_TICKS(DETECT_BURST_GAP_MS));
        }
    }

    ESP_LOGW(TAG, "[burst] done");
    return send_detect_response(req, &out, ret);
}

// New:
static esp_err_t do_detect_burst_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret = ESP_OK;

    ESP_LOGW(TAG, "[burst] start: %d rounds, %d ms gap (gate held for whole burst)",
             DETECT_BURST_ROUNDS, DETECT_BURST_GAP_MS);

    /* Gate the entire burst, not per-iteration. Simpler (one pause/resume
     * cycle instead of 5) and the burst is a debug tool so the longer
     * streamer freeze is acceptable. Uses GATE_WATCHDOG_BURST_US instead
     * of the single-call GATE_WATCHDOG_US because the 5-iteration loop
     * with 1 s gaps takes ~5 s total — longer than the 3 s single-call
     * watchdog. */
    ws_transport_pause();
    camera_stream_pause();
    esp_timer_start_once(s_gate_watchdog, GATE_WATCHDOG_BURST_US);

    for (int i = 0; i < DETECT_BURST_ROUNDS; i++) {
        /* Release the previous round's JSON body before running the next
         * round, or the final send_detect_response() would leak every
         * intermediate allocation. */
        if (i > 0) {
            detection_response_free(&out);
        }
        memset(&out, 0, sizeof(out));
        ret = detection_run_on_embedded(&out);

        ESP_LOGW(TAG, "[burst] iter=%d/%d ret=%d",
                 i + 1, DETECT_BURST_ROUNDS, (int)ret);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[burst] iter=%d inference failed — aborting", i + 1);
            break;
        }

        if (i < DETECT_BURST_ROUNDS - 1) {
            vTaskDelay(pdMS_TO_TICKS(DETECT_BURST_GAP_MS));
        }
    }

    esp_timer_stop(s_gate_watchdog);
    camera_stream_resume();
    ws_transport_resume();

    ESP_LOGW(TAG, "[burst] done");
    return send_detect_response(req, &out, ret);
}
```

- [ ] **Step 4.6: Create the watchdog timer in detect_server_start**

Read `detect_server_start()` to find the existing `s_detect_mutex` creation block, then Edit to add the timer creation right after it:

```c
// Old (the mutex creation block near the top of detect_server_start):
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

// New:
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

    /* Create the inference-gate watchdog alongside the mutex.
     * Process-lifetime; not destroyed. */
    if (s_gate_watchdog == NULL) {
        const esp_timer_create_args_t args = {
            .callback = gate_watchdog_cb,
            .arg      = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name     = "detect_gate_wd",
        };
        esp_err_t tret = esp_timer_create(&args, &s_gate_watchdog);
        if (tret != ESP_OK) {
            ESP_LOGE(TAG, "esp_timer_create(gate watchdog) failed: %s",
                     esp_err_to_name(tret));
            return tret;
        }
    }
```

- [ ] **Step 4.7: Build to verify**

```bash
idf.py build
```

Expected: successful build. Binary size slightly larger than Task 1 due to added gate code (~200 bytes of .text). If the build fails, the most likely causes:
- Missing `transports/ws_transport.h` include path — the file lives at `main/transports/`, so `#include "transports/ws_transport.h"` is correct given `main` is on the include path
- `esp_timer.h` not found — it's in ESP-IDF standard, should be available without CMake changes

- [ ] **Step 4.8: Commit**

```bash
git add main/detect_server.c
git commit -m "$(cat <<'EOF'
feat(detect): inference gate — suspend streamers during esp-dl run

Bracket do_detect_locked, do_detect_800_locked, and do_detect_burst_locked
with ws_transport_pause + camera_stream_pause before detection_run_on_*
and matching resumes after. Arms a 3-second one-shot esp_timer watchdog
(10 s for burst) that force-resumes both streamers if inference runs
absurdly long, providing recovery for the deadlock case where
vTaskSuspend caught a streamer holding a shared lock esp-dl wants.

The watchdog is created process-lifetime in detect_server_start()
alongside s_detect_mutex. When it fires it logs a loud ESP_LOGE line
("INFERENCE GATE WATCHDOG: inference exceeded ...") which is the
monitoring signal if the Option-B race window turns out to be a real
problem in practice.

Serialization invariant unchanged: s_detect_mutex still non-blocking-
takes around every handler, so concurrent /detect requests still get
HTTP 429 and only one gate acquisition is ever in flight.

Design: docs/superpowers/specs/2026-04-11-inference-gate-design.md

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5 — Flash and run T4 (negative test: no-dashboard baseline)

The negative test runs first because it needs no user interaction — confirms the gate doesn't break the no-dashboard case (which worked before this plan).

**This task requires hardware access (flash + curl). Cannot be run by a subagent.**

- [ ] **Step 5.1: Flash the new firmware**

```bash
idf.py flash
```

Expected: successful flash with "Hard resetting via RTS pin..." followed by "Done". Takes ~25 seconds. Device reboots automatically.

- [ ] **Step 5.2: Start serial capture in background**

```bash
timeout 20 cat /dev/ttyACM0 > /tmp/gate_t4.log 2>&1 &
```

Let it run for 20 seconds while the next steps execute.

- [ ] **Step 5.3: Wait for boot and curl /detect (no dashboard)**

Do NOT open the dashboard. The point of T4 is to verify the gate doesn't regress the no-dashboard path.

```bash
curl -s --max-time 15 --retry 30 --retry-connrefused --retry-delay 1 http://192.168.1.201:82/detect
```

Expected output: `{"ok":true,"model":"espdet_pico_224_224_cat","source":"embedded:espdet.jpg","image":{"w":224,"h":224},"inference_ms":<~500>,"detections":[...]}` — HTTP 200, one detection, cold-start latency.

- [ ] **Step 5.4: Check serial log for clean gate cycle**

```bash
grep -E "DETECT|WS_TRANSPORT|CORRUPT|Guru|panic|WATCHDOG" /tmp/gate_t4.log
```

Expected in the log:
- `DETECT_SRV: Detect server ready on port 82: /detect, /detect/image.jpg, /detect/800, /detect/800/image.jpg, /detect/burst`
- `DETECT: first /detect hit — instantiating CatDetect`
- `DETECT: /detect (embedded:espdet.jpg): 1 detections in ~500 ms`

Expected NOT in the log:
- No `Guru Meditation Error`
- No `CORRUPT HEAP`
- No `INFERENCE GATE WATCHDOG` (watchdog should not fire — inference completed well under 3 s)
- No `Rebooting...`

**Pass criteria:** Curl returned 200 OK. Serial log shows clean detection with no panic. Device is still running (can verify with a second curl).

If this fails, the gate broke the no-dashboard path — debug before moving on. Most likely causes:
- `camera_stream_pause()` is suspending the port-81 httpd worker even though no client connected (it shouldn't — `s_mjpeg_task` should be NULL if no request has hit `/stream`)
- The watchdog fired for some reason (log would show it)

---

## Task 6 — Run T1 (primary success test, dashboard connected)

**This task requires you to manually open the dashboard in a browser.**

- [ ] **Step 6.1: Open dashboard in browser**

Open `http://192.168.1.201/` in a fresh browser tab. Confirm:
- Dashboard HTML loads
- Sensor data updates (IMU values / ToF grid moving at 20 Hz)
- Camera MJPEG feed shows live frames (800x640 video updating)

If any of the three isn't working, refresh the tab and wait 2 seconds. If still not working, the device might have an unrelated issue — stop and debug.

- [ ] **Step 6.2: Start serial capture in background**

```bash
timeout 20 cat /dev/ttyACM0 > /tmp/gate_t1.log 2>&1 &
```

- [ ] **Step 6.3: Curl /detect (dashboard is live)**

```bash
curl -s --max-time 15 http://192.168.1.201:82/detect
```

Expected output: same JSON as T4 — `{"ok":true,..."inference_ms":~60,..."detections":[...]}`. Latency may be 60 ms (steady-state, if the dashboard's own /detect button has already triggered a model load) or ~500 ms (cold start, if this is the first inference).

**While inference is running (for the ~500 ms or ~60 ms), the dashboard MJPEG `<img>` will briefly freeze on one frame and WS sensor data will show a brief gap.** That's the gate pausing them. They should resume immediately after curl returns.

- [ ] **Step 6.4: Check serial log**

```bash
grep -E "DETECT|WS_TRANSPORT|CORRUPT|Guru|panic|WATCHDOG" /tmp/gate_t1.log
```

**Expected (pass criteria):**
- `WS_TRANSPORT: Client connected (fd=XX, total=1)` — the dashboard's WebSocket
- `DETECT: first /detect hit` OR `DETECT: /detect (embedded:espdet.jpg): 1 detections in ~60 ms`
- NO `Guru Meditation Error`
- NO `CORRUPT HEAP`
- NO `INFERENCE GATE WATCHDOG`
- NO `Rebooting`
- Device still responsive (can do a second curl)

**If the crash still reproduces** (you see Guru/CORRUPT/Rebooting), the gate is not catching one or both of the colliders. Most likely:
- `s_mjpeg_task` didn't get captured because something about the stream handler doesn't match my assumption — verify by adding a debug log in `stream_handler` and re-testing
- The WS-side collision is actually from a different task than `ws_tx_task` — unlikely given the earlier crash fingerprint pointed directly at `ws_tx_task`

---

## Task 7 — Run T2 (steady-state back-to-back)

- [ ] **Step 7.1: Start fresh serial capture**

```bash
timeout 30 cat /dev/ttyACM0 > /tmp/gate_t2.log 2>&1 &
```

- [ ] **Step 7.2: Run 10 sequential /detect calls**

```bash
for i in 1 2 3 4 5 6 7 8 9 10; do
    echo "--- call $i ---"
    curl -s --max-time 5 http://192.168.1.201:82/detect
    echo
    sleep 1
done
```

Expected: all 10 print HTTP 200 JSON. First might be 500 ms if the previous test's model load was lost, rest should be ~60 ms each.

- [ ] **Step 7.3: Verify log**

```bash
grep -cE "DETECT:.*1 detections" /tmp/gate_t2.log
grep -cE "CORRUPT|Guru|Rebooting" /tmp/gate_t2.log
```

Expected:
- `1 detections` count: 10
- `CORRUPT|Guru|Rebooting` count: 0

Dashboard should have ticked through 10 micro-freezes (~60 ms each) but recovered each time.

---

## Task 8 — Run T3 (burst stress test)

- [ ] **Step 8.1: Start serial capture**

```bash
timeout 20 cat /dev/ttyACM0 > /tmp/gate_t3.log 2>&1 &
```

- [ ] **Step 8.2: Curl /detect/burst**

```bash
curl -s --max-time 20 http://192.168.1.201:82/detect/burst
```

Expected: returns after ~5 seconds with a single detection JSON body (the last iteration's result). Dashboard MJPEG freezes for the full 5 seconds, then recovers. Browser may show a stale frame during the entire pause.

- [ ] **Step 8.3: Verify log**

```bash
grep -E "burst|CORRUPT|Guru|WATCHDOG|Rebooting" /tmp/gate_t3.log
```

**Expected:**
- `[burst] start: 5 rounds, 1000 ms gap (gate held for whole burst)`
- `[burst] iter=1/5 ret=0`
- `[burst] iter=2/5 ret=0`
- `[burst] iter=3/5 ret=0`
- `[burst] iter=4/5 ret=0`
- `[burst] iter=5/5 ret=0`
- `[burst] done`
- NO `CORRUPT`, NO `Guru`, NO `WATCHDOG`, NO `Rebooting`

**Pass criteria:** all 5 iterations succeed, no crash, burst watchdog (10 s) did not fire.

---

## Task 9 — Run T5 (watchdog recovery verification)

This test temporarily injects a 5-second delay into `do_detect_locked` to force the watchdog to fire, verifies recovery, then reverts.

**Files:**
- Temporarily modify: `main/detect_server.c` (inject vTaskDelay, then revert)

- [ ] **Step 9.1: Inject a 5-second delay**

Read the current `do_detect_locked`, then Edit to add a temporary `vTaskDelay(pdMS_TO_TICKS(5000));` between the watchdog start and the inference call:

```c
// Temporarily modified (for T5 only — REVERT IN STEP 9.7):
static esp_err_t do_detect_locked(httpd_req_t *req)
{
    detection_response_t out = {0};
    esp_err_t ret;

    ws_transport_pause();
    camera_stream_pause();
    esp_timer_start_once(s_gate_watchdog, GATE_WATCHDOG_US);

    /* T5 INJECTED DELAY — REVERT BEFORE COMMITTING */
    vTaskDelay(pdMS_TO_TICKS(5000));

    ret = detection_run_on_embedded(&out);

    esp_timer_stop(s_gate_watchdog);
    camera_stream_resume();
    ws_transport_resume();

    return send_detect_response(req, &out, ret);
}
```

- [ ] **Step 9.2: Rebuild and flash**

```bash
idf.py build
idf.py flash
```

- [ ] **Step 9.3: Open dashboard (if not still open from T1)**

Same as Task 6 Step 1. Dashboard must be live for this test to have meaning.

- [ ] **Step 9.4: Start serial capture**

```bash
timeout 20 cat /dev/ttyACM0 > /tmp/gate_t5.log 2>&1 &
```

- [ ] **Step 9.5: Curl /detect with long timeout**

```bash
curl -s --max-time 15 http://192.168.1.201:82/detect
```

Expected behavior:
- Curl hangs for ~5.5 seconds (5 s injected delay + ~500 ms real inference)
- Eventually returns 200 OK with detection JSON — **device does not crash**
- While curl is hanging, the dashboard streamers are initially suspended, then force-resumed after 3 seconds by the watchdog, so the dashboard recovers ~2 seconds before curl returns

- [ ] **Step 9.6: Verify watchdog fired and recovered**

```bash
grep -E "WATCHDOG|Rebooting|Guru|CORRUPT" /tmp/gate_t5.log
```

**Expected:**
- `INFERENCE GATE WATCHDOG: inference exceeded 3000 ms, force-resuming streamers` — this is the line that proves the watchdog fired and recovered
- NO `Rebooting`, NO `Guru`, NO `CORRUPT`

**Pass criteria:** curl eventually returned 200, watchdog log line present, no device crash.

- [ ] **Step 9.7: Revert the injected delay**

Remove the `vTaskDelay` line and any associated comment. `do_detect_locked` should be back to the Task 4 state:

```c
// Reverted (back to Task 4 state):
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

- [ ] **Step 9.8: Rebuild and re-flash to confirm revert**

```bash
idf.py build
idf.py flash
```

- [ ] **Step 9.9: Re-run T1 to confirm revert worked**

Same steps as Task 6: start capture, curl /detect with dashboard open, verify clean log. Expected: inference completes in ~60ms-500ms normally, no watchdog fires, no crash.

No commit for Task 9 — T5 is transient, revert must be clean.

---

## Task 10 — Final clean-state verification + done

- [ ] **Step 10.1: Verify git state**

```bash
git status
git log --oneline -5
```

Expected:
- Working tree clean (Task 9's revert should be complete)
- Last 4 commits are the ones from Tasks 1-4 (revert, ws, camera, detect)
- No untracked changes to source files

If `git status` shows unstaged changes in `detect_server.c`, the T5 delay revert was incomplete — go back to Task 9 Step 7 and finish.

- [ ] **Step 10.2: Re-run T4 one final time (negative test with clean build)**

The final build must also pass T4 — the no-dashboard case — to confirm nothing regressed in the revert + rebuild cycle:

```bash
# Close dashboard browser tab first
timeout 15 cat /dev/ttyACM0 > /tmp/gate_final.log 2>&1 &
curl -s --max-time 15 http://192.168.1.201:82/detect
grep -E "DETECT|CORRUPT|Guru" /tmp/gate_final.log
```

Expected: curl returns 200 with detection JSON. Log shows clean detection. No crash markers.

- [ ] **Step 10.3: Summary report for the user**

Print a short report:
- Tasks 5-8 (T1, T2, T3, T4) results: pass/fail
- Task 9 (T5 watchdog test) result: pass/fail
- Any anomalies observed

If everything passed, the gate is in place and the implementation plan is complete. The /detect endpoint can now be hit while the dashboard is open without crashing the device.

---

## Post-implementation notes

**What to watch for in future logs:**

```
INFERENCE GATE WATCHDOG: inference exceeded 3000 ms, force-resuming streamers
```

If this line appears in production logs during normal (non-injected) use, it means Option B's deadlock race actually fires. Depending on frequency, the response is:
- **Rare (< 1/1000 requests):** leave as-is, the watchdog recovers
- **Occasional (1/100):** tighten the watchdog from 3 s to 2 s, re-measure
- **Frequent (every few requests):** fall back to Option A (cooperative self-suspension) — this is a new design session, not a fix to this plan

**What to remove when the production pivot lands:**

When the device pivots to local inference + periodic ESP-NOW report (no streaming, no /detect HTTP endpoint), the gate becomes dead code:
1. Delete `ws_transport_pause`/`ws_transport_resume` from `ws_transport.{h,c}`
2. Delete `camera_stream_pause`/`camera_stream_resume` + `s_mjpeg_task` from `camera_stream.{h,c}`
3. Delete the gate wrapping from `detect_server.c` handlers + watchdog timer + callback
4. Delete the entire `ws_transport.c` and `camera_stream.c` files if the streaming infrastructure is fully removed

That's a ~15 line revert per file. The spec's Non-goals section acknowledges this removal path.

---

## Self-review

**Spec coverage check** (against `2026-04-11-inference-gate-design.md`):

| Spec section | Plan task(s) | Status |
|---|---|---|
| Architecture order-of-ops (pause → pause → watchdog → infer → stop → resume → resume) | Task 4 Steps 4.3, 4.4, 4.5 | ✓ |
| Tasks NOT suspended (sensor, IMU/ToF, SDIO, port-80/82 httpds) | Not touched; implicit | ✓ |
| ws_transport.{h,c} — pause/resume | Task 2 | ✓ |
| camera_stream.{h,c} — pause/resume + lazy capture | Task 3 | ✓ |
| detect_server.c — includes, watchdog state, callback, handler rewrites (3 of them), timer creation in detect_server_start | Task 4 Steps 4.1–4.6 | ✓ |
| Data flow diagram | Implemented by Task 4 implementation | ✓ |
| Error handling: inference fails mid-way (single-exit) | Task 4.3/4/5 — ret captured before cleanup; return after cleanup | ✓ |
| Error handling: watchdog fires | Task 4.2 callback | ✓ |
| Error handling: NULL task handle | Task 2.2 + Task 3.4 null-checks | ✓ |
| Error handling: double-suspend/resume | FreeRTOS semantics; covered by using idempotent operations | ✓ |
| Error handling: suspended task holds lock | Task 4.2 watchdog is the recovery path | ✓ |
| T1 primary success test (dashboard live, curl /detect) | Task 6 | ✓ |
| T2 steady-state × 10 | Task 7 | ✓ |
| T3 burst with dashboard | Task 8 | ✓ |
| T4 negative test (no dashboard) | Task 5, re-verified Task 10.2 | ✓ |
| T5 watchdog recovery via injected 5 s delay | Task 9 | ✓ |
| Phase-0 diagnostic cleanup (revert main.c, sdkconfig, HWM logs) | Task 1 | ✓ |

**Placeholder scan:** searched for "TBD", "TODO", "implement later", "appropriate", "similar to" — none present. All code blocks are complete.

**Type consistency check:**
- `ws_transport_pause` / `ws_transport_resume` — same spelling in header, implementation, and all 3 handler call sites ✓
- `camera_stream_pause` / `camera_stream_resume` — same spelling throughout ✓
- `s_gate_watchdog` — declared once, used in create/start_once/stop/callback ✓
- `GATE_WATCHDOG_US` (3s) used in do_detect_locked and do_detect_800_locked; `GATE_WATCHDOG_BURST_US` (10s) used in do_detect_burst_locked ✓
- `gate_watchdog_cb` — declared static, passed to `esp_timer_create_args_t.callback` ✓
- `s_mjpeg_task` — declared as static TaskHandle_t, set in stream_handler, read in pause/resume ✓
- `s_tx_task` — pre-existing, used unchanged ✓

**Known gaps / things the plan explicitly defers:**
- Co-residency regression harness rewrite (`tools/test_detect_coresidency.py`) — still broken from ultraplan Phase A, not addressed here, tracked separately
- fd budget audit (ultraplan S2) — orthogonal
- Production pivot to local inference + ESP-NOW — new design session, not a fix to this plan
