# /detect Serialization — Rapid-Fire Hardening

**Date:** 2026-04-10
**Status:** Design
**Follows:** `2026-04-10-detect-endpoint-phase1-design.md`
**Precedes:** maritime model swap, live-camera inference

---

## Summary

Make `GET /detect` on port 82 safe against rapid-fire client requests by serializing the handler with a non-blocking FreeRTOS mutex. Concurrent requests return `HTTP 429 Too Many Requests` with a JSON error body instead of crashing the device. Remove the dashboard's `max="5"` rounds cap (which today exists as a cowardly workaround for the crash) and make the dashboard retry on 429.

This is the "simple working concept" milestone. The cat model is a placeholder; the Pico architecture is proven. Before we swap in the maritime Pico model and before we wire live camera inference, `/detect` must be unbreakable from the client side. This spec ensures that.

## Problem

The current `/detect` endpoint works correctly for single and well-spaced calls. Under rapid-fire (dashboard auto-run firing simultaneously with a manual click, or a benchmark loop, or two tabs open) the device panics and reboots mid-inference. The existing mitigation is a UI-side cap of `max="5"` rounds on `#detect-rounds` with a tooltip reading *"esp-dl crashes under rapid repeat calls"*. That's not a working concept — it's a lie the dashboard tells itself.

The previous attempt (commit `4a2f539`) isolated `/detect` on its own httpd instance on port 82 to prevent WebSocket starvation on port 80 during inference. That helped the WS coexistence problem but did not address the rapid-fire case, because the new server is still reachable by multiple concurrent TCP connections and the underlying esp-dl state is still not safe against back-to-back calls.

## Goals

1. **Client-unreachable crash.** No sequence of HTTP requests from any client can reboot the device. The rapid-fire crash becomes impossible to trigger through the `/detect` surface.
2. **Honest contract.** Clients that fire too fast get a crisp `429` they can retry, not an invisible wait followed by a panic.
3. **Zero changes to the proven infrastructure.** `detection.cpp`, `detection.h`, partition table, ports, CORS, WebSocket transport, camera stream — all untouched. The fix lives entirely in `detect_server.c` and `dashboard.html`.
4. **Preserve the `LOAD-BEARING INVARIANT`.** `detection_run_on_embedded()` must still only be called from the single httpd worker task, and the static globals (`s_detect`, `s_decoded_img`, `s_decode_done`) must still be accessed without their own mutex. Explicit serialization at the handler strengthens this invariant rather than weakening it.
5. **Forward-compatible.** When the maritime Pico model swaps in, no serialization code needs to change. When Phase 3 adds a live-camera request path, the same mutex guards it.

## Non-Goals

- **Not** root-causing why esp-dl panics under back-to-back calls. Making the condition unreachable from clients is sufficient for "working concept." Root cause investigation can happen later on its own branch with a standalone reproducer.
- **Not** adding rate limiting (minimum gap between successful calls). A client that spaces calls 100 ms apart should succeed; only *overlapping* calls return 429.
- **Not** adding an application-level request queue or a dedicated detect task. The TCP accept queue is the only queue; callers that collide are rejected immediately.
- **Not** touching the maritime model, live camera capture, WS protobuf pipeline, or partition layout.
- **Not** changing the `detection_run_on_embedded` C API or the success JSON schema.

## Design

### Architecture

One FreeRTOS binary semaphore (`s_detect_mutex`) lives in `detect_server.c` as a static module-scope handle. It is created in `detect_server_start()` immediately before `httpd_start()`. It guards the entire body of `detect_handler()`. `detect_image_handler()` (the static JPEG overlay endpoint) does **not** take the mutex — it's a rodata byte dump with no inference state and no heap pressure.

The mutex is acquired non-blocking. `xSemaphoreTake(s_detect_mutex, 0)` returns immediately with `pdTRUE` (acquired) or `pdFALSE` (busy). On `pdFALSE` the handler writes a 429 response and returns. On `pdTRUE` the handler runs `detection_run_on_embedded()` and the existing send logic, then releases the mutex on every exit path.

### Handler flow

```
detect_handler(req):
    if xSemaphoreTake(s_detect_mutex, 0) != pdTRUE:
        set CORS header
        set status "429 Too Many Requests"
        set content-type application/json
        send kBusyJson (static literal, see below)
        return ESP_OK

    # ---- critical section ----
    out = {0}
    ret = detection_run_on_embedded(&out)

    if ret != ESP_OK:
        set CORS header
        send 500
        xSemaphoreGive(s_detect_mutex)
        return ret

    set content-type application/json
    set CORS header
    set status (200 OK or 500 based on out.http_status)

    if out.json && out.json_len > 0:
        send_ret = httpd_resp_send(req, out.json, out.json_len)
        detection_response_free(&out)
        xSemaphoreGive(s_detect_mutex)
        return send_ret

    detection_response_free(&out)
    send 500
    xSemaphoreGive(s_detect_mutex)
    return ESP_OK
    # ---- end critical section ----
```

Release is present on every possible exit. To avoid "forgot to release" bugs in future edits, the refactored handler uses a single `cleanup:` label (goto on error) or — preferred — lifts the body into a helper `do_detect_locked(req)` that runs under the mutex with a single caller-side `give` after the helper returns. Both are acceptable; the helper form is slightly cleaner and is what I'll propose in the implementation plan.

### 429 response body

Static literal (no allocation, no escaping, no builders):

```c
static const char kBusyJson[] =
    "{\"ok\":false,\"stage\":\"busy\",\"error\":\"detect already in flight\"}";
static const size_t kBusyJsonLen = sizeof(kBusyJson) - 1;  /* exclude NUL */
```

Matches the shape of `build_error_json()` responses (`ok`, `stage`, `error`) so clients don't need a special parser. No `DETECTION_STAGE_BUSY` enum value is added to `detection.h` because busy is a transport-layer state, not a detection pipeline stage. Keeping it out of the enum preserves `detection.h`'s single-purpose API.

### CORS

All three response paths (200, 500, 429) set `Access-Control-Allow-Origin: *`, matching the existing contract. The dashboard fetches `/detect` from port 80 while it's served from port 82, so CORS is non-negotiable on every exit.

### Dashboard changes

`main/dashboard.html`:

1. **Remove the cap.** Change `<input id="detect-rounds" type="number" min="1" max="5" value="1" title="capped at 5 — esp-dl crashes under rapid repeat calls">` to `<input id="detect-rounds" type="number" min="1" value="1">`. Remove the tooltip entirely.
2. **Handle 429 in `runDetect()`.** When `fetch(\`${DETECT_BASE}/detect\`)` returns `response.status === 429`, await a 50 ms delay and retry the same round without incrementing the round counter. Cap total retries per round at 20 (1 second of backoff) so a genuinely wedged server doesn't spin forever. On retry exhaustion, log the retry count and treat it as a failure for that round.
3. **No UI spinner change.** The user doesn't need to see 429s; retries are invisible unless they're exhausted.

The dashboard's existing "auto-run on mode switch" logic stays as is. If it fires simultaneously with a manual click, one of them gets a 429 and retries transparently.

### Mutex initialization

`detect_server_start()`:

```c
s_detect_mutex = xSemaphoreCreateMutex();
if (!s_detect_mutex) {
    ESP_LOGE(TAG, "failed to create detect mutex");
    return ESP_ERR_NO_MEM;
}
```

Created **before** `httpd_start()` so that if a request arrives on the first packet after httpd is up, the mutex is already live. Created **once** — `detect_server_start()` is called once at boot from `main.c`. No destroy path; the detect server lives for the entire process lifetime.

A binary mutex (created via `xSemaphoreCreateMutex`) is preferable to a counting semaphore here because the ownership model is "one holder at a time" and the mutex supports priority inheritance. We aren't using priority inheritance today (the httpd worker is the only taker), but it's free insurance when Phase 3 introduces a live-camera task that might also contend for the mutex.

## Components Changed

| File | Change |
|------|--------|
| `main/detect_server.c` | Add `s_detect_mutex`, create in `_start()`, take/give in handler, add `kBusyJson` literal, refactor handler to ensure release on every exit |
| `main/dashboard.html` | Remove `max="5"` and tooltip, add 429 retry loop in `runDetect()` |

No changes to:
- `main/detection/detection.cpp`
- `main/detection/detection.h`
- `main/detect_server.h`
- `main/http_server.c`
- `main/camera_stream.c`
- `main/main.c`
- `partitions.csv`
- `sdkconfig.defaults`
- `main/CMakeLists.txt`

## Testing

Each test is a manual verification on the physical boat (no automated test harness exists for HTTP handlers in this project). All tests use `curl` against the boat's IP on port 82, or the browser dashboard.

1. **Single call baseline.** `curl http://<boat>:82/detect` → 200 with one cat detection, roughly 545 ms first-call latency, ~56 ms steady-state. No regression from current behavior.
2. **Simultaneous double call.** `curl http://<boat>:82/detect & curl http://<boat>:82/detect &` → one returns 200 with a detection, the other returns 429 with the busy body. Device does not reboot.
3. **Curl rapid-fire loop.** `while true; do curl -s -o /dev/null -w "%{http_code}\n" http://<boat>:82/detect; done | head -100` → mix of 200 and 429 codes, no reboots, no crashes, serial log stays clean. Run for at least one minute.
4. **Dashboard auto-run + manual click race.** Switch the dashboard to detect mode and immediately (within 100 ms) click Run. Expected: one round succeeds, the racing click is transparently retried by the dashboard and eventually succeeds. No visible error in the UI.
5. **Sequential sustained load.** `for i in {1..100}; do curl http://<boat>:82/detect; sleep 0.2; done` — all 200, no reboot. Validates that serialization does not hide a latent memory leak across sequential calls.
6. **CORS preservation.** `curl -i -H "Origin: http://localhost" http://<boat>:82/detect` on all three status codes (200, 500, 429) shows `Access-Control-Allow-Origin: *` in the response headers.
7. **Co-residency during contention.** While running test 3, keep the dashboard open on port 80 with WS connected. WS frames continue streaming (no stall), IMU fusion stays continuous (heading doesn't freeze), MJPEG stream on port 81 stays alive. This is the Phase 1 co-residency test re-run under load.

Pass criteria: zero reboots across all seven tests, and test 7 shows no WS/IMU/MJPEG interruption.

## Failure Modes and Risks

| Failure mode | Mitigation |
|--------------|------------|
| Handler panics between `Take` and `Give` | Device reboots, mutex state is reset on boot — no permanent stuck-busy state. Light concern only. |
| `xSemaphoreCreateMutex()` fails at boot | `detect_server_start()` returns `ESP_ERR_NO_MEM`, `main.c` `ESP_ERROR_CHECK` reboots the device at boot. Loud, bisectable, acceptable. |
| Client loops on 429 without backoff | Server returns 429 as fast as it can handle — not a crash risk, just a useless load. Dashboard has the 50 ms backoff built in; third-party clients are on their own. Not in scope. |
| Mutex acquired but httpd timeout fires mid-inference | httpd's `send_wait_timeout` and `recv_wait_timeout` only affect socket I/O, not the handler. The handler owns the mutex for the duration of inference regardless. No risk. |
| Live-camera inference (Phase 3) acquires the same mutex from a different task | Exactly what we want — the mutex is the serialization point for all inference paths, regardless of who triggers them. This is forward-compatible by design. |
| Underlying esp-dl crash on some unknown sequential pattern | Not addressed. Out of scope. If it surfaces in testing, it becomes its own spec. |

## Open Questions

None that block implementation. The one honest uncertainty is whether the rapid-fire crash is purely a concurrency artifact (fixed by this spec) or a deeper esp-dl state-reuse bug that will still manifest on sequential calls at some rate. Test 5 (100 sequential calls with 200 ms gap) is the empirical check. If test 5 fails, we reopen the question and investigate esp-dl internals; that becomes a separate spec, not a scope expansion of this one.

## What This Unblocks

After this lands:
- **Maritime swap** can drop the `.espdl` file into the cat_detect component's `models/p4/` directory, update the class labels, and ship. No concurrency concerns.
- **Live camera inference (Phase 3)** can introduce a second trigger path for `detection_run_on_embedded()` and the mutex will serialize it against client-triggered calls automatically.
- The dashboard can finally expose an "unlimited rounds" benchmark mode if desired, with 429s handled transparently.
