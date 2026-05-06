# MJPEG Stream Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the dashboard's MJPEG stream survive browser refreshes, inference cycles, and long idle periods without thrashing, leaking sockets, or wedging — without abandoning the ESP-IDF-idiomatic `multipart/x-mixed-replace` architecture.

**Architecture:** Three coordinated surgical fixes: (1) port-81 httpd gains the same socket-hygiene config as port 80 (`close_fn`, LRU purge, tighter send timeout, larger pool); (2) the dashboard's JS watchdog gains exponential backoff and a WS-health gate so it doesn't restart the stream during known-bad windows; (3) a `beforeunload` listener gives the firmware a clean FIN on browser navigation instead of an RST storm. No protobuf, no schema, no new transport.

**Tech Stack:** ESP-IDF v5.4 `esp_http_server`, vanilla browser JavaScript, embedded HTML (baked into firmware via `EMBED_TXTFILES`).

---

## File Structure

| File | What changes | Why |
|---|---|---|
| `main/camera_stream.c` | Adds `camera_stream_on_close()`; extends `httpd_config_t` in `camera_stream_server_start()` | Port 81 socket hygiene (matches port 80 pattern) |
| `main/dashboard.html` | Adds `_streamBackoffMs` state, updates `startStream()`, adds WS-health gate in the 5s interval watchdog, adds `beforeunload` listener | Debounce restarts, align with WS lifecycle, clean shutdown |

Two files. No new files. No protobuf. No sdkconfig. No header changes.

---

## Flash Strategy

The dashboard HTML is embedded into the firmware binary (`main/CMakeLists.txt: EMBED_TXTFILES ${project_dir}/main/dashboard.html`). Every change — firmware or dashboard — requires `idf.py flash`. To save cycles:

- **Flash once after Task 1** — to validate the firmware httpd change in isolation before dashboard changes layer on top.
- **Flash once after Task 4** — picks up all three dashboard changes together.
- Task 5 is verification; no flash unless something needs fixing.

Every flash requires the monitor be stopped first (port held by `idf.py flash`), then restarted after. Use `TaskStop` on the boatmon Monitor before flashing.

---

### Task 1: Firmware port-81 httpd hardening

**Files:**
- Modify: `main/camera_stream.c` (add new function + extend one config block)

**Context:** `camera_stream_server_start()` configures the port-81 httpd instance for the MJPEG stream. Currently it sets only `server_port`, `ctrl_port`, `stack_size`, and `max_open_sockets=4`. All other fields inherit `HTTPD_DEFAULT_CONFIG()` defaults. We need to add a close callback, enable LRU purge, shorten the send timeout, and grow the pool — the same pattern port 80 already uses via `ws_transport_close_fd`.

**ESP-IDF reference:** `httpd_config_t` docs ([httpd_config_t](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32p4/api-reference/protocols/esp_http_server.html#_CPPv415httpd_config_t)). `close_fn` is called whenever httpd tears a socket down; its responsibility is to `close()` the fd (the default impl just calls `close(sockfd)`).

- [ ] **Step 1: Read the current httpd_config block to verify exact location and surrounding context.**

Run:
```bash
grep -n "camera_stream_server_start\|httpd_config_t cfg\|max_open_sockets" /workspaces/BoatEspP4/main/camera_stream.c
```

Expected: shows `static esp_err_t camera_stream_server_start(void)` declaration and the `cfg` block around lines 112-125.

- [ ] **Step 2: Add the `close_fn` helper at the top of the file.**

Insert right before `camera_stream_server_start()` (or anywhere in the static-function area — use `#include <unistd.h>` to pull in `close()` if it isn't already present; grep confirms `unistd.h` is included in http_server.c so it's a standard pattern in this codebase):

```c
static void camera_stream_on_close(httpd_handle_t hd, int sockfd) {
    (void)hd;
    ESP_LOGI(TAG, "socket %d closed", sockfd);
    close(sockfd);
}
```

Verify `<unistd.h>` is included at the top of the file. If not, add `#include <unistd.h>` to the existing include block.

- [ ] **Step 3: Extend the httpd_config block in `camera_stream_server_start()`.**

Find:
```c
httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
cfg.server_port      = 81;
cfg.ctrl_port        = 32769;
cfg.stack_size       = 8192;
cfg.max_open_sockets = 4;
```

Replace with:
```c
httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
cfg.server_port      = 81;
cfg.ctrl_port        = 32769;
cfg.stack_size       = 8192;
cfg.max_open_sockets = 7;                      // was 4 — more headroom for refresh races
cfg.send_wait_timeout = 2;                     // was default 5s — drop slow clients faster
cfg.lru_purge_enable = true;                   // on pool full, evict oldest instead of refusing new
cfg.close_fn         = camera_stream_on_close; // clean fd close on browser FIN/RST
```

- [ ] **Step 4: Build.**

Run:
```bash
. $IDF_PATH/export.sh >/dev/null 2>&1 && idf.py build 2>&1 | tail -10
```

Expected: `Project build complete.` No compile errors. If you see `implicit declaration of function 'close'`, add `#include <unistd.h>` to the include block.

- [ ] **Step 5: Flash + verify with monitor.**

If a monitor is running, stop it first (`TaskStop` on the boatmon task). Then:
```bash
. $IDF_PATH/export.sh >/dev/null 2>&1 && idf.py -p /dev/ttyACM0 flash 2>&1 | tail -5
```

Restart the monitor:
```
Monitor(
  description: "BoatEspP4 events — port 81 hardening verify",
  command: "python3 -u /workspaces/BoatEspP4/.claude/skills/firmware-event-monitor/boatmon.py",
  persistent: true,
  timeout_ms: 3600000
)
```

Ask user to open dashboard, let stream run 10s, then F5-refresh. Expected in monitor output: a new `CAM_STREAM: socket N closed` line on each refresh. If not, the `close_fn` isn't wired — re-read Step 3.

- [ ] **Step 6: Commit.**

```bash
git add main/camera_stream.c
git commit -m "$(cat <<'EOF'
fix(stream): harden port 81 httpd socket hygiene

Matches port 80's pattern — adds close_fn, enables LRU purge on pool
full, tightens send_wait_timeout to 2s, and grows max_open_sockets
4→7. Together these fix the refresh-storm pool lockout: weak socket
cleanup let abandoned fds accumulate until accept() returned ENFILE.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: Dashboard `startStream()` exponential backoff

**Files:**
- Modify: `main/dashboard.html` (`startStream()` function, ~line 895)

**Context:** Currently `startStream()` is fire-and-forget — every call sets a fresh cache-busted URL on `<img>`. The watchdog can call it every 5s forever. If port 81 is briefly unreachable (WiFi hiccup, inference cycle), the restart storm accelerates the pool lockout. Add per-module backoff state; reset on `img.onload`; grow on `img.onerror`.

- [ ] **Step 1: Add module-level state.**

Find the existing stream-related `let` declarations (search for `_streamAlive` — should be near other WS/stream state, ~line 890). After that line, add:

```js
let _streamBackoffMs  = 0;    // grows on err, resets on load
let _streamLastAttempt = 0;
```

- [ ] **Step 2: Add the backoff guard at the top of `startStream()`.**

Find the function signature:
```js
function startStream() {
```

Insert as the FIRST lines of the function body (before the existing body):
```js
  const now = Date.now();
  if (now - _streamLastAttempt < _streamBackoffMs) return;
  _streamLastAttempt = now;
```

- [ ] **Step 3: Reset backoff on `img.onload`.**

Find the `img.onload` handler inside `startStream()`. It currently sets `_streamAlive = Date.now();`. Add one line AFTER that:
```js
  _streamBackoffMs = 0;
```

- [ ] **Step 4: Grow backoff on `img.onerror`.**

Find the `img.onerror` handler. Add ONE line at the top of that handler:
```js
  _streamBackoffMs = Math.min((_streamBackoffMs || 500) * 2, 15000);
```

Produces sequence: 1000, 2000, 4000, 8000, 15000 (capped). First failure sets 500 → next attempt delays 1000ms.

- [ ] **Step 5: Do NOT flash yet.**

Tasks 3 and 4 also modify `main/dashboard.html`. Batch the flash after Task 4.

- [ ] **Step 6: Commit.**

```bash
git add main/dashboard.html
git commit -m "$(cat <<'EOF'
fix(dashboard): exponential backoff on stream restart

Rapid-fire startStream() calls (every 5s from the watchdog) were the
client-side half of the port-81 pool lockout. Add 0.5→15s exponential
backoff that resets on successful img.onload.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: Dashboard watchdog WS-health gate

**Files:**
- Modify: `main/dashboard.html` (watchdog `setInterval`, ~line 999-1020)

**Context:** The 5s watchdog fires `startStream()` whenever `_streamAlive` is >10s old. But during a detect cycle, WiFi is disconnected for ~2-6s and the stream WILL be stale — restarting it just causes a port-81 accept during WiFi teardown (the exact storm that fd-leaks). Skip the restart if WS is unhealthy: WS being down is the proxy for "network is inherently down right now, don't try."

- [ ] **Step 1: Find the existing watchdog block.**

Run:
```bash
grep -n "Camera stream stale" /workspaces/BoatEspP4/main/dashboard.html
```

Expected: one match around line 1014 inside a `setInterval` body.

- [ ] **Step 2: Replace the existing stale-check with the gated version.**

Current code looks like (approximate):
```js
if (Date.now() - _streamAlive > 10000) {
  console.log('[Watch] Camera stream stale, restarting');
  startStream();
}
```

Replace with:
```js
const wsHealthy = ws && ws.readyState === WebSocket.OPEN
                  && (Date.now() - _lastMsgAt < 5000);
if (!wsHealthy) return;
if (Date.now() - _streamAlive > 10000) {
  console.log('[Watch] Camera stream stale (WS healthy), restarting');
  startStream();
}
```

Verify `_lastMsgAt` is already a declared variable — grep confirms it's set in the WS `onmessage` handler (search for `_lastMsgAt =`). If somehow missing, it's a latent bug; add `let _lastMsgAt = Date.now();` near the other state vars.

- [ ] **Step 3: Do NOT flash yet.** Batching with Task 4.

- [ ] **Step 4: Commit.**

```bash
git add main/dashboard.html
git commit -m "$(cat <<'EOF'
fix(dashboard): gate stream watchdog restart on WS health

Inference cycles drop WiFi for 2-6s. During that window the stream
is expected to be stale — restarting it just forces a port-81 accept
during teardown, feeding the socket-lockout storm. Skip restart if
WS isn't OPEN with recent traffic.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: Dashboard `beforeunload` cleanup + flash

**Files:**
- Modify: `main/dashboard.html` (near WS init, wherever event listeners are set up)

**Context:** On F5 / tab-close / navigation the browser abruptly RSTs TCP connections. With port 80's existing `close_fn` this is fine. With port 81's new `close_fn` from Task 1 it's also fine. But we can help: send an explicit WebSocket close frame and clear the `<img>` src so FINs beat RSTs. Cleaner logs and occasionally faster socket release on the firmware side.

- [ ] **Step 1: Find a spot to register the listener.**

Any module-scope location after the initial WS/stream setup works. Recommended: directly after the `startStream()` init call near line ~1523, or at the very end of the main script block. Grep:
```bash
grep -n "startStream();" /workspaces/BoatEspP4/main/dashboard.html
```

- [ ] **Step 2: Add the listener.**

Insert:
```js
window.addEventListener('beforeunload', () => {
  try {
    if (ws && ws.readyState === WebSocket.OPEN) ws.close(1000, 'page-unload');
  } catch (_) { /* swallow — browser is tearing down */ }
  const img = document.getElementById('cam-img');
  if (img) img.src = '';
});
```

`1000` is the WebSocket "normal closure" code. `cam-img` is the existing MJPEG `<img>` element (verify with `grep -n 'id="cam-img"' main/dashboard.html` — should match the HTML where the stream renders).

- [ ] **Step 3: Build.**

```bash
. $IDF_PATH/export.sh >/dev/null 2>&1 && idf.py build 2>&1 | tail -5
```

Expected: `Project build complete.`

- [ ] **Step 4: Flash all three dashboard changes (Task 2, 3, 4) together.**

Stop the monitor first. Then:
```bash
. $IDF_PATH/export.sh >/dev/null 2>&1 && idf.py -p /dev/ttyACM0 flash 2>&1 | tail -5
```

Restart the monitor.

- [ ] **Step 5: Commit.**

```bash
git add main/dashboard.html
git commit -m "$(cat <<'EOF'
fix(dashboard): clean WS + stream teardown on beforeunload

Browser F5/close sent RSTs; now we send a WS close frame + clear the
MJPEG img.src so firmware sees a graceful FIN and frees the fd via
close_fn without waiting for RST timeout.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: End-to-end verification + memory update

**Files:** None modified. Testing + docs only.

**Context:** Runs the four acceptance scenarios from the spec against the just-flashed firmware+dashboard. Updates the memory baseline pointer so future sessions know this is the new LKG.

- [ ] **Step 1: T1 — 60s idle.**

With dashboard open and monitor running, let the system sit idle for 60 seconds. Do not click Detect. Do not refresh.

Expected:
- Zero `[Watch] Camera stream stale` console messages in browser DevTools.
- Zero `[SOCK]` events in monitor.
- Zero `CAM_STREAM: socket N closed` events (nothing should be closing).

If `[Watch] stale` fires: check that WS-health gate from Task 3 is actually in place — grep `wsHealthy` in the flashed dashboard (hard-refresh first to drop browser cache).

- [ ] **Step 2: T2 — 10 hard-refreshes in 10 seconds.**

Hard-refresh the dashboard (`Ctrl+Shift+R`) 10 times within 10 seconds.

Expected in monitor:
- `CAM_STREAM: socket N closed` fires on each refresh.
- No `[ERR] accept errno=23` (ENFILE) — that's the pool-lockout signature.
- After the tenth refresh, stream resumes within 2 seconds.

If ENFILE fires: means 7 sockets + LRU wasn't enough. Likely the `lru_purge_enable` flag isn't being respected — re-check firmware build contains Task 1 changes.

- [ ] **Step 3: T3 — detect cycle watchdog quiet.**

Click Detect. Watch DevTools console for 5 seconds after the click.

Expected:
- WS drops briefly (you'll see `[WS] closed`).
- NO `[Watch] Camera stream stale (WS healthy), restarting` fires during the WS-down window.
- After WS reconnects, stream resumes and `_streamBackoffMs` stays 0 (inspect via DevTools console: `_streamBackoffMs`).

If the watchdog fires during the WS-down window: gate condition is wrong. Verify the `!wsHealthy` early return is actually returning (not just logging-then-continuing).

- [ ] **Step 4: T4 — slow-client recovery.**

In Chrome DevTools, open Network tab → set throttling to "Slow 3G" for 30 seconds. Then switch back to "Online".

Expected during throttling:
- Firmware logs `httpd_txrx: send : 11 EAGAIN` (or similar timeout)
- After 2 seconds of timeout, `CAM_STREAM: socket N closed` fires — send_wait_timeout worked.

Expected after throttle-off:
- Stream re-establishes within 10s (exponential backoff kicks in if multiple errors happened).

- [ ] **Step 5: Update `last_known_good_baseline.md`.**

File: `/root/.claude/projects/-workspaces-BoatEspP4/memory/last_known_good_baseline.md`

Get the current HEAD:
```bash
git log -1 --format='%h %s' HEAD
```

Update:
- Frontmatter `name:` line → new commit hash + "stream hardening" line.
- `description:` line → mention stream hardening on top of ToF+bbox fusion.
- Top of body: bump the baseline hash + short "what it adds" paragraph.
- Add to "winning recipe" section: "Port 81 httpd uses close_fn + lru_purge_enable + send_wait_timeout=2s + max_open_sockets=7, matching port 80's socket hygiene."
- Remove from "Known quirks" section: any line about stream instability / watchdog thrash / refresh-kills-stream.

- [ ] **Step 6: Update `MEMORY.md` index.**

File: `/root/.claude/projects/-workspaces-BoatEspP4/memory/MEMORY.md`

Change the one-liner for `last_known_good_baseline.md` to reference the new commit hash + summary including "stream hardening".

- [ ] **Step 7: No further commit.**

Memory files live outside the repo (the memory dir is under `/root/.claude/projects/`, not `/workspaces/BoatEspP4`). No `git add` needed.

---

## Self-Review Notes

1. **Spec coverage** — all 8 design decisions map to tasks:
   - #1 architecture direction → Task 1 (firmware stays MJPEG)
   - #2 pool size → Task 1 step 3
   - #3 send timeout → Task 1 step 3
   - #4 LRU purge → Task 1 step 3
   - #5 close_fn → Task 1 steps 2+3
   - #6 exp backoff → Task 2
   - #7 WS-health gate → Task 3
   - #8 beforeunload → Task 4

2. **Placeholder scan** — no "TBD" / "TODO" / "appropriate error handling" / vague references found.

3. **Type consistency** — `_streamBackoffMs` spelled consistently across Task 2 steps. `wsHealthy` condition uses the same `_lastMsgAt` name as the existing code.

4. **Task independence** — Task 1 (firmware) ships standalone. Tasks 2-4 (dashboard) land together but commit individually. Task 5 validates the bundle.

## Open Risk Carried From Spec

None — the spec captured all risks inline. The `<unistd.h>` include check in Task 1 Step 2 is the one build-time pitfall; if it's missed, the build fails loudly with `implicit declaration of close`.

## Out of Scope

Carried forward from the spec:
- Port-80 fd leak (separate investigation).
- `snap-btn` / `snap-overlay-btn` tainted-canvas fix (separate task).
- WebRTC + H.264 upgrade (not needed — dashboard is debug-only; production runs ESP-NOW).
