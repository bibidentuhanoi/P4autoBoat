# MJPEG Stream Hardening — Design Spec

**Date:** 2026-04-13
**Scope:** `main/camera_stream.c` + `main/dashboard.html` only. No protobuf, no schema, no architectural rework.
**Status:** Approved section-by-section; proceeding to implementation plan.

## Goal

Stop the dashboard's MJPEG stream from thrashing, leaking sockets, and permanently wedging under two everyday conditions:

1. **Browser refresh** (F5) sometimes kills the stream AND the whole system, requiring either a wait (~30s-2min) or a reboot.
2. **Idle operation** regularly logs `[Watch] Camera stream stale, restarting` even when the stream is fine — watchdog thrashing.

Keep the architecture ESP-IDF-idiomatic: the `multipart/x-mixed-replace` MJPEG server on port 81 is exactly what Espressif's reference video server uses. Our instability comes from the stack we added on top (inference cycles tearing down WiFi, a dashboard watchdog with no backoff, and weak socket hygiene on port 81) — not from MJPEG itself.

## Non-Goals

- No WebRTC, no H.264, no esp-webrtc-solution. Those are the "real pro" answer but the dashboard is a local debug tool; production communicates over ESP-NOW.
- No protobuf frames over WS. Abandoning MJPEG isn't worth it for a debug UI.
- No rewrite of the per-frame loop, `g_inference_active` handling, 200ms throttle, or drain task — those work.
- No port-80 changes. The fd leak on port 80 after long reload marathons is a separate investigation.
- No `snap-btn` / `snap-overlay-btn` fix (tainted canvas) — separate task.

## Design Decisions

| # | Question | Decision |
|---|---|---|
| 1 | Architecture direction | Fix MJPEG in place. No WS-JPEG, no WebRTC. Matches ESP-IDF reference. |
| 2 | Port 81 socket pool size | `max_open_sockets` **4 → 7** to match port 80 and survive refresh races. |
| 3 | Port 81 send timeout | `send_wait_timeout=2s` (was default 5s). 2s is generous for 5 FPS × 40 KB on healthy WiFi. |
| 4 | Full-pool behavior | `lru_purge_enable=true` — evict oldest socket on accept, instead of refusing new. Recommended ESP-IDF pattern for streaming. |
| 5 | Surprise-close handling | New `close_fn = camera_stream_on_close` callback — logs + closes the fd when browser FINs/RSTs. Mirrors port 80's `ws_transport_close_fd` pattern. |
| 6 | Watchdog restart cadence | Exponential backoff: 0.5s → 1s → 2s → 4s → 8s → **15s cap**. Reset on `img.onload`. |
| 7 | Watchdog WS gating | Only restart stream if WS is `OPEN` AND `lastMsgAt < 5s ago`. During detect cycles WS is briefly down; restarts in that window caused port-81 accept during WiFi teardown — the exact storm that locks out the pool. |
| 8 | Browser unload cleanup | `beforeunload` listener → `ws.close(1000)` + clear `img.src`. Firmware's new `close_fn` handles the FIN. |

## Firmware Changes

**File:** `main/camera_stream.c`
**Function:** `camera_stream_server_start()` only.

httpd_config additions (to the existing block, ~line 115-120):

```c
cfg.max_open_sockets  = 7;                      // was 4
cfg.send_wait_timeout = 2;                      // was default 5
cfg.lru_purge_enable  = true;                   // new
cfg.close_fn          = camera_stream_on_close; // new
```

New function (same file, near the top):

```c
static void camera_stream_on_close(httpd_handle_t hd, int sockfd) {
    (void)hd;
    ESP_LOGI(TAG, "socket %d closed", sockfd);
    close(sockfd);
}
```

**Not changed:** per-frame loop, `g_inference_active` exit path, 200ms vTaskDelay throttle, `camera_drain_task`, or any MJPEG protocol detail.

## Dashboard Changes

**File:** `main/dashboard.html`

### 3a. Exponential backoff in `startStream()`

Module-level state:
```js
let _streamBackoffMs  = 0;    // 0 on success, grows on failure
let _streamLastAttempt = 0;
```

Entry guard inside `startStream()`:
```js
const now = Date.now();
if (now - _streamLastAttempt < _streamBackoffMs) return;
_streamLastAttempt = now;
```

In `img.onload`: `_streamBackoffMs = 0;` (reset backoff).
In `img.onerror`: `_streamBackoffMs = Math.min((_streamBackoffMs || 500) * 2, 15000);`

### 3b. WS-health gate in the watchdog

Replace the stale-check block (~line 1014):
```js
const wsHealthy = ws && ws.readyState === WebSocket.OPEN
                  && (Date.now() - _lastMsgAt < 5000);
if (!wsHealthy) return;
if (Date.now() - _streamAlive > 10000) {
  console.log('[Watch] Camera stream stale (WS healthy), restarting');
  startStream();
}
```

### 3c. `beforeunload` listener

Added near the existing init block:
```js
window.addEventListener('beforeunload', () => {
  try { if (ws && ws.readyState === WebSocket.OPEN) ws.close(1000, 'page-unload'); } catch {}
  const img = document.getElementById('cam-img');
  if (img) img.src = '';
});
```

**Not changed:** WS reconnect code, detect modal code, layout, any rendering logic.

## Acceptance Criteria

| # | Scenario | Pass criterion |
|---|---|---|
| T1 | 60s idle | Zero `[Watch] restarting` logs. Zero `[SOCK]` events. |
| T2 | 10 hard-refreshes in 10s | No `[ERR] accept errno=23` (ENFILE). Stream resumes within 2s of last load. `close_fn` logs one event per refresh. |
| T3 | Detect cycle | WS drops briefly; watchdog skips restart (WS unhealthy gate). After WS recovers + stream `onload`, `_streamBackoffMs` stays 0. |
| T4 | Slow client (DevTools throttle to Slow 3G for 30s, then Online) | Firmware `send_wait_timeout` fires at 2s → handler exits → fd released (visible in `close_fn` log). On throttle-off, dashboard exp-backoff re-establishes within 10s. |

**Rollback:** `git revert` the single implementation commit. No schema / protobuf / cross-layer compat concerns.

## Memory Update (after ship)

Append to `/root/.claude/projects/-workspaces-BoatEspP4/memory/last_known_good_baseline.md`:
- New commit hash.
- Add to "winning recipe": "Port 81 httpd uses `close_fn` + `lru_purge_enable` + `send_wait_timeout=2s` + `max_open_sockets=7`, matching port 80's socket hygiene."
- Remove from "known quirks" list: the stream-instability line.

## Out of Scope (later, separate specs)

- Port-80 fd leak (`ENFILE` after ~15min reload marathon).
- `snap-btn` / `snap-overlay-btn` tainted canvas — fix via `/snapshot` fetch pattern.
- WebRTC + hardware H.264 — if dashboard becomes production-facing (unlikely per user's note that production runs ESP-NOW).
