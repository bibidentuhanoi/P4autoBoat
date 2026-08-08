# Design: ESP-NOW Long-Range RC Control

**Status:** Draft for review · **Date:** 2026-08-08 · **Branch:** `feat/winch-servo-web-control`

## 1. Goal & scope

Drive the boat like an RC vehicle over the ESP-NOW field link, at long range,
with a control link that stays **reliable** and **fails safe** at the edge.

**In scope**
- **Range:** enable Espressif Long Range (LR) PHY on the field link → serves R4 (≥300 m).
- **Link/control reliability:** raise the command rate, surface link health to the
  operator so the edge is visible *before* the failsafe cuts, tune the failsafe margin.

**Out of scope (explicitly)**
- Heading-hold / gyro-assisted steering (fly-by-wire). The boat does exactly what the
  stick says — no self-driving.
- New input device (gamepad). Keep the existing `espnow_drive.py` web control surface.
- The WiFi/dashboard (`dashboard.html`) path — this is field-mode only.

## 2. Current state (what already works — do not rebuild)

- **Field-mode failsafe is sound.** `control_link_alive()` (motor_control.c) returns
  `ws_transport_client_count() > 0 || pipeline_recent_command(CONTROL_LINK_TIMEOUT_US)`.
  In ESP-NOW field mode (no WS client) it keys off **command arrival**
  (`pipeline.c: s_last_rx_us`, stamped on every decodable inbound message, portMUX-
  guarded so the 100 ms watchdog poll never blocks). Timeout = **400 ms**, poll = 100 ms.
- **Failsafe is RC-friendly.** On link loss `watchdog_cb` zeros throttle + winch,
  centers rudder, and cuts the servo rail — **but does NOT disarm the ESC**. On
  reconnect, commands take effect immediately (no re-arm dance). Transient dropout =
  brief coast, then seamless resume.
- **Control RX is decoupled from telemetry TX.** pipeline.c uses separate RX/TX
  envelopes+mutexes (the deadlock fix), so a stuck telemetry send can't block command
  decode/dispatch.
- **Stateless resend.** `espnow_drive.py` sends full throttle+rudder state every tick
  (currently 15 Hz), so a dropped packet is corrected by the next one — the right model
  for an unreliable broadcast link.
- **Channel set-then-readback precedent.** `espnow_transport_probe()` already sets the
  co-processor channel via `esp_wifi_set_channel()` (esp_wifi_remote RPC to the C6) and
  verifies with `esp_wifi_get_channel()`, hard-logging a mismatch. **This is the exact
  discipline the LR change reuses.**

## 3. Key findings & constraints (from research 2026-08-08)

Sources: esp-now issue #144, ESP32-C6 IDF WiFi/ESP-NOW docs, `RomanLut/hx_espnow_rc`.

- **C6 supports LR** (IDF C6 docs): ~1 km LOS, **~4 dB** better RX sensitivity than
  802.11b, **~2–2.5×** the 11b distance; LR raw PHY rate only 250/500 kbps (fine for
  control). IDF ≥5.1.2 fixed *"set LR rate fail for ESP-NOW"*; we're on 5.4.
- **⚠ esp-now #144 (CLOSED, unconfirmed):** a C6↔C6 LR test at 19 dBm showed **zero
  range gain** vs normal mode; Espressif said "enable `CONFIG_ESPNOW_ENABLE_LONG_RANGE`,
  use the espnow example", reporter never confirmed, closed as no-response. → **Range
  gain is UNPROVEN on our hardware; it must be measured, not assumed.**
- **The enable recipe is more than `WIFI_PROTOCOL_LR`:**
  1. `esp_wifi_set_protocol(ifx, 11B|11G|11N|LR)` — the **BGNLR** bitmap (per the
     official espnow example), not LR-only. Enables the capability.
  2. `esp_now_set_peer_rate_config(peer, {phymode = WIFI_PHY_MODE_LR, …})` **after**
     `esp_now_add_peer()` — forces the frames onto the LR PHY. Omitting this is the
     likely cause of #144's null result.
  3. `esp_wifi_set_ps(WIFI_PS_NONE)` — default is modem-sleep = latency/drops on a
     realtime link.
- **Architectural consequence:** `esp_now_set_peer_rate_config()` is an `esp_now_*`
  call → it runs **where ESP-NOW lives**, which on the boat is the **C6 custom
  firmware**, not the P4. So the boat-side change needs a **C6 firmware edit + reflash**
  (and likewise the S3). `esp_wifi_set_protocol` alone *can* be proxied from the P4, but
  the rate config cannot. (Earlier "P4-only, no C6 reflash" assumption was wrong.)
- **Both ends must match.** LR-rate frames are only heard by LR receivers — one-sided =
  they go deaf to each other. Flash both together; gate behind a Kconfig toggle.
- **`hx_espnow_rc` reference:** mature ESP-NOW RC link, **50 packets/sec** in LR +
  normal, bidirectional telemetry + RSSI. Validates a higher send rate and RSSI link-
  quality. Caveat: classic ESP32/ESP8266, **not C6** — its LR success doesn't transfer.

## 4. Design

Three workstreams. **A ships and is range-tested in isolation before B/C** so a range
change can't be confused with a control change.

### A. Range — Long Range PHY

Apply the same recipe on **both radios**, gated behind a build toggle so the range test
is a clean A/B:

**C6 boat co-processor** (`tools/slave_firmware/espnow_bridge.c`, in `init_cb` — runs
in field mode when the P4 sends `MSG_ESPNOW_INIT`):
```c
// after esp_wifi_start() (already done by the C6 boot path) and esp_wifi_set_channel():
esp_wifi_set_protocol(WIFI_IF_STA,
    WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR); // check
esp_wifi_set_ps(WIFI_PS_NONE);
// after esp_now_init() + esp_now_add_peer(peer):
esp_now_rate_config_t rc = { .phymode = WIFI_PHY_MODE_LR,
                             .rate    = WIFI_PHY_RATE_LORA_500K, // 250K for max range
                             .ersu = false, .dcm = false };
esp_now_set_peer_rate_config(peer_addr, &rc);   // MUST return ESP_OK
```
**S3 ground station** (`tools/espnow_bridge/main/main.c`): identical, after
`esp_wifi_start()` and after the broadcast peer is added.

**P4 verification** (`main/transports/espnow_transport.c`, in `espnow_transport_probe()`):
read back `esp_wifi_get_protocol(WIFI_IF_STA, &proto)` via RPC and hard-log if
`!(proto & WIFI_PROTOCOL_LR)` — reusing the channel-readback discipline. The C6 owns the
config (it must, for the rate config); the P4 verifies it took. **Never trust a bare
ESP_OK** (this codebase's history is silent RPC failures).

**Sequencing:** the boat must have LR active *before* it listens for the S3
`MSG_GROUND_HELLO` beacon (an LR-rate frame is undemodulable by a non-LR receiver).
`init_cb` sets LR before `espnow_transport_probe()` listens — ordering already holds. The
S3 must beacon at LR rate → its **broadcast peer** (`FF:FF:FF:FF:FF:FF`) needs the rate
config too.

### B. Reliability & visibility

- **Command rate → ~30 Hz.** `espnow_drive.py SEND_HZ = 30` (was 15). LR's 250–500 kbps
  easily carries 30 tiny packets/sec; gives ~12-packet margin before the 400 ms failsafe
  (was ~6) and tighter feel.
- **Boat telemetry carries link health** (`espnow_telemetry_t` in
  `espnow_protocol.h` + `espnow_transport.c`):
  - `cmd_age_ms` — ms since last command received (from `s_last_rx_us`). *Downlink*
    health, the direction that matters for control. **P4-only, free.**
  - `cmd_rx_rssi` — RSSI of the last command packet, captured in the **C6** esp_now recv
    callback and forwarded upstream with the command. *Downlink signal margin* = the
    early-warning before `cmd_age_ms` starts climbing.
- **Ground shows uplink health too:** the **S3** esp_now recv callback already sees the
  RSSI of the boat's telemetry; forward it over USB. `espnow_drive.py` renders a
  green/amber/red **link meter** from {cmd_rx_rssi, cmd_age_ms, telemetry_rssi,
  telemetry_rate}.

### C. Tuning knobs (no behavior rebuild)

- Make `SEND_HZ` (sender) and `CONTROL_LINK_TIMEOUT_US` (motor_control.c, 400 ms)
  explicit documented knobs; field-tune the timeout against LR's burstier edge loss so
  genuine loss still cuts promptly without nuisance cutouts.
- Existing failsafe behavior (zero + center + cut rail, stay armed, auto-resume) is
  correct for RC — **keep as-is**.

## 5. Files touched

| File | Change |
|---|---|
| `tools/slave_firmware/espnow_bridge.c` (C6) | BGNLR bitmap + `PS_NONE` + peer LR rate config; forward cmd RX RSSI upstream. **Reflash.** |
| `tools/espnow_bridge/main/main.c` (S3) | Same LR recipe; forward telemetry RSSI over USB. **Reflash.** |
| `main/transports/espnow_transport.c` (P4) | LR readback-verify via RPC; populate `cmd_age_ms` + `cmd_rx_rssi` in telemetry. |
| `main/transports/espnow_protocol.h` | Extend `espnow_telemetry_t` with `cmd_age_ms`, `cmd_rx_rssi`. |
| `tools/espnow_drive.py` | `SEND_HZ=30`; decode new telemetry; link-health meter UI. |
| `Kconfig.projbuild` / firmware build flags | `CONFIG_ESPNOW_ENABLE_LONG_RANGE` toggle on each of the three firmwares. |

## 6. Verification plan

**Bench (LR-on build, close range) — before any field test:**
1. P4 log shows `esp_wifi_get_protocol` readback includes LR on the C6; S3 log shows
   `esp_now_set_peer_rate_config` returned ESP_OK.
2. Link still works at 1 m (didn't break normal operation); telemetry + control flow.
3. Failsafe still fires: power off the S3 → boat cuts throttle / centers rudder within
   ~400 ms; power S3 back on → control auto-resumes without re-arm.
4. WiFi/dashboard mode unaffected (LR is scoped to the field-mode ESP-NOW path).

**Range test (isolated A/B — the point of the Kconfig toggle):**
- LR-**off** build both ends → walk the boat away, log `cmd_age_ms`, `cmd_rx_rssi`,
  packet rate vs distance; record the distance where control becomes unreliable
  (`cmd_age_ms` regularly > ~200 ms / failsafe starts firing). = baseline.
- LR-**on** build both ends → repeat. **Expect ≈2× baseline** if LR is engaging.
- If no improvement (the #144 outcome): rate config didn't take / PS still on / TX power
  → check ESP_OK on rate config, confirm `PS_NONE`, try `esp_wifi_set_max_tx_power`.

## 7. Risks & mitigations

1. **LR yields no gain (à la #144).** → readback the protocol; assert rate-config ESP_OK;
   `PS_NONE`; consider `esp_wifi_set_max_tx_power`; **measure A/B**. If still nothing,
   the range goal isn't met via LR — surface honestly, don't paper over.
2. **Silent RPC no-op on `esp_wifi_set_protocol`.** → set on C6 + read back on P4; never
   trust bare ESP_OK.
3. **Mismatched ends → mutual deafness.** → Kconfig toggle, flash both together, document.
4. **Broadcast peer rate config rejected/unsupported.** → verify on bench; fallback to
   bitmap-only + measure.
5. **LR clobbers dashboard/WiFi mode.** → LR is set only in the field-mode ESP-NOW path
   (C6 `init_cb`), STA interface; dashboard SoftAP path untouched. Verify (test 6.4).
6. **C6 reflash logistics.** The C6 firmware is a separate build (esp-hosted slave +
   custom `espnow_bridge`). Document the build/flash procedure; both C6 and S3 must be
   reflashed for this feature.

## 8. Sequencing

1. **A + Kconfig toggle** → bench tests 6.1–6.4 → **isolated range A/B**. Prove (or
   disprove) the range gain first, in isolation.
2. **B** (30 Hz + link-health telemetry/UI) → re-run edge-of-range behavior; confirm the
   meter tracks reality and the failsafe/auto-resume feels right at distance.
3. **C** tuning as the field data dictates.

## 9. Open items to verify during implementation

- Exact IDF 5.4 identifiers: `esp_now_rate_config_t` field names, `WIFI_PHY_MODE_LR`,
  `WIFI_PHY_RATE_LORA_250K` / `_500K` — confirm against the C6 `esp_now.h`.
- esp_now recv-callback RSSI access on the C6/S3 IDF version
  (`esp_now_recv_info_t.rx_ctrl->rssi`).
- Whether the BGNLR bitmap alone already forces LR for our broadcast traffic, making the
  per-peer rate config redundant — decide empirically (belt-and-suspenders: set both).
- `LORA_500K` vs `LORA_250K`: 250K = more sensitivity/range, 500K = more headroom. Pick
  from the range-test data.
