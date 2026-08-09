# ESP-NOW Long Range + RC Control Reliability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the ESP-NOW field-mode link to Espressif Long Range (LR) PHY for ≥300m range (R4), and give the operator live link-health visibility (command age + RSSI both directions) so the control link stays trustworthy and fails safe at the edge — no heading-hold/autonomy, purely link/control reliability.

**Architecture:** Three independent build targets share one wire protocol. LR is enabled where ESP-NOW itself lives — the ESP32-C6 co-processor firmware (`tools/slave_firmware/espnow_bridge.c`) and the ESP32-S3 ground-station bridge (`tools/espnow_bridge/main/main.c`) — with the P4 main firmware only verifying it took (LR set/rate-config calls are C6-local; only `esp_wifi_get_protocol` is RPC-proxied to the P4). Link-health telemetry piggybacks on two channels that already exist and already run at the right cadence — the compact `espnow_telemetry_t` struct (P4→ground, IMU+GPS today) and `MSG_BRIDGE_STATUS` (S3→laptop, USB-only) — instead of adding new RPC traffic to the hot 30Hz control path.

**Tech Stack:** ESP-IDF v5.4 (C, FreeRTOS), esp_wifi_remote (C6 RPC proxy), Python 3 (espnow_drive.py / visualize.py, stdlib only).

**Design reference:** `docs/design/espnow-lr-rc-control.md` (this plan implements it; a few details were corrected during research — see task notes marked "diverges from the design doc").

**Starting state:** the pre-existing uncommitted pile (deadlock fix, I2C-starvation fix, field telemetry + control tooling) has already been committed as three separate commits (`97b2aa7`, `dc8feb8`, `c1ccbd8`) on `feat/winch-servo-web-control`. This plan's commits land on top of that clean base, on the same branch.

---

## File Structure

| File | Responsibility |
|---|---|
| `tools/slave_firmware/espnow_bridge.c` | C6 co-processor: enable LR (bitmap + peer rate config + PS_NONE), capture+forward cmd RSSI |
| `tools/espnow_bridge/main/main.c` | S3 ground station: enable LR (bitmap + peer rate config), capture+report uplink RSSI |
| `main/transports/espnow_protocol.h` | Shared wire structs: extend `espnow_telemetry_t` (+cmd_age_ms, +cmd_rx_rssi) and `espnow_bridge_status_t` (+last_rx_rssi) |
| `main/transports/espnow_transport.c/.h` | P4: LR readback-verify; strip/expose the C6-forwarded cmd RSSI byte |
| `main/pipeline.c/.h` | P4: `pipeline_last_command_age_ms()` — value form of the existing liveness signal |
| `main/sensor_task.c` | P4: populate the two new telemetry fields each field-mode tick |
| `tools/espnow_drive.py` | Ground control tool: decode extension, 30Hz send rate, `link_quality()` + UI meter |
| `visualize.py` | Field-mode viewer: matching decode extension (coordinated wire change, not independently testable) |
| `tools/test_espnow_drive.py` (new) | Unit test for the one pure, testable piece of new logic (`link_quality()`) |

---

## Part A — Long Range (LR) mode

**Sequencing discipline (from the design doc):** ship and bench-verify LR in isolation, run the field range A/B test, *before* Part B lands — so a range result can never be confused with a control-path change. Tasks 1–3 are code; Tasks 4–6 are build/bench/field verification.

### Task 1: C6 — enable LR (protocol bitmap + peer rate config + PS_NONE)

**Files:**
- Modify: `tools/slave_firmware/espnow_bridge.c`

- [ ] **Step 1: Add the LR toggle near the top of the file**

This build is regenerated from scratch by `build.sh` each run (`idf.py create-project-from-example` into `build_slave/`) — there's no persistent `sdkconfig` to hang a Kconfig menu entry off without inventing new Kconfig plumbing in a script that already does a lot of fragile `sed` patching. A `#define` is proportionate for a dev-only A/B toggle.

Find (in the constants block, after `#define PEER_MSG_UPSTREAM 4u`):
```c
#define PEER_MSG_VIDEO    1u
#define PEER_MSG_COMMAND  2u
#define PEER_MSG_INIT     3u
#define PEER_MSG_UPSTREAM 4u
```
Replace with:
```c
#define PEER_MSG_VIDEO    1u
#define PEER_MSG_COMMAND  2u
#define PEER_MSG_INIT     3u
#define PEER_MSG_UPSTREAM 4u

/* Long Range PHY toggle for the A/B range test -- flip to 0, rebuild via
 * build.sh, and reflash to get a same-hardware LR-off baseline. Both this
 * file and tools/espnow_bridge/main/main.c (ground station) must be flashed
 * with the SAME value, or LR-rate frames from one end simply go unheard by
 * the other (LR is only demodulable by an LR-enabled receiver). Not a
 * Kconfig entry: this build is regenerated from scratch each run (see
 * build.sh), so there's no persistent sdkconfig to hang a menu option off. */
#define ESPNOW_LR_ENABLED 1
```

- [ ] **Step 2: Enable the LR protocol bitmap + disable power-save in `init_cb`**

Find:
```c
    /* WiFi must be started before esp_now_init(); caller is responsible for
     * calling esp_wifi_start().  Set the channel here. */
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "init_cb: esp_wifi_set_channel failed: %s",
                 esp_err_to_name(err));
        /* Non-fatal — proceed anyway; the channel the AP set may be fine */
    }

    err = esp_now_init();
```
Replace with:
```c
    /* WiFi must be started before esp_now_init(); caller is responsible for
     * calling esp_wifi_start().  Set the channel here. */
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "init_cb: esp_wifi_set_channel failed: %s",
                 esp_err_to_name(err));
        /* Non-fatal — proceed anyway; the channel the AP set may be fine */
    }

#if ESPNOW_LR_ENABLED
    /* Long Range PHY: ~4dB better RX sensitivity, ~2-2.5x the 802.11b
     * distance (Espressif C6 wifi-driver docs). BGNLR, not LR-only -- matches
     * Espressif's own espnow example and keeps the STA able to fall back to
     * normal rates. esp-now issue #144 showed a bare WIFI_PROTOCOL_LR
     * producing ZERO range gain on two C6s -- the actual per-frame rate is
     * forced separately below via esp_now_set_peer_rate_config(), which
     * #144's reporter never did; that is almost certainly why they saw
     * nothing. This codebase's whole history is silent RPC/config failures
     * -- log every outcome here, never trust a bare success. */
    err = esp_wifi_set_protocol(WIFI_IF_STA,
            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: esp_wifi_set_protocol(LR) failed: %s",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "init_cb: LR protocol bitmap set");
    }

    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "init_cb: esp_wifi_set_ps(NONE) failed: %s", esp_err_to_name(err));
    }
#endif /* ESPNOW_LR_ENABLED */

    err = esp_now_init();
```

- [ ] **Step 3: Force the peer onto the LR rate after `esp_now_add_peer()`**

Find:
```c
    err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: esp_now_add_peer failed: %s",
                 esp_err_to_name(err));
        esp_now_deinit();
        return;
    }

    s_espnow_ready = true;
    ESP_LOGI(TAG, "ESP-NOW bridge ready");
```
Replace with:
```c
    err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: esp_now_add_peer failed: %s",
                 esp_err_to_name(err));
        esp_now_deinit();
        return;
    }

#if ESPNOW_LR_ENABLED
    /* This is what ACTUALLY selects the LR PHY for frames to this peer --
     * the protocol bitmap above only makes LR available, it doesn't select
     * it. esp_now_rate_config_t is a typedef of wifi_tx_rate_config_t; must
     * be called after esp_now_add_peer() per the IDF ESP-NOW docs. */
    esp_now_rate_config_t rate_cfg = {
        .phymode = WIFI_PHY_MODE_LR,
        .rate    = WIFI_PHY_RATE_LORA_250K,
        .ersu    = false,
        .dcm     = false,
    };
    err = esp_now_set_peer_rate_config(s_peer_mac, &rate_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: esp_now_set_peer_rate_config(LR) failed: %s",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "init_cb: peer rate config set to LR/250K");
    }
#endif /* ESPNOW_LR_ENABLED */

    s_espnow_ready = true;
    ESP_LOGI(TAG, "ESP-NOW bridge ready");
```

- [ ] **Step 4: Build and verify the bridge still links (symbol check)**

Run: `cd tools/slave_firmware && ./build.sh`
Expected: ends with `=== Bridge link verified: ESP-NOW symbols present in firmware ===` and `=== Build complete ===`. The script's own `nm`-based check already covers `esp_now_add_peer`/`esp_now_init`/`esp_now_send`/`espnow_bridge_init` — this step just needs to *not* hit the `ERROR: symbol ... missing` path, confirming the new code compiled and linked cleanly with the rest of the file.

- [ ] **Step 5: Commit**

```bash
git add tools/slave_firmware/espnow_bridge.c
git commit -m "$(cat <<'EOF'
feat(espnow): enable Long Range PHY on the C6 co-processor

BGNLR protocol bitmap + per-peer LR rate config (the part esp-now issue
#144 appears to have missed) + PS_NONE, behind a #define so it's a clean
A/B against the ground station's matching change.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: S3 — enable LR (protocol bitmap + peer rate config)

**Files:**
- Modify: `tools/espnow_bridge/main/main.c`

- [ ] **Step 1: Add the matching LR toggle near the top of the file**

Find:
```c
/* ---- Constants ---------------------------------------------------------- */
#define ESPNOW_CHANNEL      6
```
Replace with:
```c
/* ---- Constants ---------------------------------------------------------- */
/* Long Range PHY toggle for the A/B range test -- see the matching define
 * in tools/slave_firmware/espnow_bridge.c (boat side) for the full
 * rationale. Both ends must be flashed with the SAME value. */
#define ESPNOW_LR_ENABLED 1

#define ESPNOW_CHANNEL      6
```

- [ ] **Step 2: Enable the LR protocol bitmap in `wifi_init()`**

Find:
```c
    /* Fix the channel — must match CONFIG_ESPNOW_CHANNEL on P4 side */
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    /* Disable power-save for lowest latency */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
```
Replace with:
```c
    /* Fix the channel — must match CONFIG_ESPNOW_CHANNEL on P4 side */
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if ESPNOW_LR_ENABLED
    /* Must match the C6's bitmap exactly (tools/slave_firmware/espnow_bridge.c) --
     * see that file for the esp-now issue #144 caveat and why BGNLR, not
     * LR-only. ESP_ERROR_CHECK is correct here (matches this function's own
     * style): this call is local to this chip, not RPC-proxied, so a real
     * esp_err_t is trustworthy. */
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif

    /* Disable power-save for lowest latency */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
```

- [ ] **Step 3: Force the broadcast peer onto the LR rate in `espnow_init()`**

Find:
```c
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Add broadcast peer so we can send to it */
    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}
```
Replace with:
```c
static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Add broadcast peer so we can send to it */
    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

#if ESPNOW_LR_ENABLED
    esp_now_rate_config_t rate_cfg = {
        .phymode = WIFI_PHY_MODE_LR,
        .rate    = WIFI_PHY_RATE_LORA_250K,
        .ersu    = false,
        .dcm     = false,
    };
    /* Not ESP_ERROR_CHECK, unlike every other call in this function: whether
     * a BROADCAST peer even accepts a per-peer rate config is genuinely
     * unverified (design doc open item). Log and continue either way -- the
     * range test proves which outcome we got; aborting the whole bridge over
     * a still-open question would be the wrong failure mode. */
    esp_err_t rc_err = esp_now_set_peer_rate_config(BROADCAST_MAC, &rate_cfg);
    if (rc_err != ESP_OK) {
        ESP_LOGW(TAG, "espnow_init: esp_now_set_peer_rate_config(broadcast, LR) "
                      "failed: %s -- LR bitmap is still set; broadcast frames "
                      "may fall back to normal rate", esp_err_to_name(rc_err));
    } else {
        ESP_LOGI(TAG, "espnow_init: broadcast peer rate config set to LR/250K");
    }
#endif
}
```

- [ ] **Step 4: Build**

Run: `cd tools/espnow_bridge && idf.py build`
Expected: `Project build complete.` with no errors. (This is a normal standalone IDF project, unlike the C6's generated build — no symbol-check script exists here, so a clean build is the available signal; Task 5 covers the runtime log check that actually proves the calls succeeded.)

- [ ] **Step 5: Commit**

```bash
git add tools/espnow_bridge/main/main.c
git commit -m "$(cat <<'EOF'
feat(espnow): enable Long Range PHY on the S3 ground station

Matching BGNLR bitmap + broadcast-peer LR rate config for the boat-side
change -- both ends must run the same ESPNOW_LR_ENABLED value or they
stop hearing each other.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: P4 — verify LR took (readback, log-only)

**Files:**
- Modify: `main/transports/espnow_transport.c`

- [ ] **Step 1: Add the LR readback check to `espnow_transport_probe()`**

The LR-enabling calls (`esp_wifi_set_protocol`, `esp_now_set_peer_rate_config`) run **on the C6**, not here — the P4 can only verify, via `esp_wifi_get_protocol()`, which genuinely is RPC-proxied through `esp_wifi_remote` (same mechanism as the existing `esp_wifi_get_channel()` call right above this insertion point).

Find:
```c
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
        if (primary != CONFIG_ESPNOW_CHANNEL) {
            ESP_LOGE(TAG, "CHANNEL MISMATCH: co-processor is on %d, ESP-NOW needs %d "
                          "— the ground station will not be heard",
                     primary, CONFIG_ESPNOW_CHANNEL);
        } else {
            ESP_LOGI(TAG, "Co-processor confirmed on channel %d", primary);
        }
    } else {
        ESP_LOGW(TAG, "Could not read back the co-processor channel");
    }

    /* --- Listen for the ground station -------------------------------------
```
Replace with:
```c
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
        if (primary != CONFIG_ESPNOW_CHANNEL) {
            ESP_LOGE(TAG, "CHANNEL MISMATCH: co-processor is on %d, ESP-NOW needs %d "
                          "— the ground station will not be heard",
                     primary, CONFIG_ESPNOW_CHANNEL);
        } else {
            ESP_LOGI(TAG, "Co-processor confirmed on channel %d", primary);
        }
    } else {
        ESP_LOGW(TAG, "Could not read back the co-processor channel");
    }

    /* --- Verify Long Range mode took, if the C6 build has it enabled --------
     * esp_wifi_set_protocol()/esp_now_set_peer_rate_config() run ON THE C6
     * (tools/slave_firmware/espnow_bridge.c's init_cb), not here -- the P4
     * can't set them, only verify. esp_wifi_get_protocol() IS proxied
     * through esp_wifi_remote like the channel check above, so it genuinely
     * reflects the co-processor's state. Log-only: a mismatch doesn't fail
     * the probe (the link still works at normal range), it just means the
     * range test won't show a gain -- confirm that here, at boot, rather
     * than discovering it 300m into a field test. */
    uint8_t protocol_bitmap = 0;
    if (esp_wifi_get_protocol(WIFI_IF_STA, &protocol_bitmap) == ESP_OK) {
        if (protocol_bitmap & WIFI_PROTOCOL_LR) {
            ESP_LOGI(TAG, "Co-processor LR protocol bit is SET (bitmap=0x%02x)",
                     protocol_bitmap);
        } else {
            ESP_LOGW(TAG, "Co-processor LR protocol bit NOT set (bitmap=0x%02x) "
                          "-- ESPNOW_LR_ENABLED may be 0 on the C6 build, or "
                          "esp_wifi_set_protocol failed there (check its own log)",
                     protocol_bitmap);
        }
    } else {
        ESP_LOGW(TAG, "Could not read back the co-processor's WiFi protocol bitmap");
    }

    /* --- Listen for the ground station -------------------------------------
```

- [ ] **Step 2: Build**

Run: `idf.py build`
Expected: `Project build complete.` No new warnings from this file.

- [ ] **Step 3: Commit**

```bash
git add main/transports/espnow_transport.c
git commit -m "$(cat <<'EOF'
feat(espnow): verify LR protocol took on the co-processor at probe time

esp_wifi_get_protocol() is RPC-proxied like the existing channel readback
-- log a clear warning if the LR bit isn't set instead of finding out from
an unchanged field range.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3b: P4 — protect the WiFi-fallback path from LR's shared radio state

**Discovered during Task 1 review, not in the original design doc.** `espnow_transport_probe()` calls `send_espnow_init()` — which triggers the C6's `init_cb()`, and thus Task 1's LR-enable code — **unconditionally, at the start of every boot's probe attempt**, before it's known whether a ground station will answer (`main.c:225-257`). If no ground station is heard, the boat falls through to `wifi_connect()` (`main.c:256`, plain `WIFI_MODE_STA` — confirmed via `wifi_manager.c`, a real station joining a router, not a SoftAP). So on *every* boot with `CONFIG_ESPNOW_ENABLED=y` (the default), the C6's `WIFI_IF_STA` protocol bitmap gets set to BGNLR before plain WiFi is ever attempted — even on boots that never use ESP-NOW at all.

Espressif's own docs say BGNLR (not LR-only) negotiates down cleanly with a normal router, so this is *expected* to be harmless — but that's a vendor compatibility claim, unverified on this specific hardware, in a codebase with a repeated history of "should be fine per docs" not holding here. Rather than trust it, remove the dependency entirely: explicitly restore the STA protocol to its normal default right before the WiFi-fallback path ever uses the interface.

(The other two things Task 1 touches turn out NOT to need protecting: `esp_wifi_set_ps(WIFI_PS_NONE)` is already called unconditionally by `wifi_manager.c:111` on every boot regardless of ESP-NOW, so Task 1 doesn't change that path's behavior at all; and `esp_now_set_peer_rate_config()` targets a specific ESP-NOW peer MAC, not the AP association, so it has no bearing on infrastructure WiFi either. Only the protocol bitmap is shared, relevant state.)

**Files:**
- Modify: `main/wifi_manager.c`

- [ ] **Step 1: Reset the STA protocol bitmap at the top of `wifi_connect()`**

Find:
```c
esp_err_t wifi_connect(void)
{
    if (!s_wifi_events) {
        ESP_LOGE(TAG, "wifi_connect() before wifi_start_radio()");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Connecting to SSID: %s", CONFIG_WIFI_SSID);
```
Replace with:
```c
esp_err_t wifi_connect(void)
{
    if (!s_wifi_events) {
        ESP_LOGE(TAG, "wifi_connect() before wifi_start_radio()");
        return ESP_ERR_INVALID_STATE;
    }

    /* Undo any LR protocol bitmap the ESP-NOW probe may have set on the C6
     * (tools/slave_firmware/espnow_bridge.c's init_cb runs unconditionally
     * at the START of every probe attempt, before we know whether a ground
     * station will answer -- see main.c's boot sequence). Espressif
     * documents BGNLR as negotiating down cleanly with a normal router, but
     * this path has nothing to do with ESP-NOW/LR at all -- don't rely on
     * that claim holding on this exact hardware. Restore the chip's own
     * documented default (BGNAX -- the C6 supports 802.11ax) before this
     * interface is ever used for a real AP connection. */
    esp_err_t proto_err = esp_wifi_set_protocol(WIFI_IF_STA,
            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX);
    if (proto_err != ESP_OK) {
        ESP_LOGW(TAG, "wifi_connect: esp_wifi_set_protocol(reset) failed: %s",
                 esp_err_to_name(proto_err));
    }

    ESP_LOGI(TAG, "Connecting to SSID: %s", CONFIG_WIFI_SSID);
```

- [ ] **Step 2: Build**

Run: `idf.py build`
Expected: `Project build complete.`

- [ ] **Step 3: Commit**

```bash
git add main/wifi_manager.c
git commit -m "$(cat <<'EOF'
fix(wifi): reset STA protocol bitmap before plain WiFi connect

espnow_transport_probe() sets the C6's STA protocol to BGNLR
unconditionally at probe start, before it's known whether ESP-NOW field
mode will actually be used -- so a boot that falls back to plain WiFi was
relying on Espressif's documented (but here-unverified) BGNLR/normal-AP
compatibility. Reset to the chip's real default explicitly instead of
depending on that claim.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3c: C6→P4 — report LR rate-config status (one-shot)

**Discovered during Task 1 code-quality review.** The reviewer flagged that `esp_now_set_peer_rate_config()`'s result (Task 1, the call that actually forces frames onto the LR PHY — the bitmap alone doesn't) is only ever logged to the C6's own UART console, which nobody watches mid-field-test. That's "precisely the ambiguity that left [esp-now] issue #144 unresolved" — the reviewer's words, and accurate: the reporter there never confirmed whether Espressif's suggested fix actually took effect, because there was no way to tell from the ground.

Task 3's readback (`esp_wifi_get_protocol`) already covers the *bitmap* half reasonably well — that's a local, synchronous, RPC-proxied call with a real return value the P4 can trust. There is no equivalent getter for the *peer rate config* — ESP-NOW's API has no "read back what rate config is active for this peer." The only way to know is for the C6 to say so, proactively, once, right after it tries.

**Files:**
- Modify: `tools/slave_firmware/espnow_bridge.c`
- Modify: `main/transports/espnow_transport.c`
- Modify: `main/transports/espnow_transport.h`

- [ ] **Step 1: Add a new message type on the C6 side, separate from `PEER_MSG_UPSTREAM`**

Deliberately a new channel, not piggybacked onto `PEER_MSG_UPSTREAM`: that channel's payload is parsed by the P4's `upstream_cb` as `espnow_pkt_hdr_t` + a ground-command payload (or the `MSG_GROUND_HELLO` short-circuit) — a proven, historically-fragile contract (see Task 9's own reasoning for why cmd-RSSI was piggybacked there instead of given its own channel: that one genuinely needed to avoid a second RPC call per *command*, at 30Hz. This one is a one-shot per *boot*, so that constraint doesn't apply — a dedicated channel is both simpler and safer here.)

Find (this is Task 1's own addition — confirm it's present before editing):
```c
#define PEER_MSG_VIDEO    1u
#define PEER_MSG_COMMAND  2u
#define PEER_MSG_INIT     3u
#define PEER_MSG_UPSTREAM 4u

/* Long Range PHY toggle for the A/B range test -- flip to 0, rebuild via
 * build.sh, and reflash to get a same-hardware LR-off baseline. Both this
 * file and tools/espnow_bridge/main/main.c (ground station) must be flashed
 * with the SAME value, or LR-rate frames from one end simply go unheard by
 * the other (LR is only demodulable by an LR-enabled receiver). Not a
 * Kconfig entry: this build is regenerated from scratch each run (see
 * build.sh), so there's no persistent sdkconfig to hang a menu option off. */
#define ESPNOW_LR_ENABLED 1
```
Replace with:
```c
#define PEER_MSG_VIDEO    1u
#define PEER_MSG_COMMAND  2u
#define PEER_MSG_INIT     3u
#define PEER_MSG_UPSTREAM 4u
/* C6->P4, one-shot: reports whether esp_now_set_peer_rate_config(LR) in
 * init_cb succeeded. A dedicated channel, not folded into PEER_MSG_UPSTREAM
 * -- this never has to share that channel's espnow_pkt_hdr_t/ground-command
 * parsing contract. */
#define PEER_MSG_INIT_STATUS 5u

/* Long Range PHY toggle for the A/B range test -- flip to 0, rebuild via
 * build.sh, and reflash to get a same-hardware LR-off baseline. Both this
 * file and tools/espnow_bridge/main/main.c (ground station) must be flashed
 * with the SAME value, or LR-rate frames from one end simply go unheard by
 * the other (LR is only demodulable by an LR-enabled receiver). Not a
 * Kconfig entry: this build is regenerated from scratch each run (see
 * build.sh), so there's no persistent sdkconfig to hang a menu option off. */
#define ESPNOW_LR_ENABLED 1
```

- [ ] **Step 2: Send the one-shot status byte at the end of Task 1's rate-config block**

Find (this is Task 1's own addition):
```c
    err = esp_now_set_peer_rate_config(s_peer_mac, &rate_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: esp_now_set_peer_rate_config(LR) failed: %s",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "init_cb: peer rate config set to LR/250K");
    }
#endif /* ESPNOW_LR_ENABLED */

    s_espnow_ready = true;
    ESP_LOGI(TAG, "ESP-NOW bridge ready");
```
Replace with:
```c
    err = esp_now_set_peer_rate_config(s_peer_mac, &rate_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: esp_now_set_peer_rate_config(LR) failed: %s",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "init_cb: peer rate config set to LR/250K");
    }

    /* One-shot, not per-command: reports the RESULT of the call above, once,
     * at init time -- costs one small extra RPC at boot, never touches the
     * 30Hz control path. This is the only way for the P4 (and from there,
     * the ground station UI) to learn whether this specific call actually
     * succeeded -- see the Task 3c header comment for why. */
    uint8_t lr_status = (err == ESP_OK) ? 1u : 0u;
    esp_err_t status_err = esp_hosted_send_custom_data(PEER_MSG_INIT_STATUS, &lr_status, 1);
    if (status_err != ESP_OK) {
        ESP_LOGW(TAG, "init_cb: failed to report LR status to P4: %s",
                 esp_err_to_name(status_err));
    }
#endif /* ESPNOW_LR_ENABLED */

    s_espnow_ready = true;
    ESP_LOGI(TAG, "ESP-NOW bridge ready");
```

- [ ] **Step 3: P4 — add the matching constant**

Find:
```c
#define PEER_MSG_VIDEO    1u   /* P4→C6: sensor telemetry (queued on C6) */
#define PEER_MSG_COMMAND  2u   /* P4→C6: commands (fast path on C6) */
#define PEER_MSG_INIT     3u   /* P4→C6: ESP-NOW init */
#define PEER_MSG_UPSTREAM 4u   /* C6→P4: commands from laptop */
```
Replace with:
```c
#define PEER_MSG_VIDEO    1u   /* P4→C6: sensor telemetry (queued on C6) */
#define PEER_MSG_COMMAND  2u   /* P4→C6: commands (fast path on C6) */
#define PEER_MSG_INIT     3u   /* P4→C6: ESP-NOW init */
#define PEER_MSG_UPSTREAM 4u   /* C6→P4: commands from laptop */
#define PEER_MSG_INIT_STATUS 5u /* C6→P4: one-shot LR rate-config result */
```

- [ ] **Step 4: P4 — add the backing static, next to the reinit-watchdog statics**

Find:
```c
static volatile int64_t   s_last_upstream_us = 0;
static volatile int       s_reinit_attempts  = 0;
static esp_timer_handle_t s_reinit_watchdog  = NULL;
```
Replace with:
```c
static volatile int64_t   s_last_upstream_us = 0;
static volatile int       s_reinit_attempts  = 0;
static esp_timer_handle_t s_reinit_watchdog  = NULL;
static volatile int8_t    s_lr_status        = -1;  /* -1 = not yet reported */
```

- [ ] **Step 5: P4 — add the callback and getter, after `upstream_cb`**

Find:
```c
    const uint8_t *payload = data + ESPNOW_HDR_SIZE;
    pipeline_handle_incoming(payload, payload_len);
}

/* ---------------------------------------------------------------------------
 * espnow_send_fn — transport_send_fn registered with the pipeline
```
Replace with:
```c
    const uint8_t *payload = data + ESPNOW_HDR_SIZE;
    pipeline_handle_incoming(payload, payload_len);
}

/* ---------------------------------------------------------------------------
 * init_status_cb — registered for PEER_MSG_INIT_STATUS (C6→P4, one-shot LR
 * rate-config result; see tools/slave_firmware/espnow_bridge.c's init_cb).
 * -------------------------------------------------------------------------*/
static void init_status_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;
    if (!data || data_len < 1) {
        return;
    }
    s_lr_status = (int8_t)data[0];
    ESP_LOGI(TAG, "Co-processor reports LR rate config: %s",
             s_lr_status ? "OK" : "FAILED");
}

/* ---------------------------------------------------------------------------
 * espnow_transport_lr_status — LR rate-config result reported by the C6.
 * -------------------------------------------------------------------------*/
int8_t espnow_transport_lr_status(void)
{
    return s_lr_status;
}

/* ---------------------------------------------------------------------------
 * espnow_send_fn — transport_send_fn registered with the pipeline
```

- [ ] **Step 6: P4 — register the callback in `espnow_transport_probe()`**

Find:
```c
    /* --- Register upstream callback for C6→P4 commands --- */
    ret = esp_hosted_register_custom_callback(PEER_MSG_UPSTREAM, upstream_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register upstream callback (%s)", esp_err_to_name(ret));
        return ret;
    }
```
Replace with:
```c
    /* --- Register upstream callback for C6→P4 commands --- */
    ret = esp_hosted_register_custom_callback(PEER_MSG_UPSTREAM, upstream_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register upstream callback (%s)", esp_err_to_name(ret));
        return ret;
    }

    /* --- Register the one-shot LR-status callback --- */
    ret = esp_hosted_register_custom_callback(PEER_MSG_INIT_STATUS, init_status_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register init-status callback (%s)", esp_err_to_name(ret));
        return ret;
    }
```

- [ ] **Step 7: Declare the getter in the header**

Find:
```c
esp_err_t espnow_transport_send_telemetry(const espnow_telemetry_t *t);
```
Replace with:
```c
esp_err_t espnow_transport_send_telemetry(const espnow_telemetry_t *t);

/**
 * @brief LR rate-config result reported by the C6 (see init_cb in
 *        tools/slave_firmware/espnow_bridge.c): 1 = esp_now_set_peer_rate_config
 *        succeeded, 0 = it failed, -1 = not yet reported (LR disabled on the
 *        C6 build, or the report hasn't arrived yet).
 */
int8_t espnow_transport_lr_status(void);
```

- [ ] **Step 8: Build both**

Run: `idf.py build` (P4) — expect `Project build complete.`
Run: `cd tools/slave_firmware && ./build.sh` (C6) — expect `=== Bridge link verified: ESP-NOW symbols present in firmware ===`

- [ ] **Step 9: Commit**

```bash
git add tools/slave_firmware/espnow_bridge.c main/transports/espnow_transport.c main/transports/espnow_transport.h
git commit -m "$(cat <<'EOF'
feat(espnow): report LR rate-config success/failure to the P4

New one-shot PEER_MSG_INIT_STATUS channel, deliberately separate from
PEER_MSG_UPSTREAM's command-forwarding contract. Closes the observability
gap the Task 1 review flagged: esp_now_set_peer_rate_config()'s result was
previously only visible on the C6's own unwatched console -- the same
ambiguity that left esp-now issue #144 unresolved.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: Build all three firmwares with LR enabled (default)

**Files:** none (verification only)

- [ ] **Step 1: Build the P4 main firmware**

Run: `idf.py build`
Expected: `Project build complete.`

- [ ] **Step 2: Build the C6 co-processor firmware**

Run: `cd tools/slave_firmware && ./build.sh`
Expected: `=== Bridge link verified: ESP-NOW symbols present in firmware ===`, `=== Build complete ===`.

- [ ] **Step 3: Build the S3 ground-station firmware**

Run: `cd tools/espnow_bridge && idf.py build`
Expected: `Project build complete.`

No commit — this task is a build-only checkpoint before hardware steps.

---

### Task 5: Manual bench verification (LR ON, both ends, close range)

**Files:** none — hardware verification, requires the physical boat + S3 dongle

- [ ] **Step 1: Flash the C6**

Copy the binaries per `build.sh`'s own printed instructions:
```bash
cp tools/slave_firmware/build_slave/slave/build/bootloader/bootloader.bin       tools/sdio_flasher/target-firmware/
cp tools/slave_firmware/build_slave/slave/build/partition_table/partition-table.bin tools/sdio_flasher/target-firmware/
cp tools/slave_firmware/build_slave/slave/build/network_adapter.bin             tools/sdio_flasher/target-firmware/app.bin
```
Then flash via whatever mechanism `tools/sdio_flasher/` documents for this project (SDIO-attached C6 on the P4 board — follow the existing project flashing procedure, not a plain `idf.py flash`, since the C6 isn't USB-attached to this host).

- [ ] **Step 2: Flash the S3**

Run: `cd tools/espnow_bridge && idf.py -p <S3 serial port> flash`

- [ ] **Step 3: Flash the P4 main firmware**

Run: `idf.py -p <P4 serial port> flash monitor`

- [ ] **Step 4: Confirm the LR readback log**

In the P4 monitor output, look for:
```
I (....) ESPNOW_TRANSPORT: Co-processor LR protocol bit is SET (bitmap=0x..)
```
If instead it logs `LR protocol bit NOT set`, stop — check the C6's own serial console (separate UART, if accessible) for `init_cb: esp_wifi_set_protocol(LR) failed` or `esp_now_set_peer_rate_config(LR) failed` before proceeding to the range test.

- [ ] **Step 5: Confirm control still works at close range**

Run `.venv/bin/python tools/espnow_drive.py`, connect to the S3's serial port via the web UI, arm, and verify throttle/rudder respond. This proves LR didn't silently break the baseline link.

- [ ] **Step 6: Confirm the failsafe still fires and auto-resumes**

Power off the S3 (or unplug USB) while armed with nonzero throttle: confirm the P4 log shows `Control link lost — throttle 0, winch 0, rudders centred, servo rail cut` within ~400ms. Power the S3 back on: confirm control resumes without needing to re-arm.

No commit — this is a verification checkpoint. If any check fails, fix the relevant Task 1–3 code before proceeding.

---

### Task 6: Field range A/B test (LR off vs LR on)

**Files:** modify the `ESPNOW_LR_ENABLED` define value only, for the "off" baseline half of this task

- [ ] **Step 1: Build and flash an LR-OFF baseline**

Set `#define ESPNOW_LR_ENABLED 0` in both `tools/slave_firmware/espnow_bridge.c` and `tools/espnow_bridge/main/main.c` (do not commit this — it's a temporary local edit for the baseline run). Rebuild and reflash both (Task 4 Steps 2–3, Task 5 Steps 1–2).

- [ ] **Step 2: Walk the baseline range**

With `tools/espnow_drive.py` connected and armed (propeller clear of obstructions / boat secured — this step drives real hardware), walk the boat (or the S3 dongle, whichever is practical to move) away from the other end. Record the distance at which control becomes unreliable: watch for the `Control link lost` log line firing repeatedly, or (once Part B lands) the `cmd_age_ms` telemetry field climbing past ~200ms regularly. Record this distance as the baseline.

- [ ] **Step 3: Revert to LR-ON, rebuild, reflash**

Restore `#define ESPNOW_LR_ENABLED 1` in both files (back to the committed state from Tasks 1–2). Rebuild and reflash both (Task 4 Steps 2–3, Task 5 Steps 1–2).

- [ ] **Step 4: Walk the LR range**

Repeat Step 2's procedure. Record the distance at which control becomes unreliable.

- [ ] **Step 5: Compare and record the result**

Expect the LR distance to be **materially larger** than baseline (theoretically ~2–2.5x per the C6 docs, though esp-now issue #144 showed a real case of zero gain on this exact chip — so this comparison is the actual proof, not an assumption). If LR shows no improvement:
- Confirm the Task 5 Step 4 readback log showed the LR bit set on both runs.
- Check `esp_now_set_peer_rate_config` returned `ESP_OK` on both C6 and S3 (their own serial logs).
- Consider `esp_wifi_set_max_tx_power` as a follow-up lever (not implemented in this plan — the design doc's risk section covers it as a fallback if this A/B comes back flat).

Document the measured distances wherever this project tracks hardware test results (this plan doesn't prescribe a location — follow existing project convention). No code commit for this task; it's a data-collection checkpoint gating whether Part B's premise (LR is working) is actually true.

---

## Part B — Reliability & link-quality visibility

Only start this part after Task 6 shows LR is functioning (even if the exact range gain is still being characterized) — the point of Part A's isolation is to keep this part's changes from being blamed for a range problem that was actually caused by A.

### Task 7: Extend the wire structs (telemetry + bridge status)

**Files:**
- Modify: `main/transports/espnow_protocol.h`

- [ ] **Step 1: Add `cmd_age_ms` + `cmd_rx_rssi` to `espnow_telemetry_t`**

Find:
```c
typedef struct __attribute__((packed)) {
    float    pitch;
    float    roll;
    float    heading;
    uint8_t  gps_valid;
    double   latitude;
    double   longitude;
    float    speed_mps;
    float    course_deg;
    uint8_t  satellites;
    float    hdop;
} espnow_telemetry_t;
```
Replace with:
```c
typedef struct __attribute__((packed)) {
    float    pitch;
    float    roll;
    float    heading;
    uint8_t  gps_valid;
    double   latitude;
    double   longitude;
    float    speed_mps;
    float    course_deg;
    uint8_t  satellites;
    float    hdop;
    /* Link-health pair, added for LR range/RC-control visibility -- see
     * pipeline_last_command_age_ms() and espnow_transport_last_cmd_rssi().
     * Both describe the DOWNLINK (ground -> boat): the direction that
     * actually determines whether the operator's stick input is arriving. */
    int32_t  cmd_age_ms;   /* ms since the boat last received a decodable
                             * command on ANY transport; -1 if never. */
    int8_t   cmd_rx_rssi;  /* RSSI (dBm) of the last frame the C6 heard from
                             * the ground station. 0 if nothing heard yet. */
    int8_t   lr_active;    /* Mirrors espnow_transport_lr_status(): 1 = C6
                             * confirmed esp_now_set_peer_rate_config(LR)
                             * succeeded, 0 = it failed, -1 = not yet known.
                             * See Task 3c. */
} espnow_telemetry_t;
```

- [ ] **Step 2: Add `last_rx_rssi` + `lr_rate_config_ok` to `espnow_bridge_status_t`**

Find:
```c
/* Payload of MSG_BRIDGE_STATUS (packed, little-endian). */
typedef struct __attribute__((packed)) {
    uint32_t uptime_s;      /* bridge uptime */
    uint32_t espnow_pkts;   /* ESP-NOW packets received off-air */
    uint32_t espnow_bytes;  /* ...and their total payload bytes */
    uint32_t frames_out;    /* complete frames reassembled and sent to USB */
    uint32_t hello_sent;    /* MSG_GROUND_HELLO beacons broadcast */
    uint32_t reasm_drops;   /* frames dropped: overflow or length mismatch */
} espnow_bridge_status_t;
```
Replace with:
```c
/* Payload of MSG_BRIDGE_STATUS (packed, little-endian). */
typedef struct __attribute__((packed)) {
    uint32_t uptime_s;      /* bridge uptime */
    uint32_t espnow_pkts;   /* ESP-NOW packets received off-air */
    uint32_t espnow_bytes;  /* ...and their total payload bytes */
    uint32_t frames_out;    /* complete frames reassembled and sent to USB */
    uint32_t hello_sent;    /* MSG_GROUND_HELLO beacons broadcast */
    uint32_t reasm_drops;   /* frames dropped: overflow or length mismatch */
    int8_t   last_rx_rssi;  /* RSSI (dBm) of the most recent frame the S3
                              * heard from the boat -- UPLINK signal margin,
                              * the ground station's-eye view. 0 if nothing
                              * heard yet. */
    int8_t   lr_rate_config_ok;  /* This bridge's OWN esp_now_set_peer_rate_config()
                                   * result for the broadcast peer (Task 2): 1 = OK,
                                   * 0 = failed, -1 = LR disabled on this build. The
                                   * ground-side counterpart to lr_active in
                                   * espnow_telemetry_t (the boat's/C6's result). */
} espnow_bridge_status_t;
```

- [ ] **Step 3: Build**

Run: `idf.py build`
Expected: `Project build complete.` (This header alone doesn't complete the wire format change — `sensor_task.c` still constructs `espnow_telemetry_t` with designated initializers, which tolerate the new fields defaulting to zero, so this compiles standalone; Task 11 fills them in.)

- [ ] **Step 4: Commit**

```bash
git add main/transports/espnow_protocol.h
git commit -m "$(cat <<'EOF'
feat(espnow): add link-health fields to the field-mode wire structs

cmd_age_ms + cmd_rx_rssi + lr_active on espnow_telemetry_t (downlink
health, boat's view) and last_rx_rssi + lr_rate_config_ok on
espnow_bridge_status_t (uplink health, ground's view). Struct-only --
producers/consumers follow in later commits.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 8: P4 pipeline — expose command age as a value

**Files:**
- Modify: `main/pipeline.h`
- Modify: `main/pipeline.c`

- [ ] **Step 1: Declare `pipeline_last_command_age_ms()` in the header**

Find:
```c
/**
 * @brief True if any transport (WS or ESP-NOW) delivered a decodable
 *        BoatMessage within the last max_age_us microseconds. Transport-
 *        agnostic liveness signal for failsafes — ESP-NOW has no persistent
 *        "connected" concept, so recency of traffic is its only substitute.
 */
bool pipeline_recent_command(int64_t max_age_us);
```
Replace with:
```c
/**
 * @brief True if any transport (WS or ESP-NOW) delivered a decodable
 *        BoatMessage within the last max_age_us microseconds. Transport-
 *        agnostic liveness signal for failsafes — ESP-NOW has no persistent
 *        "connected" concept, so recency of traffic is its only substitute.
 */
bool pipeline_recent_command(int64_t max_age_us);

/**
 * @brief Milliseconds since the last decodable BoatMessage arrived on any
 *        transport, or -1 if none has ever arrived. Same underlying signal
 *        as pipeline_recent_command(), exposed as a value for telemetry/UI
 *        display rather than a threshold check.
 */
int32_t pipeline_last_command_age_ms(void);
```

- [ ] **Step 2: Implement it in pipeline.c, next to `pipeline_recent_command`**

Find:
```c
bool pipeline_recent_command(int64_t max_age_us)
{
    portENTER_CRITICAL(&s_rx_time_lock);
    int64_t last = s_last_rx_us;
    portEXIT_CRITICAL(&s_rx_time_lock);

    if (last == 0) return false;   /* nothing ever received */
    return (esp_timer_get_time() - last) <= max_age_us;
}
```
Replace with:
```c
bool pipeline_recent_command(int64_t max_age_us)
{
    portENTER_CRITICAL(&s_rx_time_lock);
    int64_t last = s_last_rx_us;
    portEXIT_CRITICAL(&s_rx_time_lock);

    if (last == 0) return false;   /* nothing ever received */
    return (esp_timer_get_time() - last) <= max_age_us;
}

int32_t pipeline_last_command_age_ms(void)
{
    portENTER_CRITICAL(&s_rx_time_lock);
    int64_t last = s_last_rx_us;
    portEXIT_CRITICAL(&s_rx_time_lock);

    if (last == 0) return -1;   /* nothing ever received */

    int64_t age_ms = (esp_timer_get_time() - last) / 1000;
    if (age_ms < 0) age_ms = 0;
    if (age_ms > INT32_MAX) age_ms = INT32_MAX;
    return (int32_t)age_ms;
}
```

- [ ] **Step 3: Build**

Run: `idf.py build`
Expected: `Project build complete.`

- [ ] **Step 4: Commit**

```bash
git add main/pipeline.h main/pipeline.c
git commit -m "$(cat <<'EOF'
feat(pipeline): expose command age as a value, not just a threshold check

pipeline_last_command_age_ms() reads the same s_last_rx_us liveness signal
pipeline_recent_command() already uses, for telemetry/UI display.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 9: C6 — capture and forward command RSSI

**Files:**
- Modify: `tools/slave_firmware/espnow_bridge.c`

- [ ] **Step 1: Append RSSI as a trailing byte in `espnow_recv_cb`**

This is safe against the P4's existing `upstream_cb` parsing — verified by reading it (see Task 10): it dispatches using the *header's own* `payload_len`, not the RPC's `data_len`, for both the `MSG_GROUND_HELLO` branch and `pipeline_handle_incoming()`, so one extra trailing byte is silently ignored by every current consumer. This avoids a second `esp_hosted_send_custom_data()` call per received frame — at the new 30Hz command rate (Task 13), that would double C6→P4 RPC traffic on the exact link this file already has to reason carefully about (see the `video_cb`/`command_cb` comments).

Find:
```c
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    if (len <= 0 || !data) {
        return;
    }

    ESP_LOGD(TAG, "espnow_recv_cb: %d bytes from " MACSTR,
             len, MAC2STR(info->src_addr));

    esp_err_t err = esp_hosted_send_custom_data(PEER_MSG_UPSTREAM,
                                                data, (size_t)len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "espnow_recv_cb: esp_hosted_send_custom_data failed: %s",
                 esp_err_to_name(err));
    }
}
```
Replace with:
```c
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    if (len <= 0 || !data) {
        return;
    }

    ESP_LOGD(TAG, "espnow_recv_cb: %d bytes from " MACSTR " (rssi=%d)",
             len, MAC2STR(info->src_addr), info->rx_ctrl->rssi);

    /* Append the RSSI of THIS frame as one trailing byte before forwarding.
     * upstream_cb on the P4 (main/transports/espnow_transport.c) dispatches
     * using the forwarded HEADER's own payload_len, not this call's data_len,
     * for both the MSG_GROUND_HELLO short-circuit and pipeline_handle_incoming()
     * -- verified by reading both branches, not assumed -- so the extra
     * trailing byte is silently ignored by every existing consumer. */
    if (len > (int)ESPNOW_MAX_PAYLOAD) {
        len = (int)ESPNOW_MAX_PAYLOAD;   /* defensive; ESP-NOW caps this well below anyway */
    }
    uint8_t buf[ESPNOW_MAX_PAYLOAD + 1];
    memcpy(buf, data, (size_t)len);
    buf[len] = (uint8_t)info->rx_ctrl->rssi;

    esp_err_t err = esp_hosted_send_custom_data(PEER_MSG_UPSTREAM, buf, (size_t)len + 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "espnow_recv_cb: esp_hosted_send_custom_data failed: %s",
                 esp_err_to_name(err));
    }
}
```

- [ ] **Step 2: Build**

Run: `cd tools/slave_firmware && ./build.sh`
Expected: `=== Bridge link verified: ESP-NOW symbols present in firmware ===`

- [ ] **Step 3: Commit**

```bash
git add tools/slave_firmware/espnow_bridge.c
git commit -m "$(cat <<'EOF'
feat(espnow): forward command RSSI to the P4 as a trailing byte

Piggybacks on the existing PEER_MSG_UPSTREAM forward instead of adding a
second RPC call per command -- verified upstream_cb ignores trailing bytes
on both its MSG_GROUND_HELLO and pipeline_handle_incoming() paths.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 10: P4 — strip the RSSI byte and expose it

**Files:**
- Modify: `main/transports/espnow_transport.c`
- Modify: `main/transports/espnow_transport.h`

- [ ] **Step 1: Strip the trailing byte in `upstream_cb`, before existing parsing**

Find:
```c
static void upstream_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;
    s_last_upstream_us = esp_timer_get_time();
    s_reinit_attempts  = 0;   /* real traffic heard -- link is genuinely alive again */

    if (!data || data_len < ESPNOW_HDR_SIZE) {
        ESP_LOGW(TAG, "upstream_cb: short frame (%u bytes)", (unsigned)data_len);
        return;
    }

    const espnow_pkt_hdr_t *hdr = (const espnow_pkt_hdr_t *)data;
```
Replace with:
```c
static void upstream_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;
    s_last_upstream_us = esp_timer_get_time();
    s_reinit_attempts  = 0;   /* real traffic heard -- link is genuinely alive again */

    /* +1: espnow_recv_cb() on the C6 (tools/slave_firmware/espnow_bridge.c)
     * appends one trailing RSSI byte to every frame it forwards, ahead of
     * the existing espnow_pkt_hdr_t/payload contract below -- strip it
     * first so nothing downstream has to know it exists. */
    if (!data || data_len < ESPNOW_HDR_SIZE + 1) {
        ESP_LOGW(TAG, "upstream_cb: short frame (%u bytes)", (unsigned)data_len);
        return;
    }
    s_last_cmd_rssi = (int8_t)data[data_len - 1];
    data_len -= 1;

    const espnow_pkt_hdr_t *hdr = (const espnow_pkt_hdr_t *)data;
```

- [ ] **Step 2: Add the backing static, next to `s_last_upstream_us`**

Find:
```c
static volatile int64_t   s_last_upstream_us = 0;
static volatile int       s_reinit_attempts  = 0;
static esp_timer_handle_t s_reinit_watchdog  = NULL;
```
Replace with:
```c
static volatile int64_t   s_last_upstream_us = 0;
static volatile int       s_reinit_attempts  = 0;
static esp_timer_handle_t s_reinit_watchdog  = NULL;
static volatile int8_t    s_last_cmd_rssi    = 0;
```

- [ ] **Step 3: Add the getter, after `upstream_cb`**

Find:
```c
    const uint8_t *payload = data + ESPNOW_HDR_SIZE;
    pipeline_handle_incoming(payload, payload_len);
}

/* ---------------------------------------------------------------------------
 * espnow_send_fn — transport_send_fn registered with the pipeline
```
Replace with:
```c
    const uint8_t *payload = data + ESPNOW_HDR_SIZE;
    pipeline_handle_incoming(payload, payload_len);
}

/* ---------------------------------------------------------------------------
 * espnow_transport_last_cmd_rssi — RSSI of the last frame the C6 heard from
 * the ground station (see upstream_cb's trailing-byte strip above).
 * -------------------------------------------------------------------------*/
int8_t espnow_transport_last_cmd_rssi(void)
{
    return s_last_cmd_rssi;
}

/* ---------------------------------------------------------------------------
 * espnow_send_fn — transport_send_fn registered with the pipeline
```

- [ ] **Step 4: Declare the getter in the header**

Find:
```c
/**
 * @brief Send field-mode telemetry (IMU + GPS only) as a compact,
 *        non-protobuf struct — bypasses the pipeline/boat.proto path
 *        entirely. Called directly by sensor_task.c when g_field_mode is
 *        true, instead of pipeline_publish_sensors(). See espnow_telemetry_t
 *        in espnow_protocol.h for why.
 */
esp_err_t espnow_transport_send_telemetry(const espnow_telemetry_t *t);
```
Replace with:
```c
/**
 * @brief Send field-mode telemetry (IMU + GPS only) as a compact,
 *        non-protobuf struct — bypasses the pipeline/boat.proto path
 *        entirely. Called directly by sensor_task.c when g_field_mode is
 *        true, instead of pipeline_publish_sensors(). See espnow_telemetry_t
 *        in espnow_protocol.h for why.
 */
esp_err_t espnow_transport_send_telemetry(const espnow_telemetry_t *t);

/**
 * @brief RSSI (dBm) of the last frame the C6 heard from the ground station,
 *        as reported by its esp_now_recv_info_t.rx_ctrl. 0 if nothing has
 *        been heard yet. Downlink signal-margin companion to
 *        pipeline_last_command_age_ms() -- populate both into
 *        espnow_telemetry_t before each send (see sensor_task.c).
 */
int8_t espnow_transport_last_cmd_rssi(void);
```

- [ ] **Step 5: Build**

Run: `idf.py build`
Expected: `Project build complete.`

- [ ] **Step 6: Commit**

```bash
git add main/transports/espnow_transport.c main/transports/espnow_transport.h
git commit -m "$(cat <<'EOF'
feat(espnow): strip the C6-forwarded RSSI byte and expose it

espnow_transport_last_cmd_rssi() reads the trailing byte espnow_recv_cb()
now appends on the C6 (previous commit) -- upstream_cb strips it before
any of the existing header/payload parsing runs.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 11: P4 — populate the new telemetry fields

**Files:**
- Modify: `main/sensor_task.c`

- [ ] **Step 1: Confirm `pipeline.h` is included**

Run: `grep -n '#include "pipeline.h"' main/sensor_task.c`
Expected: one match. If there is no match, add `#include "pipeline.h"` alongside `sensor_task.c`'s other local includes (it already includes `espnow_transport.h`, since it calls `espnow_transport_send_telemetry`).

- [ ] **Step 2: Populate `cmd_age_ms` and `cmd_rx_rssi` in the field-mode branch**

Find:
```c
        if (g_field_mode) {
            /* ESP-NOW: compact struct, IMU+GPS only -- see espnow_telemetry_t
             * in espnow_protocol.h for why (no ToF, no detections, no
             * boat.proto at all on this path). WiFi/WS mode never reaches
             * this branch, so its full-fidelity snapshot is untouched. */
            espnow_telemetry_t tel = {
                .pitch   = imu.pitch,
                .roll    = imu.roll,
                .heading = imu.heading,
            };
```
Replace with:
```c
        if (g_field_mode) {
            /* ESP-NOW: compact struct, IMU+GPS only -- see espnow_telemetry_t
             * in espnow_protocol.h for why (no ToF, no detections, no
             * boat.proto at all on this path). WiFi/WS mode never reaches
             * this branch, so its full-fidelity snapshot is untouched. */
            espnow_telemetry_t tel = {
                .pitch       = imu.pitch,
                .roll        = imu.roll,
                .heading     = imu.heading,
                .cmd_age_ms  = pipeline_last_command_age_ms(),
                .cmd_rx_rssi = espnow_transport_last_cmd_rssi(),
                .lr_active   = espnow_transport_lr_status(),
            };
```

- [ ] **Step 3: Build**

Run: `idf.py build`
Expected: `Project build complete.`

- [ ] **Step 4: Commit**

```bash
git add main/sensor_task.c
git commit -m "$(cat <<'EOF'
feat(espnow): populate cmd_age_ms/cmd_rx_rssi/lr_active in field telemetry

All three values were already computed for free (pipeline's existing
liveness timestamp, the C6's forwarded RSSI byte, and Task 3c's one-shot
LR status report) -- this just wires them into the
struct sensor_task.c already sends every field-mode tick.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 12: S3 — capture uplink RSSI, extend bridge status

**Files:**
- Modify: `tools/espnow_bridge/main/main.c`

- [ ] **Step 1: Add the backing static near the other counters**

Find:
```c
static volatile uint32_t s_espnow_pkts   = 0;
static volatile uint32_t s_espnow_bytes  = 0;
static volatile uint32_t s_frames_out    = 0;
static volatile uint32_t s_hello_sent    = 0;
static volatile uint32_t s_reasm_drops   = 0;
```
Replace with:
```c
static volatile uint32_t s_espnow_pkts   = 0;
static volatile uint32_t s_espnow_bytes  = 0;
static volatile uint32_t s_frames_out    = 0;
static volatile uint32_t s_hello_sent    = 0;
static volatile uint32_t s_reasm_drops   = 0;
static volatile int8_t   s_last_rx_rssi  = 0;
static volatile int8_t   s_lr_rate_config_ok = -1;  /* -1 = LR disabled on this build */
```

- [ ] **Step 2: Capture this bridge's own LR rate-config result, in `espnow_init()`**

This is Task 2's `esp_now_set_peer_rate_config(BROADCAST_MAC, ...)` call — the one the design doc flagged as genuinely uncertain (does a broadcast peer even accept a rate config?). Capture its result instead of only logging it, so it can be reported to the laptop below.

Find (from Task 2's own addition — confirm present before editing):
```c
    esp_err_t rc_err = esp_now_set_peer_rate_config(BROADCAST_MAC, &rate_cfg);
    if (rc_err != ESP_OK) {
        ESP_LOGW(TAG, "espnow_init: esp_now_set_peer_rate_config(broadcast, LR) "
                      "failed: %s -- LR bitmap is still set; broadcast frames "
                      "may fall back to normal rate", esp_err_to_name(rc_err));
    } else {
        ESP_LOGI(TAG, "espnow_init: broadcast peer rate config set to LR/250K");
    }
#endif
}
```
Replace with:
```c
    esp_err_t rc_err = esp_now_set_peer_rate_config(BROADCAST_MAC, &rate_cfg);
    s_lr_rate_config_ok = (rc_err == ESP_OK) ? 1 : 0;
    if (rc_err != ESP_OK) {
        ESP_LOGW(TAG, "espnow_init: esp_now_set_peer_rate_config(broadcast, LR) "
                      "failed: %s -- LR bitmap is still set; broadcast frames "
                      "may fall back to normal rate", esp_err_to_name(rc_err));
    } else {
        ESP_LOGI(TAG, "espnow_init: broadcast peer rate config set to LR/250K");
    }
#endif
}
```

- [ ] **Step 3: Capture RSSI at the top of `espnow_recv_cb`**

Find:
```c
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int data_len)
{
    if (data_len < (int)FRAG_HDR_SIZE) {
        ESP_LOGW(TAG, "Short ESP-NOW packet (%d bytes) — ignored", data_len);
        return;
    }

    uint32_t total_len;
```
Replace with:
```c
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int data_len)
{
    if (data_len < (int)FRAG_HDR_SIZE) {
        ESP_LOGW(TAG, "Short ESP-NOW packet (%d bytes) — ignored", data_len);
        return;
    }

    /* Uplink signal margin, the ground station's-eye view -- every frame
     * the S3 hears from the boat updates this, regardless of what kind of
     * frame it turns out to be. Reported to the laptop via MSG_BRIDGE_STATUS
     * below. */
    s_last_rx_rssi = (int8_t)info->rx_ctrl->rssi;

    uint32_t total_len;
```

- [ ] **Step 4: Extend the `status_task()` payload**

Find:
```c
        uint8_t payload[4 + 24];
        payload[0] = MSG_BRIDGE_STATUS;
        uint16_t plen = 24;
        memcpy(payload + 1, &plen, 2);
        payload[3] = seq++;

        uint32_t vals[6] = {
            (uint32_t)(esp_timer_get_time() / 1000000),
            s_espnow_pkts, s_espnow_bytes,
            s_frames_out,  s_hello_sent, s_reasm_drops,
        };
        memcpy(payload + 4, vals, sizeof(vals));

        uint8_t *cobs = malloc(sizeof(payload) + COBS_MAX_OVERHEAD(sizeof(payload)));
```
Replace with:
```c
        uint8_t payload[4 + 26];
        payload[0] = MSG_BRIDGE_STATUS;
        uint16_t plen = 26;
        memcpy(payload + 1, &plen, 2);
        payload[3] = seq++;

        uint32_t vals[6] = {
            (uint32_t)(esp_timer_get_time() / 1000000),
            s_espnow_pkts, s_espnow_bytes,
            s_frames_out,  s_hello_sent, s_reasm_drops,
        };
        memcpy(payload + 4, vals, sizeof(vals));
        int8_t rssi = s_last_rx_rssi;
        memcpy(payload + 4 + sizeof(vals), &rssi, sizeof(rssi));
        int8_t lr_ok = s_lr_rate_config_ok;
        memcpy(payload + 4 + sizeof(vals) + sizeof(rssi), &lr_ok, sizeof(lr_ok));

        uint8_t *cobs = malloc(sizeof(payload) + COBS_MAX_OVERHEAD(sizeof(payload)));
```

- [ ] **Step 5: Build**

Run: `cd tools/espnow_bridge && idf.py build`
Expected: `Project build complete.`

- [ ] **Step 6: Commit**

```bash
git add tools/espnow_bridge/main/main.c
git commit -m "$(cat <<'EOF'
feat(espnow): report uplink RSSI + LR status in MSG_BRIDGE_STATUS

Both captured for free (existing espnow_recv_cb for RSSI, Task 2's own
rate-config call for LR status) and piggybacked onto the bridge's
existing 1Hz USB status report -- no new RPC or send path.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 13: espnow_drive.py — decode extension + 30Hz send rate

**Files:**
- Modify: `tools/espnow_drive.py`

- [ ] **Step 1: Extend `FIELD_TELEMETRY_FMT` and raise `SEND_HZ`**

Find:
```python
MSG_FIELD_TELEMETRY = 0x08
FIELD_TELEMETRY_FMT = '<fffBddffBf'
ESPNOW_HDR_SIZE = 4
ESP_NOW_MAX_DATA_LEN = 250     # ESP-NOW v1 single-packet limit
SEND_HZ = 15
```
Replace with:
```python
MSG_FIELD_TELEMETRY = 0x08
# Layout must match espnow_telemetry_t exactly: pitch,roll,heading (f) +
# gps_valid (B) + lat,lon (d) + speed,course (f) + satellites (B) + hdop (f)
# + cmd_age_ms (i, int32) + cmd_rx_rssi (b, int8) + lr_active (b, int8),
# packed, little-endian.
FIELD_TELEMETRY_FMT = '<fffBddffBfibb'
ESPNOW_HDR_SIZE = 4
ESP_NOW_MAX_DATA_LEN = 250     # ESP-NOW v1 single-packet limit
# 30Hz: ~12-packet margin before CONTROL_LINK_TIMEOUT_MS (main/motor_control.c)
# trips, up from ~6 at the old 15Hz -- tighter feel, more loss headroom at range.
SEND_HZ = 30
CONTROL_LINK_TIMEOUT_MS = 400  # mirrors CONTROL_LINK_TIMEOUT_US in main/motor_control.c
```

- [ ] **Step 2: Extend the telemetry decode**

Find:
```python
        if msg_type == MSG_FIELD_TELEMETRY:
            expect_len = struct.calcsize(FIELD_TELEMETRY_FMT)
            if len(payload) != plen or plen != expect_len:
                self._diag('field telemetry length mismatch',
                            f'expected {expect_len}B, got {len(payload)}B (header said {plen}B)')
                return
            (pitch, roll, heading, gps_valid, lat, lon,
             speed_mps, course_deg, satellites, hdop) = struct.unpack(FIELD_TELEMETRY_FMT, payload)
            self._diag_counts['ok'] = self._diag_counts.get('ok', 0) + 1
            with self._lock:
                self.telemetry = {
                    'have': True, 'last_rx_monotonic': time.monotonic(),
                    'heading': heading, 'pitch': pitch, 'roll': roll,
                    'gps_valid': bool(gps_valid), 'lat': lat, 'lon': lon,
                    'speed_mps': speed_mps, 'course_deg': course_deg,
                    'satellites': satellites, 'hdop': hdop,
                }
            return
```
Replace with:
```python
        if msg_type == MSG_FIELD_TELEMETRY:
            expect_len = struct.calcsize(FIELD_TELEMETRY_FMT)
            if len(payload) != plen or plen != expect_len:
                self._diag('field telemetry length mismatch',
                            f'expected {expect_len}B, got {len(payload)}B (header said {plen}B)')
                return
            (pitch, roll, heading, gps_valid, lat, lon,
             speed_mps, course_deg, satellites, hdop,
             cmd_age_ms, cmd_rx_rssi, lr_active) = struct.unpack(FIELD_TELEMETRY_FMT, payload)
            self._diag_counts['ok'] = self._diag_counts.get('ok', 0) + 1
            with self._lock:
                self.telemetry = {
                    'have': True, 'last_rx_monotonic': time.monotonic(),
                    'heading': heading, 'pitch': pitch, 'roll': roll,
                    'gps_valid': bool(gps_valid), 'lat': lat, 'lon': lon,
                    'speed_mps': speed_mps, 'course_deg': course_deg,
                    'satellites': satellites, 'hdop': hdop,
                    'cmd_age_ms': cmd_age_ms, 'cmd_rx_rssi': cmd_rx_rssi,
                    'lr_active': lr_active,
                }
            return
```

- [ ] **Step 3: Extend `_blank_telemetry()`**

Find:
```python
    @staticmethod
    def _blank_telemetry() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'heading': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'gps_valid': False, 'lat': 0.0, 'lon': 0.0,
            'speed_mps': 0.0, 'course_deg': 0.0,
            'satellites': 0, 'hdop': 0.0,
        }
```
Replace with:
```python
    @staticmethod
    def _blank_telemetry() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'heading': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'gps_valid': False, 'lat': 0.0, 'lon': 0.0,
            'speed_mps': 0.0, 'course_deg': 0.0,
            'satellites': 0, 'hdop': 0.0,
            'cmd_age_ms': -1, 'cmd_rx_rssi': 0, 'lr_active': -1,
        }
```

- [ ] **Step 4: Extend the bridge-status decode and `_blank_bridge_status()`**

Find:
```python
        if msg_type == MSG_BRIDGE_STATUS:
            if len(payload) != plen or len(payload) < 24:
                return
            uptime_s, pkts, byts, frames, hello, drops = struct.unpack('<6I', payload[:24])
            with self._lock:
                self.bridge_status = {
                    'have': True, 'last_rx_monotonic': time.monotonic(),
                    'uptime_s': uptime_s, 'espnow_pkts': pkts, 'espnow_bytes': byts,
                    'frames_out': frames, 'hello_sent': hello, 'reasm_drops': drops,
                }
            return
```
Replace with:
```python
        if msg_type == MSG_BRIDGE_STATUS:
            if len(payload) != plen or len(payload) < 26:
                return
            uptime_s, pkts, byts, frames, hello, drops, last_rx_rssi, lr_rate_config_ok = \
                struct.unpack('<6Ibb', payload[:26])
            with self._lock:
                self.bridge_status = {
                    'have': True, 'last_rx_monotonic': time.monotonic(),
                    'uptime_s': uptime_s, 'espnow_pkts': pkts, 'espnow_bytes': byts,
                    'frames_out': frames, 'hello_sent': hello, 'reasm_drops': drops,
                    'last_rx_rssi': last_rx_rssi, 'lr_rate_config_ok': lr_rate_config_ok,
                }
            return
```

Find:
```python
    @staticmethod
    def _blank_bridge_status() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'uptime_s': 0, 'espnow_pkts': 0, 'espnow_bytes': 0,
            'frames_out': 0, 'hello_sent': 0, 'reasm_drops': 0,
        }
```
Replace with:
```python
    @staticmethod
    def _blank_bridge_status() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'uptime_s': 0, 'espnow_pkts': 0, 'espnow_bytes': 0,
            'frames_out': 0, 'hello_sent': 0, 'reasm_drops': 0,
            'last_rx_rssi': 0, 'lr_rate_config_ok': -1,
        }
```

- [ ] **Step 5: Verify the module still imports cleanly**

Run: `.venv/bin/python -c "import sys; sys.path.insert(0, 'tools'); import espnow_drive"`
Expected: no output, exit code 0 (a syntax/import error would print a traceback).

- [ ] **Step 6: Commit**

```bash
git add tools/espnow_drive.py
git commit -m "$(cat <<'EOF'
feat(control): decode link-health telemetry; raise send rate to 30Hz

FIELD_TELEMETRY_FMT and the bridge-status format both grew a field
(coordinated wire change with the firmware commits above). SEND_HZ 15->30
widens the margin before CONTROL_LINK_TIMEOUT_MS from ~6 packets to ~12.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 14: visualize.py — matching decode extension

**Files:**
- Modify: `visualize.py`

This is a coordinated wire-format change, not independent new logic — `visualize.py` maintains its own copy of `FIELD_TELEMETRY_FMT` (documented in the file itself as "replicated... since this is a separate build/tool"). Skipping this file would leave its telemetry decode permanently broken (exact-length check) the moment the firmware ships the wider struct.

- [ ] **Step 1: Update `FIELD_TELEMETRY_FMT`**

Find:
```python
MSG_FIELD_TELEMETRY = 0x08
FIELD_TELEMETRY_FMT = '<fffBddffBf'
```
Replace with:
```python
MSG_FIELD_TELEMETRY = 0x08
FIELD_TELEMETRY_FMT = '<fffBddffBfibb'  # + cmd_age_ms (i), cmd_rx_rssi (b),
                                         # lr_active (b) -- keep in sync with
                                         # main/transports/espnow_protocol.h
```

- [ ] **Step 2: Update the unpack call to match the new tuple width**

Find:
```python
    elif msg_type == MSG_FIELD_TELEMETRY:
        expect_len = struct.calcsize(FIELD_TELEMETRY_FMT)
        if len(payload) == payload_len == expect_len:
            pitch, roll, heading, _valid, _lat, _lon, _spd, _crs, _sats, _hdop = \
                struct.unpack(FIELD_TELEMETRY_FMT, payload)
            with imu_lock:
                imu_data['pitch']   = pitch
                imu_data['roll']    = roll
                imu_data['heading'] = heading
```
Replace with:
```python
    elif msg_type == MSG_FIELD_TELEMETRY:
        expect_len = struct.calcsize(FIELD_TELEMETRY_FMT)
        if len(payload) == payload_len == expect_len:
            (pitch, roll, heading, _valid, _lat, _lon, _spd, _crs, _sats, _hdop,
             _cmd_age_ms, _cmd_rx_rssi, _lr_active) = struct.unpack(FIELD_TELEMETRY_FMT, payload)
            with imu_lock:
                imu_data['pitch']   = pitch
                imu_data['roll']    = roll
                imu_data['heading'] = heading
```

(The bridge-status decode at line ~318-322 uses `len(payload) >= 24` and only reads `payload[:24]` — it already tolerates any number of new trailing bytes from Tasks 12 without any change here.)

- [ ] **Step 3: Verify the module still imports cleanly**

Run: `.venv/bin/python -c "import ast; ast.parse(open('visualize.py').read())"`
Expected: no output, exit code 0.

- [ ] **Step 4: Commit**

```bash
git add visualize.py
git commit -m "$(cat <<'EOF'
fix(visualize): match the widened FIELD_TELEMETRY_FMT wire format

Coordinated change with the firmware/espnow_drive.py commits above --
visualize.py keeps its own copy of this format string and would otherwise
permanently fail its length check once the boat starts sending the wider
struct.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 15: espnow_drive.py — `link_quality()` pure function (TDD)

**Files:**
- Modify: `tools/espnow_drive.py`
- Create: `tools/test_espnow_drive.py`

This is the one piece of new logic in this plan that's a pure function, runnable off-hardware — genuinely TDD-able, unlike the firmware tasks (no test framework exists for the embedded C in this project) or the coordinated-decode tasks above (mechanical, not new logic).

- [ ] **Step 1: Write the failing test**

```python
# tools/test_espnow_drive.py
"""Unit tests for the pure, host-runnable logic in espnow_drive.py.

Run: .venv/bin/python -m unittest tools.test_espnow_drive -v
(or: cd tools && ../.venv/bin/python -m unittest test_espnow_drive -v)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import espnow_drive as ed


class LinkQualityTest(unittest.TestCase):
    def test_no_telemetry_yet_is_unknown(self):
        self.assertEqual(ed.link_quality(cmd_age_ms=0, have_telemetry=False), 'unknown')

    def test_never_received_a_command_is_unknown(self):
        self.assertEqual(ed.link_quality(cmd_age_ms=-1, have_telemetry=True), 'unknown')

    def test_fresh_command_is_green(self):
        self.assertEqual(ed.link_quality(cmd_age_ms=0, have_telemetry=True), 'green')
        self.assertEqual(
            ed.link_quality(cmd_age_ms=ed.CONTROL_LINK_TIMEOUT_MS // 2 - 1, have_telemetry=True),
            'green')

    def test_halfway_to_timeout_is_amber(self):
        self.assertEqual(
            ed.link_quality(cmd_age_ms=ed.CONTROL_LINK_TIMEOUT_MS // 2, have_telemetry=True),
            'amber')
        self.assertEqual(
            ed.link_quality(cmd_age_ms=ed.CONTROL_LINK_TIMEOUT_MS - 1, have_telemetry=True),
            'amber')

    def test_at_or_past_firmware_timeout_is_red(self):
        # Matches CONTROL_LINK_TIMEOUT_MS exactly -- by this point
        # motor_control.c's own watchdog has already zeroed throttle.
        self.assertEqual(
            ed.link_quality(cmd_age_ms=ed.CONTROL_LINK_TIMEOUT_MS, have_telemetry=True), 'red')
        self.assertEqual(
            ed.link_quality(cmd_age_ms=ed.CONTROL_LINK_TIMEOUT_MS + 500, have_telemetry=True),
            'red')


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run it to confirm it fails**

Run: `.venv/bin/python -m unittest tools.test_espnow_drive -v`
Expected: `AttributeError: module 'espnow_drive' has no attribute 'link_quality'` (and `CONTROL_LINK_TIMEOUT_MS` already exists from Task 13, so that part resolves fine — only `link_quality` is missing).

- [ ] **Step 3: Implement `link_quality()`**

Find (in `tools/espnow_drive.py`, right after the `CONTROL_LINK_TIMEOUT_MS` constant added in Task 13):
```python
SEND_HZ = 30
CONTROL_LINK_TIMEOUT_MS = 400  # mirrors CONTROL_LINK_TIMEOUT_US in main/motor_control.c
```
Replace with:
```python
SEND_HZ = 30
CONTROL_LINK_TIMEOUT_MS = 400  # mirrors CONTROL_LINK_TIMEOUT_US in main/motor_control.c


def link_quality(cmd_age_ms, have_telemetry: bool) -> str:
    """Classify downlink health from the boat's own cmd_age_ms report.

    'red' at/after CONTROL_LINK_TIMEOUT_MS matches the firmware's own
    failsafe threshold exactly -- by the time this says red, motor_control.c
    has already zeroed throttle. 'amber' gives the operator warning before
    that happens. 'unknown' (not 'red') when telemetry itself hasn't arrived
    or no command has ever been received -- a fresh connection just doesn't
    know yet, that's a different state from a link that WAS good and died.
    """
    if not have_telemetry or cmd_age_ms is None or cmd_age_ms < 0:
        return 'unknown'
    if cmd_age_ms >= CONTROL_LINK_TIMEOUT_MS:
        return 'red'
    if cmd_age_ms >= CONTROL_LINK_TIMEOUT_MS // 2:
        return 'amber'
    return 'green'
```

- [ ] **Step 4: Run it to confirm it passes**

Run: `.venv/bin/python -m unittest tools.test_espnow_drive -v`
Expected: `OK` with 5 tests passed (`test_no_telemetry_yet_is_unknown`, `test_never_received_a_command_is_unknown`, `test_fresh_command_is_green`, `test_halfway_to_timeout_is_amber`, `test_at_or_past_firmware_timeout_is_red`).

- [ ] **Step 5: Commit**

```bash
git add tools/espnow_drive.py tools/test_espnow_drive.py
git commit -m "$(cat <<'EOF'
feat(control): add link_quality() classifier, with unit tests

Pure function, thresholds tied directly to CONTROL_LINK_TIMEOUT_MS so
"red" in the UI means the firmware failsafe has actually fired, not an
arbitrary guess. First unit tests in this project's Python tooling --
plain unittest, no new dependency.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 16: espnow_drive.py — wire link quality into the UI

**Files:**
- Modify: `tools/espnow_drive.py`

- [ ] **Step 1: Add `link_quality` to the `status()` dict**

Find:
```python
    def status(self) -> dict:
        with self._lock:
            return {
                'connected': self.connected,
                'port': self.port,
                'throttle': self.throttle,
                'rudder': self.rudder,
                'armed_cmd': self.armed_cmd,
                'force': self.force,
                'seq': self.seq,
                'last_error': self.last_error,
                'telemetry': self._with_age(self.telemetry, TELEMETRY_STALE_S),
                # Bridge status arrives ~1/s from the S3 -- a longer stale
                # window than telemetry is correct, not a copy/paste of it.
                'bridge_status': self._with_age(self.bridge_status, 3.0),
            }
```
Replace with:
```python
    def status(self) -> dict:
        with self._lock:
            telemetry = self._with_age(self.telemetry, TELEMETRY_STALE_S)
            return {
                'connected': self.connected,
                'port': self.port,
                'throttle': self.throttle,
                'rudder': self.rudder,
                'armed_cmd': self.armed_cmd,
                'force': self.force,
                'seq': self.seq,
                'last_error': self.last_error,
                'telemetry': telemetry,
                # Bridge status arrives ~1/s from the S3 -- a longer stale
                # window than telemetry is correct, not a copy/paste of it.
                'bridge_status': self._with_age(self.bridge_status, 3.0),
                # Downlink classification, computed here (not client-side) so
                # the threshold lives in one place next to CONTROL_LINK_TIMEOUT_MS.
                'link_quality': link_quality(
                    telemetry.get('cmd_age_ms', -1) if telemetry['have'] and not telemetry['stale'] else -1,
                    telemetry['have'] and not telemetry['stale']),
            }
```

- [ ] **Step 2: Add link-health rows to the telemetry and bridge cards**

Find:
```html
<div class="card" id="telemetry-card">
  <div class="card-title">Telemetry <span class="pill" id="telem-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>Heading</label><span class="val" id="t-heading">--</span></div>
  <div class="telem-row"><label>Pitch / Roll</label><span class="val" id="t-attitude">--</span></div>
  <div class="telem-row"><label>GPS Fix</label><span class="val" id="t-fix">--</span></div>
  <div class="telem-row"><label>Lat / Lon</label><span class="val" id="t-latlon">--</span></div>
  <div class="telem-row"><label>Sats / HDOP</label><span class="val" id="t-sats">--</span></div>
  <div class="telem-row"><label>Speed / Course</label><span class="val" id="t-speed">--</span></div>
</div>

<div class="card" id="bridge-card">
  <div class="card-title">Bridge (S3) <span class="pill" id="bridge-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>ESP-NOW pkts</label><span class="val" id="b-pkts">--</span></div>
  <div class="telem-row"><label>Frames fwd'd</label><span class="val" id="b-frames">--</span></div>
  <div class="telem-row"><label>Hello sent</label><span class="val" id="b-hello">--</span></div>
  <div class="telem-row"><label>Reasm drops</label><span class="val" id="b-drops">--</span></div>
```
Replace with:
```html
<div class="card" id="telemetry-card">
  <div class="card-title">Telemetry <span class="pill" id="telem-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>Link (downlink)</label><span class="val" id="t-link">--</span></div>
  <div class="telem-row"><label>LR mode (boat)</label><span class="val" id="t-lr">--</span></div>
  <div class="telem-row"><label>Heading</label><span class="val" id="t-heading">--</span></div>
  <div class="telem-row"><label>Pitch / Roll</label><span class="val" id="t-attitude">--</span></div>
  <div class="telem-row"><label>GPS Fix</label><span class="val" id="t-fix">--</span></div>
  <div class="telem-row"><label>Lat / Lon</label><span class="val" id="t-latlon">--</span></div>
  <div class="telem-row"><label>Sats / HDOP</label><span class="val" id="t-sats">--</span></div>
  <div class="telem-row"><label>Speed / Course</label><span class="val" id="t-speed">--</span></div>
</div>

<div class="card" id="bridge-card">
  <div class="card-title">Bridge (S3) <span class="pill" id="bridge-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>Link (uplink RSSI)</label><span class="val" id="b-link">--</span></div>
  <div class="telem-row"><label>LR mode (ground)</label><span class="val" id="b-lr">--</span></div>
  <div class="telem-row"><label>ESP-NOW pkts</label><span class="val" id="b-pkts">--</span></div>
  <div class="telem-row"><label>Frames fwd'd</label><span class="val" id="b-frames">--</span></div>
  <div class="telem-row"><label>Hello sent</label><span class="val" id="b-hello">--</span></div>
  <div class="telem-row"><label>Reasm drops</label><span class="val" id="b-drops">--</span></div>
```

- [ ] **Step 3: Render both rows in `applyStatus()`**

Find:
```javascript
    $('t-heading').textContent = t.have ? `${t.heading.toFixed(1)}°` : '--';
```
Replace with:
```javascript
    const linkEl = $('t-link');
    if (s.link_quality === 'unknown') {
      linkEl.textContent = '--';
      linkEl.className = 'val';
    } else {
      const ageTxt = t.cmd_age_ms < 0 ? 'never' : `${t.cmd_age_ms}ms ago`;
      const rssiTxt = t.cmd_rx_rssi ? `, ${t.cmd_rx_rssi}dBm` : '';
      linkEl.textContent = `${s.link_quality.toUpperCase()} (cmd ${ageTxt}${rssiTxt})`;
      linkEl.className = 'val link-' + s.link_quality;
    }
    // lr_active: 1 = C6 confirmed esp_now_set_peer_rate_config(LR) succeeded,
    // 0 = it failed, -1 = not yet known (LR disabled on the boat's build, or
    // no telemetry yet). See Task 3c / espnow_telemetry_t.lr_active.
    const tlrEl = $('t-lr');
    if (!t.have || t.lr_active === undefined || t.lr_active < 0) {
      tlrEl.textContent = '--';
      tlrEl.className = 'val';
    } else {
      tlrEl.textContent = t.lr_active ? 'ON' : 'FAILED';
      tlrEl.className = 'val ' + (t.lr_active ? 'link-green' : 'link-red');
    }
    $('t-heading').textContent = t.have ? `${t.heading.toFixed(1)}°` : '--';
```

Find:
```javascript
    $('b-pkts').textContent = b.have ? `${b.espnow_pkts} (${b.espnow_bytes} B)` : '--';
```
Replace with:
```javascript
    const blinkEl = $('b-link');
    blinkEl.textContent = (b.have && b.last_rx_rssi) ? `${b.last_rx_rssi} dBm` : '--';
    // lr_rate_config_ok: this bridge's OWN esp_now_set_peer_rate_config()
    // result for the broadcast peer (Task 2/12) -- the ground-side
    // counterpart to t-lr above (the boat's/C6's result).
    const blrEl = $('b-lr');
    if (!b.have || b.lr_rate_config_ok === undefined || b.lr_rate_config_ok < 0) {
      blrEl.textContent = '--';
      blrEl.className = 'val';
    } else {
      blrEl.textContent = b.lr_rate_config_ok ? 'ON' : 'FAILED';
      blrEl.className = 'val ' + (b.lr_rate_config_ok ? 'link-green' : 'link-red');
    }
    $('b-pkts').textContent = b.have ? `${b.espnow_pkts} (${b.espnow_bytes} B)` : '--';
```

- [ ] **Step 4: Add the three `link-*` CSS classes, reusing the existing `.warn`/`.pill.up` color scheme**

Find (inside the existing `<style>` block — locate the `.pill.up` / `.pill.stale` rules and add after them):
```
Run: grep -n '\.pill\.up\|\.pill\.stale\|\.warn' tools/espnow_drive.py
```
Read the 5 lines of context around that match, then add immediately after that rule block:
```css
    .link-green { color: var(--green); }
    .link-amber { color: #e0a030; }
    .link-red   { color: #e05050; font-weight: 600; }
```
(Match whichever of `var(--green)` / a literal hex the existing `.pill.up` rule already uses in this file, for consistency — copy its exact color value for `.link-green` rather than assuming `var(--green)` resolves correctly here.)

- [ ] **Step 5: Manual UI check**

Run: `.venv/bin/python tools/espnow_drive.py` and open the printed URL. Confirm the page loads without a JS console error and the new "Link" and "LR mode" rows all show `--` before any serial connection (no telemetry yet -> `unknown`/`-1` -> `--`, not a crash).

- [ ] **Step 6: Commit**

```bash
git add tools/espnow_drive.py
git commit -m "$(cat <<'EOF'
feat(control): show link-quality + LR status in the drive UI

Downlink (cmd age + RSSI, boat's view) on the telemetry card, uplink
(RSSI, ground's view) on the bridge card -- both directions now visible,
not just "did the last command arrive". Also surfaces each end's own
esp_now_set_peer_rate_config(LR) result (Task 3c/12) so LR success or
failure is visible without a UART tap on either radio.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 17: Part B build/flash verification

**Files:** none — hardware verification

- [ ] **Step 1: Rebuild all three firmwares**

Run: `idf.py build` (P4), `cd tools/slave_firmware && ./build.sh` (C6), `cd tools/espnow_bridge && idf.py build` (S3).
Expected: all three report success, matching Task 4's checks.

- [ ] **Step 2: Reflash all three**

Repeat Task 5 Steps 1–3.

- [ ] **Step 3: Confirm link-quality end to end**

Run `.venv/bin/python tools/espnow_drive.py`, connect, arm. Confirm:
- Telemetry card's "Link (downlink)" row shows `GREEN` with a small `cmd Xms ago` while actively sending.
- Bridge card's "Link (uplink RSSI)" row shows a real dBm number.
- Disconnecting the serial port drives `have_telemetry` stale within `TELEMETRY_STALE_S`, and the link row correctly falls back to `--`/`unknown` rather than freezing on a stale `GREEN`.
- Powering off the S3 while armed: within ~400ms the P4 failsafe fires (as in Task 5 Step 6) AND, once telemetry resumes, the UI's downlink row transitions GREEN → AMBER → RED in step with rising `cmd_age_ms`, then back to GREEN on reconnect.

No commit — verification checkpoint.

---

## Part C — Tuning knobs (documentation only)

### Task 18: Document the tunable constants; defer numeric changes

**Files:**
- Modify: `main/motor_control.c`

Retuning `CONTROL_LINK_TIMEOUT_US` against LR's real burst-loss characteristics needs actual field data from Task 6 and Task 17 — changing a safety-relevant failsafe timeout without that data would be a guess, not an improvement. This task only makes the two knobs (already-existing `CONTROL_LINK_TIMEOUT_US` here, and `SEND_HZ` in `espnow_drive.py`, changed in Task 13) explicitly cross-referenced so a future tuning pass finds both easily.

- [ ] **Step 1: Cross-reference the constant**

Find:
```c
/* How long a control link may go silent before it's treated as lost. Only
 * matters when no WS client is connected (see control_link_alive()) — field
```
Replace with:
```c
/* How long a control link may go silent before it's treated as lost. Only
 * matters when no WS client is connected (see control_link_alive()) — field
 *
 * TUNING: paired with SEND_HZ in tools/espnow_drive.py (currently 30, giving
 * ~12 packets of margin before this fires) and mirrored as
 * CONTROL_LINK_TIMEOUT_MS in that same file for the UI's link-quality
 * classification. If LR's real-world burst loss (see the Part A field test)
 * turns out to need a larger margin, or a tighter one is safe, change this
 * value and espnow_drive.py's copy together.
```

- [ ] **Step 2: Build**

Run: `idf.py build`
Expected: `Project build complete.`

- [ ] **Step 3: Commit**

```bash
git add main/motor_control.c
git commit -m "$(cat <<'EOF'
docs(espnow): cross-reference CONTROL_LINK_TIMEOUT_US with its Python mirror

No numeric change -- retuning needs real field-loss data from the LR range
test, not a guess. This just makes the two knobs easy to find together.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Summary

20 tasks, ~22 commits, sequenced so Part A (range) is fully bench- and field-verified in isolation before Part B (telemetry/UI) lands, per the design doc's own discipline. Tasks 3b and 3c were added mid-execution (not in the original design doc): 3b closes a shared-radio-state gap found while answering a question about WiFi-fallback safety after Task 1 landed; 3c closes an LR-observability gap Task 1's code-quality review flagged independently, which turned out to be the same root concern from a different angle. Part C is intentionally left as documentation only — the numeric tuning it points at needs the Part A/B field data this plan produces, not a guess made before that data exists.
