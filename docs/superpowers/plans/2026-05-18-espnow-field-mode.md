# ESP-NOW Field Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add ESP-NOW as an alternative transport for field testing — auto-detect WiFi vs ESP-NOW at boot, stream JPEG + telemetry through C6 coprocessor → ESP-NOW → S3 bridge → USB CDC → laptop.

**Architecture:** P4 sends data to C6 via `esp_hosted_send_custom_data()` (SDIO peer_data_transfer, 8166B max). C6 fragments into 244B ESP-NOW packets and sends over air. S3 bridge reassembles and forwards via USB CDC to laptop. Commands flow in reverse. Pipeline already supports multiple transports — `espnow_transport.c` plugs in alongside `ws_transport.c`.

**Tech Stack:** ESP-IDF v5.4, esp_hosted peer_data_transfer API, ESP-NOW, TinyUSB CDC (`espressif/esp_tinyusb` v2.2.0), COBS framing, nanopb protobuf.

**Spec:** `docs/superpowers/specs/2026-05-18-espnow-field-mode-design.md`

---

## File Structure

### P4 firmware (existing project — `main/`)

| Action | Path | Responsibility |
|--------|------|---------------|
| Create | `main/transports/espnow_protocol.h` | Shared packet header, msg types, COBS |
| Create | `main/transports/espnow_transport.h` | ESP-NOW transport public API |
| Create | `main/transports/espnow_transport.c` | P4↔C6 peer_data bridge, pipeline plugin |
| Modify | `main/main.c` | Auto-detect: WiFi fail → ESP-NOW fallback |
| Modify | `main/Kconfig.projbuild` | ESP-NOW field mode menu |
| Modify | `main/CMakeLists.txt` | Add espnow_transport.c to SRCS |
| Modify | `sdkconfig.defaults` | Enable peer_data_transfer, bump handler count |

### C6 slave firmware (`tools/slave_firmware/`)

| Action | Path | Responsibility |
|--------|------|---------------|
| Create | `tools/slave_firmware/espnow_bridge.c` | C6 ESP-NOW ↔ peer_data bridge |
| Create | `tools/slave_firmware/espnow_bridge.h` | Bridge public API |
| Modify | `tools/slave_firmware/build.sh` | Patch bridge into slave build |

### S3 bridge firmware (`tools/espnow_bridge/` — new project)

| Action | Path | Responsibility |
|--------|------|---------------|
| Create | `tools/espnow_bridge/CMakeLists.txt` | Top-level project CMake |
| Create | `tools/espnow_bridge/main/CMakeLists.txt` | Main component CMake |
| Create | `tools/espnow_bridge/main/idf_component.yml` | esp_tinyusb dependency |
| Create | `tools/espnow_bridge/main/main.c` | ESP-NOW ↔ USB CDC bridge |
| Create | `tools/espnow_bridge/main/cobs.h` | COBS encode/decode (inline) |
| Create | `tools/espnow_bridge/sdkconfig.defaults` | S3 target, USB CDC, ESP-NOW config |
| Create | `tools/espnow_bridge/README.md` | Build & flash instructions |

### Python tooling

| Action | Path | Responsibility |
|--------|------|---------------|
| Create | `tools/extract_overlay_params.py` | Extract projection constants from dashboard.html → JSON |
| Modify | `visualize.py` | Add `--serial` mode with COBS, JPEG reassembly, overlay |

---

## Task 1: Shared Protocol Header

**Files:**
- Create: `main/transports/espnow_protocol.h`

This header defines the packet format and COBS codec used by P4, C6, S3, and Python. All four codebases include or replicate these definitions.

- [ ] **Step 1: Create the protocol header**

```c
// main/transports/espnow_protocol.h
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ---- Message types ----
enum espnow_msg_type {
    MSG_JPEG_CHUNK     = 0x01,
    MSG_SENSOR         = 0x02,
    MSG_MOTOR_CMD      = 0x03,
    MSG_ARM_CMD        = 0x04,
    MSG_DETECT_CMD     = 0x05,
    MSG_MOTOR_STATUS   = 0x06,
    MSG_ESPNOW_INIT    = 0x10,
    MSG_ESPNOW_CONFIG  = 0x11,
};

// ---- Packet header (4 bytes, packed) ----
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;
    uint16_t payload_len;
    uint8_t  seq;
} espnow_pkt_hdr_t;

#define ESPNOW_HDR_SIZE sizeof(espnow_pkt_hdr_t)

// ---- ESP-NOW fragmentation (ESPNowCam-style) ----
// Intermediate chunk: total_len=0. Last chunk: total_len=frame_size.
typedef struct __attribute__((packed)) {
    uint32_t total_len;
} espnow_frag_hdr_t;

#define ESPNOW_MAX_PAYLOAD  244  // 250 - 6 bytes ESP-NOW overhead
#define ESPNOW_FRAG_PAYLOAD (ESPNOW_MAX_PAYLOAD - sizeof(espnow_frag_hdr_t))  // 240

// ---- COBS encode/decode (inline, ~30 lines each) ----

// Encode src[len] into dst. dst must be at least len + len/254 + 2 bytes.
// Returns encoded length (not including trailing 0x00 delimiter).
static inline size_t cobs_encode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t read_idx = 0, write_idx = 1, code_idx = 0;
    uint8_t code = 1;
    while (read_idx < len) {
        if (src[read_idx] == 0) {
            dst[code_idx] = code;
            code = 1;
            code_idx = write_idx++;
        } else {
            dst[write_idx++] = src[read_idx];
            code++;
            if (code == 0xFF) {
                dst[code_idx] = code;
                code = 1;
                code_idx = write_idx++;
            }
        }
        read_idx++;
    }
    dst[code_idx] = code;
    return write_idx;
}

// Decode COBS-encoded src[len] into dst. Returns decoded length, 0 on error.
static inline size_t cobs_decode(const uint8_t *src, size_t len, uint8_t *dst)
{
    size_t read_idx = 0, write_idx = 0;
    while (read_idx < len) {
        uint8_t code = src[read_idx++];
        if (code == 0) return 0; // unexpected zero
        for (uint8_t i = 1; i < code; i++) {
            if (read_idx >= len) return 0;
            dst[write_idx++] = src[read_idx++];
        }
        if (code < 0xFF && read_idx < len) {
            dst[write_idx++] = 0;
        }
    }
    if (write_idx > 0 && dst[write_idx - 1] == 0) write_idx--; // trim trailing zero
    return write_idx;
}
```

- [ ] **Step 2: Verify it compiles**

Run: `idf.py build 2>&1 | tail -5`

Header is not yet included by any .c file, so this just checks for syntax errors if included. We'll verify in Task 3.

- [ ] **Step 3: Commit**

```bash
git add main/transports/espnow_protocol.h
git commit -m "feat(espnow): add shared protocol header with packet types and COBS codec"
```

---

## Task 2: P4 Kconfig and Build Config

**Files:**
- Modify: `main/Kconfig.projbuild`
- Modify: `sdkconfig.defaults`
- Modify: `main/CMakeLists.txt`

- [ ] **Step 1: Add ESP-NOW menu to Kconfig.projbuild**

Append after the last `endmenu` in `main/Kconfig.projbuild`:

```kconfig
menu "ESP-NOW Field Mode"

    config ESPNOW_ENABLED
        bool "Enable ESP-NOW field mode"
        default y

    config ESPNOW_WIFI_TIMEOUT_S
        int "WiFi connect timeout before ESP-NOW fallback (seconds)"
        default 10
        depends on ESPNOW_ENABLED

    config ESPNOW_JPEG_QUALITY
        int "JPEG quality for ESP-NOW (1-63, lower=smaller)"
        default 12
        depends on ESPNOW_ENABLED
        help
            Controls JPEG compression for ESP-NOW streaming.
            Lower value = smaller frames = higher FPS but worse quality.
            At quality 12, native 800x640 frames are ~15-25KB giving 3-5 fps.

    config ESPNOW_CHANNEL
        int "ESP-NOW WiFi channel"
        default 6
        range 1 14
        depends on ESPNOW_ENABLED

    config ESPNOW_PEER_MAC
        string "ESP32-S3 bridge MAC address (hex, colon-separated)"
        default "FF:FF:FF:FF:FF:FF"
        depends on ESPNOW_ENABLED

endmenu
```

- [ ] **Step 2: Add sdkconfig.defaults entries**

Append to `sdkconfig.defaults`:

```
# --- ESP-NOW field mode ---
CONFIG_ESPNOW_ENABLED=y
CONFIG_ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS=8
```

- [ ] **Step 3: Add espnow_transport.c to CMakeLists.txt SRCS**

In `main/CMakeLists.txt`, add `"transports/espnow_transport.c"` to the SRCS list (after `"transports/ws_transport.c"`).

- [ ] **Step 4: Build to verify Kconfig parses**

Run: `idf.py build 2>&1 | tail -10`

Expected: build succeeds (espnow_transport.c doesn't exist yet — add an empty file or skip this build check until Task 3).

- [ ] **Step 5: Commit**

```bash
git add main/Kconfig.projbuild sdkconfig.defaults main/CMakeLists.txt
git commit -m "feat(espnow): add Kconfig menu and build config for ESP-NOW field mode"
```

---

## Task 3: P4 ESP-NOW Transport

**Files:**
- Create: `main/transports/espnow_transport.h`
- Create: `main/transports/espnow_transport.c`

This is the core P4-side transport that plugs into the pipeline. It sends data to the C6 via `esp_hosted_send_custom_data()` and receives commands back via registered callbacks.

- [ ] **Step 1: Create the header**

```c
// main/transports/espnow_transport.h
#pragma once
#include "esp_err.h"

/**
 * @brief Initialize ESP-NOW transport via esp_hosted peer_data_transfer.
 *        Sends MSG_ESPNOW_INIT to C6, registers with pipeline.
 *        Call after pipeline_init(). Do NOT call if WiFi mode is active.
 */
esp_err_t espnow_transport_init(void);
```

- [ ] **Step 2: Create the implementation**

```c
// main/transports/espnow_transport.c
#include "espnow_transport.h"
#include "espnow_protocol.h"
#include "pipeline.h"
#include "esp_log.h"
#include "esp_hosted_misc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "proto/boat.pb.h"

static const char *TAG = "ESPNOW_TX";

// ---- msg_ids for peer_data_transfer ----
// These are the esp_hosted custom_data msg_ids (uint32_t), NOT the espnow_msg_type.
// P4→C6 direction:
#define PEER_MSG_VIDEO     1   // JPEG chunks + sensor data (queued on C6)
#define PEER_MSG_COMMAND   2   // motor/arm/detect commands FROM laptop (fast path on C6)
#define PEER_MSG_INIT      3   // ESP-NOW init command
// C6→P4 direction:
#define PEER_MSG_UPSTREAM  4   // commands received from laptop via ESP-NOW

#define PEER_DATA_MAX  8166

// ---- Pipeline send callback ----
// Called by pipeline_publish_sensors() and pipeline_publish_motor_status()
// with nanopb-encoded BoatMessage bytes.
static esp_err_t espnow_send_fn(const uint8_t *buf, size_t len, void *ctx)
{
    if (len == 0 || len > PEER_DATA_MAX - ESPNOW_HDR_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Wrap with our packet header
    static uint8_t pkt[PEER_DATA_MAX];
    espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)pkt;
    hdr->msg_type = MSG_SENSOR;
    hdr->payload_len = len;
    hdr->seq = 0;
    memcpy(pkt + ESPNOW_HDR_SIZE, buf, len);

    return esp_hosted_send_custom_data(PEER_MSG_VIDEO, pkt, ESPNOW_HDR_SIZE + len);
}

// ---- JPEG send (called from camera/sensor task) ----
// Fragments JPEG into ≤PEER_DATA_MAX chunks.
esp_err_t espnow_transport_send_jpeg(const uint8_t *jpg, size_t len)
{
    static uint8_t frame_seq = 0;
    frame_seq++;

    size_t max_chunk = PEER_DATA_MAX - ESPNOW_HDR_SIZE;
    size_t offset = 0;

    while (offset < len) {
        size_t chunk_len = len - offset;
        if (chunk_len > max_chunk) chunk_len = max_chunk;

        static uint8_t pkt[PEER_DATA_MAX];
        espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)pkt;
        hdr->msg_type = MSG_JPEG_CHUNK;
        hdr->payload_len = chunk_len;
        // seq: high nibble = frame counter, low nibble = chunk index within frame
        // last chunk signaled by payload_len < max_chunk
        hdr->seq = frame_seq;
        memcpy(pkt + ESPNOW_HDR_SIZE, jpg + offset, chunk_len);

        esp_err_t ret = esp_hosted_send_custom_data(PEER_MSG_VIDEO, pkt, ESPNOW_HDR_SIZE + chunk_len);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "JPEG chunk send failed at offset %u: %s", (unsigned)offset, esp_err_to_name(ret));
            return ret;
        }
        offset += chunk_len;
    }
    return ESP_OK;
}

// ---- Upstream callback (commands from laptop via C6) ----
static void upstream_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    if (data_len < ESPNOW_HDR_SIZE) return;

    const espnow_pkt_hdr_t *hdr = (const espnow_pkt_hdr_t *)data;
    const uint8_t *payload = data + ESPNOW_HDR_SIZE;
    size_t payload_len = hdr->payload_len;

    if (ESPNOW_HDR_SIZE + payload_len > data_len) return;

    switch (hdr->msg_type) {
    case MSG_MOTOR_CMD:
    case MSG_ARM_CMD:
    case MSG_DETECT_CMD:
        // Pipeline handles protobuf decode and dispatch to registered handlers
        pipeline_handle_incoming(payload, payload_len);
        break;
    default:
        ESP_LOGW(TAG, "Unknown upstream msg_type: 0x%02x", hdr->msg_type);
        break;
    }
}

// ---- Parse MAC string "AA:BB:CC:DD:EE:FF" → 6 bytes ----
static void parse_mac_string(const char *str, uint8_t *mac)
{
    unsigned int m[6];
    if (sscanf(str, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
        for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
    } else {
        memset(mac, 0xFF, 6); // broadcast fallback
    }
}

// ---- Init ----
esp_err_t espnow_transport_init(void)
{
    ESP_LOGI(TAG, "Initializing ESP-NOW transport (peer_data_transfer)...");

    // Register upstream callback for commands from laptop
    esp_err_t ret = esp_hosted_register_custom_callback(PEER_MSG_UPSTREAM, upstream_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register upstream callback: %s", esp_err_to_name(ret));
        return ret;
    }

    // Send ESP-NOW init command to C6
    // Payload: channel (1 byte) + peer MAC (6 bytes)
    uint8_t init_payload[ESPNOW_HDR_SIZE + 7];
    espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)init_payload;
    hdr->msg_type = MSG_ESPNOW_INIT;
    hdr->payload_len = 7;
    hdr->seq = 0;
    init_payload[ESPNOW_HDR_SIZE] = (uint8_t)CONFIG_ESPNOW_CHANNEL;
    parse_mac_string(CONFIG_ESPNOW_PEER_MAC, &init_payload[ESPNOW_HDR_SIZE + 1]);

    ret = esp_hosted_send_custom_data(PEER_MSG_INIT, init_payload, sizeof(init_payload));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send ESPNOW_INIT to C6: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "ESP-NOW init sent to C6 (channel=%d)", CONFIG_ESPNOW_CHANNEL);

    // Send config message (resolution) — will be forwarded to laptop
    uint8_t cfg_payload[ESPNOW_HDR_SIZE + 4];
    espnow_pkt_hdr_t *cfg_hdr = (espnow_pkt_hdr_t *)cfg_payload;
    cfg_hdr->msg_type = MSG_ESPNOW_CONFIG;
    cfg_hdr->payload_len = 4;
    cfg_hdr->seq = 0;
    uint16_t w = 800, h = 640;
    memcpy(&cfg_payload[ESPNOW_HDR_SIZE], &w, 2);
    memcpy(&cfg_payload[ESPNOW_HDR_SIZE + 2], &h, 2);
    esp_hosted_send_custom_data(PEER_MSG_VIDEO, cfg_payload, sizeof(cfg_payload));

    // Register with pipeline as transport
    ret = pipeline_register_transport(espnow_send_fn, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register with pipeline: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW transport ready");
    return ESP_OK;
}
```

- [ ] **Step 3: Build**

Run: `idf.py build 2>&1 | tail -10`

Expected: compiles. If `esp_hosted_misc.h` is not found, check include path — it's at `managed_components/espressif__esp_hosted/host/esp_hosted_misc.h`. May need to add `espressif__esp_hosted` to `PRIV_REQUIRES` in `main/CMakeLists.txt`.

- [ ] **Step 4: Commit**

```bash
git add main/transports/espnow_transport.h main/transports/espnow_transport.c
git commit -m "feat(espnow): add P4 ESP-NOW transport via peer_data_transfer"
```

---

## Task 4: P4 Auto-Detect in main.c

**Files:**
- Modify: `main/main.c`

- [ ] **Step 1: Add include and modify WiFi fallback block**

At the top of `main.c`, add:
```c
#include "transports/espnow_transport.h"
```

Replace the WiFi failure block (around line 180):

```c
    // 10. Connect to WiFi (blocks until connected or timeout)
    ESP_LOGI(TAG, "Connecting to WiFi...");
    esp_err_t wifi_ret = wifi_init();

#if CONFIG_ESPNOW_ENABLED
    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi unavailable — switching to ESP-NOW field mode");
        esp_err_t en_ret = espnow_transport_init();
        if (en_ret != ESP_OK) {
            ESP_LOGE(TAG, "ESP-NOW init failed (%s) — no connectivity", esp_err_to_name(en_ret));
        }
    } else {
#else
    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi unavailable (%s) — camera stream disabled", esp_err_to_name(wifi_ret));
    } else {
#endif
        // 11. Start servers — stream on port 81, API/dashboard on port 80
        if (camera_ok) {
            ESP_ERROR_CHECK(camera_stream_server_start());
        }
        ESP_ERROR_CHECK(http_server_start());

        // 11b. Arm ESCs
        ESP_LOGI(TAG, "Arming ESCs...");
        esp_err_t esc_ret = motor_control_arm();
        if (esc_ret != ESP_OK) {
            ESP_LOGW(TAG, "ESC arming failed (%s) — arm via dashboard later",
                     esp_err_to_name(esc_ret));
        }
    }
```

- [ ] **Step 2: Build**

Run: `idf.py build 2>&1 | tail -10`

Expected: compiles successfully.

- [ ] **Step 3: Commit**

```bash
git add main/main.c
git commit -m "feat(espnow): add auto-detect WiFi/ESP-NOW fallback in app_main"
```

---

## Task 5: C6 ESP-NOW Bridge

**Files:**
- Create: `tools/slave_firmware/espnow_bridge.h`
- Create: `tools/slave_firmware/espnow_bridge.c`

The C6 receives data from P4 via `esp_hosted_register_custom_callback()`, fragments into ESP-NOW packets, and sends. Incoming ESP-NOW data is forwarded back to P4.

- [ ] **Step 1: Create bridge header**

```c
// tools/slave_firmware/espnow_bridge.h
#pragma once
#include "esp_err.h"

/**
 * @brief Register peer_data callbacks for ESP-NOW bridge.
 *        Call from slave app_main or example init.
 *        ESP-NOW is NOT initialized here — it starts when P4 sends MSG_ESPNOW_INIT.
 */
esp_err_t espnow_bridge_init(void);
```

- [ ] **Step 2: Create bridge implementation**

```c
// tools/slave_firmware/espnow_bridge.c
#include "espnow_bridge.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_hosted_peer_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "ESPNOW_BRIDGE";

// ---- Constants (must match P4 espnow_protocol.h) ----
#define MSG_ESPNOW_INIT  0x10
#define MSG_JPEG_CHUNK   0x01
#define MSG_SENSOR       0x02
#define MSG_MOTOR_CMD    0x03
#define MSG_ARM_CMD      0x04
#define MSG_MOTOR_STATUS 0x06
#define ESPNOW_HDR_SIZE  4

// peer_data msg_ids (must match P4)
#define PEER_MSG_VIDEO     1
#define PEER_MSG_COMMAND   2
#define PEER_MSG_INIT      3
#define PEER_MSG_UPSTREAM  4

// ESP-NOW fragmentation
#define ESPNOW_MAX_PAYLOAD 244
#define FRAG_HDR_SIZE      4  // uint32_t total_len
#define FRAG_DATA_SIZE     (ESPNOW_MAX_PAYLOAD - FRAG_HDR_SIZE)  // 240

static uint8_t s_peer_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static bool s_espnow_ready = false;

// ---- TX queue for video (slow path) ----
typedef struct {
    uint8_t *data;
    size_t len;
} tx_item_t;

static QueueHandle_t s_tx_queue = NULL;
static TaskHandle_t s_tx_task = NULL;

// ---- ESP-NOW fragment and send ----
static esp_err_t espnow_frag_send(const uint8_t *data, size_t len)
{
    uint8_t pkt[ESPNOW_MAX_PAYLOAD];
    size_t offset = 0;

    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > FRAG_DATA_SIZE) chunk = FRAG_DATA_SIZE;

        uint32_t total_len = 0;
        bool is_last = (offset + chunk >= len);
        if (is_last) total_len = (uint32_t)len;

        memcpy(pkt, &total_len, FRAG_HDR_SIZE);
        memcpy(pkt + FRAG_HDR_SIZE, data + offset, chunk);

        esp_err_t ret = esp_now_send(s_peer_mac, pkt, FRAG_HDR_SIZE + chunk);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(ret));
            return ret;
        }
        // Busy-wait for send callback (ESPNowCam pattern)
        vTaskDelay(pdMS_TO_TICKS(1));

        offset += chunk;
    }
    return ESP_OK;
}

// ---- TX task: drains video queue, fragments, sends ----
static void espnow_tx_task(void *arg)
{
    tx_item_t item;
    while (1) {
        if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            espnow_frag_send(item.data, item.len);
            free(item.data);
        }
    }
}

// ---- Callback: video data from P4 (queue to slow task) ----
static void video_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    if (!s_espnow_ready || data_len == 0) return;

    uint8_t *copy = malloc(data_len);
    if (!copy) {
        ESP_LOGW(TAG, "OOM for video packet (%u bytes)", (unsigned)data_len);
        return;
    }
    memcpy(copy, data, data_len);

    tx_item_t item = { .data = copy, .len = data_len };
    if (xQueueSend(s_tx_queue, &item, 0) != pdTRUE) {
        free(copy);
        ESP_LOGW(TAG, "TX queue full, dropping video packet");
    }
}

// ---- Callback: command data from P4 (fast path, direct send) ----
static void command_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    if (!s_espnow_ready || data_len == 0) return;
    // Commands are small (<250B) — send as single ESP-NOW packet, no fragmentation
    if (data_len <= ESPNOW_MAX_PAYLOAD) {
        esp_now_send(s_peer_mac, data, data_len);
    }
}

// ---- Callback: ESP-NOW init from P4 ----
static void init_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    if (data_len < ESPNOW_HDR_SIZE + 7) {
        ESP_LOGE(TAG, "ESPNOW_INIT payload too short (%u)", (unsigned)data_len);
        return;
    }

    uint8_t channel = data[ESPNOW_HDR_SIZE];
    uint8_t *mac = (uint8_t *)&data[ESPNOW_HDR_SIZE + 1];
    memcpy(s_peer_mac, mac, 6);

    ESP_LOGI(TAG, "ESP-NOW init: channel=%d peer=%02x:%02x:%02x:%02x:%02x:%02x",
             channel, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Set WiFi channel (WiFi stack is already initialized by hosted)
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    // Init ESP-NOW
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Add peer
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_peer_mac, 6);
    peer.channel = channel;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;

    ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register recv callback
    esp_now_register_recv_cb(espnow_recv_cb);

    s_espnow_ready = true;
    ESP_LOGI(TAG, "ESP-NOW ready");
}

// ---- ESP-NOW receive callback → forward to P4 ----
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len <= 0) return;
    // Forward raw data to P4 as upstream command
    esp_hosted_send_custom_data(PEER_MSG_UPSTREAM, data, (size_t)len);
}

// ---- Public init ----
esp_err_t espnow_bridge_init(void)
{
    ESP_LOGI(TAG, "Registering ESP-NOW bridge callbacks...");

    s_tx_queue = xQueueCreate(8, sizeof(tx_item_t));
    if (!s_tx_queue) return ESP_ERR_NO_MEM;

    xTaskCreate(espnow_tx_task, "espnow_tx", 4096, NULL, 3, &s_tx_task);

    ESP_ERROR_CHECK(esp_hosted_register_custom_callback(PEER_MSG_VIDEO, video_cb));
    ESP_ERROR_CHECK(esp_hosted_register_custom_callback(PEER_MSG_COMMAND, command_cb));
    ESP_ERROR_CHECK(esp_hosted_register_custom_callback(PEER_MSG_INIT, init_cb));

    ESP_LOGI(TAG, "ESP-NOW bridge ready (waiting for INIT from host)");
    return ESP_OK;
}
```

Note: The `espnow_recv_cb` forward declaration needs to be above `init_cb`. Move the function or add a forward declaration at the top of the file.

- [ ] **Step 3: Commit**

```bash
git add tools/slave_firmware/espnow_bridge.h tools/slave_firmware/espnow_bridge.c
git commit -m "feat(espnow): add C6 ESP-NOW bridge for slave firmware"
```

---

## Task 6: C6 Slave Firmware Build Script

**Files:**
- Modify: `tools/slave_firmware/build.sh`

The existing `build.sh` creates a slave project from esp_hosted example. We need to patch in our `espnow_bridge.c` and configure Kconfig.

- [ ] **Step 1: Update build.sh**

Replace `tools/slave_firmware/build.sh` content. The key additions after `idf.py create-project-from-example`:

```bash
#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build_slave"
VERSION="${1:-2.12.3}"

echo "=== Building ESP-Hosted slave firmware v${VERSION} for ESP32-C6 ==="
echo "=== WITH ESP-NOW bridge ==="

# Create temp project
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

idf.py create-project-from-example "espressif/esp_hosted=${VERSION}:slave"
cd slave

# Configure for C6 with SDIO
idf.py set-target esp32c6

# Patch in ESP-NOW bridge
cp "$SCRIPT_DIR/espnow_bridge.c" main/
cp "$SCRIPT_DIR/espnow_bridge.h" main/

# Add bridge to slave CMakeLists.txt SRCS
# The slave main/CMakeLists.txt lists SRCS — append our file
sed -i '/SRCS/,/)/{ /)/i\        "espnow_bridge.c"
}' main/CMakeLists.txt

# Add esp_now to PRIV_REQUIRES if not already there
sed -i 's/PRIV_REQUIRES/PRIV_REQUIRES esp_wifi/' main/CMakeLists.txt 2>/dev/null || true

# sdkconfig overrides
cat >> sdkconfig.defaults << 'SDKEOF'
CONFIG_ESP_SDIO_HOST_INTERFACE=y
CONFIG_ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS=8
CONFIG_ESPNOW_ENABLED=y
SDKEOF

# Add call to espnow_bridge_init() in slave app_main
# The slave has CONFIG_ESP_HOSTED_COPROCESSOR_APP_MAIN controlling app_main.
# We need to hook into slave init. Simplest: add to the example init area.
# This may need manual adjustment depending on the exact slave template.

idf.py build

echo ""
echo "=== Build complete ==="
echo "Binaries:"
echo "  bootloader:      $BUILD_DIR/slave/build/bootloader/bootloader.bin"
echo "  partition-table:  $BUILD_DIR/slave/build/partition_table/partition-table.bin"
echo "  app:              $BUILD_DIR/slave/build/network_adapter.bin"
echo ""
echo "To copy to SDIO flasher:"
echo "  cp build/bootloader/bootloader.bin       $SCRIPT_DIR/../sdio_flasher/target-firmware/"
echo "  cp build/partition_table/partition-table.bin $SCRIPT_DIR/../sdio_flasher/target-firmware/"
echo "  cp build/network_adapter.bin             $SCRIPT_DIR/../sdio_flasher/target-firmware/app.bin"
```

Note: The `sed` command to inject into CMakeLists.txt is fragile — verify the slave template format when building. May need manual fixup.

- [ ] **Step 2: Commit**

```bash
git add tools/slave_firmware/build.sh
git commit -m "feat(espnow): update C6 slave build script to include ESP-NOW bridge"
```

---

## Task 7: S3 Bridge Firmware

**Files:**
- Create: `tools/espnow_bridge/CMakeLists.txt`
- Create: `tools/espnow_bridge/main/CMakeLists.txt`
- Create: `tools/espnow_bridge/main/idf_component.yml`
- Create: `tools/espnow_bridge/main/main.c`
- Create: `tools/espnow_bridge/main/cobs.h`
- Create: `tools/espnow_bridge/sdkconfig.defaults`
- Create: `tools/espnow_bridge/README.md`

This is a standalone ESP-IDF project for the ESP32-S3. It bridges ESP-NOW ↔ USB CDC.

- [ ] **Step 1: Create project skeleton**

`tools/espnow_bridge/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(espnow_bridge)
```

`tools/espnow_bridge/main/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_wifi esp_event nvs_flash esp_timer)
```

`tools/espnow_bridge/main/idf_component.yml`:
```yaml
dependencies:
  espressif/esp_tinyusb: "^2.2.0"
  idf: ">=5.3"
```

`tools/espnow_bridge/sdkconfig.defaults`:
```
CONFIG_IDF_TARGET="esp32s3"
CONFIG_TINYUSB_CDC_ENABLED=y
CONFIG_TINYUSB_CDC_RX_BUFSIZE=512
CONFIG_TINYUSB_CDC_TX_BUFSIZE=512
CONFIG_ESP_WIFI_PS_NONE=y
```

- [ ] **Step 2: Create COBS header (copy from espnow_protocol.h)**

`tools/espnow_bridge/main/cobs.h`: Copy just the `cobs_encode()` and `cobs_decode()` inline functions from `main/transports/espnow_protocol.h`. Same code, standalone header for the S3 project.

- [ ] **Step 3: Create main.c**

```c
// tools/espnow_bridge/main/main.c
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "cobs.h"

static const char *TAG = "BRIDGE";

// ---- Config (must match P4 Kconfig) ----
#define ESPNOW_CHANNEL 6
// C6 peer MAC — set this to your C6's STA MAC address
static uint8_t s_c6_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---- ESP-NOW fragmentation reassembly ----
#define FRAG_HDR_SIZE 4
#define REASSEMBLY_BUF_SIZE (64 * 1024)
static uint8_t s_reassembly_buf[REASSEMBLY_BUF_SIZE];
static size_t s_reassembly_pos = 0;

// ---- USB TX queue ----
typedef struct {
    uint8_t *data;
    size_t len;
} usb_tx_item_t;

static QueueHandle_t s_usb_tx_queue = NULL;

// ---- USB CDC RX buffer ----
#define USB_RX_BUF_SIZE 4096
static uint8_t s_usb_rx_buf[USB_RX_BUF_SIZE];
static size_t s_usb_rx_pos = 0;

// ---- Forward decl ----
static void usb_tx_task(void *arg);
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);

// ---- Send data to laptop via USB CDC (COBS-framed) ----
static void send_to_usb(const uint8_t *data, size_t len)
{
    // COBS encode: max overhead = len/254 + 2
    size_t max_enc = len + (len / 254) + 2;
    uint8_t *enc = malloc(max_enc + 1); // +1 for delimiter
    if (!enc) return;

    size_t enc_len = cobs_encode(data, len, enc);
    enc[enc_len] = 0x00; // COBS delimiter

    usb_tx_item_t item = { .data = enc, .len = enc_len + 1 };
    if (xQueueSend(s_usb_tx_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        free(enc);
    }
}

// ---- ESP-NOW recv: reassemble fragments, forward to USB ----
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < FRAG_HDR_SIZE) return;

    uint32_t total_len;
    memcpy(&total_len, data, FRAG_HDR_SIZE);
    size_t payload_len = len - FRAG_HDR_SIZE;
    const uint8_t *payload = data + FRAG_HDR_SIZE;

    if (s_reassembly_pos + payload_len > REASSEMBLY_BUF_SIZE) {
        ESP_LOGW(TAG, "Reassembly overflow, resetting");
        s_reassembly_pos = 0;
        return;
    }

    memcpy(s_reassembly_buf + s_reassembly_pos, payload, payload_len);
    s_reassembly_pos += payload_len;

    if (total_len > 0) {
        // Last chunk — send complete message to USB
        send_to_usb(s_reassembly_buf, s_reassembly_pos);
        s_reassembly_pos = 0;
    }
}

// ---- USB CDC RX callback ----
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    uint8_t buf[512];
    size_t rx_size = 0;
    if (tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size) != ESP_OK) return;

    // Accumulate and look for COBS delimiter (0x00)
    for (size_t i = 0; i < rx_size; i++) {
        if (buf[i] == 0x00) {
            // Decode COBS packet
            if (s_usb_rx_pos > 0) {
                uint8_t decoded[USB_RX_BUF_SIZE];
                size_t dec_len = cobs_decode(s_usb_rx_buf, s_usb_rx_pos, decoded);
                if (dec_len > 0) {
                    // Send as single ESP-NOW packet to C6 (commands are small)
                    if (dec_len <= 250) {
                        esp_now_send(s_c6_mac, decoded, dec_len);
                    }
                }
                s_usb_rx_pos = 0;
            }
        } else {
            if (s_usb_rx_pos < USB_RX_BUF_SIZE) {
                s_usb_rx_buf[s_usb_rx_pos++] = buf[i];
            }
        }
    }
}

// ---- USB TX task ----
static void usb_tx_task(void *arg)
{
    usb_tx_item_t item;
    while (1) {
        if (xQueueReceive(s_usb_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, item.data, item.len);
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(100));
            free(item.data);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP-NOW ↔ USB CDC Bridge ===");

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // WiFi (STA mode, for ESP-NOW)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_c6_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // USB CDC
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = tinyusb_cdc_rx_callback,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    // TX queue and task
    s_usb_tx_queue = xQueueCreate(16, sizeof(usb_tx_item_t));
    xTaskCreate(usb_tx_task, "usb_tx", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "Bridge ready. USB CDC → ESP-NOW ch%d", ESPNOW_CHANNEL);
}
```

- [ ] **Step 4: Create README**

`tools/espnow_bridge/README.md`:
```markdown
# ESP-NOW ↔ USB CDC Bridge (ESP32-S3)

Bridges ESP-NOW radio traffic to USB CDC serial for laptop communication.

## Build

```bash
cd tools/espnow_bridge
idf.py set-target esp32s3
idf.py build
idf.py flash
```

## Configuration

Before building, edit `main/main.c`:
- `ESPNOW_CHANNEL` — must match P4 Kconfig `CONFIG_ESPNOW_CHANNEL`
- `s_c6_mac` — set to your C6 coprocessor's STA MAC address

## Usage

After flashing, the S3 appears as `/dev/ttyACM0` on Linux.
Run: `python visualize.py --serial /dev/ttyACM0`
```

- [ ] **Step 5: Commit**

```bash
git add tools/espnow_bridge/
git commit -m "feat(espnow): add S3 ESP-NOW ↔ USB CDC bridge firmware"
```

---

## Task 8: Overlay Params Extraction Script

**Files:**
- Create: `tools/extract_overlay_params.py`

Extracts projection constants from `dashboard.html` JS source into `overlay_params.json` for `visualize.py`.

- [ ] **Step 1: Create the script**

```python
#!/usr/bin/env python3
"""Extract overlay projection constants from dashboard.html into JSON."""
import re, json, sys, os

def extract(html_path):
    with open(html_path, 'r') as f:
        html = f.read()

    def find_const(pattern, cast=float):
        m = re.search(pattern, html)
        return cast(m.group(1)) if m else None

    params = {
        "cam": {"w": 800, "h": 640},
        "fx": find_const(r"const\s+FX\s*=\s*([0-9.]+)"),
        "cx": find_const(r"const\s+CX\s*=\s*([0-9.]+)"),
        "cy": find_const(r"const\s+CY\s*=\s*([0-9.]+)"),
        "az_offset": find_const(r"let\s+AZ_OFFSET\s*=\s*([0-9.\-]+)"),
        "el_offset": find_const(r"let\s+EL_OFFSET\s*=\s*([0-9.\-]+)"),
        "sensor_tz": find_const(r"const\s+SENSOR_TZ\s*=\s*([0-9.]+)"),
        "zone_step": find_const(r"const\s+ZONE_STEP\s*=\s*([0-9.]+)"),
        "cam_scale": find_const(r"let\s+camScale\s*=\s*([0-9.]+)"),
    }

    # Sensor A/B positions
    m_a = re.search(r"SENSOR_A\s*=\s*\{\s*x:\s*([0-9.\-]+),\s*tilt:\s*([0-9.\-]+)", html)
    m_b = re.search(r"SENSOR_B\s*=\s*\{\s*x:\s*([0-9.\-]+),\s*tilt:\s*([0-9.\-]+)", html)
    params["sensor_a"] = {"x": float(m_a.group(1)), "tilt": float(m_a.group(2))} if m_a else None
    params["sensor_b"] = {"x": float(m_b.group(1)), "tilt": float(m_b.group(2))} if m_b else None

    # Flip flags
    params["flip"] = {
        "ha": "flipHA: true" in html,
        "va": "flipVA: true" in html,
        "ta": "transA: true" in html,
        "hb": "flipHB: true" in html,
        "vb": "flipVB: true" in html,
        "tb": "transB: true" in html,
    }

    # Drag offsets (default 0)
    params["drag"] = {"x": 0, "y": 0}

    return params

if __name__ == "__main__":
    html_path = sys.argv[1] if len(sys.argv) > 1 else "main/dashboard.html"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "overlay_params.json"

    params = extract(html_path)
    with open(out_path, 'w') as f:
        json.dump(params, f, indent=2)

    print(f"Extracted overlay params from {html_path} → {out_path}")
    print(json.dumps(params, indent=2))
```

- [ ] **Step 2: Run it**

```bash
cd /workspaces/BoatEspP4
python3 tools/extract_overlay_params.py main/dashboard.html overlay_params.json
```

Expected: prints JSON with FX=1242, CX=104, CY=199, etc.

- [ ] **Step 3: Commit**

```bash
git add tools/extract_overlay_params.py overlay_params.json
git commit -m "feat(espnow): add overlay params extraction script for visualize.py"
```

---

## Task 9: visualize.py Serial Mode

**Files:**
- Modify: `visualize.py`

Add `--serial` flag that reads COBS-framed data from USB CDC, reassembles JPEG, decodes protobuf telemetry, and renders the overlay using the same projection math as dashboard.html.

This is the largest single task. It adds:
1. COBS decode from serial
2. Packet parsing (espnow_pkt_hdr_t)
3. JPEG reassembly from chunks
4. Protobuf decode (using protobuf Python package with the same .proto schema)
5. Overlay rendering (port of JS projection math to Python/matplotlib)
6. Motor command input (keyboard → protobuf → COBS → serial TX)

Due to the size and the fact that visualize.py already has complex structure, this task should be implemented incrementally:

- [ ] **Step 1: Add argument parsing for --serial**

Add to the `argparse` section:
```python
parser.add_argument('--serial', type=str, default=None,
                    help='Serial port for ESP-NOW mode (e.g. /dev/ttyACM0)')
```

- [ ] **Step 2: Add COBS decode function**

```python
def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]; idx += 1
        if code == 0: return bytes()
        for i in range(1, code):
            if idx >= len(data): return bytes()
            out.append(data[idx]); idx += 1
        if code < 0xFF and idx < len(data):
            out.append(0)
    if out and out[-1] == 0: out = out[:-1]
    return bytes(out)

def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_idx = len(out); out.append(0); code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code; code = 1
            code_idx = len(out); out.append(0)
        else:
            out.append(b); code += 1
            if code == 0xFF:
                out[code_idx] = code; code = 1
                code_idx = len(out); out.append(0)
    out[code_idx] = code
    return bytes(out)
```

- [ ] **Step 3: Add serial reader thread**

```python
def serial_reader(port, baud=921600):
    """Read COBS-framed packets from serial, dispatch to handlers."""
    import serial
    ser = serial.Serial(port, baud, timeout=0.1)
    buf = bytearray()

    while True:
        chunk = ser.read(4096)
        if not chunk: continue
        buf.extend(chunk)

        while b'\x00' in buf:
            delim = buf.index(b'\x00')
            if delim > 0:
                decoded = cobs_decode(bytes(buf[:delim]))
                if len(decoded) >= 4:  # espnow_pkt_hdr_t minimum
                    handle_serial_packet(decoded)
            buf = buf[delim + 1:]
```

- [ ] **Step 4: Add packet handler and JPEG reassembly**

```python
import struct

jpeg_chunks = {}  # seq -> list of chunks
current_jpeg_seq = 0

def handle_serial_packet(data):
    global current_jpeg_seq
    msg_type, payload_len, seq = struct.unpack('<BHB', data[:4])
    payload = data[4:4 + payload_len]

    if msg_type == 0x01:  # MSG_JPEG_CHUNK
        if seq != current_jpeg_seq:
            jpeg_chunks.clear()
            current_jpeg_seq = seq
        jpeg_chunks.setdefault(seq, bytearray()).extend(payload)
        if payload_len < 8162:  # last chunk (smaller than max)
            display_jpeg(bytes(jpeg_chunks[seq]))
            jpeg_chunks.clear()

    elif msg_type == 0x02:  # MSG_SENSOR
        handle_sensor_snapshot(payload)

    elif msg_type == 0x06:  # MSG_MOTOR_STATUS
        handle_motor_status(payload)
```

- [ ] **Step 5: Add overlay rendering using overlay_params.json**

This ports the `projectZone()` math from dashboard.html JS to Python. Load params at startup:

```python
import json, math

def load_overlay_params(path='overlay_params.json'):
    with open(path) as f:
        return json.load(f)

def project_zone(row, col, sensor_key, mm, params, sf=1.0):
    """Port of dashboard.html projectZone() — 3D rotation projection."""
    az_offset = params['az_offset']
    el_offset = params['el_offset']
    zone_step = params['zone_step']
    fx = params['fx'] * sf
    cx = params['cx'] * sf
    cy = params['cy'] * sf
    tz = params['sensor_tz']

    sensor = params[f'sensor_{sensor_key.lower()}']
    sx = sensor['x']
    tilt = math.radians(sensor['tilt'])
    cos_t, sin_t = math.cos(tilt), math.sin(tilt)

    corners = []
    for dr, dc in [(-0.5,-0.5),(0.5,-0.5),(0.5,0.5),(-0.5,0.5)]:
        az = math.radians((col + dc - 3.5) * zone_step + az_offset)
        el = math.radians((3.5 - row - dr) * zone_step + el_offset)

        # Direction in sensor frame
        dx = math.tan(az)
        dy = -math.tan(el)
        dz = 1.0

        # Rotate around Y (sensor tilt)
        rx = dx * cos_t + dz * sin_t
        rz = -dx * sin_t + dz * cos_t

        # Scale to distance
        t = mm / rz if rz > 1 else mm
        wx = rx * t + sx
        wz = rz * t + tz

        if wz <= 1: continue
        u = fx * wx / wz + cx
        v = fx * (-dy * t) / wz + cy
        corners.append((u, v))

    return corners
```

- [ ] **Step 6: Integrate into main display loop**

At the top of `app_main` / `__main__`, check for `--serial` and branch:

```python
if args.serial:
    params = load_overlay_params()
    # Start serial reader thread
    threading.Thread(target=serial_reader, args=(args.serial,), daemon=True).start()
    # Run matplotlib display loop with overlay
    run_serial_display(params)
else:
    # Existing WiFi mode (unchanged)
    ...
```

- [ ] **Step 7: Test with mock data**

Create a simple test: generate a COBS-encoded JPEG packet, write to a virtual serial port, verify visualize.py displays it.

- [ ] **Step 8: Commit**

```bash
git add visualize.py
git commit -m "feat(espnow): add --serial mode to visualize.py with COBS, JPEG reassembly, and overlay"
```

---

## Task 10: Integration Test

- [ ] **Step 1: Build P4 firmware**

```bash
cd /workspaces/BoatEspP4
idf.py build
```

Verify: compiles with espnow_transport.c, Kconfig options visible in `idf.py menuconfig`.

- [ ] **Step 2: Build C6 slave firmware**

```bash
cd tools/slave_firmware
./build.sh
```

Verify: compiles with espnow_bridge.c patched in.

- [ ] **Step 3: Build S3 bridge firmware**

```bash
cd tools/espnow_bridge
idf.py set-target esp32s3
idf.py build
```

Verify: compiles with esp_tinyusb and esp_now.

- [ ] **Step 4: Extract overlay params**

```bash
python3 tools/extract_overlay_params.py
cat overlay_params.json
```

Verify: JSON matches dashboard.html constants.

- [ ] **Step 5: Hardware test sequence**

1. Flash C6 with new slave firmware
2. Flash P4 firmware (with no WiFi AP available)
3. Verify serial monitor shows "switching to ESP-NOW field mode"
4. Flash S3 bridge
5. Connect S3 to laptop USB
6. Run: `python visualize.py --serial /dev/ttyACM0`
7. Verify: JPEG frames appear, overlay renders, motor commands work

- [ ] **Step 6: Final commit**

```bash
git add -A
git commit -m "feat(espnow): complete ESP-NOW field mode integration"
```
