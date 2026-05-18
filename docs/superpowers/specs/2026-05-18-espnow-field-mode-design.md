# ESP-NOW Field Mode — Design Spec

**Date:** 2026-05-18
**Status:** Approved (revised after devil's advocate + research sweep)
**Author:** Kiet + Claude

## 1. Purpose

Add ESP-NOW as an alternative transport for field testing the boat without a WiFi router. At home, WiFi gives full dashboard + MJPEG. In the field, ESP-NOW gives direct peer-to-peer control + low-FPS video for training data collection + full telemetry + overlay.

## 2. Architecture

```
HOME (WiFi AP — unchanged):
  Browser <──WS+MJPEG──> ESP32-P4

FIELD (ESP-NOW — new):
  Laptop <──USB CDC──> ESP32-S3 (bridge)
                          |  ESP-NOW (250B packets, ~60-100 KB/s)
                       ESP32-C6 (custom slave fw)
                          |  SDIO peer_data_transfer (up to 8166B, ~10 MB/s)
                       ESP32-P4 (boat firmware)
```

### Mode Selection

Auto-detect at boot:
1. P4 boots → attempts WiFi STA connect for N seconds (configurable via Kconfig)
2. If AP found → WiFi mode (existing behavior, no changes)
3. If no AP → switch to ESP-NOW mode via peer_data_transfer commands to C6

### WiFi → ESP-NOW Transition Sequence

The C6 WiFi stack must stay initialized (ESP-NOW requires it). Transition:
1. P4 sends `wifi_disconnect` + `wifi_stop` RPCs to C6
2. P4 waits for confirmation
3. C6 WiFi stack remains initialized but not connected/scanning
4. P4 sends `MSG_ESPNOW_INIT` via `esp_hosted_send_custom_data()`
5. C6 calls `esp_now_init()` + `esp_now_add_peer()` on the already-initialized WiFi stack

## 3. Shared Projection Constants (CRITICAL INVARIANT)

The ToF-to-camera overlay uses a 3D geometric projection with calibration constants measured from the physical PCB layout and tuned via dashboard sliders. These constants MUST be identical in every renderer.

### Projection Chain

```
ToF zone (row, col) + distance_mm
    → Angular position: az = (col - 3.5) × ZONE_STEP + AZ_OFFSET
                         el = (3.5 - row) × ZONE_STEP + EL_OFFSET
    → 3D point: rotate by sensorTilt around Y-axis
                + translate by sensorX (lateral) + SENSOR_TZ (depth)
    → Camera projection: FX, CX, CY (pinhole model)
    → Pixel coords in 800×640 space (+ dragOff user adjustment)
    → Display scale: pixel × overlayScale + overlayOff
```

### Source of Truth

Dashboard.html JS constants ARE the source of truth (baked into source after tuning):

```
FX=1242  CX=104  CY=199
AZ_OFFSET=2.0  EL_OFFSET=0.3  ZONE_STEP=5.625
SENSOR_TZ=6.78  camScale=0.72
SENSOR_A={x:-22.8, tilt:-4.45}  SENSOR_B={x:22.8, tilt:7.02}
FLIP: HA=false VA=false TA=false HB=true VB=true TB=false
dragOff: X=0 Y=0
```

A Python build script extracts these constants from `dashboard.html` into `overlay_params.json` for `visualize.py`. When you re-tune at home and update the HTML source, re-run the script.

### Resolution Scaling Rule

Camera always outputs 800×640 (hardware-configured). Images are sent at native resolution over ESP-NOW (no software downscale — see §7). For display at any size:

```
sf = display_width / 800.0

Scale with sf:     FX, CX, CY
Keep unchanged:    AZ_OFFSET, EL_OFFSET, SENSOR_TZ, ZONE_STEP,
                   sensor positions (x), sensor tilts, flip flags
```

Detection bboxes (x1,y1,x2,y2) are always in 800×640 coords — multiply by `sf` to render.

## 4. Data Protocol

### Packet Header

All messages over the ESP-NOW link share a common packet header:

```c
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     // enum: see below
    uint16_t payload_len;  // bytes following this header
    uint8_t  seq;          // frame sequence for JPEG reassembly
} espnow_pkt_hdr_t;       // 4 bytes total
```

### Message Types

```c
enum espnow_msg_type {
    MSG_JPEG_CHUNK     = 0x01,  // JPEG fragment (P4 -> laptop)
    MSG_SENSOR         = 0x02,  // SensorSnapshot protobuf (P4 -> laptop)
    MSG_MOTOR_CMD      = 0x03,  // MotorCommand protobuf (laptop -> P4)
    MSG_ARM_CMD        = 0x04,  // ArmCommand protobuf (laptop -> P4)
    MSG_DETECT_CMD     = 0x05,  // DetectCommand protobuf (laptop -> P4)
    MSG_MOTOR_STATUS   = 0x06,  // MotorStatus protobuf (P4 -> laptop)
    MSG_ESPNOW_INIT    = 0x10,  // Init ESP-NOW on C6 (P4 -> C6 only, not forwarded)
    MSG_ESPNOW_CONFIG  = 0x11,  // JPEG resolution info (P4 -> laptop, sent once)
};
```

### Downstream (P4 -> Laptop)

| Data | Format | Size | Rate |
|------|--------|------|------|
| JPEG frame | Chunked raw bytes, `seq` increments per chunk | 15-30KB total | 3-5fps @ native 800×640 |
| SensorSnapshot | Nanopb protobuf (IMU + ToF A/B + detections) | 200-400B | 20Hz |
| MotorStatus | Nanopb protobuf | ~12B | On change |
| Config | JPEG width/height once at init | 4B | Once |

### Upstream (Laptop -> P4)

| Data | Format | Size | Rate |
|------|--------|------|------|
| MotorCommand | Nanopb protobuf | ~16B | 20Hz |
| ArmCommand | Nanopb protobuf | ~2B | On press |
| DetectCommand | Nanopb protobuf | ~1B | On press |

### JPEG Fragmentation (two layers)

**P4 → C6** (SDIO `esp_hosted_send_custom_data`): chunks up to 8166 bytes. A 25KB frame = 4 chunks. API is synchronous — blocks until C6 acknowledges, providing natural backpressure.

**C6 → Air** (ESP-NOW): chunks of 244 bytes payload (250 - 6 overhead). Uses ESPNowCam-style protocol:
- Intermediate chunk: `{ total_len = 0, data[244] }`
- Last chunk: `{ total_len = frame_size, data[remainder] }`
- Sequential send with busy-wait on send callback (guarantees ordering)

**S3 receiver**: accumulates chunks into buffer, fires callback when `total_len > 0`, COBS-frames and forwards over USB CDC.

### USB Serial Framing (S3 ↔ Laptop)

COBS (Consistent Overhead Byte Stuffing) with 0x00 delimiter:
- Each packet: `espnow_pkt_hdr_t` + payload → COBS-encode → append 0x00
- ~1% overhead, zero false sync, self-recovering after dropped bytes
- Implementation: ~30 lines C (S3), ~20 lines Python (visualize.py), no external dependency

### Command Priority on C6

The C6 registers separate `esp_hosted_register_custom_callback()` handlers per msg_id:
- **Video msg_ids** (MSG_JPEG_CHUNK): callback queues to a FreeRTOS send task that fragments and sends via ESP-NOW sequentially. Slow path (~100-400ms per frame).
- **Command msg_ids** (MSG_MOTOR_CMD, MSG_ARM_CMD): callback sends directly via single `esp_now_send()` (~1ms). Fast path, never blocked by video.

This ensures motor commands have <5ms C6 transit time even during JPEG streaming.

## 5. Firmware Components

### 5A. P4 — `main/transports/espnow_transport.c` (~300 lines)

New file alongside existing `ws_transport.c`. Plugs into existing pipeline via `pipeline_register_transport()`.

Verified API (exists in esp_hosted v2.12.3, `host/esp_hosted_misc.h:128`):
```c
// Send to C6 (synchronous, blocks until C6 ACKs)
esp_err_t esp_hosted_send_custom_data(uint32_t msg_id, const uint8_t *data, size_t data_len);

// Register per-msg_id callback for data FROM C6
esp_err_t esp_hosted_register_custom_callback(uint32_t msg_id,
    void (*callback)(uint32_t msg_id, const uint8_t *data, size_t data_len));
```

Build gate: requires `H_PEER_DATA_TRANSFER` defined (add to sdkconfig.defaults).

Pipeline integration — same pattern as ws_transport:
```c
espnow_transport_init()
  → pipeline_register_transport(espnow_send_fn, ctx)
  → pipeline_register_motor_handler(espnow_motor_handler)
  → pipeline_register_arm_handler(espnow_arm_handler)
```

### 5B. P4 — Auto-detect in `main.c` (~30 lines)

After `wifi_init()` fails or times out:
```c
if (wifi_ret != ESP_OK) {
    ESP_LOGW(TAG, "WiFi unavailable — switching to ESP-NOW field mode");
    espnow_transport_init();  // sends MSG_ESPNOW_INIT to C6 via peer_data
    // skip httpd_start, camera_stream_start (no WiFi = no HTTP)
    // sensor_task still runs, feeds espnow_transport instead of ws_transport
}
```

### 5C. C6 — `espnow_bridge.c` (added to custom slave firmware, ~250 lines)

Added to `tools/slave_firmware/`. The existing `build.sh` modified to patch this file into the slave build.

Verified slave-side API (`slave/main/esp_hosted_peer_data.h`):
```c
esp_err_t esp_hosted_send_custom_data(uint32_t msg_id, const uint8_t *data, size_t data_len);
esp_err_t esp_hosted_register_custom_callback(uint32_t msg_id,
    void (*callback)(uint32_t msg_id, const uint8_t *data, size_t data_len));
// Note: "Callback runs in RPC RX thread — keep it fast!"
```

Architecture:
- Registers callbacks for each msg_id from P4
- `MSG_ESPNOW_INIT` callback: calls `esp_now_init()`, `esp_now_add_peer()`, sets channel
- Video callbacks: enqueue to `espnow_tx_queue` (FreeRTOS) → `espnow_tx_task` fragments and sends
- Command callbacks: direct `esp_now_send()` (single packet, <1ms)
- `esp_now_recv_cb()`: wraps received data with header, sends to P4 via `esp_hosted_send_custom_data()`

C6 resources: 512KB SRAM, ~100-200KB free after hosted stack. 8KB send buffer + queue = fine.

### 5D. ESP32-S3 — Bridge Firmware (new project, ~200 lines)

Separate ESP-IDF project in `tools/espnow_bridge/`.

```
ESP-NOW recv → reassemble JPEG / pass through telemetry → COBS-frame → USB CDC TX
USB CDC RX → COBS-decode → parse command → ESP-NOW send to C6
```

Uses:
- Standard ESP-IDF `esp_now` API (S3 has native WiFi)
- `espressif/esp_tinyusb` v2.2.0 managed component for USB CDC
  - `tinyusb_cdcacm_write_queue()` + `tinyusb_cdcacm_write_flush()` for TX
  - `callback_rx` + `tinyusb_cdcacm_read()` for RX
  - Throughput: ~925 KB/s (8K FIFO), 9x more than ESP-NOW bottleneck
  - Shows as `/dev/ttyACM0` on Linux, no driver needed

S3 is a dumb pipe — no protobuf decode/encode, no overlay logic.

### 5E. `visualize.py` — Serial Mode Extension (~150 lines)

New `--serial` flag:
```bash
python visualize.py --serial /dev/ttyACM0
```

- Loads `overlay_params.json` for projection constants (extracted from dashboard.html)
- COBS-decodes serial stream → parses `espnow_pkt_hdr_t`
- Reassembles JPEG chunks, decodes with PIL/OpenCV
- Decodes SensorSnapshot protobuf (same nanopb schema as WS)
- Renders overlay: ToF zone projection (identical math to dashboard.html JS, ported to Python), detection bboxes, IMU horizon
- Applies scale factor `sf = display_width / 800` to FX/CX/CY
- Keyboard/gamepad input → MotorCommand/ArmCommand → COBS-encode → serial TX

## 6. Kconfig Additions

### P4 (main firmware)

```
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
            Lower = smaller frames = higher FPS but worse image quality.
            At quality 12, native 800x640 frames are ~15-25KB → 3-5 fps.
            Camera always outputs native 800x640 (no downscale).

    config ESPNOW_CHANNEL
        int "ESP-NOW WiFi channel"
        default 6
        range 1 14
        depends on ESPNOW_ENABLED

    config ESPNOW_PEER_MAC
        string "ESP32-S3 bridge MAC address"
        default "FF:FF:FF:FF:FF:FF"
        depends on ESPNOW_ENABLED
        help
            Set to broadcast for discovery, or specific MAC for P2P.

endmenu
```

### P4 sdkconfig.defaults additions

```
CONFIG_ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS=8
```

Note: `H_PEER_DATA_TRANSFER` must also be enabled (verify Kconfig path during implementation).

### C6 slave firmware

```
CONFIG_ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS=8
CONFIG_ESP_HOSTED_COPROCESSOR_APP_MAIN=n
```

## 7. FPS Budget

ESP-NOW air link throughput: ~60-100 KB/s (empirical, from ESPNowCam benchmarks).

Camera outputs native 800×640. No software downscale (HW JPEG encoder doesn't support runtime resolution change; software resize is too expensive on P4). FPS controlled via JPEG quality only.

| JPEG Quality | Approx Frame Size | Video FPS | Telemetry 20Hz overhead |
|-------------|-------------------|-----------|------------------------|
| 8 (low) | 10-15KB | 5-8 fps | ~8KB/s, fits |
| 12 (default) | 15-25KB | 3-5 fps | ~8KB/s, fits |
| 20 (medium) | 25-40KB | 2-3 fps | ~8KB/s, fits |

Default: quality 12 → ~3-5 fps at 800×640. Sufficient for training data collection.

## 8. Out of Scope

- WiFi raw 802.11tx mode (standard ESP-NOW sufficient)
- Software JPEG downscaling (HW encoder is fixed resolution; quality-only control)
- NVS persistence of overlay params (source code is config)
- ESP-NOW encryption (field testing, not production)
- Multi-peer / broadcast (1:1 P2P only)
- S3 bridge display (laptop only, S3 is a dumb pipe)
- S3 protobuf decode (S3 forwards raw bytes, laptop decodes)

## 9. Verified Dependencies

| Dependency | Version | Verified? |
|-----------|---------|-----------|
| `espressif/esp_hosted` | v2.12.3 (installed) | Yes — `esp_hosted_send_custom_data()` at `rpc_wrap.c:2557`, added v2.8.1 |
| `espressif/esp_tinyusb` | v2.2.0 (S3 bridge) | Yes — CDC ACM API, ~925KB/s throughput |
| ESP-IDF | v5.4 (P4, C6) | Yes — existing |
| ESP-IDF | any recent (S3) | Yes — `esp_now` + `esp_tinyusb` both available |
| nanopb | already in project | Yes — `livekit__nanopb` in CMakeLists |
| COBS | inline (~30 lines C, ~20 lines Python) | No external dependency |
