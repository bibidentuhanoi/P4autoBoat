# ESP-NOW Field Mode — Design Spec

**Date:** 2026-05-18
**Status:** Approved
**Author:** Kiet + Claude

## 1. Purpose

Add ESP-NOW as an alternative transport for field testing the boat without a WiFi router. At home, WiFi gives full dashboard + MJPEG. In the field, ESP-NOW gives direct peer-to-peer control + low-FPS video for training data collection + full telemetry + overlay.

## 2. Architecture

```
HOME (WiFi AP — unchanged):
  Browser <──WS+MJPEG──> ESP32-P4

FIELD (ESP-NOW — new):
  Laptop <──USB CDC──> ESP32-S3 (bridge)
                          |  ESP-NOW (250B packets)
                       ESP32-C6 (custom slave fw)
                          |  SDIO peer_data_transfer (up to 8166B)
                       ESP32-P4 (boat firmware)
```

### Mode Selection

Auto-detect at boot:
1. P4 boots → attempts WiFi STA connect for N seconds (configurable via Kconfig)
2. If AP found → WiFi mode (existing behavior, no changes)
3. If no AP → switch to ESP-NOW mode via peer_data_transfer commands to C6

## 3. Shared Projection Constants (CRITICAL INVARIANT)

The ToF-to-camera overlay uses a 3D geometric projection with calibration constants measured from the physical PCB layout and tuned via dashboard sliders. These constants MUST be identical in every renderer.

### Source of Truth

Create `main/overlay_params.json`:

```json
{
  "cam": { "w": 800, "h": 640 },
  "fx": 1242,
  "cx": 104,
  "cy": 199,
  "az_offset": 2.0,
  "el_offset": 0.3,
  "sensor_tz": 6.78,
  "zone_step": 5.625,
  "sensor_a": { "x": -22.8, "tilt": -4.45 },
  "sensor_b": { "x": 22.8, "tilt": 7.02 },
  "cam_scale": 0.72,
  "flip": {
    "ha": false, "va": false, "ta": false,
    "hb": true, "vb": true, "tb": false
  },
  "drag": { "x": 0, "y": 0 }
}
```

Consumers:
- `dashboard.html` — loads via EMBED_TXTFILES or inline import
- `visualize.py` — loads at startup via `json.load()`

### Resolution Scaling Rule

When rendering at a resolution other than native 800x640:

```
sf = display_width / 800.0

Scale with sf:     FX, CX, CY
Keep unchanged:    AZ_OFFSET, EL_OFFSET, SENSOR_TZ, ZONE_STEP,
                   sensor positions (x), sensor tilts, flip flags
```

All angular and physical parameters are resolution-independent. Only the pinhole camera model parameters (focal length, principal point) scale with pixel count. Detection bboxes (x1,y1,x2,y2) are always in 800x640 coords — multiply by `sf` to render.

### When You Re-Tune at Home

1. Adjust sliders/drag in dashboard
2. Update `overlay_params.json` with new values
3. Rebuild firmware (`idf.py build` embeds updated JSON)
4. `visualize.py` picks up changes on next launch (reads JSON directly from repo)

One file. Zero drift between renderers.

## 4. Data Protocol

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
    MSG_ESPNOW_INIT    = 0x10,  // Init ESP-NOW on C6 (P4 -> C6 only)
    MSG_ESPNOW_CONFIG  = 0x11,  // JPEG resolution info (P4 -> laptop, sent once)
};
```

### Downstream (P4 -> Laptop)

| Data | Format | Size | Rate |
|------|--------|------|------|
| JPEG frame | Chunked raw bytes, `seq` increments per chunk, last chunk has `payload_len` < max | 5-60KB total | 2-10fps |
| SensorSnapshot | Nanopb protobuf (IMU + ToF A/B + detections) | 200-400B | 20Hz |
| MotorStatus | Nanopb protobuf | ~12B | On change |
| Config | JPEG width/height once at init | 4B | Once |

### Upstream (Laptop -> P4)

| Data | Format | Size | Rate |
|------|--------|------|------|
| MotorCommand | Nanopb protobuf | ~16B | 20Hz |
| ArmCommand | Nanopb protobuf | ~2B | On press |
| DetectCommand | Nanopb protobuf | ~1B | On press |

### JPEG Fragmentation

P4-to-C6 (SDIO peer_data_transfer): chunks up to 8166 bytes. A 30KB frame = 4 chunks.

C6-to-Air (ESP-NOW): chunks of 244 bytes payload (250 - 6 overhead). Uses ESPNowCam-style protocol:
- Intermediate chunk: `{ total_len = 0, data[244] }`
- Last chunk: `{ total_len = frame_size, data[remainder] }`
- Sequential send with busy-wait on send callback (guarantees ordering)

S3 receiver: accumulates chunks into buffer, fires callback when `total_len > 0`, forwards reassembled JPEG + header over USB CDC.

## 5. Firmware Components

### 5A. P4 — `main/transports/espnow_transport.c` (~300 lines)

New file alongside existing `ws_transport.c`.

API:
```c
esp_err_t espnow_transport_init(void);
esp_err_t espnow_transport_send_jpeg(const uint8_t *jpg, size_t len);
esp_err_t espnow_transport_send_sensor(const SensorSnapshot *snap);
esp_err_t espnow_transport_send_motor_status(const MotorStatus *status);
void      espnow_transport_set_recv_cb(espnow_recv_cb_t cb);
```

Implementation:
- Uses `esp_hosted_send_custom_data()` / `esp_hosted_register_rx_callback_custom_data()`
- Fragments JPEG into ≤8166B chunks with `espnow_pkt_hdr_t` header
- Registers with `pipeline.c` as alternative to WS transport
- On recv: parses header, dispatches MotorCommand/ArmCommand/DetectCommand to existing handlers

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

Added to `tools/slave_firmware/`. The existing `build.sh` modified to include this file.

Responsibilities:
- Registers `esp_hosted_register_rx_callback_custom_data()` on the slave side
- On `MSG_ESPNOW_INIT`: calls `esp_now_init()`, `esp_now_add_peer()`, sets channel
- On any other message from P4: fragments into 244B ESP-NOW packets, sends via `esp_now_send()`
- On `esp_now_recv_cb()`: wraps received data with header, sends to P4 via `esp_hosted_send_custom_data()`

ESP-NOW config (Kconfig on C6 side):
- Peer MAC address (S3 bridge)
- Channel (must match S3)
- Encryption: off (field testing only)

### 5D. ESP32-S3 — Bridge Firmware (new project, ~200 lines)

Separate ESP-IDF project in `tools/espnow_bridge/`.

```
ESP-NOW recv -> reassemble JPEG / pass through telemetry -> USB CDC TX
USB CDC RX -> parse commands -> ESP-NOW send to C6
```

Uses standard ESP-IDF `esp_now` API (S3 has native WiFi, no remote proxy needed).
USB CDC via `tinyusb` driver for maximum throughput (no baud rate limit).

### 5E. `visualize.py` — Serial Mode Extension (~150 lines)

New `--serial` flag:
```bash
python visualize.py --serial /dev/ttyACM0
```

- Reads `overlay_params.json` for projection constants
- Parses `espnow_pkt_hdr_t` from serial stream
- Reassembles JPEG chunks, decodes with PIL/OpenCV
- Decodes SensorSnapshot protobuf (same nanopb schema as WS)
- Renders overlay: ToF zone projection (identical math to dashboard.html JS, ported to Python), detection bboxes, IMU horizon
- Applies scale factor `sf = jpeg_width / 800` to FX/CX/CY
- Keyboard/gamepad input → MotorCommand/ArmCommand → serial TX

## 6. Kconfig Additions

```
menu "ESP-NOW Field Mode"

    config ESPNOW_ENABLED
        bool "Enable ESP-NOW field mode"
        default y

    config ESPNOW_WIFI_TIMEOUT_S
        int "WiFi connect timeout before ESP-NOW fallback (seconds)"
        default 10
        depends on ESPNOW_ENABLED

    config ESPNOW_JPEG_DOWNSCALE
        int "JPEG downscale factor for ESP-NOW (1=native, 2=half, 4=quarter)"
        default 2
        depends on ESPNOW_ENABLED
        help
            Native 800x640 at factor 1 gives ~2-4fps.
            Half 400x320 at factor 2 gives ~8-12fps.
            Quarter 200x160 at factor 4 gives ~15-20fps.
            Aspect ratio 5:4 is preserved at all factors.

    config ESPNOW_JPEG_QUALITY
        int "JPEG quality for ESP-NOW (1-63, lower=smaller)"
        default 12
        depends on ESPNOW_ENABLED

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

## 7. FPS Budget

ESP-NOW air link throughput: ~60-100 KB/s (empirical, from ESPNowCam benchmarks).

| Downscale | Resolution | JPEG size (q=12) | Video FPS | Telemetry 20Hz fits? |
|-----------|-----------|-------------------|-----------|---------------------|
| 1 (native) | 800x640 | 20-30KB | 2-4 | Yes (~8KB/s overhead) |
| 2 (half) | 400x320 | 5-8KB | 8-12 | Yes |
| 4 (quarter) | 200x160 | 2-3KB | 15-20 | Yes |

Default: factor 2 (400x320) — best balance of usable video and driving responsiveness.

## 8. Out of Scope

- WiFi raw 802.11tx mode (standard ESP-NOW sufficient)
- NVS persistence of overlay params (source code is config)
- ESP-NOW encryption (field testing, not production)
- Camera resolution negotiation (Kconfig sets it)
- Multi-peer / broadcast (1:1 P2P only)
- S3 bridge display (laptop only, S3 is a dumb pipe)

## 9. Dependencies

- `espressif/esp_hosted` managed component (already in project, v2.12.3)
- `esp_hosted_send_custom_data()` API (peer_data_transfer, available since esp-hosted-mcu)
- ESP-IDF v5.4 on P4 (existing) and C6 (slave firmware)
- ESP-IDF on S3 (new project, any recent version)
- `tinyusb` on S3 for USB CDC
