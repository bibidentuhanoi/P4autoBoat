# AutoBoat ESP32-P4 Firmware

Autonomous boat firmware for ESP32-P4 using ESP-IDF v5.4. Combines camera vision, 9-DoF IMU sensor fusion, and dual ToF obstacle detection with a real-time web dashboard.

## Hardware

- **MCU**: ESP32-P4
- **Camera**: OV5647 (800x640 MJPEG via CSI/ISP pipeline)
- **IMU**: ICM20948 9-DoF (accel + gyro + magnetometer) over I2C
- **ToF**: 2x VL53L5CX 8x8 zone Time-of-Flight sensors (I2C, addr 0x22/0x24)
- **WiFi**: esp_hosted SDIO coprocessor

### Pin Configuration (Kconfig-tunable)

| Function | GPIO |
|---|---|
| I2C SDA (sensors) | 8 |
| I2C SCL (sensors) | 7 |
| ToF-A LPN | 52 |
| ToF-B LPN | 29 |
| Camera SCCB | I2C_NUM_0 (7/8) |

### Sensor Mounting

Both ToF sensors are offset from camera center by +/-22.8mm laterally and 6.78mm behind. Sensor B is physically flipped 180 degrees for symmetric RX placement. Both angled outward for wider obstacle coverage.

## Architecture

```
app_main() [main/main.c]
├── fs_init()              → NVS for calibration + xtalk persistence
├── camera_init()          → OV5647 via CSI → ISP → HW JPEG encoder
├── imu_init()             → ICM20948 over I2C_NUM_1
├── tof_init()             → Two VL53L5CX sensors (addr 0x22, 0x24)
├── fusion_init()          → Complementary filter setup
├── wifi_manager_start()   → esp_hosted SDIO WiFi
├── http_server_start()    → Dashboard (port 80) + WebSocket
├── camera_stream_start()  → MJPEG multipart stream (port 81)
└── FreeRTOS Tasks:
    ├── IMU_Task    (pri 4) → 10ms: reads IMU, runs complementary filter
    ├── Snap_Task   (pri 4) → 50ms: reads ToF + IMU, publishes protobuf
    ├── WS_TX       (pri 3) → WebSocket binary transport
    └── CamDrain    (pri 2) → Drains camera frames when no MJPEG client
```

### Data Pipeline

```
Sensors → task_sensor_snapshot (protobuf encode) → pipeline → ws_transport → browser
                                                            → camera_stream (MJPEG, port 81)
```

- **Protobuf** (nanopb): `BoatMessage` envelope with `SensorSnapshot`, `MotorCommand`, `SystemStatus`
- **ToFGrid** fields: distances (64), sigma (64), target_status (64), nb_target_detected (64)
- **3-frame median filter** on ToF distances before publish (kills single-frame spikes)

### ToF Sensor Configuration

| Setting | Value |
|---|---|
| Resolution | 8x8 (64 zones) |
| Ranging frequency | 10 Hz (Kconfig-tunable) |
| Integration time | 10 ms (Kconfig-tunable) |
| Target order | Strongest signal first |
| Sharpener | 20% (Kconfig-tunable) |
| Xtalk calibration | NVS-persisted, auto-runs on first boot |
| Valid status codes | 5 (valid), 9 (valid + noisy) |

### Sensor Fusion

Complementary filter blending gyro integration with accelerometer tilt. Alpha weighting (default 0.96) configurable via `CONFIG_FUSION_COMPLEMENTARY_ALPHA`. Magnetometer provides heading with hard-iron correction. Results are mutex-protected.

### Calibration

Triggered by GPIO 35 (boot button) at startup. Collects gyro bias samples and magnetometer min/max for hard-iron correction. ToF xtalk calibration saved to NVS via `file_system.c`.

## Web Dashboard

Single-file HTML dashboard served from flash (`main/dashboard.html`, embedded via `EMBED_TXTFILES`).

### Features

- **Camera feed**: MJPEG stream from port 81, scaled with configurable `camScale` (dark border = pre-alert zone)
- **ToF overlay modes**:
  - **3D Threshold** (default): proper Y-rotation projection, sigma-weighted opacity, edge zone detection
  - **Flat Threshold**: legacy pinhole projection for comparison
  - **Heatmap**: distance-colored soft circles, respects threshold slider
  - **Raw Grid**: projected distance values per zone
- **IMU display**: pitch/roll/heading values + artificial horizon
- **ToF grids**: 8x8 color-coded distance grids for both sensors
- **Settings modal**: overlay mode, threshold distance, blend mode, flip/transpose per sensor, Az/El/Scale/Cam sliders, drag toggle
- **Snap buttons**: capture camera frame or camera+overlay composite as JPEG

### Overlay Pipeline

- **Projection**: dual-mode (flat pinhole vs 3D Y-rotation) with live-tunable Az/El offsets
- **Sigma opacity**: `clamp(1 - sigma/25, 0.15, 0.9)` — clean surfaces solid, edge zones ghost out
- **Edge detection**: `nb_target_detected > 1` draws magenta dotted ring
- **Blend modes**: Best sigma (lowest noise wins), Average, Show both (A+B visible)
- **A+B overlap**: yellow dot where sensors agree on same canvas cell
- **WebSocket**: exponential backoff reconnect (1s-8s), watchdog for stale connections

## Build & Flash

```bash
idf.py build              # Build firmware
idf.py flash monitor      # Flash and open serial monitor
idf.py menuconfig         # Kconfig configuration (pins, WiFi, ToF sharpener, etc.)
idf.py fullclean          # Required after changing sdkconfig defaults
```

Targets ESP32-P4. Dev environment runs in a Docker devcontainer with ESP-IDF v5.4 toolchain.

## Dependencies

- ESP-IDF v5.4 (FreeRTOS, I2C, NVS, ESP Timer, HTTP Server, WiFi)
- `rjrp44/vl53l5cx` v4.0.0 — VL53L5CX managed component
- `livekit/nanopb` — protobuf encoding
- `espressif/esp_hosted` — SDIO WiFi coprocessor
- `espressif/esp_video` + `esp_cam_sensor` — CSI camera pipeline

## Project Structure

```
main/
├── main.c                  Entry point, init sequence, task creation
├── sensor_task.c           Sensor snapshot task (IMU + ToF → protobuf)
├── sensor_fusion.c         Complementary filter (pitch/roll/heading)
├── calibration.c           Gyro/mag calibration routines
├── pipeline.c              Protobuf encode + fan-out to transports
├── http_server.c           HTTP server (dashboard + API, port 80)
├── camera_stream.c         MJPEG multipart stream (port 81)
├── wifi_manager.c          WiFi STA connection management
├── file_system.c           NVS storage (calibration + xtalk data)
├── dashboard.html          Embedded web dashboard (single file)
├── drivers/
│   ├── imu_driver.c        ICM20948 register-level I2C driver
│   ├── tof_driver.c        VL53L5CX config + read wrapper
│   └── camera_driver.c     OV5647 CSI → ISP → JPEG pipeline
├── transports/
│   └── ws_transport.c      WebSocket binary transport
├── proto/
│   ├── boat.proto          Protobuf schema
│   ├── boat.pb.c           Nanopb generated encoder
│   └── boat.pb.h           Nanopb generated header
├── include/
│   └── common.h            Shared types (CalibrationData, etc.)
└── Kconfig.projbuild       All tunable parameters
```
