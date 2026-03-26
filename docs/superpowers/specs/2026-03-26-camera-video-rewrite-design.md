# Camera Video Pipeline Rewrite — Design Spec

**Date:** 2026-03-26
**Status:** Approved
**Scope:** MJPEG stream only — fix purple/line color artifact

---

## Problem

The current camera video pipeline produces a purple-tinted image with horizontal line artifacts. Root causes identified:

1. `esp_video_init_config_t` is passed a `.jpeg` field (`esp_video_init_jpeg_config_t`) that the reference example never uses. This likely mis-configures the ISP→HW-JPEG pipeline chain during `esp_video_init()`.
2. The fallback raw-encoding path (for non-JPEG formats) has a format mismatch — it encodes Bayer/YUV data with wrong byte-order or stride assumptions, producing the purple+lines artifact.
3. `camera_init()` is called after WiFi init, contrary to the reference guidance that video init should happen immediately after device restart.

---

## Reference

All implementation patterns are taken directly from:
```
docs/simple_video_server/
  components/example_video_common/example_init_video.c   ← camera init pattern
  components/example_video_common/example_encoder.c      ← JPEG quality ioctl pattern
  main/simple_video_server_example.c                     ← V4L2 buffer loop + stream handler
  components/example_video_common/include/boards/
    esp32-p4-function-ev-board-v1.4/example_video_common_board.h  ← pin config
```

**Key lessons from the reference:**
- `esp_video_init_config_t` takes `.csi` only — never `.jpeg`
- `init_sccb = true` — let `esp_video_init()` create its own I2C bus for SCCB
- JPEG quality set via `VIDIOC_S_EXT_CTRLS` with `ctrl_class = V4L2_CID_JPEG_CLASS`, control id `V4L2_CID_JPEG_COMPRESSION_QUALITY`, after querying valid range with `VIDIOC_QUERY_EXT_CTRL`
- `VIDIOC_G_FMT` tells you the actual output format — log the FourCC, never assume
- If format is not `V4L2_PIX_FMT_JPEG`, something in sdkconfig is wrong — fail loudly
- Stream loop: DQBUF → check `V4L2_BUF_FLAG_DONE` → send boundary → send JPEG → QBUF
- `esp_video_init()` must be called **before WiFi** (reference comment: "camera device may not start due to lack of main clock")

---

## Hardware Context

- **Board:** ESP32-P4
- **Sensor:** OV5647 via MIPI-CSI
- **SCCB pins:** SCL=8, SDA=7 (dedicated, not shared with IMU/ToF I2C)
- **XCLK:** not needed (pin = -1, sensor uses internal clock)
- **Reset/PWDN:** not used (-1)
- **Pipeline:** OV5647 RAW8 800×640 → ISP (debayer+color) → HW JPEG → `V4L2_PIX_FMT_JPEG`
- **Flip:** VFLIP=1, HFLIP=1 (180° rotation, sensor mounted upside-down)

The IMU (ICM20948) and ToF (VL53L5CX ×2) remain on their own I2C bus — completely unaffected.

---

## Design

### `camera_driver.c` — Full Rewrite

**Init sequence (matches `example_init_video.c` + `init_web_cam_video()`):**

```
camera_init()
  1. esp_video_init(&cam_cfg)
       cam_cfg.csi only — NO .jpeg field
       csi.sccb_config.init_sccb = true        ← esp_video owns the I2C bus
       csi.sccb_config.freq = 400000
       csi.reset_pin = CONFIG_CAM_RESET_PIN     (-1)
       csi.pwdn_pin  = CONFIG_CAM_PWDN_PIN      (-1)

  2. open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR)

  3. VIDIOC_S_EXT_CTRLS — set VFLIP + HFLIP
       ctrl_class = V4L2_CID_USER_CLASS
       V4L2_CID_VFLIP = CONFIG_CAM_VFLIP
       V4L2_CID_HFLIP = CONFIG_CAM_HFLIP

  4. VIDIOC_G_FMT — read actual format
       Log FourCC string so it's always visible in boot log
       If pixel_format != V4L2_PIX_FMT_JPEG → ESP_LOGE + return ESP_ERR_NOT_SUPPORTED
       (This means sdkconfig ISP/JPEG pipeline is wrong — fix there, not here)

  5. VIDIOC_QUERY_EXT_CTRL (V4L2_CID_JPEG_COMPRESSION_QUALITY)
       Get min/max/step
       Clamp CONFIG_CAM_JPEG_QUALITY to valid range
       VIDIOC_S_EXT_CTRLS with ctrl_class = V4L2_CID_JPEG_CLASS
       Log the clamped quality value

  6. VIDIOC_REQBUFS — request CAM_BUF_COUNT (2) mmap buffers

  7. For each buffer: VIDIOC_QUERYBUF → mmap → VIDIOC_QBUF

  8. VIDIOC_STREAMON
```

**Public API — unchanged signatures except `camera_init`:**
```c
esp_err_t camera_init(void);                          // was camera_init(i2c_handle)
esp_err_t camera_capture_frame(...);                  // unchanged
void      camera_release_frame(void);                 // unchanged
void      camera_get_frame_info(...);                 // unchanged
```

---

### `camera_stream.c` — Full Rewrite

**Startup (`camera_stream_server_start`):**
```
  Read format via camera_get_frame_info()
  If not V4L2_PIX_FMT_JPEG → return ESP_ERR_NOT_SUPPORTED (camera_driver already checked)
  Log confirmed format + resolution
  Start httpd on CONFIG_HTTP_STREAM_PORT
  Register /stream URI handler
```

No encoder engine. No output buffer. No semaphore. No format-mapping table.

**Stream loop (matches `image_stream_handler` in reference):**
```
  httpd_resp_set_type("multipart/x-mixed-replace;boundary=frame")
  httpd_resp_set_hdr("Access-Control-Allow-Origin", "*")

  loop:
    VIDIOC_DQBUF
    if !(flags & V4L2_BUF_FLAG_DONE) → VIDIOC_QBUF, continue
    send "--frame\r\n"
    send "Content-Type: image/jpeg\r\nContent-Length: <bytesused>\r\n\r\n"
    send jpeg bytes (buf[index], bytesused)
    VIDIOC_QBUF
    on send error → break
```

Note: `bytesused` is used directly — not the full buffer size. This avoids sending garbage bytes past the JPEG end marker.

---

### `main.c` — Boot Order Change

```c
// Before (current — wrong order):
nvs → fs → imu → tof → fusion → wifi → camera_init(i2c_handle) → stream

// After (reference pattern — camera first):
nvs → fs → camera_init() → imu → tof → fusion → wifi → stream
```

`camera_init()` no longer takes `i2c_master_bus_handle_t`. Call site in `main.c` updated accordingly.

---

## Files Changed

| File | Change |
|---|---|
| `main/drivers/camera_driver.c` | Full rewrite |
| `main/drivers/camera_driver.h` | `camera_init(void)` signature |
| `main/camera_stream.c` | Full rewrite |
| `main/main.c` | Move camera_init(), remove i2c_handle arg |

## Files NOT Changed

`camera_stream.h`, `Kconfig.projbuild`, `sdkconfig.defaults`, `main.c` (structure), all IMU/ToF/fusion code.

---

## Success Criteria

- Boot log shows `Format: 800x640 fmt=JPEG`
- Stream at `http://<ip>:80/stream` shows **full color image** with no purple tint and no line artifacts
- JPEG quality matches `CONFIG_CAM_JPEG_QUALITY` (default 60)
- All existing sensor/fusion functionality unaffected
