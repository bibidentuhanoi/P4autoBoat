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
camera_init(sccb_handle)
  1. esp_video_init(&cam_cfg)
       cam_cfg.csi only — NO .jpeg field
       csi.sccb_config.init_sccb = false       ← app owns the I2C bus (passed as sccb_handle)
       csi.sccb_config.i2c_handle = sccb_handle
       csi.sccb_config.freq = 400000
       csi.reset_pin = CONFIG_CAM_RESET_PIN     (-1)
       csi.pwdn_pin  = CONFIG_CAM_PWDN_PIN      (-1)
       On failure: ESP_LOGE with hint to check CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER
       and CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE in sdkconfig

  2. open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR)

  3. Set VFLIP + HFLIP using individual VIDIOC_S_CTRL calls (one per control)
       This matches the reference — it does NOT use VIDIOC_S_EXT_CTRLS for flip.
       V4L2_CID_VFLIP = CONFIG_CAM_VFLIP
       V4L2_CID_HFLIP = CONFIG_CAM_HFLIP
       Log result; warn but do not fail if ioctl returns error (some sensors ignore it)

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

**Public API — updated signatures:**
```c
esp_err_t camera_init(i2c_master_bus_handle_t sccb_handle);  // app creates SCCB bus, passes handle
esp_err_t camera_capture_frame(...);                          // unchanged
void      camera_release_frame(void);                         // unchanged
void      camera_get_frame_info(...);                         // unchanged
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
    send "\r\n--frame\r\n"                          ← leading \r\n required by multipart MIME
    send "Content-Type: image/jpeg\r\nContent-Length: <bytesused>\r\n\r\n"
    send jpeg bytes (buf[index], bytesused)          ← bytesused only, NOT full buf length
    VIDIOC_QBUF
    on send error → break
```

Note: `bytesused` is used directly — not the full buffer size. This avoids sending garbage bytes past the JPEG end marker.

---

### `main.c` — Boot Order Change

```c
// Before (current — wrong order):
nvs → fs → imu → tof → fusion → wifi → camera_init(i2c_handle) → stream

// After (GPIO handoff pattern — camera first):
nvs
→ i2c_new_master_bus(I2C0, SCL=8, SDA=7)   ← temporary SCCB bus
→ camera_init(sccb_handle)                  ← programs OV5647, starts MIPI CSI
→ i2c_del_master_bus(sccb_handle)           ← frees GPIO7/GPIO8
→ i2c_new_master_bus(I2C0, SCL=8, SDA=7)   ← sensor bus (IMU + ToF)
→ imu → tof → fusion → wifi → stream
```

`camera_init()` takes `i2c_master_bus_handle_t sccb_handle` — a temporary bus the caller creates and deletes after init.

---

## Files Changed

| File | Change |
|---|---|
| `main/drivers/camera_driver.c` | Full rewrite |
| `main/drivers/camera_driver.h` | `camera_init(void)` signature |
| `main/camera_stream.c` | Full rewrite |
| `main/main.c` | Move camera_init(), remove i2c_handle arg |

## Files NOT Changed

`camera_stream.h`, `Kconfig.projbuild`, `sdkconfig.defaults`, all IMU/ToF/fusion code.

---

## Success Criteria

- Boot log shows `Format: 800x640 fmt=JPEG`
- Boot log shows no I2C arbitration errors on port 0 after `camera_init()` (confirms SCCB bus is independent)
- Stream at `http://<ip>:80/stream` shows **full color image** with no purple tint and no line artifacts
- JPEG quality log confirms clamped value applied via ioctl (matches `CONFIG_CAM_JPEG_QUALITY` default 60)
- All existing sensor/fusion tasks start and log normally after `camera_init()` completes
