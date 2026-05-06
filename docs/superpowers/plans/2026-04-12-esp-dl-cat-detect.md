# esp-dl Cat Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add on-device cat detection via esp-dl pico 224x224 model, triggered by dashboard button, results returned via WS protobuf telemetry.

**Architecture:** Dedicated `detect_task` owns a CatDetect instance. On trigger: pause MJPEG stream, capture one RGB565 frame from ISP, run inference (auto-resizes 800x640 to 224x224), package results into protobuf, resume stream. Stop-the-world approach eliminates concurrent DMA collisions.

**Tech Stack:** ESP-IDF 5.4, esp-dl 3.3 (cat_detect pico), nanopb protobuf, FreeRTOS

**Spec:** `docs/superpowers/specs/2026-04-12-esp-dl-cat-detect-design.md`

---

### Task 1: Add esp-dl / cat_detect component dependency

**Files:**
- Modify: `main/idf_component.yml`
- Modify: `main/CMakeLists.txt`

- [ ] **Step 1: Add cat_detect to idf_component.yml**

Add after the nanopb dependency:

```yaml
  ## esp-dl cat detection model (pico 224x224)
  espressif/cat_detect:
    version: "*"
    override_path: "../tools/esp-dl/models/cat_detect"
```

- [ ] **Step 2: Update CMakeLists.txt — add C++ source and fix PRIV_REQUIRES**

Add `detect_task.cpp` to SRCS, add `espressif__cat_detect` and `espressif__esp-dl` to PRIV_REQUIRES, remove `esp_driver_ppa` (PPA was removed in stability commit):

```cmake
idf_component_register(
    SRCS
        "main.c"
        "file_system.c"
        "sensor_fusion.c"
        "calibration.c"
        "drivers/imu_driver.c"
        "drivers/tof_driver.c"
        "drivers/camera_driver.c"
        "wifi_manager.c"
        "camera_stream.c"
        "pipeline.c"
        "http_server.c"
        "sensor_task.c"
        "transports/ws_transport.c"
        "proto/boat.pb.c"
        "detect_task.cpp"
    PRIV_REQUIRES
        spi_flash esp_timer nvs_flash driver
        esp_video esp_wifi esp_wifi_remote esp_netif
        esp_http_server esp_driver_jpeg
        livekit__nanopb espressif__cat_detect espressif__esp-dl
    INCLUDE_DIRS "include" "drivers" "." "proto"
    EMBED_TXTFILES "dashboard.html")
```

- [ ] **Step 3: Create stub detect_task files so it compiles**

Create `main/detect_task.h`:

```c
#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the detection task. Call after pipeline_init(). */
esp_err_t detect_init(void);

/** Trigger one inference cycle. Safe to call from any task. */
void detect_trigger(void);

#ifdef __cplusplus
}
#endif
```

Create `main/detect_task.cpp` (stub):

```cpp
#include "detect_task.h"
#include "esp_log.h"

static const char *TAG = "DETECT";

extern "C" esp_err_t detect_init(void)
{
    ESP_LOGI(TAG, "Detect task initialized (stub)");
    return ESP_OK;
}

extern "C" void detect_trigger(void)
{
    ESP_LOGI(TAG, "Detect triggered (stub)");
}
```

- [ ] **Step 4: Build to verify esp-dl links**

Run: `ninja -C build` (or `idf.py fullclean && idf.py build` if cmake cache is stale)

Expected: Build succeeds. The cat_detect component is found via override_path, esp-dl links.

Note: If cmake doesn't find the component, delete `build/` and run `idf.py build` to force component resolution.

- [ ] **Step 5: Commit**

```bash
git add main/idf_component.yml main/CMakeLists.txt main/detect_task.h main/detect_task.cpp
git commit -m "feat(detect): add esp-dl cat_detect dependency + stub detect_task"
```

---

### Task 2: Update protobuf schema with Detection message

**Files:**
- Modify: `main/proto/boat.proto`
- Modify: `main/proto/boat.options`
- Regenerate: `main/proto/boat.pb.c`, `main/proto/boat.pb.h`

- [ ] **Step 1: Add Detection message and DetectCommand to boat.proto**

Add before the `BoatMessage` definition:

```protobuf
message Detection {
  int32 category = 1;
  float score    = 2;
  int32 x1       = 3;
  int32 y1       = 4;
  int32 x2       = 5;
  int32 y2       = 6;
}

message DetectCommand {
}
```

Add `repeated Detection detections = 5;` to `SensorSnapshot`:

```protobuf
message SensorSnapshot {
  uint64  timestamp_us = 1;
  IMUData imu          = 2;
  ToFGrid tof_a        = 3;
  ToFGrid tof_b        = 4;
  repeated Detection detections = 5;
}
```

Add `DetectCommand detect = 4;` to the `BoatMessage` oneof:

```protobuf
message BoatMessage {
  oneof payload {
    SensorSnapshot sensors  = 1;
    MotorCommand   motor    = 2;
    SystemStatus   status   = 3;
    DetectCommand  detect   = 4;
  }
}
```

- [ ] **Step 2: Add nanopb options for Detection array**

Add to `main/proto/boat.options`:

```
boat.SensorSnapshot.detections   max_count:10
boat.ToFGrid.sigma               max_count:64
boat.ToFGrid.target_status       max_count:64
boat.ToFGrid.nb_target_detected  max_count:64
```

Note: also add the missing max_count entries for sigma, target_status, and nb_target_detected if not already present (nanopb requires max_count for all repeated fields).

- [ ] **Step 3: Regenerate nanopb files**

Find the nanopb generator:

```bash
find /workspaces/BoatEspP4/managed_components -name "nanopb_generator.py" 2>/dev/null || \
find /workspaces/BoatEspP4/build/managed_components -name "nanopb_generator.py" 2>/dev/null
```

Then run:

```bash
python3 <path_to_nanopb_generator>/nanopb_generator.py \
  -I main/proto -D main/proto main/proto/boat.proto
```

This regenerates `main/proto/boat.pb.c` and `main/proto/boat.pb.h`.

- [ ] **Step 4: Build to verify proto compiles**

Run: `ninja -C build`

Expected: Build succeeds with new protobuf types available.

- [ ] **Step 5: Commit**

```bash
git add main/proto/boat.proto main/proto/boat.options main/proto/boat.pb.c main/proto/boat.pb.h
git commit -m "feat(proto): add Detection message and DetectCommand to protobuf schema"
```

---

### Task 3: Implement detect_task with CatDetect inference

**Files:**
- Modify: `main/detect_task.cpp`
- Modify: `main/main.c` (call detect_init)

- [ ] **Step 1: Implement full detect_task.cpp**

```cpp
#include "detect_task.h"
#include "drivers/camera_driver.h"
#include "camera_stream.h"
#include "pipeline.h"
#include "proto/boat.pb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cat_detect.hpp"
#include "dl_image_define.hpp"

static const char *TAG = "DETECT";

static TaskHandle_t s_detect_task = NULL;
static CatDetect *s_detector = NULL;

static void detect_task_fn(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t t0 = esp_timer_get_time();

        /* Lazy-load model on first trigger */
        if (!s_detector) {
            ESP_LOGI(TAG, "Loading cat_detect model (first trigger)...");
            s_detector = new CatDetect();
            ESP_LOGI(TAG, "Model loaded in %lld ms",
                     (esp_timer_get_time() - t0) / 1000);
            t0 = esp_timer_get_time();
        }

        /* 1. Pause MJPEG streaming */
        camera_stop_streaming();
        vTaskDelay(pdMS_TO_TICKS(50)); /* let in-flight sends complete */

        /* 2. Capture one frame (RGB565 from ISP) */
        void *frame_buf = NULL;
        size_t frame_len = 0;
        uint32_t width = 0, height = 0, pix_fmt = 0;

        /* We need the raw RGB565, but camera_capture_frame() returns JPEG.
         * We need to capture the raw ISP buffer directly.
         * Use camera_start_streaming() + camera_capture_raw() or access
         * the mmap buffer before JPEG encode.
         *
         * ALTERNATIVE: Since camera_capture_frame does ISP -> JPEG,
         * and we need RGB565, we capture via the drain path:
         * restart streaming briefly, grab one raw frame, stop again.
         */
        camera_start_streaming();

        esp_err_t cap_ret = camera_capture_frame(&frame_buf, &frame_len,
                                                  &width, &height, &pix_fmt);
        if (cap_ret != ESP_OK) {
            ESP_LOGE(TAG, "Frame capture failed: %s", esp_err_to_name(cap_ret));
            camera_release_frame();
            camera_start_streaming();
            goto done;
        }

        /* NOTE: camera_capture_frame returns JPEG. For esp-dl we need
         * the pre-JPEG RGB565 buffer. This requires exposing the raw
         * ISP buffer from camera_driver. See Step 2 for the modification. */

        ESP_LOGI(TAG, "Captured %ux%u frame (%u bytes)", width, height, (unsigned)frame_len);

        /* 3. Run inference -- will be connected after camera_driver exposes raw buffer */

        camera_release_frame();
        camera_stop_streaming();

done:
        /* 4. Resume MJPEG streaming */
        camera_start_streaming();

        int64_t elapsed = (esp_timer_get_time() - t0) / 1000;
        ESP_LOGI(TAG, "Detection cycle complete in %lld ms", elapsed);
    }
}

extern "C" esp_err_t detect_init(void)
{
    BaseType_t ret = xTaskCreate(detect_task_fn, "Detect", 8192,
                                  NULL, 5, &s_detect_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create detect task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Detect task ready (trigger via WS command)");
    return ESP_OK;
}

extern "C" void detect_trigger(void)
{
    if (s_detect_task) {
        ESP_LOGI(TAG, "Inference triggered");
        xTaskNotifyGive(s_detect_task);
    }
}
```

- [ ] **Step 2: Add camera_capture_raw() to camera_driver**

The current `camera_capture_frame()` returns JPEG-encoded data. esp-dl needs the raw RGB565 buffer from ISP. Add a new function to `main/drivers/camera_driver.c`:

```c
esp_err_t camera_capture_raw(void **buf, size_t *len,
                              uint32_t *width, uint32_t *height)
{
    if (s_cam_fd < 0 || s_frame_held) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_current_buf, 0, sizeof(s_current_buf));
    s_current_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    s_current_buf.memory = V4L2_MEMORY_MMAP;

    ESP_RETURN_ON_ERROR(ioctl(s_cam_fd, VIDIOC_DQBUF, &s_current_buf),
                        TAG, "VIDIOC_DQBUF failed");

    if (!(s_current_buf.flags & V4L2_BUF_FLAG_DONE)) {
        ioctl(s_cam_fd, VIDIOC_QBUF, &s_current_buf);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *buf = s_cam_buf[s_current_buf.index];
    *len = s_current_buf.bytesused ? s_current_buf.bytesused : s_cam_buf_size;
    if (width)  *width  = s_cam_width;
    if (height) *height = s_cam_height;

    s_frame_held = true;
    return ESP_OK;
}
```

Add declaration to `main/drivers/camera_driver.h`:

```c
/** Capture raw RGB565 frame from ISP (no JPEG encode). Call camera_release_frame() when done. */
esp_err_t camera_capture_raw(void **buf, size_t *len, uint32_t *width, uint32_t *height);
```

- [ ] **Step 3: Update detect_task.cpp to use camera_capture_raw + run inference**

Replace the frame capture section in `detect_task_fn`:

```cpp
        /* 2. Capture one raw RGB565 frame from ISP */
        camera_start_streaming();
        vTaskDelay(pdMS_TO_TICKS(100)); /* let ISP produce a fresh frame */

        void *raw_buf = NULL;
        size_t raw_len = 0;
        uint32_t width = 0, height = 0;

        esp_err_t cap_ret = camera_capture_raw(&raw_buf, &raw_len, &width, &height);
        if (cap_ret != ESP_OK) {
            ESP_LOGE(TAG, "Raw frame capture failed: %s", esp_err_to_name(cap_ret));
            camera_start_streaming();
            goto done;
        }

        ESP_LOGI(TAG, "Captured raw %ux%u RGB565 (%u bytes)", width, height, (unsigned)raw_len);

        /* 3. Run inference */
        {
            dl::image::img_t img = {
                .data     = raw_buf,
                .width    = (uint16_t)width,
                .height   = (uint16_t)height,
                .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE,
            };

            auto &results = s_detector->run(img);

            ESP_LOGI(TAG, "Inference done: %d detections", (int)results.size());

            /* 4. Package results into protobuf */
            boat_SensorSnapshot snap = boat_SensorSnapshot_init_zero;
            snap.timestamp_us = (uint64_t)esp_timer_get_time();
            snap.detections_count = 0;

            for (const auto &r : results) {
                if (snap.detections_count >= 10) break;  /* max_count from .options */
                boat_Detection *d = &snap.detections[snap.detections_count];
                d->category = r.category;
                d->score    = r.score;
                d->x1       = r.box[0];
                d->y1       = r.box[1];
                d->x2       = r.box[2];
                d->y2       = r.box[3];
                snap.detections_count++;
                ESP_LOGI(TAG, "  [cat:%d score:%.2f box:(%d,%d)-(%d,%d)]",
                         r.category, r.score, r.box[0], r.box[1], r.box[2], r.box[3]);
            }

            /* Publish via pipeline */
            pipeline_publish_sensors(&snap);
        }

        camera_release_frame();
        camera_stop_streaming();
```

- [ ] **Step 4: Add detect_init() call in main.c**

After `pipeline_init()` and before `wifi_init()`:

```c
#include "detect_task.h"
// ...
ESP_LOGI(TAG, "Initializing detection task...");
detect_init();
```

- [ ] **Step 5: Build and verify**

Run: `ninja -C build`

Expected: Build succeeds with detect_task linked against cat_detect and esp-dl.

- [ ] **Step 6: Commit**

```bash
git add main/detect_task.cpp main/detect_task.h main/drivers/camera_driver.c main/drivers/camera_driver.h main/main.c
git commit -m "feat(detect): implement stop-the-world inference with CatDetect"
```

---

### Task 4: Wire up detect trigger from WS command

**Files:**
- Modify: `main/pipeline.c`
- Modify: `main/pipeline.h`

- [ ] **Step 1: Add detect command handler to pipeline**

In `pipeline.c`, add include and handler:

```c
#include "detect_task.h"
```

In `pipeline_handle_incoming()`, add a case for the detect command:

```c
    case boat_BoatMessage_detect_tag:
        ESP_LOGI(TAG, "Detect command received");
        detect_trigger();
        break;
```

- [ ] **Step 2: Build and verify**

Run: `ninja -C build`

Expected: Build succeeds.

- [ ] **Step 3: Commit**

```bash
git add main/pipeline.c
git commit -m "feat(detect): wire detect trigger from WS protobuf command"
```

---

### Task 5: Add dashboard Detect button and bbox rendering

**Files:**
- Modify: `main/dashboard.html`

- [ ] **Step 1: Add Detect button to camera card header**

Find the camera card header in dashboard.html and add a detect button next to existing controls.

- [ ] **Step 2: Add JS to send DetectCommand via WS**

The dashboard already encodes/sends MotorCommand via protobuf. Add a function that encodes a `BoatMessage { detect: {} }` and sends it as a WS binary frame.

Note: The dashboard JS uses nanopb-generated protobuf OR raw encoding. Check the existing motor command send pattern and replicate it for detect.

- [ ] **Step 3: Add bbox rendering on the overlay canvas**

When `msg.sensors.detections` is present and non-empty in the received telemetry:
- Draw each detection as a colored rectangle on the `tof-overlay` canvas
- Label with category + score
- Scale box coordinates from 800x640 to canvas dimensions
- Account for the CSS 180° rotation on `#cam-img`

- [ ] **Step 4: Build, flash, and test end-to-end**

Run: `ninja -C build`
Flash: `python3 -m esptool ... write_flash ...`

Test:
1. Open dashboard at 192.168.1.201
2. Point camera at a cat (or cat picture on phone)
3. Click "Detect" button
4. Verify serial log shows detection results
5. Verify bounding boxes appear on dashboard overlay

- [ ] **Step 5: Commit**

```bash
git add main/dashboard.html
git commit -m "feat(dashboard): add Detect button with bounding box overlay"
```

---

### Task 6: Final verification and cleanup

- [ ] **Step 1: Full build from clean**

```bash
ninja -C build
```

Verify zero warnings related to detect_task, pipeline, or proto.

- [ ] **Step 2: Flash and boot test**

Flash and verify boot log shows:
- `DETECT: Detect task ready (trigger via WS command)`
- No crashes, no WDT resets
- WiFi connects normally

- [ ] **Step 3: Inference soak test**

With dashboard open (MJPEG + WS active):
1. Click Detect 5 times with ~10s gaps
2. Verify each inference completes (serial log shows timing)
3. Verify MJPEG stream resumes after each inference
4. Verify no heap drift (check SystemStatus.heap_free)
5. Run for 10+ minutes with periodic detect triggers

- [ ] **Step 4: Commit final state**

```bash
git add -A
git commit -m "feat(detect): esp-dl cat detection with stop-the-world inference

Adds on-device cat detection using esp-dl pico 224x224 model.
Dashboard button triggers inference, results returned via WS protobuf.
MJPEG stream pauses during inference (~300ms) to prevent DMA collisions."
```
