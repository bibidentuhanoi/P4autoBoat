# esp-dl Cat Detection Integration Design

## Goal

Add on-device object detection (cat_detect pico 224x224) to the boat firmware. Manual trigger via dashboard button, results returned via WS protobuf telemetry. Foundation for future ToF-triggered event-driven detection.

## Architecture

### Approach: Stop-the-World Inference

A dedicated `detect_task` owns a CatDetect instance. When triggered:

1. Pause MJPEG stream (stop drain task + stream handler)
2. Capture one RGB565 frame from ISP via existing `camera_capture_frame()`
3. Build `dl::image::img_t{data=rgb565_buf, w=800, h=640, pix_type=RGB565LE}`
4. Run `detect->run(img)` -- preprocessor auto-resizes 800x640 to 224x224, SIMD-accelerated
5. Package results into protobuf `Detection` messages
6. Release frame, resume MJPEG stream
7. Push results through pipeline to WS clients

ISP stays in STREAMON throughout -- only the application-level streaming pauses. Inference has sole access to PSRAM DMA during its window (~300ms). This eliminates concurrent DMA collisions.

### Data Flow

```
Dashboard "Detect" button
  -> WS binary command (0x02)
  -> pipeline_handle_incoming() -> detect_trigger()
  -> detect_task wakes (xTaskNotifyGive)
  -> camera_stop_streaming()
  -> camera_capture_frame() -> raw RGB565 800x640
  -> CatDetect::run(img_t) -> list<result_t>
  -> camera_release_frame()
  -> camera_start_streaming()
  -> encode Detection[] into SensorSnapshot protobuf
  -> pipeline_publish() -> WS -> dashboard
```

## New Files

| File | Purpose |
|------|---------|
| `main/detect_task.cpp` | Owns CatDetect, handles trigger, runs inference, publishes results. `extern "C"` wrappers for C API. |
| `main/detect_task.h` | `detect_init()`, `detect_trigger()` |

## Modified Files

| File | Change |
|------|--------|
| `main/idf_component.yml` | Add `espressif/cat_detect` with `override_path: "../tools/esp-dl/models/cat_detect"` |
| `main/proto/boat.proto` | Add `Detection` message (category, score, x1/y1/x2/y2) and `repeated Detection detections` field to `SensorSnapshot` |
| `main/proto/boat.pb.c/h` | Regenerated from proto |
| `main/main.c` | Call `detect_init()` after pipeline init |
| `main/pipeline.c` | Route command byte 0x02 to `detect_trigger()` in `pipeline_handle_incoming()` |
| `main/dashboard.html` | Add "Detect" button in camera card, send 0x02 command on click, render bounding boxes on canvas overlay |
| `main/CMakeLists.txt` | Add `.cpp` source support for detect_task |

## Model Details

- **Model**: ESPDET_PICO_224_224_CAT (flash rodata, embedded binary)
- **Input**: Any resolution RGB565LE, auto-resized with letterbox to 224x224
- **Output**: `list<result_t>` with category, score (0-1), box[x1,y1,x2,y2] in source image coordinates
- **Thresholds**: score_thr=0.6, nms_thr=0.7 (defaults)
- **PSRAM**: Single bulk allocation at model load (~hundreds of KB), minimal per-inference churn
- **Thread safety**: None -- dedicated task solves this

## detect_task Specifics

- **Stack**: 8192 bytes
- **Priority**: 5 (above sensor=4, ensures CPU during inference)
- **Lazy load**: CatDetect constructed on first trigger, lives forever
- **Idle**: Blocks on `ulTaskNotifyTake()`, zero CPU when not inferring
- **Byte order**: ISP outputs RGB565LE (no SWAP_BYTE in sdkconfig) -> `DL_IMAGE_PIX_TYPE_RGB565LE`

## Protobuf Schema Addition

```protobuf
message Detection {
    int32 category = 1;
    float score = 2;
    int32 x1 = 3;
    int32 y1 = 4;
    int32 x2 = 5;
    int32 y2 = 6;
}

// In SensorSnapshot:
repeated Detection detections = 8;
```

Empty on normal 20Hz telemetry frames. Populated only after inference. Dashboard ignores empty arrays.

## Dashboard UI

- "Detect" button in camera card header (alongside existing controls)
- On click: send WS binary `[0x02]`
- On receiving detections in telemetry: draw bounding boxes on the ToF overlay canvas
- Boxes rendered with category label + score, colored by confidence

## Stability Constraints

- No concurrent DMA during inference (stop-the-world)
- CatDetect lives in one task only (not thread-safe)
- Model loaded once, never freed (avoids PSRAM fragmentation)
- Brief MJPEG freeze (~300ms) during inference is acceptable
- If inference fails, resume streaming and log error -- never crash

## Future Extensions (not in this spec)

- ToF proximity trigger (< 2m threshold, configurable via Kconfig)
- Suspend WiFi app traffic during inference for maximum DMA isolation
- Route planning based on detection + ToF distance fusion
- Multiple model support (pedestrian, boat, obstacle)
