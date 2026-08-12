# Continuous Synchronized Dataset Recording Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a 30-second, safety-bounded BoatLog recorder that captures 20 FPS camera data plus causal full-fidelity ToF, IMU/fusion, GPS, and manual-control history, then exports a normalized 10 Hz dataset through a Python notebook.

**Architecture:** Keep Feature 1 intact and add Feature 2 behind a separate command mode. The existing Core 1 CamDrain task becomes the exclusive camera producer during recording; bounded preallocated channels feed the existing low-priority TrainingLog task, which is the sole append-only BoatLog writer. Pure-C format/policy/channel modules receive host coverage, while a Python library and thin notebook recover BoatLog files and export JPEGs plus normalized CSV tables.

**Tech Stack:** ESP-IDF 5.4, FreeRTOS, C11 atomics, V4L2/ESP camera driver, FATFS/SDMMC, nanopb 0.4.9.1, protobuf.js, Python 3, `struct`, `csv`, `zlib`, pandas/Jupyter for interactive inspection, pytest/unittest host harnesses.

## Global Constraints

- A session lasts exactly 30 seconds by one absolute P4 microsecond deadline; dropped frames never extend it.
- Camera target is 20 FPS / 600 attempts, with adaptive 15 FPS then 10 FPS and a 10 FPS hardware acceptance floor.
- Fused dataset export targets approximately 10 Hz / 300 samples.
- Live MJPEG is unavailable during recording and is restored after success and every recoverable failure.
- Duplicate recording triggers during STARTING, RECORDING, or FINALIZING are ignored without restarting or extending the session.
- Existing empty `TrainingLogCommand` payloads retain Feature 1 single-capture behavior.
- Full four-target 8x8 ToF data remains intact: 256 distance, 256 sigma, 256 status, and 64 target-count values per sensor generation.
- No high-rate raw record is protobuf-encoded on the P4.
- Core 0 control/safety work, physical sensor acquisition, and incoming WiFi/ESP-NOW control traffic are never throttled for recording.
- Only heavy outbound dashboard telemetry may reduce while recording.
- Camera/sensor/control producers never open, append, flush, close, or rename files.
- The TrainingLog writer is the sole BoatLog appender and remains Core 1 priority 2.
- Producer submission is allocation-free and nonblocking after startup; full channels drop/count recording data rather than wait.
- The SD card and C6 share SDMMC. Writer chunks must be hardware-tuned and yield between transactions; recording terminates before command age approaches the existing 400 ms failsafe.
- A missing/slow/failing SD card disables or terminates recording only; manual driving remains available.
- On-card names remain FAT 8.3-compatible: `training/00021.tmp` becomes `training/00021.blg`; long `session_0021` names exist only in laptop exports.
- Record matching uses P4 monotonic acquisition/acceptance/application timestamps, never SD write time or laptop arrival time.
- Matching is causal: no camera, sensor, or control record completed after a decision instant may label that decision.
- Initial accepted camera-to-ToF delta is at most 75 ms.
- Torn/rejected ToF grids never enter BoatLog as valid grids.
- Motor/rudder command latency must remain at most 20 ms, Control cadence remains 100 Hz, recording-induced failsafes remain zero, and C6/SDIO restarts remain zero.
- PicoDet inference, autonomous avoidance, waypoint driving, browser-side real-time fusion, on-device MP4, JSON export, and closest-target ToF reduction are out of scope.
- Preserve the unrelated existing `main/drivers/tof_driver.c`, `.superpowers/brainstorm/`, and `boatMainReimagine.FCStd` working-tree state unless a later task explicitly needs an overlapping line and the user approves it.

## File Structure

| File | Responsibility |
|---|---|
| `main/boatlog_format.h/.c` | Versioned little-endian on-disk constants, record headers, payload structs, CRC32, and scan validation independent of FreeRTOS/filesystem code |
| `main/recorder_policy.h/.c` | Pure state machine, 30-second deadline, duplicate-trigger behavior, adaptive FPS, and termination decisions |
| `main/training_record_bus.h/.c` | Preallocated PSRAM camera pool plus bounded nonblocking record channels and drop counters |
| `main/file_system.h/.c` | Long-lived SD stream, bounded append chunks, preallocation, final truncate/rename, and lock timing |
| `main/training_log_task.h/.c` | Preserve Feature 1; orchestrate Feature 2 state, session naming, writer loop, status snapshots, and pressure handling |
| `main/camera_stream.h/.c` | Exclusive-recorder mode and CamDrain-to-camera-pool production at the current target FPS |
| `main/drivers/camera_driver.h/.c` | Return the completed-frame timestamp with copied JPEGs |
| `main/sensor_task.c`, `main/sensor_fusion.c`, `main/drivers/gps_driver.c`, `main/motor_control.c` | Submit bounded timestamped source records after existing coherent publication/acceptance/application points |
| `main/proto/boat.proto`, generated bindings, `main/pipeline.c` | Compatible command mode and compact recorder status |
| `main/dashboard.html`, `tools/espnow_drive.py` | Separate single-capture/30-second controls and recorder status; pause live camera UI during recording |
| `tools/boatlog/boatlog.py` | BoatLog recovery, causal matching, JPEG extraction, normalized CSV export, and optional `.txt` sidecars |
| `tools/boatlog/BoatLog_Export.ipynb` | Thin interactive notebook that calls the tested Python library |
| `tests/` | Pure-C format/policy/channel tests, compiled runtime behavior harnesses, UI tests, converter tests, fixtures, and hardware checklist |

---

### Task 1: Freeze the BoatLog v1 binary contract

**Files:**
- Create: `main/boatlog_format.h`
- Create: `main/boatlog_format.c`
- Modify: `main/CMakeLists.txt`
- Create: `tests/test_boatlog_format.c`
- Create: `tests/test_boatlog_format.py`

**Interfaces:**
- Consumes: standard fixed-width integer types only.
- Produces: `boatlog_file_header_v1_t`, `boatlog_record_header_v1_t`, `boatlog_record_type_t`, all v1 payload structs, `boatlog_crc32_*()`, `boatlog_record_total_size()`, `boatlog_validate_file_header()`, and `boatlog_validate_record()`.

- [ ] **Step 1: Write the failing format test**

Create a C test that asserts fixed sizes, enum values, little-endian fixture bytes, incremental CRC equivalence, payload limits, unknown-record forward compatibility, and rejection of bad magic/length/CRC:

```c
static const uint8_t k_payload[] = {0x10, 0x20, 0x30, 0x40};

int main(void)
{
    assert(BOATLOG_FORMAT_VERSION == 1);
    assert(BOATLOG_RECORD_CAMERA_JPEG == 1);
    assert(BOATLOG_RECORD_SESSION_END == 11);
    assert(sizeof(boatlog_record_header_v1_t) == 32);

    boatlog_record_header_v1_t h;
    assert(boatlog_record_header_init(&h, BOATLOG_RECORD_CAMERA_JPEG,
                                      7, 1234567, sizeof(k_payload)) == ESP_OK);
    assert(boatlog_record_total_size(&h) == 32 + sizeof(k_payload) + 4);

    boatlog_crc32_t crc;
    boatlog_crc32_begin(&crc);
    boatlog_crc32_update(&crc, &h, sizeof(h));
    boatlog_crc32_update(&crc, k_payload, sizeof(k_payload));
    uint32_t split = boatlog_crc32_end(&crc);
    assert(split == boatlog_crc32_record(&h, k_payload));
    assert(boatlog_validate_record(&h, k_payload, split) == ESP_OK);
    assert(boatlog_validate_record(&h, k_payload, split ^ 1U) == ESP_ERR_INVALID_CRC);
    return 0;
}
```

- [ ] **Step 2: Run the test and confirm RED**

Run: `python -m pytest tests/test_boatlog_format.py -q`

Expected: FAIL because `boatlog_format.h/.c` do not exist.

- [ ] **Step 3: Implement the exact v1 format**

Use explicit little-endian scalar fields and `_Static_assert` all sizes. Do not serialize compiler-native aggregate structs without size assertions. Define:

```c
#define BOATLOG_FILE_MAGIC        UINT32_C(0x474C5442) /* BTLG */
#define BOATLOG_RECORD_MAGIC      UINT32_C(0x31434552) /* REC1 */
#define BOATLOG_FORMAT_VERSION    1U
#define BOATLOG_MAX_PAYLOAD_BYTES (256U * 1024U)

typedef enum {
    BOATLOG_RECORD_CAMERA_JPEG = 1,
    BOATLOG_RECORD_TOF_A_GRID = 2,
    BOATLOG_RECORD_TOF_B_GRID = 3,
    BOATLOG_RECORD_IMU_SAMPLE = 4,
    BOATLOG_RECORD_FUSION_RESULT = 5,
    BOATLOG_RECORD_GPS_FIX = 6,
    BOATLOG_RECORD_CONTROL_ACCEPTED = 7,
    BOATLOG_RECORD_CONTROL_APPLIED = 8,
    BOATLOG_RECORD_FRAME_DROPPED = 9,
    BOATLOG_RECORD_RECORDER_EVENT = 10,
    BOATLOG_RECORD_SESSION_END = 11,
} boatlog_record_type_t;

typedef struct __attribute__((packed)) {
    uint32_t magic_le;
    uint16_t format_version_le;
    uint16_t record_type_le;
    uint32_t flags_le;
    uint32_t sequence_le;
    uint64_t timestamp_us_le;
    uint32_t payload_length_le;
    uint32_t reserved_le;
} boatlog_record_header_v1_t;
```

The file header is exactly 128 bytes:

```c
typedef struct __attribute__((packed)) {
    uint32_t magic_le;
    uint16_t format_version_le;
    uint16_t header_bytes_le;
    uint32_t session_id_le;
    uint16_t trigger_source_le;
    uint16_t requested_fps_le;
    uint64_t start_us_le;
    uint32_t camera_width_le;
    uint32_t camera_height_le;
    uint16_t tof_zones_le;
    uint16_t tof_targets_per_zone_le;
    uint32_t calibration_version_le;
    uint8_t firmware_git_sha[20];
    uint8_t reserved[68];
} boatlog_file_header_v1_t;

_Static_assert(sizeof(boatlog_file_header_v1_t) == 128, "BoatLog file header drift");
_Static_assert(sizeof(boatlog_record_header_v1_t) == 32, "BoatLog record header drift");
```

Use these exact payload prefixes:

```c
typedef struct __attribute__((packed)) {
    uint32_t frame_id_le;
    uint32_t attempt_id_le;
    uint32_t width_le;
    uint32_t height_le;
    uint32_t pixel_format_le;
    uint32_t jpeg_length_le;
    uint32_t target_fps_le;
    uint32_t reserved_le;
} boatlog_camera_v1_t;

typedef struct __attribute__((packed)) {
    uint32_t generation_le;
    uint8_t stream_count;
    uint8_t sensor_id;
    uint16_t flags_le;
    uint64_t data_ready_us_le;
    uint64_t read_started_us_le;
    uint64_t read_completed_us_le;
    int32_t distance_mm_le[256];
    uint32_t sigma_mm_le[256];
    uint32_t target_status_le[256];
    uint32_t targets_detected_le[64];
} boatlog_tof_grid_v1_t;

typedef struct __attribute__((packed)) {
    uint32_t sequence_le;
    int16_t ax_le, ay_le, az_le;
    int16_t gx_le, gy_le, gz_le;
    int16_t mx_le, my_le, mz_le;
    uint16_t validity_flags_le;
} boatlog_imu_v1_t;

typedef struct __attribute__((packed)) {
    uint32_t generation_le;
    uint32_t source_imu_sequence_le;
    uint64_t source_capture_us_le;
    uint32_t pitch_f32_le;
    uint32_t roll_f32_le;
    uint32_t heading_f32_le;
    uint32_t validity_flags_le;
} boatlog_fusion_v1_t;
```

Define GPS/control/event/session payloads with the same rule: fixed-width little-endian scalars only, an explicit generation/sequence, validity/flags, and no pointer, `bool`, `size_t`, enum-sized field, native `float`/`double`, or compiler padding on disk. IEEE-754 values are bit-cast to `uint32_t`/`uint64_t` and then converted to little endian. GPS includes every field in `gps_fix_t` plus protocol authority and receive time. Accepted control includes source, command kind, acceptance result, axes/raw pulse/event, and receive time. Applied control includes the complete `control_decision_t` state plus application time. Frame-drop/event/session records contain stable numeric reason enums and the counters named in the specification.

CRC32 covers the complete 32-byte record header followed by exactly `payload_length` bytes; the little-endian CRC32 trailer is not included in its own checksum. Use polynomial `0xEDB88320`, initial `0xFFFFFFFF`, final XOR `0xFFFFFFFF`, so Python `zlib.crc32(header + payload) & 0xffffffff` matches.

Define fixed v1 payload structs for camera metadata, ToF, IMU, fusion, GPS, accepted/applied control, dropped-frame event, recorder event, and session end. Keep ToF values lossless with `int32_t distance_mm[256]`, `uint32_t sigma_mm[256]`, `uint32_t target_status[256]`, and `uint32_t targets_detected[64]`. Camera payload is `boatlog_camera_v1_t` immediately followed by `jpeg_length` bytes.

- [ ] **Step 4: Run focused tests and compile warnings as errors**

Run: `python -m pytest tests/test_boatlog_format.py -q`

Expected: PASS, including `cc -std=c11 -Wall -Wextra -Werror`.

- [ ] **Step 5: Commit the format contract**

```bash
git add main/boatlog_format.h main/boatlog_format.c main/CMakeLists.txt tests/test_boatlog_format.c tests/test_boatlog_format.py
git commit -m "feat(dataset): define BoatLog v1 format"
```

---

### Task 2: Implement the pure recorder policy

**Files:**
- Create: `main/recorder_policy.h`
- Create: `main/recorder_policy.c`
- Modify: `main/CMakeLists.txt`
- Create: `tests/test_recorder_policy.c`
- Create: `tests/test_recorder_policy.py`

**Interfaces:**
- Consumes: monotonic `now_us`, queue percentage, command age, transport health, and SD-write duration samples.
- Produces: `recorder_policy_t`, `recorder_policy_trigger()`, `recorder_policy_frame_due()`, `recorder_policy_note_pressure()`, `recorder_policy_begin_finalizing()`, `recorder_policy_finish()`, and `recorder_policy_status()`.

- [ ] **Step 1: Write behavior-first tests**

Cover the exact state graph and boundaries:

```c
recorder_policy_t p;
recorder_policy_init(&p);
assert(recorder_policy_trigger(&p, 1000000) == RECORDER_TRIGGER_ACCEPTED);
assert(p.state == RECORDER_STARTING);
recorder_policy_started(&p, 1000000);
assert(p.stop_us == 31000000);
assert(recorder_policy_trigger(&p, 2000000) == RECORDER_TRIGGER_ALREADY_ACTIVE);
assert(!recorder_policy_should_stop(&p, 30999999));
assert(recorder_policy_should_stop(&p, 31000000));

recorder_policy_note_queue(&p, 76);  /* sustained high-water window */
assert(recorder_policy_target_fps(&p) == 15);
recorder_policy_note_queue(&p, 91);
assert(recorder_policy_target_fps(&p) == 10);
```

Also prove that a dropped frame increments counters without moving `stop_us`, 600 attempts ends capture even if the wall deadline has not arrived, safe low pressure recovers only one FPS step after a hysteresis window, command age above 150 ms pauses writes, command age above 250 ms terminates recording before the 400 ms failsafe, and a transport fault terminates.

- [ ] **Step 2: Run and confirm RED**

Run: `python -m pytest tests/test_recorder_policy.py -q`

Expected: FAIL on missing policy files.

- [ ] **Step 3: Implement fixed thresholds and hysteresis**

Define no dynamic allocation and no FreeRTOS dependencies:

```c
#define RECORDER_DURATION_US       UINT64_C(30000000)
#define RECORDER_MAX_ATTEMPTS      600U
#define RECORDER_WRITE_PAUSE_AGE_US 150000U
#define RECORDER_ABORT_AGE_US       250000U

typedef enum { RECORDER_IDLE, RECORDER_STARTING, RECORDER_RECORDING,
               RECORDER_FINALIZING } recorder_state_t;
typedef enum { RECORDER_FPS_20 = 20, RECORDER_FPS_15 = 15,
               RECORDER_FPS_10 = 10 } recorder_fps_t;
```

Use absolute camera deadlines derived from `1,000,000 / target_fps`; do not create a timer task. Require five consecutive pressure samples to step down and ten consecutive safe samples to step up, preventing oscillation.

- [ ] **Step 4: Run focused tests**

Run: `python -m pytest tests/test_recorder_policy.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add main/recorder_policy.h main/recorder_policy.c main/CMakeLists.txt tests/test_recorder_policy.c tests/test_recorder_policy.py
git commit -m "feat(dataset): add bounded recorder policy"
```

---

### Task 3: Add bounded preallocated recording channels

**Files:**
- Create: `main/training_record_bus.h`
- Create: `main/training_record_bus.c`
- Modify: `main/CMakeLists.txt`
- Create: `tests/test_training_record_bus.c`
- Create: `tests/test_training_record_bus.py`

**Interfaces:**
- Consumes: caller-provided PSRAM camera slot storage at startup and fixed payload records from source tasks.
- Produces: `training_record_bus_init()`, camera acquire/commit/cancel/pop/release APIs, source-specific `try_push` APIs, `training_record_bus_pop_next()`, and immutable counters.

- [ ] **Step 1: Write tests for queue isolation and memory ordering**

Use a host backing allocation and prove:

- 32 camera slots of 64 KiB each can be acquired, committed, popped, and released in order;
- a 65 KiB JPEG is rejected as `TRAINING_DROP_OVERSIZE`;
- camera exhaustion does not prevent ToF/IMU/GPS/control submission;
- each source preserves its sequence and timestamp;
- full source channels return `false` immediately and increment only that source's drop counter;
- consumer never observes a partially committed payload;
- 100,000 producer/consumer iterations pass under pthread stress.

The camera slot constants are:

```c
#define TRAINING_CAMERA_SLOT_COUNT 32U
#define TRAINING_CAMERA_SLOT_BYTES (64U * 1024U)
```

At observed 17–18 KiB JPEGs this holds more than 1.5 seconds at 20 FPS; an unusually large frame is explicitly dropped instead of allocating.

- [ ] **Step 2: Run and confirm RED**

Run: `python -m pytest tests/test_training_record_bus.py -q`

Expected: FAIL on missing module.

- [ ] **Step 3: Implement channels using lock-free 32-bit indices**

Use C11 `atomic_uint` head/tail indices with release publication and acquire consumption. Do not use 64-bit atomics on ESP32-P4. Each slot owns a normal payload plus a 32-bit committed generation. Camera is one producer (CamDrain) and one consumer (TrainingLog writer). ToF, IMU, fusion, and GPS each receive their own SPSC channel. Accepted-control events use a bounded `portMUX_TYPE`-protected ring because WiFi and ESP-NOW callbacks are multiple producers; the critical section copies only the fixed control payload and never waits on storage.

Allocate the 2 MiB camera payload block once with `heap_caps_malloc(..., MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)` in firmware; allow caller-provided memory under `TRAINING_RECORD_BUS_HOST_TEST`.

- [ ] **Step 4: Run focused tests and ThreadSanitizer where available**

Run: `python -m pytest tests/test_training_record_bus.py -q`

Expected: PASS. The wrapper may skip `-fsanitize=thread` if the compiler lacks it, but the ordinary stress test remains mandatory.

- [ ] **Step 5: Commit**

```bash
git add main/training_record_bus.h main/training_record_bus.c main/CMakeLists.txt tests/test_training_record_bus.c tests/test_training_record_bus.py
git commit -m "feat(dataset): add bounded recording channels"
```

---

### Task 4: Add a long-lived bounded SD stream API

**Files:**
- Modify: `main/file_system.h`
- Modify: `main/file_system.c`
- Create: `tests/test_sdcard_stream.c`
- Create: `tests/test_sdcard_stream.py`
- Create: `tests/sdcard_stream_stubs/` headers required by the compiled harness

**Interfaces:**
- Consumes: validated SD-relative paths, reserve length, data spans, and a maximum physical write chunk.
- Produces:

```c
typedef struct fs_sdcard_stream fs_sdcard_stream_t;
typedef enum {
    FS_SDCARD_CHUNK_CONTINUE = 0,
    FS_SDCARD_CHUNK_PAUSE = 1,
    FS_SDCARD_CHUNK_ABORT = 2,
} fs_sdcard_chunk_action_t;
typedef fs_sdcard_chunk_action_t (*fs_sdcard_chunk_cb)(void *ctx,
                                                       size_t bytes_written,
                                                       uint32_t duration_us);
esp_err_t fs_sdcard_stream_prepare(fs_sdcard_stream_t *s, const char *tmp_path,
                                   size_t reserve_bytes);
esp_err_t fs_sdcard_stream_append(fs_sdcard_stream_t *s, const void *data,
                                  size_t len, size_t max_chunk,
                                  fs_sdcard_chunk_cb cb, void *ctx);
esp_err_t fs_sdcard_stream_finalize(fs_sdcard_stream_t *s,
                                    const char *final_path);
void fs_sdcard_stream_abort(fs_sdcard_stream_t *s);
```

Expose storage sufficient for stack/static ownership without exposing `FILE`:

```c
struct fs_sdcard_stream {
    void *file;
    size_t logical_size;
    size_t reserved_size;
    bool open;
    bool preallocated;
    char tmp_path[64];
};
```

- [ ] **Step 1: Write a fake-filesystem behavior harness**

Prove path traversal is rejected, one file remains open across many appends, no fake `write()` exceeds `max_chunk`, the callback runs between chunks, logical length is truncated on finalize, `.tmp` is renamed atomically, and abort closes without rename. Inject a short write and confirm `ESP_FAIL` with the stream unusable afterward.

- [ ] **Step 2: Run and confirm RED**

Run: `python -m pytest tests/test_sdcard_stream.py -q`

Expected: FAIL because the streaming API is absent.

- [ ] **Step 3: Implement the streaming API**

Reuse the existing SD path validator and filesystem mutex. Keep `FILE *` opaque behind the header-defined storage. Set the stream unbuffered so the caller's chunk boundary reaches FATFS. `prepare()` opens `wb+`, attempts `ftruncate(fileno(file), reserve_bytes)`, seeks back to zero, and records whether reservation succeeded. Reservation failure is logged and returned only for real I/O failure; unsupported preallocation falls back to a non-preallocated stream and is exposed in status.

`append()` takes/releases the filesystem lock per chunk, measures each chunk, calls the supplied pressure callback after releasing the lock, and stops before issuing the next chunk when the callback says pause/abort. Start with `TLOG_SD_CHUNK_BYTES=4096`; hardware Task 12 tunes it downward if a chunk exceeds the 20 ms control-latency budget.

`finalize()` flushes, `fsync()`s once, truncates to logical bytes, closes, and renames `.tmp` to `.blg`. It never syncs per record or per frame.

- [ ] **Step 4: Run focused filesystem tests**

Run: `python -m pytest tests/test_sdcard_stream.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add main/file_system.h main/file_system.c tests/test_sdcard_stream.c tests/test_sdcard_stream.py tests/sdcard_stream_stubs
git commit -m "feat(dataset): add bounded SD stream writer"
```

---

### Task 5: Add exclusive camera recording through CamDrain

**Files:**
- Modify: `main/drivers/camera_driver.h`
- Modify: `main/drivers/camera_driver.c`
- Modify: `main/camera_stream.h`
- Modify: `main/camera_stream.c`
- Modify: `main/training_record_bus.h`
- Create: `tests/test_camera_recording_mode.c`
- Create: `tests/test_camera_recording_mode.py`

**Interfaces:**
- Consumes: camera slots from `training_record_bus`, target FPS from `recorder_policy`, and an absolute stop deadline.
- Produces:

```c
typedef struct {
    bool (*acquire_slot)(void *ctx, uint32_t *slot, uint8_t **dst, size_t *cap);
    void (*commit_slot)(void *ctx, uint32_t slot, size_t len,
                        uint32_t width, uint32_t height, uint64_t capture_us);
    void (*cancel_slot)(void *ctx, uint32_t slot, training_drop_reason_t reason);
    uint32_t (*target_fps)(void *ctx);
} camera_record_sink_t;

esp_err_t camera_stream_begin_recording(const camera_record_sink_t *sink, void *ctx);
esp_err_t camera_stream_end_recording(void);
bool camera_stream_recording_active(void);
```

- [ ] **Step 1: Write a compiled fake-camera test**

Demonstrate that begin-recording makes a connected MJPEG handler exit, prevents new `/stream` acquisition with `ESP_ERR_INVALID_STATE`, makes CamDrain capture to the sink at absolute 20/15/10 FPS deadlines, commits the V4L2 completion timestamp, cancels a slot on capture failure, and resumes ordinary drain/stream behavior after end-recording.

- [ ] **Step 2: Run and confirm RED**

Run: `python -m pytest tests/test_camera_recording_mode.py -q`

Expected: FAIL on missing recording-mode interface.

- [ ] **Step 3: Extend the camera-copy timestamp contract**

Change the copied-capture API to:

```c
esp_err_t camera_capture_copy(uint8_t *dst, size_t capacity, size_t *out_len,
                              uint32_t *width, uint32_t *height,
                              uint64_t *capture_complete_us);
```

Set `capture_complete_us` immediately after successful V4L2 dequeue/encode completion and before the frame is requeued. Update Feature 1 to pass a timestamp pointer so it keeps working.

- [ ] **Step 4: Implement exclusive mode in the existing CamDrain task**

Do not create another camera task. Store a copied sink descriptor under a short mutex. The stream loop checks exclusive mode every iteration and exits cleanly; the drain task switches from `camera_drain_frame()` to slot acquire → `camera_capture_copy()` → commit. Use `vTaskDelayUntil()`/absolute microsecond scheduling and re-read target FPS after every frame.

- [ ] **Step 5: Run camera tests and Feature 1 regression tests**

Run: `python -m pytest tests/test_camera_recording_mode.py tests/test_training_log.py -q`

Expected: PASS.

- [ ] **Step 6: Measure camera-only throughput before adding SD load**

Build/flash a temporary diagnostics mode that runs the exclusive CamDrain sink for 200 frames with a sink that releases each slot immediately and performs no SD/network work. Record frame encode/copy time distribution, achieved FPS, JPEG byte distribution, and oversize count. The gate is at least 10 FPS camera-only and zero 64 KiB oversize frames; if it fails, stop the plan and revise camera resolution/quality or slot size with the user before implementing the writer. Remove the temporary trigger after capturing the report data; retain bounded metrics useful to Task 12.

- [ ] **Step 7: Commit**

```bash
git add main/drivers/camera_driver.h main/drivers/camera_driver.c main/camera_stream.h main/camera_stream.c main/training_record_bus.h tests/test_camera_recording_mode.c tests/test_camera_recording_mode.py tests/test_training_log.py
git commit -m "feat(dataset): give recorder exclusive camera mode"
```

---

### Task 6: Publish timestamped sensor and control records without blocking Core 0

**Files:**
- Modify: `main/drivers/tof_driver.h`
- Modify: `main/drivers/tof_driver.c`
- Modify: `main/sensor_task.c`
- Modify: `main/sensor_fusion.c`
- Modify: `main/drivers/gps_driver.c`
- Modify: `main/motor_control.c`
- Modify: `main/training_record_bus.h/.c`
- Create: `tests/test_training_record_producers.c`
- Create: `tests/test_training_record_producers.py`

**Interfaces:**
- Consumes: successful existing sensor publication, accepted manual commands, and applied Control decisions.
- Produces: `training_record_try_tof()`, `training_record_try_imu()`, `training_record_try_fusion()`, `training_record_try_gps()`, `training_record_try_control_accepted()`, and `training_record_try_control_applied()` calls at authoritative timestamps.

- [ ] **Step 1: Write behavior tests with a fake record bus**

Assert that:

- a successful, non-torn ToF read publishes once with sensor ID, generation, stream count, data-ready time, read start, and read completion;
- `ESP_ERR_NOT_FINISHED` and `ESP_ERR_INVALID_CRC` publish no valid ToF record;
- each coherent IMU sample is submitted after `sample_snapshot_publish()` with its existing sequence/captured timestamp;
- fusion submission identifies the source IMU sequence/time and output time;
- GPS submission occurs only when `last_update_us` changes;
- rejected commands create no accepted record;
- accepted throttle/rudder/winch/steer/servo/arm events preserve acceptance time and source;
- applied records occur on state change/failsafe transition plus a 100 ms heartbeat, allowing causal forward-fill without 100 identical records per second;
- every fake full-channel return leaves the original producer path successful and increments a recording drop counter only.

- [ ] **Step 2: Run and confirm RED**

Run: `python -m pytest tests/test_training_record_producers.py -q`

Expected: FAIL because producer hooks/metadata are absent.

- [ ] **Step 3: Add explicit ToF acquisition metadata**

Define:

```c
typedef struct {
    uint8_t stream_count;
    uint64_t data_ready_us;
    uint64_t read_started_us;
    uint64_t read_completed_us;
} tof_read_metadata_t;

esp_err_t tof_read_grid(VL53L5CX_Configuration *dev,
                        VL53L5CX_ResultsData *results,
                        const char *label,
                        tof_read_metadata_t *meta);
```

Populate metadata only for the frame actually read. The tear check remains authoritative: a torn result returns `ESP_ERR_INVALID_CRC` and is never submitted.

- [ ] **Step 4: Add nonblocking source hooks**

Place hooks after existing validation/publication points, never around I2C/UART/actuator calls. Control accepted hooks run after `CONTROL_ACCEPTED`; applied hooks run after `control_apply_decision()` and status commit. The hooks perform fixed copies/atomic publication only and return `bool`; callers never retry or wait.

- [ ] **Step 5: Run producer, control, sensor, and runtime regression tests**

Run:

```bash
python -m pytest tests/test_training_record_producers.py \
  tests/test_runtime_architecture.py tests/test_sensor_schedule.py \
  tests/test_control_arbiter.py tests/test_gps_snapshot.py -q
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add main/drivers/tof_driver.h main/drivers/tof_driver.c main/sensor_task.c main/sensor_fusion.c main/drivers/gps_driver.c main/motor_control.c main/training_record_bus.h main/training_record_bus.c tests/test_training_record_producers.c tests/test_training_record_producers.py
git commit -m "feat(dataset): record causal sensor and control events"
```

---

### Task 7: Implement the 30-second TrainingLog writer and recovery-safe finalization

**Files:**
- Modify: `main/training_log_task.h`
- Modify: `main/training_log_task.c`
- Modify: `main/main.c`
- Modify: `main/runtime_metrics.h`
- Modify: `main/runtime_metrics.c`
- Modify: `main/CMakeLists.txt`
- Create: `tests/test_training_recorder_runtime.c`
- Create: `tests/test_training_recorder_runtime.py`

**Interfaces:**
- Consumes: Tasks 1–6 format, policy, record bus, SD stream, camera mode, and source channels.
- Produces:

```c
typedef enum { TRAINING_LOG_SINGLE_CAPTURE = 0,
               TRAINING_LOG_RECORD_30_SECONDS = 1 } training_log_mode_t;
typedef enum { TRAINING_TRIGGER_WIFI = 1,
               TRAINING_TRIGGER_ESPNOW = 2 } training_trigger_source_t;

typedef struct {
    recorder_state_t state;
    uint32_t session_id;
    uint32_t remaining_ms;
    uint32_t frames_attempted;
    uint32_t frames_saved;
    uint32_t frames_dropped;
    uint16_t effective_fps_x100;
    uint8_t queue_percent;
    uint16_t termination_reason;
    uint32_t generation;
} training_log_status_t;

void training_log_trigger(void); /* existing Feature 1 */
esp_err_t training_log_trigger_mode(training_log_mode_t mode,
                                    training_trigger_source_t source);
bool training_log_status_read(training_log_status_t *out);
bool training_log_heavy_telemetry_allowed(void);
```

- [ ] **Step 1: Write a compiled runtime harness**

Use fake time, camera sink, file stream, command age, and source queues. Prove:

- Feature 1 still writes one JPEG plus one `.txt` sidecar;
- Feature 2 prepares `training/NNNNN.tmp`, starts exclusive camera mode, writes a valid file header and records, ignores duplicate triggers, and finalizes as `training/NNNNN.blg` at exactly 30 seconds or 600 attempts;
- sequence ordering is monotonic within each record type;
- writer emits header/payload/CRC through bounded stream appends without constructing a second 200 KiB record copy;
- write pause leaves queues intact; abort at 250 ms command age stops before the 400 ms failsafe;
- normal completion writes `SESSION_END`, finalizes `.blg`, and restores MJPEG;
- I/O/camera failure restores MJPEG, preserves `.tmp`, and reports the exact reason;
- status is a coherent generation snapshot;
- task priority/core remain TrainingLog Core 1 priority 2.

- [ ] **Step 2: Run and confirm RED**

Run: `python -m pytest tests/test_training_recorder_runtime.py -q`

Expected: FAIL on missing Feature 2 orchestration.

- [ ] **Step 3: Split Feature 1 helpers from Feature 2 orchestration inside the existing module**

Keep the existing single-capture behavior and its NVS session collision protection. Initialize the record bus and buffers once. If any required Feature 2 PSRAM allocation fails, mark only continuous recording unavailable; Feature 1 and manual operation continue.

Prepare the next `.tmp` file while IDLE when the card is ready. Estimate a conservative 24 MiB reserve from the fixed 64 KiB × 600 upper bound, but cap reservation to available space and require at least 12 MiB before accepting a session. The actual file is truncated to written bytes on finalize.

- [ ] **Step 4: Implement streaming record writes and pressure feedback**

The writer chooses among nonempty source channels without allowing camera records to starve small sensor/control records. Round-robin one record per source, then one camera record. For a camera record, append header, fixed camera metadata, JPEG bytes, and CRC incrementally. After every 4 KiB chunk, feed measured duration, queue occupancy, transport health, and command age into `recorder_policy`.

During recording, `training_log_heavy_telemetry_allowed()` returns true only for a 2 Hz full-ToF dashboard sample when pressure is safe; basic telemetry remains allowed.

- [ ] **Step 5: Add bounded metrics**

Track attempted/saved/dropped camera frames, source record drops, rejected ToFs, valid source records, peak queue utilization, SD pause count, maximum SD chunk duration, maximum observed command age, effective FPS, and termination reason. Aggregate logs at one second; do not print from each producer.

- [ ] **Step 6: Run runtime and full host suites**

Run:

```bash
python -m pytest tests/test_training_recorder_runtime.py tests/test_training_log.py -q
python -m pytest tests -q
```

Expected: focused PASS; full suite PASS with no new failures.

- [ ] **Step 7: Commit**

```bash
git add main/training_log_task.h main/training_log_task.c main/main.c main/runtime_metrics.h main/runtime_metrics.c main/CMakeLists.txt tests/test_training_recorder_runtime.c tests/test_training_recorder_runtime.py tests/test_training_log.py
git commit -m "feat(dataset): record bounded 30-second BoatLog sessions"
```

---

### Task 8: Extend protobuf commands, status, and telemetry shedding compatibly

**Files:**
- Modify: `main/proto/boat.proto`
- Modify: `main/proto/boat.options`
- Modify: `main/proto/boat.pb.h`
- Modify: `main/proto/boat.pb.c`
- Modify: `proto/boat_pb2.py`
- Modify: `main/pipeline.h`
- Modify: `main/pipeline.c`
- Modify: `main/transports/espnow_protocol.h`
- Modify: `main/transports/espnow_transport.c`
- Modify: `main/sensor_task.c`
- Modify: `tools/gen_proto.sh`
- Create: `tools/gen_nanopb.sh`
- Create: `tests/test_training_log_proto.py`

**Interfaces:**
- Consumes: `training_log_trigger_mode()`, `training_log_status_read()`, and `training_log_heavy_telemetry_allowed()`.
- Produces: `TrainingLogMode`, `TrainingLogStatus`, a `training_log_status` BoatMessage payload, and `pipeline_publish_training_log_status()`.

- [ ] **Step 1: Write wire-compatibility tests before changing the schema**

Test that an empty serialized `TrainingLogCommand` decodes as `SINGLE_CAPTURE`, mode 1 decodes as `RECORD_30_SECONDS`, status round-trips every counter, nanopb and Python use identical field/tag numbers, and dashboard schema drift is detected.

- [ ] **Step 2: Run and confirm RED**

Run: `python -m pytest tests/test_training_log_proto.py -q`

Expected: FAIL because mode/status are absent.

- [ ] **Step 3: Add the compatible schema**

Add:

```proto
enum TrainingLogMode {
  SINGLE_CAPTURE = 0;
  RECORD_30_SECONDS = 1;
}

message TrainingLogCommand {
  TrainingLogMode mode = 1;
}

message TrainingLogStatus {
  uint32 state = 1;
  uint32 session_id = 2;
  uint32 remaining_ms = 3;
  uint32 frames_attempted = 4;
  uint32 frames_saved = 5;
  uint32 frames_dropped = 6;
  uint32 effective_fps_x100 = 7;
  uint32 queue_percent = 8;
  uint32 termination_reason = 9;
}
```

Add `TrainingLogStatus training_log_status = 13;` to `BoatMessage` (retain every existing tag). Regenerate nanopb C with `/opt/esp/python_env/idf5.4_py3.12_env/bin/nanopb_generator -I main/proto -D main/proto main/proto/boat.proto`, then regenerate Python via `tools/gen_proto.sh`. Update `tools/gen_proto.sh` so its nanopb guard permits a deliberate, separate `tools/gen_nanopb.sh` workflow while still detecting accidental drift.

- [ ] **Step 4: Route command source and status**

Pipeline dispatch passes `mode` plus transport source. Add source context rather than guessing from protobuf:

```c
typedef enum {
    PIPELINE_SOURCE_UNKNOWN = 0,
    PIPELINE_SOURCE_WIFI = 1,
    PIPELINE_SOURCE_ESPNOW = 2,
} pipeline_source_t;

void pipeline_handle_incoming_from(const uint8_t *buf, size_t len,
                                   pipeline_source_t source);
void pipeline_handle_incoming(const uint8_t *buf, size_t len); /* compatibility wrapper */
void pipeline_publish_training_log_status(const boat_TrainingLogStatus *status);
```

`ws_transport.c` calls `_from(..., PIPELINE_SOURCE_WIFI)` and `espnow_transport.c` calls `_from(..., PIPELINE_SOURCE_ESPNOW)`. Publish status once per second and immediately on state/termination change. Add `MSG_TRAINING_STATUS = 0x09` classification to the ESP-NOW wrapper; the status payload remains small enough for one field packet.

- [ ] **Step 5: Reduce heavy dashboard telemetry only**

Keep the 20 Hz snapshot cadence for IMU/basic state. While recording, include full ToF protobuf arrays only at 2 Hz and only when `training_log_heavy_telemetry_allowed()` is true. This does not alter ToF acquisition/cache/BoatLog rates.

- [ ] **Step 6: Run protocol, transport, and snapshot tests**

Run:

```bash
python -m pytest tests/test_training_log_proto.py tests/test_pipeline.py \
  tests/test_espnow_transport.py tests/test_runtime_architecture.py -q
```

Expected: PASS and generated bindings clean.

- [ ] **Step 7: Commit**

```bash
git add main/proto/boat.proto main/proto/boat.options main/proto/boat.pb.h main/proto/boat.pb.c proto/boat_pb2.py main/pipeline.h main/pipeline.c main/transports/espnow_protocol.h main/transports/espnow_transport.c main/sensor_task.c tools/gen_proto.sh tools/gen_nanopb.sh tests/test_training_log_proto.py
git commit -m "feat(dataset): add continuous-record command and status"
```

---

### Task 9: Add dashboard and ESP-NOW recorder controls

**Files:**
- Modify: `main/dashboard.html`
- Modify: `tools/espnow_drive.py`
- Modify: `tests/test_dashboard.py`
- Modify: `tests/test_espnow_drive.py`

**Interfaces:**
- Consumes: `TrainingLogCommand.mode`, `TrainingLogStatus`, and `MSG_TRAINING_STATUS` from Task 8.
- Produces: separate `Capture Photo` and `Record 30 Seconds` actions plus visible recording state/counters in both UIs.

- [ ] **Step 1: Add failing UI behavior tests**

Dashboard tests must prove:

- `Capture Photo` sends mode 0;
- `Record 30 Seconds` sends mode 1 exactly once per click;
- repeated clicks while STARTING/RECORDING/FINALIZING are disabled locally but firmware status remains authoritative;
- camera `<img>` source is detached/paused while recording and restored only after IDLE;
- motor/rudder/winch/servo-power handlers remain enabled;
- status renders remaining time, saved/dropped frames, effective FPS, and queue percentage.

ESP-NOW tests must prove the Python UI sends mode 1 through the same framing path, decodes `MSG_TRAINING_STATUS`, exposes it at `/api/status`, and does not stop its existing 15 Hz motor/rudder renewal loop during recording.

- [ ] **Step 2: Run and confirm RED**

Run: `python -m pytest tests/test_dashboard.py tests/test_espnow_drive.py -q`

Expected: FAIL on missing controls/status.

- [ ] **Step 3: Mirror the protobuf schema and implement dashboard behavior**

Update `dashboard.html`'s hand-maintained `protoSchema` in the same commit. Keep the control card active. Replace only the camera panel with `Recording to SD — live view paused` while state is non-IDLE. Do not infer completion from a browser timer; use firmware status.

- [ ] **Step 4: Implement ESP-NOW behavior**

Use generated `proto/boat_pb2.py` to build `BoatMessage(training_log=TrainingLogCommand(mode=1))`. Add a `recording_status` dictionary guarded by the existing BoatLink lock. The field controller remains a control tool: show compact state/counters, not camera or full ToF.

- [ ] **Step 5: Run UI tests**

Run: `python -m pytest tests/test_dashboard.py tests/test_espnow_drive.py -q`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add main/dashboard.html tools/espnow_drive.py tests/test_dashboard.py tests/test_espnow_drive.py
git commit -m "feat(dataset): add 30-second recording controls"
```

---

### Task 10: Build the BoatLog recovery and normalized CSV exporter

**Files:**
- Create: `tools/boatlog/__init__.py`
- Create: `tools/boatlog/boatlog.py`
- Create: `tools/boatlog/BoatLog_Export.ipynb`
- Create: `tests/fixtures/boatlog_v1_clean.blg`
- Create: `tests/fixtures/boatlog_v1_truncated.tmp`
- Create: `tests/test_boatlog_export.py`
- Modify: `requirements.txt`

**Interfaces:**
- Consumes: BoatLog v1 from Task 1.
- Produces: `scan_boatlog()`, `recover_records()`, `match_causal_samples()`, `export_session()`, a CLI, normalized CSV files, extracted JPEGs, optional text sidecars, and a thin notebook.

- [ ] **Step 1: Generate cross-language fixtures and write failing parser tests**

Use the Task 1 C codec to emit a clean fixture with camera/dual-ToF/IMU/fusion/GPS/control records and a second copy truncated halfway through its last JPEG. Assert Python recovers every complete record, rejects a bad CRC, reports interruption without discarding the valid prefix, and refuses payload lengths above 256 KiB.

- [ ] **Step 2: Write matching/export tests**

Construct timestamps that prove:

- the closest future camera is rejected even if numerically nearer;
- latest completed ToF-A and ToF-B are selected independently;
- camera-to-ToF deltas above 75 ms mark a row invalid;
- dropped frame IDs remain gaps;
- full ToF wide columns map `index = zone*4+target` exactly;
- each CSV references existing generation rows;
- optional `.txt` sidecars contain full distances/sigma/status/counts and signed deltas;
- no JSON file is created.

- [ ] **Step 3: Run and confirm RED**

Run: `python -m pytest tests/test_boatlog_export.py -q`

Expected: FAIL because the exporter does not exist.

- [ ] **Step 4: Implement the tested library and CLI**

Use only the standard library for parsing/export (`struct`, `zlib`, `csv`, `pathlib`). Keep pandas in the notebook layer. CLI:

```bash
python -m tools.boatlog.boatlog 00021.blg \
  --output exported/session_0021 --text-sidecars
```

Export exactly:

```text
images/
frames.csv
tof_a.csv
tof_b.csv
imu.csv
fusion.csv
gps.csv
controls.csv
matches.csv
```

`matches.csv` holds generation references and signed deltas; full ToF grids live once in their sensor CSV. Invalid samples remain with `valid=0` and a stable reason code.

- [ ] **Step 5: Add the thin notebook**

Add pinned-purpose entries `pandas` and `jupyter` to `requirements.txt`. The notebook imports `scan_boatlog`, `export_session`, and pandas; it contains cells for selecting a path, showing recovery/session summary, exporting, loading CSV tables, plotting timing-delta histograms, displaying a matched JPEG, and reshaping one ToF generation to `(64, 4)`. Business logic remains in `boatlog.py`, not duplicated in notebook cells.

- [ ] **Step 6: Run exporter tests**

Run: `python -m pytest tests/test_boatlog_export.py -q`

Expected: PASS for clean and interrupted fixtures.

- [ ] **Step 7: Commit**

```bash
git add tools/boatlog tests/fixtures/boatlog_v1_clean.blg tests/fixtures/boatlog_v1_truncated.tmp tests/test_boatlog_export.py requirements.txt
git commit -m "feat(dataset): export BoatLog sessions to synchronized CSV"
```

---

### Task 11: Perform integrated host verification and a real P4 build

**Files:**
- Modify only files required by failures attributable to Tasks 1–10.
- Create: `docs/testing/continuous-dataset-recording-host-report.md`

**Interfaces:**
- Consumes: completed firmware/tooling implementation.
- Produces: reproducible clean host/build evidence and a reviewed diff.

- [ ] **Step 1: Run focused suites together**

```bash
python -m pytest \
  tests/test_boatlog_format.py tests/test_recorder_policy.py \
  tests/test_training_record_bus.py tests/test_sdcard_stream.py \
  tests/test_camera_recording_mode.py tests/test_training_record_producers.py \
  tests/test_training_recorder_runtime.py tests/test_training_log_proto.py \
  tests/test_dashboard.py tests/test_espnow_drive.py \
  tests/test_boatlog_export.py -q
```

Expected: all pass.

- [ ] **Step 2: Run the complete host suite**

Run: `python -m pytest tests -q`

Expected: all tests pass; any environment-only skip is named in the report.

- [ ] **Step 3: Build the P4 firmware from the repository IDF environment**

```bash
export IDF_PYTHON_ENV_PATH=/opt/esp/python_env/idf5.4_py3.12_env
source /opt/esp/idf/export.sh >/dev/null
idf.py reconfigure
idf.py build
```

Expected: successful ESP32-P4 build with no undefined producer/codec/UI schema symbols and no new warnings in project files.

- [ ] **Step 4: Review memory and task budgets**

Record static DRAM/IRAM/PSRAM usage, camera-pool allocation result, minimum free heap after init, each changed task's stack high-water mark, and confirm only the existing CamDrain and TrainingLog tasks implement recording. Verify no producer path calls `fopen`, `fwrite`, `fsync`, or nanopb.

- [ ] **Step 5: Review the full diff against scope**

Confirm no PicoDet/avoidance/waypoint behavior was added, full ToF arrays remain, Feature 1 remains mode 0, and unrelated user changes were not staged.

- [ ] **Step 6: Write and commit the host report**

```bash
git add docs/testing/continuous-dataset-recording-host-report.md
git commit -m "test(dataset): verify continuous recorder integration"
```

---

### Task 12: Tune SD chunks and complete hardware acceptance

**Files:**
- Modify: `main/training_log_task.c` only if measured chunk/FPS thresholds require tuning.
- Create: `docs/testing/continuous-dataset-recording-hardware-report.md`

**Interfaces:**
- Consumes: flashed P4/C6/S3-compatible build and both control paths.
- Produces: measured safe SD chunk size, completed 30-second BoatLog files, notebook exports, and acceptance evidence.

- [ ] **Step 1: Establish the no-recording baseline**

For both WiFi and ESP-NOW, record Control max execution/gap/jitter/misses, command-to-actuator latency, SensorBus/Fusion/ToF metrics, SDIO errors/restarts, free heap, and live telemetry behavior over 60 seconds.

- [ ] **Step 2: Run one WiFi-control recording session**

Drive throttle/rudder continuously, operate winch and servo power, and keep basic dashboard telemetry connected. Verify MJPEG pauses, the duplicate trigger is ignored, the session ends at 30 seconds, and MJPEG restores. Capture recorder counters and all RTM/SDIO logs.

- [ ] **Step 3: Run one ESP-NOW-control recording session**

Repeat the same maneuvers from `espnow_drive.py`. Verify its 15 Hz control renewal continues throughout and compact recording status updates arrive.

- [ ] **Step 4: Tune bounded write chunks from measurements**

Start at 4096 bytes. If any in-flight chunk correlates with command latency above 20 ms or SDIO failure, test 2048, 1024, and 512 bytes in that order. Select the largest chunk satisfying all acceptance constraints in three consecutive 30-second sessions. Do not compensate by raising task priority or relaxing the 20 ms requirement.

- [ ] **Step 5: Exercise degradation/failure paths**

Force queue pressure/slow storage, a full queue, missing ToF generation, camera failure if a safe test seam exists, and power removal mid-session. Confirm the recorder reduces 20→15→10 FPS, drops camera frames before sensor/control records, terminates before command age reaches 250 ms, restores the camera, and leaves a recoverable `.tmp` prefix.

- [ ] **Step 6: Export and inspect every session**

Run the notebook/library on clean `.blg` and interrupted `.tmp` files. Confirm:

```text
duration                       30 seconds
minimum camera frames          300
healthy target                 near 600
fused rows                     approximately 300
accepted camera-ToF delta      <= 75 ms
full ToF columns               256 distance + 256 sigma + 256 status + 64 counts per sensor
recording-induced failsafes    0
C6/SDIO restarts               0
torn ToF records               0 valid
motor/rudder latency           <= 20 ms
```

- [ ] **Step 7: Run back-to-back thermal/leak testing**

Record at least five consecutive 30-second sessions. Compare PSRAM/internal heap and stack high-water marks before/after; session resources must return to stable IDLE values with no monotonic leak.

- [ ] **Step 8: Write the hardware report and commit final measured tuning**

The report names hardware revisions, firmware commit, selected SD chunk size, actual FPS/drop rates, queue peak, command latency, task metrics, SDIO health, recovered files, and any non-blocking limitations.

```bash
git add main/training_log_task.c docs/testing/continuous-dataset-recording-hardware-report.md
git commit -m "test(dataset): validate recorder under live control"
```

---

## Completion Gate

Feature 2 is complete only when Tasks 1–12 are committed, the P4 build passes, both WiFi and ESP-NOW hardware sessions satisfy control/SDIO constraints, a clean and interrupted BoatLog export successfully, full dual-ToF data is present in CSV, and live MJPEG restores after all tested termination paths.
