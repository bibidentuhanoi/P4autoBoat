#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** True while TrainingLog is actively inside an fs_sdcard_write() call (JPEG
 *  or sidecar). For directly correlating other tasks' slow cycles against a
 *  concurrent SD write -- e.g. task_sensor_snapshot samples this into its
 *  own overrun log, since both share the same physical SDMMC transport (SD
 *  card and the C6/esp_hosted link are different slots on one controller).
 *  Only training_log_task.c writes it; anyone may read it. Matches the
 *  existing g_inference_active pattern (detect_task.h) for this kind of
 *  cross-task "is X happening right now" signal. */
extern volatile bool g_training_log_sd_active;

/** True while TrainingLog is inside camera_capture_copy() (camera DQBUF +
 *  HW JPEG encode). hw-confirmed 2026-08-11: a pipeline.c pb_encode() call
 *  measured 18ms for a 1915-byte message (nanopb should be microseconds)
 *  during this exact window, with s_msg itself confirmed in internal SRAM
 *  (not the shared-PSRAM-buffer explanation) -- mechanism not yet proven,
 *  this flag is what nails the correlation down before guessing further. */
extern volatile bool g_training_log_camera_active;

/**
 * @brief Initialize the dataset-capture task.
 *
 * Call after sensor_task_init() (needs sensor_tof_cache_load()). Does NOT
 * require SD or the camera to be ready yet -- both are checked per-capture,
 * matching the rest of this project's "SD is soft-optional" philosophy.
 * Allocates its JPEG/sidecar buffers from PSRAM once here, not per-capture.
 */
esp_err_t training_log_init(void);

/**
 * @brief Trigger one dataset capture: a full-quality JPEG + a time-synced
 *        sensor sidecar (.txt) written to SD. Safe to call from any task.
 *        Fire-and-forget -- no ack; failures are logged and counted via
 *        RUNTIME_TASK_TRAINING_LOG's runtime metrics, not surfaced to the
 *        caller.
 */
void training_log_trigger(void);

#ifdef __cplusplus
}
#endif
