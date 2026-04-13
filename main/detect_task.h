#pragma once
#include "esp_err.h"
#include "proto/boat.pb.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** True while inference is running — other tasks should yield. */
extern volatile bool g_inference_active;

/** Initialize the detection task. Call after pipeline_init(). */
esp_err_t detect_init(void);

/** Trigger one inference cycle. Safe to call from any task. */
void detect_trigger(void);

/** Copy the latest cached detections into `out` if younger than `max_age_ms`.
 *  Lets sensor_task re-emit results over WS after WiFi/WS reconnects —
 *  the detection frame lands even if no client was connected the moment
 *  inference completed. `out` must have room for BOAT_MAX_DETECTIONS entries. */
void detect_get_cached_results(boat_Detection *out, pb_size_t *out_count,
                               uint32_t max_age_ms);

#ifdef __cplusplus
}
#endif
