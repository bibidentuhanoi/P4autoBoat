#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** True while inference is running — other tasks should yield. */
extern volatile bool g_inference_active;

/** Initialize the detection task. Call after pipeline_init(). */
esp_err_t detect_init(void);

/** Trigger one inference cycle. Safe to call from any task. */
void detect_trigger(void);

#ifdef __cplusplus
}
#endif
