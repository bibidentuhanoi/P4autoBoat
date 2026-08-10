#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "runtime_schedule.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Apply the schedule's startup-failure policy and return the original error.
 * Critical services force actuators safe; optional services are counted and
 * left unavailable without preventing manual Core 0 control from starting. */
esp_err_t runtime_startup_handle_task_failure(runtime_task_id_t id,
                                              esp_err_t error);

/** True until an optional task's startup has failed. Critical tasks never
 * become optional merely because their creation was attempted. */
bool runtime_startup_feature_available(runtime_task_id_t id);

#ifdef __cplusplus
}
#endif
