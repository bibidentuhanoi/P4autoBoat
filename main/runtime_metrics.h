#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "runtime_schedule.h"

typedef enum {
    RUNTIME_EVENT_DEADLINE_MISS,
    RUNTIME_EVENT_COMMAND_OVERWRITE,
    RUNTIME_EVENT_INVALID_COMMAND,
    RUNTIME_EVENT_SENSOR_SKIP,
    RUNTIME_EVENT_SENSOR_ERROR,
    RUNTIME_EVENT_FEATURE_DISABLED,
    RUNTIME_EVENT_COUNT,
} runtime_metric_event_t;

typedef struct {
    uint32_t runs;
    uint32_t max_exec_us;
    uint32_t max_gap_us;
    uint32_t max_jitter_us;
    uint32_t deadline_misses;
    uint32_t stack_free_words;
    uint32_t exec_histogram[8];
    uint32_t events[RUNTIME_EVENT_COUNT];
} runtime_metric_snapshot_t;

void runtime_metrics_init(void);
void runtime_metrics_record_cycle(runtime_task_id_t id, uint64_t scheduled_us,
                                  uint64_t started_us, uint64_t finished_us);
void runtime_metrics_cycle_begin(runtime_task_id_t id, uint64_t scheduled_us,
                                 uint64_t started_us);
void runtime_metrics_cycle_end(runtime_task_id_t id, uint64_t finished_us);
void runtime_metrics_count(runtime_task_id_t id, runtime_metric_event_t event);
void runtime_metrics_set_stack(runtime_task_id_t id, uint32_t words);
void runtime_metrics_snapshot(runtime_task_id_t id, runtime_metric_snapshot_t *out);
bool runtime_metrics_core_attribution_complete(const uint8_t *affinity_masks,
                                               uint32_t task_count);

/* Host-test support; production calls runtime_metrics_init() once at startup. */
void runtime_metrics_reset_for_test(void);

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void runtime_metrics_register_handle(runtime_task_id_t id, TaskHandle_t handle);
void task_runtime_diagnostics(void *arg);
#endif
