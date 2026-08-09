#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RUNTIME_TASK_CONTROL,
    RUNTIME_TASK_ARM_SEQUENCE,
    RUNTIME_TASK_SENSOR_BUS,
    RUNTIME_TASK_FUSION,
    RUNTIME_TASK_GPS,
    RUNTIME_TASK_DETECT,
    RUNTIME_TASK_CAMERA_DRAIN,
    RUNTIME_TASK_SNAPSHOT,
    RUNTIME_TASK_TOF_PROCESS,
    RUNTIME_TASK_WS_TX,
    RUNTIME_TASK_DIAGNOSTICS,
    RUNTIME_TASK_TRAINING_LOG,
    RUNTIME_TASK_STATUS_LED,
    RUNTIME_TASK_COUNT,
    RUNTIME_TASK_WAYPOINT,
    RUNTIME_TASK_ML_CONTROL,
} runtime_task_id_t;

typedef struct {
    const char *name;
    uint32_t stack_size;
    uint8_t priority;
    int8_t core;
    uint32_t period_us;
    uint32_t deadline_us;
    bool critical;
} runtime_task_spec_t;

const runtime_task_spec_t *runtime_schedule_get(runtime_task_id_t id);
bool runtime_schedule_validate(void);
