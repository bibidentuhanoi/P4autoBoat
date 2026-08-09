#include "runtime_schedule.h"

#include <stddef.h>

static const runtime_task_spec_t s_schedule[RUNTIME_TASK_COUNT] = {
    [RUNTIME_TASK_CONTROL] = {"Control", 4096, 10, 0, 10000, 10000, true},
    [RUNTIME_TASK_ARM_SEQUENCE] = {"ArmSeq", 4096, 9, 0, 0, 0, true},
    [RUNTIME_TASK_SENSOR_BUS] = {"SensorBus", 8192, 8, 0, 20000, 20000, true},
    [RUNTIME_TASK_FUSION] = {"Fusion", 4096, 7, 0, 0, 40000, true},
    [RUNTIME_TASK_GPS] = {"GPS", 4096, 6, 0, 0, 0, false},
    [RUNTIME_TASK_DETECT] = {"Detect", 32768, 7, 1, 0, 0, false},
    [RUNTIME_TASK_CAMERA_DRAIN] = {"CamDrain", 2048, 6, 1, 0, 0, false},
    [RUNTIME_TASK_SNAPSHOT] = {"Snapshot", 16384, 5, 1, 50000, 50000, false},
    [RUNTIME_TASK_TOF_PROCESS] = {"ToFProc", 6144, 4, 1, 200000, 0, false},
    [RUNTIME_TASK_WS_TX] = {"WS_TX", 8192, 3, 1, 0, 0, false},
    [RUNTIME_TASK_DIAGNOSTICS] = {"Diagnostics", 4096, 2, 1, 1000000, 0, false},
    [RUNTIME_TASK_TRAINING_LOG] = {"TrainingLog", 8192, 2, 1, 0, 0, false},
    [RUNTIME_TASK_STATUS_LED] = {"StatusLED", 2048, 2, 1, 0, 0, false},
};

const runtime_task_spec_t *runtime_schedule_get(runtime_task_id_t id)
{
    if (id < 0 || id >= RUNTIME_TASK_COUNT) {
        return NULL;
    }
    return &s_schedule[id];
}

bool runtime_schedule_validate(void)
{
    for (runtime_task_id_t id = RUNTIME_TASK_CONTROL; id < RUNTIME_TASK_COUNT; ++id) {
        const runtime_task_spec_t *spec = &s_schedule[id];
        if (!spec->name || !spec->stack_size || !spec->priority ||
            (spec->core != 0 && spec->core != 1)) {
            return false;
        }
    }

    return s_schedule[RUNTIME_TASK_CONTROL].core == 0 &&
           s_schedule[RUNTIME_TASK_CONTROL].critical &&
           s_schedule[RUNTIME_TASK_SENSOR_BUS].core == 0 &&
           s_schedule[RUNTIME_TASK_SENSOR_BUS].critical;
}
