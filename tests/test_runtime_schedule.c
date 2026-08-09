#include <assert.h>
#include <string.h>

#include "runtime_schedule.h"

static void assert_task(runtime_task_id_t id, const char *name,
                        uint32_t stack_size, uint8_t priority, int8_t core,
                        uint32_t period_us, uint32_t deadline_us, bool critical)
{
    const runtime_task_spec_t *spec = runtime_schedule_get(id);

    assert(spec != NULL);
    assert(strcmp(spec->name, name) == 0);
    assert(spec->stack_size == stack_size);
    assert(spec->priority == priority);
    assert(spec->core == core);
    assert(spec->period_us == period_us);
    assert(spec->deadline_us == deadline_us);
    assert(spec->critical == critical);
}

int main(void)
{
    assert(runtime_schedule_validate());

    assert_task(RUNTIME_TASK_CONTROL, "Control", 4096, 10, 0, 10000, 10000, true);
    assert_task(RUNTIME_TASK_ARM_SEQUENCE, "ArmSeq", 4096, 9, 0, 0, 0, true);
    assert_task(RUNTIME_TASK_SENSOR_BUS, "SensorBus", 8192, 8, 0, 20000, 20000, true);
    assert_task(RUNTIME_TASK_FUSION, "Fusion", 4096, 7, 0, 0, 40000, true);
    assert_task(RUNTIME_TASK_GPS, "GPS", 4096, 6, 0, 0, 0, false);
    assert_task(RUNTIME_TASK_DETECT, "Detect", 32768, 7, 1, 0, 0, false);
    assert_task(RUNTIME_TASK_CAMERA_DRAIN, "CamDrain", 2048, 6, 1, 0, 0, false);
    assert_task(RUNTIME_TASK_SNAPSHOT, "Snapshot", 16384, 5, 1, 50000, 50000, false);
    assert_task(RUNTIME_TASK_TOF_PROCESS, "ToFProc", 6144, 4, 1, 200000, 0, false);
    assert_task(RUNTIME_TASK_WS_TX, "WS_TX", 8192, 3, 1, 0, 0, false);
    assert_task(RUNTIME_TASK_DIAGNOSTICS, "Diagnostics", 4096, 2, 1, 1000000, 0, false);
    assert_task(RUNTIME_TASK_TRAINING_LOG, "TrainingLog", 8192, 2, 1, 0, 0, false);
    assert_task(RUNTIME_TASK_STATUS_LED, "StatusLED", 2048, 2, 1, 0, 0, false);

    assert(runtime_schedule_get(RUNTIME_TASK_WAYPOINT) == 0);
    assert(runtime_schedule_get(RUNTIME_TASK_ML_CONTROL) == 0);

    return 0;
}
