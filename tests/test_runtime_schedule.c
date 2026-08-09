#include <assert.h>

#include "runtime_schedule.h"

int main(void)
{
    assert(runtime_schedule_validate());

    const runtime_task_spec_t *control = runtime_schedule_get(RUNTIME_TASK_CONTROL);
    const runtime_task_spec_t *bus = runtime_schedule_get(RUNTIME_TASK_SENSOR_BUS);
    const runtime_task_spec_t *gps = runtime_schedule_get(RUNTIME_TASK_GPS);
    const runtime_task_spec_t *detect = runtime_schedule_get(RUNTIME_TASK_DETECT);
    const runtime_task_spec_t *training = runtime_schedule_get(RUNTIME_TASK_TRAINING_LOG);

    assert(control->core == 0 && control->priority == 10 && control->period_us == 10000);
    assert(bus->core == 0 && bus->priority == 8 && bus->period_us == 20000);
    assert(gps->core == 0 && gps->priority == 6);
    assert(detect->core == 1 && detect->priority == 7 && detect->stack_size == 32768);
    assert(training->core == 1 && training->priority == 2 && !training->critical);
    assert(runtime_schedule_get(RUNTIME_TASK_WAYPOINT) == 0);
    assert(runtime_schedule_get(RUNTIME_TASK_ML_CONTROL) == 0);

    return 0;
}
