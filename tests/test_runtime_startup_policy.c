#include <assert.h>
#include <stdbool.h>

#include "runtime_startup.h"
#include "runtime_metrics.h"

static unsigned disarm_calls;
static unsigned winch_off_calls;
static unsigned optional_disabled_calls;

const runtime_task_spec_t *runtime_schedule_get(runtime_task_id_t id)
{
    static const runtime_task_spec_t critical = {
        .name = "SensorBus", .critical = true,
    };
    static const runtime_task_spec_t optional = {
        .name = "GPS", .critical = false,
    };
    return id == RUNTIME_TASK_SENSOR_BUS ? &critical : &optional;
}

void motor_control_disarm(void)
{
    ++disarm_calls;
}

esp_err_t winch_driver_set_power(bool on)
{
    assert(!on);
    ++winch_off_calls;
    return ESP_OK;
}

void runtime_metrics_count(runtime_task_id_t id, runtime_metric_event_t event)
{
    assert(id == RUNTIME_TASK_GPS || id == RUNTIME_TASK_TRAINING_LOG);
    assert(event == RUNTIME_EVENT_FEATURE_DISABLED);
    ++optional_disabled_calls;
}

void test_log(const char *tag, const char *format, ...)
{
    (void)tag;
    (void)format;
}

int main(void)
{
    assert(runtime_startup_feature_available(RUNTIME_TASK_GPS));
    assert(runtime_startup_handle_task_failure(RUNTIME_TASK_SENSOR_BUS,
                                               ESP_ERR_NO_MEM) == ESP_ERR_NO_MEM);
    assert(disarm_calls == 1);
    assert(winch_off_calls == 1);
    assert(optional_disabled_calls == 0);

    assert(runtime_startup_handle_task_failure(RUNTIME_TASK_GPS,
                                               ESP_ERR_NO_MEM) == ESP_ERR_NO_MEM);
    assert(disarm_calls == 1);
    assert(winch_off_calls == 1);
    assert(optional_disabled_calls == 1);
    assert(!runtime_startup_feature_available(RUNTIME_TASK_GPS));
    assert(runtime_startup_feature_available(RUNTIME_TASK_DETECT));

    assert(runtime_startup_handle_task_failure(RUNTIME_TASK_TRAINING_LOG,
                                               ESP_ERR_NO_MEM) == ESP_ERR_NO_MEM);
    assert(disarm_calls == 1);
    assert(winch_off_calls == 1);
    assert(optional_disabled_calls == 2);
    assert(!runtime_startup_feature_available(RUNTIME_TASK_TRAINING_LOG));
    return 0;
}
