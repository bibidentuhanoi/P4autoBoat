#include "runtime_startup.h"

#include "drivers/winch_driver.h"
#include "runtime_metrics.h"

#include "esp_log.h"

#include <stdatomic.h>

static const char *TAG = "RUNTIME_STARTUP";
static atomic_uint s_disabled_features;

void motor_control_disarm(void);

esp_err_t runtime_startup_handle_task_failure(runtime_task_id_t id,
                                              esp_err_t error)
{
    const runtime_task_spec_t *spec = runtime_schedule_get(id);
    if (!spec) {
        ESP_LOGE(TAG, "unknown task failed to start: id=%d error=%d", id, error);
        return error;
    }

    if (spec->critical) {
        motor_control_disarm();
        (void)winch_driver_set_power(false);
        ESP_LOGE(TAG, "critical task failed to start: %s error=%d — actuators forced safe",
                 spec->name, error);
    } else {
        atomic_fetch_or_explicit(&s_disabled_features, 1U << (unsigned)id,
                                 memory_order_relaxed);
        runtime_metrics_count(id, RUNTIME_EVENT_FEATURE_DISABLED);
        ESP_LOGW(TAG, "optional task unavailable: %s error=%d", spec->name, error);
    }
    return error;
}

bool runtime_startup_feature_available(runtime_task_id_t id)
{
    const runtime_task_spec_t *spec = runtime_schedule_get(id);
    if (!spec) return false;
    if (spec->critical) return true;
    return (atomic_load_explicit(&s_disabled_features, memory_order_relaxed) &
            (1U << (unsigned)id)) == 0;
}
