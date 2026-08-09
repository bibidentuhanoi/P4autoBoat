#include "runtime_task.h"

#include "esp_log.h"

esp_err_t runtime_task_create(runtime_task_id_t id, TaskFunction_t fn,
                              void *arg, TaskHandle_t *out)
{
    const runtime_task_spec_t *spec = runtime_schedule_get(id);
    if (!spec || !fn) {
        return ESP_ERR_INVALID_ARG;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(fn, spec->name, spec->stack_size,
                                            arg, spec->priority, out, spec->core);
    if (ok != pdPASS) {
        ESP_LOGE("RUNTIME", "task create failed: %s core=%d prio=%u stack=%u critical=%d",
                 spec->name, spec->core, (unsigned)spec->priority,
                 (unsigned)spec->stack_size, spec->critical);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
