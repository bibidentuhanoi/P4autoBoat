#include "runtime_task.h"

#include "runtime_metrics.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

#include <stddef.h>

esp_err_t runtime_task_create(runtime_task_id_t id, TaskFunction_t fn,
                              void *arg, TaskHandle_t *out)
{
    const runtime_task_spec_t *spec = runtime_schedule_get(id);
    if (!spec || !fn) {
        return ESP_ERR_INVALID_ARG;
    }

    TaskHandle_t created = NULL;
    TaskHandle_t *handle_out = out ? out : &created;
    BaseType_t ok = 0;
    if (spec->stack_in_psram) {
        /* The stack from PSRAM, the TCB (always) from internal RAM. */
        ok = xTaskCreatePinnedToCoreWithCaps(fn, spec->name, spec->stack_size, arg,
                                             spec->priority, handle_out, spec->core,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (ok == pdPASS) {
            ESP_LOGI("RUNTIME", "%s: %u B stack in PSRAM", spec->name,
                     (unsigned)spec->stack_size);
        } else {
            ESP_LOGW("RUNTIME", "%s: no PSRAM for its stack -- trying internal RAM",
                     spec->name);
        }
    }
    if (ok != pdPASS) {
        ok = xTaskCreatePinnedToCore(fn, spec->name, spec->stack_size,
                                     arg, spec->priority, handle_out, spec->core);
    }
    if (ok != pdPASS) {
        ESP_LOGE("RUNTIME", "task create failed: %s core=%d prio=%u stack=%u critical=%d",
                 spec->name, spec->core, (unsigned)spec->priority,
                 (unsigned)spec->stack_size, spec->critical);
        return ESP_ERR_NO_MEM;
    }

    runtime_metrics_register_handle(id, *handle_out);

    return ESP_OK;
}
