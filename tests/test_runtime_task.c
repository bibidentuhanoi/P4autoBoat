#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "runtime_schedule.h"
#include "runtime_task.h"

static runtime_task_id_t registered_id;
static TaskHandle_t registered_handle;
static int internal_calls, psram_calls;
static bool fail_internal, fail_psram;

static void check_spec(TaskFunction_t fn, const char *name, uint32_t stack_size,
                       UBaseType_t priority, BaseType_t core)
{
    assert(fn != NULL);
    const runtime_task_spec_t *spec = runtime_schedule_get(registered_id);
    assert(spec != NULL);
    assert(strcmp(name, spec->name) == 0);
    assert(stack_size == spec->stack_size);
    assert(priority == spec->priority);
    assert(core == spec->core);
}

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name,
                                   uint32_t stack_size, void *arg,
                                   UBaseType_t priority, TaskHandle_t *out,
                                   BaseType_t core)
{
    (void)arg;
    check_spec(fn, name, stack_size, priority, core);
    internal_calls++;
    if (fail_internal) return 0;
    *out = (TaskHandle_t)(uintptr_t)(registered_id + 1);
    registered_handle = *out;
    return pdPASS;
}

BaseType_t xTaskCreatePinnedToCoreWithCaps(TaskFunction_t fn, const char *name,
                                           uint32_t stack_size, void *arg,
                                           UBaseType_t priority, TaskHandle_t *out,
                                           BaseType_t core, UBaseType_t caps)
{
    (void)arg;
    check_spec(fn, name, stack_size, priority, core);
    assert(runtime_schedule_get(registered_id)->stack_in_psram);
    assert(caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    psram_calls++;
    if (fail_psram) return 0;
    *out = (TaskHandle_t)(uintptr_t)(registered_id + 1);
    registered_handle = *out;
    return pdPASS;
}

void runtime_metrics_register_handle(runtime_task_id_t id, TaskHandle_t handle)
{
    assert(id == registered_id);
    assert(handle == registered_handle);
}

void test_log(const char *tag, const char *format, ...)
{
    (void)tag;
    (void)format;
}

static void test_task(void *arg)
{
    (void)arg;
}

static esp_err_t create(runtime_task_id_t id)
{
    registered_id = id;
    registered_handle = NULL;
    internal_calls = psram_calls = 0;
    TaskHandle_t out = NULL;
    const esp_err_t err = runtime_task_create(id, test_task, NULL, &out);
    if (err == ESP_OK) assert(out == (TaskHandle_t)(uintptr_t)(id + 1));
    return err;
}

int main(void)
{
    assert(runtime_schedule_validate());

    /* Every task from its own schedule entry, its stack where the entry says:
     * a PSRAM stack through xTaskCreatePinnedToCoreWithCaps(SPIRAM | 8BIT),
     * every other task through plain xTaskCreatePinnedToCore (internal RAM). */
    for (runtime_task_id_t id = RUNTIME_TASK_CONTROL; id < RUNTIME_TASK_COUNT; ++id) {
        assert(create(id) == ESP_OK);
        if (runtime_schedule_get(id)->stack_in_psram) {
            assert(psram_calls == 1 && internal_calls == 0);
        } else {
            assert(psram_calls == 0 && internal_calls == 1);
        }
    }

    /* PSRAM refused (never expected with ~22 MB free): the task still starts,
     * from internal RAM -- where every task lived before. */
    fail_psram = true;
    assert(create(RUNTIME_TASK_AUTONOMY) == ESP_OK);
    assert(psram_calls == 1 && internal_calls == 1);

    /* Nowhere to put it: reported, nothing registered. */
    fail_internal = true;
    registered_handle = (TaskHandle_t)(uintptr_t)0xdead;   /* must stay untouched */
    assert(create(RUNTIME_TASK_TRAINING_LOG) == ESP_ERR_NO_MEM);
    assert(psram_calls == 1 && internal_calls == 1);
    assert(create(RUNTIME_TASK_DIAGNOSTICS) == ESP_ERR_NO_MEM);
    assert(psram_calls == 0 && internal_calls == 1);
    return 0;
}
