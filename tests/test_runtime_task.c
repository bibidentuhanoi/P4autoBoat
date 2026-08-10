#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "runtime_schedule.h"
#include "runtime_task.h"

static runtime_task_id_t registered_id;
static TaskHandle_t registered_handle;

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name,
                                   uint32_t stack_size, void *arg,
                                   UBaseType_t priority, TaskHandle_t *out,
                                   BaseType_t core)
{
    (void)arg;
    assert(fn != NULL);
    const runtime_task_spec_t *spec = runtime_schedule_get(registered_id);
    assert(spec != NULL);
    assert(strcmp(name, spec->name) == 0);
    assert(stack_size == spec->stack_size);
    assert(priority == spec->priority);
    assert(core == spec->core);
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

int main(void)
{
    assert(runtime_schedule_validate());
    for (registered_id = RUNTIME_TASK_CONTROL;
         registered_id < RUNTIME_TASK_COUNT;
         ++registered_id) {
        registered_handle = NULL;
        TaskHandle_t out = NULL;
        assert(runtime_task_create(registered_id, test_task, NULL, &out) == ESP_OK);
        assert(out == (TaskHandle_t)(uintptr_t)(registered_id + 1));
    }
    return 0;
}
