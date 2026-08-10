#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "runtime_schedule.h"

esp_err_t runtime_task_create(runtime_task_id_t id, TaskFunction_t fn,
                              void *arg, TaskHandle_t *out);
