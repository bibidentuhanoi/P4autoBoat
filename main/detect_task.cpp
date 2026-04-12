#include "detect_task.h"
#include "esp_log.h"

static const char *TAG = "DETECT";

extern "C" esp_err_t detect_init(void)
{
    ESP_LOGI(TAG, "Detect task initialized (stub)");
    return ESP_OK;
}

extern "C" void detect_trigger(void)
{
    ESP_LOGI(TAG, "Detect triggered (stub)");
}
