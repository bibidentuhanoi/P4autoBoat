#include "sensor_api.h"
#include "sensor_fusion.h"
#include "drivers/tof_driver.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "vl53l5cx_api.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "SENSOR_API";

/* ---- Shared snapshot state (mutex-protected) ---- */

typedef struct {
    FusionResult imu;
    VL53L5CX_ResultsData tof_a;
    VL53L5CX_ResultsData tof_b;
    bool tof_a_valid;
    bool tof_b_valid;
    int64_t timestamp_us;
} snapshot_data_t;

static snapshot_data_t s_snapshot;
static SemaphoreHandle_t s_snapshot_mutex;

/* ---- JSON helpers ---- */

static cJSON *imu_to_json(const FusionResult *imu, int64_t ts)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "ts", (double)ts);
    cJSON_AddNumberToObject(root, "pitch", imu->pitch);
    cJSON_AddNumberToObject(root, "roll", imu->roll);
    cJSON_AddNumberToObject(root, "heading", imu->heading);
    return root;
}

static cJSON *tof_grid_to_json(const VL53L5CX_ResultsData *res)
{
    cJSON *grid = cJSON_CreateArray();
    for (int row = 0; row < 8; row++) {
        cJSON *row_arr = cJSON_CreateArray();
        for (int col = 0; col < 8; col++) {
            int zone = row * 8 + col;
            int idx = VL53L5CX_NB_TARGET_PER_ZONE * zone;
            cJSON_AddItemToArray(row_arr, cJSON_CreateNumber(res->distance_mm[idx]));
        }
        cJSON_AddItemToArray(grid, row_arr);
    }
    return grid;
}

/* ---- HTTP Handlers ---- */

static esp_err_t imu_handler(httpd_req_t *req)
{
    FusionResult imu;
    fusion_get_result(&imu);
    int64_t ts = esp_timer_get_time();

    cJSON *root = imu_to_json(&imu, ts);
    const char *json = cJSON_PrintUnformatted(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));

    free((void *)json);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    snapshot_data_t snap;

    if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Sensor data unavailable");
        return ESP_FAIL;
    }
    memcpy(&snap, &s_snapshot, sizeof(snap));
    xSemaphoreGive(s_snapshot_mutex);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "ts", (double)snap.timestamp_us);

    /* IMU sub-object */
    cJSON *imu_obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(imu_obj, "pitch", snap.imu.pitch);
    cJSON_AddNumberToObject(imu_obj, "roll", snap.imu.roll);
    cJSON_AddNumberToObject(imu_obj, "heading", snap.imu.heading);
    cJSON_AddItemToObject(root, "imu", imu_obj);

    /* ToF grids (null if invalid) */
    if (snap.tof_a_valid) {
        cJSON_AddItemToObject(root, "tof_a", tof_grid_to_json(&snap.tof_a));
    } else {
        cJSON_AddNullToObject(root, "tof_a");
    }

    if (snap.tof_b_valid) {
        cJSON_AddItemToObject(root, "tof_b", tof_grid_to_json(&snap.tof_b));
    } else {
        cJSON_AddNullToObject(root, "tof_b");
    }

    const char *json = cJSON_PrintUnformatted(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));

    free((void *)json);
    cJSON_Delete(root);
    return ESP_OK;
}

/* ---- Public API ---- */

esp_err_t sensor_api_register(httpd_handle_t server, tof_devices_t *tof_devs)
{
    (void)tof_devs; /* stored via task parameter, not needed here */

    s_snapshot_mutex = xSemaphoreCreateMutex();
    if (!s_snapshot_mutex) {
        ESP_LOGE(TAG, "Failed to create snapshot mutex");
        return ESP_ERR_NO_MEM;
    }

    static const httpd_uri_t imu_uri = {
        .uri      = "/api/imu",
        .method   = HTTP_GET,
        .handler  = imu_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t snapshot_uri = {
        .uri      = "/api/snapshot",
        .method   = HTTP_GET,
        .handler  = snapshot_handler,
        .user_ctx = NULL,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &imu_uri),
                        TAG, "register /api/imu failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &snapshot_uri),
                        TAG, "register /api/snapshot failed");

    ESP_LOGI(TAG, "Sensor API registered: /api/imu, /api/snapshot");
    return ESP_OK;
}

/* ---- Snapshot Task ---- */

void task_sensor_snapshot(void *pvParameters)
{
    tof_devices_t *devs = (tof_devices_t *)pvParameters;

    ESP_LOGI(TAG, "sensor_snapshot task started (5Hz)");

    while (true) {
        int64_t ts = esp_timer_get_time();

        FusionResult imu;
        fusion_get_result(&imu);

        VL53L5CX_ResultsData tof_a, tof_b;
        bool tof_a_ok = (tof_read_grid(&devs->dev_a, &tof_a) == ESP_OK);
        bool tof_b_ok = (tof_read_grid(&devs->dev_b, &tof_b) == ESP_OK);

        if (xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY) == pdTRUE) {
            s_snapshot.timestamp_us = ts;
            s_snapshot.imu = imu;
            if (tof_a_ok) {
                memcpy(&s_snapshot.tof_a, &tof_a, sizeof(tof_a));
            }
            s_snapshot.tof_a_valid = tof_a_ok;
            if (tof_b_ok) {
                memcpy(&s_snapshot.tof_b, &tof_b, sizeof(tof_b));
            }
            s_snapshot.tof_b_valid = tof_b_ok;
            xSemaphoreGive(s_snapshot_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
