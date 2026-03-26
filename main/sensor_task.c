#include "sensor_task.h"
#include "pipeline.h"
#include "sensor_fusion.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l5cx_api.h"
#include <string.h>

static const char *TAG = "SENSOR_TASK";

#define SNAPSHOT_INTERVAL_MS  50   /* 20 Hz sensor publish */
#define STATUS_EVERY_N        20   /* SystemStatus every 20th iteration (~1Hz) */

void task_sensor_snapshot(void *pvParameters)
{
    tof_devices_t *devs = (tof_devices_t *)pvParameters;

    /* Static allocation — avoids stack overflow risk */
    static boat_SensorSnapshot snap;
    static VL53L5CX_ResultsData tof_res;

    uint32_t iteration = 0;

    ESP_LOGI(TAG, "Sensor snapshot task started (%d Hz)", 1000 / SNAPSHOT_INTERVAL_MS);

    while (true) {
        snap = (boat_SensorSnapshot)boat_SensorSnapshot_init_zero;
        snap.timestamp_us = (uint64_t)esp_timer_get_time();

        /* IMU — always available */
        FusionResult imu;
        fusion_get_result(&imu);
        snap.has_imu     = true;
        snap.imu.pitch   = imu.pitch;
        snap.imu.roll    = imu.roll;
        snap.imu.heading = imu.heading;

        /* ToF A */
        snap.has_tof_a = (tof_read_grid(&devs->dev_a, &tof_res) == ESP_OK);
        if (snap.has_tof_a) {
            snap.tof_a.valid = true;
            snap.tof_a.distances_count = 64;
            for (int i = 0; i < 64; i++) {
                snap.tof_a.distances[i] =
                    tof_res.distance_mm[i * VL53L5CX_NB_TARGET_PER_ZONE];
            }
        }

        /* ToF B */
        snap.has_tof_b = (tof_read_grid(&devs->dev_b, &tof_res) == ESP_OK);
        if (snap.has_tof_b) {
            snap.tof_b.valid = true;
            snap.tof_b.distances_count = 64;
            for (int i = 0; i < 64; i++) {
                snap.tof_b.distances[i] =
                    tof_res.distance_mm[i * VL53L5CX_NB_TARGET_PER_ZONE];
            }
        }

        pipeline_publish_sensors(&snap);

        /* SystemStatus at ~1Hz */
        if (++iteration % STATUS_EVERY_N == 0) {
            boat_SystemStatus sys = boat_SystemStatus_init_zero;
            sys.heap_free = (uint32_t)esp_get_free_heap_size();
            sys.uptime_us = (uint64_t)esp_timer_get_time();

            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                sys.wifi_rssi = ap.rssi;
            }

            pipeline_publish_status(&sys);
        }

        vTaskDelay(pdMS_TO_TICKS(SNAPSHOT_INTERVAL_MS));
    }
}
