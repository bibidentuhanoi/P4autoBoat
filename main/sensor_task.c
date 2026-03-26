#include "sensor_task.h"
#include "pipeline.h"
#include "sensor_fusion.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l5cx_api.h"
#include <string.h>

static const char *TAG = "SENSOR_TASK";

void task_sensor_snapshot(void *pvParameters)
{
    tof_devices_t *devs = (tof_devices_t *)pvParameters;

    /* Static allocation — boat_SensorSnapshot with two 64-element
     * ToF grids is ~540 bytes encoded; avoids stack overflow risk. */
    static boat_SensorSnapshot snap;

    ESP_LOGI(TAG, "Sensor snapshot task started (5Hz)");

    while (true) {
        snap = (boat_SensorSnapshot)boat_SensorSnapshot_init_zero;
        snap.timestamp_us = (uint64_t)esp_timer_get_time();

        /* IMU */
        FusionResult imu;
        fusion_get_result(&imu);
        snap.has_imu    = true;
        snap.imu.pitch   = imu.pitch;
        snap.imu.roll    = imu.roll;
        snap.imu.heading = imu.heading;

        /* ToF A — static to avoid ~2KB+ stack allocation per loop iteration */
        static VL53L5CX_ResultsData tof_res;
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

        vTaskDelay(pdMS_TO_TICKS(200));  /* 5 Hz */
    }
}
