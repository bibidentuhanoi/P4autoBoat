/* ═══════════════════════════════════════════════════════════════
 * CALIBRATION CHECKLIST — When changing housing/lens:
 * ───────────────────────────────────────────────────────────────
 * 1. Mount on flat wall (distance 1–3 m)
 * 2. Run IMU calibration (GPIO 35 at boot)
 * 3. Check 3D Threshold overlay alignment (dashboard)
 * 4. Adjust FX (focal length) if FOV is off
 * 5. Adjust CX/CY (principal point) for center offset
 * 6. Fine-tune AZ/EL offsets via dashboard sliders
 * 7. Verify grid scale GRID_SCALE ≈ 1.0
 * 8. Test drag offset (should be near 0,0)
 * ═══════════════════════════════════════════════════════════════ */

#include "sensor_task.h"
#include "detect_task.h"
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

static inline int16_t median3(int16_t a, int16_t b, int16_t c) {
    return (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                   : ((a > c) ? a : ((b > c) ? c : b));
}

#define SNAPSHOT_INTERVAL_MS  50   /* 20 Hz sensor publish */
#define TOF_EVERY_N           4    /* ToF at every 4th tick = 5 Hz (keeps WS traffic low) */
#define STATUS_EVERY_N        20   /* SystemStatus every 20th iteration (~1Hz) */

void task_sensor_snapshot(void *pvParameters)
{
    tof_devices_t *devs = (tof_devices_t *)pvParameters;

    /* Static allocation — avoids stack overflow risk */
    static boat_SensorSnapshot snap;
    static VL53L5CX_ResultsData tof_res;

    /* 3-frame median filter per zone×target — kills single-frame spikes */
    static int16_t med_a[64 * VL53L5CX_NB_TARGET_PER_ZONE][3];
    static int16_t med_b[64 * VL53L5CX_NB_TARGET_PER_ZONE][3];
    static uint8_t med_idx_a = 0, med_idx_b = 0;

    uint32_t iteration = 0;

    ESP_LOGI(TAG, "Sensor snapshot task started (%d Hz, ToF %d Hz)",
             1000 / SNAPSHOT_INTERVAL_MS,
             1000 / SNAPSHOT_INTERVAL_MS / TOF_EVERY_N);

    while (true) {
        /* Yield while inference is running — avoid DMA/PSRAM contention */
        while (g_inference_active) vTaskDelay(pdMS_TO_TICKS(10));

        snap = (boat_SensorSnapshot)boat_SensorSnapshot_init_zero;
        snap.timestamp_us = (uint64_t)esp_timer_get_time();

        /* IMU — always available, 20 Hz */
        FusionResult imu;
        fusion_get_result(&imu);
        snap.has_imu     = true;
        snap.imu.pitch   = imu.pitch;
        snap.imu.roll    = imu.roll;
        snap.imu.heading = imu.heading;

        /* ToF — 5 Hz to keep WS payload small (~1.4KB per full frame) */
        bool tof_tick = (iteration % TOF_EVERY_N == 0);

        if (tof_tick) {
            /* ToF A — take/release mutex per sensor to give IMU a read window */
            if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                snap.has_tof_a = (tof_read_grid(&devs->dev_a, &tof_res) == ESP_OK);
                xSemaphoreGive(g_i2c_mutex);
                if (snap.has_tof_a) {
                    snap.tof_a.valid = true;
                    snap.tof_a.distances_count = 64 * VL53L5CX_NB_TARGET_PER_ZONE;
                    snap.tof_a.sigma_count = 64 * VL53L5CX_NB_TARGET_PER_ZONE;
                    snap.tof_a.target_status_count = 64 * VL53L5CX_NB_TARGET_PER_ZONE;
                    snap.tof_a.nb_target_detected_count = 64;
                    for (int i = 0; i < 64; i++) {
                        uint8_t nb = tof_res.nb_target_detected[i];
                        for (int j = 0; j < VL53L5CX_NB_TARGET_PER_ZONE; j++) {
                            int idx = i * VL53L5CX_NB_TARGET_PER_ZONE + j;
                            uint8_t status = tof_res.target_status[idx];
                            /* Gate on nb_target_detected AND status: slots beyond
                             * nb_target_detected contain stale driver buffer data. */
                            bool slot_valid = (j < nb) && (status == 5 || status == 9);
                            int16_t raw = slot_valid ? tof_res.distance_mm[idx] : 0;
                            med_a[idx][med_idx_a] = raw;
                            snap.tof_a.distances[idx] = median3(med_a[idx][0], med_a[idx][1], med_a[idx][2]);
                            snap.tof_a.sigma[idx] = slot_valid ? tof_res.range_sigma_mm[idx] : 0;
                            snap.tof_a.target_status[idx] = slot_valid ? status : 0;
                        }
                        snap.tof_a.nb_target_detected[i] = nb;
                    }
                    med_idx_a = (med_idx_a + 1) % 3;
                }
            }

            /* ToF B — separate mutex acquisition */
            if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                snap.has_tof_b = (tof_read_grid(&devs->dev_b, &tof_res) == ESP_OK);
                xSemaphoreGive(g_i2c_mutex);
                if (snap.has_tof_b) {
                    snap.tof_b.valid = true;
                    snap.tof_b.distances_count = 64 * VL53L5CX_NB_TARGET_PER_ZONE;
                    snap.tof_b.sigma_count = 64 * VL53L5CX_NB_TARGET_PER_ZONE;
                    snap.tof_b.target_status_count = 64 * VL53L5CX_NB_TARGET_PER_ZONE;
                    snap.tof_b.nb_target_detected_count = 64;
                    for (int i = 0; i < 64; i++) {
                        uint8_t nb = tof_res.nb_target_detected[i];
                        for (int j = 0; j < VL53L5CX_NB_TARGET_PER_ZONE; j++) {
                            int idx = i * VL53L5CX_NB_TARGET_PER_ZONE + j;
                            uint8_t status = tof_res.target_status[idx];
                            bool slot_valid = (j < nb) && (status == 5 || status == 9);
                            int16_t raw = slot_valid ? tof_res.distance_mm[idx] : 0;
                            med_b[idx][med_idx_b] = raw;
                            snap.tof_b.distances[idx] = median3(med_b[idx][0], med_b[idx][1], med_b[idx][2]);
                            snap.tof_b.sigma[idx] = slot_valid ? tof_res.range_sigma_mm[idx] : 0;
                            snap.tof_b.target_status[idx] = slot_valid ? status : 0;
                        }
                        snap.tof_b.nb_target_detected[i] = nb;
                    }
                    med_idx_b = (med_idx_b + 1) % 3;
                }
            }
        }

        /* Re-emit cached detections for up to 15s so the browser gets them
         * after WS reconnect, regardless of when it comes back. */
        detect_get_cached_results(snap.detections, &snap.detections_count, 15000);

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
