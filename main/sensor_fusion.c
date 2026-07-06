#include "sensor_fusion.h"
#include "detect_task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>

static const char *FUSION_TAG = "FUSION";

// Configuration macros will be provided by Kconfig in the future,
// for now we use the ones defined in sdkconfig or default if missing
#include "sdkconfig.h"

// Filter Constants
#define MAG_LPF_ALPHA       0.2f
#define ACCEL_EPSILON       0.0001f // Prevents Div-by-Zero at 0G

// Globals
static CalibrationData* calib;
static SemaphoreHandle_t fusion_mutex;
static FusionResult current_fusion;

// Helper
static float mag_filt[3] = {0};
static float pitch = 0.0f;
static float roll = 0.0f;
static float alpha;

void fusion_init(CalibrationData* calib_data) {
    calib = calib_data;
    fusion_mutex = xSemaphoreCreateMutex();

    // Parse alpha once — was strtof() every 20ms tick (Bug 4)
    alpha = strtof(CONFIG_FUSION_COMPLEMENTARY_ALPHA, NULL);
    if (alpha <= 0.0f || alpha >= 1.0f) {
        alpha = 0.96f;
    }

    // Initialize state with tares
    pitch = calib->pitch_tare;
    roll = calib->roll_tare;
}

void fusion_get_result(FusionResult* res) {
    if (xSemaphoreTake(fusion_mutex, portMAX_DELAY) == pdTRUE) {
        *res = current_fusion;
        xSemaphoreGive(fusion_mutex);
    }
}

void task_imu_fusion(void *pvParameters) {
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    int16_t raw_mx, raw_my, raw_mz;

    int64_t last_time = esp_timer_get_time();

    while(1) {
        while (g_inference_active) vTaskDelay(pdMS_TO_TICKS(10));
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;

        // --- I2C reads under shared bus mutex ---
        // Failure visibility: a dead IMU used to freeze the outputs at zero
        // with no trace in the logs. Count failures and report ~every 2s.
        static uint32_t ag_fails = 0, mag_fails = 0;
        static bool bus_scanned = false;   /* one-shot ground-truth scan on first sustained fault */
        bool mag_ok = false;
        esp_err_t mag_err = ESP_FAIL;   /* real I2C error, surfaced in the failure log */
        esp_err_t ag_err  = ESP_FAIL;

        bool read_success = false;
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // MAG READ
            mag_err = imu_read_mag(&raw_mx, &raw_my, &raw_mz);
            mag_ok  = (mag_err == ESP_OK);
            if (mag_ok) {
                float mx_cal = ((float)raw_mx - calib->m_bias[0]) * calib->m_scale[0];
                float my_cal = ((float)raw_my - calib->m_bias[1]) * calib->m_scale[1];
                float mz_cal = ((float)raw_mz - calib->m_bias[2]) * calib->m_scale[2];

                mag_filt[0] += MAG_LPF_ALPHA * (mx_cal - mag_filt[0]);
                mag_filt[1] += MAG_LPF_ALPHA * (my_cal - mag_filt[1]);
                mag_filt[2] += MAG_LPF_ALPHA * (mz_cal - mag_filt[2]);
            }

            // ACCEL/GYRO READ
            ag_err = imu_read_accel_gyro(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
            read_success = (ag_err == ESP_OK);
            xSemaphoreGive(g_i2c_mutex);
        }

        if (!mag_ok) {
            ++mag_fails;
            if (mag_fails == 50) imu_set_mag_ok(false);   /* ~1s dead → report it */
            /* Ground-truth scan on first sustained mag fault too (not just ICM),
             * so a dead mag on a working bus is settled, not guessed. */
            if (mag_fails == 50 && !bus_scanned &&
                xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                bus_scanned = true;
                imu_bus_scan();
                xSemaphoreGive(g_i2c_mutex);
            }
            if ((mag_fails % 100) == 1) {
                ESP_LOGW(FUSION_TAG, "QMC5883L mag read failing (%lu fails, %s) — heading frozen",
                         (unsigned long)mag_fails, esp_err_to_name(mag_err));
            }
            /* Sustained NACK never returns ESP_OK, so the recover-after-success
             * path below can't fire. Force a full re-add + reconfigure every
             * ~2s so a chip that dropped off the bus can rejoin. */
            if ((mag_fails % 100) == 0 &&
                xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                esp_err_t rec = imu_recover_mag();
                xSemaphoreGive(g_i2c_mutex);
                if (rec == ESP_OK) {
                    ESP_LOGI(FUSION_TAG, "mag recovered after %lu fails — re-added + reconfigured",
                             (unsigned long)mag_fails);
                    imu_set_mag_ok(true);
                    mag_fails = 0;
                }
            }
        } else if (mag_fails) {
            /* Came back on its own (chip ACKs again) — it may have missed its
             * boot config (standby => stale zeros), so re-apply before trusting. */
            if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                imu_reinit_mag();
                xSemaphoreGive(g_i2c_mutex);
            }
            ESP_LOGI(FUSION_TAG, "mag recovered after %lu fails — reconfigured",
                     (unsigned long)mag_fails);
            imu_set_mag_ok(true);
            mag_fails = 0;
        }
        if (!read_success) {
            ++ag_fails;
            if (ag_fails == 50) imu_set_icm_ok(false);    /* ~1s dead → report it */
            /* Once, when the bus is known-good at runtime (mag reading fine),
             * dump who actually ACKs — settles chip-absent vs read-path bug. */
            if (ag_fails == 50 && !bus_scanned &&
                xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                bus_scanned = true;
                imu_bus_scan();
                xSemaphoreGive(g_i2c_mutex);
            }
            if ((ag_fails % 100) == 1) {
                ESP_LOGW(FUSION_TAG, "ICM20948 accel/gyro read failing (%lu fails, %s) — pitch/roll frozen",
                         (unsigned long)ag_fails, esp_err_to_name(ag_err));
            }
            /* Sustained NACK never returns ESP_OK, so the recover-after-success
             * path below can't fire. Force a full re-probe (both straps) +
             * reconfigure every ~2s so a reset / address-moved chip can return. */
            if ((ag_fails % 100) == 0 &&
                xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                esp_err_t rec = imu_recover_accel_gyro();
                xSemaphoreGive(g_i2c_mutex);
                if (rec == ESP_OK) {
                    ESP_LOGI(FUSION_TAG, "accel/gyro recovered after %lu fails — reprobed + reconfigured",
                             (unsigned long)ag_fails);
                    imu_set_icm_ok(true);
                    ag_fails = 0;
                }
            }
        } else if (ag_fails) {
            /* Came back on its own (chip ACKs again): wake it (boots asleep) + range. */
            if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                imu_reinit_accel_gyro();
                xSemaphoreGive(g_i2c_mutex);
            }
            ESP_LOGI(FUSION_TAG, "accel/gyro recovered after %lu fails — reconfigured",
                     (unsigned long)ag_fails);
            imu_set_icm_ok(true);
            ag_fails = 0;
        }

        // --- FUSION ---
        if (read_success) {
            float gx_rate = ((float)raw_gx - calib->g_bias[0]) / GYRO_SCALE_250DPS;
            float gy_rate = ((float)raw_gy - calib->g_bias[1]) / GYRO_SCALE_250DPS;

            float acc_roll  = atan2f(raw_ay, raw_az) * RAD_TO_DEG;

            float acc_denom = sqrtf((float)raw_ay * raw_ay + (float)raw_az * raw_az);
            if (acc_denom < ACCEL_EPSILON) {
                acc_denom = ACCEL_EPSILON;
            }
            float acc_pitch = atan2f(-raw_ax, acc_denom) * RAD_TO_DEG;

            // Simple complementary filter (alpha parsed once in fusion_init)

            roll  = alpha * (roll  + gx_rate * dt) + (1.0f - alpha) * acc_roll;
            pitch = alpha * (pitch + gy_rate * dt) + (1.0f - alpha) * acc_pitch;

            // --- COMPASS ---
            float p_rad = pitch * DEG_TO_RAD;
            float r_rad = roll * DEG_TO_RAD;

            float Xh = mag_filt[0] * cosf(p_rad) + mag_filt[2] * sinf(p_rad);
            float Yh = mag_filt[0] * sinf(r_rad) * sinf(p_rad) + mag_filt[1] * cosf(r_rad) - mag_filt[2] * sinf(r_rad) * cosf(p_rad);

            float heading = atan2f(Yh, Xh) * RAD_TO_DEG;

            heading -= calib->heading_tare;

            if (heading < 0.0f) {
                heading += 360.0f;
            }
            if (heading >= 360.0f) {
                heading -= 360.0f;
            }

            // --- THREAD SAFE WRITE ---
            if (xSemaphoreTake(fusion_mutex, portMAX_DELAY) == pdTRUE) {
                current_fusion.roll = roll - calib->roll_tare;
                current_fusion.pitch = pitch - calib->pitch_tare;
                current_fusion.heading = heading;
                xSemaphoreGive(fusion_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
