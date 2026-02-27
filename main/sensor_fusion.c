#include "sensor_fusion.h"
#include "esp_timer.h"
#include "math.h"
#include <string.h>

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

void fusion_init(CalibrationData* calib_data) {
    calib = calib_data;
    fusion_mutex = xSemaphoreCreateMutex();

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
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;

        // --- MAG READ & FILTER ---
        if (imu_read_mag(&raw_mx, &raw_my, &raw_mz) == ESP_OK) {
            float mx_cal = ((float)raw_mx - calib->m_bias[0]) * calib->m_scale[0];
            float my_cal = ((float)raw_my - calib->m_bias[1]) * calib->m_scale[1];
            float mz_cal = ((float)raw_mz - calib->m_bias[2]) * calib->m_scale[2];

            mag_filt[0] += MAG_LPF_ALPHA * (mx_cal - mag_filt[0]);
            mag_filt[1] += MAG_LPF_ALPHA * (my_cal - mag_filt[1]);
            mag_filt[2] += MAG_LPF_ALPHA * (mz_cal - mag_filt[2]);
        }

        // --- ACCEL/GYRO READ ---
        bool read_success = false;
        if (imu_read_accel_gyro(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz) == ESP_OK) {
            read_success = true;
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

            // Simple complementary filter
            float alpha = strtof(CONFIG_FUSION_COMPLEMENTARY_ALPHA, NULL);
            if (alpha <= 0.0f || alpha >= 1.0f) {
                alpha = 0.96f; // Fallback safety
            }

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
