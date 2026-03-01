#include "calibration.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "stdio.h"
#include "sdkconfig.h"

void perform_calibration_routine(CalibrationData* output_calib) {
    printf("\n\n################################################\n");
    printf("   ENTERING CALIBRATION MODE \n");
    printf("################################################\n");

    // --- Phase 1: Gyro & Level ---
    printf("PHASE 1: GYRO & LEVEL\n");
    printf("Keep board FLAT and STATIONARY.\n");
    for(int i = 3; i > 0; i--) {
        printf("%d...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf(">> Measuring...\n");

    long g_sum[3] = {0};
    long a_sum[3] = {0};
    int samples = CONFIG_CALIB_GYRO_SAMPLES;

    for(int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (imu_read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            a_sum[0] += ax;
            a_sum[1] += ay;
            a_sum[2] += az;
            g_sum[0] += gx;
            g_sum[1] += gy;
            g_sum[2] += gz;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    output_calib->g_bias[0] = (float)g_sum[0] / samples;
    output_calib->g_bias[1] = (float)g_sum[1] / samples;
    output_calib->g_bias[2] = (float)g_sum[2] / samples;

    float ax_avg = (float)a_sum[0] / samples;
    float ay_avg = (float)a_sum[1] / samples;
    float az_avg = (float)a_sum[2] / samples;

    float acc_denom = sqrtf(ay_avg * ay_avg + az_avg * az_avg);
    if (acc_denom < 0.0001f) {
        acc_denom = 0.0001f;
    }

    output_calib->roll_tare  = atan2f(ay_avg, az_avg) * RAD_TO_DEG;
    output_calib->pitch_tare = atan2f(-ax_avg, acc_denom) * RAD_TO_DEG;

    // --- Phase 2: Mag Calibration ---
    printf("\nPHASE 2: COMPASS FIGURE-8\n");
    printf("Pick up board. ROTATE in Figure-8.\n");
    for(int i = 3; i > 0; i--) {
        printf("%d...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf(">> GO! (15 Seconds)\n");

    int16_t m_min[3] = {30000, 30000, 30000};
    int16_t m_max[3] = {-30000, -30000, -30000};

    int64_t start_time = esp_timer_get_time();
    int last_print_sec = CONFIG_CALIB_MAG_DURATION + 1;

    while (true) {
        int64_t current_time = esp_timer_get_time();
        int remaining_sec = CONFIG_CALIB_MAG_DURATION - ((current_time - start_time) / 1000000);

        if (remaining_sec < 0) {
            break;
        }
        if (remaining_sec < last_print_sec) {
            printf("Time Left: %d s...\n", remaining_sec);
            last_print_sec = remaining_sec;
        }

        int16_t mx, my, mz;
        if (imu_read_mag(&mx, &my, &mz) == ESP_OK) {
            if (mx < m_min[0]) { m_min[0] = mx; }
            if (mx > m_max[0]) { m_max[0] = mx; }

            if (my < m_min[1]) { m_min[1] = my; }
            if (my > m_max[1]) { m_max[1] = my; }

            if (mz < m_min[2]) { m_min[2] = mz; }
            if (mz > m_max[2]) { m_max[2] = mz; }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    output_calib->m_bias[0] = (m_max[0] + m_min[0]) / 2.0f;
    output_calib->m_bias[1] = (m_max[1] + m_min[1]) / 2.0f;
    output_calib->m_bias[2] = (m_max[2] + m_min[2]) / 2.0f;

    float chord_x = (float)(m_max[0] - m_min[0]) / 2.0f;
    float chord_y = (float)(m_max[1] - m_min[1]) / 2.0f;
    float chord_z = (float)(m_max[2] - m_min[2]) / 2.0f;

    if (chord_x < 1.0f) { chord_x = 1.0f; }
    if (chord_y < 1.0f) { chord_y = 1.0f; }
    if (chord_z < 1.0f) { chord_z = 1.0f; }

    float avg_chord = (chord_x + chord_y + chord_z) / 3.0f;

    output_calib->m_scale[0] = avg_chord / chord_x;
    output_calib->m_scale[1] = avg_chord / chord_y;
    output_calib->m_scale[2] = avg_chord / chord_z;

    // --- Phase 3: Alignment (Tare Heading) ---
    printf("\nPHASE 3: ALIGNMENT (Set Zero)\n");
    printf("Point the sensor exactly FORWARD.\n");
    printf("Hold it still...\n");
    for(int i = 5; i > 0; i--) {
        printf("%d...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf(">> Capturing Zero Reference...\n");

    long align_x = 0, align_y = 0;
    for(int i = 0; i < 100; i++) {
        int16_t mx, my, mz;
        if (imu_read_mag(&mx, &my, &mz) == ESP_OK) {
            float mx_cal = ((float)mx - output_calib->m_bias[0]) * output_calib->m_scale[0];
            float my_cal = ((float)my - output_calib->m_bias[1]) * output_calib->m_scale[1];

            align_x += mx_cal;
            align_y += my_cal;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float ref_heading = atan2f((float)align_y, (float)align_x) * RAD_TO_DEG;
    if (ref_heading < 0.0f) {
        ref_heading += 360.0f;
    }

    output_calib->heading_tare = ref_heading;
}
