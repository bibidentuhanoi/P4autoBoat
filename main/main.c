// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c_master.h"
// #include "driver/gpio.h"
// #include "esp_timer.h"
// #include "vl53l5cx_api.h"

// // ==========================================
// // USER CONFIGURATION
// // ==========================================
// #define DO_CALIBRATE        true 

// #define I2C_PORT            I2C_NUM_1
// #define SCL_IO_PIN          GPIO_NUM_7
// #define SDA_IO_PIN          GPIO_NUM_8
// #define SENSOR_A_LPN_PIN    GPIO_NUM_52
// #define SENSOR_B_LPN_PIN    GPIO_NUM_29

// #define QMC5883L_ADDR       0x0D
// #define ICM20948_ADDR       0x69
// #define VL53_DEFAULT_ADDR   0x29
// #define VL53_ADDR_A         0x22
// #define VL53_ADDR_B         0x24

// #define PI                  3.14159265358979323846f
// #define RAD_TO_DEG          57.2957795131f
// #define DEG_TO_RAD          0.01745329251f

// #define REG_BANK_SEL        0x7F
// #define USER_CTRL           0x03
// #define PWR_MGMT_1          0x06
// #define ACCEL_XOUT_H        0x2D
// #define GYRO_XOUT_H         0x33

// // Filter Constant
// #define MAG_LPF_ALPHA       0.2f 

// // ==========================================
// // GLOBALS
// // ==========================================
// i2c_master_bus_handle_t bus_handle;
// i2c_master_dev_handle_t h_qmc;
// i2c_master_dev_handle_t h_icm;

// VL53L5CX_Configuration DevA, DevB;
// VL53L5CX_ResultsData resA, resB;

// // Calibration Data Structure
// typedef struct {
//     float g_bias[3];      
//     float m_bias[3];      
//     float m_scale[3];     
//     float pitch_tare;     
//     float roll_tare;      
//     float heading_tare;   
// } CalibrationData;

// // ==========================================
// // HARDCODED VALUES (Update these after Calib)
// // ==========================================
// CalibrationData calib = {
//     .g_bias = {0.0f, 0.0f, 0.0f},       
//     .m_bias = {0.0f, 0.0f, 0.0f},       
//     .m_scale = {1.0f, 1.0f, 1.0f},      
//     .pitch_tare = 0.0f,                 
//     .roll_tare = 0.0f,
//     .heading_tare = 0.0f 
// };

// // Shared Fusion Data
// float current_pitch = 0.0f;
// float current_roll = 0.0f;
// float current_heading = 0.0f;

// // ==========================================
// // I2C HELPERS
// // ==========================================
// esp_err_t i2c_write_byte(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data) {
//     uint8_t buf[2] = {reg, data};
//     return i2c_master_transmit(handle, buf, 2, -1);
// }

// esp_err_t i2c_read_bytes(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
//     return i2c_master_transmit_receive(handle, &reg, 1, data, len, -1);
// }

// // ==========================================
// // INIT
// // ==========================================
// void init_imu_mag() {
//     i2c_write_byte(h_qmc, 0x0A, 0x80); 
//     vTaskDelay(pdMS_TO_TICKS(10));
//     i2c_write_byte(h_qmc, 0x09, 0x05 | 0x10); 
//     i2c_write_byte(h_qmc, 0x0B, 0x01); 

//     i2c_write_byte(h_icm, REG_BANK_SEL, 0x00);
//     i2c_write_byte(h_icm, PWR_MGMT_1, 0x01); 
//     vTaskDelay(pdMS_TO_TICKS(10));
//     i2c_write_byte(h_icm, USER_CTRL, 0x00);
// }

// // ==========================================
// // CALIBRATION ROUTINE
// // ==========================================
// void perform_calibration() {
//     printf("\n\n################################################\n");
//     printf("   ENTERING CALIBRATION MODE \n");
//     printf("################################################\n");
    
//     // --- Phase 1: Gyro & Level ---
//     printf("PHASE 1: GYRO & LEVEL\n");
//     printf("Keep board FLAT and STATIONARY.\n");
//     for(int i=3; i>0; i--) { printf("%d...\n", i); vTaskDelay(pdMS_TO_TICKS(1000)); }
//     printf(">> Measuring...\n");
    
//     long g_sum[3] = {0};
//     long a_sum[3] = {0};
//     uint8_t buf[12];
//     int samples = 500;

//     for(int i=0; i<samples; i++) {
//         i2c_read_bytes(h_icm, ACCEL_XOUT_H, buf, 6);
//         int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
//         int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
//         int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
        
//         i2c_read_bytes(h_icm, GYRO_XOUT_H, buf, 6);
//         int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
//         int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
//         int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);

//         a_sum[0] += ax; a_sum[1] += ay; a_sum[2] += az;
//         g_sum[0] += gx; g_sum[1] += gy; g_sum[2] += gz;
//         vTaskDelay(pdMS_TO_TICKS(2));
//     }

//     calib.g_bias[0] = (float)g_sum[0] / samples;
//     calib.g_bias[1] = (float)g_sum[1] / samples;
//     calib.g_bias[2] = (float)g_sum[2] / samples;

//     float ax_avg = (float)a_sum[0] / samples;
//     float ay_avg = (float)a_sum[1] / samples;
//     float az_avg = (float)a_sum[2] / samples;

//     calib.roll_tare  = atan2f(ay_avg, az_avg) * RAD_TO_DEG;
//     calib.pitch_tare = atan2f(-ax_avg, sqrtf(ay_avg*ay_avg + az_avg*az_avg)) * RAD_TO_DEG;

//     // --- Phase 2: Mag Calibration ---
//     printf("\nPHASE 2: COMPASS FIGURE-8\n");
//     printf("Pick up board. ROTATE in Figure-8.\n");
//     for(int i=3; i>0; i--) { printf("%d...\n", i); vTaskDelay(pdMS_TO_TICKS(1000)); }
//     printf(">> GO! (15 Seconds)\n");

//     int16_t m_min[3] = {30000, 30000, 30000};
//     int16_t m_max[3] = {-30000, -30000, -30000};
    
//     int64_t start_time = esp_timer_get_time();
//     int last_print_sec = 16; 

//     while (true) { 
//         int64_t current_time = esp_timer_get_time();
//         int remaining_sec = 15 - ((current_time - start_time) / 1000000);

//         if (remaining_sec < 0) break;
//         if (remaining_sec < last_print_sec) {
//             printf("Time Left: %d s...\n", remaining_sec);
//             last_print_sec = remaining_sec;
//         }

//         uint8_t m_buf[6];
//         i2c_read_bytes(h_qmc, 0x00, m_buf, 6);
//         int16_t raw_x = (int16_t)(m_buf[0] | (m_buf[1] << 8));
//         int16_t raw_y = (int16_t)(m_buf[2] | (m_buf[3] << 8));
//         int16_t raw_z = (int16_t)(m_buf[4] | (m_buf[5] << 8));

//         int16_t mx = raw_y; int16_t my = -raw_x; int16_t mz = raw_z;

//         // FIXED INDENTATION ERRORS HERE
//         if(mx < m_min[0]) m_min[0] = mx; 
//         if(mx > m_max[0]) m_max[0] = mx;
        
//         if(my < m_min[1]) m_min[1] = my; 
//         if(my > m_max[1]) m_max[1] = my;
        
//         if(mz < m_min[2]) m_min[2] = mz; 
//         if(mz > m_max[2]) m_max[2] = mz;
        
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }

//     calib.m_bias[0] = (m_max[0] + m_min[0]) / 2.0f;
//     calib.m_bias[1] = (m_max[1] + m_min[1]) / 2.0f;
//     calib.m_bias[2] = (m_max[2] + m_min[2]) / 2.0f;

//     float chord_x = (float)(m_max[0] - m_min[0]) / 2.0f;
//     float chord_y = (float)(m_max[1] - m_min[1]) / 2.0f;
//     float chord_z = (float)(m_max[2] - m_min[2]) / 2.0f;
//     float avg_chord = (chord_x + chord_y + chord_z) / 3.0f;

//     calib.m_scale[0] = avg_chord / chord_x;
//     calib.m_scale[1] = avg_chord / chord_y;
//     calib.m_scale[2] = avg_chord / chord_z;

//     // --- Phase 3: Alignment (Tare Heading) ---
//     printf("\nPHASE 3: ALIGNMENT (Set Zero)\n");
//     printf("Point the sensor exactly FORWARD (or at your phone's North).\n");
//     printf("Hold it still...\n");
//     for(int i=5; i>0; i--) { printf("%d...\n", i); vTaskDelay(pdMS_TO_TICKS(1000)); }
//     printf(">> Capturing Zero Reference...\n");

//     long align_x = 0, align_y = 0;
//     for(int i=0; i<100; i++) {
//         uint8_t m_buf[6];
//         i2c_read_bytes(h_qmc, 0x00, m_buf, 6);
//         int16_t raw_x = (int16_t)(m_buf[0] | (m_buf[1] << 8));
//         int16_t raw_y = (int16_t)(m_buf[2] | (m_buf[3] << 8));
//         // int16_t raw_z = (int16_t)(m_buf[4] | (m_buf[5] << 8)); // Not needed for 2D alignment

//         // 1. Map Axes
//         int16_t mx = raw_y; int16_t my = -raw_x; 
//         // int16_t mz = raw_z; // REMOVED UNUSED VARIABLE

//         // 2. Apply Calibration (Must be done to get correct angle)
//         float mx_cal = ((float)mx - calib.m_bias[0]) * calib.m_scale[0];
//         float my_cal = ((float)my - calib.m_bias[1]) * calib.m_scale[1];

//         // 3. Simple atan2 (Assuming level for this quick tare)
//         align_x += mx_cal;
//         align_y += my_cal;
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }

//     // Calculate the angle we are currently pointing at
//     float ref_heading = atan2f((float)align_y, (float)align_x) * RAD_TO_DEG;
//     if(ref_heading < 0) ref_heading += 360.0f;

//     // We want this direction to be 0. So the offset is the current raw heading.
//     calib.heading_tare = ref_heading;

//     printf("\n--- CALIBRATION RESULTS (Copy to code) ---\n");
//     printf(".g_bias = {%.2ff, %.2ff, %.2ff},\n", calib.g_bias[0], calib.g_bias[1], calib.g_bias[2]);
//     printf(".m_bias = {%.2ff, %.2ff, %.2ff},\n", calib.m_bias[0], calib.m_bias[1], calib.m_bias[2]);
//     printf(".m_scale = {%.4ff, %.4ff, %.4ff},\n", calib.m_scale[0], calib.m_scale[1], calib.m_scale[2]);
//     printf(".pitch_tare = %.2ff,\n", calib.pitch_tare);
//     printf(".roll_tare = %.2ff,\n", calib.roll_tare);
//     printf(".heading_tare = %.2ff\n", calib.heading_tare);
//     printf("------------------------------------------\n");
    
//     printf("Starting Normal Operation in 3s...\n");
//     vTaskDelay(pdMS_TO_TICKS(3000));
// }

// // ==========================================
// // TASK: IMU FUSION
// // ==========================================
// void task_imu_fusion(void *pvParameters) {
//     uint8_t buf[6];
//     int16_t raw_ax, raw_ay, raw_az;
//     int16_t raw_gx, raw_gy;
//     int16_t raw_mx_raw, raw_my_raw, raw_mz_raw;

//     float mx_filt = 0, my_filt = 0, mz_filt = 0;
//     float pitch = calib.pitch_tare;
//     float roll = calib.roll_tare;
    
//     int64_t last_time = esp_timer_get_time();

//     while(1) {
//         int64_t now = esp_timer_get_time();
//         float dt = (float)(now - last_time) / 1000000.0f;
//         last_time = now;

//         // --- MAG READ & FILTER ---
//         i2c_read_bytes(h_qmc, 0x00, buf, 6);
//         int16_t qmc_x = (int16_t)(buf[0] | (buf[1] << 8));
//         int16_t qmc_y = (int16_t)(buf[2] | (buf[3] << 8));
//         int16_t qmc_z = (int16_t)(buf[4] | (buf[5] << 8));
        
//         raw_mx_raw = qmc_y; raw_my_raw = -qmc_x; raw_mz_raw = qmc_z;

//         float mx_cal = ((float)raw_mx_raw - calib.m_bias[0]) * calib.m_scale[0];
//         float my_cal = ((float)raw_my_raw - calib.m_bias[1]) * calib.m_scale[1];
//         float mz_cal = ((float)raw_mz_raw - calib.m_bias[2]) * calib.m_scale[2];

//         mx_filt += MAG_LPF_ALPHA * (mx_cal - mx_filt);
//         my_filt += MAG_LPF_ALPHA * (my_cal - my_filt);
//         mz_filt += MAG_LPF_ALPHA * (mz_cal - mz_filt);

//         // --- ACCEL/GYRO READ ---
//         i2c_read_bytes(h_icm, ACCEL_XOUT_H, buf, 6);
//         raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
//         raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
//         raw_az = (int16_t)((buf[4] << 8) | buf[5]);

//         i2c_read_bytes(h_icm, GYRO_XOUT_H, buf, 6);
//         raw_gx = (int16_t)((buf[0] << 8) | buf[1]);
//         raw_gy = (int16_t)((buf[2] << 8) | buf[3]);
        
//         float gx_rate = ((float)raw_gx - calib.g_bias[0]) / 131.0f;
//         float gy_rate = ((float)raw_gy - calib.g_bias[1]) / 131.0f;

//         // --- FUSION ---
//         float acc_roll  = atan2f(raw_ay, raw_az) * RAD_TO_DEG;
//         float acc_pitch = atan2f(-raw_ax, sqrtf((float)raw_ay * raw_ay + (float)raw_az * raw_az)) * RAD_TO_DEG;

//         roll  = 0.96f * (roll  + gx_rate * dt) + 0.04f * acc_roll;
//         pitch = 0.96f * (pitch + gy_rate * dt) + 0.04f * acc_pitch;

//         // --- COMPASS ---
//         float p_rad = pitch * DEG_TO_RAD;
//         float r_rad = roll * DEG_TO_RAD;

//         float Xh = mx_filt * cosf(p_rad) + mz_filt * sinf(p_rad);
//         float Yh = mx_filt * sinf(r_rad) * sinf(p_rad) + my_filt * cosf(r_rad) - mz_filt * sinf(r_rad) * cosf(p_rad);

//         float heading = atan2f(Yh, Xh) * RAD_TO_DEG;
        
//         // --- APPLY TARE (ALIGNMENT) ---
//         heading -= calib.heading_tare; // Subtract the saved offset

//         // Normalize to 0-360
//         if(heading < 0) heading += 360.0f;
//         if(heading >= 360) heading -= 360.0f;

//         current_roll = roll - calib.roll_tare;
//         current_pitch = pitch - calib.pitch_tare;
//         current_heading = heading;

//         vTaskDelay(pdMS_TO_TICKS(20)); 
//     }
// }

// // ==========================================
// // TOF TASK
// // ==========================================
// void print_8x8_grid(const char* name, VL53L5CX_ResultsData* res) {
//     printf("\n[%s] IMU: Pitch: %6.2f | Roll: %6.2f | Head: %6.2f\n", 
//            name, current_pitch, current_roll, current_heading);
//     printf("--------------------------------\n");
//     for (int row = 0; row < 8; row++) {
//         for (int col = 0; col < 8; col++) {
//             int zone = row * 8 + col;
//             int idx = VL53L5CX_NB_TARGET_PER_ZONE * zone;
//             printf("%4d ", (int)res->distance_mm[idx]);
//         }
//         printf("\n");
//     }
// }

// void task_tof(void *pvParameters) {
//     uint8_t isReady;

//     gpio_set_level(SENSOR_A_LPN_PIN, 1);
//     vTaskDelay(pdMS_TO_TICKS(10));
    
//     i2c_device_config_t dev_cfg_a = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = VL53_DEFAULT_ADDR, .scl_speed_hz = 400000
//     };
//     i2c_master_dev_handle_t hA_init;
//     i2c_master_bus_add_device(bus_handle, &dev_cfg_a, &hA_init);
//     DevA.platform.handle = hA_init;
    
//     vl53l5cx_set_i2c_address(&DevA, VL53_ADDR_A << 1); 
//     i2c_master_bus_rm_device(hA_init);
//     dev_cfg_a.device_address = VL53_ADDR_A;
//     i2c_master_bus_add_device(bus_handle, &dev_cfg_a, &DevA.platform.handle);

//     gpio_set_level(SENSOR_B_LPN_PIN, 1);
//     vTaskDelay(pdMS_TO_TICKS(10));

//     i2c_device_config_t dev_cfg_b = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = VL53_DEFAULT_ADDR, .scl_speed_hz = 400000
//     };
//     i2c_master_dev_handle_t hB_init;
//     i2c_master_bus_add_device(bus_handle, &dev_cfg_b, &hB_init);
//     DevB.platform.handle = hB_init;

//     vl53l5cx_set_i2c_address(&DevB, VL53_ADDR_B << 1);
//     i2c_master_bus_rm_device(hB_init);
//     dev_cfg_b.device_address = VL53_ADDR_B;
//     i2c_master_bus_add_device(bus_handle, &dev_cfg_b, &DevB.platform.handle);

//     printf("Initializing ToF Firmware...\n");
//     vl53l5cx_init(&DevA);
//     vl53l5cx_init(&DevB);

//     vl53l5cx_set_resolution(&DevA, VL53L5CX_RESOLUTION_8X8);
//     vl53l5cx_set_resolution(&DevB, VL53L5CX_RESOLUTION_8X8);
//     vl53l5cx_set_ranging_frequency_hz(&DevA, 10);
//     vl53l5cx_set_ranging_frequency_hz(&DevB, 10);

//     vl53l5cx_start_ranging(&DevA);
//     vl53l5cx_start_ranging(&DevB);

//     while(1) {
//         if (vl53l5cx_check_data_ready(&DevA, &isReady) == 0 && isReady) {
//             vl53l5cx_get_ranging_data(&DevA, &resA);
//             print_8x8_grid("SENSOR A", &resA);
//         }
//         if (vl53l5cx_check_data_ready(&DevB, &isReady) == 0 && isReady) {
//             vl53l5cx_get_ranging_data(&DevB, &resB);
//             print_8x8_grid("SENSOR B", &resB);
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// // ==========================================
// // APP MAIN
// // ==========================================
// void app_main(void) {
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << SENSOR_A_LPN_PIN) | (1ULL << SENSOR_B_LPN_PIN),
//         .mode = GPIO_MODE_OUTPUT,
//     };
//     gpio_config(&io_conf);
//     gpio_set_level(SENSOR_A_LPN_PIN, 0); 
//     gpio_set_level(SENSOR_B_LPN_PIN, 0);

//     i2c_master_bus_config_t bus_config = {
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .i2c_port = I2C_PORT,
//         .scl_io_num = SCL_IO_PIN,
//         .sda_io_num = SDA_IO_PIN,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };
//     ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

//     i2c_device_config_t qmc_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = QMC5883L_ADDR, .scl_speed_hz = 400000 };
//     i2c_master_bus_add_device(bus_handle, &qmc_cfg, &h_qmc);

//     i2c_device_config_t icm_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = ICM20948_ADDR, .scl_speed_hz = 400000 };
//     i2c_master_bus_add_device(bus_handle, &icm_cfg, &h_icm);

//     init_imu_mag();

//     printf("\n=== SYSTEM BOOT ===\n");
//     if(DO_CALIBRATE) {
//         perform_calibration();
//     } else {
//         printf("Skipping Calibration (Using Hardcoded Values)...\n");
//     }

//     xTaskCreate(task_imu_fusion, "IMU_Task", 4096, NULL, 5, NULL);
//     xTaskCreate(task_tof, "ToF_Task", 16384, NULL, 4, NULL); 
// }
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "vl53l5cx_api.h"

// ==========================================
// USER CONFIGURATION
// ==========================================
#define DO_CALIBRATE_DEFAULT false // Overridden by NVS or BOOT button
#define BOOT_BUTTON_PIN      GPIO_NUM_0

#define I2C_PORT            I2C_NUM_1
#define SCL_IO_PIN          GPIO_NUM_7
#define SDA_IO_PIN          GPIO_NUM_8
#define SENSOR_A_LPN_PIN    GPIO_NUM_52
#define SENSOR_B_LPN_PIN    GPIO_NUM_29

#define QMC5883L_ADDR       0x0D
#define ICM20948_ADDR       0x69
#define VL53_DEFAULT_ADDR   0x29
#define VL53_ADDR_A         0x22
#define VL53_ADDR_B         0x24

#define PI                  3.14159265358979323846f
#define RAD_TO_DEG          57.2957795131f
#define DEG_TO_RAD          0.01745329251f

#define REG_BANK_SEL        0x7F
#define USER_CTRL           0x03
#define PWR_MGMT_1          0x06
#define ACCEL_XOUT_H        0x2D
#define GYRO_XOUT_H         0x33
#define GYRO_CONFIG_1       0x1B

// Sensor Constants
#define MAG_LPF_ALPHA       0.2f 
#define GYRO_SCALE_250DPS   131.0f
#define ACCEL_EPSILON       0.0001f // Prevents Div-by-Zero at 0G

// NVS Protection
#define CALIB_MAGIC_WORD    0xCA11BEEF

// ==========================================
// GLOBALS & STRUCTURES
// ==========================================
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t h_qmc;
i2c_master_dev_handle_t h_icm;

VL53L5CX_Configuration DevA, DevB;
VL53L5CX_ResultsData resA, resB;

// Calibration Data Structure (Strictly packed for NVS safety)
typedef struct __attribute__((packed)) {
    uint32_t magic_word;  
    float g_bias[3];      
    float m_bias[3];      
    float m_scale[3];     
    float pitch_tare;     
    float roll_tare;      
    float heading_tare;   
} CalibrationData;

CalibrationData calib = {
    .magic_word = 0,
    .g_bias = {0.0f, 0.0f, 0.0f},       
    .m_bias = {0.0f, 0.0f, 0.0f},       
    .m_scale = {1.0f, 1.0f, 1.0f},      
    .pitch_tare = 0.0f,                 
    .roll_tare = 0.0f,
    .heading_tare = 0.0f 
};

// Shared Fusion Data
float current_pitch = 0.0f;
float current_roll = 0.0f;
float current_heading = 0.0f;

// Thread Safety
SemaphoreHandle_t fusion_mutex;

// ==========================================
// I2C HELPERS
// ==========================================
esp_err_t i2c_write_byte(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(handle, buf, 2, -1);
}

esp_err_t i2c_read_bytes(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(handle, &reg, 1, data, len, -1);
}

// ==========================================
// NVS STORAGE ARCHITECTURE
// ==========================================
void init_nvs_safe() {
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_LOGW("NVS", "NVS partition truncated/corrupted. Erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void save_calibration_to_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err;

    printf("\nSaving calibration to NVS...\n");
    
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    calib.magic_word = CALIB_MAGIC_WORD;

    err = nvs_set_blob(my_handle, "imu_cal", &calib, sizeof(CalibrationData));
    if (err != ESP_OK) {
        printf("Failed to write blob to NVS! (%s)\n", esp_err_to_name(err));
    } else {
        err = nvs_commit(my_handle);
        if (err == ESP_OK) {
            printf("Calibration data successfully saved to flash memory!\n");
        }
    }

    nvs_close(my_handle);
}

bool load_calibration_from_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        printf("NVS empty or uninitialized. (Normal on first boot)\n");
        return false;
    }

    size_t required_size = 0;
    err = nvs_get_blob(my_handle, "imu_cal", NULL, &required_size);
    if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND)) {
        nvs_close(my_handle);
        return false;
    }

    if (required_size != sizeof(CalibrationData)) {
        printf("NVS Data size mismatch! Firmware updated? Forcing recalibration.\n");
        nvs_close(my_handle);
        return false;
    }

    CalibrationData temp_cal;
    err = nvs_get_blob(my_handle, "imu_cal", &temp_cal, &required_size);
    nvs_close(my_handle);

    if (err != ESP_OK) {
        return false;
    }

    if (temp_cal.magic_word != CALIB_MAGIC_WORD) {
        printf("NVS Data format corrupted or outdated. Forcing recalibration.\n");
        return false;
    }

    for (int i = 0; i < 3; i++) {
        if (isnan(temp_cal.g_bias[i]) || isinf(temp_cal.g_bias[i])) { return false; }
        if (isnan(temp_cal.m_bias[i]) || isinf(temp_cal.m_bias[i])) { return false; }
        
        if (isnan(temp_cal.m_scale[i]) || isinf(temp_cal.m_scale[i]) || 
           (temp_cal.m_scale[i] <= 0.05f) || (temp_cal.m_scale[i] > 20.0f)) {
            printf("CRITICAL: Corrupted Magnetometer scale factor detected! Forcing recalibration.\n");
            return false;
        }
    }

    calib = temp_cal;

    printf("\n--- LOADED CALIBRATION FROM NVS ---\n");
    printf(".g_bias = {%.2ff, %.2ff, %.2ff}\n", calib.g_bias[0], calib.g_bias[1], calib.g_bias[2]);
    printf(".m_bias = {%.2ff, %.2ff, %.2ff}\n", calib.m_bias[0], calib.m_bias[1], calib.m_bias[2]);
    printf(".m_scale = {%.4ff, %.4ff, %.4ff}\n", calib.m_scale[0], calib.m_scale[1], calib.m_scale[2]);
    printf(".pitch_tare = %.2ff\n", calib.pitch_tare);
    printf(".roll_tare = %.2ff\n", calib.roll_tare);
    printf(".heading_tare = %.2ff\n", calib.heading_tare);
    printf("-----------------------------------\n");

    return true;
}

// ==========================================
// INIT IMU
// ==========================================
void init_imu_mag() {
    i2c_write_byte(h_qmc, 0x0A, 0x80); 
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_byte(h_qmc, 0x09, 0x05 | 0x10); 
    i2c_write_byte(h_qmc, 0x0B, 0x01); 

    i2c_write_byte(h_icm, REG_BANK_SEL, 0x00);
    i2c_write_byte(h_icm, PWR_MGMT_1, 0x01); 
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_byte(h_icm, USER_CTRL, 0x00);

    // Explicitly set Gyro range to +/- 250 dps (Bank 2, Reg 0x1B)
    i2c_write_byte(h_icm, REG_BANK_SEL, 0x20); // Switch to Bank 2
    i2c_write_byte(h_icm, GYRO_CONFIG_1, 0x00); // 250 dps
    i2c_write_byte(h_icm, REG_BANK_SEL, 0x00); // Switch back to Bank 0
}

// ==========================================
// CALIBRATION ROUTINE
// ==========================================
void perform_calibration() {
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
    uint8_t buf[12];
    int samples = 500;

    for(int i = 0; i < samples; i++) {
        if (i2c_read_bytes(h_icm, ACCEL_XOUT_H, buf, 6) == ESP_OK) {
            int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
            a_sum[0] += ax; 
            a_sum[1] += ay; 
            a_sum[2] += az;
        }
        
        if (i2c_read_bytes(h_icm, GYRO_XOUT_H, buf, 6) == ESP_OK) {
            int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);
            g_sum[0] += gx; 
            g_sum[1] += gy; 
            g_sum[2] += gz;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    calib.g_bias[0] = (float)g_sum[0] / samples;
    calib.g_bias[1] = (float)g_sum[1] / samples;
    calib.g_bias[2] = (float)g_sum[2] / samples;

    float ax_avg = (float)a_sum[0] / samples;
    float ay_avg = (float)a_sum[1] / samples;
    float az_avg = (float)a_sum[2] / samples;

    float acc_denom = sqrtf(ay_avg * ay_avg + az_avg * az_avg);
    if (acc_denom < ACCEL_EPSILON) {
        acc_denom = ACCEL_EPSILON;
    }

    calib.roll_tare  = atan2f(ay_avg, az_avg) * RAD_TO_DEG;
    calib.pitch_tare = atan2f(-ax_avg, acc_denom) * RAD_TO_DEG;

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
    int last_print_sec = 16; 

    while (true) { 
        int64_t current_time = esp_timer_get_time();
        int remaining_sec = 15 - ((current_time - start_time) / 1000000);

        if (remaining_sec < 0) {
            break;
        }
        if (remaining_sec < last_print_sec) {
            printf("Time Left: %d s...\n", remaining_sec);
            last_print_sec = remaining_sec;
        }

        uint8_t m_buf[6];
        if (i2c_read_bytes(h_qmc, 0x00, m_buf, 6) == ESP_OK) {
            int16_t raw_x = (int16_t)(m_buf[0] | (m_buf[1] << 8));
            int16_t raw_y = (int16_t)(m_buf[2] | (m_buf[3] << 8));
            int16_t raw_z = (int16_t)(m_buf[4] | (m_buf[5] << 8));

            int16_t mx = raw_y; 
            int16_t my = -raw_x; 
            int16_t mz = raw_z;

            if (mx < m_min[0]) { m_min[0] = mx; }
            if (mx > m_max[0]) { m_max[0] = mx; }
            
            if (my < m_min[1]) { m_min[1] = my; }
            if (my > m_max[1]) { m_max[1] = my; }
            
            if (mz < m_min[2]) { m_min[2] = mz; }
            if (mz > m_max[2]) { m_max[2] = mz; }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    calib.m_bias[0] = (m_max[0] + m_min[0]) / 2.0f;
    calib.m_bias[1] = (m_max[1] + m_min[1]) / 2.0f;
    calib.m_bias[2] = (m_max[2] + m_min[2]) / 2.0f;

    float chord_x = (float)(m_max[0] - m_min[0]) / 2.0f;
    float chord_y = (float)(m_max[1] - m_min[1]) / 2.0f;
    float chord_z = (float)(m_max[2] - m_min[2]) / 2.0f;
    
    if (chord_x < 1.0f) { chord_x = 1.0f; }
    if (chord_y < 1.0f) { chord_y = 1.0f; }
    if (chord_z < 1.0f) { chord_z = 1.0f; }

    float avg_chord = (chord_x + chord_y + chord_z) / 3.0f;

    calib.m_scale[0] = avg_chord / chord_x;
    calib.m_scale[1] = avg_chord / chord_y;
    calib.m_scale[2] = avg_chord / chord_z;

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
        uint8_t m_buf[6];
        if (i2c_read_bytes(h_qmc, 0x00, m_buf, 6) == ESP_OK) {
            int16_t raw_x = (int16_t)(m_buf[0] | (m_buf[1] << 8));
            int16_t raw_y = (int16_t)(m_buf[2] | (m_buf[3] << 8));

            int16_t mx = raw_y; 
            int16_t my = -raw_x; 

            float mx_cal = ((float)mx - calib.m_bias[0]) * calib.m_scale[0];
            float my_cal = ((float)my - calib.m_bias[1]) * calib.m_scale[1];

            align_x += mx_cal;
            align_y += my_cal;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float ref_heading = atan2f((float)align_y, (float)align_x) * RAD_TO_DEG;
    if (ref_heading < 0.0f) { 
        ref_heading += 360.0f; 
    }

    calib.heading_tare = ref_heading;

    // Save strictly to Flash NVS
    save_calibration_to_nvs();

    printf("Starting Normal Operation in 3s...\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
}

// ==========================================
// TASK: IMU FUSION
// ==========================================
void task_imu_fusion(void *pvParameters) {
    uint8_t buf[6];
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy;
    int16_t raw_mx_raw, raw_my_raw, raw_mz_raw;

    float mx_filt = 0, my_filt = 0, mz_filt = 0;
    float pitch = calib.pitch_tare;
    float roll = calib.roll_tare;
    
    int64_t last_time = esp_timer_get_time();

    while(1) {
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;

        // --- MAG READ & FILTER ---
        if (i2c_read_bytes(h_qmc, 0x00, buf, 6) == ESP_OK) {
            int16_t qmc_x = (int16_t)(buf[0] | (buf[1] << 8));
            int16_t qmc_y = (int16_t)(buf[2] | (buf[3] << 8));
            int16_t qmc_z = (int16_t)(buf[4] | (buf[5] << 8));
            
            raw_mx_raw = qmc_y; 
            raw_my_raw = -qmc_x; 
            raw_mz_raw = qmc_z;

            float mx_cal = ((float)raw_mx_raw - calib.m_bias[0]) * calib.m_scale[0];
            float my_cal = ((float)raw_my_raw - calib.m_bias[1]) * calib.m_scale[1];
            float mz_cal = ((float)raw_mz_raw - calib.m_bias[2]) * calib.m_scale[2];

            mx_filt += MAG_LPF_ALPHA * (mx_cal - mx_filt);
            my_filt += MAG_LPF_ALPHA * (my_cal - my_filt);
            mz_filt += MAG_LPF_ALPHA * (mz_cal - mz_filt);
        }

        // --- ACCEL/GYRO READ ---
        bool read_success = true;
        
        if (i2c_read_bytes(h_icm, ACCEL_XOUT_H, buf, 6) == ESP_OK) {
            raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
            raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
            raw_az = (int16_t)((buf[4] << 8) | buf[5]);
        } else {
            read_success = false;
        }

        if (i2c_read_bytes(h_icm, GYRO_XOUT_H, buf, 6) == ESP_OK) {
            raw_gx = (int16_t)((buf[0] << 8) | buf[1]);
            raw_gy = (int16_t)((buf[2] << 8) | buf[3]);
        } else {
            read_success = false;
        }

        // --- FUSION ---
        if (read_success) {
            float gx_rate = ((float)raw_gx - calib.g_bias[0]) / GYRO_SCALE_250DPS;
            float gy_rate = ((float)raw_gy - calib.g_bias[1]) / GYRO_SCALE_250DPS;

            float acc_roll  = atan2f(raw_ay, raw_az) * RAD_TO_DEG;
            
            float acc_denom = sqrtf((float)raw_ay * raw_ay + (float)raw_az * raw_az);
            if (acc_denom < ACCEL_EPSILON) {
                acc_denom = ACCEL_EPSILON;
            }
            float acc_pitch = atan2f(-raw_ax, acc_denom) * RAD_TO_DEG;

            roll  = 0.96f * (roll  + gx_rate * dt) + 0.04f * acc_roll;
            pitch = 0.96f * (pitch + gy_rate * dt) + 0.04f * acc_pitch;

            // --- COMPASS ---
            float p_rad = pitch * DEG_TO_RAD;
            float r_rad = roll * DEG_TO_RAD;

            float Xh = mx_filt * cosf(p_rad) + mz_filt * sinf(p_rad);
            float Yh = mx_filt * sinf(r_rad) * sinf(p_rad) + my_filt * cosf(r_rad) - mz_filt * sinf(r_rad) * cosf(p_rad);

            float heading = atan2f(Yh, Xh) * RAD_TO_DEG;
            
            heading -= calib.heading_tare; 

            if (heading < 0.0f) { 
                heading += 360.0f; 
            }
            if (heading >= 360.0f) { 
                heading -= 360.0f; 
            }

            // --- THREAD SAFE WRITE ---
            if (xSemaphoreTake(fusion_mutex, portMAX_DELAY) == pdTRUE) {
                current_roll = roll - calib.roll_tare;
                current_pitch = pitch - calib.pitch_tare;
                current_heading = heading;
                xSemaphoreGive(fusion_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

// ==========================================
// TOF TASK
// ==========================================
void print_8x8_grid(const char* name, VL53L5CX_ResultsData* res) {
    float local_pitch = 0.0f;
    float local_roll = 0.0f;
    float local_heading = 0.0f;

    // --- THREAD SAFE READ ---
    if (xSemaphoreTake(fusion_mutex, portMAX_DELAY) == pdTRUE) {
        local_pitch = current_pitch;
        local_roll = current_roll;
        local_heading = current_heading;
        xSemaphoreGive(fusion_mutex);
    }

    printf("\n[%s] IMU: Pitch: %6.2f | Roll: %6.2f | Head: %6.2f\n", 
           name, local_pitch, local_roll, local_heading);
    printf("--------------------------------\n");
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            int zone = row * 8 + col;
            int idx = VL53L5CX_NB_TARGET_PER_ZONE * zone;
            printf("%4d ", (int)res->distance_mm[idx]);
        }
        printf("\n");
    }
}

void task_tof(void *pvParameters) {
    uint8_t isReady;

    gpio_set_level(SENSOR_A_LPN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    i2c_device_config_t dev_cfg_a = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, 
        .device_address = VL53_DEFAULT_ADDR, 
        .scl_speed_hz = 400000
    };
    i2c_master_dev_handle_t hA_init;
    i2c_master_bus_add_device(bus_handle, &dev_cfg_a, &hA_init);
    DevA.platform.handle = hA_init;
    
    vl53l5cx_set_i2c_address(&DevA, VL53_ADDR_A << 1); 
    i2c_master_bus_rm_device(hA_init);
    dev_cfg_a.device_address = VL53_ADDR_A;
    i2c_master_bus_add_device(bus_handle, &dev_cfg_a, &DevA.platform.handle);

    gpio_set_level(SENSOR_B_LPN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_device_config_t dev_cfg_b = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, 
        .device_address = VL53_DEFAULT_ADDR, 
        .scl_speed_hz = 400000
    };
    i2c_master_dev_handle_t hB_init;
    i2c_master_bus_add_device(bus_handle, &dev_cfg_b, &hB_init);
    DevB.platform.handle = hB_init;

    vl53l5cx_set_i2c_address(&DevB, VL53_ADDR_B << 1);
    i2c_master_bus_rm_device(hB_init);
    dev_cfg_b.device_address = VL53_ADDR_B;
    i2c_master_bus_add_device(bus_handle, &dev_cfg_b, &DevB.platform.handle);

    printf("Initializing ToF Firmware...\n");
    vl53l5cx_init(&DevA);
    vl53l5cx_init(&DevB);

    vl53l5cx_set_resolution(&DevA, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_resolution(&DevB, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(&DevA, 10);
    vl53l5cx_set_ranging_frequency_hz(&DevB, 10);

    vl53l5cx_start_ranging(&DevA);
    vl53l5cx_start_ranging(&DevB);

    while(1) {
        if ((vl53l5cx_check_data_ready(&DevA, &isReady) == 0) && isReady) {
            vl53l5cx_get_ranging_data(&DevA, &resA);
            print_8x8_grid("SENSOR A", &resA);
        }
        if ((vl53l5cx_check_data_ready(&DevB, &isReady) == 0) && isReady) {
            vl53l5cx_get_ranging_data(&DevB, &resB);
            print_8x8_grid("SENSOR B", &resB);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// APP MAIN
// ==========================================
void app_main(void) {
    // 1. Initialize Thread Safety
    fusion_mutex = xSemaphoreCreateMutex();

    // 2. Initialize Hardware & Bus
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SENSOR_A_LPN_PIN) | (1ULL << SENSOR_B_LPN_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(SENSOR_A_LPN_PIN, 0); 
    gpio_set_level(SENSOR_B_LPN_PIN, 0);

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t qmc_cfg = { 
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, 
        .device_address = QMC5883L_ADDR, 
        .scl_speed_hz = 400000 
    };
    i2c_master_bus_add_device(bus_handle, &qmc_cfg, &h_qmc);

    i2c_device_config_t icm_cfg = { 
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, 
        .device_address = ICM20948_ADDR, 
        .scl_speed_hz = 400000 
    };
    i2c_master_bus_add_device(bus_handle, &icm_cfg, &h_icm);

    init_nvs_safe();
    init_imu_mag();

    printf("\n=== SYSTEM BOOT ===\n");

    // 3. Hardware Override Check (BOOT button)
    gpio_set_direction(BOOT_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOOT_BUTTON_PIN, GPIO_PULLUP_ONLY);
    
    bool force_calib_via_button = false;
    printf("Press and hold the BOOT button (GPIO 0) NOW to force recalibration...\n");
    for(int i = 3; i > 0; i--) {
        if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            force_calib_via_button = true;
            printf("\n>> BUTTON DETECTED! Forcing Recalibration...\n");
            break;
        }
        printf("%d...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 4. Smart Boot Logic
    bool nvs_load_success = load_calibration_from_nvs();

    if (force_calib_via_button || !nvs_load_success || DO_CALIBRATE_DEFAULT) {
        printf("Proceeding to Calibration Routine...\n");
        perform_calibration();
    } else {
        printf("Valid Calibration found in NVS. Skipping Calibration.\n");
    }

    // 5. Start RTOS Tasks
    xTaskCreate(task_imu_fusion, "IMU_Task", 4096, NULL, 5, NULL);
    xTaskCreate(task_tof, "ToF_Task", 16384, NULL, 4, NULL); 
}