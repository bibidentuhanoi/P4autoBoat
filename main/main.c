#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Logic Layers
#include "common.h"
#include "file_system.h"
#include "sensor_fusion.h"
#include "calibration.h"

// Drivers
#include "drivers/imu_driver.h"
#include "drivers/tof_driver.h"

// Configuration
#define DO_CALIBRATE_DEFAULT false
#define BOOT_BUTTON_PIN      GPIO_NUM_0
#define I2C_SDA_PIN          CONFIG_I2C_SDA_PIN
#define I2C_SCL_PIN          CONFIG_I2C_SCL_PIN

// Globals
static i2c_master_bus_handle_t bus_handle;
static CalibrationData calib_data = {
    .magic_word = 0,
    .g_bias = {0.0f, 0.0f, 0.0f},       
    .m_bias = {0.0f, 0.0f, 0.0f},       
    .m_scale = {1.0f, 1.0f, 1.0f},      
    .pitch_tare = 0.0f,                 
    .roll_tare = 0.0f,
    .heading_tare = 0.0f 
};

static tof_devices_t tof_devs;

// ==========================================
// TOF TASK
// ==========================================
void print_8x8_grid(const char* name, VL53L5CX_ResultsData* res) {
    FusionResult fusion_res;
    fusion_get_result(&fusion_res);

    printf("\n[%s] IMU: Pitch: %6.2f | Roll: %6.2f | Head: %6.2f\n", 
           name, fusion_res.pitch, fusion_res.roll, fusion_res.heading);
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

void task_tof_reporting(void *pvParameters) {
    VL53L5CX_ResultsData resA, resB;
    
    while(1) {
        if (tof_read_grid(&tof_devs.dev_a, &resA) == ESP_OK) {
            print_8x8_grid("SENSOR A", &resA);
        }
        if (tof_read_grid(&tof_devs.dev_b, &resB) == ESP_OK) {
            print_8x8_grid("SENSOR B", &resB);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// APP MAIN
// ==========================================
void app_main(void) {
    // 1. Initialize NVS
    fs_init();

    // 2. Initialize Hardware & Bus
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // 3. Initialize Drivers
    ESP_ERROR_CHECK(imu_init(bus_handle));
    // ToF Init handles its own LPN pins and address hacking
    ESP_ERROR_CHECK(tof_init(bus_handle, &tof_devs));

    printf("\n=== SYSTEM BOOT ===\n");

    // 4. Hardware Override Check (BOOT button)
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

    // 5. Smart Boot Logic
    bool nvs_load_success = fs_load_calibration(&calib_data);

    if (force_calib_via_button || !nvs_load_success || DO_CALIBRATE_DEFAULT) {
        printf("Proceeding to Calibration Routine...\n");
        perform_calibration_routine(&calib_data);
        calib_data.magic_word = CALIB_MAGIC_WORD;
        fs_save_calibration(&calib_data);
    } else {
        printf("Valid Calibration found in NVS. Skipping Calibration.\n");
    }

    // 6. Initialize Fusion Logic
    fusion_init(&calib_data);

    // 7. Start RTOS Tasks
    xTaskCreate(task_imu_fusion, "IMU_Task", 4096, NULL, 5, NULL);
    xTaskCreate(task_tof_reporting, "ToF_Task", 16384, NULL, 4, NULL);
}
