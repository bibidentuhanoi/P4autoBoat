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
#include "drivers/camera_driver.h"

// Connectivity
#include "wifi_manager.h"
#include "camera_stream.h"
#include "sensor_api.h"
static const char* TAG = "MAIN";

// Configuration
#define DO_CALIBRATE_DEFAULT false
#define BOOT_BUTTON_PIN      GPIO_NUM_35
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
// APP MAIN
// ==========================================
void app_main(void) {
    ESP_LOGI(TAG, "=== SYSTEM BOOT ===");

    // 1. Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    fs_init();

    // 2. Initialize Camera (XCLK must start ASAP after boot — before I2C sensor init)
    //    Uses I2C_NUM_0 (SCL=8, SDA=7) for SCCB. Sequential with I2C_NUM_1 below.
    ESP_LOGI(TAG, "Initializing camera...");
    ESP_ERROR_CHECK(camera_init());

    // 3. Initialize sensor I2C Bus (I2C_NUM_1: SCL=7, SDA=8)
    ESP_LOGI(TAG, "Initializing I2C bus...");
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // 3. Initialize IMU (fast — just register writes)
    ESP_LOGI(TAG, "Initializing IMU...");
    ESP_ERROR_CHECK(imu_init(bus_handle));

    // 4. Initialize ToF (slow — uploads ~84KB firmware blob to 2 sensors over I2C)
    ESP_LOGI(TAG, "Initializing ToF sensors (uploading firmware, please wait)...");
    ESP_ERROR_CHECK(tof_init(bus_handle, &tof_devs));
    ESP_LOGI(TAG, "ToF ready.");

    // 5. Hardware Override Check (BOOT button)
    gpio_set_direction(BOOT_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOOT_BUTTON_PIN, GPIO_PULLUP_ONLY);

    bool force_calib_via_button = false;
    ESP_LOGI(TAG, "Press and hold the BOOT button (GPIO 35) NOW to force recalibration...");
    for (int i = 3; i > 0; i--) {
        if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
            force_calib_via_button = true;
            ESP_LOGW(TAG, "BUTTON DETECTED! Forcing recalibration...");
            break;
        }
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 6. Smart Boot Logic
    bool nvs_load_success = fs_load_calibration(&calib_data);

    if (force_calib_via_button || !nvs_load_success || DO_CALIBRATE_DEFAULT) {
        ESP_LOGI(TAG, "Proceeding to calibration routine...");
        perform_calibration_routine(&calib_data);
        calib_data.magic_word = CALIB_MAGIC_WORD;
        fs_save_calibration(&calib_data);
    } else {
        ESP_LOGI(TAG, "Valid calibration found in NVS. Skipping calibration.");
    }

    // 7. Initialize Sensor Fusion
    ESP_LOGI(TAG, "Initializing sensor fusion...");
    fusion_init(&calib_data);

    // 8. Connect to WiFi (blocks until connected or timeout)
    ESP_LOGI(TAG, "Connecting to WiFi...");
    esp_err_t wifi_ret = wifi_init();
    httpd_handle_t httpd_handle = NULL;
    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi unavailable (%s) — camera stream disabled", esp_err_to_name(wifi_ret));
    } else {
        // 9. Start MJPEG HTTP stream server + sensor API
        ESP_ERROR_CHECK(camera_stream_server_start(&httpd_handle));
        ESP_ERROR_CHECK(sensor_api_register(httpd_handle, &tof_devs));
    }

    // 10. Start RTOS Tasks
    ESP_LOGI(TAG, "Starting tasks...");
    xTaskCreate(task_imu_fusion,       "IMU_Task",  4096,  NULL,      5, NULL);
    xTaskCreate(task_sensor_snapshot,  "Snap_Task", 16384, &tof_devs, 4, NULL);

    ESP_LOGI(TAG, "System running.");
}