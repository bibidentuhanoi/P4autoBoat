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
#include "drivers/gps_driver.h"
#include "drivers/winch_driver.h"

// Connectivity
#include "wifi_manager.h"
#include "camera_stream.h"
#include "pipeline.h"
#include "http_server.h"
#include "sensor_task.h"
#include "detect_task.h"
#include "motor_control.h"
#include "transports/espnow_transport.h"
static const char* TAG = "MAIN";

// Configuration
#define DO_CALIBRATE_DEFAULT false
#define BOOT_BUTTON_PIN      GPIO_NUM_35

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
SemaphoreHandle_t g_i2c_mutex = NULL;
volatile bool g_camera_ok = false;

// ==========================================
// APP MAIN
// ==========================================
void app_main(void) {
    ESP_LOGI(TAG, "=== SYSTEM BOOT ===");

    // 1. Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    fs_init();

    // 1b. Start ESC PWM at neutral — must be running before ESCs see power
    ESP_LOGI(TAG, "Initializing ESC PWM (neutral)...");
    ESP_ERROR_CHECK(motor_control_init_hw());

    // 2. Create temporary SCCB bus for camera init, then hand off GPIO7/GPIO8 to sensor bus
    ESP_LOGI(TAG, "Initializing camera SCCB bus (temporary)...");
    i2c_master_bus_handle_t sccb_handle;
    i2c_master_bus_config_t sccb_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = CONFIG_CAM_SCCB_SCL_PIN,  // GPIO 8
        .sda_io_num = CONFIG_CAM_SCCB_SDA_PIN,  // GPIO 7
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&sccb_config, &sccb_handle));
    ESP_LOGI(TAG, "SCCB bus: port=%d SCL=%d SDA=%d",
             I2C_NUM_0, CONFIG_CAM_SCCB_SCL_PIN, CONFIG_CAM_SCCB_SDA_PIN);

    /* Let SCCB bus stabilize — pull-ups charge, OV5647 finishes POR */
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. Initialize Camera — programs OV5647 over SCCB bus
    ESP_LOGI(TAG, "Initializing camera...");
    bool camera_ok = (camera_init(sccb_handle) == ESP_OK);
    g_camera_ok = camera_ok;
    if (!camera_ok) {
        ESP_LOGW(TAG, "Camera init failed — streaming disabled, sensors still run");
    }

    // 3b. esp_video attached its OV5647 device to sccb_handle internally — i2c_del_master_bus
    //     would fail with ESP_ERR_INVALID_STATE. We don't need to delete: once camera_init()
    //     returns, SCCB is done forever (camera streams via MIPI CSI). I2C_NUM_0 stays alive
    //     but idle. I2C_NUM_1 below will re-route GPIO7/GPIO8 via the GPIO matrix.

    // 4. Create sensor I2C bus on same GPIO7/GPIO8 (GPIO matrix re-routes from I2C_NUM_0)
    ESP_LOGI(TAG, "Initializing sensor I2C bus...");
    g_i2c_mutex = xSemaphoreCreateMutex();
    assert(g_i2c_mutex);
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .scl_io_num = CONFIG_I2C_SCL_PIN,  // GPIO 7
        .sda_io_num = CONFIG_I2C_SDA_PIN,  // GPIO 8
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(TAG, "Sensor I2C bus: port=%d SCL=%d SDA=%d",
             I2C_NUM_1, CONFIG_I2C_SCL_PIN, CONFIG_I2C_SDA_PIN);

    // 5. Initialize IMU (sensor bus)
    ESP_LOGI(TAG, "Initializing IMU...");
    ESP_ERROR_CHECK(imu_init(bus_handle));

    // 6. Initialize ToF (sensor bus — slow, uploads ~84KB firmware).
    //     Fault-tolerant: a missing sensor is logged and skipped; boot continues.
    ESP_LOGI(TAG, "Initializing ToF sensors (uploading firmware, please wait)...");
    tof_init(bus_handle, &tof_devs);
    if (!tof_devs.a_ok || !tof_devs.b_ok) {
        ESP_LOGW(TAG, "ToF degraded: A=%s B=%s — continuing without missing sensor(s)",
                 tof_devs.a_ok ? "ok" : "MISSING", tof_devs.b_ok ? "ok" : "MISSING");
    } else {
        ESP_LOGI(TAG, "ToF ready.");
    }

    // 7. Hardware Override Check (BOOT button)
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

    // 8. Smart Boot Logic
    bool nvs_load_success = fs_load_calibration(&calib_data);

    if (force_calib_via_button || !nvs_load_success || DO_CALIBRATE_DEFAULT) {
        ESP_LOGI(TAG, "Proceeding to calibration routine...");
        perform_calibration_routine(&calib_data);
        calib_data.magic_word = CALIB_MAGIC_WORD;
        fs_save_calibration(&calib_data);
    } else {
        ESP_LOGI(TAG, "Valid calibration found in NVS. Skipping calibration.");
    }

    // 8b. Winch servo + servo power. GPIO35 is dual-use with the BOOT button,
    //     so this MUST come after the button read above. Reconfigures GPIO35
    //     from input to MCPWM output (neutral/stop) and GPIO36 to output (off).
    ESP_LOGI(TAG, "Initializing winch servo + servo power...");
    ESP_ERROR_CHECK(winch_driver_init());

    // 9. Initialize Sensor Fusion
    ESP_LOGI(TAG, "Initializing sensor fusion...");
    fusion_init(&calib_data);

    // 9b. Initialize GPS (UART, soft-optional)
    ESP_LOGI(TAG, "Initializing GPS (UART%d RX=%d TX=%d @ %d baud)...",
             CONFIG_GPS_UART_NUM, CONFIG_GPS_RX_PIN, CONFIG_GPS_TX_PIN, CONFIG_GPS_BAUD);
    esp_err_t gps_ret = gps_driver_init(CONFIG_GPS_UART_NUM,
                                         CONFIG_GPS_RX_PIN,
                                         CONFIG_GPS_TX_PIN,
                                         CONFIG_GPS_BAUD);
    if (gps_ret != ESP_OK) {
        ESP_LOGW(TAG, "GPS init failed (%s) — snapshots will omit GPS fix",
                 esp_err_to_name(gps_ret));
    }

    // Initialize data pipeline
    ESP_LOGI(TAG, "Initializing data pipeline...");
    ESP_ERROR_CHECK(pipeline_init());

    // Register motor control handlers (after pipeline_init so handlers aren't zeroed)
    ESP_LOGI(TAG, "Registering motor control handlers...");
    ESP_ERROR_CHECK(motor_control_init());

    // Initialize detection task (lazy-loads model on first trigger)
    ESP_LOGI(TAG, "Initializing detection task...");
    detect_init();

    // 10. Connect to WiFi (blocks until connected or timeout)
    ESP_LOGI(TAG, "Connecting to WiFi...");
    esp_err_t wifi_ret = wifi_init();

#if CONFIG_ESPNOW_ENABLED
    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi unavailable — switching to ESP-NOW field mode");
        esp_err_t en_ret = espnow_transport_init();
        if (en_ret != ESP_OK) {
            ESP_LOGE(TAG, "ESP-NOW init failed (%s) — no connectivity", esp_err_to_name(en_ret));
        }
    } else {
#else
    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi unavailable (%s) — camera stream disabled", esp_err_to_name(wifi_ret));
    } else {
#endif
        // 11. Start servers — stream on port 81, API/dashboard on port 80
        if (camera_ok) {
            ESP_ERROR_CHECK(camera_stream_server_start());
        }
        ESP_ERROR_CHECK(http_server_start());

        // 11b. Arm ESCs (blocking ~3 s — neutral PWM running since step 1b)
        ESP_LOGI(TAG, "Arming ESCs...");
        esp_err_t esc_ret = motor_control_arm();
        if (esc_ret != ESP_OK) {
            ESP_LOGW(TAG, "ESC arming failed (%s) — arm via dashboard later",
                     esp_err_to_name(esc_ret));
        }
    }

    // 12. Start RTOS Tasks
    ESP_LOGI(TAG, "Starting tasks...");
    xTaskCreate(task_imu_fusion,       "IMU_Task",  4096,  NULL,      4, NULL);
    xTaskCreate(task_sensor_snapshot,  "Snap_Task", 16384, &tof_devs, 4, NULL);

    ESP_LOGI(TAG, "System running.");
}
