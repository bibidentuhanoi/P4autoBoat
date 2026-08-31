#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
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
#include "drivers/steer_driver.h"
#include "drivers/status_led.h"

// Connectivity
#include "wifi_manager.h"
#include "camera_stream.h"
#include "pipeline.h"
#include "http_server.h"
#include "sensor_task.h"
#include "detect_task.h"
#include "training_log_task.h"
#include "motor_control.h"
#include "esc_trim.h"
#include "transports/espnow_transport.h"
#include "drivers/sd_card.h"
#include "runtime_metrics.h"
#include "runtime_startup.h"
#include "runtime_task.h"
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
volatile bool g_camera_ok = false;
volatile bool g_field_mode = false;

// ==========================================
// APP MAIN
// ==========================================
void app_main(void) {
    ESP_LOGI(TAG, "=== SYSTEM BOOT ===");
    runtime_metrics_init();

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
        /* Camera absent ⇒ esp_video attached NO device to the SCCB bus, so we
         * CAN delete it. That releases GPIO7/8 so the sensor bus (I2C_NUM_1)
         * below is the SOLE controller on those pins. Leaving I2C_NUM_0 bound to
         * the same pins made both sensors glitch in lockstep (two controllers,
         * one pin pair). Falls back to the old shared-pin path if delete fails. */
        esp_err_t del = i2c_del_master_bus(sccb_handle);
        if (del == ESP_OK) {
            sccb_handle = NULL;
            ESP_LOGI(TAG, "SCCB bus released — GPIO7/8 now owned solely by the sensor bus");
        } else {
            ESP_LOGW(TAG, "SCCB bus delete failed (%s) — sensor bus will share the pins",
                     esp_err_to_name(del));
        }
    }

    // 3b. If the camera IS present, esp_video attached its OV5647 device to
    //     sccb_handle, so i2c_del_master_bus returns ESP_ERR_INVALID_STATE and
    //     I2C_NUM_0 stays alive. The sensor bus below then re-routes GPIO7/GPIO8
    //     via the GPIO matrix, and the boot settle-delay in imu_init covers it.

    // 4. Create sensor I2C bus on same GPIO7/GPIO8 (GPIO matrix re-routes from I2C_NUM_0)
    ESP_LOGI(TAG, "Initializing sensor I2C bus...");
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

    // User LED (GPIO3): blink through the button window so the operator knows —
    // without a serial terminal — that pressing BOOT now forces recalibration.
    status_led_init();
    status_led_set(STATUS_LED_CAL_WINDOW);

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
        status_led_set(STATUS_LED_OFF);   // cal-only LED: dark once we're running
    }

    // ESC differential-trim table -- own NVS record ("esc_trim"), entirely
    // independent of the IMU calibration blob loaded above. Empty table
    // (count == 0) is always a safe default.
    EscTrimNvsBlob esc_trim_blob;
    fs_load_esc_trim(&esc_trim_blob);
    {
        /* Priority: proportional c  >  flat hand-measured  >  NVS table.
         *
         * A hand-measured trim OVERRIDES whatever calibration is in NVS. The
         * stored table comes from a sweep that was never confirmed, and it
         * would otherwise silently outrank a value measured directly on the
         * bench. The bench measures the RAW motors (esc_trim_mix is skipped
         * while a run is active), so a bench figure is the true mismatch, not a
         * residual on top of the table. */
        float c = strtof(CONFIG_ESC_TRIM_C, NULL);
        float flat = strtof(CONFIG_ESC_TRIM_DEFAULT, NULL);
        EscTrimPoint prop[2];
        uint8_t prop_count = esc_trim_build_proportional(c, prop);

        if (prop_count > 0) {
            if (esc_trim_blob.count > 0) {
                ESP_LOGW(TAG, "ESC trim: IGNORING the %u-point table in NVS",
                         (unsigned)esc_trim_blob.count);
            }
            memset(&esc_trim_blob, 0, sizeof(esc_trim_blob));
            memcpy(esc_trim_blob.points, prop, sizeof(prop));
            esc_trim_blob.count = prop_count;
            /* Print what it actually commands at the levels that get bench-run,
             * so the boot log can be checked against the SD files directly. */
            ESP_LOGI(TAG, "ESC trim: c=%.3f -- split scales with throttle "
                          "(left = T*%.2f, right = T*%.2f)",
                     (double)c, (double)(1.0f - c), (double)(1.0f + c));
            for (int pct = 10; pct <= 50; pct += 10) {
                float t = (float)pct / 100.0f;
                float l = t, r = t;
                esc_trim_mix(t, 0.0f, esc_trim_blob.points, esc_trim_blob.count,
                             &l, &r);
                ESP_LOGI(TAG, "ESC trim:   T%02d -> L %.1f%% / R %.1f%% "
                              "(split %+.2f%%)",
                         pct, (double)(l * 100.0f), (double)(r * 100.0f),
                         (double)((r - l) * 50.0f));
            }
        } else if (isfinite(flat) && flat != 0.0f) {
            if (esc_trim_blob.count > 0) {
                ESP_LOGW(TAG, "ESC trim: IGNORING the %u-point table in NVS",
                         (unsigned)esc_trim_blob.count);
            }
            memset(&esc_trim_blob, 0, sizeof(esc_trim_blob));
            esc_trim_blob.points[0].throttle_frac = 1.0f;
            esc_trim_blob.points[0].trim_diff = flat;
            esc_trim_blob.count = 1;
            ESP_LOGW(TAG, "ESC trim: FLAT %.3f (%+.1f%% to the RIGHT motor) at "
                          "every throttle -- correct only near the throttle it "
                          "was measured at; prefer ESC_TRIM_C",
                     (double)flat, (double)(flat * 50.0f));
        } else if (esc_trim_blob.count > 0) {
            ESP_LOGI(TAG, "ESC trim: using the %u-point table from NVS "
                          "(no hand-measured value configured)",
                     (unsigned)esc_trim_blob.count);
        } else {
            ESP_LOGI(TAG, "ESC trim: none -- motors run untrimmed");
        }
    }
    motor_control_set_esc_trim(esc_trim_blob.points, esc_trim_blob.count);

    // 8b. Winch servo + servo power. GPIO35 is dual-use with the BOOT button,
    //     so this MUST come after the button read above. Reconfigures GPIO35
    //     from input to MCPWM output (neutral/stop) and GPIO36 to output (off).
    ESP_LOGI(TAG, "Initializing winch servo + servo power...");
    ESP_ERROR_CHECK(winch_driver_init());

    // 8c. Steering rudder servos (GPIO32, MCPWM group 1) — homes to full-left on boot.
    ESP_LOGI(TAG, "Initializing steering servos...");
    ESP_ERROR_CHECK(steer_driver_init());

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
        runtime_startup_handle_task_failure(RUNTIME_TASK_GPS, gps_ret);
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
    esp_err_t detect_ret = detect_init();
    if (detect_ret != ESP_OK) {
        ESP_LOGW(TAG, "Detection unavailable (%s) — manual control continues",
                 esp_err_to_name(detect_ret));
    }

    // 10. Bring up the radio WITHOUT associating. The co-processor needs
    //     esp_wifi_start() before esp_now_init(), and not connecting yet keeps
    //     the radio parked on one channel so the ESP-NOW probe below is valid.
    ESP_LOGI(TAG, "Starting WiFi radio...");
    esp_err_t radio_ret = wifi_start_radio();
    esp_err_t wifi_ret  = ESP_FAIL;

#if CONFIG_ESPNOW_ENABLED
    // 10a. ESP-NOW FIRST: listen for the S3 ground station's beacon. Hearing it
    //      proves the field link is actually live, which is a far better signal
    //      than "WiFi failed" — and it skips the 30 s AP timeout entirely.
    bool field_mode = false;
    if (radio_ret == ESP_OK) {
        esp_err_t probe = espnow_transport_probe(CONFIG_ESPNOW_PROBE_MS);
        if (probe == ESP_OK) {
            ESP_LOGI(TAG, "Ground station present — ESP-NOW field mode");
            /* Latch WiFi off: its reconnect/scan would drag the radio away
             * from the ESP-NOW channel and never restore it. */
            wifi_manager_stop_reconnect();
            if (espnow_transport_activate() == ESP_OK) {
                field_mode = true;
                /* Trim the ToF diagnostic arrays out of every snapshot; set
                 * BEFORE the sensor task starts, so no full-size frame can
                 * escape first. */
                g_field_mode = true;
            }
        } else if (probe == ESP_ERR_NOT_FOUND) {
            ESP_LOGI(TAG, "No ground station — falling back to WiFi");
        } else {
            ESP_LOGW(TAG, "ESP-NOW probe failed (%s) — falling back to WiFi",
                     esp_err_to_name(probe));
        }
    }

    if (!field_mode && radio_ret == ESP_OK) {
        wifi_ret = wifi_connect();
    }

    if (field_mode) {
        /* nothing more to start: no HTTP/WS servers in field mode */
    } else if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi unavailable (%s) — no connectivity", esp_err_to_name(wifi_ret));
    } else {
#else
    if (radio_ret == ESP_OK) {
        wifi_ret = wifi_connect();
    }

    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi unavailable (%s) — camera stream disabled", esp_err_to_name(wifi_ret));
    } else {
#endif
        // 11. Start servers — stream on port 81, API/dashboard on port 80
        if (camera_ok) {
            esp_err_t stream_ret = camera_stream_server_start();
            if (stream_ret != ESP_OK) {
                ESP_LOGW(TAG, "Camera stream unavailable (%s)",
                         esp_err_to_name(stream_ret));
            }
        }
        esp_err_t http_ret = http_server_start();
        if (http_ret != ESP_OK) {
            ESP_LOGW(TAG, "HTTP/WebSocket unavailable (%s) — manual control continues",
                     esp_err_to_name(http_ret));
        }

        // 11b. ESCs stay DISARMED at boot. Arming is gated on a GPS lock and
        //      requested by the user from the dashboard (bench override available).
        ESP_LOGI(TAG, "ESCs disarmed — arm from dashboard once GPS locks (or override).");
    }

    /* GPIO sleep-switching: the IDF enables it at boot (SleepSelEn=1 on every pad).
     * This boat never sleeps (WiFi PS_NONE, no PM), so kill the switching globally
     * HERE — after every stage that could turn it back on — so actuator outputs stay
     * solid, and release any hold a later stage re-applied to the actuator pads. */
    esp_sleep_enable_gpio_switch(false);
    gpio_hold_dis(CONFIG_ESC_PWM_LEFT_PIN);
    gpio_hold_dis(CONFIG_ESC_PWM_RIGHT_PIN);
    gpio_hold_dis(CONFIG_STEER_PIN);
    gpio_hold_dis(CONFIG_WINCH_PWM_PIN);

    // 12. Start RTOS Tasks — loud failure: a silent Snap_Task death means no
    //     telemetry at all (dashboard shows no IMU/ToF/GPS and arming stays locked).
    ESP_LOGI(TAG, "Starting tasks...");
    TaskHandle_t fusion_task = NULL;
    esp_err_t task_error = runtime_task_create(RUNTIME_TASK_FUSION,
                                               task_imu_fusion, NULL,
                                               &fusion_task);
    if (task_error != ESP_OK) {
        ESP_ERROR_CHECK(runtime_startup_handle_task_failure(RUNTIME_TASK_FUSION,
                                                            task_error));
    } else {
        sensor_task_register_fusion_task(fusion_task);
    }
    /* 12b. microSD — LAST, and soft-optional. It shares the SDMMC peripheral
     *      with the C6 (card on slot 0, co-processor on slot 1), so it is
     *      brought up only after the radio is established and a failure here
     *      must never take connectivity down with it. */
    if (sd_card_init() != ESP_OK) {
        ESP_LOGW(TAG, "No SD card — data logging unavailable, flight continues");
    }

    ESP_ERROR_CHECK(sensor_task_init());
    task_error = runtime_task_create(RUNTIME_TASK_SENSOR_BUS, task_sensor_bus,
                                     NULL, NULL);
    if (task_error != ESP_OK) {
        ESP_ERROR_CHECK(runtime_startup_handle_task_failure(RUNTIME_TASK_SENSOR_BUS,
                                                            task_error));
    }
    /* ToF acquisition is its own task now (2026-08-11 split, see sensor_task.c) --
     * decoupled from SensorBus's IMU timing, lower priority so IMU always
     * wins any I2C contention. */
    task_error = runtime_task_create(RUNTIME_TASK_TOF_READ,
                                     task_tof_read, &tof_devs, NULL);
    if (task_error != ESP_OK) {
        runtime_startup_handle_task_failure(RUNTIME_TASK_TOF_READ, task_error);
    }
    task_error = runtime_task_create(RUNTIME_TASK_TOF_PROCESS,
                                     task_tof_processor, NULL, NULL);
    if (task_error != ESP_OK) {
        runtime_startup_handle_task_failure(RUNTIME_TASK_TOF_PROCESS, task_error);
    }
    task_error = runtime_task_create(RUNTIME_TASK_SNAPSHOT,
                                     task_sensor_snapshot, &tof_devs, NULL);
    if (task_error != ESP_OK) {
        runtime_startup_handle_task_failure(RUNTIME_TASK_SNAPSHOT, task_error);
    }
    task_error = runtime_task_create(RUNTIME_TASK_DIAGNOSTICS,
                                     task_runtime_diagnostics, NULL, NULL);
    if (task_error != ESP_OK) {
        runtime_startup_handle_task_failure(RUNTIME_TASK_DIAGNOSTICS, task_error);
    }

    // Dataset capture (JPEG + sensor sidecar to SD, trigger via WS/ESP-NOW).
    // Needs sensor_task_init() (sensor_tof_cache_load()) above; does not
    // need SD/camera to already be ready -- both are checked per-capture.
    esp_err_t training_log_ret = training_log_init();
    if (training_log_ret != ESP_OK) {
        ESP_LOGW(TAG, "Dataset capture unavailable (%s) — manual control continues",
                 esp_err_to_name(training_log_ret));
    }

    ESP_LOGI(TAG, "System running.");
}
