#include "tof_driver.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Configuration macros will be provided by Kconfig in the future,
// for now we use the ones defined in sdkconfig or default if missing
#include "sdkconfig.h"

#define TOF_A_LPN_PIN CONFIG_TOF_A_LPN_PIN
#define TOF_B_LPN_PIN CONFIG_TOF_B_LPN_PIN
#define TOF_A_ADDR    CONFIG_TOF_A_ADDR
#define TOF_B_ADDR    CONFIG_TOF_B_ADDR

esp_err_t tof_init(i2c_master_bus_handle_t bus_handle, tof_devices_t* devices) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOF_A_LPN_PIN) | (1ULL << TOF_B_LPN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Reset both sensors
    gpio_set_level(TOF_A_LPN_PIN, 0);
    gpio_set_level(TOF_B_LPN_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable Sensor A
    gpio_set_level(TOF_A_LPN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_device_config_t dev_cfg_a = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53_DEFAULT_ADDR,
        .scl_speed_hz = 400000
    };
    i2c_master_dev_handle_t hA_init;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_a, &hA_init));
    devices->dev_a.platform.handle = hA_init;

    // Change address of Sensor A
    vl53l5cx_set_i2c_address(&devices->dev_a, TOF_A_ADDR << 1);

    // Re-add device with new address
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(hA_init));
    dev_cfg_a.device_address = TOF_A_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_a, &devices->dev_a.platform.handle));

    // Enable Sensor B
    gpio_set_level(TOF_B_LPN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_device_config_t dev_cfg_b = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53_DEFAULT_ADDR,
        .scl_speed_hz = 400000
    };
    i2c_master_dev_handle_t hB_init;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_b, &hB_init));
    devices->dev_b.platform.handle = hB_init;

    // Change address of Sensor B
    vl53l5cx_set_i2c_address(&devices->dev_b, TOF_B_ADDR << 1);

    // Re-add device with new address
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(hB_init));
    dev_cfg_b.device_address = TOF_B_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_b, &devices->dev_b.platform.handle));

    // Initialize Firmware
    vl53l5cx_init(&devices->dev_a);
    vl53l5cx_init(&devices->dev_b);

    vl53l5cx_set_resolution(&devices->dev_a, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_resolution(&devices->dev_b, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(&devices->dev_a, 10);
    vl53l5cx_set_ranging_frequency_hz(&devices->dev_b, 10);

    vl53l5cx_start_ranging(&devices->dev_a);
    vl53l5cx_start_ranging(&devices->dev_b);

    return ESP_OK;
}

esp_err_t tof_read_grid(VL53L5CX_Configuration* dev, VL53L5CX_ResultsData* results) {
    uint8_t isReady;
    if (vl53l5cx_check_data_ready(dev, &isReady) == 0 && isReady) {
        vl53l5cx_get_ranging_data(dev, results);
        return ESP_OK;
    }
    return ESP_FAIL;
}
