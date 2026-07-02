#include "tof_driver.h"
#include "file_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Configuration macros will be provided by Kconfig in the future,
// for now we use the ones defined in sdkconfig or default if missing
#include "sdkconfig.h"

static const char *TAG = "TOF";

/* Xtalk calibration: 600mm target, 3% reflectance, 16 samples (ST recommendation) */
#define XTALK_DISTANCE_MM   1000
#define XTALK_REFLECTANCE   3
#define XTALK_SAMPLES       16

static void tof_apply_xtalk(VL53L5CX_Configuration *dev, const char *nvs_key, const char *label)
{
    uint8_t xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE];

    if (fs_load_tof_xtalk(nvs_key, xtalk_data, sizeof(xtalk_data))) {
        uint8_t status = vl53l5cx_set_caldata_xtalk(dev, xtalk_data);
        if (status == 0) {
            ESP_LOGI(TAG, "Sensor %s: xtalk cal loaded from NVS", label);
        } else {
            ESP_LOGW(TAG, "Sensor %s: xtalk set failed (%d), running calibration", label, status);
            goto calibrate;
        }
        return;
    }

calibrate:
    ESP_LOGI(TAG, "Sensor %s: running xtalk calibration (place %dmm target)…", label, XTALK_DISTANCE_MM);
    uint8_t status = vl53l5cx_calibrate_xtalk(dev, XTALK_REFLECTANCE, XTALK_SAMPLES, XTALK_DISTANCE_MM);
    if (status == 0) {
        vl53l5cx_get_caldata_xtalk(dev, xtalk_data);
        fs_save_tof_xtalk(nvs_key, xtalk_data, sizeof(xtalk_data));
        ESP_LOGI(TAG, "Sensor %s: xtalk calibration complete, dumping %d bytes:", label, VL53L5CX_XTALK_BUFFER_SIZE);
        for (int i = 0; i < VL53L5CX_XTALK_BUFFER_SIZE; i += 16) {
            int remaining = VL53L5CX_XTALK_BUFFER_SIZE - i;
            if (remaining > 16) remaining = 16;
            char hex[64];
            int pos = 0;
            for (int j = 0; j < remaining; j++) {
                pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", xtalk_data[i + j]);
            }
            ESP_LOGI(TAG, "  [%03X] %s", i, hex);
        }
    } else {
        ESP_LOGW(TAG, "Sensor %s: xtalk calibration failed (%d), using defaults", label, status);
    }
}

#define TOF_A_LPN_PIN CONFIG_TOF_A_LPN_PIN
#define TOF_B_LPN_PIN CONFIG_TOF_B_LPN_PIN
#define TOF_A_ADDR    CONFIG_TOF_A_ADDR
#define TOF_B_ADDR    CONFIG_TOF_B_ADDR

/* Add the device at `addr` and check it answers. On success the handle is left
 * in dev->platform.handle; on failure the handle is released and NULLed. */
static bool tof_probe_at(i2c_master_bus_handle_t bus, VL53L5CX_Configuration *dev, uint8_t addr)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    if (i2c_master_bus_add_device(bus, &cfg, &dev->platform.handle) != ESP_OK) {
        dev->platform.handle = NULL;
        return false;
    }
    uint8_t alive = 0;
    if (vl53l5cx_is_alive(dev, &alive) == 0 && alive) {
        return true;
    }
    i2c_master_bus_rm_device(dev->platform.handle);
    dev->platform.handle = NULL;
    return false;
}

/*
 * Probe and bring up one VL53L5CX. Returns true only if the sensor is present
 * and fully started. On any failure it logs a warning, releases the i2c handle,
 * and returns false so the caller can keep booting without this sensor.
 *
 * The sensor's LPN pin must already be driven high by the caller (and the OTHER
 * sensor still moved off the default address / held in reset) before this runs.
 *
 * Probe order matters: the I2C address survives soft resets (LPn gates comms,
 * it does NOT reset the chip — only a power cycle does), so on a warm boot the
 * sensor already sits at its target address. Check there first, then default.
 */
static bool tof_init_one(i2c_master_bus_handle_t bus, VL53L5CX_Configuration *dev,
                         uint8_t new_addr, const char *nvs_key, const char *label)
{
    if (tof_probe_at(bus, dev, new_addr)) {
        ESP_LOGI(TAG, "Sensor %s: already at 0x%02X (warm boot)", label, new_addr);
    } else if (tof_probe_at(bus, dev, VL53_DEFAULT_ADDR)) {
        /* Cold boot: at the default address — move it. The ULD's return status
         * is unreliable here (its verification read still uses the old-address
         * handle), so ignore it; the re-probe below is the real check. */
        (void)vl53l5cx_set_i2c_address(dev, (uint16_t)(new_addr << 1));
        i2c_master_bus_rm_device(dev->platform.handle);
        dev->platform.handle = NULL;
        if (!tof_probe_at(bus, dev, new_addr)) {
            ESP_LOGW(TAG, "Sensor %s: no response at 0x%02X after address change", label, new_addr);
            return false;
        }
    } else {
        ESP_LOGW(TAG, "Sensor %s: NOT DETECTED (0x%02X or 0x%02X) — skipping",
                 label, new_addr, VL53_DEFAULT_ADDR);
        return false;
    }

    /* Upload firmware (~84KB). */
    if (vl53l5cx_init(dev) != 0) {
        ESP_LOGW(TAG, "Sensor %s: firmware init failed", label);
        return false;
    }

    /* Xtalk is best-effort — never fatal. */
    tof_apply_xtalk(dev, nvs_key, label);

    vl53l5cx_set_resolution(dev, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(dev, CONFIG_TOF_RANGING_FREQ_HZ);
    vl53l5cx_set_integration_time_ms(dev, CONFIG_TOF_INTEGRATION_TIME_MS);
    vl53l5cx_set_target_order(dev, VL53L5CX_TARGET_ORDER_STRONGEST);

    if (vl53l5cx_start_ranging(dev) != 0) {
        ESP_LOGW(TAG, "Sensor %s: start_ranging failed", label);
        return false;
    }

    ESP_LOGI(TAG, "Sensor %s: ready (addr 0x%02X)", label, new_addr);
    return true;
}

esp_err_t tof_init(i2c_master_bus_handle_t bus_handle, tof_devices_t* devices) {
    devices->a_ok = false;
    devices->b_ok = false;

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

    // Sensor A — enable and bring up while B is still held in reset.
    gpio_set_level(TOF_A_LPN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    devices->a_ok = tof_init_one(bus_handle, &devices->dev_a, TOF_A_ADDR, "xtalk_a", "A");

    // Sensor B — enable now that A (if present) has moved off the default address.
    gpio_set_level(TOF_B_LPN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    devices->b_ok = tof_init_one(bus_handle, &devices->dev_b, TOF_B_ADDR, "xtalk_b", "B");

    ESP_LOGI(TAG, "ToF init: A=%s B=%s",
             devices->a_ok ? "OK" : "absent", devices->b_ok ? "OK" : "absent");

    // Always OK — a missing sensor is non-fatal; presence is reported via a_ok/b_ok.
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
