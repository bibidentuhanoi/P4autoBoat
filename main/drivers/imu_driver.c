#include "imu_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static i2c_master_dev_handle_t h_qmc;
static i2c_master_dev_handle_t h_icm;

static esp_err_t i2c_write_byte(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(handle, buf, 2, -1);
}

static esp_err_t i2c_read_bytes(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(handle, &reg, 1, data, len, -1);
}

esp_err_t imu_init(i2c_master_bus_handle_t bus_handle) {
    // QMC5883L Init
    i2c_device_config_t qmc_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMC5883L_ADDR,
        .scl_speed_hz = 400000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &qmc_cfg, &h_qmc));

    i2c_write_byte(h_qmc, 0x0A, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_byte(h_qmc, 0x09, 0x05); // ±2G, 50Hz, OSR=512, continuous
    i2c_write_byte(h_qmc, 0x0B, 0x01);

    // ICM20948 Init
    i2c_device_config_t icm_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_ADDR,
        .scl_speed_hz = 400000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &icm_cfg, &h_icm));

    i2c_write_byte(h_icm, REG_BANK_SEL, 0x00);
    i2c_write_byte(h_icm, PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_byte(h_icm, USER_CTRL, 0x00);

    // Explicitly set Gyro range to +/- 250 dps (Bank 2, Reg 0x1B)
    i2c_write_byte(h_icm, REG_BANK_SEL, 0x20); // Switch to Bank 2
    i2c_write_byte(h_icm, GYRO_CONFIG_1, 0x00); // 250 dps
    i2c_write_byte(h_icm, REG_BANK_SEL, 0x00); // Switch back to Bank 0

    return ESP_OK;
}

esp_err_t imu_read_accel_gyro(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buf[6];
    esp_err_t ret = ESP_OK;

    if (i2c_read_bytes(h_icm, ACCEL_XOUT_H, buf, 6) == ESP_OK) {
        *ax = (int16_t)((buf[0] << 8) | buf[1]);
        *ay = (int16_t)((buf[2] << 8) | buf[3]);
        *az = (int16_t)((buf[4] << 8) | buf[5]);
    } else {
        ret = ESP_FAIL;
    }

    if (i2c_read_bytes(h_icm, GYRO_XOUT_H, buf, 6) == ESP_OK) {
        *gx = (int16_t)((buf[0] << 8) | buf[1]);
        *gy = (int16_t)((buf[2] << 8) | buf[3]);
        *gz = (int16_t)((buf[4] << 8) | buf[5]);
    } else {
        ret = ESP_FAIL;
    }

    return ret;
}

esp_err_t imu_read_mag(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t buf[6];
    if (i2c_read_bytes(h_qmc, 0x00, buf, 6) == ESP_OK) {
        int16_t qmc_x = (int16_t)(buf[0] | (buf[1] << 8));
        int16_t qmc_y = (int16_t)(buf[2] | (buf[3] << 8));
        int16_t qmc_z = (int16_t)(buf[4] | (buf[5] << 8));

        // Axis remapping as per original code
        *mx = qmc_y;
        *my = -qmc_x;
        *mz = qmc_z;
        return ESP_OK;
    }
    return ESP_FAIL;
}
