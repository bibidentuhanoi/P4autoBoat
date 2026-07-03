#include "imu_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "IMU";

static i2c_master_dev_handle_t h_qmc;
static i2c_master_dev_handle_t h_icm;

/* Health flags: seeded by the boot probes below, kept current by the fusion
 * task (sustained failures ⇒ false, recovery ⇒ true). Plain bool writes are
 * atomic on this target — no locking needed for status reporting. */
static volatile bool s_icm_ok = false;
static volatile bool s_mag_ok = false;

bool imu_icm_ok(void)          { return s_icm_ok; }
bool imu_mag_ok(void)          { return s_mag_ok; }
void imu_set_icm_ok(bool ok)   { s_icm_ok = ok; }
void imu_set_mag_ok(bool ok)   { s_mag_ok = ok; }

/* Bounded timeout — a marginal/shorted bus with -1 (wait forever) can wedge
 * the whole boot (seen on hardware: hung mid ToF-B bring-up while the IMU
 * module was loading the bus). 100ms is orders of magnitude above any legit
 * transaction at 400kHz. */
#define IMU_I2C_TIMEOUT_MS 100

static esp_err_t i2c_write_byte(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(handle, buf, 2, IMU_I2C_TIMEOUT_MS);
}

static esp_err_t i2c_read_bytes(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(handle, &reg, 1, data, len, IMU_I2C_TIMEOUT_MS);
}

esp_err_t imu_init(i2c_master_bus_handle_t bus_handle) {
    // QMC5883L Init — 100 kHz: the IMU sits on a long wire run and a marginal
    // joint that passes at 100 kHz (Arduino scanner default) fails at 400 kHz.
    // 6-byte reads every 20 ms need < 1 ms even at 100 kHz.
    i2c_device_config_t qmc_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMC5883L_ADDR,
        .scl_speed_hz = 100000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &qmc_cfg, &h_qmc));

    /* Presence check with retries: any successful read means the mag answers. */
    uint8_t qmc_status = 0;
    esp_err_t qmc_probe = ESP_FAIL;
    for (int attempt = 0; attempt < 3 && qmc_probe != ESP_OK; attempt++) {
        if (attempt) vTaskDelay(pdMS_TO_TICKS(20));
        qmc_probe = i2c_read_bytes(h_qmc, 0x06, &qmc_status, 1);
    }
    s_mag_ok = (qmc_probe == ESP_OK);
    if (qmc_probe != ESP_OK) {
        ESP_LOGE(TAG, "QMC5883L (0x%02X) NOT RESPONDING (%s) — heading will be dead",
                 QMC5883L_ADDR, esp_err_to_name(qmc_probe));
    } else {
        ESP_LOGI(TAG, "QMC5883L detected");
    }

    i2c_write_byte(h_qmc, 0x0A, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_byte(h_qmc, 0x09, 0x05); // ±2G, 50Hz, OSR=512, continuous
    i2c_write_byte(h_qmc, 0x0B, 0x01);

    // ICM20948 Init
    i2c_device_config_t icm_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_ADDR,
        .scl_speed_hz = 100000   /* see QMC comment — robustness over long wires */
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &icm_cfg, &h_icm));

    /* WHO_AM_I check (bank 0, reg 0x00) — ICM20948 answers 0xEA.
     * The AD0 strap selects the address: high/floating = 0x69, GND = 0x68.
     * Probe 0x69 first (with retries), fall back to 0x68 so either strap works. */
    uint8_t whoami = 0;
    esp_err_t icm_probe = ESP_FAIL;
    for (int attempt = 0; attempt < 3 && icm_probe != ESP_OK; attempt++) {
        if (attempt) vTaskDelay(pdMS_TO_TICKS(20));
        i2c_write_byte(h_icm, REG_BANK_SEL, 0x00);
        icm_probe = i2c_read_bytes(h_icm, 0x00, &whoami, 1);
    }
    if (icm_probe != ESP_OK) {
        i2c_master_bus_rm_device(h_icm);
        icm_cfg.device_address = ICM20948_ADDR_ALT;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &icm_cfg, &h_icm));
        for (int attempt = 0; attempt < 3 && icm_probe != ESP_OK; attempt++) {
            if (attempt) vTaskDelay(pdMS_TO_TICKS(20));
            i2c_write_byte(h_icm, REG_BANK_SEL, 0x00);
            icm_probe = i2c_read_bytes(h_icm, 0x00, &whoami, 1);
        }
        if (icm_probe == ESP_OK) {
            ESP_LOGW(TAG, "ICM20948 found at 0x%02X (AD0 low) — using it", ICM20948_ADDR_ALT);
        }
    }
    s_icm_ok = (icm_probe == ESP_OK);
    if (icm_probe != ESP_OK) {
        ESP_LOGE(TAG, "ICM20948 NOT RESPONDING at 0x%02X or 0x%02X (%s) — pitch/roll will be dead",
                 ICM20948_ADDR, ICM20948_ADDR_ALT, esp_err_to_name(icm_probe));
    } else if (whoami != 0xEA) {
        ESP_LOGW(TAG, "IMU WHO_AM_I=0x%02X (ICM20948 expects 0xEA) — different chip?", whoami);
    } else {
        ESP_LOGI(TAG, "ICM20948 detected (WHO_AM_I=0xEA)");
    }

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

/* Re-apply the QMC5883L config (soft reset -> continuous mode). Needed when
 * the chip was unreachable during imu_init (flaky wiring seen on hardware):
 * unconfigured it sits in standby and returns stale zeros despite ACKing. */
esp_err_t imu_reinit_mag(void) {
    esp_err_t e0 = i2c_write_byte(h_qmc, 0x0A, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_err_t e1 = i2c_write_byte(h_qmc, 0x09, 0x05); // ±2G, 50Hz, OSR=512, continuous
    esp_err_t e2 = i2c_write_byte(h_qmc, 0x0B, 0x01);
    return (e0 == ESP_OK && e1 == ESP_OK && e2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

/* Re-apply the ICM20948 config (wake + gyro range). Unconfigured, the chip
 * boots ASLEEP (PWR_MGMT_1=0x41) and returns zeros despite ACKing. */
esp_err_t imu_reinit_accel_gyro(void) {
    esp_err_t e0 = i2c_write_byte(h_icm, REG_BANK_SEL, 0x00);
    esp_err_t e1 = i2c_write_byte(h_icm, PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_byte(h_icm, USER_CTRL, 0x00);
    i2c_write_byte(h_icm, REG_BANK_SEL, 0x20);
    i2c_write_byte(h_icm, GYRO_CONFIG_1, 0x00); // 250 dps
    esp_err_t e2 = i2c_write_byte(h_icm, REG_BANK_SEL, 0x00);
    return (e0 == ESP_OK && e1 == ESP_OK && e2 == ESP_OK) ? ESP_OK : ESP_FAIL;
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
