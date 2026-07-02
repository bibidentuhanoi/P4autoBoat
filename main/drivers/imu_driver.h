#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "driver/i2c_master.h"
#include "esp_err.h"

// Register Addresses
#define QMC5883L_ADDR       0x0D
#define ICM20948_ADDR       0x69   /* AD0 high/floating (breakout default) */
#define ICM20948_ADDR_ALT   0x68   /* AD0 bridged to GND */

#define REG_BANK_SEL        0x7F
#define USER_CTRL           0x03
#define PWR_MGMT_1          0x06
#define ACCEL_XOUT_H        0x2D
#define GYRO_XOUT_H         0x33
#define GYRO_CONFIG_1       0x1B

// Constants
#define GYRO_SCALE_250DPS   131.0f

esp_err_t imu_init(i2c_master_bus_handle_t bus_handle);
esp_err_t imu_read_accel_gyro(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
esp_err_t imu_read_mag(int16_t* mx, int16_t* my, int16_t* mz);
/* Re-apply chip config after a lost-connection recovery (see imu_driver.c). */
esp_err_t imu_reinit_mag(void);
esp_err_t imu_reinit_accel_gyro(void);

#endif // IMU_DRIVER_H
