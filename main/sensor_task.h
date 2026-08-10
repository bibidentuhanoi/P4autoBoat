#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drivers/tof_driver.h"
#include "imu_sample.h"
#include "sample_snapshot.h"

/** @brief Publish the existing 20 Hz cached sensor snapshot. */
void task_sensor_snapshot(void *pvParameters);

/**
 * @brief Sole runtime I2C owner: acquire IMU at 50 Hz and scheduled ToF grids.
 *        Pass tof_devices_t* as pvParameters.
 */
void task_sensor_bus(void *pvParameters);

/** @brief Convert stable raw ToF generations into the existing processed cache. */
void task_tof_processor(void *pvParameters);

/** @brief Create the ToF cache mutex. Call once before starting either task. */
esp_err_t sensor_task_init(void);

/** Acquire one raw IMU generation and update chip health/recovery state. */
bool sensor_read_imu_sample(imu_sample_t *sample);

/** Latest raw IMU generation for the fusion consumer. */
sample_snapshot_t *sensor_imu_sample_snapshot(void);

/** Called by the fusion task before acquisition starts sending notifications. */
void sensor_task_register_fusion_task(TaskHandle_t task);
