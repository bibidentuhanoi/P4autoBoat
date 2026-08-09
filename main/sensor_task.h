#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drivers/tof_driver.h"
#include "imu_sample.h"
#include "sample_snapshot.h"

/**
 * @brief FreeRTOS task: samples IMU + both ToFs at ~5Hz,
 *        encodes as protobuf, publishes via pipeline.
 *        Pass tof_devices_t* as pvParameters.
 */
void task_sensor_snapshot(void *pvParameters);

/**
 * @brief Poll the ToF sensors at their own rate and cache processed grids.
 *
 * Must be started BEFORE task_sensor_snapshot, which only reads the cache.
 * Takes the same tof_devices_t* argument.
 */
void task_tof_reader(void *pvParameters);

/** @brief Create the ToF cache mutex. Call once before starting either task. */
esp_err_t sensor_task_init(void);

/** Acquire one raw IMU generation and update chip health/recovery state. */
bool sensor_read_imu_sample(imu_sample_t *sample);

/** Latest raw IMU generation for the fusion consumer. */
sample_snapshot_t *sensor_imu_sample_snapshot(void);

/** Called by the fusion task before acquisition starts sending notifications. */
void sensor_task_register_fusion_task(TaskHandle_t task);
