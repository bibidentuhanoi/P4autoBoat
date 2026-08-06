#pragma once
#include "drivers/tof_driver.h"

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
