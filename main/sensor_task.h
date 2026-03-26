#pragma once
#include "drivers/tof_driver.h"

/**
 * @brief FreeRTOS task: samples IMU + both ToFs at ~5Hz,
 *        encodes as protobuf, publishes via pipeline.
 *        Pass tof_devices_t* as pvParameters.
 */
void task_sensor_snapshot(void *pvParameters);
