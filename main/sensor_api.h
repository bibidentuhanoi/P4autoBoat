#pragma once
#include "esp_err.h"
#include "esp_http_server.h"
#include "drivers/tof_driver.h"

/**
 * @brief Register /api/imu and /api/snapshot HTTP handlers on an existing server.
 */
esp_err_t sensor_api_register(httpd_handle_t server, tof_devices_t *tof_devs);

/**
 * @brief FreeRTOS task: samples IMU + both ToFs at ~5Hz, writes to mutex-protected snapshot.
 *        Pass tof_devices_t* as pvParameters.
 */
void task_sensor_snapshot(void *pvParameters);
