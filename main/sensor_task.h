#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drivers/tof_driver.h"
#include "imu_sample.h"
#include "sample_snapshot.h"
#include "sensor_schedule.h"
#include "proto/boat.pb.h"

/** @brief Publish the existing 20 Hz cached sensor snapshot. */
void task_sensor_snapshot(void *pvParameters);

/**
 * @brief Acquire IMU at 50 Hz. Does not touch ToF -- see task_tof_read().
 */
void task_sensor_bus(void *pvParameters);

/**
 * @brief Acquire ToF-A/ToF-B on their own ~10 Hz-each schedule, independent
 *        of SensorBus's IMU timing. Shares the I2C bus with SensorBus via
 *        ESP-IDF's own per-transaction bus lock (no application mutex around
 *        the read itself). Pass tof_devices_t* as pvParameters.
 */
void task_tof_read(void *pvParameters);

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

/**
 * @brief Load the latest coherent ToF grid (torn frames already excluded
 *        upstream by task_tof_read()) for the given sensor.
 * @param generation  optional out: the verified generation number this grid
 *                     came from (NULL if not needed).
 * @param age_us      optional out: microseconds since this generation was
 *                     published into the cache (NULL if not needed).
 * @return false if nothing has been published yet or the cache has gone
 *         stale (TOF_STALE_US) -- out params are untouched in that case.
 */
bool sensor_tof_cache_load(sensor_tof_id_t which, boat_ToFGrid *out,
                           uint32_t *generation, int64_t *age_us);
