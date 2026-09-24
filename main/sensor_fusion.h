#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "imu_sample.h"

// Structures
typedef struct {
    float pitch;
    float roll;
    float heading;
    bool heading_valid; /* fused yaw has been initialized from accepted mag data */
    float yaw_rate;   /* deg/s about vertical axis, from gyro-Z (mag-independent) */
    uint32_t sequence; /* published sample sequence; use to detect fusion stalls */
    uint64_t captured_us; /* IMU sample timestamp this result was derived from */
} FusionResult;

/* ICM20948 at +/-250 dps; positive installed gyro Z = the boat turning LEFT. */
#define FUSION_GYRO_COUNTS_PER_DPS 131.0f
#define FUSION_YAW_GYRO_SIGN       (+1.0f)

/* Takes a copy; the caller's struct is not referenced afterwards. */
void fusion_init(const CalibrationData* calib_data);
/* Hand a new calibration to the running fusion task.  Returns false if the
 * previous one has not been picked up yet -- retry after one sample. */
bool fusion_set_calibration(const CalibrationData *calib_data);
bool fusion_calibration_pending(void);
/* Live |corrected compass| / field strength at calibration, ~1 s smoothed.
 * 1.0 = same field as when calibrated; 0 = not calibrated / no data yet. */
float fusion_get_field_ratio(void);
void fusion_update_sample(const imu_sample_t *sample);
void fusion_get_result(FusionResult* res);
void task_imu_fusion(void *pvParameters);

#endif // SENSOR_FUSION_H
