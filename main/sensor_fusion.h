#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "common.h"
#include "imu_sample.h"

// Structures
typedef struct {
    float pitch;
    float roll;
    float heading;
} FusionResult;

void fusion_init(CalibrationData* calib_data);
void fusion_update_sample(const imu_sample_t *sample);
void fusion_get_result(FusionResult* res);
void task_imu_fusion(void *pvParameters);

#endif // SENSOR_FUSION_H
