#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "common.h"
#include "drivers/imu_driver.h"

void perform_calibration_routine(CalibrationData* output_calib);

#endif // CALIBRATION_H
