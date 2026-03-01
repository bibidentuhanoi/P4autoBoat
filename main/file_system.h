#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

#include "common.h"
#include <stdbool.h>

void fs_init(void);
void fs_save_calibration(const CalibrationData* calib);
bool fs_load_calibration(CalibrationData* calib);

#endif // FILE_SYSTEM_H
