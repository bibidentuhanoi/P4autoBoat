#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

#include "common.h"
#include <stdbool.h>

void fs_init(void);
void fs_save_calibration(const CalibrationData* calib);
bool fs_load_calibration(CalibrationData* calib);

void fs_save_tof_xtalk(const char* key, const uint8_t* data, size_t len);
bool fs_load_tof_xtalk(const char* key, uint8_t* data, size_t len);

#endif // FILE_SYSTEM_H
