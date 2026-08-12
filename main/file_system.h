#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

#include "common.h"
#include "esc_trim.h"
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

void fs_init(void);
void fs_save_calibration(const CalibrationData* calib);
bool fs_load_calibration(CalibrationData* calib);

void fs_save_tof_xtalk(const char* key, const uint8_t* data, size_t len);
bool fs_load_tof_xtalk(const char* key, uint8_t* data, size_t len);

void fs_save_esc_trim(const EscTrimNvsBlob *blob);
bool fs_load_esc_trim(EscTrimNvsBlob *blob);   /* true = valid blob loaded; false = defaulted to empty (blob->count = 0) */

/* SD-card file API. Paths are relative to /sdcard and may not contain '..'. */
bool fs_sdcard_ready(void);
esp_err_t fs_sdcard_read(const char *path, void *buffer, size_t capacity,
                         size_t *out_len);
esp_err_t fs_sdcard_write(const char *path, const void *data, size_t len);
esp_err_t fs_sdcard_append(const char *path, const void *data, size_t len);

#endif // FILE_SYSTEM_H
