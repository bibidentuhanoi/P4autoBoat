#include "file_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "math.h"
#include "mag_cal.h"
#include "sd_card.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char* TAG = "FS";
static SemaphoreHandle_t s_sdcard_lock;

#define FS_SDCARD_PATH_MAX 192

static bool fs_sdcard_path(const char *path, char resolved[FS_SDCARD_PATH_MAX])
{
    if (!path || !path[0] || path[0] == '/' || strstr(path, "..")) {
        return false;
    }
    int n = snprintf(resolved, FS_SDCARD_PATH_MAX, "%s/%s", SD_MOUNT_POINT, path);
    return n > 0 && n < FS_SDCARD_PATH_MAX;
}

static bool fs_sdcard_lock(void)
{
    return s_sdcard_lock && sd_card_ready() &&
           xSemaphoreTake(s_sdcard_lock, pdMS_TO_TICKS(1000)) == pdTRUE;
}

static void fs_sdcard_unlock(void)
{
    xSemaphoreGive(s_sdcard_lock);
}

void fs_init(void) {
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_LOGW(TAG, "NVS partition truncated/corrupted. Erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    s_sdcard_lock = xSemaphoreCreateMutex();
    if (!s_sdcard_lock) {
        ESP_LOGE(TAG, "SD-card file mutex allocation failed");
    }
}

bool fs_sdcard_ready(void)
{
    return sd_card_ready() && s_sdcard_lock != NULL;
}

esp_err_t fs_sdcard_read(const char *path, void *buffer, size_t capacity,
                         size_t *out_len)
{
    if (out_len) *out_len = 0;
    if (!buffer || !capacity || !out_len) return ESP_ERR_INVALID_ARG;
    char resolved[FS_SDCARD_PATH_MAX];
    if (!fs_sdcard_path(path, resolved)) return ESP_ERR_INVALID_ARG;
    if (!fs_sdcard_lock()) return ESP_ERR_INVALID_STATE;
    FILE *fp = fopen(resolved, "rb");
    if (!fp) { fs_sdcard_unlock(); return ESP_ERR_NOT_FOUND; }
    size_t n = fread(buffer, 1, capacity, fp);
    bool too_large = n == capacity && fgetc(fp) != EOF;
    bool read_failed = ferror(fp);
    int close_err = fclose(fp);
    fs_sdcard_unlock();
    if (read_failed || close_err != 0) return ESP_FAIL;
    if (too_large) return ESP_ERR_INVALID_SIZE;
    *out_len = n;
    return ESP_OK;
}

esp_err_t fs_sdcard_write(const char *path, const void *data, size_t len)
{
    if (!data && len) return ESP_ERR_INVALID_ARG;
    char resolved[FS_SDCARD_PATH_MAX], temp[FS_SDCARD_PATH_MAX];
    if (!fs_sdcard_path(path, resolved)) return ESP_ERR_INVALID_ARG;

    /* 8.3 FAT (CONFIG_FATFS_LFN_NONE=y, no Long File Name support) allows
     * exactly one dot per name. Simply appending ".tmp" to a path that
     * already has a real extension (e.g. "00000.JPG" -> "00000.JPG.tmp")
     * is structurally invalid, not just too long -- fopen() fails with
     * EINVAL, surfaced here as a generic ESP_FAIL (hw-confirmed). Swap the
     * extension instead of appending one: find the basename's own '.' and
     * replace everything from there with ".tmp". Always fits -- "tmp" is
     * exactly 3 chars, and the original extension had to be <=3 chars too
     * for the caller's own fopen() to have any chance of working. */
    const char *base = strrchr(resolved, '/');
    base = base ? base + 1 : resolved;
    const char *dot = strrchr(base, '.');
    int n = dot
        ? snprintf(temp, sizeof(temp), "%.*s.tmp", (int)(dot - resolved), resolved)
        : snprintf(temp, sizeof(temp), "%s.tmp", resolved);
    if (n <= 0 || n >= (int)sizeof(temp)) return ESP_ERR_INVALID_ARG;
    if (!fs_sdcard_lock()) return ESP_ERR_INVALID_STATE;
    FILE *fp = fopen(temp, "wb");
    if (!fp) { fs_sdcard_unlock(); return ESP_FAIL; }
    size_t written = fwrite(data, 1, len, fp);
    bool failed = written != len || fflush(fp) != 0 || fclose(fp) != 0;
    if (!failed && rename(temp, resolved) != 0) failed = true;
    if (failed) remove(temp);
    fs_sdcard_unlock();
    return failed ? ESP_FAIL : ESP_OK;
}

esp_err_t fs_sdcard_append(const char *path, const void *data, size_t len)
{
    if (!data && len) return ESP_ERR_INVALID_ARG;
    char resolved[FS_SDCARD_PATH_MAX];
    if (!fs_sdcard_path(path, resolved)) return ESP_ERR_INVALID_ARG;
    if (!fs_sdcard_lock()) return ESP_ERR_INVALID_STATE;
    FILE *fp = fopen(resolved, "ab");
    if (!fp) { fs_sdcard_unlock(); return ESP_FAIL; }
    size_t written = fwrite(data, 1, len, fp);
    bool failed = written != len || fflush(fp) != 0 || fclose(fp) != 0;
    fs_sdcard_unlock();
    return failed ? ESP_FAIL : ESP_OK;
}

bool fs_calibration_sane(const CalibrationData* calib) {
    if (!calib || calib->magic_word != CALIB_MAGIC_WORD) return false;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(calib->g_bias[i]) || fabsf(calib->g_bias[i]) > 5000.0f) return false;
    }
    if (!isfinite(calib->pitch_tare) || fabsf(calib->pitch_tare) > 45.0f ||
        !isfinite(calib->roll_tare) || fabsf(calib->roll_tare) > 45.0f) {
        return false;
    }
    mag_cal_2d_t mag;
    memcpy(mag.center, calib->mag_center, sizeof(mag.center));
    memcpy(mag.soft, calib->mag_soft, sizeof(mag.soft));
    mag.radius = calib->mag_radius;
    if (!mag_cal_valid(&mag)) return false;
    /* A PASSed compass calibration always carries a real field strength. */
    if (calib->mag_calibrated > 1U) return false;
    if (calib->mag_calibrated && !(calib->mag_radius >= MAG_CAL_RADIUS_MIN_LSB)) return false;
    if (!isfinite(calib->mag_gyro_dev_deg) || calib->mag_gyro_dev_deg < 0.0f ||
        calib->mag_gyro_dev_deg > 180.0f) return false;
    if (!isfinite(calib->mag_tolerance_deg) || calib->mag_tolerance_deg < 0.0f ||
        calib->mag_tolerance_deg > MAG_CAL_GYRO_DEV_MAX) return false;
    if (calib->mag_calibrated && calib->mag_gyro_dev_deg > calib->mag_tolerance_deg) return false;
    return true;
}

bool fs_save_calibration(const CalibrationData* calib) {
    nvs_handle_t my_handle;
    esp_err_t err;

    if (!fs_calibration_sane(calib)) {
        ESP_LOGE(TAG, "Refusing to save a calibration that fails its own sanity check");
        return false;
    }

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Calibration save: cannot open NVS (%s)", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_blob(my_handle, "imu_cal", calib, sizeof(CalibrationData));
    if (err == ESP_OK) err = nvs_commit(my_handle);
    nvs_close(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Calibration save failed (%s)", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Calibration saved to flash");
    return true;
}

bool fs_load_calibration(CalibrationData* calib) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG,"NVS empty or uninitialized. (Normal on first boot)\n");
        return false;
    }

    size_t required_size = 0;
    err = nvs_get_blob(my_handle, "imu_cal", NULL, &required_size);
    if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND)) {
        nvs_close(my_handle);
        return false;
    }

    if (required_size != sizeof(CalibrationData)) {
        ESP_LOGI(TAG,"NVS Data size mismatch! Firmware updated? Forcing recalibration.\n");
        nvs_close(my_handle);
        return false;
    }

    CalibrationData temp_cal;
    err = nvs_get_blob(my_handle, "imu_cal", &temp_cal, &required_size);
    nvs_close(my_handle);

    if (err != ESP_OK) {
        return false;
    }

    if (temp_cal.magic_word != CALIB_MAGIC_WORD) {
        ESP_LOGI(TAG,"NVS Data format corrupted or outdated. Forcing recalibration.\n");
        return false;
    }

    if (!fs_calibration_sane(&temp_cal)) {
        ESP_LOGW(TAG, "Stored calibration fails its sanity check. Forcing recalibration.");
        return false;
    }

    *calib = temp_cal;

    ESP_LOGI(TAG, "Loaded calibration: gyro bias {%.1f, %.1f, %.1f}  level p=%.2f r=%.2f",
             calib->g_bias[0], calib->g_bias[1], calib->g_bias[2],
             calib->pitch_tare, calib->roll_tare);
    ESP_LOGI(TAG, "  compass %s (gyro %.1f deg, limit %.0f): centre {%.0f, %.0f}  "
                  "soft {%.4f %.4f; %.4f %.4f}  radius %.0f",
             calib->mag_calibrated ? "calibrated" : "NOT calibrated",
             calib->mag_gyro_dev_deg, calib->mag_tolerance_deg,
             calib->mag_center[0], calib->mag_center[1],
             calib->mag_soft[0], calib->mag_soft[1], calib->mag_soft[2], calib->mag_soft[3],
             calib->mag_radius);

    return true;
}

void fs_save_tof_xtalk(const char* key, const uint8_t* data, size_t len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed for xtalk save: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(h, key, data, len);
    if (err == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "Xtalk cal [%s] saved (%d bytes)", key, (int)len);
    } else {
        ESP_LOGE(TAG, "Xtalk cal [%s] save failed: %s", key, esp_err_to_name(err));
    }
    nvs_close(h);
}

bool fs_load_tof_xtalk(const char* key, uint8_t* data, size_t len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &h);
    if (err != ESP_OK) return false;

    size_t stored_len = 0;
    err = nvs_get_blob(h, key, NULL, &stored_len);
    if (err != ESP_OK || stored_len != len) {
        nvs_close(h);
        return false;
    }
    err = nvs_get_blob(h, key, data, &stored_len);
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Xtalk cal [%s] loaded (%d bytes)", key, (int)len);
        return true;
    }
    return false;
}

static bool esc_trim_blob_valid(const EscTrimNvsBlob *blob)
{
    if (!blob || blob->magic_word != ESC_TRIM_NVS_MAGIC ||
        blob->count > ESC_TRIM_MAX_POINTS) return false;
    for (uint8_t i = 0; i < blob->count; ++i) {
        if (!isfinite(blob->points[i].throttle_frac) ||
            !isfinite(blob->points[i].trim_diff) ||
            blob->points[i].throttle_frac < 0.0f ||
            blob->points[i].throttle_frac > 1.0f ||
            blob->points[i].trim_diff < -1.0f ||
            blob->points[i].trim_diff > 1.0f) return false;
    }
    return true;
}

bool fs_save_esc_trim(const EscTrimNvsBlob *blob)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed for esc_trim save: %s", esp_err_to_name(err));
        return false;
    }
    if (!esc_trim_blob_valid(blob)) {
        nvs_close(h);
        ESP_LOGE(TAG, "ESC trim save rejected invalid table");
        return false;
    }
    err = nvs_set_blob(h, "esc_trim", blob, sizeof(*blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
        if (err == ESP_OK)
            ESP_LOGI(TAG, "ESC trim table saved (%u points)", (unsigned)blob->count);
        else
            ESP_LOGE(TAG, "ESC trim commit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGE(TAG, "ESC trim save failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);
    return err == ESP_OK;
}

bool fs_load_esc_trim(EscTrimNvsBlob *blob)
{
    memset(blob, 0, sizeof(*blob));   /* safe default if anything below fails */

    nvs_handle_t h;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &h);
    if (err != ESP_OK) return false;

    EscTrimNvsBlob temp;
    size_t len = sizeof(temp);
    err = nvs_get_blob(h, "esc_trim", &temp, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(temp) || !esc_trim_blob_valid(&temp)) {
        ESP_LOGI(TAG, "No valid ESC trim table in NVS -- starting with none");
        return false;
    }
    *blob = temp;
    ESP_LOGI(TAG, "ESC trim table loaded (%u points)", (unsigned)blob->count);
    return true;
}
