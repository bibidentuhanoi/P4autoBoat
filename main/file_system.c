#include "file_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "math.h"
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

void fs_save_calibration(const CalibrationData* calib) {
    nvs_handle_t my_handle;
    esp_err_t err;

    ESP_LOGI(TAG,"\nSaving calibration to NVS...\n");

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(my_handle, "imu_cal", calib, sizeof(CalibrationData));
    if (err != ESP_OK) {
        ESP_LOGI(TAG,"Failed to write blob to NVS! (%s)\n", esp_err_to_name(err));
    } else {
        err = nvs_commit(my_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG,"Calibration data successfully saved to flash memory!\n");
        }
    }

    nvs_close(my_handle);
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

    for (int i = 0; i < 3; i++) {
        if (isnan(temp_cal.g_bias[i]) || isinf(temp_cal.g_bias[i])) { return false; }
        if (isnan(temp_cal.m_bias[i]) || isinf(temp_cal.m_bias[i])) { return false; }

        if (isnan(temp_cal.m_scale[i]) || isinf(temp_cal.m_scale[i]) ||
           (temp_cal.m_scale[i] <= 0.05f) || (temp_cal.m_scale[i] > 20.0f)) {
            ESP_LOGI(TAG,"CRITICAL: Corrupted Magnetometer scale factor detected! Forcing recalibration.\n");
            return false;
        }
    }

    *calib = temp_cal;

    ESP_LOGI(TAG,"\n--- LOADED CALIBRATION FROM NVS ---\n");
    ESP_LOGI(TAG,".g_bias = {%.2ff, %.2ff, %.2ff}\n", calib->g_bias[0], calib->g_bias[1], calib->g_bias[2]);
    ESP_LOGI(TAG,".m_bias = {%.2ff, %.2ff, %.2ff}\n", calib->m_bias[0], calib->m_bias[1], calib->m_bias[2]);
    ESP_LOGI(TAG,".m_scale = {%.4ff, %.4ff, %.4ff}\n", calib->m_scale[0], calib->m_scale[1], calib->m_scale[2]);
    ESP_LOGI(TAG,".pitch_tare = %.2ff\n", calib->pitch_tare);
    ESP_LOGI(TAG,".roll_tare = %.2ff\n", calib->roll_tare);
    ESP_LOGI(TAG,".heading_tare = %.2ff\n", calib->heading_tare);
    ESP_LOGI(TAG,"-----------------------------------\n");

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
