#include "file_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "math.h"

static const char* TAG = "FS";

void fs_init(void) {
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_LOGW(TAG, "NVS partition truncated/corrupted. Erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
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
