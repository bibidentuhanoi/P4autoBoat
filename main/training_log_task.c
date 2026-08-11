#include "training_log_task.h"
#include "drivers/camera_driver.h"
#include "drivers/gps_driver.h"
#include "drivers/sd_card.h"
#include "file_system.h"
#include "sensor_fusion.h"
#include "sensor_task.h"
#include "runtime_metrics.h"
#include "runtime_startup.h"
#include "runtime_task.h"
#include "proto/boat.pb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>

static const char *TAG = "TLOG";

/* Generous upper bounds, not tight ones -- a capture that doesn't fit just
 * fails cleanly (ESP_ERR_INVALID_SIZE from camera_capture_copy(), or the
 * bounded tlog_writer_t below silently truncates rather than overflowing).
 * Real-world JPEG size at full quality hasn't been measured on hardware
 * yet; 200KB is comfortably above the ~50KB the reduced-quality MJPEG
 * stream produces (camera_stream.c), worth revisiting once observed. */
#define TLOG_JPEG_CAPACITY     (200 * 1024)
#define TLOG_SIDECAR_CAPACITY  (32 * 1024)

/* Same diagnostic pattern as SENSOR_BUS_SLOW_OP_US / TOF_READ_ANOMALY_US
 * this session: SD and ESP-Hosted/C6 share one SDMMC controller (one
 * module-global transaction mutex in ESP-IDF's sdmmc_transaction.c), so a
 * slow SD write can delay ESP-NOW control traffic. This threshold is what
 * makes that visible in the log instead of just being a mystery latency
 * spike in Control's own RTM stats. */
#define TLOG_WRITE_SLOW_US     50000U

static TaskHandle_t s_training_log_task = NULL;
static uint32_t     s_session_id;
static uint32_t     s_frame_index;
static uint8_t     *s_jpeg_buf;      /* PSRAM, allocated once in training_log_init() */
static char        *s_sidecar_buf;   /* PSRAM, allocated once in training_log_init() */

volatile bool g_training_log_sd_active = false;
volatile bool g_training_log_camera_active = false;

/* ---- NVS session counter ----
 * Own namespace ("training_log"), not "storage" (used by fs_save_calibration)
 * -- avoids any key collision with unrelated calibration data. */

static uint32_t training_log_next_session_id(void)
{
    nvs_handle_t h;
    uint32_t id = 1;

    esp_err_t err = nvs_open("training_log", NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed (%s) -- session id will not persist across boots",
                 esp_err_to_name(err));
        return id;
    }

    uint32_t stored = 0;
    if (nvs_get_u32(h, "session", &stored) == ESP_OK) {
        id = stored + 1;
    }

    esp_err_t set_err = nvs_set_u32(h, "session", id);
    if (set_err == ESP_OK) {
        nvs_commit(h);
    } else {
        ESP_LOGW(TAG, "NVS session counter write failed (%s) -- next boot may reuse this id",
                 esp_err_to_name(set_err));
    }
    nvs_close(h);
    return id;
}

/* ---- Bounded string builder for the sidecar ----
 * Chains snprintf calls into one buffer without ever writing past capacity.
 * A field that doesn't fit is silently dropped (used < capacity always
 * holds), not a buffer overflow -- correct behavior, just incomplete output,
 * which is far preferable to corrupting adjacent heap for a rare oversized
 * capture (e.g. an unusually large nb_target run). */
typedef struct {
    char  *buf;
    size_t capacity;
    size_t used;
} tlog_writer_t;

static void tlog_writer_init(tlog_writer_t *w, char *buf, size_t capacity)
{
    w->buf = buf;
    w->capacity = capacity;
    w->used = 0;
    if (capacity) buf[0] = '\0';
}

static void tlog_appendf(tlog_writer_t *w, const char *fmt, ...)
{
    if (w->used >= w->capacity) return;
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(w->buf + w->used, w->capacity - w->used, fmt, args);
    va_end(args);
    if (n > 0) {
        size_t remaining = w->capacity - w->used;
        w->used += ((size_t)n < remaining) ? (size_t)n : remaining;
    }
}

static void tlog_append_i32_csv(tlog_writer_t *w, const char *label,
                                 const int32_t *values, pb_size_t count)
{
    tlog_appendf(w, "%s=", label);
    for (pb_size_t i = 0; i < count; i++) {
        tlog_appendf(w, i ? ",%ld" : "%ld", (long)values[i]);
    }
    tlog_appendf(w, "\n");
}

static void tlog_append_u32_csv(tlog_writer_t *w, const char *label,
                                 const uint32_t *values, pb_size_t count)
{
    tlog_appendf(w, "%s=", label);
    for (pb_size_t i = 0; i < count; i++) {
        tlog_appendf(w, i ? ",%lu" : "%lu", (unsigned long)values[i]);
    }
    tlog_appendf(w, "\n");
}

/* 8.3 short-filename FAT (CONFIG_FATFS_LFN_NONE=y -- no Long File Name
 * support compiled in): names are capped at 8 characters, extensions at 3.
 * "session_%04lu" (12 chars) and "frame_%05lu" (11 chars) both exceed that
 * -- mkdir()/fopen() fail with EINVAL, hw-confirmed. Bare zero-padded
 * numbers fit (5 digits, well under 8); the sidecar's own session=/frame=
 * fields carry the human-readable labels, the path doesn't need to.
 * Centralized here rather than rebuilt at each call site, so the stat()
 * collision-check, mkdir(), and the actual fs_sdcard_write() paths can't
 * drift out of sync with each other again. */
static void training_log_session_dir_abs(char *out, size_t cap, uint32_t session_id)
{
    snprintf(out, cap, "%s/training/%05lu", SD_MOUNT_POINT, (unsigned long)session_id);
}

static void training_log_frame_path(char *out, size_t cap, uint32_t session_id,
                                     uint32_t frame, const char *ext)
{
    /* Real extension is fine here: fs_sdcard_write() (file_system.c) now
     * builds its own temp name by swapping the extension for ".tmp" rather
     * than appending onto it, so "00000.jpg" -> temp "00000.tmp" -> renamed
     * back to "00000.jpg" -- one dot at a time, always 8.3-valid. */
    snprintf(out, cap, "training/%05lu/%05lu.%s",
             (unsigned long)session_id, (unsigned long)frame, ext);
}

/* Idempotent -- mkdir on an existing dir returns EEXIST, treated as success.
 * Called every capture rather than cached behind a "ready" flag: SD has no
 * card-detect wired (sd_card.c), so state doesn't change after boot, but
 * this way there's no stale-flag risk if that ever changes, and mkdir on an
 * SD card is cheap next to the JPEG/sidecar writes that follow it. */

/* NVS survives reboots but not an nvs_flash_erase() recovery cycle (fs_init()
 * does exactly that on a corrupted NVS partition). If that ever happens
 * while an SD card with old sessions is still inserted, the counter would
 * restart at 1 and collide with a real, already-populated session dir
 * from this card's previous life. Cheap check, not a directory scan: does
 * this session's first frame already exist? If so, it was already used for
 * real captures -- advance past it. Only meaningful once SD is actually
 * available, so it runs here (first successful call), not at init time. */
static bool training_log_session_dir_taken(uint32_t session_id)
{
    char rel[64], path[96];
    training_log_frame_path(rel, sizeof(rel), session_id, 0, "jpg");
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, rel);
    struct stat st;
    return stat(path, &st) == 0;
}

static bool training_log_ensure_session_dir(void)
{
    static bool s_session_confirmed = false;
    char path[80];

    snprintf(path, sizeof(path), "%s/training", SD_MOUNT_POINT);
    if (mkdir(path, 0777) != 0 && errno != EEXIST) {
        ESP_LOGW(TAG, "mkdir %s failed: %s", path, strerror(errno));
        return false;
    }

    if (!s_session_confirmed) {
        uint32_t original = s_session_id;
        while (training_log_session_dir_taken(s_session_id)) {
            s_session_id++;
        }
        if (s_session_id != original) {
            ESP_LOGW(TAG, "NVS session id %lu already had captures on this card -- using %lu instead",
                     (unsigned long)original, (unsigned long)s_session_id);
        }
        s_session_confirmed = true;
    }

    training_log_session_dir_abs(path, sizeof(path), s_session_id);
    if (mkdir(path, 0777) != 0 && errno != EEXIST) {
        ESP_LOGW(TAG, "mkdir %s failed: %s", path, strerror(errno));
        return false;
    }
    return true;
}

static void training_log_write_capture(void)
{
    int64_t t0 = esp_timer_get_time();

    if (!fs_sdcard_ready()) {
        /* Soft-optional by design (main.c boots SD last, non-fatal if
         * absent) -- a capture with no card mounted is an expected miss,
         * not a fault. */
        ESP_LOGW(TAG, "SD not ready -- capture dropped");
        runtime_metrics_count(RUNTIME_TASK_TRAINING_LOG, RUNTIME_EVENT_SENSOR_SKIP);
        return;
    }
    if (!training_log_ensure_session_dir()) {
        runtime_metrics_count(RUNTIME_TASK_TRAINING_LOG, RUNTIME_EVENT_SENSOR_ERROR);
        return;
    }

    size_t   jpeg_len = 0;
    uint32_t width = 0, height = 0;
    g_training_log_camera_active = true;
    esp_err_t cap_ret = camera_capture_copy(s_jpeg_buf, TLOG_JPEG_CAPACITY, &jpeg_len,
                                            &width, &height);
    g_training_log_camera_active = false;
    if (cap_ret != ESP_OK) {
        ESP_LOGW(TAG, "Camera capture failed: %s", esp_err_to_name(cap_ret));
        runtime_metrics_count(RUNTIME_TASK_TRAINING_LOG, RUNTIME_EVENT_SENSOR_ERROR);
        return;
    }

    /* Camera is already released at this point (camera_capture_copy() does
     * capture+copy+release atomically) -- none of what follows holds it. */
    FusionResult imu;
    fusion_get_result(&imu);

    gps_fix_t gps;
    bool have_gps = (gps_driver_get_fix(&gps) == ESP_OK) && gps.last_update_us != 0;
    int64_t gps_age_ms = have_gps ? (esp_timer_get_time() - gps.last_update_us) / 1000 : -1;

    static boat_ToFGrid tof_a, tof_b;   /* static: ~3.3KB each, too big for
                                          * this task's 8KB stack as locals
                                          * (same reasoning as pipeline.c's
                                          * s_msg/s_rx_msg). */
    uint32_t gen_a = 0, gen_b = 0;
    int64_t  age_a_us = 0, age_b_us = 0;
    bool have_a = sensor_tof_cache_load(SENSOR_TOF_A, &tof_a, &gen_a, &age_a_us);
    bool have_b = sensor_tof_cache_load(SENSOR_TOF_B, &tof_b, &gen_b, &age_b_us);

    uint32_t frame = s_frame_index++;
    char jpeg_path[64], sidecar_path[64];
    training_log_frame_path(jpeg_path, sizeof(jpeg_path), s_session_id, frame, "jpg");
    training_log_frame_path(sidecar_path, sizeof(sidecar_path), s_session_id, frame, "txt");

    int64_t write_t0 = esp_timer_get_time();
    g_training_log_sd_active = true;
    esp_err_t jpeg_ret = fs_sdcard_write(jpeg_path, s_jpeg_buf, jpeg_len);
    g_training_log_sd_active = false;
    int64_t jpeg_write_us = esp_timer_get_time() - write_t0;
    if (jpeg_write_us > TLOG_WRITE_SLOW_US) {
        ESP_LOGW(TAG, "TrainingLog: JPEG write took %lldus (%s) -- SD/SDIO contention?",
                 (long long)jpeg_write_us, esp_err_to_name(jpeg_ret));
    }
    if (jpeg_ret != ESP_OK) {
        ESP_LOGW(TAG, "JPEG write failed: %s", esp_err_to_name(jpeg_ret));
        runtime_metrics_count(RUNTIME_TASK_TRAINING_LOG, RUNTIME_EVENT_SENSOR_ERROR);
        return;
    }

    tlog_writer_t w;
    tlog_writer_init(&w, s_sidecar_buf, TLOG_SIDECAR_CAPACITY);
    tlog_appendf(&w, "capture_us=%llu\n", (unsigned long long)t0);
    tlog_appendf(&w, "session=%lu\n", (unsigned long)s_session_id);
    tlog_appendf(&w, "frame=%lu\n", (unsigned long)frame);
    tlog_appendf(&w, "jpeg_bytes=%lu\n", (unsigned long)jpeg_len);
    tlog_appendf(&w, "jpeg_width=%lu\n", (unsigned long)width);
    tlog_appendf(&w, "jpeg_height=%lu\n", (unsigned long)height);
    tlog_appendf(&w, "imu_pitch=%.3f\n", (double)imu.pitch);
    tlog_appendf(&w, "imu_roll=%.3f\n", (double)imu.roll);
    tlog_appendf(&w, "imu_heading=%.3f\n", (double)imu.heading);
    tlog_appendf(&w, "gps_valid=%d\n", (have_gps && gps.valid) ? 1 : 0);
    if (have_gps) {
        tlog_appendf(&w, "gps_lat=%.7f\n", gps.latitude);
        tlog_appendf(&w, "gps_lon=%.7f\n", gps.longitude);
        tlog_appendf(&w, "gps_age_ms=%lld\n", (long long)gps_age_ms);
    } else {
        tlog_appendf(&w, "gps_age_ms=unavailable\n");
    }
    tlog_appendf(&w, "tof_a_valid=%d\n", have_a ? 1 : 0);
    if (have_a) {
        tlog_appendf(&w, "tof_a_generation=%lu\n", (unsigned long)gen_a);
        tlog_appendf(&w, "tof_a_age_ms=%lld\n", (long long)(age_a_us / 1000));
        tlog_append_i32_csv(&w, "tof_a_distances", tof_a.distances, tof_a.distances_count);
        tlog_append_u32_csv(&w, "tof_a_sigma", tof_a.sigma, tof_a.sigma_count);
        tlog_append_u32_csv(&w, "tof_a_target_status", tof_a.target_status, tof_a.target_status_count);
        tlog_append_u32_csv(&w, "tof_a_nb_target", tof_a.nb_target_detected, tof_a.nb_target_detected_count);
    }
    tlog_appendf(&w, "tof_b_valid=%d\n", have_b ? 1 : 0);
    if (have_b) {
        tlog_appendf(&w, "tof_b_generation=%lu\n", (unsigned long)gen_b);
        tlog_appendf(&w, "tof_b_age_ms=%lld\n", (long long)(age_b_us / 1000));
        tlog_append_i32_csv(&w, "tof_b_distances", tof_b.distances, tof_b.distances_count);
        tlog_append_u32_csv(&w, "tof_b_sigma", tof_b.sigma, tof_b.sigma_count);
        tlog_append_u32_csv(&w, "tof_b_target_status", tof_b.target_status, tof_b.target_status_count);
        tlog_append_u32_csv(&w, "tof_b_nb_target", tof_b.nb_target_detected, tof_b.nb_target_detected_count);
    }

    write_t0 = esp_timer_get_time();
    g_training_log_sd_active = true;
    esp_err_t side_ret = fs_sdcard_write(sidecar_path, s_sidecar_buf, w.used);
    g_training_log_sd_active = false;
    int64_t side_write_us = esp_timer_get_time() - write_t0;
    if (side_write_us > TLOG_WRITE_SLOW_US) {
        ESP_LOGW(TAG, "TrainingLog: sidecar write took %lldus (%s) -- SD/SDIO contention?",
                 (long long)side_write_us, esp_err_to_name(side_ret));
    }
    if (side_ret != ESP_OK) {
        ESP_LOGW(TAG, "Sidecar write failed: %s", esp_err_to_name(side_ret));
        runtime_metrics_count(RUNTIME_TASK_TRAINING_LOG, RUNTIME_EVENT_SENSOR_ERROR);
        return;
    }

    ESP_LOGI(TAG, "Captured session %lu frame %lu: %lu byte JPEG (%lux%lu) tof_a=%d tof_b=%d, %lldms total",
             (unsigned long)s_session_id, (unsigned long)frame, (unsigned long)jpeg_len,
             (unsigned long)width, (unsigned long)height, (int)have_a, (int)have_b,
             (long long)((esp_timer_get_time() - t0) / 1000));
}

static void training_log_task_fn(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "TrainingLog task started (session %lu, trigger via WS/ESP-NOW)",
             (unsigned long)s_session_id);

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t started = (uint64_t)esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_TRAINING_LOG, started, started);
        training_log_write_capture();
        runtime_metrics_cycle_end(RUNTIME_TASK_TRAINING_LOG, (uint64_t)esp_timer_get_time());
    }
}

esp_err_t training_log_init(void)
{
    s_jpeg_buf    = heap_caps_malloc(TLOG_JPEG_CAPACITY, MALLOC_CAP_SPIRAM);
    s_sidecar_buf = heap_caps_malloc(TLOG_SIDECAR_CAPACITY, MALLOC_CAP_SPIRAM);
    if (!s_jpeg_buf || !s_sidecar_buf) {
        ESP_LOGE(TAG, "PSRAM allocation failed (jpeg=%p sidecar=%p)",
                 (void *)s_jpeg_buf, (void *)s_sidecar_buf);
        return runtime_startup_handle_task_failure(RUNTIME_TASK_TRAINING_LOG, ESP_ERR_NO_MEM);
    }

    s_session_id = training_log_next_session_id();
    s_frame_index = 0;

    esp_err_t task_error = runtime_task_create(RUNTIME_TASK_TRAINING_LOG, training_log_task_fn,
                                               NULL, &s_training_log_task);
    if (task_error != ESP_OK) {
        return runtime_startup_handle_task_failure(RUNTIME_TASK_TRAINING_LOG, task_error);
    }
    return ESP_OK;
}

void training_log_trigger(void)
{
    if (s_training_log_task) {
        ESP_LOGI(TAG, "Capture triggered");
        xTaskNotifyGive(s_training_log_task);
    }
}
