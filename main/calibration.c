#include "calibration.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "drivers/status_led.h"
#include "file_system.h"
#include "mag_cal.h"
#include "pipeline.h"
#include "runtime_task.h"
#include "sensor_fusion.h"
#include "sensor_task.h"

static const char *TAG = "CALIB";

/* Step 1: hold still.  Samples arrive at the 50 Hz IMU rate. */
#define STILL_WINDOW          50        /* ~1 s per stillness decision */
#define STILL_COLLECT         150       /* ~3 s of still samples averaged */
#define STILL_ACCEL_STD_MAX   90.0f     /* counts at +/-2 g (~5.5 mg) */
#define STILL_GYRO_STD_MAX    65.0f     /* counts at 131/dps (0.5 deg/s) */
#define STILL_TIMEOUT_US      (60LL * 1000000LL)
#define GYRO_BIAS_SANITY_MAX  2000.0f   /* counts; larger = moved, or a bad IMU */

/* Step 2: spin. */
#define SPIN_TIMEOUT_US       (180LL * 1000000LL)
#define SPIN_CAPACITY         4000      /* samples kept; thinned beyond that */

#define POLL_MS               10
#define PUBLISH_RUN_US        200000LL  /* 5 Hz while calibrating */
#define PUBLISH_IDLE_US       1000000LL /* 1 Hz afterwards */

static CalibrationData *s_live;
static bool s_run_now;
static atomic_bool s_active;
static boat_CompassCalStatus s_status;   /* owned by the CompassCal task */

bool compass_cal_active(void)
{
    return atomic_load_explicit(&s_active, memory_order_acquire);
}

static void publish(void)
{
    s_status.calibrated = s_live && s_live->mag_calibrated;
    s_status.field_ratio = fusion_get_field_ratio();
    FusionResult r;
    fusion_get_result(&r);
    s_status.heading_deg = r.heading_valid ? r.heading : -1.0f;
    /* WiFi only: over ESP-NOW an unknown message would go out labelled as
     * sensor telemetry, on a link that is already short of bandwidth. */
    if (!g_field_mode) pipeline_publish_compass_cal_status(&s_status);
}

static void enter(compass_cal_state_t state)
{
    s_status.state = state;
    s_status.elapsed_s = 0.0f;
}

static bool next_sample(uint32_t *last_seq, imu_sample_t *out)
{
    sample_snapshot_t *snap = sensor_imu_sample_snapshot();
    if (!snap || !sample_snapshot_read(snap, out) || out->sequence == *last_seq) {
        return false;
    }
    *last_seq = out->sequence;
    return true;
}

static void fail(compass_cal_reason_t reason)
{
    enter(COMPASS_CAL_FAIL);
    s_status.reason = reason;
    status_led_set(STATUS_LED_CAL_DONE_FAIL);
    ESP_LOGW(TAG, "Calibration FAILED (reason %d) -- previous calibration kept",
             (int)reason);
}

/* ---- Step 1 ---- */

static bool hold_still(float gyro_bias[3], float accel_mean[3])
{
    enter(COMPASS_CAL_STILL);
    status_led_set(STATUS_LED_CAL_STILL);
    ESP_LOGI(TAG, "Step 1: HOLD STILL -- set the boat level and do not touch it");

    mag_still_t still;
    mag_still_init(&still, STILL_WINDOW, STILL_COLLECT,
                   STILL_ACCEL_STD_MAX, STILL_GYRO_STD_MAX);
    uint32_t seq = 0;
    int valid = 0;
    int64_t t0 = esp_timer_get_time(), last_pub = 0;
    for (;;) {
        imu_sample_t s;
        while (next_sample(&seq, &s)) {
            if (!s.accel_gyro_valid) continue;
            valid++;
            int16_t a[3] = {s.ax, s.ay, s.az};
            int16_t g[3] = {s.gx, s.gy, s.gz};
            mag_still_add(&still, a, g);
        }
        int64_t now = esp_timer_get_time();
        s_status.still_progress = mag_still_progress(&still);
        s_status.elapsed_s = (float)(now - t0) / 1e6f;
        if (still.state == MAG_STILL_DONE) break;
        if (now - t0 > STILL_TIMEOUT_US) {
            fail(valid ? COMPASS_CAL_REASON_STILL_TIMEOUT : COMPASS_CAL_REASON_NO_IMU);
            return false;
        }
        if (now - last_pub >= PUBLISH_RUN_US) { publish(); last_pub = now; }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }

    mag_still_result(&still, gyro_bias, accel_mean);
    float mag = sqrtf(gyro_bias[0] * gyro_bias[0] + gyro_bias[1] * gyro_bias[1] +
                      gyro_bias[2] * gyro_bias[2]);
    ESP_LOGI(TAG, "  still: gyro bias {%.1f, %.1f, %.1f} counts (|%.1f|)",
             gyro_bias[0], gyro_bias[1], gyro_bias[2], mag);
    if (!(mag < GYRO_BIAS_SANITY_MAX)) {
        fail(COMPASS_CAL_REASON_GYRO_BIAS);
        return false;
    }
    return true;
}

/* ---- Step 2 + 3 ---- */

static bool spin_and_fit(const float gyro_bias[3], mag_fit_t *fit)
{
    size_t bytes = SPIN_CAPACITY * sizeof(float);
    float *bx = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    float *by = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    float *bt = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!bx || !by || !bt) {
        heap_caps_free(bx); heap_caps_free(by); heap_caps_free(bt);
        fail(COMPASS_CAL_REASON_NO_MEMORY);
        return false;
    }

    enter(COMPASS_CAL_SPIN);
    s_status.still_progress = 1.0f;
    status_led_set(STATUS_LED_CAL_MOVE);
    ESP_LOGI(TAG, "Step 2: SPIN -- turn the boat slowly, flat, about two full circles");

    mag_circle_t circle;
    mag_circle_init(&circle, bx, by, bt, SPIN_CAPACITY);
    uint32_t seq = 0;
    uint64_t last_us = 0;
    int64_t t0 = esp_timer_get_time(), last_pub = 0;
    bool complete = false;
    for (;;) {
        imu_sample_t s;
        while (next_sample(&seq, &s)) {
            if (!s.accel_gyro_valid) continue;
            float dt = (last_us && s.captured_us > last_us)
                     ? (float)(s.captured_us - last_us) / 1e6f : 0.0f;
            last_us = s.captured_us;
            float rate = FUSION_YAW_GYRO_SIGN * ((float)s.gz - gyro_bias[2]) /
                         FUSION_GYRO_COUNTS_PER_DPS;
            mag_circle_add(&circle,
                           s.mag_valid ? (float)s.mx : NAN,
                           s.mag_valid ? (float)s.my : NAN, rate, dt);
        }
        int64_t now = esp_timer_get_time();
        s_status.turn_deg = circle.turn_deg;
        s_status.elapsed_s = (float)(now - t0) / 1e6f;
        if (now - last_pub >= PUBLISH_RUN_US) {
            s_status.coverage_mask = mag_circle_live_mask(&circle);
            publish();
            last_pub = now;
        }
        if (mag_circle_complete(&circle)) { complete = true; break; }
        if (now - t0 > SPIN_TIMEOUT_US) break;
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
    s_status.coverage_mask = mag_circle_live_mask(&circle);

    if (!complete) {
        ESP_LOGW(TAG, "  spin: not completed (%d samples, %.0f deg turned, %d/32 slices)",
                 circle.n, circle.turn_deg, __builtin_popcount(s_status.coverage_mask));
        heap_caps_free(bx); heap_caps_free(by); heap_caps_free(bt);
        fail(COMPASS_CAL_REASON_SPIN_TIMEOUT);
        return false;
    }

    enter(COMPASS_CAL_CHECKING);
    publish();
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, circle.n, fit);
    heap_caps_free(bx); heap_caps_free(by); heap_caps_free(bt);

    s_status.axis_ratio = fit->axis_ratio;
    s_status.fit_rms = fit->rms;
    s_status.gyro_scale = fit->gyro_scale;
    s_status.gyro_dev_deg = fit->gyro_dev_deg;
    s_status.radius = fit->cal.radius;
    s_status.coverage_mask = fit->mask;
    ESP_LOGI(TAG, "  fit: %s -- centre {%.0f, %.0f} radius %.0f ratio %.3f rms %.4f "
                  "slices %d/32 gyro scale %.3f worst dev %.2f deg (%d samples)",
             mag_cal_verdict_text(v), fit->cal.center[0], fit->cal.center[1],
             fit->cal.radius, fit->axis_ratio, fit->rms, fit->bins,
             fit->gyro_scale, fit->gyro_dev_deg, circle.n);
    if (v != MAG_CAL_PASS) {
        fail((compass_cal_reason_t)(COMPASS_CAL_REASON_FIT_BASE + (int)v));
        return false;
    }
    return true;
}

static void run_calibration(void)
{
    float gyro_bias[3], accel[3];
    mag_fit_t fit;
    if (!hold_still(gyro_bias, accel) || !spin_and_fit(gyro_bias, &fit)) return;

    CalibrationData next = *s_live;
    next.magic_word = CALIB_MAGIC_WORD;
    memcpy(next.g_bias, gyro_bias, sizeof(next.g_bias));
    next.roll_tare = atan2f(accel[1], accel[2]) * RAD_TO_DEG;
    next.pitch_tare = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) *
                      RAD_TO_DEG;
    memcpy(next.mag_center, fit.cal.center, sizeof(next.mag_center));
    memcpy(next.mag_soft, fit.cal.soft, sizeof(next.mag_soft));
    next.mag_radius = fit.cal.radius;
    next.mag_calibrated = 1;

    /* Saved first: a calibration that would vanish at the next power-on is
     * not applied either, so what runs always matches what is stored. */
    if (!fs_save_calibration(&next)) {
        fail(COMPASS_CAL_REASON_SAVE_FAILED);
        return;
    }
    int tries = 0;
    while (!fusion_set_calibration(&next) && ++tries < 200) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
    if (tries >= 200) {
        ESP_LOGE(TAG, "Fusion did not take the new calibration; it applies after reboot");
    }
    *s_live = next;

    enter(COMPASS_CAL_PASS);
    s_status.reason = COMPASS_CAL_REASON_NONE;
    status_led_set(STATUS_LED_CAL_DONE_OK);
    ESP_LOGI(TAG, "Calibration PASSED -- saved and in use (level p=%.2f r=%.2f)",
             next.pitch_tare, next.roll_tare);
}

static void task_compass_cal(void *arg)
{
    (void)arg;
    if (s_run_now) {
        run_calibration();
        atomic_store_explicit(&s_active, false, memory_order_release);
    }
    /* Stay alive: the runtime metrics keep this task's handle, and the
     * dashboard needs the calibration's health after a reboot too. */
    for (;;) {
        publish();
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_IDLE_US / 1000));
    }
}

esp_err_t compass_cal_start(CalibrationData *live, bool run_now)
{
    if (!live) return ESP_ERR_INVALID_ARG;
    s_live = live;
    s_run_now = run_now;
    s_status = (boat_CompassCalStatus)boat_CompassCalStatus_init_zero;
    s_status.state = COMPASS_CAL_IDLE;
    /* Lock arming before the task even exists. */
    atomic_store_explicit(&s_active, run_now, memory_order_release);
    esp_err_t err = runtime_task_create(RUNTIME_TASK_COMPASS_CAL, task_compass_cal,
                                        NULL, NULL);
    if (err != ESP_OK) {
        atomic_store_explicit(&s_active, false, memory_order_release);
    }
    return err;
}
