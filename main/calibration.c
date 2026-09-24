#include "calibration.h"

#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "drivers/esc_driver.h"
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
#define STILL_ACCEL_STD_MAX   250.0f    /* counts at +/-2 g (~15 mg); the gyro does the fine work */
#define STILL_GYRO_STD_MAX    80.0f     /* counts, 0.6 deg/s; measured floor at rest 0.135 deg/s */
#define STILL_TIMEOUT_US      (60LL * 1000000LL)
#define GYRO_BIAS_SANITY_MAX  2000.0f   /* counts; larger = moved, or a bad IMU */

/* Step 2: spin. */
#define SPIN_TIMEOUT_US       (180LL * 1000000LL)
/* Nobody spinning (e.g. the automatic first-boot calibration on a boat
 * that was just set down): give up quickly so arming is not locked for
 * the whole timeout.  The previous calibration stays in use. */
#define SPIN_START_TIMEOUT_US (30LL * 1000000LL)
#define SPIN_START_MIN_DEG    45.0f
#define SPIN_CAPACITY         4000      /* samples kept; thinned beyond that */

#define POLL_MS               10
#define PUBLISH_RUN_US        200000LL  /* 5 Hz while calibrating */
#define PUBLISH_IDLE_US       1000000LL /* 1 Hz afterwards */

static CalibrationData *s_live;
static bool s_run_now;
static bool s_stored_valid;   /* a valid calibration was loaded from flash */
static atomic_bool s_active;
/* Dashboard requests, set from the pipeline's receive path. */
static atomic_bool s_start_requested;
static atomic_bool s_cancel_requested;
/* Compass-vs-gyro limit for the next run, float bits (written before the
 * start flag, read after it).  The run itself uses s_tolerance_deg. */
static atomic_uint s_requested_tolerance_bits;
static float s_tolerance_deg = MAG_CAL_GYRO_DEV_RELAXED;
static boat_CompassCalStatus s_status;   /* owned by the CompassCal task */

bool compass_cal_active(void)
{
    return atomic_load_explicit(&s_active, memory_order_acquire);
}

static void publish(void)
{
    s_status.calibrated = s_live && s_live->mag_calibrated;
    s_status.saved_dev_deg = s_status.calibrated ? s_live->mag_gyro_dev_deg : 0.0f;
    s_status.saved_tolerance_deg = s_status.calibrated ? s_live->mag_tolerance_deg : 0.0f;
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

static bool cancelled(void)
{
    return atomic_load_explicit(&s_cancel_requested, memory_order_acquire);
}

static void fail(compass_cal_reason_t reason)
{
    enter(COMPASS_CAL_FAIL);
    s_status.reason = reason;
    status_led_set(STATUS_LED_CAL_DONE_FAIL);
    ESP_LOGW(TAG, "Calibration FAILED (reason %d)", (int)reason);
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
        if (cancelled()) {
            fail(COMPASS_CAL_REASON_CANCELLED);
            return false;
        }
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

/* One raw sample of the spin, kept so the whole run can be written to the SD
 * card afterwards (CALSPIN.CSV): a FAIL can then be diagnosed from the data
 * instead of guessed at. */
typedef struct {
    uint32_t t_ms;
    int16_t mx, my, mz, ax, ay, az, gz;
    uint8_t mag_valid;
    uint8_t stored;          /* 1 = used for the fit, 0 = skipped (tilt / no compass) */
} spin_raw_t;

#define SPIN_RAW_CAPACITY   9500           /* 180 s at 50 Hz, with margin */
#define SPIN_MAX_TILT_DEG   3.0f           /* tilted further than this: reading skipped */
#define SPIN_CSV            "CALSPIN.CSV"  /* 8.3 name (FATFS without long names) */

static void write_spin_csv(const spin_raw_t *raw, int n, const float gyro_bias[3],
                           const float accel_ref[3], const char *verdict,
                           const mag_fit_t *fit)
{
    if (!raw || n <= 0 || !fs_sdcard_ready()) return;
    const size_t cap = 4096;
    char *buf = malloc(cap);
    if (!buf) return;
    int len = snprintf(buf, cap,
        "# BoatEspP4 compass spin. verdict=%s centre=%.1f,%.1f radius=%.1f ratio=%.4f "
        "rms=%.4f slices=%d gyro_scale=%.4f worst_dev_deg=%.2f limit_deg=%.1f\n"
        "# gyro_bias=%.2f,%.2f,%.2f counts  level_ref_accel=%.1f,%.1f,%.1f  max_tilt_deg=%.1f\n"
        "t_ms,mx,my,mz,ax,ay,az,gz,mag_valid,stored\n",
        verdict, fit->cal.center[0], fit->cal.center[1], fit->cal.radius,
        fit->axis_ratio, fit->rms, fit->bins, fit->gyro_scale, fit->gyro_dev_deg,
        s_tolerance_deg, gyro_bias[0], gyro_bias[1], gyro_bias[2],
        accel_ref[0], accel_ref[1], accel_ref[2], SPIN_MAX_TILT_DEG);
    esp_err_t err = fs_sdcard_write(SPIN_CSV, buf, (size_t)len);
    for (int i = 0; i < n && err == ESP_OK; ) {
        len = 0;
        for (; i < n && len < (int)cap - 96; i++) {
            const spin_raw_t *r = &raw[i];
            len += snprintf(buf + len, cap - len, "%lu,%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
                            (unsigned long)r->t_ms, r->mx, r->my, r->mz,
                            r->ax, r->ay, r->az, r->gz, r->mag_valid, r->stored);
        }
        err = fs_sdcard_append(SPIN_CSV, buf, (size_t)len);
    }
    free(buf);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "  spin data: %d samples written to /sdcard/%s", n, SPIN_CSV);
    } else {
        ESP_LOGW(TAG, "  spin data: SD write failed (%s)", esp_err_to_name(err));
    }
}

static bool spin_and_fit(const float gyro_bias[3], const float accel_ref[3], mag_fit_t *fit)
{
    size_t bytes = SPIN_CAPACITY * sizeof(float);
    /* Only for the length of the spin.  PSRAM first; internal RAM if PSRAM
     * is short, so a busy moment is never a false FAIL.  The raw record is
     * optional: without it the calibration still runs, only the CSV is lost. */
    float *bx = heap_caps_malloc_prefer(bytes, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT);
    float *by = heap_caps_malloc_prefer(bytes, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT);
    float *bt = heap_caps_malloc_prefer(bytes, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT);
    spin_raw_t *raw = heap_caps_malloc(SPIN_RAW_CAPACITY * sizeof(spin_raw_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int n_raw = 0;
    bool ok = false;
    mag_cal_verdict_t v = MAG_CAL_FAIL_TOO_FEW;
    memset(fit, 0, sizeof(*fit));
    if (!bx || !by || !bt) {
        fail(COMPASS_CAL_REASON_NO_MEMORY);
        goto out;
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
            /* A tilted boat leaks its strong vertical field into x/y: keep
             * only readings taken within SPIN_MAX_TILT_DEG of the level
             * reference.  The gyro keeps counting either way. */
            float a[3] = {(float)s.ax, (float)s.ay, (float)s.az};
            bool level = mag_cal_tilt_deg(a, accel_ref) <= SPIN_MAX_TILT_DEG;
            bool use = s.mag_valid && level;
            if (s.mag_valid && !level) {
                s_status.tilted = true;
                s_status.tilt_skipped++;
            }
            mag_circle_add(&circle, use ? (float)s.mx : NAN, use ? (float)s.my : NAN,
                           rate, dt);
            if (raw && n_raw < SPIN_RAW_CAPACITY) {
                raw[n_raw++] = (spin_raw_t){
                    .t_ms = (uint32_t)((int64_t)s.captured_us / 1000 - t0 / 1000),
                    .mx = s.mx, .my = s.my, .mz = s.mz,
                    .ax = s.ax, .ay = s.ay, .az = s.az, .gz = s.gz,
                    .mag_valid = s.mag_valid, .stored = use,
                };
            }
        }
        int64_t now = esp_timer_get_time();
        s_status.turn_deg = circle.turn_deg;
        s_status.elapsed_s = (float)(now - t0) / 1e6f;
        if (now - last_pub >= PUBLISH_RUN_US) {
            s_status.coverage_mask = mag_circle_live_mask(&circle);
            publish();
            s_status.tilted = false;          /* "tilted" = since the last update */
            last_pub = now;
        }
        if (mag_circle_complete(&circle)) { complete = true; break; }
        if (cancelled()) {
            fail(COMPASS_CAL_REASON_CANCELLED);
            goto out;
        }
        if (now - t0 > SPIN_START_TIMEOUT_US &&
            fabsf(circle.turn_deg) < SPIN_START_MIN_DEG) {
            ESP_LOGW(TAG, "  spin: never started (%.0f deg in 30 s)", circle.turn_deg);
            fail(COMPASS_CAL_REASON_SPIN_NOT_STARTED);
            goto out;
        }
        if (now - t0 > SPIN_TIMEOUT_US) break;
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
    s_status.coverage_mask = mag_circle_live_mask(&circle);
    s_status.tilted = false;
    ESP_LOGI(TAG, "  spin: %lu compass readings skipped for tilt > %.0f deg",
             (unsigned long)s_status.tilt_skipped, SPIN_MAX_TILT_DEG);

    if (!complete) {
        ESP_LOGW(TAG, "  spin: not completed (%d samples, %.0f deg turned, %d/32 slices)",
                 circle.n, circle.turn_deg, __builtin_popcount(s_status.coverage_mask));
        fail(COMPASS_CAL_REASON_SPIN_TIMEOUT);
        write_spin_csv(raw, n_raw, gyro_bias, accel_ref, "SPIN_TIMEOUT", fit);
        goto out;
    }

    enter(COMPASS_CAL_CHECKING);
    publish();
    v = mag_cal_fit_and_judge(bx, by, bt, circle.n, s_tolerance_deg, fit);

    s_status.axis_ratio = fit->axis_ratio;
    s_status.fit_rms = fit->rms;
    s_status.gyro_scale = fit->gyro_scale;
    s_status.gyro_dev_deg = fit->gyro_dev_deg;
    s_status.radius = fit->cal.radius;
    s_status.coverage_mask = fit->mask;
    ESP_LOGI(TAG, "  fit: %s -- centre {%.0f, %.0f} radius %.0f ratio %.3f rms %.4f "
                  "slices %d/32 gyro scale %.3f worst dev %.2f deg (limit %.0f, %d samples)",
             mag_cal_verdict_text(v), fit->cal.center[0], fit->cal.center[1],
             fit->cal.radius, fit->axis_ratio, fit->rms, fit->bins,
             fit->gyro_scale, fit->gyro_dev_deg, s_tolerance_deg, circle.n);
    write_spin_csv(raw, n_raw, gyro_bias, accel_ref, mag_cal_verdict_text(v), fit);
    if (v != MAG_CAL_PASS) {
        fail((compass_cal_reason_t)(COMPASS_CAL_REASON_FIT_BASE + (int)v));
        goto out;
    }
    ok = true;

out:
    heap_caps_free(bx);
    heap_caps_free(by);
    heap_caps_free(bt);
    heap_caps_free(raw);
    return ok;
}

static bool apply(const CalibrationData *next)
{
    /* Saved first: a calibration that would vanish at the next power-on is
     * not applied either, so what runs always matches what is stored. */
    if (!fs_save_calibration(next)) return false;
    s_stored_valid = true;          /* flash now holds a valid record */
    int tries = 0;
    while (!fusion_set_calibration(next) && ++tries < 200) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
    if (tries >= 200) {
        ESP_LOGE(TAG, "Fusion has not taken the new calibration yet (fusion stalled?); "
                      "it will on its next sample, and it is saved for the next boot");
    }
    *s_live = *next;
    return true;
}

static void run_calibration(void)
{
    float gyro_bias[3], accel[3];
    mag_fit_t fit;
    if (!hold_still(gyro_bias, accel)) return;

    CalibrationData next = *s_live;
    next.magic_word = CALIB_MAGIC_WORD;
    memcpy(next.g_bias, gyro_bias, sizeof(next.g_bias));
    next.roll_tare = atan2f(accel[1], accel[2]) * RAD_TO_DEG;
    next.pitch_tare = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) *
                      RAD_TO_DEG;

    if (!spin_and_fit(gyro_bias, accel, &fit)) {
        /* With a good calibration stored, a FAIL changes nothing.  With none
         * (first boot after a flash), keep the fresh gyro drift + level from
         * step 1 rather than run the heading hold on an uncorrected gyro;
         * the compass stays marked NOT calibrated.  Said so in the status. */
        if (!s_stored_valid) {
            memcpy(next.mag_center, (float[2]){0.0f, 0.0f}, sizeof(next.mag_center));
            memcpy(next.mag_soft, (float[4]){1.0f, 0.0f, 0.0f, 1.0f}, sizeof(next.mag_soft));
            next.mag_radius = 0.0f;
            next.mag_calibrated = 0;
            next.mag_gyro_dev_deg = 0.0f;
            next.mag_tolerance_deg = 0.0f;
            if (apply(&next)) {
                s_status.gyro_level_saved = true;
                ESP_LOGW(TAG, "No stored calibration: gyro drift + level saved, "
                              "compass still NOT calibrated");
            } else {
                ESP_LOGE(TAG, "No stored calibration and saving gyro drift + level failed");
            }
        }
        return;
    }
    memcpy(next.mag_center, fit.cal.center, sizeof(next.mag_center));
    memcpy(next.mag_soft, fit.cal.soft, sizeof(next.mag_soft));
    next.mag_radius = fit.cal.radius;
    next.mag_calibrated = 1;
    next.mag_gyro_dev_deg = fit.gyro_dev_deg;
    next.mag_tolerance_deg = s_tolerance_deg;

    if (!apply(&next)) {
        fail(COMPASS_CAL_REASON_SAVE_FAILED);
        return;
    }

    enter(COMPASS_CAL_PASS);
    s_status.reason = COMPASS_CAL_REASON_NONE;
    status_led_set(STATUS_LED_CAL_DONE_OK);
    ESP_LOGI(TAG, "Calibration PASSED -- saved and in use (gyro %.1f deg, limit %.0f, "
                  "level p=%.2f r=%.2f)", next.mag_gyro_dev_deg, next.mag_tolerance_deg,
             next.pitch_tare, next.roll_tare);
}

/* A finished run's PASS/FAIL (or a refused start) stays on the dashboard for
 * this long, then the card goes back to the live health view; the result is
 * kept in last_state / last_reason. */
#define RESULT_SHOW_US  (10LL * 1000000LL)
static uint32_t s_run_id;
static int64_t s_result_until_us;

/* One full run.  s_active must already be set (arming locked). */
static void run_once(void)
{
    atomic_store_explicit(&s_cancel_requested, false, memory_order_release);
    uint32_t last_state = s_status.last_state, last_reason = s_status.last_reason;
    s_status = (boat_CompassCalStatus)boat_CompassCalStatus_init_zero;   /* fresh run */
    s_status.run_id = ++s_run_id;
    s_status.tolerance_deg = s_tolerance_deg;
    s_status.last_state = last_state;
    s_status.last_reason = last_reason;
    run_calibration();
    s_status.last_state = s_status.state;
    s_status.last_reason = s_status.reason;
    s_result_until_us = esp_timer_get_time() + RESULT_SHOW_US;
    atomic_store_explicit(&s_active, false, memory_order_release);
    publish();
}

/* Motors armed or arming: never ask the operator to handle the boat.  The
 * previous result (last_*) and metrics are left as they were. */
static void refuse_armed(void)
{
    atomic_store_explicit(&s_active, false, memory_order_release);
    s_status.run_id = ++s_run_id;
    enter(COMPASS_CAL_FAIL);
    s_status.reason = COMPASS_CAL_REASON_ARMED;
    s_status.gyro_level_saved = false;
    s_result_until_us = esp_timer_get_time() + RESULT_SHOW_US;
    ESP_LOGW(TAG, "Calibration refused: motors armed -- disarm first");
    publish();
}

/* s_active is already set when this is called (boot: from the moment boot
 * decided to calibrate; dashboard: just before), so an arm step arriving
 * now is refused by motor_control's lock instead of racing past the check. */
static void start_checked(const char *source)
{
    if (esc_driver_get_state() != ESC_STATE_DISARMED) {
        refuse_armed();
        return;
    }
    ESP_LOGI(TAG, "Calibration started (%s)", source);
    run_once();
}

static void task_compass_cal(void *arg)
{
    (void)arg;
    /* Power-on runs are the indoor, by-hand case: relaxed. */
    s_tolerance_deg = MAG_CAL_GYRO_DEV_RELAXED;
    if (s_run_now) start_checked("power-on");

    /* Stay alive: the runtime metrics keep this task's handle, the dashboard
     * can start a calibration at any time, and it shows the health. */
    int64_t last_pub = 0;
    for (;;) {
        if (atomic_exchange_explicit(&s_start_requested, false, memory_order_acq_rel)) {
            atomic_store_explicit(&s_active, true, memory_order_release);
            uint32_t bits = atomic_load_explicit(&s_requested_tolerance_bits, memory_order_acquire);
            float requested;
            memcpy(&requested, &bits, sizeof(requested));
            s_tolerance_deg = mag_cal_tolerance_deg(requested);
            start_checked("dashboard");
            last_pub = esp_timer_get_time();
        }
        int64_t now = esp_timer_get_time();
        if ((s_status.state == COMPASS_CAL_PASS || s_status.state == COMPASS_CAL_FAIL) &&
            now >= s_result_until_us) {
            s_status.state = COMPASS_CAL_IDLE;
            s_status.reason = COMPASS_CAL_REASON_NONE;
        }
        if (now - last_pub >= PUBLISH_IDLE_US) {
            publish();
            last_pub = now;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void on_dashboard_command(bool start, bool cancel, float tolerance_deg)
{
    if (cancel) {
        /* Only meaningful while a run is in progress; ignored otherwise. */
        if (compass_cal_active()) {
            atomic_store_explicit(&s_cancel_requested, true, memory_order_release);
        }
        return;
    }
    if (start && !compass_cal_active()) {
        uint32_t bits;
        memcpy(&bits, &tolerance_deg, sizeof(bits));
        atomic_store_explicit(&s_requested_tolerance_bits, bits, memory_order_release);
        atomic_store_explicit(&s_start_requested, true, memory_order_release);
    }
}

void compass_cal_lock_for_boot(void)
{
    atomic_store_explicit(&s_active, true, memory_order_release);
}

esp_err_t compass_cal_start(CalibrationData *live, bool run_now, bool stored_valid)
{
    if (!live) return ESP_ERR_INVALID_ARG;
    s_live = live;
    s_run_now = run_now;
    s_stored_valid = stored_valid;
    s_status = (boat_CompassCalStatus)boat_CompassCalStatus_init_zero;
    s_status.state = COMPASS_CAL_IDLE;
    /* Keeps (or releases) the lock compass_cal_lock_for_boot() took. */
    atomic_store_explicit(&s_active, run_now, memory_order_release);
    atomic_store_explicit(&s_start_requested, false, memory_order_release);
    atomic_store_explicit(&s_cancel_requested, false, memory_order_release);
    esp_err_t err = runtime_task_create(RUNTIME_TASK_COMPASS_CAL, task_compass_cal,
                                        NULL, NULL);
    if (err != ESP_OK) {
        atomic_store_explicit(&s_active, false, memory_order_release);
        return err;
    }
    pipeline_register_compass_cal_handler(on_dashboard_command);
    return ESP_OK;
}
