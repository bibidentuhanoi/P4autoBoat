#include "sensor_fusion.h"

#include "sample_snapshot.h"
#include "mag_cal.h"
#include "esp_timer.h"
#include "runtime_metrics.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#define MAG_LPF_ALPHA       0.2f
#define ACCEL_EPSILON       0.0001f
#define GYRO_SCALE_250DPS   FUSION_GYRO_COUNTS_PER_DPS
#define YAW_GYRO_SIGN       FUSION_YAW_GYRO_SIGN
#define YAW_MAG_TILT_LIMIT   45.0f
#define YAW_MAG_NORM_MIN     800.0f
/* Field-strength ratio smoothing: ~1 s at the 50 Hz fusion rate. */
#define FIELD_RATIO_ALPHA    0.02f

typedef struct {
    atomic_uint version;
    atomic_uint pitch_bits;
    atomic_uint roll_bits;
    atomic_uint heading_bits;
    atomic_bool heading_valid;
    atomic_uint yaw_rate_bits;
    atomic_uint captured_us_lo;
    atomic_uint captured_us_hi;
} fusion_result_slot_t;

typedef struct {
    fusion_result_slot_t slots[2];
    atomic_uint published_sequence;
} fusion_result_snapshot_t;

_Static_assert(sizeof(unsigned int) == 4, "fusion snapshot requires 32-bit unsigned int");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "fusion snapshot atomics must be lock-free");

extern sample_snapshot_t *sensor_imu_sample_snapshot(void);
extern void sensor_task_register_fusion_task(TaskHandle_t task);

/* The fusion task's own copy.  A new calibration arrives through `pending`:
 * the writer fills it only while the flag is clear, then sets the flag; the
 * fusion task copies it and clears the flag.  One writer, one reader, no
 * half-written struct is ever read. */
static CalibrationData s_cal;
static bool s_have_cal;
static CalibrationData s_pending;
static atomic_bool s_pending_ready;
static atomic_uint s_field_ratio_bits;
static float field_ratio = 0.0f;
static bool mag_seeded;

static fusion_result_snapshot_t result_snapshot;
static float mag_filt[2] = {0};
static float pitch;
static float roll;
static float alpha;
static float yaw_alpha;
static float declination_deg;
static float heading_yaw;
static bool yaw_init;
static uint64_t last_capture_us;

static uint32_t float_bits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static float bits_float(uint32_t bits)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void fusion_result_snapshot_init(void)
{
    for (unsigned index = 0; index < 2; ++index) {
        fusion_result_slot_t *slot = &result_snapshot.slots[index];
        atomic_init(&slot->version, 0);
        atomic_init(&slot->pitch_bits, float_bits(0.0f));
        atomic_init(&slot->roll_bits, float_bits(0.0f));
        atomic_init(&slot->heading_bits, float_bits(0.0f));
        atomic_init(&slot->heading_valid, false);
        atomic_init(&slot->yaw_rate_bits, float_bits(0.0f));
        atomic_init(&slot->captured_us_lo, 0);
        atomic_init(&slot->captured_us_hi, 0);
    }
    atomic_init(&result_snapshot.published_sequence, 0);
}

static void fusion_publish_result(uint32_t sequence, const FusionResult *result)
{
    fusion_result_slot_t *slot = &result_snapshot.slots[sequence & 1U];
    unsigned stable_version = sequence << 1U;
    atomic_exchange_explicit(&slot->version, stable_version | 1U, memory_order_acq_rel);
    atomic_store_explicit(&slot->pitch_bits, float_bits(result->pitch), memory_order_relaxed);
    atomic_store_explicit(&slot->roll_bits, float_bits(result->roll), memory_order_relaxed);
    atomic_store_explicit(&slot->heading_bits, float_bits(result->heading), memory_order_relaxed);
    atomic_store_explicit(&slot->heading_valid, result->heading_valid, memory_order_relaxed);
    atomic_store_explicit(&slot->yaw_rate_bits, float_bits(result->yaw_rate), memory_order_relaxed);
    atomic_store_explicit(&slot->captured_us_lo,
                          (uint32_t)(result->captured_us & 0xFFFFFFFFu), memory_order_relaxed);
    atomic_store_explicit(&slot->captured_us_hi,
                          (uint32_t)(result->captured_us >> 32), memory_order_relaxed);
    atomic_store_explicit(&slot->version, stable_version, memory_order_release);
    atomic_store_explicit(&result_snapshot.published_sequence, sequence, memory_order_release);
}

void fusion_init(const CalibrationData *calib_data)
{
    s_have_cal = calib_data != NULL;
    if (s_have_cal) s_cal = *calib_data;
    atomic_init(&s_pending_ready, false);
    atomic_init(&s_field_ratio_bits, float_bits(0.0f));
    field_ratio = 0.0f;
    fusion_result_snapshot_init();
    alpha = strtof(CONFIG_FUSION_COMPLEMENTARY_ALPHA, NULL);
    if (alpha <= 0.0f || alpha >= 1.0f) alpha = 0.96f;
    yaw_alpha = strtof(CONFIG_FUSION_YAW_ALPHA, NULL);
    if (yaw_alpha <= 0.0f || yaw_alpha >= 1.0f) yaw_alpha = 0.98f;
    declination_deg = strtof(CONFIG_HEADING_DECLINATION_DEG, NULL);
    pitch = s_have_cal ? s_cal.pitch_tare : 0.0f;
    roll = s_have_cal ? s_cal.roll_tare : 0.0f;
    memset(mag_filt, 0, sizeof(mag_filt));
    mag_seeded = false;
    heading_yaw = 0.0f;
    yaw_init = false;
    last_capture_us = 0;
}

bool fusion_set_calibration(const CalibrationData *calib_data)
{
    if (!calib_data) return false;
    if (atomic_load_explicit(&s_pending_ready, memory_order_acquire)) {
        return false;                       /* previous hand-over not taken yet */
    }
    s_pending = *calib_data;
    atomic_store_explicit(&s_pending_ready, true, memory_order_release);
    return true;
}

bool fusion_calibration_pending(void)
{
    return atomic_load_explicit(&s_pending_ready, memory_order_acquire);
}

float fusion_get_field_ratio(void)
{
    return bits_float((uint32_t)atomic_load_explicit(&s_field_ratio_bits,
                                                     memory_order_relaxed));
}

static void take_pending_calibration(void)
{
    if (!atomic_load_explicit(&s_pending_ready, memory_order_acquire)) return;
    s_cal = s_pending;
    s_have_cal = true;
    atomic_store_explicit(&s_pending_ready, false, memory_order_release);
    /* Start the compass path over from the next real reading, and take the
     * heading straight from it: the old estimate was made with the old
     * correction.  Pitch/roll state is raw and stays; only the level
     * reference subtracted at the output changes. */
    mag_seeded = false;
    yaw_init = false;
    field_ratio = 0.0f;
}

void fusion_get_result(FusionResult *res)
{
    for (;;) {
        unsigned sequence = atomic_load_explicit(&result_snapshot.published_sequence, memory_order_acquire);
        if (sequence == 0) {
            *res = (FusionResult){0};
            return;
        }
        const fusion_result_slot_t *slot = &result_snapshot.slots[sequence & 1U];
        unsigned expected_version = sequence << 1U;
        unsigned before = atomic_load_explicit(&slot->version, memory_order_acquire);
        if (before != expected_version) continue;
        uint64_t captured_us =
            ((uint64_t)atomic_load_explicit(&slot->captured_us_hi, memory_order_relaxed) << 32) |
            (uint64_t)atomic_load_explicit(&slot->captured_us_lo, memory_order_relaxed);
        FusionResult candidate = {
            .pitch = bits_float((uint32_t)atomic_load_explicit(&slot->pitch_bits, memory_order_relaxed)),
            .roll = bits_float((uint32_t)atomic_load_explicit(&slot->roll_bits, memory_order_relaxed)),
            .heading = bits_float((uint32_t)atomic_load_explicit(&slot->heading_bits, memory_order_relaxed)),
            .heading_valid = atomic_load_explicit(&slot->heading_valid, memory_order_relaxed),
            .yaw_rate = bits_float((uint32_t)atomic_load_explicit(&slot->yaw_rate_bits, memory_order_relaxed)),
            .sequence = sequence,
            .captured_us = captured_us,
        };
        /* Full reader-side barrier: validate only after this entire result
         * payload was loaded, including on weakly ordered RISC-V cores. */
        atomic_thread_fence(memory_order_seq_cst);
        unsigned after = atomic_load_explicit(&slot->version, memory_order_acquire);
        if (before == after) {
            *res = candidate;
            return;
        }
    }
}

void fusion_update_sample(const imu_sample_t *sample)
{
    take_pending_calibration();
    if (!sample || !sample->accel_gyro_valid || !s_have_cal) return;
    const CalibrationData *calib = &s_cal;

    float dt = 0.0f;
    if (last_capture_us != 0 && sample->captured_us > last_capture_us) {
        dt = (float)(sample->captured_us - last_capture_us) / 1000000.0f;
        if (dt > 0.1f) dt = 0.1f;
    }
    last_capture_us = sample->captured_us;

    if (sample->mag_valid) {
        mag_cal_2d_t mag;
        memcpy(mag.center, calib->mag_center, sizeof(mag.center));
        memcpy(mag.soft, calib->mag_soft, sizeof(mag.soft));
        mag.radius = calib->mag_radius;
        float cx, cy;
        mag_cal_apply(&mag, (float)sample->mx, (float)sample->my, &cx, &cy);
        if (!mag_seeded) {
            mag_filt[0] = cx;
            mag_filt[1] = cy;
            mag_seeded = true;
        } else {
            mag_filt[0] += MAG_LPF_ALPHA * (cx - mag_filt[0]);
            mag_filt[1] += MAG_LPF_ALPHA * (cy - mag_filt[1]);
        }
        /* Health check against the field strength measured at calibration:
         * 1.0 = same field as then.  Reported only, not used to gate. */
        if (calib->mag_calibrated && calib->mag_radius > 0.0f) {
            float r = sqrtf(cx * cx + cy * cy) / calib->mag_radius;
            field_ratio = (field_ratio == 0.0f) ? r
                        : field_ratio + FIELD_RATIO_ALPHA * (r - field_ratio);
        } else {
            field_ratio = 0.0f;
        }
        atomic_store_explicit(&s_field_ratio_bits, float_bits(field_ratio),
                              memory_order_relaxed);
    }

    float gx_rate = ((float)sample->gx - calib->g_bias[0]) / GYRO_SCALE_250DPS;
    float gy_rate = ((float)sample->gy - calib->g_bias[1]) / GYRO_SCALE_250DPS;
    float acc_roll = atan2f(sample->ay, sample->az) * RAD_TO_DEG;
    float acc_denom = sqrtf((float)sample->ay * sample->ay + (float)sample->az * sample->az);
    if (acc_denom < ACCEL_EPSILON) acc_denom = ACCEL_EPSILON;
    float acc_pitch = atan2f(-sample->ax, acc_denom) * RAD_TO_DEG;
    roll = alpha * (roll + gx_rate * dt) + (1.0f - alpha) * acc_roll;
    pitch = alpha * (pitch + gy_rate * dt) + (1.0f - alpha) * acc_pitch;

    /* Flat compass: the boat stays within a few degrees of level and the
     * calibration is fitted on these same untilted x/y readings, so heading
     * is taken from them directly.  The old tilt compensation mixed the
     * compass axes with the IMU chip's axes, whose relative mounting has
     * never been verified, and needed a vertical offset a flat calibration
     * cannot measure. */
    float xh = mag_filt[0];
    float yh = mag_filt[1];
    float mag_hdg = mag_cal_heading_deg(xh, yh);
    float gz_rate = YAW_GYRO_SIGN * ((float)sample->gz - calib->g_bias[2]) / GYRO_SCALE_250DPS;

    if (!yaw_init && sample->mag_valid && mag_seeded) {
        heading_yaw = mag_hdg;
        yaw_init = true;
    } else if (yaw_init) {
        /* Positive installed gyro yaw is left/counter-clockwise. Compass
         * heading increases clockwise, so the gyro prediction has the
         * opposite sign. */
        heading_yaw -= gz_rate * dt;
        if (sample->mag_valid) {
            float error = mag_hdg - heading_yaw;
            while (error > 180.0f) error -= 360.0f;
            while (error < -180.0f) error += 360.0f;
            float mag_norm = sqrtf(xh * xh + yh * yh);
            if (mag_norm > YAW_MAG_NORM_MIN &&
                fabsf(pitch) < YAW_MAG_TILT_LIMIT &&
                fabsf(roll) < YAW_MAG_TILT_LIMIT) {
                heading_yaw += (1.0f - yaw_alpha) * error;
            }
        }
    }
    while (heading_yaw >= 360.0f) heading_yaw -= 360.0f;
    while (heading_yaw < 0.0f) heading_yaw += 360.0f;

    float heading = heading_yaw + declination_deg;
    while (heading >= 360.0f) heading -= 360.0f;
    while (heading < 0.0f) heading += 360.0f;
    fusion_publish_result(sample->sequence, &(FusionResult){
        .roll = roll - calib->roll_tare,
        .pitch = pitch - calib->pitch_tare,
        .heading = heading,
        .heading_valid = yaw_init,
        .yaw_rate = gz_rate,
        .captured_us = sample->captured_us,
    });
}

void task_imu_fusion(void *pvParameters)
{
    (void)pvParameters;
    sensor_task_register_fusion_task(xTaskGetCurrentTaskHandle());
    sample_snapshot_t *samples = sensor_imu_sample_snapshot();
    uint32_t last_sequence = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_sample_t sample;
        if (!sample_snapshot_read(samples, &sample) || sample.sequence == last_sequence) continue;
        if (last_sequence != 0 && sample.sequence > last_sequence + 1U) {
            for (uint32_t skipped = last_sequence + 1U; skipped < sample.sequence; ++skipped) {
                runtime_metrics_count(RUNTIME_TASK_FUSION, RUNTIME_EVENT_SENSOR_SKIP);
            }
        }
        uint64_t started = esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_FUSION, sample.captured_us, started);
        fusion_update_sample(&sample);
        runtime_metrics_cycle_end(RUNTIME_TASK_FUSION, esp_timer_get_time());
        last_sequence = sample.sequence;
    }
}
