#include "sensor_fusion.h"

#include "sample_snapshot.h"
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
#define GYRO_SCALE_250DPS   131.0f
#define YAW_GYRO_SIGN        (+1.0f)
#define YAW_MAG_TILT_LIMIT   45.0f
#define YAW_MAG_NORM_MIN     800.0f

typedef struct {
    atomic_uint version;
    atomic_uint pitch_bits;
    atomic_uint roll_bits;
    atomic_uint heading_bits;
    atomic_uint yaw_rate_bits;
} fusion_result_slot_t;

typedef struct {
    fusion_result_slot_t slots[2];
    atomic_uint published_sequence;
} fusion_result_snapshot_t;

_Static_assert(sizeof(unsigned int) == 4, "fusion snapshot requires 32-bit unsigned int");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "fusion snapshot atomics must be lock-free");

extern sample_snapshot_t *sensor_imu_sample_snapshot(void);
extern void sensor_task_register_fusion_task(TaskHandle_t task);

static CalibrationData *calib;
static fusion_result_snapshot_t result_snapshot;
static float mag_filt[3] = {0};
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
        atomic_init(&slot->yaw_rate_bits, float_bits(0.0f));
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
    atomic_store_explicit(&slot->yaw_rate_bits, float_bits(result->yaw_rate), memory_order_relaxed);
    atomic_store_explicit(&slot->version, stable_version, memory_order_release);
    atomic_store_explicit(&result_snapshot.published_sequence, sequence, memory_order_release);
}

void fusion_init(CalibrationData *calib_data)
{
    calib = calib_data;
    fusion_result_snapshot_init();
    alpha = strtof(CONFIG_FUSION_COMPLEMENTARY_ALPHA, NULL);
    if (alpha <= 0.0f || alpha >= 1.0f) alpha = 0.96f;
    yaw_alpha = strtof(CONFIG_FUSION_YAW_ALPHA, NULL);
    if (yaw_alpha <= 0.0f || yaw_alpha >= 1.0f) yaw_alpha = 0.98f;
    declination_deg = strtof(CONFIG_HEADING_DECLINATION_DEG, NULL);
    pitch = calib->pitch_tare;
    roll = calib->roll_tare;
    memset(mag_filt, 0, sizeof(mag_filt));
    heading_yaw = 0.0f;
    yaw_init = false;
    last_capture_us = 0;
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
        FusionResult candidate = {
            .pitch = bits_float((uint32_t)atomic_load_explicit(&slot->pitch_bits, memory_order_relaxed)),
            .roll = bits_float((uint32_t)atomic_load_explicit(&slot->roll_bits, memory_order_relaxed)),
            .heading = bits_float((uint32_t)atomic_load_explicit(&slot->heading_bits, memory_order_relaxed)),
            .yaw_rate = bits_float((uint32_t)atomic_load_explicit(&slot->yaw_rate_bits, memory_order_relaxed)),
            .sequence = sequence,
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
    if (!sample || !sample->accel_gyro_valid || !calib) return;

    float dt = 0.0f;
    if (last_capture_us != 0 && sample->captured_us > last_capture_us) {
        dt = (float)(sample->captured_us - last_capture_us) / 1000000.0f;
        if (dt > 0.1f) dt = 0.1f;
    }
    last_capture_us = sample->captured_us;

    if (sample->mag_valid) {
        float mx_cal = ((float)sample->mx - calib->m_bias[0]) * calib->m_scale[0];
        float my_cal = ((float)sample->my - calib->m_bias[1]) * calib->m_scale[1];
        float mz_cal = ((float)sample->mz - calib->m_bias[2]) * calib->m_scale[2];
        mag_filt[0] += MAG_LPF_ALPHA * (mx_cal - mag_filt[0]);
        mag_filt[1] += MAG_LPF_ALPHA * (my_cal - mag_filt[1]);
        mag_filt[2] += MAG_LPF_ALPHA * (mz_cal - mag_filt[2]);
    }

    float gx_rate = ((float)sample->gx - calib->g_bias[0]) / GYRO_SCALE_250DPS;
    float gy_rate = ((float)sample->gy - calib->g_bias[1]) / GYRO_SCALE_250DPS;
    float acc_roll = atan2f(sample->ay, sample->az) * RAD_TO_DEG;
    float acc_denom = sqrtf((float)sample->ay * sample->ay + (float)sample->az * sample->az);
    if (acc_denom < ACCEL_EPSILON) acc_denom = ACCEL_EPSILON;
    float acc_pitch = atan2f(-sample->ax, acc_denom) * RAD_TO_DEG;
    roll = alpha * (roll + gx_rate * dt) + (1.0f - alpha) * acc_roll;
    pitch = alpha * (pitch + gy_rate * dt) + (1.0f - alpha) * acc_pitch;

    float p_rad = pitch * DEG_TO_RAD;
    float r_rad = roll * DEG_TO_RAD;
    float xh = mag_filt[0] * cosf(p_rad) + mag_filt[2] * sinf(p_rad);
    float yh = mag_filt[0] * sinf(r_rad) * sinf(p_rad) + mag_filt[1] * cosf(r_rad) - mag_filt[2] * sinf(r_rad) * cosf(p_rad);
    float mag_hdg = atan2f(yh, xh) * RAD_TO_DEG;
    float gz_rate = YAW_GYRO_SIGN * ((float)sample->gz - calib->g_bias[2]) / GYRO_SCALE_250DPS;

    if (!yaw_init && sample->mag_valid) {
        heading_yaw = mag_hdg;
        yaw_init = true;
    } else if (yaw_init) {
        heading_yaw += gz_rate * dt;
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

    float heading = heading_yaw - calib->heading_tare + declination_deg;
    while (heading >= 360.0f) heading -= 360.0f;
    while (heading < 0.0f) heading += 360.0f;
    fusion_publish_result(sample->sequence, &(FusionResult){
        .roll = roll - calib->roll_tare,
        .pitch = pitch - calib->pitch_tare,
        .heading = heading,
        .yaw_rate = gz_rate,
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
