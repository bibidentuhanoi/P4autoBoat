#include <assert.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include "sensor_fusion.h"

static uint32_t float_bits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static imu_sample_t make_sample(uint32_t sequence)
{
    return (imu_sample_t){
        .ax = 0,
        .ay = 0,
        .az = 16384,
        .gx = (sequence & 1U) ? 131 : -131,
        .gy = (sequence & 1U) ? -131 : 131,
        .gz = (sequence & 1U) ? 131 : -131,
        .mx = (int16_t)(3000 + sequence),
        .my = (int16_t)(900 + sequence),
        .mz = 200,
        .accel_gyro_valid = true,
        .mag_valid = true,
        .captured_us = (uint64_t)sequence * 20000U,
        .sequence = sequence,
    };
}

static CalibrationData calibration(void)
{
    return (CalibrationData){
        .mag_soft = {1.0f, 0.0f, 0.0f, 1.0f},
    };
}

static void test_fusion_uses_sample_timestamp_and_ignores_mag_only_sample(void)
{
    CalibrationData calib = calibration();
    fusion_init(&calib);

    imu_sample_t first = make_sample(1);
    first.gx = 0;
    first.gy = 0;
    first.gz = 0;
    first.captured_us = 10;
    fusion_update_sample(&first);

    imu_sample_t delayed = first;
    delayed.sequence = 2;
    delayed.gx = 131;
    delayed.captured_us = 1000010;
    fusion_update_sample(&delayed);

    FusionResult before_partial;
    fusion_get_result(&before_partial);
    assert(before_partial.roll > 0.09f && before_partial.roll < 0.10f);

    imu_sample_t mag_only = delayed;
    mag_only.sequence = 3;
    mag_only.accel_gyro_valid = false;
    mag_only.mag_valid = true;
    mag_only.captured_us += 20000;
    fusion_update_sample(&mag_only);

    FusionResult after_partial;
    fusion_get_result(&after_partial);
    assert(float_bits(after_partial.pitch) == float_bits(before_partial.pitch));
    assert(float_bits(after_partial.roll) == float_bits(before_partial.roll));
    assert(float_bits(after_partial.heading) == float_bits(before_partial.heading));
}

static float heading_delta(float after, float before)
{
    float delta = after - before;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

static void test_positive_left_yaw_decreases_compass_heading_prediction(void)
{
    CalibrationData calib = calibration();
    fusion_init(&calib);

    imu_sample_t first = make_sample(1);
    first.gx = 0;
    first.gy = 0;
    first.gz = 0;
    first.captured_us = 1000;
    fusion_update_sample(&first);
    FusionResult before;
    fusion_get_result(&before);

    imu_sample_t turning = first;
    turning.sequence = 2;
    turning.gz = 1310;             /* +10 deg/s: installed left-turn sign */
    turning.mag_valid = false;     /* isolate gyro prediction */
    turning.captured_us = 101000;  /* 0.1 s -> about one degree */
    fusion_update_sample(&turning);
    FusionResult after;
    fusion_get_result(&after);

    const float delta = heading_delta(after.heading, before.heading);
    assert(delta < -0.9f && delta > -1.1f);
    assert(after.yaw_rate > 9.9f && after.yaw_rate < 10.1f);
}

enum { FUSION_STRESS_SAMPLES = 2048 };

typedef struct {
    FusionResult expected[FUSION_STRESS_SAMPLES];
    atomic_bool done;
    atomic_bool torn_result;
} fusion_stress_state_t;

static bool is_expected_result(const FusionResult *result, const fusion_stress_state_t *state)
{
    for (unsigned index = 0; index < FUSION_STRESS_SAMPLES; ++index) {
        const FusionResult *expected = &state->expected[index];
        if (float_bits(result->pitch) == float_bits(expected->pitch) &&
            float_bits(result->roll) == float_bits(expected->roll) &&
            float_bits(result->heading) == float_bits(expected->heading)) {
            return true;
        }
    }
    return result->pitch == 0.0f && result->roll == 0.0f && result->heading == 0.0f;
}

static void *publish_fusion_results(void *arg)
{
    fusion_stress_state_t *state = arg;
    for (uint32_t sequence = 1; sequence <= FUSION_STRESS_SAMPLES; ++sequence) {
        imu_sample_t sample = make_sample(sequence);
        fusion_update_sample(&sample);
    }
    atomic_store(&state->done, true);
    return NULL;
}

static void *read_fusion_results(void *arg)
{
    fusion_stress_state_t *state = arg;
    do {
        FusionResult result;
        fusion_get_result(&result);
        if (!is_expected_result(&result, state)) {
            atomic_store(&state->torn_result, true);
            break;
        }
    } while (!atomic_load(&state->done));
    return NULL;
}

static void test_result_reader_observes_only_complete_fusion_generations(void)
{
    CalibrationData calib = calibration();
    fusion_stress_state_t state = {
        .done = ATOMIC_VAR_INIT(false),
        .torn_result = ATOMIC_VAR_INIT(false),
    };

    fusion_init(&calib);
    for (uint32_t sequence = 1; sequence <= FUSION_STRESS_SAMPLES; ++sequence) {
        imu_sample_t sample = make_sample(sequence);
        fusion_update_sample(&sample);
        fusion_get_result(&state.expected[sequence - 1]);
    }

    fusion_init(&calib);
    pthread_t writer;
    pthread_t reader;
    assert(pthread_create(&writer, NULL, publish_fusion_results, &state) == 0);
    assert(pthread_create(&reader, NULL, read_fusion_results, &state) == 0);
    assert(pthread_join(writer, NULL) == 0);
    assert(pthread_join(reader, NULL) == 0);
    assert(!atomic_load(&state.torn_result));
}

static imu_sample_t level_sample(uint32_t sequence, int16_t mx, int16_t my)
{
    imu_sample_t s = make_sample(sequence);
    s.gx = s.gy = s.gz = 0;
    s.mx = mx;
    s.my = my;
    return s;
}

static void test_compass_correction_gives_heading_with_no_zero_offset(void)
{
    /* Hard iron (1000, -500) and a soft iron that doubles x: the corrected
     * field for this raw reading points along +y -> 90 deg, straight from
     * the field, with no stored "zero heading" anywhere. */
    CalibrationData calib = calibration();
    calib.mag_center[0] = 1000.0f;
    calib.mag_center[1] = -500.0f;
    calib.mag_soft[0] = 2.0f;
    calib.mag_radius = 3000.0f;
    calib.mag_calibrated = 1;
    fusion_init(&calib);

    imu_sample_t s = level_sample(1, 1000, 2500);
    fusion_update_sample(&s);
    FusionResult r;
    fusion_get_result(&r);
    assert(r.heading_valid);
    assert(fabsf(r.heading - 90.0f) < 0.01f);
    assert(fabsf(fusion_get_field_ratio() - 1.0f) < 0.01f);

    /* Same raw reading with the x-offset undone: 45 deg after the 2x on x. */
    fusion_init(&calib);
    s = level_sample(1, 1000 + 1500, -500 + 3000);
    fusion_update_sample(&s);
    fusion_get_result(&r);
    assert(fabsf(r.heading - 45.0f) < 0.01f);
}

static void test_uncalibrated_compass_reports_no_field_ratio(void)
{
    CalibrationData calib = calibration();
    fusion_init(&calib);
    imu_sample_t s = level_sample(1, 3000, 0);
    fusion_update_sample(&s);
    assert(fusion_get_field_ratio() == 0.0f);
}

static void test_new_calibration_takes_over_on_the_next_sample(void)
{
    CalibrationData calib = calibration();
    fusion_init(&calib);
    imu_sample_t s = level_sample(1, 3000, 0);
    fusion_update_sample(&s);
    FusionResult r;
    fusion_get_result(&r);
    assert(fabsf(r.heading - 0.0f) < 0.01f);

    /* A calibration that moves the centre so the same raw reading now points
     * along +y.  The heading must jump to it at once, not drift there over
     * the complementary filter's seconds-long time constant. */
    CalibrationData next = calib;
    next.mag_center[0] = 3000.0f;
    next.mag_center[1] = -3000.0f;
    next.mag_radius = 3000.0f;
    next.mag_calibrated = 1;
    next.pitch_tare = 1.5f;
    assert(fusion_set_calibration(&next));
    assert(fusion_calibration_pending());
    assert(!fusion_set_calibration(&next));        /* not taken yet */

    s = level_sample(2, 3000, 0);
    fusion_update_sample(&s);
    assert(!fusion_calibration_pending());
    fusion_get_result(&r);
    assert(fabsf(r.heading - 90.0f) < 0.01f);
    assert(fabsf(fusion_get_field_ratio() - 1.0f) < 0.01f);
    assert(fusion_set_calibration(&next));         /* free again */
    s = level_sample(3, 3000, 0);
    fusion_update_sample(&s);
}

int main(void)
{
    test_fusion_uses_sample_timestamp_and_ignores_mag_only_sample();
    test_positive_left_yaw_decreases_compass_heading_prediction();
    test_result_reader_observes_only_complete_fusion_generations();
    test_compass_correction_gives_heading_with_no_zero_offset();
    test_uncalibrated_compass_reports_no_field_ratio();
    test_new_calibration_takes_over_on_the_next_sample();
    return 0;
}
