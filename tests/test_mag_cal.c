#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mag_cal.h"

#define PI_F 3.14159265f
#define DEG(r) ((r) * 180.0f / PI_F)
#define RAD(d) ((d) * PI_F / 180.0f)

static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* Deterministic noise in [-amp, amp]. */
static unsigned s_rng = 12345u;
static float noise(float amp)
{
    s_rng = s_rng * 1103515245u + 12345u;
    return amp * (((float)((s_rng >> 8) & 0xFFFF) / 32767.5f) - 1.0f);
}

typedef struct {
    float a[4];        /* distortion matrix applied to the true field */
    float off[2];      /* hard-iron offset */
    float field;       /* true horizontal field, counts */
    float noise;
} distortion_t;

/* The raw reading for a true heading psi (deg, clockwise).  heading =
 * atan2(y, x) in this firmware, so the undistorted field is (cos, sin). */
static void raw_for(const distortion_t *d, float psi_deg, float *mx, float *my)
{
    float hx = d->field * cosf(RAD(psi_deg));
    float hy = d->field * sinf(RAD(psi_deg));
    *mx = d->a[0] * hx + d->a[1] * hy + d->off[0] + noise(d->noise);
    *my = d->a[2] * hx + d->a[3] * hy + d->off[1] + noise(d->noise);
}

#define CAP 4000
static float bx[CAP], by[CAP], bt[CAP];

/* A left spin at 20 deg/s sampled at 50 Hz: gyro turn goes up, heading down. */
static void spin(mag_circle_t *c, const distortion_t *d, float turns, float start_psi)
{
    const float rate = 20.0f, dt = 0.02f;
    int steps = (int)(turns * 360.0f / (rate * dt));
    float psi = start_psi;
    for (int i = 0; i < steps; i++) {
        float mx, my;
        raw_for(d, psi, &mx, &my);
        mag_circle_add(c, mx, my, rate, dt);
        psi -= rate * dt;
    }
}

static const distortion_t SYMMETRIC = {
    .a = {1.30f, 0.20f, 0.20f, 0.80f}, .off = {1500.0f, -900.0f},
    .field = 4000.0f, .noise = 5.0f,
};

/* ---------------------------------------------------------------- */

static void test_identity_is_valid_and_passes_values_through(void)
{
    mag_cal_2d_t cal;
    mag_cal_identity(&cal);
    assert(mag_cal_valid(&cal));
    float x, y;
    mag_cal_apply(&cal, 123.0f, -45.0f, &x, &y);
    assert(fabsf(x - 123.0f) < 1e-4f && fabsf(y + 45.0f) < 1e-4f);
}

static void test_invalid_calibrations_are_rejected(void)
{
    mag_cal_2d_t cal;
    mag_cal_identity(&cal);
    cal.soft[0] = NAN;
    assert(!mag_cal_valid(&cal));
    mag_cal_identity(&cal);
    cal.soft[0] = -1.0f;                      /* mirror image: det < 0 */
    assert(!mag_cal_valid(&cal));
    mag_cal_identity(&cal);
    cal.center[1] = INFINITY;
    assert(!mag_cal_valid(&cal));
    assert(!mag_cal_valid(NULL));
}

static void test_heading_convention_and_wrap(void)
{
    assert(fabsf(mag_cal_heading_deg(1.0f, 0.0f) - 0.0f) < 1e-3f);
    assert(fabsf(mag_cal_heading_deg(0.0f, 1.0f) - 90.0f) < 1e-3f);
    assert(fabsf(mag_cal_heading_deg(0.0f, -1.0f) - 270.0f) < 1e-3f);
    float h = mag_cal_heading_deg(-1.0f, -1e-6f);
    assert(h >= 0.0f && h < 360.0f);
}

static void test_fit_removes_offset_and_squash(void)
{
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    spin(&c, &SYMMETRIC, 2.0f, 30.0f);
    assert(mag_circle_complete(&c));

    mag_fit_t fit;
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, c.n, &fit);
    printf("  symmetric: verdict=%s ratio=%.3f rms=%.4f bins=%d scale=%.4f dev=%.2f\n",
           mag_cal_verdict_text(v), fit.axis_ratio, fit.rms, fit.bins,
           fit.gyro_scale, fit.gyro_dev_deg);
    assert(v == MAG_CAL_PASS);
    assert(fabsf(fit.cal.center[0] - 1500.0f) < 20.0f);
    assert(fabsf(fit.cal.center[1] + 900.0f) < 20.0f);
    /* eigenvalues of [[1.3,0.2],[0.2,0.8]] are 1.370 and 0.730 -> 1.877 */
    assert(fabsf(fit.axis_ratio - 1.877f) < 0.03f);

    /* The point of it all: corrected heading follows the true heading. */
    float worst = 0.0f;
    distortion_t clean = SYMMETRIC;
    clean.noise = 0.0f;
    for (float psi = 0.0f; psi < 360.0f; psi += 5.0f) {
        float mx, my, x, y;
        raw_for(&clean, psi, &mx, &my);
        mag_cal_apply(&fit.cal, mx, my, &x, &y);
        float err = fabsf(wrap180(mag_cal_heading_deg(x, y) - psi));
        if (err > worst) worst = err;
        float r = sqrtf(x * x + y * y);
        assert(fabsf(r / fit.cal.radius - 1.0f) < 0.02f);
    }
    printf("  symmetric: worst corrected heading error %.3f deg\n", worst);
    assert(worst < 1.0f);
}

static void test_uncorrected_data_really_is_wrong(void)
{
    /* Guards the test itself: without correction this distortion must be
     * visibly wrong, or the test above proves nothing. */
    distortion_t clean = SYMMETRIC;
    clean.noise = 0.0f;
    float worst = 0.0f;
    for (float psi = 0.0f; psi < 360.0f; psi += 5.0f) {
        float mx, my;
        raw_for(&clean, psi, &mx, &my);
        float err = fabsf(wrap180(mag_cal_heading_deg(mx, my) - psi));
        if (err > worst) worst = err;
    }
    assert(worst > 20.0f);
}

static void test_rotated_squash_leaves_only_a_constant_offset(void)
{
    /* A non-symmetric distortion hides a rotation no circle fit can see.
     * What remains must be one constant offset, not a heading-dependent
     * error -- that is what a mounting offset is. */
    distortion_t d = {.a = {1.2f, 0.35f, -0.05f, 0.9f}, .off = {-600.0f, 2200.0f},
                      .field = 4000.0f, .noise = 0.0f};
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    spin(&c, &d, 2.0f, 0.0f);
    mag_fit_t fit;
    assert(mag_cal_fit_and_judge(bx, by, bt, c.n, &fit) == MAG_CAL_PASS);
    float e0 = 0.0f, lo = 1e9f, hi = -1e9f;
    for (float psi = 0.0f; psi < 360.0f; psi += 5.0f) {
        float mx, my, x, y;
        raw_for(&d, psi, &mx, &my);
        mag_cal_apply(&fit.cal, mx, my, &x, &y);
        float err = wrap180(mag_cal_heading_deg(x, y) - psi);
        if (psi == 0.0f) e0 = err;
        float rel = wrap180(err - e0);
        if (rel < lo) lo = rel;
        if (rel > hi) hi = rel;
    }
    printf("  rotated: offset %.2f deg, spread %.3f deg\n", e0, hi - lo);
    assert(hi - lo < 1.0f);
}

static void test_gyro_disagreement_fails_the_calibration(void)
{
    /* Compass data from a 2-turn spin, but the gyro claims 3 turns: the
     * scale gate must refuse it. */
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    spin(&c, &SYMMETRIC, 2.0f, 0.0f);
    for (int i = 0; i < c.n; i++) bt[i] *= 1.5f;
    mag_fit_t fit;
    assert(mag_cal_fit_and_judge(bx, by, bt, c.n, &fit) == MAG_CAL_FAIL_GYRO);
    assert(fit.gyro_scale < MAG_CAL_GYRO_SCALE_MIN);
}

static void test_missing_part_of_the_circle_is_not_complete(void)
{
    /* 300 degrees of a very slow partial turn: the gyro total may be large
     * enough, but a slice of the circle was never seen. */
    distortion_t d = SYMMETRIC;
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    for (int rep = 0; rep < 3; rep++) {
        for (float psi = 0.0f; psi < 300.0f; psi += 0.4f) {
            float mx, my;
            raw_for(&d, psi, &mx, &my);
            mag_circle_add(&c, mx, my, 20.0f, 0.02f);
        }
    }
    assert(c.turn_deg >= MAG_CAL_MIN_TURN_DEG);
    assert(!mag_circle_complete(&c));
    mag_fit_t fit;
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, c.n, &fit);
    assert(v != MAG_CAL_PASS);
}

static void test_less_than_one_and_a_half_turns_is_not_complete(void)
{
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    spin(&c, &SYMMETRIC, 1.2f, 0.0f);
    assert(__builtin_popcount(mag_circle_live_mask(&c)) >= 30);
    assert(!mag_circle_complete(&c));
    spin(&c, &SYMMETRIC, 0.5f, 0.0f);
    assert(mag_circle_complete(&c));
}

static void test_scattered_readings_fail_as_noisy(void)
{
    distortion_t d = SYMMETRIC;
    d.noise = 700.0f;
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    spin(&c, &d, 2.0f, 0.0f);
    mag_fit_t fit;
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, c.n, &fit);
    printf("  noisy: verdict=%s rms=%.3f\n", mag_cal_verdict_text(v), fit.rms);
    assert(v == MAG_CAL_FAIL_NOISY || v == MAG_CAL_FAIL_GYRO);
}

static void test_pulsing_field_strength_fails_as_noisy(void)
{
    /* Direction is exact, only the strength pulses (e.g. a current-carrying
     * wire next to the compass): the gyro agrees, the noise gate alone must
     * catch it. */
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    float psi = 0.0f;
    for (int i = 0; i < 1800; i++) {
        float mx, my;
        distortion_t d = SYMMETRIC;
        d.noise = 0.0f;
        d.field = 4000.0f * (1.0f + 0.15f * sinf((float)i * 1.3f));
        raw_for(&d, psi, &mx, &my);
        mag_circle_add(&c, mx, my, 20.0f, 0.02f);
        psi -= 0.4f;
    }
    mag_fit_t fit;
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, c.n, &fit);
    printf("  pulsing: verdict=%s rms=%.3f dev=%.2f\n", mag_cal_verdict_text(v),
           fit.rms, fit.gyro_dev_deg);
    assert(v == MAG_CAL_FAIL_NOISY);
}

static void test_single_partial_sweep_fails_as_coverage(void)
{
    /* One clean 300-degree sweep, gyro consistent: only coverage is short. */
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    float psi = 0.0f;
    for (int i = 0; i < 750; i++) {
        float mx, my;
        raw_for(&SYMMETRIC, psi, &mx, &my);
        mag_circle_add(&c, mx, my, 20.0f, 0.02f);
        psi -= 0.4f;
    }
    mag_fit_t fit;
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, c.n, &fit);
    printf("  partial: verdict=%s bins=%d\n", mag_cal_verdict_text(v), fit.bins);
    assert(v == MAG_CAL_FAIL_COVERAGE);
}

static void test_heading_error_the_ellipse_cannot_fix_fails(void)
{
    /* Compass error that swings with heading but is not an ellipse (e.g.
     * something moved during the spin): the overall scale still matches,
     * the per-heading disagreement gate alone must catch it. */
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    float psi = 0.0f;
    for (int i = 0; i < 1800; i++) {
        float mx, my;
        raw_for(&SYMMETRIC, psi, &mx, &my);
        float wobble = 1.0f + 0.3f * sinf(2.0f * RAD(psi));
        mag_circle_add(&c, mx, my, 20.0f * wobble, 0.02f);
        psi -= 0.4f;
    }
    mag_fit_t fit;
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, c.n, &fit);
    printf("  wobble: verdict=%s scale=%.3f dev=%.2f\n", mag_cal_verdict_text(v),
           fit.gyro_scale, fit.gyro_dev_deg);
    assert(v == MAG_CAL_FAIL_GYRO);
    assert(fit.gyro_scale > MAG_CAL_GYRO_SCALE_MIN && fit.gyro_scale < MAG_CAL_GYRO_SCALE_MAX);
}

static void test_heavily_squashed_field_fails_as_distorted(void)
{
    distortion_t d = {.a = {1.6f, 0.0f, 0.0f, 0.6f}, .off = {0.0f, 0.0f},
                      .field = 4000.0f, .noise = 2.0f};
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    spin(&c, &d, 2.0f, 0.0f);
    mag_fit_t fit;
    assert(mag_cal_fit_and_judge(bx, by, bt, c.n, &fit) == MAG_CAL_FAIL_DISTORTED);
}

static void test_weak_field_fails_as_not_earth_like(void)
{
    distortion_t d = {.a = {1.0f, 0.0f, 0.0f, 1.0f}, .off = {100.0f, 50.0f},
                      .field = 300.0f, .noise = 1.0f};
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    spin(&c, &d, 2.0f, 0.0f);
    mag_fit_t fit;
    assert(mag_cal_fit_and_judge(bx, by, bt, c.n, &fit) == MAG_CAL_FAIL_FIELD);
}

static void test_degenerate_data_fails_without_crashing(void)
{
    for (int i = 0; i < 300; i++) { bx[i] = 100.0f; by[i] = 200.0f; bt[i] = (float)i; }
    mag_fit_t fit;
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, 300, &fit);
    assert(v == MAG_CAL_FAIL_FIT);
    for (int i = 0; i < 300; i++) { bx[i] = (float)i; by[i] = 2.0f * (float)i; }
    v = mag_cal_fit_and_judge(bx, by, bt, 300, &fit);
    assert(v == MAG_CAL_FAIL_FIT);
    assert(mag_cal_fit_and_judge(bx, by, bt, 10, &fit) == MAG_CAL_FAIL_TOO_FEW);
}

static void test_missing_compass_readings_still_count_the_turn(void)
{
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    mag_circle_add(&c, 100.0f, 0.0f, 20.0f, 0.02f);
    mag_circle_add(&c, NAN, NAN, 20.0f, 0.02f);        /* compass read failed */
    mag_circle_add(&c, 100.0f, 1.0f, 20.0f, 0.02f);
    assert(c.n == 2);
    assert(fabsf(c.turn_deg - 1.2f) < 1e-4f);
    assert(fabsf(bt[1] - 1.2f) < 1e-4f);
}

static void test_buffer_thinning_keeps_order_and_capacity(void)
{
    enum { SMALL = 256 };
    static float x[SMALL], y[SMALL], t[SMALL];
    mag_circle_t c;
    mag_circle_init(&c, x, y, t, SMALL);
    spin(&c, &SYMMETRIC, 2.0f, 0.0f);      /* 1800 samples into 256 slots */
    assert(c.n <= SMALL && c.n > SMALL / 2);
    for (int i = 1; i < c.n; i++) assert(t[i] > t[i - 1]);   /* time order kept */
    assert(fabsf(c.turn_deg - 720.0f) < 1.0f);
    mag_fit_t fit;
    assert(mag_cal_fit_and_judge(x, y, t, c.n, &fit) == MAG_CAL_PASS);
}

static void test_uneven_hand_spin_with_pause_and_gyro_drift_passes(void)
{
    /* A person spinning a boat by hand: speed wanders 5..35 deg/s, one 5 s
     * pause half-way, and 0.05 deg/s of gyro drift left over after the
     * hold-still step.  A good calibration must still pass. */
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    const float dt = 0.02f, drift = 0.05f;
    float psi = 45.0f, t = 0.0f, turned = 0.0f;
    while (turned < 2.0f * 360.0f) {
        float rate = (t > 20.0f && t < 25.0f) ? 0.0f : 20.0f + 15.0f * sinf(t * 0.7f);
        float mx, my;
        raw_for(&SYMMETRIC, psi, &mx, &my);
        mag_circle_add(&c, mx, my, rate + drift, dt);
        psi -= rate * dt;
        turned += rate * dt;
        t += dt;
    }
    assert(mag_circle_complete(&c));
    mag_fit_t fit;
    mag_cal_verdict_t v = mag_cal_fit_and_judge(bx, by, bt, c.n, &fit);
    printf("  hand spin: %.0f s, verdict=%s scale=%.4f dev=%.2f deg\n",
           t, mag_cal_verdict_text(v), fit.gyro_scale, fit.gyro_dev_deg);
    assert(v == MAG_CAL_PASS);
}

static void test_right_hand_spin_passes_too(void)
{
    /* Spinning the other way (clockwise): gyro negative, heading rising. */
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    float psi = 0.0f;
    for (int i = 0; i < 1800; i++) {
        float mx, my;
        raw_for(&SYMMETRIC, psi, &mx, &my);
        mag_circle_add(&c, mx, my, -20.0f, 0.02f);
        psi += 0.4f;
    }
    assert(mag_circle_complete(&c));
    mag_fit_t fit;
    assert(mag_cal_fit_and_judge(bx, by, bt, c.n, &fit) == MAG_CAL_PASS);
    assert(fabsf(fit.gyro_scale - 1.0f) < 0.02f);
}

static void test_heading_that_runs_backwards_fails(void)
{
    /* If the compass and gyro disagree on DIRECTION (a mirrored axis), the
     * calibration must not pass: that would make the heading hold steer
     * the wrong way. */
    mag_circle_t c;
    mag_circle_init(&c, bx, by, bt, CAP);
    float psi = 0.0f;
    for (int i = 0; i < 1800; i++) {
        float mx, my;
        raw_for(&SYMMETRIC, psi, &mx, &my);
        mag_circle_add(&c, mx, my, 20.0f, 0.02f);   /* gyro says left... */
        psi += 0.4f;                                /* ...compass says right */
    }
    mag_fit_t fit;
    assert(mag_cal_fit_and_judge(bx, by, bt, c.n, &fit) == MAG_CAL_FAIL_GYRO);
    assert(fit.gyro_scale < 0.0f);
}

static void test_tilt_angle_between_accelerometer_readings(void)
{
    float ref[3] = {0.0f, 0.0f, 16384.0f};
    float same[3] = {0.0f, 0.0f, 16000.0f};                 /* length differs, angle 0 */
    assert(mag_cal_tilt_deg(same, ref) < 0.01f);
    float t5[3] = {16384.0f * sinf(RAD(5.0f)), 0.0f, 16384.0f * cosf(RAD(5.0f))};
    assert(fabsf(mag_cal_tilt_deg(t5, ref) - 5.0f) < 0.01f);
    float t3[3] = {0.0f, -16384.0f * sinf(RAD(3.0f)), 16384.0f * cosf(RAD(3.0f))};
    assert(fabsf(mag_cal_tilt_deg(t3, ref) - 3.0f) < 0.01f);
    float zero[3] = {0.0f, 0.0f, 0.0f};
    assert(mag_cal_tilt_deg(zero, ref) == 180.0f);
    /* a level-but-not-flat mounting: reference itself tilted, boat still level */
    float mount[3] = {1000.0f, -500.0f, 16300.0f};
    assert(mag_cal_tilt_deg(mount, mount) < 0.01f);
}

/* ---- stillness ---- */

static void feed_still(mag_still_t *s, int n, float gyro_noise, float accel_noise)
{
    for (int i = 0; i < n; i++) {
        int16_t a[3] = {(int16_t)(120 + noise(accel_noise)),
                        (int16_t)(-80 + noise(accel_noise)),
                        (int16_t)(16384 + noise(accel_noise))};
        int16_t g[3] = {(int16_t)(35 + noise(gyro_noise)),
                        (int16_t)(-12 + noise(gyro_noise)),
                        (int16_t)(7 + noise(gyro_noise))};
        mag_still_add(s, a, g);
    }
}

static void test_still_board_measures_gyro_bias_and_level(void)
{
    mag_still_t s;
    mag_still_init(&s, 50, 150, 60.0f, 40.0f);
    feed_still(&s, 149, 10.0f, 20.0f);
    assert(s.state != MAG_STILL_DONE);
    feed_still(&s, 1, 10.0f, 20.0f);
    assert(s.state == MAG_STILL_DONE);
    float g[3], a[3];
    mag_still_result(&s, g, a);
    assert(fabsf(g[0] - 35.0f) < 3.0f && fabsf(g[1] + 12.0f) < 3.0f && fabsf(g[2] - 7.0f) < 3.0f);
    assert(fabsf(a[2] - 16384.0f) < 10.0f);
    assert(mag_still_progress(&s) == 1.0f);
}

static void test_moving_board_never_finishes(void)
{
    mag_still_t s;
    mag_still_init(&s, 50, 150, 60.0f, 40.0f);
    feed_still(&s, 2000, 800.0f, 20.0f);        /* gyro swinging */
    assert(s.state == MAG_STILL_WAITING);
    assert(mag_still_progress(&s) == 0.0f);
}

static void test_a_bump_restarts_the_collection(void)
{
    mag_still_t s;
    mag_still_init(&s, 50, 150, 60.0f, 40.0f);
    feed_still(&s, 100, 10.0f, 20.0f);
    assert(s.state == MAG_STILL_COLLECTING);
    assert(mag_still_progress(&s) > 0.5f);
    feed_still(&s, 50, 10.0f, 3000.0f);          /* bumped */
    assert(s.state == MAG_STILL_WAITING);
    assert(mag_still_progress(&s) == 0.0f);
    feed_still(&s, 150, 10.0f, 20.0f);
    assert(s.state == MAG_STILL_DONE);
}

int main(void)
{
    test_identity_is_valid_and_passes_values_through();
    test_invalid_calibrations_are_rejected();
    test_heading_convention_and_wrap();
    test_fit_removes_offset_and_squash();
    test_uncorrected_data_really_is_wrong();
    test_rotated_squash_leaves_only_a_constant_offset();
    test_gyro_disagreement_fails_the_calibration();
    test_missing_part_of_the_circle_is_not_complete();
    test_less_than_one_and_a_half_turns_is_not_complete();
    test_scattered_readings_fail_as_noisy();
    test_pulsing_field_strength_fails_as_noisy();
    test_single_partial_sweep_fails_as_coverage();
    test_heading_error_the_ellipse_cannot_fix_fails();
    test_heavily_squashed_field_fails_as_distorted();
    test_weak_field_fails_as_not_earth_like();
    test_degenerate_data_fails_without_crashing();
    test_missing_compass_readings_still_count_the_turn();
    test_buffer_thinning_keeps_order_and_capacity();
    test_uneven_hand_spin_with_pause_and_gyro_drift_passes();
    test_right_hand_spin_passes_too();
    test_heading_that_runs_backwards_fails();
    test_tilt_angle_between_accelerometer_readings();
    test_still_board_measures_gyro_bias_and_level();
    test_moving_board_never_finishes();
    test_a_bump_restarts_the_collection();
    printf("mag_cal: all tests passed\n");
    return 0;
}
