#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "trim_assist.h"

static trim_assist_cfg_t cfg(void)
{
    /* the flashed candidates: kp is ~32% of unity loop gain (plant is
     * 16.1 deg/s per unit c), tau is half the measured 1.10 s hull constant,
     * cap moves each motor at most 1.2 percentage points at T40 */
    trim_assist_cfg_t c = { .kp = 0.020f, .tau_s = 0.50f, .cap = 0.030f };
    return c;
}
static int near(float a, float b) { return fabsf(a - b) < 1e-6f; }

/* Settle the filter on a steady yaw and return the correction. */
static float settle(trim_assist_t *s, const trim_assist_cfg_t *c, float yaw)
{
    float out = 0.0f;
    for (int i = 0; i < 400; ++i) out = trim_assist_update(s, c, 0.02f, yaw, true);
    return out;
}

/* NEGATIVE yaw is what too-little-trim reads on this boat (221 recorded runs),
 * so it must RAISE c, which gives the RIGHT motor more. Positive yaw lowers
 * it. Getting this backwards drives the boat the wrong way at twice the
 * strength, so both signs are pinned. */
static void both_yaw_signs_push_the_right_way(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t s;

    trim_assist_reset(&s);
    float neg = settle(&s, &c, -1.0f);
    assert(neg > 0.0f);
    assert(near(neg, 0.020f));            /* kp * 1.0, under the cap */

    trim_assist_reset(&s);
    float pos = settle(&s, &c, +1.0f);
    assert(pos < 0.0f);
    assert(near(pos, -0.020f));

    /* and it is odd-symmetric: same size, opposite sign */
    assert(near(neg, -pos));
}

static void zero_yaw_does_nothing(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t s;
    trim_assist_reset(&s);
    assert(near(settle(&s, &c, 0.0f), 0.0f));
    assert(!s.at_cap);
}

/* The cap is the safety bound: whatever the gyro says, the correction may not
 * move each motor by more than ~1.2 percentage points at T40. */
static void the_cap_binds_in_both_directions(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t s;

    trim_assist_reset(&s);
    float big_neg = settle(&s, &c, -9.0f);   /* kp*9 = 0.18, way over cap */
    assert(near(big_neg, c.cap));
    assert(s.at_cap);

    trim_assist_reset(&s);
    float big_pos = settle(&s, &c, +9.0f);
    assert(near(big_pos, -c.cap));
    assert(s.at_cap);

    /* just under the cap is not flagged */
    trim_assist_reset(&s);
    (void)settle(&s, &c, -1.0f);
    assert(!s.at_cap);
}

/* effective_c must stay inside the learner's own bounds even when the learned
 * value is already against one of them -- the assist may never push c
 * somewhere the learner itself is forbidden to go. */
static void the_total_stays_inside_the_learner_bounds(void)
{
    assert(near(trim_assist_effective_c(0.170f, 0.030f, 0.10f, 0.35f), 0.200f));
    assert(near(trim_assist_effective_c(0.170f, -0.030f, 0.10f, 0.35f), 0.140f));
    /* learned c already at a bound: the assist cannot push past it */
    assert(near(trim_assist_effective_c(0.350f, 0.030f, 0.10f, 0.35f), 0.350f));
    assert(near(trim_assist_effective_c(0.100f, -0.030f, 0.10f, 0.35f), 0.100f));
    /* a non-finite learned value must not propagate into the mixer */
    assert(near(trim_assist_effective_c(NAN, 0.010f, 0.10f, 0.35f), 0.0f));
    assert(near(trim_assist_effective_c(0.170f, NAN, 0.10f, 0.35f), 0.170f));
}

/* NaN survives every clamp (all comparisons with NaN are false), so it has to
 * be caught here or it reaches a real ESC. */
static void non_finite_input_resets_rather_than_propagates(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t s;
    trim_assist_reset(&s);
    (void)settle(&s, &c, -1.0f);
    assert(s.correction > 0.0f);

    assert(near(trim_assist_update(&s, &c, 0.02f, NAN, true), 0.0f));
    assert(!s.initialized && near(s.correction, 0.0f));

    (void)settle(&s, &c, -1.0f);
    assert(near(trim_assist_update(&s, &c, NAN, -1.0f, true), 0.0f));
    assert(!s.initialized);

    (void)settle(&s, &c, -1.0f);
    assert(near(trim_assist_update(&s, &c, 0.02f, INFINITY, true), 0.0f));
    assert(!s.initialized);
}

/* Every gate the caller can close -- disabled, stale gyro, low throttle,
 * steering, impact, calibration owning the motors -- arrives here as the same
 * thing: gate false. It must RESET, not decay, so that "off" and "gated off"
 * are one state and the control arm of an A/B is exactly the pre-P path. */
static void a_closed_gate_resets_and_never_decays(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t s;
    trim_assist_reset(&s);
    float held = settle(&s, &c, -1.0f);
    assert(held > 0.0f);

    assert(near(trim_assist_update(&s, &c, 0.02f, -1.0f, false), 0.0f));
    assert(near(s.correction, 0.0f));
    assert(!s.initialized);              /* the estimate is gone, not paused */
    assert(!s.at_cap);

    /* and it stays zero however long the gate is shut */
    for (int i = 0; i < 200; ++i) {
        assert(near(trim_assist_update(&s, &c, 0.02f, -5.0f, false), 0.0f));
    }
    /* re-opening starts from nothing: the first sample only resyncs */
    assert(near(trim_assist_update(&s, &c, 0.02f, -1.0f, true), 0.0f));
    assert(settle(&s, &c, -1.0f) > 0.0f);
}

/* An explicit reset must leave no trace -- this is what makes the correction
 * temporary by construction rather than by convention. */
static void reset_clears_everything(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t s;
    trim_assist_reset(&s);
    (void)settle(&s, &c, -9.0f);
    assert(s.at_cap && s.correction > 0.0f && s.initialized);

    trim_assist_reset(&s);
    assert(near(s.correction, 0.0f));
    assert(near(s.yaw_filt, 0.0f));
    assert(!s.initialized);
    assert(!s.at_cap);
}

/* kp = 0 is the OFF path expressed in config: identical to gate-false. */
static void zero_gain_is_the_off_path(void)
{
    trim_assist_cfg_t c = cfg(); c.kp = 0.0f;
    trim_assist_t s; trim_assist_reset(&s);
    assert(near(settle(&s, &c, -5.0f), 0.0f));
    trim_assist_cfg_t z = cfg(); z.cap = 0.0f;
    trim_assist_reset(&s);
    assert(near(settle(&s, &z, -5.0f), 0.0f));
}

/* A null config is treated as "no assist", not as a crash. */
static void null_inputs_are_safe(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t s;
    trim_assist_reset(&s);
    assert(near(trim_assist_update(&s, NULL, 0.02f, -1.0f, true), 0.0f));
    assert(near(trim_assist_update(NULL, &c, 0.02f, -1.0f, true), 0.0f));
    trim_assist_reset(NULL);
}

/* The filter must be dt-aware: the same elapsed time reaches the same place
 * whatever the sample spacing, or a scheduling hiccup changes the gain. */
static void the_filter_is_dt_aware(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t a, b;
    trim_assist_reset(&a); trim_assist_reset(&b);
    for (int i = 0; i < 100; ++i) trim_assist_update(&a, &c, 0.02f, -1.0f, true);
    for (int i = 0; i < 50; ++i)  trim_assist_update(&b, &c, 0.04f, -1.0f, true);
    assert(fabsf(a.yaw_filt - b.yaw_filt) < 0.01f);
}

/* OFF is the CONTROL ARM of the experiment. It must be bit-identical to the
 * boat before P existed -- not "a P that outputs small numbers". If OFF and
 * the pre-P path differ by even one ulp, the A arm is not a control and the
 * whole A/B measures nothing. */
static void off_is_bit_identical_to_the_pre_p_path(void)
{
    trim_assist_cfg_t c = cfg(); trim_assist_t s;
    trim_assist_reset(&s);

    /* whatever the gyro is doing, a closed gate contributes EXACTLY zero */
    const float yaws[] = {0.0f, -0.001f, -1.0f, +1.0f, -9.0f, +50.0f, -0.5f};
    for (unsigned i = 0; i < sizeof yaws / sizeof yaws[0]; ++i) {
        float out = trim_assist_update(&s, &c, 0.02f, yaws[i], false);
        assert(out == 0.0f);                     /* exactly, not approximately */
    }

    /* ...and zero added to the learned c returns that c UNCHANGED, bit for
     * bit, across the whole legal range */
    for (int i = 0; i <= 250; ++i) {
        float learned = 0.10f + (float)i * 0.001f;
        float eff = trim_assist_effective_c(learned, 0.0f, 0.10f, 0.35f);
        assert(eff == learned);
    }

    /* even a gate that closes MID-RUN, after P had built a real correction,
     * lands back on exactly the learned value */
    trim_assist_reset(&s);
    for (int i = 0; i < 400; ++i) trim_assist_update(&s, &c, 0.02f, -3.0f, true);
    assert(s.correction != 0.0f);
    assert(trim_assist_update(&s, &c, 0.02f, -3.0f, false) == 0.0f);
    assert(trim_assist_effective_c(0.1734f, 0.0f, 0.10f, 0.35f) == 0.1734f);
}

int main(void)
{
    both_yaw_signs_push_the_right_way();
    zero_yaw_does_nothing();
    the_cap_binds_in_both_directions();
    the_total_stays_inside_the_learner_bounds();
    non_finite_input_resets_rather_than_propagates();
    a_closed_gate_resets_and_never_decays();
    reset_clears_everything();
    zero_gain_is_the_off_path();
    null_inputs_are_safe();
    the_filter_is_dt_aware();
    off_is_bit_identical_to_the_pre_p_path();
    printf("test_trim_assist: OK\n");
    return 0;
}
