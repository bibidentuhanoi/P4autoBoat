#include <assert.h>
#include <math.h>
#include <string.h>
#include "trim_learn.h"

static trim_learn_cfg_t cfg(void)
{
    trim_learn_cfg_t c = {
        .c_init = 0.20f, .c_min = 0.10f, .c_max = 0.35f,
        .deadband_dps = 0.5f, .step_per_s = 0.005f,
        .yaw_tau_s = 2.0f, .min_throttle = 0.15f,
    };
    return c;
}

/* Feed the same drift repeatedly at 50 Hz, incrementing the sample sequence. */
static void feed(trim_learn_t *s, const trim_learn_cfg_t *c, float yaw,
                 float throttle, bool steering, int ticks)
{
    for (int i = 0; i < ticks; ++i) {
        (void)trim_learn_update(s, c, s->last_seq + 1, 0.02f, yaw,
                                throttle, steering, true);
    }
}

/* SIGN. Measured on hardware: with no trim the boat read -4.34 deg/s, and
 * applying a POSITIVE trim (+0.152 through esc_trim_apply_pair) straightened
 * it. So a negative drift means "not enough trim" -> c must go UP. Backwards
 * here drives the boat the wrong way, harder and harder. */
static void negative_drift_raises_c(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    feed(&s, &c, -3.0f, 0.40f, false, 250);      /* 5 s of drift */
    assert(s.c > 0.20f);
}

static void positive_drift_lowers_c(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    feed(&s, &c, +3.0f, 0.40f, false, 250);
    assert(s.c < 0.20f);
}

/* Once straight, noise must not walk c around. */
static void small_drift_inside_the_deadband_changes_nothing(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    feed(&s, &c, +0.2f, 0.40f, false, 500);      /* 10 s below deadband */
    assert(fabsf(s.c - 0.20f) < 1e-6f);
}

/* A commanded turn is not an error -- learning from it would poison c. */
static void steering_freezes_learning(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    feed(&s, &c, -5.0f, 0.40f, true, 500);
    assert(fabsf(s.c - 0.20f) < 1e-6f);
}

/* Below the floor-dominated region the proportional model is not justified. */
static void low_throttle_freezes_learning(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    feed(&s, &c, -5.0f, 0.05f, false, 500);
    assert(fabsf(s.c - 0.20f) < 1e-6f);
}

/* The control loop is 100 Hz but fusion only updates at 50 Hz. Re-feeding the
 * same sequence must be ignored, or every sample is integrated twice. */
static void a_repeated_sample_is_ignored(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    (void)trim_learn_update(&s, &c, 7, 0.02f, -5.0f, 0.40f, false, true);
    float after_first = s.c;
    for (int i = 0; i < 50; ++i) {
        (void)trim_learn_update(&s, &c, 7, 0.02f, -5.0f, 0.40f, false, true);
    }
    assert(fabsf(s.c - after_first) < 1e-9f);
}

/* Hitting the clamp is a FAULT (wrong sign, blocked jet, bad gyro), not just a
 * limit: latch it and stop adapting rather than sitting on the rail. */
static void clamp_latches_a_fault_and_stops(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    feed(&s, &c, -20.0f, 0.40f, false, 5000);    /* drive it to the rail */
    assert(s.c <= c.c_max + 1e-6f);
    assert(s.faulted);
    float held = s.c;
    feed(&s, &c, -20.0f, 0.40f, false, 500);
    assert(fabsf(s.c - held) < 1e-9f);           /* stays put once faulted */
}

static void unhealthy_gyro_freezes_learning(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    for (int i = 0; i < 500; ++i) {
        (void)trim_learn_update(&s, &c, s.last_seq + 1, 0.02f, -5.0f,
                                0.40f, false, false);
    }
    assert(fabsf(s.c - 0.20f) < 1e-6f);
}

static void nonfinite_input_is_rejected(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    (void)trim_learn_update(&s, &c, 1, 0.02f, NAN, 0.40f, false, true);
    assert(fabsf(s.c - 0.20f) < 1e-6f);
    assert(isfinite(s.c));
}

/* The caller must be told when c moved, because the existing drive path only
 * refreshes the ESCs on a new pilot command. */
static void update_reports_whether_c_changed(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);

    /* The FIRST sample only resyncs the filter. One raw gyro reading is the
     * lone outlier this whole design exists to ignore -- snapping the filter
     * onto it and integrating it in the same breath would hand a single
     * sample the authority the filter is there to deny it. */
    assert(!trim_learn_update(&s, &c, 1, 0.02f, -5.0f, 0.40f, false, true));
    assert(fabsf(s.c - 0.20f) < 1e-9f);

    /* Sustained drift then moves it, and says so. */
    bool moved = false;
    for (uint32_t seq = 2; seq <= 6; ++seq) {
        moved |= trim_learn_update(&s, &c, seq, 0.02f, -5.0f, 0.40f, false, true);
    }
    assert(moved);

    trim_learn_t s2;                    /* a straight boat never reports a move */
    trim_learn_init(&s2, &c);
    for (uint32_t seq = 1; seq <= 20; ++seq) {
        assert(!trim_learn_update(&s2, &c, seq, 0.02f, 0.0f, 0.40f, false, true));
    }
}

/* The learner is frozen through a bench run and through an ESC calibration. If
 * that gap counted as integration time, the first sample afterwards would move
 * c by step_per_s * gap -- 0.0225 after a 4.5 s bench run, 0.15 after a 30 s
 * calibration, straight into the clamp and latched faulted. */
static void a_long_freeze_is_not_a_giant_correction(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    uint32_t seq = 1;
    for (int i = 0; i < 200; ++i)
        trim_learn_update(&s, &c, seq++, 0.02f, -6.0f, 0.30f, false, true);
    assert(s.c > 0.20f && !s.faulted);
    const float before = s.c;

    assert(!trim_learn_update(&s, &c, seq++, 30.0f, -6.0f, 0.30f, false, true));
    assert(s.c == before);                  /* not one step */
    assert(!s.faulted);

    for (int i = 0; i < 100; ++i)           /* and it picks straight back up */
        trim_learn_update(&s, &c, seq++, 0.02f, -6.0f, 0.30f, false, true);
    assert(s.c > before);

    /* the boundary: just inside integrates, just past it resyncs */
    trim_learn_t a, b;
    trim_learn_init(&a, &c);
    trim_learn_init(&b, &c);
    uint32_t sa = 1;
    for (int i = 0; i < 200; ++i, ++sa) {
        trim_learn_update(&a, &c, sa, 0.02f, -6.0f, 0.30f, false, true);
        trim_learn_update(&b, &c, sa, 0.02f, -6.0f, 0.30f, false, true);
    }
    const float ca = a.c, cb = b.c;
    trim_learn_update(&a, &c, sa, TRIM_LEARN_MAX_DT_S * 0.9f, -6.0f, 0.30f, false, true);
    trim_learn_update(&b, &c, sa, TRIM_LEARN_MAX_DT_S * 1.1f, -6.0f, 0.30f, false, true);
    assert(a.c > ca);
    assert(b.c == cb);
}

/* In a 1.2 m pool the hull reaches the wall in a couple of seconds and then
 * bounces. Across 63 recorded runs, 12 carried a collision (peaks 10-33 deg/s)
 * while the 51 clean ones peaked at a median of 6.8. An impact is 5-30x the
 * imbalance being measured and lasts seconds -- filtered in, it would drag the
 * estimate through the deadband and walk c somewhere arbitrary. */
static void an_impact_is_rejected_not_learned_from(void)
{
    trim_learn_cfg_t c = cfg(); c.reject_dps = 10.0f;
    trim_learn_t s;
    trim_learn_init(&s, &c);
    uint32_t seq = 1;

    /* settle on a real, small drift */
    for (int i = 0; i < 200; ++i)
        trim_learn_update(&s, &c, seq++, 0.02f, -3.0f, 0.30f, false, true);
    const float before = s.c;
    assert(before > 0.20f);

    /* 3 s of wall bounce: big, sustained, alternating -- must move nothing */
    for (int i = 0; i < 150; ++i) {
        float bounce = (i % 2) ? 24.0f : -19.0f;
        assert(!trim_learn_update(&s, &c, seq++, 0.02f, bounce, 0.30f, false, true));
    }
    assert(s.c == before);
    assert(!s.faulted);

    /* the estimate is dropped, not merely paused: the first sample after the
     * impact resyncs rather than resuming a filter full of bounce */
    assert(!s.initialized);
    assert(!trim_learn_update(&s, &c, seq++, 0.02f, -3.0f, 0.30f, false, true));
    assert(s.c == before);

    /* ...and normal drift is learned from again straight after */
    for (int i = 0; i < 200; ++i)
        trim_learn_update(&s, &c, seq++, 0.02f, -3.0f, 0.30f, false, true);
    assert(s.c > before);

    /* just under the threshold is still a measurement, not an impact */
    trim_learn_t t;
    trim_learn_init(&t, &c);
    uint32_t q = 1;
    for (int i = 0; i < 300; ++i)
        trim_learn_update(&t, &c, q++, 0.02f, -9.5f, 0.30f, false, true);
    assert(t.c > 0.20f);
}

/* The reset exists for ONE experiment: start low, start high, and see whether
 * both ends walk to the same c. It must put the learner at exactly the asked-for
 * value and wipe what it thought it knew -- a yaw estimate built under a
 * different trim is worse than no estimate, and a latched clamp fault would
 * make the whole run a no-op. */
static void reset_sets_c_and_wipes_what_it_knew(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);

    /* drive it into a latched clamp fault with a filter full of history */
    uint32_t seq = 1;
    for (int i = 0; i < 4000; ++i)
        trim_learn_update(&s, &c, seq++, 0.02f, -8.0f, 0.40f, false, true);
    assert(s.faulted);
    assert(s.initialized);
    assert(s.c >= c.c_max - 1e-6f);

    assert(trim_learn_reset(&s, &c, 0.12f));
    assert(fabsf(s.c - 0.12f) < 1e-9f);      /* exactly what was asked for */
    assert(!s.faulted);                       /* latch cleared */
    assert(!s.initialized);                   /* estimate dropped */

    /* and it learns again afterwards, from the new starting point */
    for (int i = 0; i < 400; ++i)
        trim_learn_update(&s, &c, seq++, 0.02f, -6.0f, 0.30f, false, true);
    assert(s.c > 0.12f);
}

/* An out-of-range value must change NOTHING. Clamping it to the edge would
 * start the experiment from a c the operator never chose, and silently. */
static void reset_refuses_values_outside_the_clamp(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    uint32_t seq = 1;
    for (int i = 0; i < 300; ++i)
        trim_learn_update(&s, &c, seq++, 0.02f, -6.0f, 0.30f, false, true);
    const float before = s.c;
    const bool init_before = s.initialized;

    assert(!trim_learn_reset(&s, &c, c.c_min - 0.001f));
    assert(!trim_learn_reset(&s, &c, c.c_max + 0.001f));
    assert(!trim_learn_reset(&s, &c, -0.20f));
    assert(!trim_learn_reset(&s, &c, NAN));
    assert(!trim_learn_reset(&s, &c, INFINITY));
    assert(!trim_learn_reset(NULL, &c, 0.20f));
    assert(!trim_learn_reset(&s, NULL, 0.20f));

    assert(s.c == before);                    /* nothing moved */
    assert(s.initialized == init_before);     /* nothing wiped */

    /* the bounds themselves ARE accepted -- refusing them would make the
     * clamp unreachable by an operator deliberately probing the edge */
    assert(trim_learn_reset(&s, &c, c.c_min));
    assert(trim_learn_reset(&s, &c, c.c_max));
}

static void split_scales_with_throttle(void)
{
    trim_learn_cfg_t c = cfg(); trim_learn_t s;
    trim_learn_init(&s, &c);
    assert(fabsf(trim_learn_split(&s, 0.40f) - 0.080f) < 1e-6f);  /* 8% at 40% */
    assert(fabsf(trim_learn_split(&s, 0.10f) - 0.020f) < 1e-6f);  /* 2% at 10% */
    assert(trim_learn_split(&s, 0.0f) == 0.0f);                   /* stopped */
}

int main(void)
{
    negative_drift_raises_c();
    positive_drift_lowers_c();
    small_drift_inside_the_deadband_changes_nothing();
    steering_freezes_learning();
    low_throttle_freezes_learning();
    a_repeated_sample_is_ignored();
    clamp_latches_a_fault_and_stops();
    unhealthy_gyro_freezes_learning();
    nonfinite_input_is_rejected();
    update_reports_whether_c_changed();
    a_long_freeze_is_not_a_giant_correction();
    an_impact_is_rejected_not_learned_from();
    reset_sets_c_and_wipes_what_it_knew();
    reset_refuses_values_outside_the_clamp();
    split_scales_with_throttle();
    return 0;
}
