/* The learner must work while the pilot simply drives forward.
 *
 * Every other trim test drives it through a BASE bench run, because that is
 * how the measurements were taken. But the feature is not "press BASE and
 * watch a number" -- it is "hold the throttle and the boat straightens out".
 * If learning only ever happened inside a bench run, all of those tests would
 * still pass and the boat would never trim itself on the water.
 *
 * So this file exercises the ORDINARY-DRIVING path: no bench, no calibration,
 * just an armed boat with the stick pushed forward. It mirrors exactly what
 * motor_control.c's trim_learn_tick() computes on that branch, then feeds the
 * result through the real mixer to prove the moved c reaches the jets.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esc_trim.h"
#include "trim_learn.h"

#define FUSION_HZ 50
#define DT (1.0f / (float)FUSION_HZ)

/* The shipped configuration -- Kconfig defaults, not invented numbers. A test
 * that passes only under friendlier tuning proves nothing about the boat. */
static trim_learn_cfg_t shipped_cfg(void)
{
    trim_learn_cfg_t c = {
        .c_init = 0.17f,          /* CONFIG_ESC_TRIM_C */
        .c_min = 0.10f, .c_max = 0.35f,
        .deadband_dps = 0.5f,     /* CONFIG_STABILITY_TRIMLEARN_DEADBAND_DPS */
        .step_per_s = 0.005f,     /* CONFIG_STABILITY_TRIMLEARN_STEP_PER_S */
        .yaw_tau_s = 2.0f,
        .min_throttle = 0.15f,    /* CONFIG_STABILITY_TRIMLEARN_MIN_THROTTLE */
        .reject_dps = TRIM_LEARN_REJECT_DPS,
    };
    return c;
}

/* ---------------------------------------------------------------------------
 * The ordinary-driving branch of trim_learn_tick(), mirrored.
 *
 * motor_control.c, with no bench run and no calibration in progress:
 *
 *     bool steering = fabsf(decision->rudder) > 0.02f;
 *     bool driving  = (esc_driver_get_state() == ESC_STATE_ARMED);
 *     float thr     = driving ? decision->throttle : 0.0f;
 *     trim_learn_update(..., f.sequence, dt_s, f.yaw_rate, thr, steering, healthy);
 *
 * tests/test_runtime_architecture.py pins those four lines against the real
 * source, so this mirror cannot drift away from the firmware unnoticed.
 * ------------------------------------------------------------------------- */
static uint32_t s_seq = 0;

static bool drive_sample(trim_learn_t *s, const trim_learn_cfg_t *cfg,
                         float yaw_dps, float throttle, float rudder,
                         bool armed, bool healthy)
{
    const bool steering = fabsf(rudder) > 0.02f;
    const float thr = armed ? throttle : 0.0f;
    return trim_learn_update(s, cfg, ++s_seq, DT, yaw_dps, thr, steering, healthy);
}

/* Hold a steady stick for `seconds` and report how many times c moved. */
static int drive_for(trim_learn_t *s, const trim_learn_cfg_t *cfg,
                     float seconds, float yaw_dps, float throttle, float rudder,
                     bool armed, bool healthy)
{
    const int n = (int)(seconds * FUSION_HZ);
    int moved = 0;
    for (int i = 0; i < n; ++i) {
        if (drive_sample(s, cfg, yaw_dps, throttle, rudder, armed, healthy)) moved++;
    }
    return moved;
}

/* --- 1. the whole point: forward driving, no bench run, and c moves -------- */
static void ordinary_forward_driving_teaches_the_learner(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    trim_learn_t s;
    trim_learn_init(&s, &cfg);
    const float c0 = s.c;

    /* Armed, 40% throttle, stick centred, healthy gyro, boat leaning the way
     * too-little-trim leans it. No bench run anywhere in sight. */
    const int moved = drive_for(&s, &cfg, 3.0f, -2.0f, 0.40f, 0.0f, true, true);

    assert(moved > 0);                  /* it actually adapted */
    assert(s.c > c0);                   /* negative drift => MORE trim */
    assert(!s.faulted);
    /* Rate limited to step_per_s: 3 s can move c by at most 0.015. */
    assert(s.c - c0 <= 0.005f * 3.0f + 1e-6f);
    printf("  ordinary driving: c %.4f -> %.4f over 3 s (%d steps)\n",
           (double)c0, (double)s.c, moved);
}

/* --- 2. and the other direction, from the same starting point ------------- */
static void too_much_trim_is_walked_back_down(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    trim_learn_t s;
    trim_learn_init(&s, &cfg);
    const float c0 = s.c;

    drive_for(&s, &cfg, 3.0f, +2.0f, 0.40f, 0.0f, true, true);

    assert(s.c < c0);                   /* positive drift => LESS trim */
    assert(!s.faulted);
}

/* --- 3. the moved c has to reach the jets, not just the log --------------- */
static void the_learned_c_changes_the_motor_commands(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    trim_learn_t s;
    trim_learn_init(&s, &cfg);

    EscTrimPoint before[2];
    uint8_t nb = esc_trim_build_proportional(s.c, before);
    float l0, r0;
    esc_trim_mix(0.40f, 0.0f, before, nb, &l0, &r0);

    drive_for(&s, &cfg, 3.0f, -2.0f, 0.40f, 0.0f, true, true);

    EscTrimPoint after[2];
    uint8_t na = esc_trim_build_proportional(s.c, after);
    float l1, r1;
    esc_trim_mix(0.40f, 0.0f, after, na, &l1, &r1);

    assert(l1 < l0 && r1 > r0);         /* right jet gained, left gave up */
    /* the split is still symmetric about the commanded throttle */
    assert(fabsf((l1 + r1) * 0.5f - 0.40f) < 1e-5f);
    printf("  motors at T40: (%.4f, %.4f) -> (%.4f, %.4f)\n",
           (double)l0, (double)r0, (double)l1, (double)r1);
}

/* --- 4. c must survive letting go of the throttle ------------------------- */
static void c_is_retained_across_a_throttle_release(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    trim_learn_t s;
    trim_learn_init(&s, &cfg);

    drive_for(&s, &cfg, 2.0f, -2.0f, 0.40f, 0.0f, true, true);
    const float learned = s.c;
    assert(learned > cfg.c_init);

    /* Stick released for four seconds. The yaw filter keeps tracking (so it is
     * already settled when the motors come back), but c must not budge. */
    const int moved_idle = drive_for(&s, &cfg, 4.0f, -2.0f, 0.0f, 0.0f, true, true);
    assert(moved_idle == 0);
    assert(s.c == learned);             /* exactly, not approximately */

    /* Back on the throttle: it carries on from what it learned, it does not
     * restart from the flashed value. */
    drive_for(&s, &cfg, 2.0f, -2.0f, 0.40f, 0.0f, true, true);
    assert(s.c > learned);
    printf("  across a release: %.4f held, then continued to %.4f\n",
           (double)learned, (double)s.c);
}

/* --- 5. every safety gate must stop it ------------------------------------ */
static void the_safety_gates_all_freeze_learning(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();

    struct {
        const char *what;
        float yaw, throttle, rudder;
        bool armed, healthy;
    } cases[] = {
        /* yaw is a real, in-range lean unless the case is about the yaw itself */
        { "disarmed (stick up, jets dead)", -2.0f, 0.40f,  0.00f, false, true  },
        { "stopped",                        -2.0f, 0.00f,  0.00f, true,  true  },
        { "below the minimum throttle",     -2.0f, 0.10f,  0.00f, true,  true  },
        { "reverse",                        -2.0f, -0.40f, 0.00f, true,  true  },
        { "turning (rudder demand)",        -2.0f, 0.40f,  0.50f, true,  true  },
        { "a nudge off centre",             -2.0f, 0.40f,  0.03f, true,  true  },
        { "stale / unhealthy gyro",         -2.0f, 0.40f,  0.00f, true,  false },
        /* Already straight. Below the deadband the sign of what is left is
         * noise, and integrating it walks c off on a random walk forever --
         * the boat would never settle even once it had the answer. */
        { "drift inside the deadband",      -0.3f, 0.40f,  0.00f, true,  true  },
        { "the same, leaning the other way", 0.3f, 0.40f,  0.00f, true,  true  },
    };

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        trim_learn_t s;
        trim_learn_init(&s, &cfg);
        const float c0 = s.c;
        const int moved = drive_for(&s, &cfg, 5.0f, cases[i].yaw,
                                    cases[i].throttle, cases[i].rudder,
                                    cases[i].armed, cases[i].healthy);
        assert(moved == 0);
        assert(s.c == c0);
        assert(!s.faulted);
        printf("  frozen: %s\n", cases[i].what);
    }
}

/* --- 6. a wall is not a mistrim ------------------------------------------- */
static void hitting_something_is_not_learned_from(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    trim_learn_t s;
    trim_learn_init(&s, &cfg);
    const float c0 = s.c;

    /* Well above TRIM_LEARN_REJECT_DPS for a sustained bounce. */
    const int moved = drive_for(&s, &cfg, 3.0f, -25.0f, 0.40f, 0.0f, true, true);
    assert(moved == 0);
    assert(s.c == c0);
}

/* --- 7. the gates open and shut, they do not latch ------------------------ */
static void learning_resumes_once_the_gate_reopens(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    trim_learn_t s;
    trim_learn_init(&s, &cfg);

    drive_for(&s, &cfg, 2.0f, -2.0f, 0.40f, 0.50f, true, true);   /* turning */
    assert(s.c == cfg.c_init);

    drive_for(&s, &cfg, 2.0f, -2.0f, 0.40f, 0.0f, true, true);    /* straight */
    assert(s.c > cfg.c_init);
    assert(!s.faulted);
}

int main(void)
{
    ordinary_forward_driving_teaches_the_learner();
    too_much_trim_is_walked_back_down();
    the_learned_c_changes_the_motor_commands();
    c_is_retained_across_a_throttle_release();
    the_safety_gates_all_freeze_learning();
    hitting_something_is_not_learned_from();
    learning_resumes_once_the_gate_reopens();
    printf("test_normal_driving_learns: OK\n");
    return 0;
}
