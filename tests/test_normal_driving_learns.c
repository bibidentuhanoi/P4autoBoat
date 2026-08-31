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
#include "trim_assist.h"

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

/* The shipped P-assist configuration (CONFIG_STABILITY_PASSIST_*). */
static trim_assist_cfg_t shipped_assist_cfg(void)
{
    trim_assist_cfg_t c = { .kp = 0.035f, .tau_s = 0.25f, .cap = 0.075f };
    return c;
}

/* ---------------------------------------------------------------------------
 * The ordinary-driving branch of trim_learn_tick(), mirrored.
 *
 * motor_control.c, with no bench run and no calibration in progress:
 *
 *     bool steering = fabsf(decision->rudder) > 0.02f ||
 *                     fabsf(decision->steer) > 0.02f ||
 *                     decision->steer_raw;
 *     bool driving  = (esc_driver_get_state() == ESC_STATE_ARMED);
 *     float thr     = driving ? decision->throttle : 0.0f;
 *     trim_learn_update(..., f.sequence, dt_s, f.yaw_rate, thr, steering, healthy);
 *     p_gate = s_p_assist_on && healthy && driving && !steering &&
 *              thr >= min_throttle && fabsf(yaw) <= TRIM_LEARN_REJECT_DPS;
 *     s_p_correction = trim_assist_update(..., dt_s, yaw, p_gate);
 *
 * tests/test_runtime_architecture.py pins those lines against the real source,
 * so this mirror cannot drift away from the firmware unnoticed.
 * ------------------------------------------------------------------------- */
static uint32_t s_seq = 0;

/* One tick's worth of pilot input -- the three ways this boat can be turned,
 * plus the throttle. Mirrors the fields of control_decision_t that the tick
 * actually reads. */
typedef struct {
    float yaw_dps;
    float throttle;
    float rudder;      /* differential thrust */
    float steer;       /* physical rudder servos */
    bool  steer_raw;   /* raw microsecond pulse in force */
    bool  armed;
    bool  healthy;
    bool  p_on;        /* the runtime P switch */
} pilot_t;

/* A neutral, straight-ahead, everything-healthy input at 40% throttle. */
static pilot_t cruising(void)
{
    pilot_t p = { .yaw_dps = -2.0f, .throttle = 0.40f, .rudder = 0.0f,
                  .steer = 0.0f, .steer_raw = false, .armed = true,
                  .healthy = true, .p_on = false };
    return p;
}

static bool mirror_steering(const pilot_t *p)
{
    return fabsf(p->rudder) > 0.02f || fabsf(p->steer) > 0.02f || p->steer_raw;
}

/* Returns true when c moved; writes the P correction actually applied. */
static bool drive_sample(trim_learn_t *s, const trim_learn_cfg_t *cfg,
                         trim_assist_t *pa, const trim_assist_cfg_t *pcfg,
                         const pilot_t *p, float *p_corr_out)
{
    const bool steering = mirror_steering(p);
    const float thr = p->armed ? p->throttle : 0.0f;
    const bool moved = trim_learn_update(s, cfg, ++s_seq, DT, p->yaw_dps,
                                         thr, steering, p->healthy);
    const bool p_gate = p->p_on && p->healthy && p->armed && !steering &&
                        (thr >= cfg->min_throttle) &&
                        (fabsf(p->yaw_dps) <= TRIM_LEARN_REJECT_DPS);
    const float corr = trim_assist_update(pa, pcfg, DT, p->yaw_dps, p_gate);
    if (p_corr_out) *p_corr_out = corr;
    return moved;
}

/* Hold a steady stick for `seconds` and report how many times c moved. */
static int drive_pilot_for(trim_learn_t *s, const trim_learn_cfg_t *cfg,
                           trim_assist_t *pa, const trim_assist_cfg_t *pcfg,
                           float seconds, const pilot_t *p, float *p_corr_out)
{
    const int n = (int)(seconds * FUSION_HZ);
    int moved = 0;
    for (int i = 0; i < n; ++i) {
        if (drive_sample(s, cfg, pa, pcfg, p, p_corr_out)) moved++;
    }
    return moved;
}

/* Convenience for the cases that do not care about P. */
static int drive_for(trim_learn_t *s, const trim_learn_cfg_t *cfg,
                     float seconds, float yaw_dps, float throttle, float rudder,
                     bool armed, bool healthy)
{
    trim_assist_t pa;
    trim_assist_reset(&pa);
    const trim_assist_cfg_t pcfg = shipped_assist_cfg();
    pilot_t p = cruising();
    p.yaw_dps = yaw_dps; p.throttle = throttle; p.rudder = rudder;
    p.armed = armed; p.healthy = healthy;
    return drive_pilot_for(s, cfg, &pa, &pcfg, seconds, &p, NULL);
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

/* --- 8. a SteerCommand on the PHYSICAL rudder freezes c and zeroes P ------
 *
 * This is the channel the old rudder-only test missed entirely: turning on the
 * rudder servos leaves decision->rudder at 0, so the learner saw a straight
 * line and integrated the yaw of a deliberate turn straight into c.
 */
static void a_physical_rudder_command_freezes_c_and_zeroes_p(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    const trim_assist_cfg_t pcfg = shipped_assist_cfg();
    trim_learn_t s;
    trim_assist_t pa;
    trim_learn_init(&s, &cfg);
    trim_assist_reset(&pa);

    /* Straight and level with P switched on: both loops are working. */
    pilot_t p = cruising();
    p.p_on = true;
    float corr = 0.0f;
    const int moved_straight = drive_pilot_for(&s, &cfg, &pa, &pcfg, 2.0f, &p, &corr);
    assert(moved_straight > 0);
    assert(corr > 0.0f);                /* P is pushing back on the lean */
    const float c_before = s.c;
    assert(c_before > cfg.c_init);

    /* Now the pilot turns the rudder. Differential thrust is untouched --
     * only the SteerCommand moves. */
    p.steer = 0.50f;
    const int moved_turning = drive_pilot_for(&s, &cfg, &pa, &pcfg, 3.0f, &p, &corr);

    assert(moved_turning == 0);         /* c FROZEN */
    assert(s.c == c_before);            /* and unchanged, not merely slowed */
    assert(corr == 0.0f);               /* P zeroed, exactly -- no decay */
    assert(pa.correction == 0.0f);
    assert(!pa.initialized);            /* reset, so it cannot carry over */
    assert(!s.faulted);
    printf("  physical rudder: c held at %.4f, P zeroed\n", (double)c_before);

    /* Rudder back to neutral. A SteerCommand of 0.0 is what does this on the
     * real boat -- it clears both the value and the raw flag. */
    p.steer = 0.0f;
    const int moved_after = drive_pilot_for(&s, &cfg, &pa, &pcfg, 2.0f, &p, &corr);

    assert(moved_after > 0);            /* learning RESUMED */
    assert(s.c > c_before);             /* and carried on from where it was */
    assert(corr > 0.0f);                /* P came back too */
    printf("  back to neutral: learning resumed, c %.4f -> %.4f\n",
           (double)c_before, (double)s.c);
}

/* --- 9. all three steering channels, each on its own ---------------------- */
static void each_steering_channel_freezes_independently(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    const trim_assist_cfg_t pcfg = shipped_assist_cfg();

    struct { const char *what; float rudder, steer; bool raw; } chans[] = {
        { "differential thrust (decision->rudder)", 0.50f, 0.00f, false },
        { "physical rudder     (decision->steer)",  0.00f, 0.50f, false },
        { "raw pulse           (steer_raw)",        0.00f, 0.00f, true  },
        /* A raw pulse carries value 0.0f by construction -- submit_steer_raw
         * forces it -- so the flag is the ONLY thing that can catch it. */
    };

    for (unsigned i = 0; i < sizeof(chans) / sizeof(chans[0]); ++i) {
        trim_learn_t s;
        trim_assist_t pa;
        trim_learn_init(&s, &cfg);
        trim_assist_reset(&pa);
        pilot_t p = cruising();
        p.p_on = true;
        p.rudder = chans[i].rudder;
        p.steer = chans[i].steer;
        p.steer_raw = chans[i].raw;

        const float c0 = s.c;
        float corr = 1.0f;              /* seed non-zero so a no-op would show */
        const int moved = drive_pilot_for(&s, &cfg, &pa, &pcfg, 4.0f, &p, &corr);

        assert(moved == 0);
        assert(s.c == c0);
        assert(corr == 0.0f);
        assert(!s.faulted);
        printf("  frozen by %s\n", chans[i].what);
    }
}

/* --- 10. a rudder nudge below the threshold is still straight driving ----- */
static void a_tiny_rudder_offset_does_not_stop_learning(void)
{
    const trim_learn_cfg_t cfg = shipped_cfg();
    const trim_assist_cfg_t pcfg = shipped_assist_cfg();
    trim_learn_t s;
    trim_assist_t pa;
    trim_learn_init(&s, &cfg);
    trim_assist_reset(&pa);

    /* 0.02 is the threshold; 0.01 on the physical rudder is trim slop, not a
     * turn. If this froze, a servo that never quite reads zero would disable
     * the whole feature. */
    pilot_t p = cruising();
    p.steer = 0.01f;
    const int moved = drive_pilot_for(&s, &cfg, &pa, &pcfg, 2.0f, &p, NULL);
    assert(moved > 0);
    assert(s.c > cfg.c_init);
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
    a_physical_rudder_command_freezes_c_and_zeroes_p();
    each_steering_channel_freezes_independently();
    a_tiny_rudder_offset_does_not_stop_learning();
    printf("test_normal_driving_learns: OK\n");
    return 0;
}
