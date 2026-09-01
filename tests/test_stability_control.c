/* Rudder yaw-rate stabilizer: signs first, everything else second.
 *
 * The sign is the whole point. The previous version of this loop computed
 * rudder = k_r * (target - yaw), treating stick sign and yaw sign as the same
 * quantity. On this boat they are OPPOSITE (21 recorded runs: rudder -0.80
 * gave a median +2.30 deg/s, +0.80 gave -1.82), so that law asked for a left
 * turn by commanding right rudder -- which drives yaw further the wrong way,
 * grows the error, and commands more. It only never did that on the water
 * because CONFIG_STABILITY_SAS_ENABLE defaulted n.
 *
 * So the four sign cases below are not style checks. Each one, inverted, is a
 * boat that puts the rudder hard over and holds it there.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "stability_control.h"

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-4f)

/* The first pool configuration, mirrored from Kconfig. */
static stab_cfg_t pool_cfg(void)
{
    stab_cfg_t c = {
        .r_max_dps = 2.0f,
        .k_p = 0.05f,
        .ff_left = 0.18f,     /* positive yaw targets */
        .ff_right = 0.31f,    /* negative yaw targets */
        .yaw_tau_s = 0.25f,
        .out_cap = 0.80f,
        .slew_per_s = 2.0f,
    };
    return c;
}

/* One settled update: snap the filter to `yaw`, then hold both inputs steady
 * long enough that the slew limiter is not what we are measuring. */
static float settled(const stab_cfg_t *cfg, float stick, float yaw,
                     stab_debug_t *dbg)
{
    stab_state_t s;
    stab_reset(&s);
    float out = stab_rudder_update(&s, cfg, 0.0f, stick, yaw, dbg);
    for (int i = 0; i < 400; ++i) {
        out = stab_rudder_update(&s, cfg, 0.02f, stick, yaw, dbg);
    }
    return out;
}

/* ---- the four sign cases the design must satisfy ------------------------ */

static void left_stick_requests_positive_yaw_and_gives_left_rudder(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_debug_t d;
    /* Canonical steering: -1 is LEFT. */
    assert(NEAR(stab_target_dps(&cfg, -1.0f), +2.0f));
    float out = settled(&cfg, -1.0f, 0.0f, &d);
    assert(d.target_dps > 0.0f);        /* left asks for POSITIVE yaw */
    assert(out < 0.0f);                 /* and commands NEGATIVE (left) rudder */
    printf("  left stick  -> target %+.2f deg/s, rudder %+.3f\n",
           (double)d.target_dps, (double)out);
}

static void right_stick_requests_negative_yaw_and_gives_right_rudder(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_debug_t d;
    assert(NEAR(stab_target_dps(&cfg, +1.0f), -2.0f));
    float out = settled(&cfg, +1.0f, 0.0f, &d);
    assert(d.target_dps < 0.0f);        /* right asks for NEGATIVE yaw */
    assert(out > 0.0f);                 /* and commands POSITIVE (right) rudder */
    printf("  right stick -> target %+.2f deg/s, rudder %+.3f\n",
           (double)d.target_dps, (double)out);
}

static void holding_zero_against_positive_yaw_gives_right_rudder(void)
{
    /* The boat is drifting LEFT (positive yaw) and we asked it not to. The
     * correction must be RIGHT rudder, i.e. positive. */
    const stab_cfg_t cfg = pool_cfg();
    stab_debug_t d;
    float out = settled(&cfg, 0.0f, +2.0f, &d);
    assert(NEAR(d.target_dps, 0.0f));
    assert(out > 0.0f);
    printf("  hold vs +yaw -> rudder %+.3f (right, counter)\n", (double)out);
}

static void holding_zero_against_negative_yaw_gives_left_rudder(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_debug_t d;
    float out = settled(&cfg, 0.0f, -2.0f, &d);
    assert(NEAR(d.target_dps, 0.0f));
    assert(out < 0.0f);
    printf("  hold vs -yaw -> rudder %+.3f (left, counter)\n", (double)out);
}

/* ---- feedforward -------------------------------------------------------- */

static void feedforward_matches_the_data_derived_values(void)
{
    /* At the moment the target is applied and yaw is still zero, the FF term
     * is what dominates. These are the numbers the pool configuration was
     * chosen to produce. */
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_debug_t d;

    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, -1.0f, 0.0f, &d);      /* left */
    assert(NEAR(d.ff_term, -0.36f));
    printf("  left  FF %+.3f (0.18 x 2.0)\n", (double)d.ff_term);

    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, +1.0f, 0.0f, &d);      /* right */
    assert(NEAR(d.ff_term, +0.62f));
    printf("  right FF %+.3f (0.31 x 2.0)\n", (double)d.ff_term);
}

static void feedforward_is_chosen_by_target_direction_not_error(void)
{
    /* Same target, wildly different measured yaw: the FF term must not move,
     * because the hull's asymmetry belongs to the direction being asked for. */
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_debug_t a, b;
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, -1.0f, -5.0f, &a);
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, -1.0f, +5.0f, &b);
    assert(NEAR(a.ff_term, b.ff_term));
    assert(NEAR(a.ff_term, -0.36f));
}

static void the_two_directions_use_different_gains(void)
{
    /* One gain would under-drive one way and over-drive the other. */
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_debug_t l, r;
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, -1.0f, 0.0f, &l);
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, +1.0f, 0.0f, &r);
    assert(fabsf(l.ff_term) < fabsf(r.ff_term));   /* left needs less rudder */
}

/* ---- proportional term -------------------------------------------------- */

static void the_p_term_opposes_the_error(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_debug_t d;

    /* target 0, yaw +2 -> error -2 -> P must be POSITIVE (right) */
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, 0.0f, +2.0f, &d);
    assert(NEAR(d.p_term, +0.10f));

    /* target 0, yaw -2 -> error +2 -> P must be NEGATIVE (left) */
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, 0.0f, -2.0f, &d);
    assert(NEAR(d.p_term, -0.10f));
}

static void reaching_the_target_leaves_only_feedforward(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_debug_t d;
    settled(&cfg, -1.0f, +2.0f, &d);      /* asked +2, measuring +2 */
    assert(NEAR(d.p_term, 0.0f));
    assert(NEAR(d.ff_term, -0.36f));
}

/* ---- cap and slew ------------------------------------------------------- */

static void the_output_is_capped(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_debug_t d;
    /* A huge error drives the raw command far past the cap. */
    float out = settled(&cfg, -1.0f, -60.0f, &d);
    assert(fabsf(out) <= cfg.out_cap + 1e-6f);
    assert(d.saturated);
    assert(fabsf(d.raw) > cfg.out_cap);
    printf("  capped: raw %+.3f -> out %+.3f\n", (double)d.raw, (double)out);
}

static void the_slew_limit_bounds_the_step_per_second(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_debug_t d;
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, 0.0f, 0.0f, &d);   /* settle at 0 */
    /* One 20 ms step may move at most 2.0 * 0.02 = 0.04. */
    float out = stab_rudder_update(&s, &cfg, 0.02f, +1.0f, -60.0f, &d);
    assert(fabsf(out) <= 0.04f + 1e-6f);
    assert(d.slewed);
    /* and it keeps walking, not jumping */
    float prev = out;
    for (int i = 0; i < 5; ++i) {
        out = stab_rudder_update(&s, &cfg, 0.02f, +1.0f, -60.0f, &d);
        assert(fabsf(out - prev) <= 0.04f + 1e-6f);
        prev = out;
    }
}

static void the_slew_limit_never_exceeds_the_cap(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, 0.0f, 0.0f, NULL);
    for (int i = 0; i < 500; ++i) {
        float out = stab_rudder_update(&s, &cfg, 0.02f, +1.0f, -60.0f, NULL);
        assert(fabsf(out) <= cfg.out_cap + 1e-6f);
    }
}

static void a_zero_slew_config_disables_the_limiter(void)
{
    stab_cfg_t cfg = pool_cfg();
    cfg.slew_per_s = 0.0f;
    stab_state_t s;
    stab_debug_t d;
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, 0.0f, 0.0f, &d);
    float out = stab_rudder_update(&s, &cfg, 0.02f, +1.0f, -60.0f, &d);
    assert(!d.slewed);
    /* It takes the whole command in one step. With the limiter on, the same
     * step would have been held to slew_per_s * dt = 0.04 -- so exceeding that
     * is the evidence the limiter is genuinely off, and matching `raw` is the
     * evidence nothing else clipped it either. */
    assert(NEAR(out, d.raw));
    assert(fabsf(out) > 0.04f);
    printf("  slew off: one step straight to %+.3f (limiter would allow 0.04)\n",
           (double)out);
}

/* ---- failure handling --------------------------------------------------- */

static void a_nonfinite_input_centres_and_resets(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, -1.0f, 1.0f, NULL);

    assert(stab_rudder_update(&s, &cfg, 0.02f, -1.0f, NAN, NULL) == 0.0f);
    assert(!s.initialized);
    assert(s.rudder == 0.0f);

    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, -1.0f, 1.0f, NULL);
    assert(stab_rudder_update(&s, &cfg, 0.02f, NAN, 1.0f, NULL) == 0.0f);
    assert(stab_rudder_update(&s, &cfg, NAN, -1.0f, 1.0f, NULL) == 0.0f);
}

static void reset_clears_the_slew_memory_too(void)
{
    /* Otherwise re-enabling the loop would walk from wherever it left off,
     * which after a centring fault is not where the rudder is. */
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, 0.0f, 0.0f, NULL);
    for (int i = 0; i < 100; ++i) {
        stab_rudder_update(&s, &cfg, 0.02f, +1.0f, -20.0f, NULL);
    }
    assert(fabsf(s.rudder) > 0.1f);
    stab_reset(&s);
    assert(s.rudder == 0.0f);
    assert(s.yaw_filt == 0.0f);
    assert(!s.initialized);
}

static void the_filter_snaps_on_the_first_sample(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_state_t s;
    stab_debug_t d;
    stab_reset(&s);
    stab_rudder_update(&s, &cfg, 0.0f, 0.0f, 7.5f, &d);
    assert(NEAR(d.yaw_filt, 7.5f));   /* not blended up from zero */
}

static void zero_stick_and_zero_yaw_commands_nothing(void)
{
    const stab_cfg_t cfg = pool_cfg();
    stab_debug_t d;
    float out = settled(&cfg, 0.0f, 0.0f, &d);
    assert(NEAR(out, 0.0f));
    assert(NEAR(d.ff_term, 0.0f));
    assert(NEAR(d.p_term, 0.0f));
}

int main(void)
{
    left_stick_requests_positive_yaw_and_gives_left_rudder();
    right_stick_requests_negative_yaw_and_gives_right_rudder();
    holding_zero_against_positive_yaw_gives_right_rudder();
    holding_zero_against_negative_yaw_gives_left_rudder();
    feedforward_matches_the_data_derived_values();
    feedforward_is_chosen_by_target_direction_not_error();
    the_two_directions_use_different_gains();
    the_p_term_opposes_the_error();
    reaching_the_target_leaves_only_feedforward();
    the_output_is_capped();
    the_slew_limit_bounds_the_step_per_second();
    the_slew_limit_never_exceeds_the_cap();
    a_zero_slew_config_disables_the_limiter();
    a_nonfinite_input_centres_and_resets();
    reset_clears_the_slew_memory_too();
    the_filter_snaps_on_the_first_sample();
    zero_stick_and_zero_yaw_commands_nothing();
    printf("test_stability_control: OK\n");
    return 0;
}
