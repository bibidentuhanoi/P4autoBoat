#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "yaw_heading_control.h"

#define DT_S 0.02f

static bool closef(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

static yaw_heading_cfg_t shipped_cfg(void)
{
    return (yaw_heading_cfg_t){
        .yaw_tau_s = 0.15f,
        .rate_kp = 0.050f,
        .rate_ki = 0.020f,
        .heading_tau_s = 0.35f,
        .heading_kp = 0.80f,
        .max_yaw_target_dps = 8.0f,
        .min_throttle = 0.15f,
        .steering_deadband = 0.02f,
        .recapture_delay_s = 0.50f,
    };
}

static yaw_heading_output_t tick(yaw_heading_control_t *ctl,
                                 const yaw_heading_cfg_t *cfg,
                                 float yaw_dps,
                                 float heading_deg,
                                 bool heading_valid,
                                 float throttle,
                                 float feedforward_c,
                                 float steering,
                                 bool enabled,
                                 bool gyro_fresh,
                                 bool base_capture_now)
{
    const yaw_heading_input_t in = {
        .dt_s = DT_S,
        .yaw_rate_dps = yaw_dps,
        .heading_deg = heading_deg,
        .throttle = throttle,
        .feedforward_c = feedforward_c,
        .steering = steering,
        .enabled = enabled,
        .driving = true,
        .gyro_fresh = gyro_fresh,
        .heading_valid = heading_valid,
        .base_capture_now = base_capture_now,
    };
    return yaw_heading_control_update(ctl, cfg, &in);
}

static void correction_is_immediate_strong_and_bidirectional(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);

    yaw_heading_output_t out = tick(&ctl, &cfg, -20.0f, 100.0f, true,
                                    0.40f, 0.21f, 0.0f, true, true, false);
    assert(out.active);
    assert(out.p_term > 0.0f);
    assert(out.dynamic_c > 0.0f);
    assert(out.saturated);
    assert(closef(out.c_limit, 1.0f, 1e-6f));
    assert(out.effective_c >= -1.0f && out.effective_c <= 1.0f);

    yaw_heading_control_reset(&ctl);
    out = tick(&ctl, &cfg, +20.0f, 100.0f, true,
               0.40f, 0.21f, 0.0f, true, true, false);
    assert(out.active);
    assert(out.p_term < 0.0f);
    assert(out.dynamic_c < 0.0f);
}

static void integral_cancels_a_persistent_rate_error(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);

    yaw_heading_output_t first = tick(&ctl, &cfg, -1.0f, 0.0f, false,
                                      0.40f, 0.21f, 0.0f, true, true, false);
    yaw_heading_output_t out = first;
    for (int i = 0; i < 100; ++i) {
        out = tick(&ctl, &cfg, -1.0f, 0.0f, false,
                   0.40f, 0.21f, 0.0f, true, true, false);
    }
    assert(out.i_term > first.i_term + 0.02f);
    assert(out.dynamic_c > first.dynamic_c);
    assert(closef(out.yaw_target_dps, 0.0f, 1e-6f));
}

static void anti_windup_stops_outward_growth_and_allows_unwind(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);
    yaw_heading_output_t out = {0};

    for (int i = 0; i < 300; ++i) {
        out = tick(&ctl, &cfg, -50.0f, 0.0f, false,
                   0.80f, 0.21f, 0.0f, true, true, false);
    }
    assert(out.saturated);
    assert(closef(out.c_limit, 0.25f, 1e-5f));
    const float held_i = out.i_term;
    for (int i = 0; i < 100; ++i) {
        out = tick(&ctl, &cfg, -50.0f, 0.0f, false,
                   0.80f, 0.21f, 0.0f, true, true, false);
    }
    assert(closef(out.i_term, held_i, 1e-5f));

    for (int i = 0; i < 100; ++i) {
        out = tick(&ctl, &cfg, +5.0f, 0.0f, false,
                   0.80f, 0.21f, 0.0f, true, true, false);
    }
    /* Back-calculation releases the stored upper-limit cancellation as the P
     * term crosses through zero.  The requested correction must reverse; the
     * numerical I value rises because held_i was negative while cancelling a
     * large positive P term. */
    assert(out.i_term > held_i);
    assert(out.dynamic_c < 0.0f);
}

static void heading_wrap_and_invalid_fallback_are_safe(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);

    yaw_heading_output_t out = tick(&ctl, &cfg, 0.0f, 359.0f, true,
                                    0.40f, 0.21f, 0.0f, true, true, false);
    assert(out.heading_hold);
    assert(closef(out.heading_target_deg, 359.0f, 1e-5f));

    for (int i = 0; i < 30; ++i) {
        out = tick(&ctl, &cfg, 0.0f, 1.0f, true,
                   0.40f, 0.21f, 0.0f, true, true, false);
    }
    assert(out.heading_error_deg < 0.0f);
    assert(out.yaw_target_dps < 0.0f);
    assert(fabsf(out.heading_error_deg) < 3.0f);

    out = tick(&ctl, &cfg, -2.0f, 90.0f, false,
               0.40f, 0.21f, 0.0f, true, true, false);
    assert(out.active);
    assert(!out.heading_hold);
    assert(closef(out.yaw_target_dps, 0.0f, 1e-6f));
    assert(out.dynamic_c > 0.0f);

    out = tick(&ctl, &cfg, 0.0f, 90.0f, true,
               0.40f, 0.21f, 0.0f, true, true, false);
    assert(out.heading_hold);
    assert(closef(out.heading_target_deg, 90.0f, 1e-5f));
}

static void manual_steering_freezes_i_then_recaptures(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);
    yaw_heading_output_t out = {0};

    for (int i = 0; i < 100; ++i) {
        out = tick(&ctl, &cfg, -1.0f, 100.0f, true,
                   0.40f, 0.21f, 0.0f, true, true, false);
    }
    const float learned_i = out.i_term;

    for (int i = 0; i < 50; ++i) {
        out = tick(&ctl, &cfg, -20.0f, 120.0f, true,
                   0.40f, 0.21f, 0.30f, true, true, false);
        assert(!out.active);
        assert(closef(out.dynamic_c, 0.0f, 1e-6f));
        assert(closef(ctl.integral, learned_i, 1e-6f));
    }

    for (int i = 0; i < 24; ++i) {
        out = tick(&ctl, &cfg, 0.0f, 120.0f, true,
                   0.40f, 0.21f, 0.0f, true, true, false);
        assert(out.active);
        assert(!out.heading_hold);
    }
    out = tick(&ctl, &cfg, 0.0f, 120.0f, true,
               0.40f, 0.21f, 0.0f, true, true, false);
    assert(out.heading_hold);
    assert(closef(out.heading_target_deg, 120.0f, 1e-5f));
}

static void safety_gates_reset_dynamic_state(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);

    for (int i = 0; i < 100; ++i) {
        (void)tick(&ctl, &cfg, -1.0f, 10.0f, true,
                   0.40f, 0.21f, 0.0f, true, true, false);
    }
    assert(ctl.integral > 0.0f);

    yaw_heading_output_t out = tick(&ctl, &cfg, -1.0f, 10.0f, true,
                                    0.10f, 0.21f, 0.0f, true, true, false);
    assert(!out.active);
    assert(closef(ctl.integral, 0.0f, 1e-6f));

    (void)tick(&ctl, &cfg, -1.0f, 10.0f, true,
               0.40f, 0.21f, 0.0f, true, true, false);
    out = tick(&ctl, &cfg, -1.0f, 10.0f, true,
               0.40f, 0.21f, 0.0f, true, false, false);
    assert(!out.active);
    assert(closef(ctl.integral, 0.0f, 1e-6f));
}

int main(void)
{
    correction_is_immediate_strong_and_bidirectional();
    integral_cancels_a_persistent_rate_error();
    anti_windup_stops_outward_growth_and_allows_unwind();
    heading_wrap_and_invalid_fallback_are_safe();
    manual_steering_freezes_i_then_recaptures();
    safety_gates_reset_dynamic_state();
    puts("yaw heading controller tests passed");
    return 0;
}
