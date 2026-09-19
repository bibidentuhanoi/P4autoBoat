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
        .yaw_tau_s = 0.08f,
        .rate_kp = 0.100f,
        .rate_ki = 0.080f,
        .heading_tau_s = 0.10f,
        .heading_kp = 1.50f,
        .max_yaw_target_dps = 15.0f,
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
    /* Once the error reverses, I is allowed to move negative and support the
     * newly requested correction. */
    assert(out.i_term < held_i);
    assert(out.dynamic_c < 0.0f);
}

static void saturation_recovery_never_reverses_a_remaining_rate_error(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();

    for (int heading_case = 0; heading_case < 2; ++heading_case) {
        yaw_heading_control_t ctl;
        yaw_heading_control_init(&ctl);
        float heading = 0.0f;
        yaw_heading_output_t out = {0};

        for (int i = 0; i < 25; ++i) {
            heading -= -50.0f * DT_S;
            out = tick(&ctl, &cfg, -50.0f, heading, heading_case != 0,
                       0.40f, 0.21f, 0.0f, true, true, false);
        }
        for (int i = 0; i < 50; ++i) {
            heading -= -5.0f * DT_S;
            out = tick(&ctl, &cfg, -5.0f, heading, heading_case != 0,
                       0.40f, 0.21f, 0.0f, true, true, false);
        }

        /* Negative measured yaw with a zero-or-positive target requires a
         * positive correction (right motor stronger).  Saturation history
         * must never make the controller reinforce the remaining turn. */
        assert(out.rate_error_dps > 0.0f);
        assert(out.effective_c > 0.0f);
    }
}

static void integral_can_reach_full_authority_against_feedforward(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);
    yaw_heading_output_t out = {0};

    for (int i = 0; i < 4000; ++i) {
        out = tick(&ctl, &cfg, +1.0f, 0.0f, false,
                   0.40f, 0.21f, 0.0f, true, true, false);
    }

    /* I corrects the total command, including an opposing learned baseline.
     * It must be able to reach the physical mixer limit in either direction. */
    assert(out.c_limit > 0.99f);
    assert(out.effective_c < -0.99f);
}

static void long_sample_gap_does_not_create_a_large_integral_step(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);

    yaw_heading_output_t out = tick(&ctl, &cfg, -1.0f, 0.0f, false,
                                    0.40f, 0.21f, 0.0f, true, true, false);
    const float before = out.i_term;
    const yaw_heading_input_t delayed = {
        .dt_s = 2.0f,
        .yaw_rate_dps = -1.0f,
        .heading_deg = 0.0f,
        .throttle = 0.40f,
        .feedforward_c = 0.21f,
        .steering = 0.0f,
        .enabled = true,
        .driving = true,
        .gyro_fresh = true,
        .heading_valid = false,
        .base_capture_now = false,
    };
    out = yaw_heading_control_update(&ctl, &cfg, &delayed);

    assert(out.i_term > before);
    /* dt is capped at 0.1 s, so even the stronger shipped I gain may only
     * take one bounded 0.008-c step after a long scheduling gap. */
    assert(out.i_term - before < 0.009f);
}

static void invalid_heading_still_allows_gyro_rate_damping(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);
    const yaw_heading_input_t in = {
        .dt_s = DT_S,
        .yaw_rate_dps = -4.0f,
        .heading_deg = NAN,
        .throttle = 0.40f,
        .feedforward_c = 0.21f,
        .steering = 0.0f,
        .enabled = true,
        .driving = true,
        .gyro_fresh = true,
        .heading_valid = false,
        .base_capture_now = false,
    };

    yaw_heading_output_t out = yaw_heading_control_update(&ctl, &cfg, &in);
    assert(out.active);
    assert(!out.heading_hold);
    assert(out.rate_error_dps > 0.0f);
    assert(out.effective_c > 0.0f);
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
    /* This boat reports positive yaw for a left turn, while compass heading
     * decreases to the left.  Current=1 with target=359 therefore needs a
     * positive/left yaw request. */
    assert(out.yaw_target_dps > 0.0f);
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

static void heading_error_commands_a_fast_physical_return(void)
{
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);

    /* Capture 160 degrees, then simulate a left drift to 150.  Positive yaw
     * is left on the installed IMU, so recovery must request negative yaw and
     * reduce c (left motor stronger). */
    (void)tick(&ctl, &cfg, 0.0f, 160.0f, true,
               0.40f, 0.21f, 0.0f, true, true, false);
    yaw_heading_output_t out = {0};
    for (int i = 0; i < 10; ++i) {
        out = tick(&ctl, &cfg, 0.0f, 150.0f, true,
                   0.40f, 0.21f, 0.0f, true, true, false);
    }
    assert(out.heading_error_deg > 8.0f);
    assert(out.yaw_target_dps < -12.0f);
    assert(out.dynamic_c < -0.75f);
    assert(out.effective_c < -0.50f);
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

typedef struct {
    float final_yaw;
    float final_heading_error;
    float final_i;
    float last_10_abs_yaw_mean;
    int first_correction_sample;
    int high_yaw_active_samples;
    int high_yaw_wrong_sign_samples;
    int final_10_saturated_samples;
} sim_result_t;

static sim_result_t run_measured_plant(float duration_s,
                                       float disturbance_dps,
                                       float initial_yaw_dps,
                                       bool controller_on)
{
    enum { DELAY_SAMPLES = 15, MAX_SAMPLES = 2000 };
    const yaw_heading_cfg_t cfg = shipped_cfg();
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);
    float delay[DELAY_SAMPLES] = {0};
    int delay_pos = 0;
    float yaw = initial_yaw_dps;
    float heading = 100.0f;
    const float target_heading = heading;
    const int samples = (int)(duration_s / DT_S);
    assert(samples <= MAX_SAMPLES);
    sim_result_t result = {.first_correction_sample = -1};
    float final_10_abs_sum = 0.0f;
    int final_10_count = 0;

    for (int n = 0; n < samples; ++n) {
        yaw_heading_output_t out = tick(&ctl, &cfg, yaw, heading, true,
                                        0.40f, 0.21f, 0.0f,
                                        controller_on, true, false);
        const float requested = controller_on ? out.dynamic_c : 0.0f;
        if (result.first_correction_sample < 0 && fabsf(requested) > 1e-5f) {
            result.first_correction_sample = n;
        }
        if (fabsf(yaw) > 10.0f) {
            result.high_yaw_active_samples += out.active ? 1 : 0;
            if ((yaw < 0.0f && requested <= 0.0f) ||
                (yaw > 0.0f && requested >= 0.0f)) {
                result.high_yaw_wrong_sign_samples++;
            }
        }

        const float applied = delay[delay_pos];
        delay[delay_pos] = requested;
        delay_pos = (delay_pos + 1) % DELAY_SAMPLES;
        const float steady_yaw = 16.1f * applied + disturbance_dps;
        yaw += ((steady_yaw - yaw) / 1.10f) * DT_S;
        /* Installed convention: positive/left yaw decreases compass heading. */
        heading = fmodf(heading - yaw * DT_S + 360.0f, 360.0f);

        if (n >= samples - 500) {
            final_10_abs_sum += fabsf(yaw);
            final_10_count++;
            result.final_10_saturated_samples += out.saturated ? 1 : 0;
        }
        result.final_i = out.i_term;
    }
    result.final_yaw = yaw;
    result.final_heading_error = yaw_heading_wrap_180(target_heading - heading);
    result.last_10_abs_yaw_mean = final_10_abs_sum / (float)final_10_count;
    return result;
}

static void measured_plant_meets_three_second_acceptance(void)
{
    const sim_result_t off = run_measured_plant(3.0f, -10.0f, -20.0f, false);
    const sim_result_t on = run_measured_plant(3.0f, -10.0f, -20.0f, true);
    assert(on.first_correction_sample >= 0 && on.first_correction_sample <= 1);
    assert(on.high_yaw_active_samples > 0);
    assert(on.high_yaw_wrong_sign_samples == 0);
    assert(fabsf(on.final_yaw) < fabsf(off.final_yaw));
}

static void measured_plant_meets_thirty_second_acceptance(void)
{
    const sim_result_t out = run_measured_plant(30.0f, -3.0f, 0.0f, true);
    assert(out.last_10_abs_yaw_mean < 0.5f);
    assert(fabsf(out.final_heading_error) < 1.0f);
    assert(out.final_10_saturated_samples < 50);
    assert(fabsf(out.final_i) > 0.01f);
}

int main(void)
{
    correction_is_immediate_strong_and_bidirectional();
    integral_cancels_a_persistent_rate_error();
    anti_windup_stops_outward_growth_and_allows_unwind();
    saturation_recovery_never_reverses_a_remaining_rate_error();
    integral_can_reach_full_authority_against_feedforward();
    long_sample_gap_does_not_create_a_large_integral_step();
    invalid_heading_still_allows_gyro_rate_damping();
    heading_wrap_and_invalid_fallback_are_safe();
    heading_error_commands_a_fast_physical_return();
    manual_steering_freezes_i_then_recaptures();
    safety_gates_reset_dynamic_state();
    measured_plant_meets_three_second_acceptance();
    measured_plant_meets_thirty_second_acceptance();
    puts("yaw heading controller tests passed");
    return 0;
}
