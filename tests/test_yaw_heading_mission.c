/* The mission-owned mode of the heading hold (2026-09-25).  The existing
 * behaviour is pinned by tests/test_yaw_heading_control.c (unchanged) and,
 * bit for bit, by tests/test_heading_hold_unchanged.py. */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "yaw_heading_control.h"

static yaw_heading_cfg_t shipped(void)
{
    return (yaw_heading_cfg_t){ .yaw_tau_s = 0.08f, .rate_kp = 0.100f, .rate_ki = 0.080f,
                                .heading_tau_s = 0.10f, .heading_kp = 1.50f,
                                .max_yaw_target_dps = 15.0f, .min_throttle = 0.15f,
                                .steering_deadband = 0.02f, .recapture_delay_s = 0.50f };
}

static yaw_heading_output_t step(yaw_heading_control_t *ctl, float heading, float target,
                                 bool owned, float steering, float throttle)
{
    const yaw_heading_cfg_t cfg = shipped();
    const yaw_heading_input_t in = {
        .dt_s = 0.02f, .yaw_rate_dps = 0.0f, .heading_deg = heading, .throttle = throttle,
        .feedforward_c = 0.21f, .steering = steering, .enabled = true, .driving = true,
        .gyro_fresh = true, .heading_valid = true, .base_capture_now = false,
        .mission_owned = owned, .target_heading_deg = target,
    };
    return yaw_heading_control_update(ctl, &cfg, &in);
}

static void the_target_is_used_at_once_not_captured(void)
{
    yaw_heading_control_t ctl; yaw_heading_control_init(&ctl);
    yaw_heading_output_t o = step(&ctl, 90.0f, 120.0f, true, 0.0f, 0.40f);
    assert(o.active && o.heading_hold && fabsf(o.heading_target_deg - 120.0f) < 1e-4f);
    for (int i = 0; i < 20; ++i) o = step(&ctl, 90.0f, 120.0f, true, 0.0f, 0.40f);
    /* Target 30 deg clockwise of the heading: a RIGHT turn = negative yaw target. */
    assert(o.heading_error_deg > 25.0f && o.yaw_target_dps < -14.9f);
    /* Across north: target 350, heading 10 -> turn LEFT (positive). */
    yaw_heading_control_reset(&ctl);
    for (int i = 0; i < 20; ++i) o = step(&ctl, 10.0f, 350.0f, true, 0.0f, 0.40f);
    assert(o.heading_error_deg < -15.0f && o.yaw_target_dps > 14.9f);
}

static void a_steering_input_does_not_suspend_a_mission(void)
{
    yaw_heading_control_t ctl; yaw_heading_control_init(&ctl);
    yaw_heading_output_t o = step(&ctl, 90.0f, 90.0f, true, 0.5f, 0.40f);
    assert(o.active && o.heading_hold && !ctl.steering_suspended);
    /* ...the same input without the mission is a manual turn and suspends. */
    yaw_heading_control_reset(&ctl);
    o = step(&ctl, 90.0f, 90.0f, false, 0.5f, 0.40f);
    assert(!o.active && ctl.steering_suspended);
}

static void a_moved_target_is_followed_without_a_recapture_delay(void)
{
    yaw_heading_control_t ctl; yaw_heading_control_init(&ctl);
    for (int i = 0; i < 10; ++i) step(&ctl, 90.0f, 90.0f, true, 0.0f, 0.40f);
    const yaw_heading_output_t o = step(&ctl, 90.0f, 200.0f, true, 0.0f, 0.40f);
    assert(fabsf(o.heading_target_deg - 200.0f) < 1e-4f && o.heading_hold);
    /* The error filter slews the jump instead of stepping it. */
    assert(o.heading_error_deg > 0.0f && o.heading_error_deg < 110.0f);
}

static void the_safety_gates_still_apply(void)
{
    yaw_heading_control_t ctl; yaw_heading_control_init(&ctl);
    step(&ctl, 90.0f, 120.0f, true, 0.0f, 0.40f);
    const yaw_heading_output_t o = step(&ctl, 90.0f, 120.0f, true, 0.0f, 0.10f);  /* under 0.15 */
    assert(!o.active && !ctl.initialized);
    /* After the reset, the mission target is used again -- never the heading. */
    const yaw_heading_output_t p = step(&ctl, 90.0f, 120.0f, true, 0.0f, 0.40f);
    assert(fabsf(p.heading_target_deg - 120.0f) < 1e-4f);
    /* A non-finite target resets instead of steering somewhere random. */
    const yaw_heading_output_t q = step(&ctl, 90.0f, NAN, true, 0.0f, 0.40f);
    assert(!q.active && !ctl.initialized);
}

static void releasing_the_mission_keeps_the_last_target_until_reset(void)
{
    /* Documents WHY the control task resets the hold when a mission lets go:
     * without it, manual P would keep steering to the mission's last target. */
    yaw_heading_control_t ctl; yaw_heading_control_init(&ctl);
    for (int i = 0; i < 10; ++i) step(&ctl, 90.0f, 150.0f, true, 0.0f, 0.40f);
    const yaw_heading_output_t o = step(&ctl, 90.0f, 0.0f, false, 0.0f, 0.40f);
    assert(o.heading_hold && fabsf(o.heading_target_deg - 150.0f) < 1e-4f);
    yaw_heading_control_reset(&ctl);
    const yaw_heading_output_t p = step(&ctl, 90.0f, 0.0f, false, 0.0f, 0.40f);
    assert(fabsf(p.heading_target_deg - 90.0f) < 1e-4f);    /* captures the heading */
}

int main(void)
{
    the_target_is_used_at_once_not_captured();
    a_steering_input_does_not_suspend_a_mission();
    a_moved_target_is_followed_without_a_recapture_delay();
    the_safety_gates_still_apply();
    releasing_the_mission_keeps_the_last_target_until_reset();
    printf("yaw heading mission-mode tests passed\n");
    return 0;
}
