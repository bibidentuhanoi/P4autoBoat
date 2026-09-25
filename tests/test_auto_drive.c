#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "auto_drive.h"

static int close_to(double a, double b, double eps) { return fabs(a - b) <= eps; }

static auto_setpoint_t driving(uint32_t run, int64_t stamp)
{
    auto_setpoint_t sp = { .run_id = run, .active = true, .drive = true, .dry_run = false,
                           .heading_deg = 90.0f, .throttle = 0.40f, .stamp_us = stamp };
    return sp;
}

static auto_inputs_t calm(int64_t now)
{
    auto_inputs_t in = { .now_us = now, .armed = true };
    return in;
}

static void it_owns_and_drives_a_fresh_armed_run(void)
{
    auto_drive_t st; auto_drive_init(&st);
    auto_setpoint_t sp = driving(1, 1000);
    auto_out_t o = auto_drive_step(&st, &sp, &(auto_inputs_t){ .now_us = 2000, .armed = true });
    assert(o.own && o.drive && close_to(o.throttle, 0.40, 1e-6) && close_to(o.heading_deg, 90.0, 1e-6));
    assert(o.abort == AUTO_ABORT_NONE);
    /* HOME: owned, held at 0. */
    sp.drive = false;
    o = auto_drive_step(&st, &sp, &(auto_inputs_t){ .now_us = 2000, .armed = true });
    assert(o.own && !o.drive && o.throttle == 0.0f);
    /* Nothing to own. */
    sp.active = false;
    assert(!auto_drive_step(&st, &sp, &(auto_inputs_t){ .now_us = 2000, .armed = true }).own);
    sp = driving(0, 1000);
    assert(!auto_drive_step(&st, &sp, &(auto_inputs_t){ .now_us = 2000, .armed = true }).own);
}

static void every_stop_reason_latches_for_that_run_only(void)
{
    struct { auto_inputs_t in; auto_abort_t want; } cases[] = {
        { { .now_us = 2000, .armed = true, .stop_request = true }, AUTO_ABORT_STOP },
        { { .now_us = 2000, .armed = true, .manual_input = true }, AUTO_ABORT_MANUAL },
        { { .now_us = 2000, .armed = true, .rail_cut = true }, AUTO_ABORT_RAIL_CUT },
        { { .now_us = 2000, .armed = true, .other_owner = true }, AUTO_ABORT_OTHER_OWNER },
        { { .now_us = 2000, .armed = false }, AUTO_ABORT_DISARMED },
        { { .now_us = 1000 + AUTO_SETPOINT_MAX_AGE_US + 1, .armed = true }, AUTO_ABORT_STALE },
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        auto_drive_t st; auto_drive_init(&st);
        auto_setpoint_t sp = driving(7, 1000);
        auto_out_t o = auto_drive_step(&st, &sp, &cases[i].in);
        assert(!o.own && o.abort == cases[i].want);
        assert(st.aborted_run_id == 7u && st.abort_reason == cases[i].want);
        /* Latched: the same run never comes back, even with calm inputs. */
        o = auto_drive_step(&st, &sp, &(auto_inputs_t){ .now_us = 1500, .armed = true });
        assert(!o.own && o.abort == AUTO_ABORT_NONE);
        /* A NEW run is owned again. */
        sp = driving(8, 1000);
        assert(auto_drive_step(&st, &sp, &(auto_inputs_t){ .now_us = 1500, .armed = true }).own);
    }
    /* Exactly at the age limit is still fresh. */
    auto_drive_t st; auto_drive_init(&st);
    auto_setpoint_t sp = driving(3, 1000);
    assert(auto_drive_step(&st, &sp, &(auto_inputs_t){ .now_us = 1000 + AUTO_SETPOINT_MAX_AGE_US, .armed = true }).own);
    (void)calm;
}

static void a_dry_run_never_drives_and_ignores_the_arm_state(void)
{
    auto_drive_t st; auto_drive_init(&st);
    auto_setpoint_t sp = driving(2, 1000);
    sp.dry_run = true;
    auto_out_t o = auto_drive_step(&st, &sp, &(auto_inputs_t){ .now_us = 2000, .armed = false });
    assert(o.own && !o.drive && o.throttle == 0.0f && o.abort == AUTO_ABORT_NONE);
}

static void the_hold_input_is_mission_owned_with_the_same_gates(void)
{
    yaw_heading_input_t in = { .dt_s = 0.02f, .yaw_rate_dps = 1.0f, .heading_deg = 10.0f,
                               .throttle = 0.0f, .feedforward_c = 0.21f, .steering = 1.0f,
                               .enabled = false, .driving = true, .gyro_fresh = true,
                               .heading_valid = true, .base_capture_now = true };
    const auto_out_t o = { .own = true, .drive = true, .throttle = 0.40f, .heading_deg = 123.0f };
    auto_drive_hold_input(&o, &in);
    assert(in.enabled && in.mission_owned && in.steering == 0.0f && !in.base_capture_now);
    assert(close_to(in.throttle, 0.40, 1e-6) && close_to(in.target_heading_deg, 123.0, 1e-6));
    assert(in.gyro_fresh && in.heading_valid);              /* untouched safety gates */
    in.driving = false;                                      /* disarmed: no throttle */
    auto_drive_hold_input(&o, &in);
    assert(in.throttle == 0.0f);
}

static void the_mix_matches_the_manual_path(void)
{
    float L, R;
    /* No hold: the learned trim alone, T*(1-c) / T*(1+c). */
    auto_drive_mix(0.40f, 0.21f, 0.5f, false, &L, &R);
    assert(close_to(L, 0.40 * 0.79, 1e-5) && close_to(R, 0.40 * 1.21, 1e-5));
    /* With the hold: c + dynamic, limited to 1/T - 1 (capped at 1). */
    auto_drive_mix(0.40f, 0.21f, 0.30f, true, &L, &R);
    assert(close_to(L, 0.40 * 0.49, 1e-5) && close_to(R, 0.40 * 1.51, 1e-5));
    auto_drive_mix(0.40f, 0.21f, 2.0f, true, &L, &R);     /* saturates at c = 1 */
    assert(close_to(L, 0.0, 1e-6) && close_to(R, 0.80, 1e-5));
    auto_drive_mix(0.80f, 0.21f, 2.0f, true, &L, &R);     /* headroom 0.25 at T80 */
    assert(close_to(L, 0.80 * 0.75, 1e-5) && close_to(R, 1.0, 1e-5));
    auto_drive_mix(0.0f, 0.21f, 0.3f, true, &L, &R);      /* stopped stays stopped */
    assert(L == 0.0f && R == 0.0f);
}

int main(void)
{
    it_owns_and_drives_a_fresh_armed_run();
    every_stop_reason_latches_for_that_run_only();
    a_dry_run_never_drives_and_ignores_the_arm_state();
    the_hold_input_is_mission_owned_with_the_same_gates();
    the_mix_matches_the_manual_path();
    printf("auto_drive tests passed\n");
    return 0;
}
