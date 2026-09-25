/* Golden-test driver for tests/test_yaw_heading_golden.py.  Reads one
 * controller input per line from stdin, prints every output field and the
 * whole controller state as hex floats, so two builds of
 * yaw_heading_control.c can be compared bit for bit.  It never sets the
 * 2026-09-25 mission fields, so it also builds against the old header. */
#include <stdio.h>

#include "yaw_heading_control.h"

int main(void)
{
    const yaw_heading_cfg_t cfg = {
        .yaw_tau_s = 0.08f, .rate_kp = 0.100f, .rate_ki = 0.080f,
        .heading_tau_s = 0.10f, .heading_kp = 1.50f,
        .max_yaw_target_dps = 15.0f, .min_throttle = 0.15f,
        .steering_deadband = 0.02f, .recapture_delay_s = 0.50f,
    };
    yaw_heading_control_t ctl;
    yaw_heading_control_init(&ctl);
    float dt, yaw, hdg, thr, ff, steer;
    int valid, en, drv, fresh, base, reset;
    while (scanf("%f %f %f %d %f %f %f %d %d %d %d %d", &dt, &yaw, &hdg, &valid, &thr, &ff,
                 &steer, &en, &drv, &fresh, &base, &reset) == 12) {
        if (reset) yaw_heading_control_reset(&ctl);
        const yaw_heading_input_t in = {
            .dt_s = dt, .yaw_rate_dps = yaw, .heading_deg = hdg,
            .throttle = thr, .feedforward_c = ff, .steering = steer,
            .enabled = en, .driving = drv, .gyro_fresh = fresh,
            .heading_valid = valid, .base_capture_now = base,
        };
        const yaw_heading_output_t o = yaw_heading_control_update(&ctl, &cfg, &in);
        printf("%d %d %d %a %a %a %a %a %a %a %a %a %a | %d %d %d %a %a %a %a %a\n",
               o.active, o.heading_hold, o.saturated, o.heading_target_deg,
               o.heading_error_deg, o.yaw_target_dps, o.yaw_filt_dps,
               o.rate_error_dps, o.p_term, o.i_term, o.dynamic_c,
               o.effective_c, o.c_limit,
               ctl.initialized, ctl.heading_hold, ctl.steering_suspended,
               ctl.yaw_filt, ctl.heading_target, ctl.heading_error_filt,
               ctl.integral, ctl.recapture_elapsed_s);
    }
    return 0;
}
