#include "yaw_heading_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float wrap_360(float angle_deg)
{
    float wrapped = fmodf(angle_deg, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    return wrapped;
}

float yaw_heading_wrap_180(float angle_deg)
{
    float wrapped = fmodf(angle_deg + 180.0f, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    return wrapped - 180.0f;
}

static float lowpass_alpha(float dt_s, float tau_s)
{
    return (tau_s <= 0.0f) ? 1.0f : clampf(dt_s / (tau_s + dt_s), 0.0f, 1.0f);
}

static float authority_limit(float throttle)
{
    if (!isfinite(throttle) || throttle <= 0.0f) return 0.0f;
    return clampf((1.0f / throttle) - 1.0f, 0.0f, 1.0f);
}

static bool config_valid(const yaw_heading_cfg_t *cfg)
{
    return cfg != NULL &&
           isfinite(cfg->yaw_tau_s) && cfg->yaw_tau_s >= 0.0f &&
           isfinite(cfg->rate_kp) && cfg->rate_kp >= 0.0f &&
           isfinite(cfg->rate_ki) && cfg->rate_ki >= 0.0f &&
           isfinite(cfg->heading_tau_s) && cfg->heading_tau_s >= 0.0f &&
           isfinite(cfg->heading_kp) && cfg->heading_kp >= 0.0f &&
           isfinite(cfg->max_yaw_target_dps) && cfg->max_yaw_target_dps >= 0.0f &&
           isfinite(cfg->min_throttle) && cfg->min_throttle >= 0.0f &&
           isfinite(cfg->steering_deadband) && cfg->steering_deadband >= 0.0f &&
           isfinite(cfg->recapture_delay_s) && cfg->recapture_delay_s >= 0.0f;
}

static yaw_heading_output_t inactive_output(const yaw_heading_input_t *in)
{
    yaw_heading_output_t out = {0};
    if (in == NULL || !isfinite(in->throttle) || !isfinite(in->feedforward_c)) {
        return out;
    }
    out.c_limit = authority_limit(in->throttle);
    out.effective_c = clampf(in->feedforward_c, -out.c_limit, out.c_limit);
    return out;
}

void yaw_heading_control_reset(yaw_heading_control_t *ctl)
{
    if (ctl != NULL) memset(ctl, 0, sizeof(*ctl));
}

void yaw_heading_control_init(yaw_heading_control_t *ctl)
{
    yaw_heading_control_reset(ctl);
}

static void capture_heading(yaw_heading_control_t *ctl, float heading_deg)
{
    ctl->heading_target = wrap_360(heading_deg);
    ctl->heading_error_filt = 0.0f;
    ctl->heading_hold = true;
    ctl->recapture_elapsed_s = 0.0f;
}

yaw_heading_output_t yaw_heading_control_update(
    yaw_heading_control_t *ctl,
    const yaw_heading_cfg_t *cfg,
    const yaw_heading_input_t *in)
{
    yaw_heading_output_t out = inactive_output(in);
    if (ctl == NULL || !config_valid(cfg) || in == NULL ||
        !isfinite(in->dt_s) || in->dt_s <= 0.0f ||
        !isfinite(in->yaw_rate_dps) || !isfinite(in->heading_deg) ||
        !isfinite(in->throttle) || !isfinite(in->feedforward_c) ||
        !isfinite(in->steering)) {
        yaw_heading_control_reset(ctl);
        return out;
    }

    const bool hard_gate = in->enabled && in->driving && in->gyro_fresh &&
                           in->throttle >= cfg->min_throttle;
    if (!hard_gate) {
        yaw_heading_control_reset(ctl);
        return out;
    }

    const bool steering = fabsf(in->steering) > cfg->steering_deadband;
    if (steering) {
        /* Deliberate turns own the boat.  Preserve only the mismatch estimate. */
        ctl->initialized = true;
        ctl->yaw_filt = in->yaw_rate_dps;
        ctl->heading_hold = false;
        ctl->steering_suspended = true;
        ctl->recapture_elapsed_s = 0.0f;
        out.i_term = ctl->integral;
        return out;
    }

    const bool first_active = !ctl->initialized;
    if (first_active) {
        ctl->initialized = true;
        ctl->yaw_filt = in->yaw_rate_dps;
        ctl->integral = 0.0f;
        ctl->heading_hold = false;
        ctl->heading_error_filt = 0.0f;
        ctl->recapture_elapsed_s = 0.0f;
    } else if (ctl->steering_suspended) {
        /* Resume rate damping immediately without carrying turn-rate history. */
        ctl->yaw_filt = in->yaw_rate_dps;
    } else {
        const float yaw_alpha = lowpass_alpha(in->dt_s, cfg->yaw_tau_s);
        ctl->yaw_filt += yaw_alpha * (in->yaw_rate_dps - ctl->yaw_filt);
    }

    if (!in->heading_valid) {
        ctl->heading_hold = false;
        ctl->heading_error_filt = 0.0f;
        ctl->recapture_elapsed_s = 0.0f;
    } else if (first_active || in->base_capture_now) {
        capture_heading(ctl, in->heading_deg);
        ctl->steering_suspended = false;
    } else if (ctl->steering_suspended) {
        ctl->recapture_elapsed_s += in->dt_s;
        if (ctl->recapture_elapsed_s + 1e-6f >= cfg->recapture_delay_s) {
            capture_heading(ctl, in->heading_deg);
            ctl->steering_suspended = false;
        }
    } else if (!ctl->heading_hold) {
        /* Heading has just recovered after an invalid interval. */
        capture_heading(ctl, in->heading_deg);
    }

    float yaw_target = 0.0f;
    if (ctl->heading_hold) {
        const float raw_error = yaw_heading_wrap_180(ctl->heading_target - in->heading_deg);
        const float heading_alpha = lowpass_alpha(in->dt_s, cfg->heading_tau_s);
        const float filter_delta = yaw_heading_wrap_180(raw_error - ctl->heading_error_filt);
        ctl->heading_error_filt = yaw_heading_wrap_180(
            ctl->heading_error_filt + heading_alpha * filter_delta);
        yaw_target = clampf(cfg->heading_kp * ctl->heading_error_filt,
                            -cfg->max_yaw_target_dps,
                            cfg->max_yaw_target_dps);
    }

    out.active = true;
    out.heading_hold = ctl->heading_hold;
    out.heading_target_deg = ctl->heading_target;
    out.heading_error_deg = ctl->heading_error_filt;
    out.yaw_target_dps = yaw_target;
    out.yaw_filt_dps = ctl->yaw_filt;
    out.rate_error_dps = yaw_target - ctl->yaw_filt;
    out.p_term = cfg->rate_kp * out.rate_error_dps;
    out.c_limit = authority_limit(in->throttle);

    const float dynamic_min = -out.c_limit - in->feedforward_c;
    const float dynamic_max = +out.c_limit - in->feedforward_c;
    const float candidate_i = ctl->integral +
                              cfg->rate_ki * out.rate_error_dps * in->dt_s;
    const float candidate_dynamic = out.p_term + candidate_i;
    const bool above = candidate_dynamic > dynamic_max;
    const bool below = candidate_dynamic < dynamic_min;
    if ((!above && !below) ||
        (above && out.rate_error_dps < 0.0f) ||
        (below && out.rate_error_dps > 0.0f)) {
        ctl->integral = candidate_i;
    }

    /* Back-calculate to a realizable I state at the current throttle. */
    ctl->integral = clampf(ctl->integral,
                           dynamic_min - out.p_term,
                           dynamic_max - out.p_term);
    const float unsaturated = out.p_term + ctl->integral;
    out.dynamic_c = clampf(unsaturated, dynamic_min, dynamic_max);
    out.saturated = above || below || fabsf(out.dynamic_c - unsaturated) > 1e-6f;
    out.i_term = ctl->integral;
    out.effective_c = clampf(in->feedforward_c + out.dynamic_c,
                             -out.c_limit, out.c_limit);
    return out;
}

