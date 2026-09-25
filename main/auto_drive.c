#include "auto_drive.h"

#include <math.h>
#include <string.h>

#include "esc_trim.h"

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float wrap_360(float deg)
{
    float w = fmodf(deg, 360.0f);
    if (w < 0.0f) w += 360.0f;
    if (w >= 360.0f) w -= 360.0f;
    return w;
}

void auto_drive_init(auto_drive_t *st)
{
    if (st) memset(st, 0, sizeof(*st));
}

auto_out_t auto_drive_step(auto_drive_t *st, const auto_setpoint_t *sp,
                           const auto_inputs_t *in)
{
    auto_out_t out = {0};
    if (!st || !sp || !in) return out;
    if (sp->run_id == 0u || !sp->active) return out;       /* nothing to own */
    if (sp->run_id == st->aborted_run_id) return out;       /* this run is over */

    /* Order = what the operator did first, then the boat's own state. */
    auto_abort_t why = AUTO_ABORT_NONE;
    if (in->stop_request) why = AUTO_ABORT_STOP;
    else if (in->manual_input) why = AUTO_ABORT_MANUAL;
    else if (in->rail_cut) why = AUTO_ABORT_RAIL_CUT;
    else if (in->other_owner) why = AUTO_ABORT_OTHER_OWNER;
    else if (!sp->dry_run && !in->armed) why = AUTO_ABORT_DISARMED;
    else if (in->now_us - sp->stamp_us > AUTO_SETPOINT_MAX_AGE_US) why = AUTO_ABORT_STALE;

    if (why != AUTO_ABORT_NONE) {
        st->aborted_run_id = sp->run_id;
        st->abort_reason = why;
        out.abort = why;
        return out;
    }

    out.own = true;
    out.drive = sp->drive && !sp->dry_run && in->armed &&
                isfinite(sp->throttle) && sp->throttle > 0.0f;
    out.throttle = out.drive ? clampf(sp->throttle, 0.0f, 1.0f) : 0.0f;
    out.heading_deg = isfinite(sp->heading_deg) ? wrap_360(sp->heading_deg) : 0.0f;
    return out;
}

void auto_drive_hold_input(const auto_out_t *out, yaw_heading_input_t *in)
{
    if (!out || !in) return;
    in->enabled = true;
    in->steering = 0.0f;
    in->base_capture_now = false;
    in->throttle = (in->driving && out->drive) ? out->throttle : 0.0f;
    in->mission_owned = true;
    in->target_heading_deg = out->heading_deg;
}

void auto_drive_mix(float throttle, float learned_c, float dynamic_c,
                    bool hold_active, float *left, float *right)
{
    if (!left || !right) return;
    const float t = clampf(isfinite(throttle) ? throttle : 0.0f, 0.0f, 1.0f);
    float c = isfinite(learned_c) ? learned_c : 0.0f;
    if (hold_active && isfinite(dynamic_c)) {
        const float c_limit = (t > 0.0f) ? clampf((1.0f / t) - 1.0f, 0.0f, 1.0f) : 0.0f;
        c = clampf(c + dynamic_c, -c_limit, c_limit);
    }
    const EscTrimPoint pt = { .throttle_frac = 1.0f, .trim_diff = 2.0f * c * t };
    esc_trim_mix(t, 0.0f, &pt, 1, left, right);
}
