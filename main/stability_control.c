#include "stability_control.h"
#include <math.h>

static float clampf(float value, float lo, float hi)
{
    return (value < lo) ? lo : (value > hi ? hi : value);
}

void stab_reset(stab_state_t *state)
{
    state->yaw_filt = 0.0f;
    state->rudder = 0.0f;
    state->initialized = false;
}

float stab_target_dps(const stab_cfg_t *cfg, float steer_cmd_norm)
{
    if (!cfg || !isfinite(steer_cmd_norm)) return 0.0f;
    /* THE flip. Stick -1 is physical LEFT, and physical left produces POSITIVE
     * yaw, so a left stick asks for a POSITIVE rate. Getting this backwards is
     * what made the old loop positive-feedback. */
    return -clampf(steer_cmd_norm, -1.0f, 1.0f) * cfg->r_max_dps;
}

float stab_rudder_update(stab_state_t *state, const stab_cfg_t *cfg,
                         float dt_s, float steer_cmd_norm, float yaw_rate_dps,
                         stab_debug_t *dbg)
{
    if (dbg) {
        const stab_debug_t zero = {0};
        *dbg = zero;
    }
    if (!state || !cfg) return 0.0f;

    /* Fail closed on a non-finite input rather than let it propagate: NaN
     * survives clampf() unclamped (every comparison with NaN is false), and
     * (uint32_t)NaN in steer_to_us() is undefined behaviour feeding a real
     * servo. yaw_rate_dps can go bad from a corrupted sensor read;
     * steer_cmd_norm from a malformed network float -- neither is defended
     * upstream, so this pure function is the one place that must catch both. */
    if (!isfinite(yaw_rate_dps) || !isfinite(steer_cmd_norm) || !isfinite(dt_s)) {
        stab_reset(state);
        return 0.0f;
    }

    if (!state->initialized || dt_s <= 0.0f) {
        state->yaw_filt = yaw_rate_dps;   /* snap on first sample / non-positive dt */
        state->initialized = true;
    } else {
        /* Discrete first-order low-pass, dt-aware: alpha = dt/(tau+dt). Same
         * steady-state behaviour as the classic exp(-dt/tau) form without
         * needing expf() on this target, and stable for any dt_s >= 0. */
        const float alpha = dt_s / (cfg->yaw_tau_s + dt_s);
        state->yaw_filt += alpha * (yaw_rate_dps - state->yaw_filt);
    }

    const float target = stab_target_dps(cfg, steer_cmd_norm);
    const float error = target - state->yaw_filt;

    /* Feedforward picked by the direction of the TARGET, not of the error: the
     * hull turns at different rates for the same rudder each way, so the gain
     * belongs to the way we are asking it to go. */
    const float ff_gain = (target >= 0.0f) ? cfg->ff_left : cfg->ff_right;

    /* Both terms negated, for one physical reason: rudder and yaw carry
     * opposite signs on this boat (see the header). To rotate the hull one way
     * the rudder goes the other. */
    const float ff_term = -ff_gain * target;
    const float p_term = -cfg->k_p * error;
    const float raw = ff_term + p_term;

    const float cap = fabsf(cfg->out_cap);
    float out = clampf(raw, -cap, cap);
    const bool saturated = (cap > 0.0f) && (fabsf(raw) > cap);

    /* Slew AFTER the cap, so the limiter walks toward the value that will
     * actually be commanded rather than chasing one the cap is about to
     * discard. dt_s <= 0 is the post-reset snap: let it take the value
     * outright, or the very first command would be limited against a stale
     * previous output. */
    bool slewed = false;
    if (cfg->slew_per_s > 0.0f && state->initialized && dt_s > 0.0f) {
        const float step = cfg->slew_per_s * dt_s;
        const float delta = out - state->rudder;
        if (delta > step) {
            out = state->rudder + step;
            slewed = true;
        } else if (delta < -step) {
            out = state->rudder - step;
            slewed = true;
        }
    }
    state->rudder = out;

    if (dbg) {
        dbg->target_dps = target;
        dbg->yaw_filt = state->yaw_filt;
        dbg->ff_term = ff_term;
        dbg->p_term = p_term;
        dbg->raw = raw;
        dbg->saturated = saturated;
        dbg->slewed = slewed;
    }
    return out;
}
