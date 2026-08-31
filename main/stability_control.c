#include "stability_control.h"
#include <math.h>

static float clampf(float value, float lo, float hi)
{
    return (value < lo) ? lo : (value > hi ? hi : value);
}

void stab_reset(stab_state_t *state)
{
    state->yaw_filt = 0.0f;
    state->initialized = false;
}

float stab_rudder_update(stab_state_t *state, const stab_cfg_t *cfg,
                         float dt_s, float steer_cmd_norm, float yaw_rate_dps)
{
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
        float alpha = dt_s / (cfg->yaw_tau_s + dt_s);
        state->yaw_filt += alpha * (yaw_rate_dps - state->yaw_filt);
    }

    float rate_command = clampf(steer_cmd_norm, -1.0f, 1.0f) * cfg->r_max_dps;
    float rudder = cfg->k_r * (rate_command - state->yaw_filt);
    return clampf(rudder, -cfg->out_cap, cfg->out_cap);
}
