#include "stability_control.h"

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
                         float steer_cmd_norm, float yaw_rate_dps)
{
    if (!state->initialized) {
        state->yaw_filt = yaw_rate_dps;
        state->initialized = true;
    } else {
        state->yaw_filt += cfg->yaw_lpf * (yaw_rate_dps - state->yaw_filt);
    }

    float rate_command = clampf(steer_cmd_norm, -1.0f, 1.0f) * cfg->r_max_dps;
    float rudder = cfg->k_r * (rate_command - state->yaw_filt);
    return clampf(rudder, -cfg->out_cap, cfg->out_cap);
}
