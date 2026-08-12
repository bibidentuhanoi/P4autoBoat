#pragma once

#include <stdbool.h>

typedef struct {
    float r_max_dps;  /* stick=1.0 commands this yaw rate (deg/s) */
    float k_r;        /* rudder-normalized per (deg/s) of rate error */
    float yaw_lpf;    /* 0..1 low-pass factor on measured yaw rate */
    float out_cap;    /* max |rudder| the loop may command (0..1) */
} stab_cfg_t;

typedef struct {
    float yaw_filt;
    bool initialized;
} stab_state_t;

void stab_reset(stab_state_t *state);

/* steer_cmd_norm is the pilot's [-1, 1] yaw-rate demand. The function only
 * updates controller state and returns a capped rudder command. */
float stab_rudder_update(stab_state_t *state, const stab_cfg_t *cfg,
                         float steer_cmd_norm, float yaw_rate_dps);
