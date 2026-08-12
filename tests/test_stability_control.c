#include <assert.h>
#include <math.h>

#include "stability_control.h"

int main(void)
{
    stab_cfg_t cfg = {
        .r_max_dps = 45.0f,
        .k_r = 0.010f,
        .yaw_lpf = 1.0f,
        .out_cap = 0.50f,
    };
    stab_state_t state;

    /* Removing the rate error term would no longer oppose an unwanted turn. */
    stab_reset(&state);
    float rudder = stab_rudder_update(&state, &cfg, 0.0f, 20.0f);
    assert(fabsf(rudder - (-0.20f)) < 1e-4f);

    /* Removing the pilot demand term would make deliberate turns impossible. */
    stab_reset(&state);
    rudder = stab_rudder_update(&state, &cfg, 1.0f, 0.0f);
    assert(fabsf(rudder - 0.45f) < 1e-4f);

    /* An omitted authority clamp would allow an unsafe rudder command. */
    stab_reset(&state);
    rudder = stab_rudder_update(&state, &cfg, 0.0f, 10000.0f);
    assert(fabsf(rudder + 0.50f) < 1e-6f);

    /* The normalized pilot input must retain its API boundary. */
    stab_reset(&state);
    rudder = stab_rudder_update(&state, &cfg, 5.0f, 0.0f);
    assert(fabsf(rudder - 0.45f) < 1e-4f);
    return 0;
}
