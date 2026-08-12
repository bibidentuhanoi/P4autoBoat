#include <assert.h>
#include <math.h>

#include "stability_control.h"

int main(void)
{
    stab_cfg_t cfg = {
        .r_max_dps = 45.0f,
        .k_r = 0.010f,
        .yaw_tau_s = 0.10f,
        .out_cap = 0.50f,
    };
    stab_state_t state;

    /* Removing the rate error term would no longer oppose an unwanted turn.
     * dt=0 snaps the filter straight to the measurement (first-sample case). */
    stab_reset(&state);
    float rudder = stab_rudder_update(&state, &cfg, 0.0f, 0.0f, 20.0f);
    assert(fabsf(rudder - (-0.20f)) < 1e-4f);

    /* Removing the pilot demand term would make deliberate turns impossible. */
    stab_reset(&state);
    rudder = stab_rudder_update(&state, &cfg, 0.0f, 1.0f, 0.0f);
    assert(fabsf(rudder - 0.45f) < 1e-4f);

    /* An omitted authority clamp would allow an unsafe rudder command. */
    stab_reset(&state);
    rudder = stab_rudder_update(&state, &cfg, 0.0f, 0.0f, 10000.0f);
    assert(fabsf(rudder + 0.50f) < 1e-6f);

    /* The normalized pilot input must retain its API boundary. */
    stab_reset(&state);
    rudder = stab_rudder_update(&state, &cfg, 0.0f, 5.0f, 0.0f);
    assert(fabsf(rudder - 0.45f) < 1e-4f);

    /* A filter that ignored elapsed time would give the same blend regardless
     * of dt, hiding gain changes when the fusion tick rate varies. With
     * tau=0.10s, a dt=0.10s step must blend exactly halfway
     * (alpha = dt/(tau+dt) = 0.5) toward the new measurement. */
    stab_reset(&state);
    stab_rudder_update(&state, &cfg, 0.0f, 0.0f, 0.0f);        /* prime filter at 0 */
    rudder = stab_rudder_update(&state, &cfg, 0.10f, 0.0f, 20.0f);
    float expected_filt = 0.5f * 20.0f;                        /* alpha=0.5 blend from 0 */
    assert(fabsf(rudder - (-0.010f * expected_filt)) < 1e-3f);
    return 0;
}
