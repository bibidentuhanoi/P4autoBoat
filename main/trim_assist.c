#include "trim_assist.h"
#include <math.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void trim_assist_reset(trim_assist_t *s)
{
    if (!s) return;
    s->yaw_filt = 0.0f;
    s->initialized = false;
    s->correction = 0.0f;
    s->at_cap = false;
}

float trim_assist_update(trim_assist_t *s, const trim_assist_cfg_t *cfg,
                         float dt_s, float yaw_rate_dps, bool gate)
{
    if (!s) return 0.0f;
    if (!cfg || !gate) {
        trim_assist_reset(s);
        return 0.0f;
    }
    /* A NaN here would survive every clamp (all comparisons with NaN are
     * false) and reach the ESC mixer. Reset rather than propagate. */
    if (!isfinite(yaw_rate_dps) || !isfinite(dt_s) ||
        !isfinite(cfg->kp) || !isfinite(cfg->tau_s) || !isfinite(cfg->cap)) {
        trim_assist_reset(s);
        return 0.0f;
    }

    if (!s->initialized || dt_s <= 0.0f) {
        /* Snap on the first sample, and apply NOTHING from it: one raw gyro
         * reading is the outlier the filter exists to reject, and acting on
         * it would put a step into the motors at every gate transition. */
        s->yaw_filt = yaw_rate_dps;
        s->initialized = true;
        s->correction = 0.0f;
        s->at_cap = false;
        return 0.0f;
    }

    const float alpha = dt_s / (cfg->tau_s + dt_s);
    s->yaw_filt += alpha * (yaw_rate_dps - s->yaw_filt);

    const float cap = fabsf(cfg->cap);
    const float raw = -cfg->kp * s->yaw_filt;
    s->correction = clampf(raw, -cap, cap);
    s->at_cap = (cap > 0.0f) && (fabsf(raw) >= cap);
    return s->correction;
}

float trim_assist_effective_c(float learned_c, float correction,
                              float c_min, float c_max)
{
    if (!isfinite(learned_c)) return 0.0f;
    if (!isfinite(correction)) correction = 0.0f;
    return clampf(learned_c + correction, c_min, c_max);
}
