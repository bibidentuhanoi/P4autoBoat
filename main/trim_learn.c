#include "trim_learn.h"

#include <math.h>
#include <string.h>

void trim_learn_init(trim_learn_t *s, const trim_learn_cfg_t *cfg)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->c = (cfg && isfinite(cfg->c_init)) ? cfg->c_init : 0.0f;
}

bool trim_learn_reset(trim_learn_t *s, const trim_learn_cfg_t *cfg, float c)
{
    if (!s || !cfg) return false;
    if (!isfinite(c) || c < cfg->c_min || c > cfg->c_max) return false;
    s->c = c;
    s->initialized = false;     /* the old yaw estimate described another trim */
    s->faulted = false;         /* a deliberate restart clears the latch */
    s->yaw_filt = 0.0f;
    return true;
}

bool trim_learn_update(trim_learn_t *s, const trim_learn_cfg_t *cfg,
                       uint32_t seq, float dt_s, float yaw_rate_dps,
                       float throttle, bool steering, bool healthy)
{
    if (!s || !cfg) return false;
    if (!isfinite(yaw_rate_dps) || !isfinite(dt_s) || !isfinite(throttle)) {
        return false;                       /* never integrate a bad number */
    }

    /* One update per FUSION SAMPLE. The control loop ticks at 100 Hz but fusion
     * publishes at 50 Hz, so acting every tick would integrate each sample
     * twice and silently double the gain. */
    if (seq == s->last_seq) return false;
    s->last_seq = seq;

    if (!healthy) return false;

    /* A commanded turn is not an error. Drop the estimate entirely rather than
     * filtering the turn in -- otherwise it leaks into the next straight run. */
    if (steering) {
        s->initialized = false;
        return false;
    }

    /* Being hit is not being mistrimmed. A wall bounce is 5-30x the imbalance
     * and lasts seconds, so it would drag the filtered estimate right through
     * the deadband and walk c somewhere arbitrary. Drop the estimate entirely
     * rather than filtering the impact in -- the same treatment a commanded
     * turn gets, and for the same reason. */
    if (cfg->reject_dps > 0.0f && fabsf(yaw_rate_dps) > cfg->reject_dps) {
        s->initialized = false;
        return false;
    }

    /* First sample, or a gap too long to be one: resync and integrate nothing.
     * The elapsed time was not spent measuring, so it must not be spent
     * correcting either. */
    if (!s->initialized || dt_s <= 0.0f || dt_s > TRIM_LEARN_MAX_DT_S) {
        s->yaw_filt = yaw_rate_dps;
        s->initialized = true;
        return false;
    }
    s->yaw_filt += (dt_s / (cfg->yaw_tau_s + dt_s)) * (yaw_rate_dps - s->yaw_filt);

    if (s->faulted) return false;
    if (throttle < cfg->min_throttle) return false;   /* floors dominate below */

    /* Deadband: once the boat is straight, the sign of the residual is just
     * noise. Without this the trim random-walks forever. */
    if (fabsf(s->yaw_filt) <= cfg->deadband_dps) return false;

    /* Direction only -- the measured noise is as large as the signal, so
     * "which way" is trustworthy and "how much" is not. Negative drift means
     * not enough trim (verified on hardware: +0.152 took -4.34 to -0.03). */
    const float before = s->c;
    const float step = cfg->step_per_s * dt_s;
    s->c += (s->yaw_filt < 0.0f) ? step : -step;

    /* A clamp hit is a FAULT, not a limit: wrong sign, blocked jet, battery
     * sag or bad gyro all land here. Latch it and stop rather than sit on the
     * rail pretending to work. */
    if (s->c <= cfg->c_min) { s->c = cfg->c_min; s->faulted = true; }
    else if (s->c >= cfg->c_max) { s->c = cfg->c_max; s->faulted = true; }

    return s->c != before;
}

float trim_learn_split(const trim_learn_t *s, float throttle)
{
    if (!s || !isfinite(throttle) || throttle <= 0.0f) return 0.0f;
    return s->c * throttle;
}
