#include "course_error.h"

#include <math.h>
#include <string.h>

#define CE_DEG2RAD 0.017453292519943295
#define CE_RAD2DEG 57.29577951308232

static float wrap180(float deg)
{
    float w = fmodf(deg + 180.0f, 360.0f);
    if (w < 0.0f) w += 360.0f;
    return w - 180.0f;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

course_err_cfg_t course_err_cfg_default(void)
{
    course_err_cfg_t c = {
        .min_speed_mps = 0.25f,
        /* 45, not 15: the u-blox sAcc on this boat is unrecorded and may be
         * pessimistic; the lake course scatter was 2.5-12 deg.  The mean
         * (>= 50 samples, std gate) and the 8 s follow average the noise:
         * the simulator gets home as well with honest 0.3 m/s noise and
         * keeps beta with sAcc up to 0.2 m/s (15 lost it from 0.1). */
        .max_course_err_deg = 45.0f,
        .max_yaw_dps = 5.0f,
        .min_samples = 50u,
        .min_travel_m = 4.0f,
        .max_std_deg = 25.0f,
        .clamp_deg = 60.0f,
        .bad_deg = 45.0f,
        .follow_tau_s = 8.0f,
    };
    return c;
}

bool course_err_sample_ok(const course_err_cfg_t *cfg, float speed_mps,
                          float speed_acc_mps, float yaw_dps)
{
    if (!cfg || !isfinite(speed_mps) || !isfinite(speed_acc_mps) || !isfinite(yaw_dps)) {
        return false;
    }
    if (speed_mps < cfg->min_speed_mps) return false;
    if (fabsf(yaw_dps) > cfg->max_yaw_dps) return false;
    if (!(speed_acc_mps > 0.0f)) return false;
    const float ratio = speed_acc_mps / speed_mps;
    if (ratio >= 1.0f) return false;
    return (float)(asin(ratio) * CE_RAD2DEG) < cfg->max_course_err_deg;
}

float course_err_sample(float course_deg, float heading_deg)
{
    return wrap180(course_deg - heading_deg);
}

void course_err_mean_reset(course_err_mean_t *m)
{
    if (m) memset(m, 0, sizeof(*m));
}

void course_err_mean_add(course_err_mean_t *m, float sample_deg, float e, float n)
{
    if (!m || !isfinite(sample_deg)) return;
    m->sum_sin += sin((double)sample_deg * CE_DEG2RAD);
    m->sum_cos += cos((double)sample_deg * CE_DEG2RAD);
    m->n++;
    if (m->have_last) m->travel_m += hypotf(e - m->last_e, n - m->last_n);
    m->last_e = e;
    m->last_n = n;
    m->have_last = true;
}

course_err_result_t course_err_mean_result(const course_err_mean_t *m,
                                           const course_err_cfg_t *cfg)
{
    course_err_result_t r = {0};
    if (!m || !cfg || m->n == 0) return r;
    const double s = m->sum_sin / m->n, c = m->sum_cos / m->n;
    const double R = sqrt(s * s + c * c);
    r.n = m->n;
    r.travel_m = m->travel_m;
    r.mean_deg = (float)(atan2(s, c) * CE_RAD2DEG);
    r.std_deg = (R > 1e-9) ? (float)(sqrt(-2.0 * log(R)) * CE_RAD2DEG) : 180.0f;
    r.valid = m->n >= cfg->min_samples && m->travel_m >= cfg->min_travel_m &&
              r.std_deg < cfg->max_std_deg;
    r.compass_bad = r.valid && fabsf(r.mean_deg) > cfg->bad_deg;
    if (r.valid) r.mean_deg = clampf(r.mean_deg, -cfg->clamp_deg, cfg->clamp_deg);
    return r;
}

float course_err_follow(float current_deg, float sample_deg, float dt_s,
                        const course_err_cfg_t *cfg)
{
    if (!cfg || !isfinite(current_deg)) current_deg = 0.0f;
    if (!cfg || !isfinite(sample_deg) || !isfinite(dt_s) || dt_s <= 0.0f) {
        return current_deg;
    }
    const float tau = cfg->follow_tau_s > 0.0f ? cfg->follow_tau_s : 0.0f;
    const float alpha = dt_s / (tau + dt_s);
    const float next = current_deg + alpha * wrap180(sample_deg - current_deg);
    return clampf(wrap180(next), -cfg->clamp_deg, cfg->clamp_deg);
}
