#include "esc_trim_cal.h"
#include <math.h>
#include <string.h>

static float clampf_local(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static etc_out_t safe_out(const char *reason)
{
    etc_out_t o = { .aborted = true, .reason = reason };
    return o;
}

static void reset_acc(etc_t *s)
{
    s->acc_sum = 0.0;
    s->acc_sumsq = 0.0;
    s->acc_n = 0;
}

static etc_out_t level_output(const etc_t *s, const etc_cfg_t *cfg)
{
    float level = cfg->levels[s->level_idx];
    etc_out_t o = { .active = true,
                    .left_cmd = clampf_local(level - s->trim_diff * 0.5f, 0.0f, 1.0f),
                    .right_cmd = clampf_local(level + s->trim_diff * 0.5f, 0.0f, 1.0f) };
    return o;
}

void esc_trim_cal_init(etc_t *s, const etc_cfg_t *cfg)
{
    (void)cfg;
    memset(s, 0, sizeof(*s));
    s->state = ETC_IDLE;
}

void esc_trim_cal_start(etc_t *s, const etc_cfg_t *cfg)
{
    memset(s, 0, sizeof(*s));
    if (!cfg || cfg->level_count == 0 || cfg->level_count > ESC_TRIM_MAX_POINTS) {
        s->state = ETC_ABORTED;
        return;
    }
    s->state = ETC_MEASURE_NOISE;
}

void esc_trim_cal_abort(etc_t *s)
{
    s->state = ETC_ABORTED;
}

etc_out_t esc_trim_cal_step(etc_t *s, const etc_cfg_t *cfg, int64_t now_us,
                            float yaw_rate_dps, float gps_speed_mps,
                            bool armed, bool link_alive, bool imu_ok,
                            bool manual_override)
{
    if (!s || !cfg) return safe_out("invalid");
    if (s->state == ETC_IDLE || s->state == ETC_DONE) {
        etc_out_t o = { .done = s->state == ETC_DONE };
        return o;
    }
    if (s->state == ETC_ABORTED) return safe_out("aborted");

    /* Safety gates are deliberately evaluated before any actuator output. */
    if (!armed) { s->state = ETC_ABORTED; return safe_out("disarmed"); }
    if (!link_alive) { s->state = ETC_ABORTED; return safe_out("link"); }
    if (!imu_ok) { s->state = ETC_ABORTED; return safe_out("imu"); }
    if (manual_override) { s->state = ETC_ABORTED; return safe_out("manual"); }
    if (!isfinite(yaw_rate_dps) || fabsf(yaw_rate_dps) > cfg->excessive_yaw_dps) {
        s->state = ETC_ABORTED; return safe_out("excessive yaw");
    }

    if (s->state != ETC_MEASURE_NOISE && s->phase_start_us != 0 &&
        now_us - s->phase_start_us >= cfg->level_timeout_us) {
        s->state = ETC_ABORTED; return safe_out("timeout");
    }

    if (s->state == ETC_MEASURE_NOISE) {
        if (s->phase_start_us == 0) s->phase_start_us = now_us;
        s->acc_sum += yaw_rate_dps;
        s->acc_sumsq += (double)yaw_rate_dps * yaw_rate_dps;
        if (++s->acc_n >= (cfg->noise_ticks ? cfg->noise_ticks : 1)) {
            s->b0 = (float)(s->acc_sum / s->acc_n);
            double variance = s->acc_sumsq / s->acc_n - (double)s->b0 * s->b0;
            s->sigma = variance > 0.0 ? sqrtf((float)variance) : 0.0f;
            s->state = ETC_RAMP;
            s->phase_start_us = now_us;
            s->last_tick_us = now_us;
            reset_acc(s);
        }
        return (etc_out_t){ .active = true };
    }

    if (s->state == ETC_RAMP) {
        if (now_us - s->phase_start_us >= cfg->settle_in_us) {
            s->state = ETC_SETTLE;
            s->phase_start_us = now_us;
            s->last_tick_us = now_us;
            reset_acc(s);
        }
        return level_output(s, cfg);
    }

    float dt = s->last_tick_us > 0 ? (float)(now_us - s->last_tick_us) / 1000000.0f : 0.1f;
    if (!(dt > 0.0f && dt <= 1.0f)) dt = 0.1f;
    s->last_tick_us = now_us;
    float corrected = yaw_rate_dps - s->b0;
    s->trim_diff += cfg->ki * corrected * dt;
    if (!isfinite(s->trim_diff) || fabsf(s->trim_diff) >= cfg->trim_clamp) {
        s->state = ETC_ABORTED; return safe_out("trim clamp");
    }
    s->acc_sum += corrected;
    s->acc_sumsq += (double)corrected * corrected;
    if (++s->acc_n >= (cfg->window_ticks ? cfg->window_ticks : 1)) {
        float mean = (float)(s->acc_sum / s->acc_n);
        float band = cfg->accept_k * s->sigma / sqrtf((float)s->acc_n);
        if (band < 0.01f) band = 0.01f;
        if (fabsf(mean) <= band && gps_speed_mps >= cfg->min_speed_mps) {
            s->out_table[s->out_count++] = (EscTrimPoint){
                .throttle_frac = cfg->levels[s->level_idx], .trim_diff = s->trim_diff };
            if (++s->level_idx >= cfg->level_count) {
                s->state = ETC_DONE;
                return (etc_out_t){ .done = true, .reason = "complete" };
            }
            s->trim_diff = 0.0f;
            s->state = ETC_RAMP;
            s->phase_start_us = now_us;
            reset_acc(s);
        } else {
            reset_acc(s);
        }
    }
    return level_output(s, cfg);
}
