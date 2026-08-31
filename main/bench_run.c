#include "bench_run.h"
#include "esc_trim.h"

#include <math.h>
#include <string.h>

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void bench_commands(bench_kind_t kind, float base, float delta,
                    float *left, float *right)
{
    float l = base, r = base;
    if (kind == BENCH_KIND_LEFT) {
        l = base + delta;
        r = base - delta;
    } else if (kind == BENCH_KIND_RIGHT) {
        l = base - delta;
        r = base + delta;
    }
    /* The jets are unidirectional: a command that would go negative clamps at
     * zero rather than meaning "reverse", which the hardware cannot do. */
    if (left)  *left  = clampf(l, 0.0f, 1.0f);
    if (right) *right = clampf(r, 0.0f, 1.0f);
}

float bench_baseline_yaw(const bench_t *b)
{
    if (!b || b->baseline_n < BENCH_BASELINE_MIN_N) return 0.0f;
    return b->baseline_sum / (float)b->baseline_n;
}

void bench_init(bench_t *b)
{
    if (!b) return;
    memset(b, 0, sizeof(*b));
    b->state = BENCH_IDLE;
}

bool bench_start(bench_t *b, bench_kind_t kind, float base, float delta,
                 int64_t now_us)
{
    if (!b) return false;
    if (b->state == BENCH_BASELINE || b->state == BENCH_RUN ||
        b->state == BENCH_COAST) {
        return false;                       /* a run is already going */
    }
    b->count = 0;
    b->overflow = false;
    b->baseline_sum = 0.0f;                 /* every run measures its own zero */
    b->baseline_n = 0;
    b->kind = kind;
    b->base = clampf(base, 0.0f, 1.0f);
    b->delta = clampf(delta, 0.0f, 1.0f);
    b->start_us = now_us;
    b->elapsed_s = 0.0f;
    b->state = BENCH_BASELINE;
    return true;
}

void bench_abort(bench_t *b)
{
    if (b) b->state = BENCH_FAILED;
}

static bench_out_t fail(bench_t *b, const char *reason)
{
    bench_out_t o = {0};
    b->state = BENCH_FAILED;
    o.aborted = true;
    o.reason = reason;
    return o;                               /* commands left at zero */
}

bench_out_t bench_step(bench_t *b, const bench_cfg_t *cfg, int64_t now_us,
                       float yaw_rate_dps, bool armed, float trim,
                       const bench_assist_t *assist)
{
    bench_out_t o = {0};
    if (!b || !cfg) return o;
    if (b->state != BENCH_BASELINE && b->state != BENCH_RUN &&
        b->state != BENCH_COAST) {
        return o;                           /* idle or already finished */
    }

    /* Safety aborts ONLY. There is deliberately no data-driven stop: a run is
     * never cut short because samples went missing. */
    if (!armed) return fail(b, "disarmed");
    if (cfg->max_yaw_dps > 0.0f && !(fabsf(yaw_rate_dps) <= cfg->max_yaw_dps)) {
        return fail(b, "excessive yaw");    /* also catches NaN */
    }

    int64_t elapsed = now_us - b->start_us;
    if (elapsed < 0) elapsed = 0;
    b->elapsed_s = (float)elapsed / 1000000.0f;

    const int64_t run_end = cfg->baseline_us + cfg->run_us;
    const int64_t all_end = run_end + cfg->coast_us;

    if (elapsed < cfg->baseline_us) {
        b->state = BENCH_BASELINE;          /* motors off: gyro drift + start state */
        b->baseline_sum += yaw_rate_dps;
        b->baseline_n++;
    } else if (elapsed < run_end) {
        b->state = BENCH_RUN;
        bench_commands(b->kind, b->base, b->delta, &o.left_cmd, &o.right_cmd);
        /* Apply the stored trim to what actually goes out, and therefore to
         * what gets recorded. The motors-off phases command (0,0) and
         * esc_trim_apply_pair leaves those alone, so the baseline stays a true
         * motors-off measurement. */
        esc_trim_apply_pair(trim, &o.left_cmd, &o.right_cmd);

    } else if (elapsed < all_end) {
        b->state = BENCH_COAST;             /* motors off, still recording */
    } else {
        b->state = BENCH_SAVED;
        o.finished = true;
        return o;
    }
    o.active = true;

    /* Record the WHOLE run -- baseline, drive and coast alike. Which part is
     * useful gets decided later, off the file. */
    if (b->count < BENCH_MAX_SAMPLES) {
        b->samples[b->count].t_s = b->elapsed_s;
        b->samples[b->count].yaw_rate_dps = yaw_rate_dps;
        b->samples[b->count].left = o.left_cmd;
        b->samples[b->count].right = o.right_cmd;
        /* trim is in mixer units (left -= t/2, right += t/2), so the
         * dimensionless c is trim / (2 * throttle). */
        b->samples[b->count].c =
            (b->base > 0.0f) ? (trim / (2.0f * b->base)) : 0.0f;
        b->samples[b->count].p_yaw  = assist ? assist->yaw_filt : 0.0f;
        b->samples[b->count].p_corr = assist ? assist->correction : 0.0f;
        b->samples[b->count].c_learn = assist ? assist->learned_c : 0.0f;
        b->samples[b->count].p_flags =
            (uint8_t)(((assist && assist->on) ? BENCH_P_ON : 0u) |
                      ((assist && assist->at_cap) ? BENCH_P_AT_CAP : 0u));
        b->count++;
    } else {
        b->overflow = true;                 /* stop recording, never overrun */
    }
    return o;
}
