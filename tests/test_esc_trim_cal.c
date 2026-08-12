#include <assert.h>
#include <math.h>
#include <string.h>
#include "esc_trim_cal.h"

static etc_cfg_t cfg(void)
{
    etc_cfg_t c = {
        .ki = 0.8f, .trim_clamp = 0.8f, .accept_k = 3.0f,
        .excessive_yaw_dps = 20.0f, .min_speed_mps = 0.1f,
        .window_ticks = 3, .noise_ticks = 5, .settle_in_us = 0,
        .level_timeout_us = 10000000, .level_count = 2,
        .levels = {0.4f, 0.7f}, .average_into_existing = false,
    };
    return c;
}

static void noise(etc_t *s, const etc_cfg_t *c, int64_t *t, float yaw)
{
    for (uint32_t i = 0; i < c->noise_ticks; ++i) {
        esc_trim_cal_step(s, c, *t, yaw, 0.0f, true, true, true, false);
        *t += 100000;
    }
}

static void converges_and_records(void)
{
    etc_cfg_t c = cfg(); etc_t s; int64_t t = 0;
    esc_trim_cal_init(&s, &c); esc_trim_cal_start(&s, &c);
    noise(&s, &c, &t, 0.0f);
    bool done = false;
    for (int i = 0; i < 300 && !done; ++i) {
        float yaw = 0.24f - s.trim_diff;
        etc_out_t o = esc_trim_cal_step(&s, &c, t, yaw, 1.0f,
                                        true, true, true, false);
        assert(o.active || o.done);
        done = o.done;
        t += 100000;
    }
    assert(done && s.out_count == 2);
    assert(fabsf(s.out_table[0].trim_diff - 0.24f) < 0.08f);
}

static void bias_is_subtracted(void)
{
    etc_cfg_t c = cfg(); c.level_count = 1; etc_t s; int64_t t = 0;
    esc_trim_cal_init(&s, &c); esc_trim_cal_start(&s, &c);
    noise(&s, &c, &t, 0.2f);
    for (int i = 0; i < 80 && s.state != ETC_DONE; ++i) {
        esc_trim_cal_step(&s, &c, t, 0.2f, 1.0f, true, true, true, false);
        t += 100000;
    }
    assert(s.state == ETC_DONE && fabsf(s.b0 - 0.2f) < 1e-5f);
    assert(fabsf(s.out_table[0].trim_diff) < 0.02f);
}

/* The wrong-sign fail-safe (spec's mandatory first-water-run check): if the
 * feedback sign is inverted, the integrator is positive feedback and must run
 * to the clamp and ABORT -- never saturate-and-keep-going, never record. Model
 * a sustained yaw the trim never nulls (measured b0 = 0, then a constant
 * corrected yaw that stays below excessive_yaw_dps so THAT abort can't mask
 * this one). trim_diff must wind to +-trim_clamp and abort with a clamp reason,
 * recording nothing. */
static void runaway_aborts_at_clamp(void)
{
    etc_cfg_t c = cfg(); etc_t s; int64_t t = 0;
    esc_trim_cal_init(&s, &c); esc_trim_cal_start(&s, &c);
    noise(&s, &c, &t, 0.0f);                 /* b0 = 0 */
    etc_out_t o = (etc_out_t){0};
    bool aborted = false;
    for (int i = 0; i < 50 && !aborted; ++i) {
        /* 3 dps << excessive_yaw_dps (20); never responds to trim_diff. */
        o = esc_trim_cal_step(&s, &c, t, 3.0f, 1.0f, true, true, true, false);
        aborted = o.aborted;
        t += 100000;
    }
    assert(aborted && strstr(o.reason, "clamp"));
    assert(s.out_count == 0);                /* nothing recorded on a runaway */
    assert(fabsf(s.trim_diff) >= c.trim_clamp);
}

static void abort_gates_and_limits(void)
{
    etc_cfg_t c = cfg(); etc_t s; int64_t t = 0;
    esc_trim_cal_init(&s, &c); esc_trim_cal_start(&s, &c);
    etc_out_t o = esc_trim_cal_step(&s, &c, t, 0, 0, false, true, true, false);
    assert(o.aborted && !o.active && o.left_cmd == 0 && o.right_cmd == 0);

    esc_trim_cal_init(&s, &c); esc_trim_cal_start(&s, &c);
    o = esc_trim_cal_step(&s, &c, 0, 25, 1, true, true, true, false);
    assert(o.aborted && strstr(o.reason, "yaw"));

    esc_trim_cal_init(&s, &c); esc_trim_cal_start(&s, &c);
    noise(&s, &c, &t, 0); t += c.level_timeout_us + 1;
    o = esc_trim_cal_step(&s, &c, t, 1, 0, true, true, true, false);
    assert(o.aborted && strstr(o.reason, "timeout"));

    /* A stationary boat cannot identify thrust bias; the making-way gate
     * prevents a false point from being recorded. */
    esc_trim_cal_init(&s, &c); esc_trim_cal_start(&s, &c); t = 0;
    noise(&s, &c, &t, 0.0f);
    for (int i = 0; i < 10 && s.state != ETC_ABORTED; ++i) {
        (void)esc_trim_cal_step(&s, &c, t, 0.0f, 0.0f,
                                true, true, true, false);
        t += 100000;
    }
    assert(s.out_count == 0);

    const bool gates[][4] = {
        {false, true, true, false}, {true, false, true, false},
        {true, true, false, false}, {true, true, true, true},
    };
    for (unsigned i = 0; i < 4; ++i) {
        esc_trim_cal_init(&s, &c); esc_trim_cal_start(&s, &c);
        o = esc_trim_cal_step(&s, &c, 0, 0, 0,
                              gates[i][0], gates[i][1], gates[i][2], gates[i][3]);
        assert(o.aborted && !o.active && o.left_cmd == 0 && o.right_cmd == 0);
    }
}

int main(void)
{
    converges_and_records();
    bias_is_subtracted();
    runaway_aborts_at_clamp();
    abort_gates_and_limits();
    return 0;
}
