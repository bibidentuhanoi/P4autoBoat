#include <assert.h>
#include <math.h>
#include <string.h>
#include "bench_run.h"

/* most tests run with no trim */
static bench_out_t bench_step_notrim(bench_t *b, const bench_cfg_t *c,
                                     int64_t now, float yaw, bool armed)
{ return bench_step(b, c, now, yaw, armed, 0.0f, NULL); }

#define US(sec) ((int64_t)((sec) * 1000000.0))

static bench_cfg_t cfg(void)
{
    bench_cfg_t c = {
        .baseline_us = US(0.5), .run_us = US(3.0), .coast_us = US(1.0),
        .max_yaw_dps = 200.0f,
    };
    return c;
}

static void commands_per_kind(void)
{
    float l, r;
    bench_commands(BENCH_KIND_BASE, 0.20f, 0.04f, &l, &r);
    assert(fabsf(l - 0.20f) < 1e-6f && fabsf(r - 0.20f) < 1e-6f);

    bench_commands(BENCH_KIND_LEFT, 0.20f, 0.04f, &l, &r);
    assert(fabsf(l - 0.24f) < 1e-6f && fabsf(r - 0.16f) < 1e-6f);

    bench_commands(BENCH_KIND_RIGHT, 0.20f, 0.04f, &l, &r);
    assert(fabsf(l - 0.16f) < 1e-6f && fabsf(r - 0.24f) < 1e-6f);

    /* Unidirectional jets: a command that would go negative clamps at zero. */
    bench_commands(BENCH_KIND_RIGHT, 0.02f, 0.05f, &l, &r);
    assert(l == 0.0f && fabsf(r - 0.07f) < 1e-6f);
}

/* Walk the whole profile at 100 Hz and check what the ESCs are told, that
 * every phase is recorded, and that the run ends exactly once. */
static void profile_drives_and_records_each_phase(void)
{
    bench_cfg_t c = cfg();
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_LEFT, 0.20f, 0.04f, 0));

    int finishes = 0;
    bool saw_baseline_cmd = false, saw_run_cmd = false, saw_coast_cmd = false;
    for (int i = 1; i <= 600; ++i) {
        int64_t now = (int64_t)i * 10000;      /* 10 ms tick */
        bench_out_t o = bench_step_notrim(&b, &c, now, 5.0f, true);
        if (o.finished) finishes++;
        if (b.state == BENCH_BASELINE && o.left_cmd == 0.0f && o.right_cmd == 0.0f)
            saw_baseline_cmd = true;
        if (b.state == BENCH_RUN && fabsf(o.left_cmd - 0.24f) < 1e-6f &&
            fabsf(o.right_cmd - 0.16f) < 1e-6f)
            saw_run_cmd = true;
        if (b.state == BENCH_COAST && o.left_cmd == 0.0f && o.right_cmd == 0.0f)
            saw_coast_cmd = true;
    }
    assert(saw_baseline_cmd && saw_run_cmd && saw_coast_cmd);
    assert(finishes == 1);
    assert(b.state == BENCH_SAVED);

    /* Every phase present in the buffer -- the whole run is recorded, so which
     * part is useful can be decided later off the file. */
    int n_base = 0, n_run = 0, n_coast = 0;
    for (uint16_t i = 0; i < b.count; ++i) {
        float t = b.samples[i].t_s;
        if (t < 0.5f) n_base++;
        else if (t < 3.5f) n_run++;
        else n_coast++;
    }
    assert(n_base > 0 && n_run > 0 && n_coast > 0);
    assert(!b.overflow);
}

/* The run is driven by the CLOCK, not by how many samples arrived. Ticking
 * sparsely must still complete normally -- nothing cuts gathering short. */
static void sparse_ticks_still_complete_on_the_clock(void)
{
    bench_cfg_t c = cfg();
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_BASE, 0.10f, 0.02f, 0));

    int finishes = 0;
    for (int i = 1; i <= 12; ++i) {                 /* 12 ticks over 6 s */
        bench_out_t o = bench_step_notrim(&b, &c, US(0.5) * i, 1.0f, true);
        if (o.finished) finishes++;
    }
    assert(finishes == 1);
    assert(b.state == BENCH_SAVED);
    assert(b.count > 0 && b.count < 20);            /* only what actually arrived */
}

static void disarm_aborts_and_zeroes_the_motors(void)
{
    bench_cfg_t c = cfg();
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_LEFT, 0.20f, 0.04f, 0));
    (void)bench_step_notrim(&b, &c, US(1.0), 5.0f, true);   /* mid-run */
    bench_out_t o = bench_step_notrim(&b, &c, US(1.1), 5.0f, false);
    assert(o.aborted && !o.active);
    assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f);
    assert(b.state == BENCH_FAILED);
}

static void excessive_yaw_aborts(void)
{
    bench_cfg_t c = cfg();
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_LEFT, 0.20f, 0.04f, 0));
    bench_out_t o = bench_step_notrim(&b, &c, US(1.0), 500.0f, true);
    assert(o.aborted && o.left_cmd == 0.0f && o.right_cmd == 0.0f);
    assert(b.state == BENCH_FAILED);
}

/* A full buffer must never corrupt memory or kill the run: recording simply
 * stops and the overflow is flagged so the file is not silently trusted. */
static void buffer_overflow_is_flagged_not_fatal(void)
{
    bench_cfg_t c = cfg();
    c.run_us = US(60.0);                             /* far more ticks than room */
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_BASE, 0.10f, 0.0f, 0));
    for (int i = 1; i <= 4000; ++i) {
        (void)bench_step_notrim(&b, &c, (int64_t)i * 10000, 1.0f, true);
    }
    assert(b.overflow);
    assert(b.count == BENCH_MAX_SAMPLES);
}

static void start_is_refused_while_a_run_is_going(void)
{
    bench_cfg_t c = cfg();
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_BASE, 0.10f, 0.0f, 0));
    (void)bench_step_notrim(&b, &c, US(0.1), 0.0f, true);
    assert(!bench_start(&b, BENCH_KIND_LEFT, 0.10f, 0.0f, US(0.2)));
}

/* With a stored trim, what is RECORDED must be what is SENT -- otherwise every
 * later analysis reads the split as zero while the boat ran trimmed. And the
 * motors-off phases must stay at (0,0), or the baseline is no longer a
 * motors-off measurement at all. */
static void trim_is_applied_and_recorded(void)
{
    bench_cfg_t c = cfg();
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_BASE, 0.40f, 0.08f, 0));
    const float trim = 0.152f;              /* right +7.6% */

    bench_out_t o = bench_step(&b, &c, US(0.2), 0.0f, true, trim, NULL);   /* baseline */
    assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f);

    o = bench_step(&b, &c, US(1.5), 0.0f, true, trim, NULL);               /* driving */
    assert(fabsf(o.left_cmd - 0.324f) < 1e-4f);
    assert(fabsf(o.right_cmd - 0.476f) < 1e-4f);

    o = bench_step(&b, &c, US(4.0), 0.0f, true, trim, NULL);               /* coasting */
    assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f);

    /* the buffer must hold those same numbers, not the pre-trim ones */
    bool saw_trimmed = false, saw_off = false;
    for (uint16_t i = 0; i < b.count; ++i) {
        if (fabsf(b.samples[i].left - 0.324f) < 1e-4f &&
            fabsf(b.samples[i].right - 0.476f) < 1e-4f) saw_trimmed = true;
        if (b.samples[i].left == 0.0f && b.samples[i].right == 0.0f) saw_off = true;
    }
    assert(saw_trimmed);
    assert(saw_off);
}

/* On a hand-held bench the hands turn the boat as hard as the motor mismatch
 * does, and both land in the same gyro. Measured over 21 runs: the raw yaw
 * read positive 17 times while the true turn was balanced, and the learner
 * walked c from 0.200 to 0.126 -- away from the 0.191 those same runs say is
 * right. The run's own motors-off zero is what separates the two. */
static void each_run_measures_its_own_motors_off_zero(void)
{
    bench_cfg_t cfg = { .baseline_us = 500000, .run_us = 3000000,
                        .coast_us = 1000000, .max_yaw_dps = 200.0f };
    bench_t b;
    bench_init(&b);
    assert(bench_baseline_yaw(&b) == 0.0f);         /* nothing measured yet */
    assert(bench_start(&b, BENCH_KIND_BASE, 0.30f, 0.0f, 0));

    /* too few samples to trust -- reports zero rather than a wild guess */
    bench_step(&b, &cfg, 10000, 3.0f, true, 0.0f, NULL);
    assert(bench_baseline_yaw(&b) == 0.0f);

    /* a full baseline of a steady +1.4 deg/s hold */
    for (int64_t us = 20000; us < 500000; us += 10000) {
        bench_step(&b, &cfg, us, 1.4f, true, 0.0f, NULL);
    }
    const float z = bench_baseline_yaw(&b);
    assert(fabsf(z - 1.4f) < 0.05f);

    /* it FREEZES once the motors run: the baseline is the zero measured
     * BEFORE they did, and must not absorb the turn it exists to reveal */
    for (int64_t us = 500000; us < 2000000; us += 10000) {
        bench_step(&b, &cfg, us, -9.0f, true, 0.0f, NULL);
    }
    assert(fabsf(bench_baseline_yaw(&b) - z) < 1e-6f);

    /* every run measures it fresh -- the hands are somewhere new each time
     * the boat is picked up. Finish this run first; a start is rightly
     * refused while one is going. */
    for (int64_t us = 2000000; us <= 4600000; us += 10000) {
        bench_step(&b, &cfg, us, 0.0f, true, 0.0f, NULL);
    }
    assert(b.state == BENCH_SAVED);
    assert(bench_start(&b, BENCH_KIND_BASE, 0.30f, 0.0f, 10000000));
    assert(bench_baseline_yaw(&b) == 0.0f);
}

int main(void)
{
    commands_per_kind();
    trim_is_applied_and_recorded();
    profile_drives_and_records_each_phase();
    sparse_ticks_still_complete_on_the_clock();
    disarm_aborts_and_zeroes_the_motors();
    excessive_yaw_aborts();
    buffer_overflow_is_flagged_not_fatal();
    start_is_refused_while_a_run_is_going();
    each_run_measures_its_own_motors_off_zero();
    return 0;
}
