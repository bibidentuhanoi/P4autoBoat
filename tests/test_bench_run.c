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


/* ---- BASE_LONG: the 10 s lake variant --------------------------------------
 *
 * A separate KIND, not a configurable duration. Everything else -- throttle,
 * commands, learner, P, aborts, recording -- is BASE's, unchanged. These pin
 * that "everything else" really is unchanged, because a long run that quietly
 * differs from BASE is worse than no long run at all: its numbers would look
 * comparable and would not be.
 */

static bench_cfg_t cfg_long(void)
{
    bench_cfg_t c = {
        .baseline_us = US(0.5), .run_us = US(10.0), .coast_us = US(1.0),
        .max_yaw_dps = 200.0f,
    };
    return c;
}

static void base_long_commands_exactly_like_base(void)
{
    float l, r, bl, br;
    bench_commands(BENCH_KIND_BASE,      0.20f, 0.04f, &bl, &br);
    bench_commands(BENCH_KIND_BASE_LONG, 0.20f, 0.04f, &l, &r);
    assert(l == bl && r == br);
    /* both motors equal: any turn IS the mismatch, exactly as for BASE */
    assert(fabsf(l - 0.20f) < 1e-6f && fabsf(r - 0.20f) < 1e-6f);

    /* and at a different throttle, and with a delta that BASE ignores */
    bench_commands(BENCH_KIND_BASE,      0.35f, 0.10f, &bl, &br);
    bench_commands(BENCH_KIND_BASE_LONG, 0.35f, 0.10f, &l, &r);
    assert(l == bl && r == br);
}

/* The whole point: 0.5 s baseline + 10 s drive + 1 s coast, every sample kept. */
static void base_long_drives_ten_seconds_and_fits(void)
{
    bench_cfg_t c = cfg_long();
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_BASE_LONG, 0.20f, 0.0f, 0));

    unsigned n_baseline = 0, n_run = 0, n_coast = 0;
    int64_t t = 0;
    bool finished = false;
    for (int i = 0; i < 2000 && !finished; ++i) {       /* 20 s of headroom */
        t = (int64_t)i * 10000;                        /* 100 Hz */
        bench_out_t o = bench_step_notrim(&b, &c, t, 1.0f, true);
        if (o.finished) { finished = true; break; }
        if (b.state == BENCH_BASELINE) { n_baseline++; assert(o.left_cmd == 0.0f); }
        else if (b.state == BENCH_RUN) {
            n_run++;
            assert(fabsf(o.left_cmd - 0.20f) < 1e-6f);
            assert(fabsf(o.right_cmd - 0.20f) < 1e-6f);
        } else if (b.state == BENCH_COAST) { n_coast++; assert(o.left_cmd == 0.0f); }
    }
    assert(finished);
    assert(b.state == BENCH_SAVED);

    /* 100 Hz: 0.5 s -> 50, 10 s -> 1000, 1 s -> 100 */
    assert(n_baseline == 50);
    assert(n_run == 1000);
    assert(n_coast == 100);

    /* THE buffer question. Every sample of the whole run, no overflow, and no
     * silent drop -- an overflowed run would look complete and be short. */
    assert(!b.overflow);
    assert(b.count == n_baseline + n_run + n_coast);
    assert(b.count == 1150);
    assert(b.count <= BENCH_MAX_SAMPLES);
}

static void base_long_leaves_ordinary_base_at_three_seconds(void)
{
    /* The existing profile, byte for byte: 0.5 + 3 + 1 at 100 Hz. */
    bench_cfg_t c = cfg();
    bench_t b;
    bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_BASE, 0.20f, 0.0f, 0));
    unsigned n_run = 0;
    bool finished = false;
    for (int i = 0; i < 2000 && !finished; ++i) {
        bench_out_t o = bench_step_notrim(&b, &c, (int64_t)i * 10000, 1.0f, true);
        if (o.finished) { finished = true; break; }
        if (b.state == BENCH_RUN) n_run++;
    }
    assert(finished);
    assert(n_run == 300);                 /* 3.0 s, unchanged */
    assert(b.count == 450);               /* 50 + 300 + 100 */
    assert(!b.overflow);
}

/* The buffer must hold the long run with real margin, not exactly. */
static void the_buffer_has_margin_over_the_long_run(void)
{
    const unsigned needed = 1150u;        /* (0.5 + 10 + 1) s at 100 Hz */
    assert(BENCH_MAX_SAMPLES >= needed);
    /* At least 10% spare, so a late tick or a slightly long phase cannot
     * silently truncate the one run this mode exists to capture. */
    assert(BENCH_MAX_SAMPLES >= needed + needed / 10u);
    assert(BENCH_MAX_SAMPLES <= 65535u);  /* b.count is uint16_t */
}

/* Every abort that protects a BASE run protects a long one identically. */
static void base_long_aborts_exactly_like_base(void)
{
    bench_cfg_t c = cfg_long();

    /* disarm, deep into the drive phase where a short run would already be over */
    {
        bench_t b; bench_init(&b);
        assert(bench_start(&b, BENCH_KIND_BASE_LONG, 0.20f, 0.0f, 0));
        for (int i = 0; i < 500; ++i)
            (void)bench_step_notrim(&b, &c, (int64_t)i * 10000, 1.0f, true);
        assert(b.state == BENCH_RUN);
        bench_out_t o = bench_step_notrim(&b, &c, 5000000, 1.0f, false);
        assert(o.aborted && b.state == BENCH_FAILED);
        assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f);
    }
    /* excessive yaw */
    {
        bench_t b; bench_init(&b);
        assert(bench_start(&b, BENCH_KIND_BASE_LONG, 0.20f, 0.0f, 0));
        for (int i = 0; i < 500; ++i)
            (void)bench_step_notrim(&b, &c, (int64_t)i * 10000, 1.0f, true);
        bench_out_t o = bench_step_notrim(&b, &c, 5000000, 9999.0f, true);
        assert(o.aborted && b.state == BENCH_FAILED);
        assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f);
    }
    /* NaN yaw -- the comparison is written so NaN trips it too */
    {
        bench_t b; bench_init(&b);
        assert(bench_start(&b, BENCH_KIND_BASE_LONG, 0.20f, 0.0f, 0));
        for (int i = 0; i < 500; ++i)
            (void)bench_step_notrim(&b, &c, (int64_t)i * 10000, 1.0f, true);
        bench_out_t o = bench_step_notrim(&b, &c, 5000000, NAN, true);
        assert(o.aborted && b.state == BENCH_FAILED);
        assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f);
    }
    /* explicit abort (the STOP path) */
    {
        bench_t b; bench_init(&b);
        assert(bench_start(&b, BENCH_KIND_BASE_LONG, 0.20f, 0.0f, 0));
        for (int i = 0; i < 500; ++i)
            (void)bench_step_notrim(&b, &c, (int64_t)i * 10000, 1.0f, true);
        bench_abort(&b);
        assert(b.state == BENCH_FAILED);
        bench_out_t o = bench_step_notrim(&b, &c, 5100000, 1.0f, true);
        assert(o.left_cmd == 0.0f && o.right_cmd == 0.0f && !o.active);
    }
}

static void a_long_run_blocks_a_second_start(void)
{
    bench_cfg_t c = cfg_long();
    bench_t b; bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_BASE_LONG, 0.20f, 0.0f, 0));
    for (int i = 0; i < 300; ++i)
        (void)bench_step_notrim(&b, &c, (int64_t)i * 10000, 1.0f, true);
    assert(!bench_start(&b, BENCH_KIND_BASE, 0.20f, 0.0f, 3000000));
    assert(!bench_start(&b, BENCH_KIND_BASE_LONG, 0.20f, 0.0f, 3000000));
}

/* Trim and P are recorded per sample through the WHOLE long drive, not just
 * its first three seconds -- the learner working over ten seconds is the
 * entire reason for this mode. */
static void trim_and_p_are_recorded_across_the_long_drive(void)
{
    bench_cfg_t c = cfg_long();
    bench_t b; bench_init(&b);
    assert(bench_start(&b, BENCH_KIND_BASE_LONG, 0.20f, 0.0f, 0));
    bench_assist_t a = {
        .heading_deg = 181.0f, .heading_target_deg = 180.0f,
        .heading_error_deg = -1.0f, .yaw_target_dps = -0.8f,
        .yaw_filt = 1.5f, .rate_error_dps = -2.3f,
        .p_term = -0.115f, .i_term = 0.035f,
        .correction = -0.08f, .effective_c = 0.11f, .c_limit = 1.0f,
        .learned_c = 0.19f, .on = true, .heading_hold = true,
        .at_cap = false,
    };
    bool finished = false;
    for (int i = 0; i < 1300 && !finished; ++i) {
        bench_out_t o = bench_step(&b, &c, (int64_t)i * 10000, 1.0f, true,
                                   0.08f, &a);
        if (o.finished) finished = true;
    }
    assert(finished && !b.overflow);

    /* a sample from LATE in the drive: 9 s in, past any 3 s profile */
    const bench_sample_t *late = NULL;
    for (uint16_t i = 0; i < b.count; ++i) {
        if (b.samples[i].t_s > 9.0f && b.samples[i].t_s < 10.0f) {
            late = &b.samples[i]; break;
        }
    }
    assert(late != NULL);
    assert(late->p_flags & BENCH_P_ON);
    assert(fabsf(late->p_yaw - 1.5f) < 1e-6f);
    assert(fabsf(late->p_corr - (-0.08f)) < 1e-6f);
    assert(fabsf(late->c_learn - 0.19f) < 1e-6f);
    assert(fabsf(late->heading_deg - 181.0f) < 1e-6f);
    assert(fabsf(late->heading_target_deg - 180.0f) < 1e-6f);
    assert(fabsf(late->heading_error_deg - (-1.0f)) < 1e-6f);
    assert(fabsf(late->yaw_target_dps - (-0.8f)) < 1e-6f);
    assert(fabsf(late->rate_error_dps - (-2.3f)) < 1e-6f);
    assert(fabsf(late->p_term - (-0.115f)) < 1e-6f);
    assert(fabsf(late->i_term - 0.035f) < 1e-6f);
    assert(fabsf(late->effective_c - 0.11f) < 1e-6f);
    assert(fabsf(late->c_limit - 1.0f) < 1e-6f);
    assert(late->p_flags & BENCH_CTRL_HEADING_HOLD);
    /* trim really reached the commands that were recorded */
    assert(late->left < late->right);
    assert(fabsf(late->c - (0.08f / (2.0f * 0.20f))) < 1e-5f);
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
    /* ---- BASE_LONG ---- */
    base_long_commands_exactly_like_base();
    base_long_drives_ten_seconds_and_fits();
    base_long_leaves_ordinary_base_at_three_seconds();
    the_buffer_has_margin_over_the_long_run();
    base_long_aborts_exactly_like_base();
    a_long_run_blocks_a_second_start();
    trim_and_p_are_recorded_across_the_long_drive();
    return 0;
}
