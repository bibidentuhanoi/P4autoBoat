/* The proof-of-concept guarantee for the proportional trim.
 *
 * Everything the boat does with the trim goes through one of two paths:
 *
 *   manual driving : esc_trim_mix()        (motor_control.c)
 *   BASE bench run : esc_trim_lookup() + esc_trim_apply_pair()  (bench_tick)
 *
 * A BASE run only proves anything if the boat drives the run with the SAME
 * correction it drives manually. These tests pin both paths to the measured
 * numbers and to each other, across the whole 10-50% range the bench sweeps.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "esc_trim.h"

#define C 0.16f

static int near(float a, float b) { return fabsf(a - b) < 1e-5f; }

/* What bench_tick does: look the trim up at the run's base throttle, then
 * apply it to the (base, base) pair a BASE run commands. */
static void bench_path(const EscTrimPoint *pts, uint8_t n, float base,
                       float *l, float *r)
{
    *l = base;
    *r = base;
    esc_trim_apply_pair(esc_trim_lookup(pts, n, base), l, r);
}

int main(void)
{
    EscTrimPoint pts[2];
    uint8_t n = esc_trim_build_proportional(C, pts);
    assert(n == 2);

    /* The line through the origin: trim(T) = 2*c*T. */
    assert(near(pts[0].throttle_frac, 0.0f) && near(pts[0].trim_diff, 0.0f));
    assert(near(pts[1].throttle_frac, 1.0f) && near(pts[1].trim_diff, 2.0f * C));

    /* Refuse an unusable c -- 0 points reads as "no trim", never a bad one. */
    assert(esc_trim_build_proportional(0.0f, pts) == 0);
    assert(esc_trim_build_proportional(NAN, pts) == 0);
    n = esc_trim_build_proportional(C, pts);

    /* A far-too-large c is clamped, not obeyed. */
    EscTrimPoint big[2];
    assert(esc_trim_build_proportional(9.0f, big) == 2);
    assert(near(big[1].trim_diff, 1.0f));            /* 2 * 0.5 */
    assert(esc_trim_build_proportional(-9.0f, big) == 2);
    assert(near(big[1].trim_diff, -1.0f));

    /* THE measurement. The bench found the straight-running split at each
     * level; c*T must reproduce them. Tolerance is the spread in the data
     * itself (c came out 0.19-0.23), not an exact fit. */
    /* From regressing RAW yaw against c over a range of c (five fits, two
     * adaptive sessions): straight at c = 0.155, 0.135, 0.159, 0.170, 0.179.
     * split = c * throttle at the levels those fits came from. */
    struct { float t, measured_split; } bench[] = {
        { 0.20f, 0.16f * 0.20f },
        { 0.30f, 0.16f * 0.30f },
        { 0.35f, 0.16f * 0.35f },
        { 0.40f, 0.16f * 0.40f },
    };
    for (unsigned i = 0; i < sizeof bench / sizeof bench[0]; ++i) {
        float split = 0.5f * esc_trim_lookup(pts, n, bench[i].t);
        float err = fabsf(split - bench[i].measured_split);
        /* within 0.75% of throttle -- tighter than the run-to-run scatter */
        assert(err <= 0.0075f * bench[i].t + 0.004f);
    }

    /* 10-50%, the range the BASE test sweeps: both paths, same answer, and
     * the split is exactly c of the throttle every time. */
    for (int pct = 10; pct <= 50; ++pct) {
        float t = (float)pct / 100.0f;

        float ml = t, mr = t;
        esc_trim_mix(t, 0.0f, pts, n, &ml, &mr);

        float bl, br;
        bench_path(pts, n, t, &bl, &br);

        assert(near(ml, bl) && near(mr, br));        /* bench == manual */
        assert(near(ml, t * (1.0f - C)));
        assert(near(mr, t * (1.0f + C)));
        assert(near(0.5f * (ml + mr), t));           /* mean throttle kept */
        assert(near(0.5f * (mr - ml), C * t));       /* split IS c * throttle */
        assert(mr > ml);                             /* RIGHT gets more */
    }

    /* Motors off stays off on both paths -- a BASE run's 0.5 s motors-off
     * baseline is the gyro-drift measurement everything else is corrected
     * against. One jet waking up there silently ruins every number. */
    float zl, zr;
    bench_path(pts, n, 0.0f, &zl, &zr);
    assert(zl == 0.0f && zr == 0.0f);
    /* ...and directly, with a real trim forced onto a stopped pair: the
     * lookup happens to return 0 at throttle 0, so without this the guard
     * inside esc_trim_apply_pair is never actually exercised. */
    zl = 0.0f; zr = 0.0f;
    esc_trim_apply_pair(2.0f * C, &zl, &zr);
    assert(zl == 0.0f && zr == 0.0f);
    float ml0 = 0.0f, mr0 = 0.0f;
    esc_trim_mix(0.0f, 0.0f, pts, n, &ml0, &mr0);
    assert(ml0 == 0.0f && mr0 == 0.0f);

    /* A flat table is what this replaces: correct at T40, 4x over at T10.
     * If this ever stops being true the two are no longer distinguishable
     * and the whole finding has been undone. */
    EscTrimPoint flat[1] = { { .throttle_frac = 1.0f, .trim_diff = 0.152f } };
    assert(near(0.5f * esc_trim_lookup(flat, 1, 0.40f), 0.076f));
    assert(near(0.5f * esc_trim_lookup(flat, 1, 0.10f), 0.076f));   /* wrong */
    assert(0.5f * esc_trim_lookup(pts, n, 0.10f) < 0.030f);         /* right */

    /* Pilot steering rides on top, unchanged: the trim shifts the pair, the
     * rudder differential is preserved. */
    float sl = 0.0f, sr = 0.0f;
    esc_trim_mix(0.30f, 0.10f, pts, n, &sl, &sr);
    assert(near(0.5f * (sl + sr), 0.30f));
    assert(near(sl - sr, 2.0f * 0.10f - 2.0f * C * 0.30f));

    /* Headroom: the mixer never clips the pilot's command to fit the trim, so
     * near full throttle the trim tapers instead of stealing thrust. */
    float fl = 0.0f, fr = 0.0f;
    esc_trim_mix(1.0f, 0.0f, pts, n, &fl, &fr);
    assert(near(fl, 1.0f) && near(fr, 1.0f));
    /* ...and that taper starts above the bench range, not inside it. */
    float hl = 0.0f, hr = 0.0f;
    esc_trim_mix(0.80f, 0.0f, pts, n, &hl, &hr);
    assert(near(0.5f * (hr - hl), C * 0.80f));

    printf("test_trim_proportional: OK\n");
    return 0;
}
