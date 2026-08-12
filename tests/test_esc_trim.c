#include <assert.h>
#include <math.h>
#include "esc_trim.h"

int main(void)
{
    EscTrimPoint pts[3] = {
        { .throttle_frac = 0.2f, .trim_diff = -0.06f },
        { .throttle_frac = 0.4f, .trim_diff = -0.10f },
        { .throttle_frac = 0.8f, .trim_diff = -0.18f },
    };

    /* Empty table -> no trim. */
    assert(esc_trim_lookup(pts, 0, 0.5f) == 0.0f);

    /* Exact at a point. */
    assert(fabsf(esc_trim_lookup(pts, 3, 0.4f) - (-0.10f)) < 1e-6f);

    /* Linear between 0.4 and 0.8: halfway (0.6) -> -0.14. */
    assert(fabsf(esc_trim_lookup(pts, 3, 0.6f) - (-0.14f)) < 1e-6f);

    /* Below the lowest / above the highest -> clamp to endpoint, no extrapolation. */
    assert(fabsf(esc_trim_lookup(pts, 3, 0.05f) - (-0.06f)) < 1e-6f);
    assert(fabsf(esc_trim_lookup(pts, 3, 0.95f) - (-0.18f)) < 1e-6f);

    /* Zero command is always off: a learned trim must never wake one jet. */
    float zero_left = 0.0f, zero_right = 0.0f;
    esc_trim_apply(&zero_left, &zero_right, pts, 3);
    assert(zero_left == 0.0f && zero_right == 0.0f);

    /* apply: common preserved, pilot turn preserved, trim symmetric.
     * left=0.5,right=0.5 -> common 0.5 -> trim(0.5)= -0.12 -> L=0.56,R=0.44. */
    float L = 0.5f, R = 0.5f;
    esc_trim_apply(&L, &R, pts, 3);
    assert(fabsf(L - 0.56f) < 1e-6f && fabsf(R - 0.44f) < 1e-6f);

    /* A pilot turn (L!=R) rides through untouched in differential terms:
     * L=0.6,R=0.4 -> common 0.5, turn +0.1 -> trim -0.12 -> L=0.66,R=0.34. */
    L = 0.6f; R = 0.4f;
    esc_trim_apply(&L, &R, pts, 3);
    assert(fabsf(L - 0.66f) < 1e-6f && fabsf(R - 0.34f) < 1e-6f);

    /* Lookup uses original throttle, not the average of clipped commands;
     * pilot steering remains the priority at saturation. */
    L = 0.0f;
    R = 0.0f;
    esc_trim_mix(0.9f, 0.4f, pts, 3, &L, &R);
    assert(fabsf(L - 1.0f) < 1e-6f && fabsf(R - 0.5f) < 1e-6f);

    /* Points need not be pre-sorted (per esc_trim_lookup's own doc comment):
     * a permuted table must produce identical results to the ascending
     * table above, at every throttle already exercised. */
    EscTrimPoint pts_unsorted[3] = {
        { .throttle_frac = 0.8f, .trim_diff = -0.18f },
        { .throttle_frac = 0.2f, .trim_diff = -0.06f },
        { .throttle_frac = 0.4f, .trim_diff = -0.10f },
    };
    assert(fabsf(esc_trim_lookup(pts_unsorted, 3, 0.4f) - esc_trim_lookup(pts, 3, 0.4f)) < 1e-6f);
    assert(fabsf(esc_trim_lookup(pts_unsorted, 3, 0.6f) - esc_trim_lookup(pts, 3, 0.6f)) < 1e-6f);
    assert(fabsf(esc_trim_lookup(pts_unsorted, 3, 0.05f) - esc_trim_lookup(pts, 3, 0.05f)) < 1e-6f);
    assert(fabsf(esc_trim_lookup(pts_unsorted, 3, 0.95f) - esc_trim_lookup(pts, 3, 0.95f)) < 1e-6f);

    /* Duplicate throttle_frac (a corrupted/degenerate table, or two points
     * learned at the same nominal throttle): the bracket search's span<=0
     * guard must return a defined trim_diff from one of the tied points --
     * never NaN/Inf, never a crash from division by zero. */
    EscTrimPoint pts_dup[4] = {
        { .throttle_frac = 0.2f, .trim_diff = -0.05f },
        { .throttle_frac = 0.5f, .trim_diff = -0.10f },
        { .throttle_frac = 0.5f, .trim_diff = -0.15f },
        { .throttle_frac = 0.8f, .trim_diff = -0.20f },
    };
    float dup_result = esc_trim_lookup(pts_dup, 4, 0.5f);
    assert(!isnan(dup_result) && !isinf(dup_result));

    /* Single-point table: every query clamps to the one point everywhere --
     * "below lowest" and "above highest" both resolve to it. */
    EscTrimPoint pts_single[1] = {
        { .throttle_frac = 0.5f, .trim_diff = -0.09f },
    };
    assert(esc_trim_lookup(pts_single, 1, 0.0f) == 0.0f);
    assert(fabsf(esc_trim_lookup(pts_single, 1, 0.5f) - (-0.09f)) < 1e-6f);
    assert(fabsf(esc_trim_lookup(pts_single, 1, 1.0f) - (-0.09f)) < 1e-6f);

    return 0;
}
