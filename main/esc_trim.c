#include "esc_trim.h"

float esc_trim_lookup(const EscTrimPoint *pts, uint8_t count, float common_throttle)
{
    if (!pts || count == 0) return 0.0f;

    /* Find the lowest point >= throttle. Points are few (<= 8) and typically
     * already ascending, but do not assume it -- scan for the bracketing pair. */
    const EscTrimPoint *lo = &pts[0];
    const EscTrimPoint *hi = &pts[0];
    for (uint8_t i = 0; i < count; ++i) {
        if (pts[i].throttle_frac <= lo->throttle_frac) lo = &pts[i];  /* global min */
        if (pts[i].throttle_frac >= hi->throttle_frac) hi = &pts[i];  /* global max */
    }
    if (common_throttle <= lo->throttle_frac) return lo->trim_diff;   /* clamp low  */
    if (common_throttle >= hi->throttle_frac) return hi->trim_diff;   /* clamp high */

    /* Bracket: largest point below, smallest point above. */
    const EscTrimPoint *a = lo, *b = hi;
    for (uint8_t i = 0; i < count; ++i) {
        float t = pts[i].throttle_frac;
        if (t <= common_throttle && t >= a->throttle_frac) a = &pts[i];
        if (t >= common_throttle && t <= b->throttle_frac) b = &pts[i];
    }
    float span = b->throttle_frac - a->throttle_frac;
    if (span <= 0.0f) return a->trim_diff;   /* duplicate throttles -> take one */
    float frac = (common_throttle - a->throttle_frac) / span;
    return a->trim_diff + frac * (b->trim_diff - a->trim_diff);
}

void esc_trim_apply(float *left, float *right, const EscTrimPoint *pts, uint8_t count)
{
    if (!left || !right) return;
    float common = 0.5f * (*left + *right);
    float trim = esc_trim_lookup(pts, count, common);
    *left  -= 0.5f * trim;
    *right += 0.5f * trim;
}
