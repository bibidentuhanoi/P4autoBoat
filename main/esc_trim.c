#include "esc_trim.h"
#include <math.h>

static float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

static float maxf_local(float a, float b) { return a > b ? a : b; }
static float minf_local(float a, float b) { return a < b ? a : b; }

float esc_trim_lookup(const EscTrimPoint *pts, uint8_t count, float common_throttle)
{
    if (!pts || count == 0 || !isfinite(common_throttle) || common_throttle <= 0.0f) {
        return 0.0f;
    }

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
    if (!isfinite(*left) || !isfinite(*right)) {
        *left = 0.0f;
        *right = 0.0f;
        return;
    }
    float common = 0.5f * (*left + *right);
    if (common <= 0.0f) {
        *left = 0.0f;
        *right = 0.0f;
        return;
    }
    float trim = esc_trim_lookup(pts, count, common);
    *left  -= 0.5f * trim;
    *right += 0.5f * trim;
}

void esc_trim_apply_pair(float trim, float *left, float *right)
{
    if (!left || !right || !isfinite(trim)) return;
    if (*left <= 0.0f && *right <= 0.0f) return;   /* stopped stays stopped */
    *left = clamp01(*left - 0.5f * trim);
    *right = clamp01(*right + 0.5f * trim);
}

uint8_t esc_trim_build_proportional(float c, EscTrimPoint *out)
{
    if (!out || !isfinite(c) || c == 0.0f) return 0;
    /* c = 0.5 already means the right motor is commanded 3x the left. Past
     * that the "small correction" reading breaks down and it is far more
     * likely a typo than a real boat, so refuse to build it. */
    if (c > 0.5f) c = 0.5f;
    if (c < -0.5f) c = -0.5f;
    out[0].throttle_frac = 0.0f;
    out[0].trim_diff = 0.0f;
    out[1].throttle_frac = 1.0f;
    out[1].trim_diff = 2.0f * c;
    return 2;
}

void esc_trim_mix(float throttle, float rudder, const EscTrimPoint *pts,
                  uint8_t count, float *left, float *right)
{
    if (!left || !right || !isfinite(throttle) || !isfinite(rudder)) {
        if (left) *left = 0.0f;
        if (right) *right = 0.0f;
        return;
    }

    /* Preserve the existing mixer semantics first. */
    float base_left = clamp01(throttle + rudder);
    float base_right = clamp01(throttle - rudder);
    if (throttle <= 0.0f) {
        *left = 0.0f;
        *right = 0.0f;
        return;
    }

    float trim = esc_trim_lookup(pts, count, clamp01(throttle));
    /* d is applied as left -= d/2, right += d/2. Restrict d so the
     * already-achievable pilot command is never clipped further. */
    float d_lo = maxf_local(2.0f * (base_left - 1.0f), -2.0f * base_right);
    float d_hi = minf_local(2.0f * base_left, 2.0f * (1.0f - base_right));
    if (trim < d_lo) trim = d_lo;
    if (trim > d_hi) trim = d_hi;
    *left = clamp01(base_left - 0.5f * trim);
    *right = clamp01(base_right + 0.5f * trim);
}
