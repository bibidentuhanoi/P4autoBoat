#include "esc_map.h"

uint32_t esc_throttle_to_us(float t, uint32_t min_us, uint32_t max_us,
                            float floor_frac)
{
    if (t <= 0.0f) return min_us;
    if (t > 1.0f) t = 1.0f;
    if (floor_frac < 0.0f) floor_frac = 0.0f;
    if (floor_frac > 0.95f) floor_frac = 0.95f;

    float effective = floor_frac + t * (1.0f - floor_frac);
    return (uint32_t)((float)min_us +
                      effective * (float)(max_us - min_us) + 0.5f);
}
