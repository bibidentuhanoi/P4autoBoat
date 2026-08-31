#pragma once

#include <stdint.h>

/* Map a normalized throttle to an ESC pulse, skipping the measured motor
 * dead-band. Zero remains the off pulse; any positive input maps at or above
 * floor_frac of the usable span. */
uint32_t esc_throttle_to_us(float t, uint32_t min_us, uint32_t max_us,
                            float floor_frac);
