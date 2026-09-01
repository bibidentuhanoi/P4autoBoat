#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the rudder MCPWM output and drive it to STEER_HOME (full-right)
 * immediately. Idempotent; ESP_ERR_INVALID_STATE on re-init.
 *
 * ONE unified steering channel: a boat has a single steering axis, so both
 * rudder servos share this one signal (wired in parallel) and always deflect
 * together. Split/independent rudders were removed — they added a degree of
 * freedom nothing steers with.
 *
 * No position feedback exists on this board (no potentiometer signal reaches
 * the MCU), so firmware cannot know where the horn physically is at boot —
 * this is the standard fix for that (per SparkFun/CMU robotics-course
 * documentation): home to a FIXED, KNOWN target, and the operator hand-turns
 * the rudder to match that same target (full-right mechanical stop, confirmed
 * against real hardware 2026-08-04) before every power-up. If the operator
 * does that, there is no snap; if not, it corrects to STEER_HOME on power-up.
 */
esp_err_t steer_driver_init(void);

/**
 * Set the rudder angle. -1.0 = full left, 0.0 = center, +1.0 = full right
 * (clamped). Drives both rudder servos together. CONFIG_STEER_REVERSE flips
 * the direction.
 */
esp_err_t steer_driver_set(float steer);

/** Last commanded rudder angle (safe from any context). */
float steer_driver_get(void);

/* The pulse width the CURRENT normalized command maps to, in microseconds --
 * the same value steer_driver_set() last wrote to the comparator, including
 * CONFIG_STEER_REVERSE. This is the COMMAND, not a position: the servo has no
 * feedback. Returns 0 before init.
 *
 * Deliberately recomputed from the stored command rather than cached from the
 * write, so it cannot silently disagree with what steer_to_us() would produce
 * today. A raw-pulse calibration write (steer_driver_set_raw_us) is NOT
 * reflected here, for the same reason it does not move steer_driver_get(). */
uint32_t steer_driver_get_pulse_us(void);

/**
 * Defensive re-assertion, NOT a new command: re-clears any hold that may have
 * re-latched on this pad since boot and rewrites the current angle fresh, so
 * the physical pad is guaranteed to match what the peripheral believes it's
 * outputting. Call immediately before energising the servo rail (e.g. from
 * ensure_servo_rail()) — a boot-time-only write can't guarantee it survives
 * untouched through however long passes before power actually arrives.
 */
void steer_driver_reassert(void);

/**
 * Calibration only: write a raw MCPWM pulse width in microseconds directly,
 * bypassing STEER_PULSE_MIN/NEUTRAL/MAX — for finding a servo's true
 * mechanical stop when it may lie outside the currently configured range.
 * Clamped to [400, 2600]us as a sanity bound, not a calibration limit.
 */
esp_err_t steer_driver_set_raw_us(uint32_t pulse_us);

#ifdef __cplusplus
}
#endif
