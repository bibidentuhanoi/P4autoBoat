#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESC_STATE_DISARMED = 0,
    ESC_STATE_ARMING   = 1,
    ESC_STATE_ARMED    = 2,
} esc_state_t;

/**
 * Initialize MCPWM peripheral, configure both ESC outputs, and output neutral
 * pulse immediately. Idempotent; returns ESP_ERR_INVALID_STATE on re-init.
 */
esp_err_t   esc_driver_init(void);

/**
 * Begin the 3-second arming sequence (neutral pulse held throughout).
 * Transitions state DISARMED -> ARMING -> ARMED on completion.
 * Returns ESP_ERR_INVALID_STATE if not currently DISARMED.
 */
esp_err_t   esc_driver_arm(void);

/**
 * Set throttle for both ESCs. Only valid in ARMED state.
 * @param left   Left  motor throttle: -1.0 = full reverse, 0.0 = neutral, +1.0 = full forward.
 * @param right  Right motor throttle: same range.
 * Returns ESP_ERR_INVALID_STATE if not ARMED.
 */
esp_err_t   esc_driver_set_throttle(float left, float right);

/**
 * Output neutral pulse on both ESCs and transition to DISARMED.
 */
esp_err_t   esc_driver_disarm(void);

/** Return current ESC state. */
esc_state_t esc_driver_get_state(void);

/** Copy the last-commanded throttle values (safe to call from any state). */
void        esc_driver_get_throttle(float *left, float *right);

#ifdef __cplusplus
}
#endif
