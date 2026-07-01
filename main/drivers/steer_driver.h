#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize both steering (rudder) servos on their MCPWM outputs and drive
 * them to center immediately. Idempotent; ESP_ERR_INVALID_STATE on re-init.
 */
esp_err_t steer_driver_init(void);

/**
 * Set each rudder servo independently. Values -1.0 = full left, 0.0 = center,
 * +1.0 = full right (clamped). For linked steering, pass the same value to both.
 * Per-servo reverse (Kconfig) is applied so mirrored-mounted servos track.
 */
esp_err_t steer_driver_set(float left, float right);

/** Last commanded rudder values (safe from any context). */
float steer_driver_get_left(void);
float steer_driver_get_right(void);

#ifdef __cplusplus
}
#endif
