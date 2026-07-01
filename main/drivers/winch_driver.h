#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Reconfigure the winch PWM pin (shared with the BOOT button) as an MCPWM
 * output and drive neutral/stop immediately; configure the servo-enable pin as
 * an output in the inactive (unpowered) state. Call AFTER the boot-button read.
 * Idempotent; returns ESP_ERR_INVALID_STATE on re-init.
 */
esp_err_t winch_driver_init(void);

/**
 * Set winch speed. speed in [-1.0, 1.0]:
 *   + = CW = DOWN (pay out), - = CCW = UP (reel in), 0 = stop. Clamped.
 */
esp_err_t winch_driver_set_speed(float speed);

/** Enable/disable the servo power rail (respects SERVO_ENABLE_ACTIVE_LOW). */
esp_err_t winch_driver_set_power(bool on);

/** Last commanded winch speed (safe from any context). */
float winch_driver_get_speed(void);

#ifdef __cplusplus
}
#endif
