#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start ESC neutral PWM immediately. Call before ESCs power up (early in boot).
 * Does NOT register pipeline handlers — call motor_control_init() for that.
 */
esp_err_t motor_control_init_hw(void);

/**
 * Register pipeline handlers and start watchdog. Call AFTER pipeline_init().
 */
esp_err_t motor_control_init(void);

esp_err_t motor_control_arm(void);
esp_err_t motor_control_disarm(void);

#ifdef __cplusplus
}
#endif
