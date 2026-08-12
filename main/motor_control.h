#pragma once

#include "esc_trim.h"
#include "esp_err.h"
#include "proto/boat.pb.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start ESC neutral PWM immediately. Call before ESCs power up (early in boot).
 * Does NOT register pipeline handlers — call motor_control_init() for that.
 */
esp_err_t motor_control_init_hw(void);

/**
 * Register manual pipeline ingress and start the persistent ControlTask.
 * Call AFTER pipeline_init().
 */
esp_err_t motor_control_init(void);

/**
 * Load the ESC differential-trim table (own NVS record — see esc_trim.h,
 * independent of CalibrationData). Call once at boot, after
 * fs_load_esc_trim(); an empty table (count == 0) is always a safe no-op.
 */
void motor_control_set_esc_trim(const EscTrimPoint *pts, uint8_t count);

/** Immediately command propulsion and auxiliary actuators to a safe state. */
void motor_control_disarm(void);

/** Refresh manual-control liveness. Call only after accepting a manual command. */
void motor_control_notify_link_rx(int64_t received_us);

/** Copy a stable MotorStatus snapshot and return its generation. */
uint32_t motor_control_get_status(boat_MotorStatus *out);

#ifdef __cplusplus
}
#endif
