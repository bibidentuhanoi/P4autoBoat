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

/**
 * Copy the latest ESC-trim CalibrateStatus snapshot and return its generation.
 * Generation 0 means calibration has never run this boot (nothing to publish);
 * a changed generation is a fresh progress update. Read by the diagnostics task.
 */
uint32_t motor_control_get_calibrate_status(boat_CalibrateStatus *out);

/* Latest bench throttle-mismatch run state; returns its generation. */
uint32_t motor_control_get_bench_status(boat_BenchStatus *out);

/* Write a finished bench run to the SD card. Call from a NON-critical task:
 * the write is far too slow for the 10 ms control loop. */
void motor_control_bench_flush(void);

/* Report what the on-the-fly trim learner is doing, rate-limited. MUST be
 * called from the core-1 diagnostics task: it logs, and a synchronous UART
 * write does not fit in the control task's 10 ms budget. No-op when the
 * learner is compiled out. */
void motor_control_trimlearn_log(void);

/* The trim learner's current c, or 0 when it is compiled out. Reported in
 * BenchStatus so the operator can see what the boat is actually running. */
float motor_control_trimlearn_c(void);

/* Whether the boat is actually applying the fast P assist. Reported so an A/B
 * file can be checked against what the operator believes they selected. */
bool motor_control_p_assist_on(void);

#ifdef __cplusplus
}
#endif
