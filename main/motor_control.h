#pragma once

#include "auto_drive.h"
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

/* ---- The out-and-back mission's AUTO owner (2026-09-25) -------------------
 * The autonomy task (core 1, 20 Hz) hands the control task a setpoint; the
 * control task flies it through the heading hold as the LAST ESC write of the
 * cycle, exempt from the manual link-loss failsafe -- the bench-run pattern.
 * STOP, a non-zero manual command, disarm, an explicit rail PWR-OFF, another
 * owner of the jets, or a setpoint older than 0.5 s end it at once.
 * Without CONFIG_MISSION_ENABLE (or the heading hold) all of this compiles to
 * "not built": the setpoint is ignored and nothing ever owns the jets. */
bool motor_control_mission_built(void);
/* Autonomy task: the setpoint of this 50 ms step (auto_drive.h). */
void motor_control_set_auto_setpoint(const auto_setpoint_t *sp);
/* RX task: MissionCommand.stop -- the jets stop on the next control cycle. */
void motor_control_request_auto_stop(void);
/* The last run the control task stopped and why (auto_abort_t); 0/0 = none. */
void motor_control_get_auto_abort(uint32_t *run_id, uint8_t *reason);
/* Calibration, a bench run or its SD save, or the compass calibration owns
 * (or is about to own) the jets: a mission START is refused. */
bool motor_control_jets_busy(void);
/* Manual-control traffic arrived within the link timeout. */
bool motor_control_link_alive(void);
bool motor_control_armed(void);
/* What the jets and the heading hold are doing, for the mission's status and
 * record.  Lock-free and side-effect free -- NOT motor_control_get_status(),
 * which also makes the caller the MotorStatus reader it notifies. */
typedef struct {
    float left;                /* ESC commands as written */
    float right;
    bool hold_active;          /* the heading hold is steering */
    float hold_target_deg;
    float p_term;
    float i_term;
} motor_drive_snapshot_t;
void motor_control_get_drive_snapshot(motor_drive_snapshot_t *out);

#ifdef __cplusplus
}
#endif
