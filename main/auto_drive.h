#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "yaw_heading_control.h"

/* The control task's side of the mission ("AUTO"): pure decisions, stepped
 * every 10 ms control tick.  The autonomy task hands over a setpoint; this
 * decides whether the mission owns the jets this tick, and stops a run for
 * good the moment anything says so.  No ESP-IDF dependencies. */

/* The autonomy task's wish for the jets. */
typedef struct {
    uint32_t run_id;       /* mission run; 0 = none has ever run */
    bool active;           /* the mission owns the jets (they may be held at 0) */
    bool drive;            /* ...and may run them */
    bool dry_run;          /* carried on land: the jets must stay 0 */
    float heading_deg;     /* wanted heading, 0..360 */
    float throttle;        /* 0..1 when drive */
    int64_t stamp_us;      /* when the autonomy task wrote it */
} auto_setpoint_t;

/* Why the control task stopped a run.  mission.c repeats these numbers. */
typedef enum {
    AUTO_ABORT_NONE = 0,
    AUTO_ABORT_STOP = 1,          /* MissionCommand.stop */
    AUTO_ABORT_MANUAL = 2,        /* a non-zero manual command was accepted */
    AUTO_ABORT_DISARMED = 3,
    AUTO_ABORT_RAIL_CUT = 4,      /* explicit servo-rail PWR-OFF */
    AUTO_ABORT_OTHER_OWNER = 5,   /* calibration or bench took the jets */
    AUTO_ABORT_STALE = 6,         /* no fresh setpoint for AUTO_SETPOINT_MAX_AGE_US */
} auto_abort_t;

typedef struct {
    int64_t now_us;
    bool armed;
    bool rail_cut;
    bool other_owner;
    bool manual_input;     /* a NEW non-zero manual command this tick */
    bool stop_request;     /* MissionCommand.stop arrived */
} auto_inputs_t;

typedef struct {
    bool own;              /* the mission owns the jets this tick */
    bool drive;            /* ...and runs them */
    float throttle;
    float heading_deg;
    auto_abort_t abort;    /* set on the tick a run is stopped */
} auto_out_t;

typedef struct {
    uint32_t aborted_run_id;     /* this run is over for the control task */
    auto_abort_t abort_reason;
} auto_drive_t;

#define AUTO_SETPOINT_MAX_AGE_US 500000LL

void auto_drive_init(auto_drive_t *st);
auto_out_t auto_drive_step(auto_drive_t *st, const auto_setpoint_t *sp,
                           const auto_inputs_t *in);

/* The heading hold's input while the mission owns the jets: the same safety
 * gates as manual P (armed, fresh gyro, throttle floor -- the caller fills
 * those), the mission's target, no steering, no capture.  `in->driving` must
 * already be set. */
void auto_drive_hold_input(const auto_out_t *out, yaw_heading_input_t *in);

/* Left/right for a straight-ahead mission throttle, the same mix as the manual
 * path: learned c plus the hold's dynamic c, limited by the throttle's
 * headroom (1/T - 1, at most 1), through esc_trim_mix. */
void auto_drive_mix(float throttle, float learned_c, float dynamic_c,
                    bool hold_active, float *left, float *right);
