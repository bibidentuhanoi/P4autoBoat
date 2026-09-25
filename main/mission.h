#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "course_error.h"
#include "nav_geo.h"

/* The out-and-back mission: pure logic, stepped by the autonomy task at 20 Hz.
 * It never touches hardware.  Its output is the setpoint the control task
 * turns into jet commands through the heading hold.
 * Spec: docs/superpowers/specs/2026-09-24-out-and-back-mission-design.md */

typedef enum {
    MISSION_IDLE = 0,
    MISSION_HOME = 1,       /* motors 0, averaging GPS for home */
    MISSION_OUTBOUND = 2,
    MISSION_TURN = 3,
    MISSION_RETURN = 4,
    MISSION_DONE = 5,
    MISSION_ABORTED = 6,
    MISSION_REFUSED = 7,
} mission_state_t;

typedef enum {
    MISSION_STAGE_OUT = 1,        /* out only */
    MISSION_STAGE_OUT_TURN = 2,   /* out, then turn to face home */
    MISSION_STAGE_FULL = 3,       /* out, turn, home */
} mission_stage_t;

/* Why the mission is where it is (MissionStatus.reason). */
typedef enum {
    MISSION_REASON_NONE = 0,
    /* DONE */
    MISSION_DONE_OUT = 1,          /* stage 1: out distance reached */
    MISSION_DONE_TURNED = 2,       /* stage 2: turned to face home */
    MISSION_DONE_IN_ZONE = 3,      /* inside the home radius */
    MISSION_DONE_PASSED = 4,       /* past the finish line through home, within the approach radius */
    MISSION_DONE_NEAR = 5,         /* moving away after the closest approach, within the approach radius */
    /* ABORTED */
    MISSION_ABORT_STOP = 10,
    MISSION_ABORT_MANUAL = 11,
    MISSION_ABORT_DISARMED = 12,
    MISSION_ABORT_RAIL_CUT = 13,
    MISSION_ABORT_OTHER_OWNER = 14,
    MISSION_ABORT_STALE = 15,       /* the control task saw no fresh setpoint */
    MISSION_ABORT_GPS_LOST = 16,
    MISSION_ABORT_HEADING_LOST = 17,
    MISSION_ABORT_HOME_NOT_STILL = 18,
    MISSION_ABORT_SPIN = 19,
    MISSION_ABORT_GPS_FROZEN = 20,   /* fixes keep coming but the position never moves */
    /* REFUSED (nothing moved) */
    MISSION_REFUSE_SETTINGS = 30,
    MISSION_REFUSE_NO_HOLD = 31,     /* heading hold not built in */
    MISSION_REFUSE_BUSY = 32,        /* calibration, bench run or record save */
    MISSION_REFUSE_NOT_ARMED = 33,
    MISSION_REFUSE_GPS_NOT_UBX = 34,
    MISSION_REFUSE_NO_3D_FIX = 35,
    MISSION_REFUSE_FEW_SATS = 36,
    MISSION_REFUSE_PDOP = 37,
    MISSION_REFUSE_FIX_OLD = 38,
    MISSION_REFUSE_COMPASS = 39,     /* not calibrated */
    MISSION_REFUSE_FIELD = 40,       /* field strength far from calibration */
    MISSION_REFUSE_IMU = 41,
    MISSION_REFUSE_HEADING = 42,
} mission_reason_t;

typedef struct {
    uint32_t stage;            /* mission_stage_t */
    float out_distance_m;      /* 3..50 */
    float home_radius_m;       /* 1.5..10 */
    float throttle;            /* 0.20..0.60 */
    float approach_throttle;   /* 0.15..throttle */
    bool turn_right;
    bool dry_run;              /* motors stay 0; the boat is carried */
} mission_settings_t;

typedef struct {
    uint32_t home_fixes;       /* fixes averaged for home */
    float home_spread_m;       /* ...all within this of their mean */
    float home_timeout_s;
    uint32_t start_min_sats;
    float start_max_pdop;
    float start_max_fix_age_s;
    uint32_t run_min_sats;
    float run_max_fix_age_s;
    float max_fusion_age_s;
    float field_ratio_min;
    float field_ratio_max;
    float turn_force_deg;      /* forced direction while |error| is above this */
    float turn_lead_deg;       /* the forced target leads the heading by this */
    float turn_done_deg;       /* the turn ends within this of the course home */
    float approach_factor;     /* approach radius = factor * home radius */
    float lookahead_m;         /* LOS lookahead on the return line */
    uint32_t arrive_fixes;     /* consecutive fixes confirming an arrival rule */
    float slip_margin_m;       /* "near": this much farther than the closest approach */
    float spin_limit_deg;      /* any leg turning more than this = abort */
    float outbound_settle_s;   /* no course samples in the first seconds out */
    float frozen_speed_mps;    /* GPS says we move faster than this ... */
    uint32_t frozen_fixes;     /* ...yet the position moved < frozen_move_m over this many fixes */
    float frozen_move_m;
    float jump_gate_m;         /* a fix this far from the predicted position is a jump */
    uint32_t jump_accept_fixes;/* ...unless it persists this many fixes: then it is real */
    course_err_cfg_t course;
} mission_cfg_t;

typedef struct {
    float t_s;                   /* monotonic seconds */
    /* operator requests: true only on the tick they arrive */
    bool start;
    uint32_t request_id;
    mission_settings_t settings;
    bool stop;
    /* GPS, UBX NAV-PVT */
    bool gps_new;                /* a fix newer than the previous tick's */
    bool gps_ubx;
    uint8_t fix_type;            /* 3 = 3D */
    uint8_t sats;
    float pdop;
    float fix_age_s;
    double lat_deg;
    double lon_deg;
    float speed_mps;
    float course_deg;
    float speed_acc_mps;
    /* attitude (fused) */
    bool heading_valid;
    float heading_deg;
    float yaw_rate_dps;          /* + = turning LEFT on this boat */
    float fusion_age_s;
    bool imu_ok;
    bool compass_calibrated;
    float field_ratio;
    /* boat */
    bool armed;
    bool busy;
    bool heading_hold_built;
    bool link_alive;
    /* the control task's report: it stopped this run, and why (auto_abort_t) */
    uint32_t control_abort_run;
    uint8_t control_abort_reason;
} mission_input_t;

typedef struct {
    bool active;          /* the mission owns the motors (they may be held at 0) */
    bool drive;           /* the motors may run */
    bool dry_run;
    float heading_deg;    /* wanted heading for the heading hold */
    float throttle;
} mission_output_t;

#define MISSION_HOME_MAX 32

typedef struct {
    mission_state_t state;
    uint8_t reason;
    uint32_t run_id;
    uint32_t request_id;
    bool any_request;
    mission_settings_t settings;
    float t_start_s;
    float t_state_s;
    float t_last_s;
    bool have_t;
    /* home */
    nav_origin_t origin;
    bool have_origin;
    nav_origin_t home_try_origin;
    uint32_t home_n;
    nav_en_t home_samples[MISSION_HOME_MAX];
    double hdg_sin;
    double hdg_cos;
    float t_home_start_s;
    double home_lat_deg;
    double home_lon_deg;
    float start_heading_deg;
    nav_en_t out_point;
    /* live navigation */
    bool have_pos;
    nav_en_t pos;            /* last accepted fix, dead-reckoned through jumps */
    float t_fix_s;
    float vel_e;             /* GPS velocity of the last fix */
    float vel_n;
    uint32_t outlier_run;
    uint32_t outliers;       /* fixes rejected as jumps this run */
    uint32_t frozen_count;
    nav_en_t frozen_ref;     /* raw fix position the frozen check compares against */
    float dist_home_m;
    float dist_target_m;
    float bearing_target_deg;
    nav_line_pos_t line;
    float wanted_course_deg;
    float wanted_heading_deg;
    bool approach;
    /* turn */
    float turned_deg;            /* + = right, gyro */
    float turn_peak_dps;
    float t_turn_start_s;
    nav_en_t turn_start;
    nav_en_t return_start;
    float leg_turn_deg;          /* signed rotation in the current leg (spin guard) */
    /* arrival */
    uint32_t zone_count;
    uint32_t pass_count;
    uint32_t slip_count;
    float closest_m;
    /* beta = GPS course - compass heading */
    course_err_mean_t out_mean;
    course_err_result_t out_result;
    float beta_deg;
    bool beta_valid;
    bool compass_bad;
    float t_beta_last_s;
    uint32_t beta_return_n;
    /* link */
    uint32_t outages;
    float longest_outage_s;
    float t_outage_start_s;
    bool in_outage;
} mission_t;

mission_cfg_t mission_cfg_default(void);
mission_settings_t mission_settings_default(void);
bool mission_settings_valid(const mission_settings_t *s);
void mission_init(mission_t *m);
bool mission_is_active(const mission_t *m);
mission_output_t mission_step(mission_t *m, const mission_cfg_t *cfg,
                              const mission_input_t *in);
/* Maps the control task's auto_abort_t code to a mission reason. */
uint8_t mission_reason_from_control(uint8_t auto_abort);
