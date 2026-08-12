#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_EVENT_CAPACITY 8u
#define CONTROL_DRIVE_TIMEOUT_US 400000LL

typedef enum {
    CONTROL_SOURCE_MANUAL,
    CONTROL_SOURCE_WAYPOINT,
    CONTROL_SOURCE_ML,
    CONTROL_SOURCE_COUNT,
} control_source_t;

typedef enum {
    CONTROL_EVENT_SERVO_POWER_ON,
    CONTROL_EVENT_SERVO_POWER_OFF,
    CONTROL_EVENT_ARM,
    CONTROL_EVENT_FORCE_ARM,
    CONTROL_EVENT_DISARM,
    CONTROL_EVENT_KIND_COUNT,
} control_event_kind_t;

typedef enum {
    CONTROL_ACCEPTED,
    CONTROL_REJECT_SOURCE_DISABLED,
    CONTROL_REJECT_RANGE,
    CONTROL_REJECT_STALE,
} control_submit_result_t;

typedef struct {
    float left;
    float right;
    /* Original manual command, retained so downstream shaping can operate
     * before the saturated left/right mix loses pilot intent. */
    float throttle;
    float rudder;
    float winch;
    float steer;
    uint32_t steer_raw_us;
    bool steer_raw;
    bool drive_changed;
    bool winch_changed;
    bool steer_changed;
    bool servo_power_on;
    bool servo_power_off;
    bool arm;
    bool force_arm;
    bool disarm;
    bool failsafe;
    int64_t newest_rx_us;
} control_decision_t;

typedef struct {
    float throttle;
    float rudder;
    int64_t rx_us;
    bool valid;
    bool changed;
} control_drive_proposal_t;

typedef struct {
    float value;
    uint32_t raw_us;
    int64_t rx_us;
    bool valid;
    bool raw;
    bool changed;
} control_continuous_proposal_t;

typedef struct {
    control_drive_proposal_t drive[CONTROL_SOURCE_COUNT];
    control_continuous_proposal_t winch;
    control_continuous_proposal_t steer;
    control_event_kind_t events[CONTROL_EVENT_CAPACITY];
    uint8_t event_head;
    uint8_t event_count;
    uint8_t urgent_events;
    bool was_failsafe;
} control_arbiter_t;

void control_arbiter_init(control_arbiter_t *arbiter);
control_submit_result_t control_arbiter_submit_drive(control_arbiter_t *arbiter,
                                                     control_source_t source,
                                                     float throttle, float rudder,
                                                     int64_t rx_us);
control_submit_result_t control_arbiter_submit_winch(control_arbiter_t *arbiter,
                                                     control_source_t source,
                                                     float winch, int64_t rx_us);
control_submit_result_t control_arbiter_submit_steer(control_arbiter_t *arbiter,
                                                     control_source_t source,
                                                     float steer, int64_t rx_us);
control_submit_result_t control_arbiter_submit_steer_raw(control_arbiter_t *arbiter,
                                                         control_source_t source,
                                                         uint32_t pulse_us,
                                                         int64_t rx_us);
bool control_arbiter_push_event(control_arbiter_t *arbiter,
                                control_event_kind_t event);
void control_arbiter_decide(control_arbiter_t *arbiter, int64_t now_us,
                            control_decision_t *out);
