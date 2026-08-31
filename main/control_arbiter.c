#include "control_arbiter.h"

#include <math.h>

#define CONTROL_URGENT_SERVO_POWER_OFF (1u << 0)
#define CONTROL_URGENT_DISARM (1u << 1)
#define CONTROL_STEER_RAW_MIN_US 400u
#define CONTROL_STEER_RAW_MAX_US 2600u

static bool value_is_normalized(float value)
{
    return isfinite(value) && value >= -1.0f && value <= 1.0f;
}

static float clamp_normalized(float value)
{
    if (value < -1.0f)
        return -1.0f;
    if (value > 1.0f)
        return 1.0f;
    return value;
}

static control_submit_result_t validate_drive_source(control_source_t source)
{
    if (source != CONTROL_SOURCE_MANUAL)
        return CONTROL_REJECT_SOURCE_DISABLED;
    return CONTROL_ACCEPTED;
}

static control_submit_result_t validate_manual_source(control_source_t source)
{
    if (source != CONTROL_SOURCE_MANUAL)
        return CONTROL_REJECT_SOURCE_DISABLED;
    return CONTROL_ACCEPTED;
}

static bool update_continuous(control_continuous_proposal_t *proposal,
                              float value, uint32_t raw_us, bool raw,
                              int64_t rx_us)
{
    if (proposal->valid && rx_us <= proposal->rx_us)
        return false;

    proposal->value = value;
    proposal->raw_us = raw_us;
    proposal->raw = raw;
    proposal->rx_us = rx_us;
    proposal->valid = true;
    proposal->changed = true;
    return true;
}

void control_arbiter_init(control_arbiter_t *arbiter)
{
    *arbiter = (control_arbiter_t){0};
    arbiter->was_failsafe = true;
}

control_submit_result_t control_arbiter_submit_drive(control_arbiter_t *arbiter,
                                                     control_source_t source,
                                                     float throttle, float rudder,
                                                     int64_t rx_us)
{
    control_submit_result_t source_result = validate_drive_source(source);
    control_drive_proposal_t *proposal;

    if (source_result != CONTROL_ACCEPTED)
        return source_result;
    if (!value_is_normalized(throttle) || !value_is_normalized(rudder))
        return CONTROL_REJECT_RANGE;

    proposal = &arbiter->drive[source];
    if (proposal->valid && rx_us <= proposal->rx_us)
        return CONTROL_REJECT_STALE;

    proposal->throttle = throttle;
    proposal->rudder = rudder;
    proposal->rx_us = rx_us;
    proposal->valid = true;
    proposal->changed = true;
    return CONTROL_ACCEPTED;
}

control_submit_result_t control_arbiter_submit_winch(control_arbiter_t *arbiter,
                                                     control_source_t source,
                                                     float winch, int64_t rx_us)
{
    control_submit_result_t source_result = validate_manual_source(source);

    if (source_result != CONTROL_ACCEPTED)
        return source_result;
    if (!value_is_normalized(winch))
        return CONTROL_REJECT_RANGE;
    if (!update_continuous(&arbiter->winch, winch, 0, false, rx_us))
        return CONTROL_REJECT_STALE;
    return CONTROL_ACCEPTED;
}

control_submit_result_t control_arbiter_submit_steer(control_arbiter_t *arbiter,
                                                     control_source_t source,
                                                     float steer, int64_t rx_us)
{
    control_submit_result_t source_result = validate_manual_source(source);

    if (source_result != CONTROL_ACCEPTED)
        return source_result;
    if (!value_is_normalized(steer))
        return CONTROL_REJECT_RANGE;
    if (!update_continuous(&arbiter->steer, steer, 0, false, rx_us))
        return CONTROL_REJECT_STALE;
    return CONTROL_ACCEPTED;
}

control_submit_result_t control_arbiter_submit_steer_raw(control_arbiter_t *arbiter,
                                                         control_source_t source,
                                                         uint32_t pulse_us,
                                                         int64_t rx_us)
{
    control_submit_result_t source_result = validate_manual_source(source);

    if (source_result != CONTROL_ACCEPTED)
        return source_result;
    if (pulse_us < CONTROL_STEER_RAW_MIN_US || pulse_us > CONTROL_STEER_RAW_MAX_US)
        return CONTROL_REJECT_RANGE;
    if (!update_continuous(&arbiter->steer, 0.0f, pulse_us, true, rx_us))
        return CONTROL_REJECT_STALE;
    return CONTROL_ACCEPTED;
}

bool control_arbiter_push_event(control_arbiter_t *arbiter,
                                control_event_kind_t event)
{
    uint8_t tail;

    if (event < CONTROL_EVENT_SERVO_POWER_ON || event >= CONTROL_EVENT_KIND_COUNT)
        return false;
    if (event == CONTROL_EVENT_SERVO_POWER_OFF)
        arbiter->urgent_events |= CONTROL_URGENT_SERVO_POWER_OFF;
    if (event == CONTROL_EVENT_DISARM)
        arbiter->urgent_events |= CONTROL_URGENT_DISARM;
    if (arbiter->event_count == CONTROL_EVENT_CAPACITY)
        return event == CONTROL_EVENT_SERVO_POWER_OFF || event == CONTROL_EVENT_DISARM;

    tail = (uint8_t)((arbiter->event_head + arbiter->event_count) % CONTROL_EVENT_CAPACITY);
    arbiter->events[tail] = event;
    ++arbiter->event_count;
    return true;
}

static void apply_event(control_decision_t *out, control_event_kind_t event)
{
    switch (event) {
    case CONTROL_EVENT_SERVO_POWER_ON:
        out->servo_power_on = true;
        break;
    case CONTROL_EVENT_SERVO_POWER_OFF:
        out->servo_power_off = true;
        break;
    case CONTROL_EVENT_ARM:
        out->arm = true;
        break;
    case CONTROL_EVENT_FORCE_ARM:
        out->force_arm = true;
        break;
    case CONTROL_EVENT_DISARM:
        out->disarm = true;
        break;
    default:
        break;
    }
}

void control_arbiter_decide(control_arbiter_t *arbiter, int64_t now_us,
                            control_decision_t *out)
{
    control_drive_proposal_t *drive = &arbiter->drive[CONTROL_SOURCE_MANUAL];

    *out = (control_decision_t){0};
    if (!drive->valid || now_us - drive->rx_us >= CONTROL_DRIVE_TIMEOUT_US) {
        out->failsafe = true;
    } else {
        out->throttle = drive->throttle;
        out->rudder = drive->rudder;
        out->left = clamp_normalized(drive->throttle + drive->rudder);
        out->right = clamp_normalized(drive->throttle - drive->rudder);
        out->newest_rx_us = drive->rx_us;
    }
    out->drive_changed = drive->changed || out->failsafe != arbiter->was_failsafe;
    drive->changed = false;
    arbiter->was_failsafe = out->failsafe;

    if (arbiter->winch.valid) {
        out->winch = arbiter->winch.value;
        out->winch_changed = arbiter->winch.changed;
        arbiter->winch.changed = false;
    }
    if (arbiter->steer.valid) {
        out->steer = arbiter->steer.value;
        out->steer_raw_us = arbiter->steer.raw_us;
        out->steer_raw = arbiter->steer.raw;
        out->steer_changed = arbiter->steer.changed;
        arbiter->steer.changed = false;
    }

    while (arbiter->event_count != 0) {
        apply_event(out, arbiter->events[arbiter->event_head]);
        arbiter->event_head = (uint8_t)((arbiter->event_head + 1) % CONTROL_EVENT_CAPACITY);
        --arbiter->event_count;
    }
    if ((arbiter->urgent_events & CONTROL_URGENT_SERVO_POWER_OFF) != 0)
        out->servo_power_off = true;
    if ((arbiter->urgent_events & CONTROL_URGENT_DISARM) != 0)
        out->disarm = true;
    arbiter->urgent_events = 0;
}
