#include "arm_sequence.h"

#include <stddef.h>

void arm_sequence_init(arm_sequence_t *sequence, int64_t arming_duration_us)
{
    if (!sequence) return;

    sequence->state = ARM_SEQUENCE_DISARMED;
    sequence->arming_duration_us = arming_duration_us;
    sequence->deadline_us = 0;
    sequence->requested_us = 0;
    sequence->force = false;
}

bool arm_sequence_request(arm_sequence_t *sequence, arm_request_t request,
                          bool force, int64_t now_us)
{
    if (!sequence) return false;

    if (request == ARM_REQUEST_DISARM) {
        sequence->state = ARM_SEQUENCE_DISARM_PENDING;
        sequence->deadline_us = 0;
        sequence->requested_us = now_us;
        sequence->force = false;
        return true;
    }

    if (request != ARM_REQUEST_ARM || sequence->state != ARM_SEQUENCE_DISARMED) {
        return false;
    }

    sequence->state = ARM_SEQUENCE_ARM_PENDING;
    sequence->requested_us = now_us;
    sequence->force = force;
    return true;
}

arm_action_t arm_sequence_step(arm_sequence_t *sequence, int64_t now_us,
                               bool gps_locked)
{
    if (!sequence) return ARM_ACTION_NONE;

    switch (sequence->state) {
    case ARM_SEQUENCE_DISARM_PENDING:
        sequence->state = ARM_SEQUENCE_DISARMED;
        sequence->deadline_us = 0;
        return ARM_ACTION_DISARM;

    case ARM_SEQUENCE_ARM_PENDING:
        if (!sequence->force && !gps_locked) {
            sequence->state = ARM_SEQUENCE_DISARMED;
            return ARM_ACTION_REJECT_NO_GPS;
        }
        sequence->state = ARM_SEQUENCE_ARMING;
        sequence->deadline_us = now_us + sequence->arming_duration_us;
        return ARM_ACTION_BEGIN;

    case ARM_SEQUENCE_ARMING:
        if (now_us >= sequence->deadline_us) {
            sequence->state = ARM_SEQUENCE_ARMED;
            sequence->deadline_us = 0;
            return ARM_ACTION_COMPLETE;
        }
        return ARM_ACTION_NONE;

    case ARM_SEQUENCE_DISARMED:
    case ARM_SEQUENCE_ARMED:
    default:
        return ARM_ACTION_NONE;
    }
}
