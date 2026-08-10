#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ARM_REQUEST_ARM,
    ARM_REQUEST_DISARM,
} arm_request_t;

typedef enum {
    ARM_ACTION_NONE,
    ARM_ACTION_REJECT_NO_GPS,
    ARM_ACTION_BEGIN,
    ARM_ACTION_COMPLETE,
    ARM_ACTION_DISARM,
} arm_action_t;

typedef enum {
    ARM_SEQUENCE_DISARMED,
    ARM_SEQUENCE_ARM_PENDING,
    ARM_SEQUENCE_ARMING,
    ARM_SEQUENCE_ARMED,
    ARM_SEQUENCE_DISARM_PENDING,
} arm_sequence_state_t;

typedef struct {
    arm_sequence_state_t state;
    int64_t arming_duration_us;
    int64_t deadline_us;
    int64_t requested_us;
    bool force;
} arm_sequence_t;

void arm_sequence_init(arm_sequence_t *sequence, int64_t arming_duration_us);
bool arm_sequence_request(arm_sequence_t *sequence, arm_request_t request,
                          bool force, int64_t now_us);
arm_action_t arm_sequence_step(arm_sequence_t *sequence, int64_t now_us,
                               bool gps_locked);

#ifdef __cplusplus
}
#endif
