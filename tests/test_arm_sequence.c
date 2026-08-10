#include "arm_sequence.h"

#include <assert.h>

static void test_arm_requires_gps_unless_forced(void)
{
    arm_sequence_t sequence;
    arm_sequence_init(&sequence, 3000000);

    assert(arm_sequence_request(&sequence, ARM_REQUEST_ARM, false, 1000));
    assert(arm_sequence_step(&sequence, 1000, false) == ARM_ACTION_REJECT_NO_GPS);

    assert(arm_sequence_request(&sequence, ARM_REQUEST_ARM, true, 2000));
    assert(arm_sequence_step(&sequence, 2000, false) == ARM_ACTION_BEGIN);
    assert(arm_sequence_step(&sequence, 3001999, false) == ARM_ACTION_NONE);
    assert(arm_sequence_step(&sequence, 3002000, false) == ARM_ACTION_COMPLETE);
}

static void test_disarm_cancels_pending_completion(void)
{
    arm_sequence_t sequence;
    arm_sequence_init(&sequence, 3000000);

    assert(arm_sequence_request(&sequence, ARM_REQUEST_ARM, true, 0));
    assert(arm_sequence_step(&sequence, 0, false) == ARM_ACTION_BEGIN);
    assert(arm_sequence_request(&sequence, ARM_REQUEST_DISARM, false, 1000000));
    assert(arm_sequence_step(&sequence, 1000000, false) == ARM_ACTION_DISARM);
    assert(arm_sequence_step(&sequence, 4000000, false) == ARM_ACTION_NONE);
}

int main(void)
{
    test_arm_requires_gps_unless_forced();
    test_disarm_cancels_pending_completion();
    return 0;
}
