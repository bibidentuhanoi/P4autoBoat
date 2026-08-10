#include <assert.h>
#include <math.h>

#include "control_arbiter.h"

static void assert_disabled_sources_are_rejected(control_arbiter_t *arbiter)
{
    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_WAYPOINT,
                                        0.2f, 0.1f, 1000) ==
           CONTROL_REJECT_SOURCE_DISABLED);
    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_ML,
                                        0.2f, 0.1f, 1001) ==
           CONTROL_REJECT_SOURCE_DISABLED);
    assert(control_arbiter_submit_winch(arbiter, CONTROL_SOURCE_ML, 0.2f,
                                        1002) == CONTROL_REJECT_SOURCE_DISABLED);
    assert(control_arbiter_submit_steer(arbiter, CONTROL_SOURCE_WAYPOINT,
                                        0.2f, 1003) ==
           CONTROL_REJECT_SOURCE_DISABLED);
    assert(control_arbiter_submit_steer_raw(arbiter, CONTROL_SOURCE_ML, 1500,
                                            1004) ==
           CONTROL_REJECT_SOURCE_DISABLED);
}

static void assert_mutable_selection_cannot_authorize_nonmanual_drive(void)
{
    control_arbiter_t arbiter;
    control_decision_t decision;

    control_arbiter_init(&arbiter);
    assert(control_arbiter_submit_drive(&arbiter, CONTROL_SOURCE_MANUAL,
                                        0.2f, 0.0f, 1000) == CONTROL_ACCEPTED);

    arbiter.drive[CONTROL_SOURCE_ML] = (control_drive_proposal_t){
        .throttle = 0.9f,
        .rudder = 0.0f,
        .rx_us = 2000,
        .valid = true,
        .changed = true,
    };
    arbiter.drive[CONTROL_SOURCE_WAYPOINT] = (control_drive_proposal_t){
        .throttle = -0.9f,
        .rudder = 0.0f,
        .rx_us = 2000,
        .valid = true,
        .changed = true,
    };
    assert(control_arbiter_submit_drive(&arbiter, CONTROL_SOURCE_WAYPOINT,
                                        -0.9f, 0.0f, 2000) ==
           CONTROL_REJECT_SOURCE_DISABLED);
    assert(control_arbiter_submit_drive(&arbiter, CONTROL_SOURCE_ML,
                                        0.9f, 0.0f, 2000) ==
           CONTROL_REJECT_SOURCE_DISABLED);

    control_arbiter_decide(&arbiter, 2001, &decision);
    assert(!decision.failsafe);
    assert(decision.left == 0.2f);
    assert(decision.right == 0.2f);
}

static void assert_invalid_continuous_commands_are_rejected(control_arbiter_t *arbiter)
{
    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        NAN, 0.0f, 1000) == CONTROL_REJECT_RANGE);
    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        0.0f, NAN, 1001) == CONTROL_REJECT_RANGE);
    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        1.01f, 0.0f, 1002) == CONTROL_REJECT_RANGE);
    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        0.0f, -1.01f, 1003) == CONTROL_REJECT_RANGE);
    assert(control_arbiter_submit_winch(arbiter, CONTROL_SOURCE_MANUAL, NAN,
                                        1004) == CONTROL_REJECT_RANGE);
    assert(control_arbiter_submit_winch(arbiter, CONTROL_SOURCE_MANUAL, 1.01f,
                                        1005) == CONTROL_REJECT_RANGE);
    assert(control_arbiter_submit_steer(arbiter, CONTROL_SOURCE_MANUAL, -1.01f,
                                        1006) == CONTROL_REJECT_RANGE);
    assert(control_arbiter_submit_steer_raw(arbiter, CONTROL_SOURCE_MANUAL, 399,
                                            1007) == CONTROL_REJECT_RANGE);
    assert(control_arbiter_submit_steer_raw(arbiter, CONTROL_SOURCE_MANUAL, 2601,
                                            1008) == CONTROL_REJECT_RANGE);
}

static void assert_latest_drive_is_selected_and_expires(control_arbiter_t *arbiter)
{
    control_decision_t decision;

    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        0.4f, 0.2f, 1000) == CONTROL_ACCEPTED);
    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_ML,
                                        0.8f, 0.0f, 1001) ==
           CONTROL_REJECT_SOURCE_DISABLED);

    control_arbiter_decide(arbiter, 2000, &decision);
    assert(!decision.failsafe);
    assert(decision.drive_changed);
    assert(decision.left == 0.6f);
    assert(decision.right == 0.2f);
    assert(decision.newest_rx_us == 1000);

    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        0.1f, -0.1f, 3000) == CONTROL_ACCEPTED);
    control_arbiter_decide(arbiter, 4000, &decision);
    assert(!decision.failsafe);
    assert(decision.left == 0.0f);
    assert(decision.right == 0.2f);
    assert(decision.newest_rx_us == 3000);

    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        0.9f, 0.0f, 2999) == CONTROL_REJECT_STALE);
    control_arbiter_decide(arbiter, 4001, &decision);
    assert(decision.left == 0.0f);
    assert(decision.right == 0.2f);

}

static void assert_drive_mix_is_bounded(control_arbiter_t *arbiter)
{
    control_decision_t decision;

    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        1.0f, 1.0f, 1000) == CONTROL_ACCEPTED);
    control_arbiter_decide(arbiter, 1001, &decision);
    assert(decision.left == 1.0f);
    assert(decision.right == 0.0f);
}

static void assert_safe_events_and_drive_liveness_are_independent(control_arbiter_t *arbiter)
{
    control_decision_t decision;

    assert(control_arbiter_push_event(arbiter, CONTROL_EVENT_SERVO_POWER_OFF));
    for (unsigned i = 0; i < CONTROL_EVENT_CAPACITY * 2; ++i)
        assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                            0.2f, 0.0f, 5001 + i) ==
               CONTROL_ACCEPTED);
    control_arbiter_decide(arbiter, 6000, &decision);
    assert(decision.servo_power_off);

    control_arbiter_decide(arbiter, 405100, &decision);
    assert(decision.failsafe);
}

static void assert_drive_liveness_expires_at_the_timeout_boundary(control_arbiter_t *arbiter)
{
    control_decision_t decision;

    assert(control_arbiter_submit_drive(arbiter, CONTROL_SOURCE_MANUAL,
                                        0.2f, 0.0f, 1000) == CONTROL_ACCEPTED);
    control_arbiter_decide(arbiter, 401000, &decision);
    assert(decision.failsafe);
}

static void assert_events_survive_saturation(control_arbiter_t *arbiter)
{
    control_decision_t decision;

    for (unsigned i = 0; i < CONTROL_EVENT_CAPACITY; ++i)
        assert(control_arbiter_push_event(arbiter, CONTROL_EVENT_SERVO_POWER_ON));
    assert(!control_arbiter_push_event(arbiter, CONTROL_EVENT_ARM));
    assert(!control_arbiter_push_event(arbiter, (control_event_kind_t)-1));
    assert(control_arbiter_push_event(arbiter, CONTROL_EVENT_SERVO_POWER_OFF));
    assert(control_arbiter_push_event(arbiter, CONTROL_EVENT_DISARM));

    control_arbiter_decide(arbiter, 6000, &decision);
    assert(decision.servo_power_off);
    assert(decision.disarm);
}

int main(void)
{
    control_arbiter_t arbiter;
    control_arbiter_init(&arbiter);

    assert_disabled_sources_are_rejected(&arbiter);
    assert_mutable_selection_cannot_authorize_nonmanual_drive();
    assert_invalid_continuous_commands_are_rejected(&arbiter);
    assert_latest_drive_is_selected_and_expires(&arbiter);

    control_arbiter_init(&arbiter);
    assert_drive_mix_is_bounded(&arbiter);

    control_arbiter_init(&arbiter);
    assert_safe_events_and_drive_liveness_are_independent(&arbiter);

    control_arbiter_init(&arbiter);
    assert_drive_liveness_expires_at_the_timeout_boundary(&arbiter);

    control_arbiter_init(&arbiter);
    assert_events_survive_saturation(&arbiter);
    return 0;
}
