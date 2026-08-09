#include <assert.h>

#include "sensor_schedule.h"

static void test_due_sensors_alternate_at_five_hz(void)
{
    sensor_schedule_t schedule;

    sensor_schedule_init(&schedule, 0);

    assert(sensor_schedule_next_imu_deadline(&schedule) == 20000);
    assert(sensor_schedule_choose_tof(&schedule, 3000, 20000, 12000,
                                      true, true) == SENSOR_TOF_A);
    sensor_schedule_note_tof_result(&schedule, SENSOR_TOF_A, 12000, true);
    assert(sensor_schedule_choose_tof(&schedule, 103000, 120000, 12000,
                                      true, true) == SENSOR_TOF_B);
    sensor_schedule_note_tof_result(&schedule, SENSOR_TOF_B, 12000, true);
    assert(sensor_schedule_choose_tof(&schedule, 203000, 220000, 12000,
                                      true, true) == SENSOR_TOF_A);
}

static void test_tof_never_uses_the_imu_guard_window(void)
{
    sensor_schedule_t schedule;

    sensor_schedule_init(&schedule, 0);

    assert(sensor_schedule_choose_tof(&schedule, 7000, 20000, 12000,
                                      true, true) == SENSOR_TOF_NONE);
    assert(schedule.deadline_protection_skips == 1);
    assert(schedule.next_due_a_us == 0);

    assert(sensor_schedule_choose_tof(&schedule, 8000, 22000, 12000,
                                      true, true) == SENSOR_TOF_A);
}

static void test_tof_can_finish_exactly_at_the_guard_boundary(void)
{
    sensor_schedule_t schedule;

    sensor_schedule_init(&schedule, 0);
    assert(sensor_schedule_choose_tof(&schedule, 6000, 20000, 12000,
                                      true, true) == SENSOR_TOF_A);
}

static void test_unavailable_sensors_do_not_block_available_due_sensor(void)
{
    sensor_schedule_t schedule;

    sensor_schedule_init(&schedule, 0);
    assert(sensor_schedule_choose_tof(&schedule, 3000, 20000, 12000,
                                      false, true) == SENSOR_TOF_NONE);
    assert(sensor_schedule_choose_tof(&schedule, 103000, 120000, 12000,
                                      false, true) == SENSOR_TOF_B);

    sensor_schedule_init(&schedule, 0);
    assert(sensor_schedule_choose_tof(&schedule, 3000, 20000, 12000,
                                      true, false) == SENSOR_TOF_A);

    sensor_schedule_init(&schedule, 0);
    assert(sensor_schedule_choose_tof(&schedule, 203000, 220000, 12000,
                                      false, false) == SENSOR_TOF_NONE);
}

static void test_failed_attempt_advances_only_the_attempted_sensor(void)
{
    sensor_schedule_t schedule;

    sensor_schedule_init(&schedule, 0);
    assert(sensor_schedule_choose_tof(&schedule, 3000, 20000, 12000,
                                      true, true) == SENSOR_TOF_A);
    sensor_schedule_note_tof_result(&schedule, SENSOR_TOF_A, 12000, false);

    assert(schedule.next_due_a_us == 200000);
    assert(schedule.next_due_b_us == 100000);
    assert(sensor_schedule_choose_tof(&schedule, 103000, 120000, 12000,
                                      true, true) == SENSOR_TOF_B);
}

static void test_explicit_skip_retries_the_oldest_due_sensor(void)
{
    sensor_schedule_t schedule;

    sensor_schedule_init(&schedule, 0);
    assert(sensor_schedule_choose_tof(&schedule, 3000, 20000, 12000,
                                      true, true) == SENSOR_TOF_A);
    sensor_schedule_note_skip(&schedule, SENSOR_TOF_A);

    assert(schedule.next_due_a_us == 0);
    assert(sensor_schedule_choose_tof(&schedule, 4000, 22000, 12000,
                                      true, true) == SENSOR_TOF_A);
}

int main(void)
{
    test_due_sensors_alternate_at_five_hz();
    test_tof_never_uses_the_imu_guard_window();
    test_tof_can_finish_exactly_at_the_guard_boundary();
    test_unavailable_sensors_do_not_block_available_due_sensor();
    test_failed_attempt_advances_only_the_attempted_sensor();
    test_explicit_skip_retries_the_oldest_due_sensor();
    return 0;
}
