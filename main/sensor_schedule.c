#include "sensor_schedule.h"

#define SENSOR_IMU_PERIOD_US 20000U
#define SENSOR_TOF_PERIOD_US 200000U
#define SENSOR_TOF_B_OFFSET_US 100000U
#define SENSOR_TOF_GUARD_US 2000U

void sensor_schedule_init(sensor_schedule_t *schedule, uint64_t start_us)
{
    *schedule = (sensor_schedule_t){
        .next_imu_deadline_us = start_us + SENSOR_IMU_PERIOD_US,
        .next_due_a_us = start_us,
        .next_due_b_us = start_us + SENSOR_TOF_B_OFFSET_US,
    };
}

uint64_t sensor_schedule_next_imu_deadline(const sensor_schedule_t *schedule)
{
    return schedule->next_imu_deadline_us;
}

static sensor_tof_id_t sensor_schedule_oldest_available_due(
    const sensor_schedule_t *schedule, uint64_t now_us, bool tof_a_available,
    bool tof_b_available)
{
    bool a_due = tof_a_available && schedule->next_due_a_us <= now_us;
    bool b_due = tof_b_available && schedule->next_due_b_us <= now_us;

    if (!a_due && !b_due) {
        return SENSOR_TOF_NONE;
    }
    if (!b_due || (a_due && schedule->next_due_a_us <= schedule->next_due_b_us)) {
        return SENSOR_TOF_A;
    }
    return SENSOR_TOF_B;
}

sensor_tof_id_t sensor_schedule_choose_tof(sensor_schedule_t *schedule,
                                            uint64_t now_us,
                                            uint64_t next_imu_deadline_us,
                                            uint32_t measured_budget_us,
                                            bool tof_a_available,
                                            bool tof_b_available)
{
    sensor_tof_id_t sensor = sensor_schedule_oldest_available_due(
        schedule, now_us, tof_a_available, tof_b_available);

    if (sensor == SENSOR_TOF_NONE) {
        return SENSOR_TOF_NONE;
    }

    if (next_imu_deadline_us < now_us ||
        next_imu_deadline_us - now_us <
            (uint64_t)measured_budget_us + SENSOR_TOF_GUARD_US) {
        schedule->deadline_protection_skips++;
        return SENSOR_TOF_NONE;
    }

    return sensor;
}

void sensor_schedule_note_tof_result(sensor_schedule_t *schedule,
                                     sensor_tof_id_t sensor,
                                     uint64_t attempted_at_us,
                                     bool succeeded)
{
    (void)attempted_at_us;
    (void)succeeded;

    if (sensor == SENSOR_TOF_A) {
        schedule->next_due_a_us += SENSOR_TOF_PERIOD_US;
    } else if (sensor == SENSOR_TOF_B) {
        schedule->next_due_b_us += SENSOR_TOF_PERIOD_US;
    }
}

void sensor_schedule_note_skip(sensor_schedule_t *schedule, sensor_tof_id_t sensor)
{
    if (sensor != SENSOR_TOF_NONE) {
        schedule->skipped_tof_reads++;
    }
}
