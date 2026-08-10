#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SENSOR_TOF_NONE,
    SENSOR_TOF_A,
    SENSOR_TOF_B,
} sensor_tof_id_t;

typedef struct {
    uint64_t next_imu_deadline_us;
    uint64_t next_due_a_us;
    uint64_t next_due_b_us;
    uint32_t deadline_protection_skips;
    uint32_t skipped_tof_reads;
} sensor_schedule_t;

void sensor_schedule_init(sensor_schedule_t *schedule, uint64_t start_us);
uint64_t sensor_schedule_next_imu_deadline(const sensor_schedule_t *schedule);
sensor_tof_id_t sensor_schedule_choose_tof(sensor_schedule_t *schedule,
                                            uint64_t now_us,
                                            uint64_t next_imu_deadline_us,
                                            uint32_t measured_budget_us,
                                            bool tof_a_available,
                                            bool tof_b_available);
void sensor_schedule_note_tof_result(sensor_schedule_t *schedule,
                                     sensor_tof_id_t sensor,
                                     uint64_t attempted_at_us,
                                     bool succeeded);
void sensor_schedule_note_skip(sensor_schedule_t *schedule, sensor_tof_id_t sensor);
