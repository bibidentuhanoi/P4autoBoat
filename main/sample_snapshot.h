#pragma once

#include <stdatomic.h>
#include <stdint.h>

#include "imu_sample.h"

/* Single-writer, many-reader double buffer.  Each slot is sequence-locked and
 * every stored field is atomic, so readers can retry without observing a C data
 * race while the writer reuses the other slot. */
typedef struct {
    atomic_uint_fast32_t version;
    atomic_int_least16_t ax;
    atomic_int_least16_t ay;
    atomic_int_least16_t az;
    atomic_int_least16_t gx;
    atomic_int_least16_t gy;
    atomic_int_least16_t gz;
    atomic_int_least16_t mx;
    atomic_int_least16_t my;
    atomic_int_least16_t mz;
    atomic_bool accel_gyro_valid;
    atomic_bool mag_valid;
    atomic_uint_fast64_t captured_us;
    atomic_uint_fast32_t sequence;
} sample_snapshot_slot_t;

typedef struct {
    sample_snapshot_slot_t slots[2];
    atomic_uint_fast32_t published_sequence;
} sample_snapshot_t;

void sample_snapshot_init(sample_snapshot_t *snapshot);
void sample_snapshot_publish(sample_snapshot_t *snapshot, const imu_sample_t *sample);
bool sample_snapshot_read(const sample_snapshot_t *snapshot, imu_sample_t *sample);
