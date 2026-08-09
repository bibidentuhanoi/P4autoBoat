#pragma once

#include <stdatomic.h>
#include <stdint.h>

#include "imu_sample.h"

_Static_assert(sizeof(unsigned int) == 4, "snapshot word atomics require 32-bit unsigned int");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "snapshot word atomics must be lock-free");

/* Single-writer, many-reader double buffer. Every payload field uses a
 * lock-free 32-bit atomic; the timestamp is two protected words rather than a
 * potentially non-lock-free 64-bit atomic. */
typedef struct {
    atomic_uint version;
    atomic_uint ax;
    atomic_uint ay;
    atomic_uint az;
    atomic_uint gx;
    atomic_uint gy;
    atomic_uint gz;
    atomic_uint mx;
    atomic_uint my;
    atomic_uint mz;
    atomic_uint validity;
    atomic_uint captured_us_low;
    atomic_uint captured_us_high;
    atomic_uint sequence;
} sample_snapshot_slot_t;

typedef struct {
    sample_snapshot_slot_t slots[2];
    atomic_uint published_sequence;
} sample_snapshot_t;

void sample_snapshot_init(sample_snapshot_t *snapshot);
void sample_snapshot_publish(sample_snapshot_t *snapshot, const imu_sample_t *sample);
bool sample_snapshot_read(const sample_snapshot_t *snapshot, imu_sample_t *sample);
