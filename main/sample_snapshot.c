#include "sample_snapshot.h"

static void sample_snapshot_store(sample_snapshot_slot_t *slot, const imu_sample_t *sample)
{
    atomic_store_explicit(&slot->ax, sample->ax, memory_order_relaxed);
    atomic_store_explicit(&slot->ay, sample->ay, memory_order_relaxed);
    atomic_store_explicit(&slot->az, sample->az, memory_order_relaxed);
    atomic_store_explicit(&slot->gx, sample->gx, memory_order_relaxed);
    atomic_store_explicit(&slot->gy, sample->gy, memory_order_relaxed);
    atomic_store_explicit(&slot->gz, sample->gz, memory_order_relaxed);
    atomic_store_explicit(&slot->mx, sample->mx, memory_order_relaxed);
    atomic_store_explicit(&slot->my, sample->my, memory_order_relaxed);
    atomic_store_explicit(&slot->mz, sample->mz, memory_order_relaxed);
    atomic_store_explicit(&slot->accel_gyro_valid, sample->accel_gyro_valid, memory_order_relaxed);
    atomic_store_explicit(&slot->mag_valid, sample->mag_valid, memory_order_relaxed);
    atomic_store_explicit(&slot->captured_us, sample->captured_us, memory_order_relaxed);
    atomic_store_explicit(&slot->sequence, sample->sequence, memory_order_relaxed);
}

static void sample_snapshot_load(const sample_snapshot_slot_t *slot, imu_sample_t *sample)
{
    sample->ax = (int16_t)atomic_load_explicit(&slot->ax, memory_order_relaxed);
    sample->ay = (int16_t)atomic_load_explicit(&slot->ay, memory_order_relaxed);
    sample->az = (int16_t)atomic_load_explicit(&slot->az, memory_order_relaxed);
    sample->gx = (int16_t)atomic_load_explicit(&slot->gx, memory_order_relaxed);
    sample->gy = (int16_t)atomic_load_explicit(&slot->gy, memory_order_relaxed);
    sample->gz = (int16_t)atomic_load_explicit(&slot->gz, memory_order_relaxed);
    sample->mx = (int16_t)atomic_load_explicit(&slot->mx, memory_order_relaxed);
    sample->my = (int16_t)atomic_load_explicit(&slot->my, memory_order_relaxed);
    sample->mz = (int16_t)atomic_load_explicit(&slot->mz, memory_order_relaxed);
    sample->accel_gyro_valid = atomic_load_explicit(&slot->accel_gyro_valid, memory_order_relaxed);
    sample->mag_valid = atomic_load_explicit(&slot->mag_valid, memory_order_relaxed);
    sample->captured_us = atomic_load_explicit(&slot->captured_us, memory_order_relaxed);
    sample->sequence = (uint32_t)atomic_load_explicit(&slot->sequence, memory_order_relaxed);
}

void sample_snapshot_init(sample_snapshot_t *snapshot)
{
    for (unsigned index = 0; index < 2; ++index) {
        sample_snapshot_slot_t *slot = &snapshot->slots[index];
        atomic_init(&slot->version, 0);
        atomic_init(&slot->ax, 0);
        atomic_init(&slot->ay, 0);
        atomic_init(&slot->az, 0);
        atomic_init(&slot->gx, 0);
        atomic_init(&slot->gy, 0);
        atomic_init(&slot->gz, 0);
        atomic_init(&slot->mx, 0);
        atomic_init(&slot->my, 0);
        atomic_init(&slot->mz, 0);
        atomic_init(&slot->accel_gyro_valid, false);
        atomic_init(&slot->mag_valid, false);
        atomic_init(&slot->captured_us, 0);
        atomic_init(&slot->sequence, 0);
    }
    atomic_init(&snapshot->published_sequence, 0);
}

void sample_snapshot_publish(sample_snapshot_t *snapshot, const imu_sample_t *sample)
{
    if (sample->sequence == 0) {
        return;
    }

    sample_snapshot_slot_t *slot = &snapshot->slots[sample->sequence & 1U];
    uint_fast32_t stable_version = (uint_fast32_t)sample->sequence << 1U;
    atomic_store_explicit(&slot->version, stable_version | 1U, memory_order_release);
    sample_snapshot_store(slot, sample);
    atomic_store_explicit(&slot->version, stable_version, memory_order_release);
    atomic_store_explicit(&snapshot->published_sequence, sample->sequence, memory_order_release);
}

bool sample_snapshot_read(const sample_snapshot_t *snapshot, imu_sample_t *sample)
{
    for (;;) {
        uint_fast32_t sequence = atomic_load_explicit(&snapshot->published_sequence, memory_order_acquire);
        if (sequence == 0) {
            return false;
        }

        const sample_snapshot_slot_t *slot = &snapshot->slots[sequence & 1U];
        uint_fast32_t expected_version = sequence << 1U;
        uint_fast32_t before = atomic_load_explicit(&slot->version, memory_order_acquire);
        if (before != expected_version) {
            continue;
        }

        sample_snapshot_load(slot, sample);
        uint_fast32_t after = atomic_load_explicit(&slot->version, memory_order_acquire);
        if (before == after && sample->sequence == sequence) {
            return true;
        }
    }
}
