#include "sample_snapshot.h"

static void sample_snapshot_store(sample_snapshot_slot_t *slot, const imu_sample_t *sample)
{
    atomic_store_explicit(&slot->ax, (uint16_t)sample->ax, memory_order_relaxed);
    atomic_store_explicit(&slot->ay, (uint16_t)sample->ay, memory_order_relaxed);
    atomic_store_explicit(&slot->az, (uint16_t)sample->az, memory_order_relaxed);
    atomic_store_explicit(&slot->gx, (uint16_t)sample->gx, memory_order_relaxed);
    atomic_store_explicit(&slot->gy, (uint16_t)sample->gy, memory_order_relaxed);
    atomic_store_explicit(&slot->gz, (uint16_t)sample->gz, memory_order_relaxed);
    atomic_store_explicit(&slot->mx, (uint16_t)sample->mx, memory_order_relaxed);
    atomic_store_explicit(&slot->my, (uint16_t)sample->my, memory_order_relaxed);
    atomic_store_explicit(&slot->mz, (uint16_t)sample->mz, memory_order_relaxed);
    atomic_store_explicit(&slot->validity,
                          (sample->accel_gyro_valid ? 1U : 0U) |
                          (sample->mag_valid ? 2U : 0U), memory_order_relaxed);
    atomic_store_explicit(&slot->captured_us_low, (uint32_t)sample->captured_us, memory_order_relaxed);
    atomic_store_explicit(&slot->captured_us_high, (uint32_t)(sample->captured_us >> 32U), memory_order_relaxed);
    atomic_store_explicit(&slot->sequence, sample->sequence, memory_order_relaxed);
}

static void sample_snapshot_load(const sample_snapshot_slot_t *slot, imu_sample_t *sample)
{
    sample->ax = (int16_t)(uint16_t)atomic_load_explicit(&slot->ax, memory_order_relaxed);
    sample->ay = (int16_t)(uint16_t)atomic_load_explicit(&slot->ay, memory_order_relaxed);
    sample->az = (int16_t)(uint16_t)atomic_load_explicit(&slot->az, memory_order_relaxed);
    sample->gx = (int16_t)(uint16_t)atomic_load_explicit(&slot->gx, memory_order_relaxed);
    sample->gy = (int16_t)(uint16_t)atomic_load_explicit(&slot->gy, memory_order_relaxed);
    sample->gz = (int16_t)(uint16_t)atomic_load_explicit(&slot->gz, memory_order_relaxed);
    sample->mx = (int16_t)(uint16_t)atomic_load_explicit(&slot->mx, memory_order_relaxed);
    sample->my = (int16_t)(uint16_t)atomic_load_explicit(&slot->my, memory_order_relaxed);
    sample->mz = (int16_t)(uint16_t)atomic_load_explicit(&slot->mz, memory_order_relaxed);
    unsigned validity = atomic_load_explicit(&slot->validity, memory_order_relaxed);
    sample->accel_gyro_valid = (validity & 1U) != 0;
    sample->mag_valid = (validity & 2U) != 0;
    uint32_t captured_us_low = atomic_load_explicit(&slot->captured_us_low, memory_order_relaxed);
    uint32_t captured_us_high = atomic_load_explicit(&slot->captured_us_high, memory_order_relaxed);
    sample->captured_us = ((uint64_t)captured_us_high << 32U) | captured_us_low;
    sample->sequence = atomic_load_explicit(&slot->sequence, memory_order_relaxed);
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
        atomic_init(&slot->validity, 0);
        atomic_init(&slot->captured_us_low, 0);
        atomic_init(&slot->captured_us_high, 0);
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
    unsigned stable_version = sample->sequence << 1U;
    /* The acq_rel RMW is the writer-side begin barrier: payload writes cannot
     * become visible before readers can observe the odd in-progress marker. */
    atomic_exchange_explicit(&slot->version, stable_version | 1U, memory_order_acq_rel);
    sample_snapshot_store(slot, sample);
    atomic_store_explicit(&slot->version, stable_version, memory_order_release);
    atomic_store_explicit(&snapshot->published_sequence, sample->sequence, memory_order_release);
}

bool sample_snapshot_read(const sample_snapshot_t *snapshot, imu_sample_t *sample)
{
    for (;;) {
        unsigned sequence = atomic_load_explicit(&snapshot->published_sequence, memory_order_acquire);
        if (sequence == 0) {
            return false;
        }

        const sample_snapshot_slot_t *slot = &snapshot->slots[sequence & 1U];
        unsigned expected_version = sequence << 1U;
        unsigned before = atomic_load_explicit(&slot->version, memory_order_acquire);
        if (before != expected_version) {
            continue;
        }

        sample_snapshot_load(slot, sample);
        /* Full reader-side barrier: all payload loads must complete before the
         * final version validation, even on weakly ordered RISC-V cores. */
        atomic_thread_fence(memory_order_seq_cst);
        unsigned after = atomic_load_explicit(&slot->version, memory_order_acquire);
        if (before == after && sample->sequence == sequence) {
            return true;
        }
    }
}
