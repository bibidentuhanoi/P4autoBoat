#include "gps_snapshot.h"

#include <stdint.h>
#include <string.h>

_Static_assert(sizeof(unsigned) == 4, "GPS snapshot atomics require 32-bit words");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "GPS snapshot atomics must be lock-free");

void gps_snapshot_init(gps_snapshot_t *snapshot)
{
    for (unsigned slot_index = 0; slot_index < 2; ++slot_index) {
        gps_snapshot_slot_t *slot = &snapshot->slots[slot_index];
        atomic_init(&slot->version, 0);
        for (size_t word = 0; word < GPS_SNAPSHOT_WORD_COUNT; ++word) {
            atomic_init(&slot->words[word], 0);
        }
    }
    atomic_init(&snapshot->published_generation, 0);
}

void gps_snapshot_publish(gps_snapshot_t *snapshot,
                          const gps_snapshot_entry_t *entry)
{
    unsigned generation = atomic_load_explicit(&snapshot->published_generation,
                                                memory_order_relaxed) + 1U;
    if (generation == 0) generation = 1;

    gps_snapshot_slot_t *slot = &snapshot->slots[generation & 1U];
    unsigned stable_version = generation << 1U;
    unsigned words[GPS_SNAPSHOT_WORD_COUNT];
    memset(words, 0, sizeof(words));
    memcpy(words, entry, sizeof(*entry));

    atomic_exchange_explicit(&slot->version, stable_version | 1U,
                             memory_order_acq_rel);
    for (size_t word = 0; word < GPS_SNAPSHOT_WORD_COUNT; ++word) {
        atomic_store_explicit(&slot->words[word], words[word],
                              memory_order_relaxed);
    }
    atomic_store_explicit(&slot->version, stable_version, memory_order_release);
    atomic_store_explicit(&snapshot->published_generation, generation,
                          memory_order_release);
}

bool gps_snapshot_read(const gps_snapshot_t *snapshot,
                       gps_snapshot_entry_t *entry)
{
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
        unsigned generation = atomic_load_explicit(&snapshot->published_generation,
                                                   memory_order_acquire);
        if (generation == 0) return false;

        const gps_snapshot_slot_t *slot = &snapshot->slots[generation & 1U];
        unsigned stable_version = generation << 1U;
        unsigned before = atomic_load_explicit(&slot->version,
                                               memory_order_acquire);
        if (before != stable_version) continue;

        unsigned words[GPS_SNAPSHOT_WORD_COUNT];
        for (size_t word = 0; word < GPS_SNAPSHOT_WORD_COUNT; ++word) {
            words[word] = atomic_load_explicit(&slot->words[word],
                                               memory_order_relaxed);
        }
        atomic_thread_fence(memory_order_seq_cst);
        unsigned after = atomic_load_explicit(&slot->version,
                                              memory_order_acquire);
        if (before == after) {
            memcpy(entry, words, sizeof(*entry));
            return true;
        }
    }
    return false;
}
