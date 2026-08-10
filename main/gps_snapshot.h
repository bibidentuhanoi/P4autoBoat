#pragma once

#include <stdbool.h>
#include <stdatomic.h>

#include "drivers/gps_driver.h"

typedef struct {
    gps_fix_t fix;
    gps_runtime_status_t runtime;
} gps_snapshot_entry_t;

#define GPS_SNAPSHOT_WORD_COUNT \
    ((sizeof(gps_snapshot_entry_t) + sizeof(unsigned) - 1U) / sizeof(unsigned))

typedef struct {
    atomic_uint version;
    atomic_uint words[GPS_SNAPSHOT_WORD_COUNT];
} gps_snapshot_slot_t;

typedef struct {
    gps_snapshot_slot_t slots[2];
    atomic_uint published_generation;
} gps_snapshot_t;

void gps_snapshot_init(gps_snapshot_t *snapshot);
void gps_snapshot_publish(gps_snapshot_t *snapshot,
                          const gps_snapshot_entry_t *entry);
bool gps_snapshot_read(const gps_snapshot_t *snapshot,
                       gps_snapshot_entry_t *entry);
