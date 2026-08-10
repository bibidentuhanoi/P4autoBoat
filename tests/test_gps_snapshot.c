#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

#include "gps_snapshot.h"

static gps_snapshot_entry_t make_entry(uint32_t generation)
{
    return (gps_snapshot_entry_t){
        .fix = {
            .valid = true,
            .latitude = (double)generation,
            .longitude = (double)generation + 0.25,
            .speed_mps = (float)generation,
            .satellites = (uint8_t)(generation & 0x0fU),
            .last_update_us = (int64_t)generation * 1000,
        },
        .runtime = {
            .uart_fifo_overflows = generation,
            .uart_buffer_full_events = generation + 1,
            .parser_line_overflows = generation + 2,
            .parse_errors = generation + 3,
            .protocol_authority = GPS_PROTOCOL_UBX,
            .last_frame_us = (int64_t)generation * 1000 + 10,
        },
    };
}

typedef struct {
    gps_snapshot_t *snapshot;
    atomic_bool done;
    atomic_bool torn;
} stress_state_t;

static void *publish_entries(void *arg)
{
    stress_state_t *state = arg;
    for (uint32_t generation = 1; generation <= 100000; ++generation) {
        gps_snapshot_entry_t entry = make_entry(generation);
        gps_snapshot_publish(state->snapshot, &entry);
    }
    atomic_store(&state->done, true);
    return NULL;
}

static void *read_entries(void *arg)
{
    stress_state_t *state = arg;
    gps_snapshot_entry_t entry;
    do {
        if (!gps_snapshot_read(state->snapshot, &entry)) continue;
        uint32_t generation = entry.runtime.uart_fifo_overflows;
        if (entry.fix.latitude != (double)generation ||
            entry.fix.longitude != (double)generation + 0.25 ||
            entry.fix.speed_mps != (float)generation ||
            entry.fix.last_update_us != (int64_t)generation * 1000 ||
            entry.runtime.uart_buffer_full_events != generation + 1 ||
            entry.runtime.parse_errors != generation + 3 ||
            entry.runtime.last_frame_us != (int64_t)generation * 1000 + 10) {
            atomic_store(&state->torn, true);
            break;
        }
    } while (!atomic_load(&state->done));
    return NULL;
}

int main(void)
{
    gps_snapshot_t snapshot;
    gps_snapshot_init(&snapshot);

    gps_snapshot_entry_t out;
    assert(!gps_snapshot_read(&snapshot, &out));

    gps_snapshot_entry_t first = make_entry(1);
    gps_snapshot_entry_t second = make_entry(2);
    gps_snapshot_publish(&snapshot, &first);
    gps_snapshot_publish(&snapshot, &second);
    assert(gps_snapshot_read(&snapshot, &out));
    assert(out.runtime.uart_fifo_overflows == 2);
    assert(out.fix.latitude == 2.0);

    stress_state_t state = {
        .snapshot = &snapshot,
        .done = ATOMIC_VAR_INIT(false),
        .torn = ATOMIC_VAR_INIT(false),
    };
    pthread_t writer;
    pthread_t reader;
    assert(pthread_create(&writer, NULL, publish_entries, &state) == 0);
    assert(pthread_create(&reader, NULL, read_entries, &state) == 0);
    assert(pthread_join(writer, NULL) == 0);
    assert(pthread_join(reader, NULL) == 0);
    assert(!atomic_load(&state.torn));
    return 0;
}
