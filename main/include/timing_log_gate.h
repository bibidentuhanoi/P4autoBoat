#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Serial logging can take milliseconds. Emit the first timing warning, then
 * aggregate repeats so a sick peripheral cannot create its own deadline
 * problem while we are diagnosing it. */
#define TIMING_LOG_GATE_INTERVAL_US 1000000LL

typedef struct {
    int64_t last_emit_us;
    uint32_t pending_count;
    uint32_t pending_max_us;
} timing_log_gate_t;

static inline bool timing_log_gate_record(timing_log_gate_t *gate,
                                          int64_t now_us,
                                          uint32_t duration_us,
                                          uint32_t *count,
                                          uint32_t *maximum_us)
{
    ++gate->pending_count;
    if (duration_us > gate->pending_max_us) {
        gate->pending_max_us = duration_us;
    }

    bool first = gate->last_emit_us == 0;
    bool interval_elapsed = now_us < gate->last_emit_us ||
        now_us - gate->last_emit_us >= TIMING_LOG_GATE_INTERVAL_US;
    if (!first && !interval_elapsed) {
        return false;
    }

    *count = gate->pending_count;
    *maximum_us = gate->pending_max_us;
    gate->pending_count = 0;
    gate->pending_max_us = 0;
    gate->last_emit_us = now_us;
    return true;
}
