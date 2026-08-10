#include <assert.h>

#include "runtime_metrics.h"

int main(void)
{
    runtime_metrics_reset_for_test();
    runtime_metrics_record_cycle(RUNTIME_TASK_CONTROL, 0, 100, 600);
    runtime_metrics_record_cycle(RUNTIME_TASK_CONTROL, 10000, 10500, 11200);
    runtime_metrics_record_cycle(RUNTIME_TASK_CONTROL, 20000, 45000, 46000);

    runtime_metric_snapshot_t m;
    runtime_metrics_snapshot(RUNTIME_TASK_CONTROL, &m);

    assert(m.runs == 3);
    assert(m.max_exec_us == 1000);
    assert(m.max_jitter_us == 25000);
    assert(m.deadline_misses == 1);
    assert(m.max_gap_us == 34500);

    runtime_metrics_reset_for_test();
    runtime_metrics_cycle_begin(RUNTIME_TASK_CONTROL, 10000, 10500);
    runtime_metrics_cycle_end(RUNTIME_TASK_CONTROL, 11200);
    runtime_metrics_snapshot(RUNTIME_TASK_CONTROL, &m);
    assert(m.runs == 1);
    assert(m.max_exec_us == 700);
    assert(m.max_jitter_us == 500);
    assert(m.deadline_misses == 0);

    const uint8_t pinned_cores[] = {1, 2, 1};
    const uint8_t includes_unpinned[] = {1, 3};
    const uint8_t includes_unknown[] = {1, 0};
    assert(runtime_metrics_core_attribution_complete(pinned_cores, 3));
    assert(!runtime_metrics_core_attribution_complete(includes_unpinned, 2));
    assert(!runtime_metrics_core_attribution_complete(includes_unknown, 2));
    return 0;
}
