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
    return 0;
}
