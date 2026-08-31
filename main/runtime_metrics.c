#include "runtime_metrics.h"

#include <limits.h>
#include <stdatomic.h>
#include <string.h>

#ifdef ESP_PLATFORM
/* File-local compile ceiling, NOT the global CONFIG_LOG_MAXIMUM_LEVEL: that
 * Kconfig option is also what managed_components/espressif__esp_video's
 * esp_video_isp_pipeline.c gates its own (commented-out, unrelated)
 * per-frame debug dump behind, via `#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG`
 * with no override -- LOG_LOCAL_LEVEL defaults to CONFIG_LOG_MAXIMUM_LEVEL
 * when a file doesn't set it. Raising that Kconfig option globally silently
 * compiled that unrelated block in too (hw-confirmed: flooded the console
 * with per-sequence ISP histogram/AF dumps, worse than RTM ever was). This
 * define raises the ceiling for ESP_LOGD calls in *this file only*, so RTM
 * can move to ESP_LOGD without touching any other component's behavior. */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drivers/gps_driver.h"
#include "motor_control.h"
#include "pipeline.h"
#include <string.h>
#endif

typedef struct {
    atomic_uint sequence;
    atomic_uint runs;
    atomic_uint max_exec_us;
    atomic_uint max_gap_us;
    atomic_uint max_jitter_us;
    atomic_uint deadline_misses;
    atomic_uint stack_free_words;
    atomic_uint histogram[8];
    atomic_uint events[RUNTIME_EVENT_COUNT];
    uint64_t cycle_scheduled_us;
    uint64_t cycle_started_us;
    uint64_t previous_started_us;
    bool have_previous_start;
#ifdef ESP_PLATFORM
    TaskHandle_t handle;
#endif
} runtime_metric_record_t;

static runtime_metric_record_t s_records[RUNTIME_TASK_COUNT];

static bool valid_id(runtime_task_id_t id)
{
    return id >= RUNTIME_TASK_CONTROL && id < RUNTIME_TASK_COUNT;
}

static uint32_t clamp_delta(uint64_t later, uint64_t earlier)
{
    if (later <= earlier) return 0;
    uint64_t delta = later - earlier;
    return delta > UINT32_MAX ? UINT32_MAX : (uint32_t)delta;
}

static uint32_t histogram_bucket(uint32_t execution_us)
{
    static const uint32_t limits[] = {100, 250, 500, 1000, 2500, 5000, 10000};
    for (uint32_t i = 0; i < 7; ++i) {
        if (execution_us <= limits[i]) return i;
    }
    return 7;
}

static void lock_record(runtime_metric_record_t *record)
{
    unsigned expected;
    do {
        expected = atomic_load_explicit(&record->sequence, memory_order_acquire);
        if (expected & 1U) continue;
    } while (!atomic_compare_exchange_weak_explicit(&record->sequence, &expected,
                                                     expected + 1U,
                                                     memory_order_acquire,
                                                     memory_order_relaxed));
}

static void unlock_record(runtime_metric_record_t *record)
{
    atomic_fetch_add_explicit(&record->sequence, 1U, memory_order_release);
}

static void update_max(atomic_uint *value, uint32_t candidate)
{
    unsigned current = atomic_load_explicit(value, memory_order_relaxed);
    while (candidate > current &&
           !atomic_compare_exchange_weak_explicit(value, &current, candidate,
                                                  memory_order_relaxed,
                                                  memory_order_relaxed)) {
    }
}

void runtime_metrics_init(void)
{
    memset(s_records, 0, sizeof(s_records));
}

void runtime_metrics_reset_for_test(void)
{
    runtime_metrics_init();
}

void runtime_metrics_record_cycle(runtime_task_id_t id, uint64_t scheduled_us,
                                  uint64_t started_us, uint64_t finished_us)
{
    if (!valid_id(id)) return;

    runtime_metric_record_t *record = &s_records[id];
    const runtime_task_spec_t *spec = runtime_schedule_get(id);
    uint32_t execution_us = clamp_delta(finished_us, started_us);
    uint32_t jitter_us = started_us >= scheduled_us
                             ? clamp_delta(started_us, scheduled_us)
                             : clamp_delta(scheduled_us, started_us);

    lock_record(record);
    atomic_fetch_add_explicit(&record->runs, 1U, memory_order_relaxed);
    update_max(&record->max_exec_us, execution_us);
    update_max(&record->max_jitter_us, jitter_us);
    atomic_fetch_add_explicit(&record->histogram[histogram_bucket(execution_us)], 1U,
                              memory_order_relaxed);
    if (record->have_previous_start) {
        update_max(&record->max_gap_us,
                   clamp_delta(started_us, record->previous_started_us));
    }
    record->previous_started_us = started_us;
    record->have_previous_start = true;
    if (spec && spec->deadline_us &&
        finished_us > scheduled_us &&
        finished_us - scheduled_us > spec->deadline_us) {
        atomic_fetch_add_explicit(&record->deadline_misses, 1U, memory_order_relaxed);
    }
    unlock_record(record);
}

void runtime_metrics_cycle_begin(runtime_task_id_t id, uint64_t scheduled_us,
                                 uint64_t started_us)
{
    if (!valid_id(id)) return;
    runtime_metric_record_t *record = &s_records[id];
    lock_record(record);
    record->cycle_scheduled_us = scheduled_us;
    record->cycle_started_us = started_us;
    unlock_record(record);
}

void runtime_metrics_cycle_end(runtime_task_id_t id, uint64_t finished_us)
{
    if (!valid_id(id)) return;
    runtime_metric_record_t *record = &s_records[id];
    uint64_t scheduled_us;
    uint64_t started_us;
    lock_record(record);
    scheduled_us = record->cycle_scheduled_us;
    started_us = record->cycle_started_us;
    unlock_record(record);
    runtime_metrics_record_cycle(id, scheduled_us, started_us, finished_us);
}

void runtime_metrics_count(runtime_task_id_t id, runtime_metric_event_t event)
{
    if (!valid_id(id) || event < 0 || event >= RUNTIME_EVENT_COUNT) return;
    atomic_fetch_add_explicit(&s_records[id].events[event], 1U, memory_order_relaxed);
}

void runtime_metrics_set_stack(runtime_task_id_t id, uint32_t words)
{
    if (!valid_id(id)) return;
    atomic_store_explicit(&s_records[id].stack_free_words, words, memory_order_relaxed);
}

void runtime_metrics_snapshot(runtime_task_id_t id, runtime_metric_snapshot_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!valid_id(id)) return;

    runtime_metric_record_t *record = &s_records[id];
    unsigned before;
    unsigned after;
    do {
        before = atomic_load_explicit(&record->sequence, memory_order_acquire);
        if (before & 1U) continue;
        out->runs = atomic_load_explicit(&record->runs, memory_order_relaxed);
        out->max_exec_us = atomic_load_explicit(&record->max_exec_us, memory_order_relaxed);
        out->max_gap_us = atomic_load_explicit(&record->max_gap_us, memory_order_relaxed);
        out->max_jitter_us = atomic_load_explicit(&record->max_jitter_us, memory_order_relaxed);
        out->deadline_misses = atomic_load_explicit(&record->deadline_misses, memory_order_relaxed);
        out->stack_free_words = atomic_load_explicit(&record->stack_free_words, memory_order_relaxed);
        for (uint32_t i = 0; i < 8; ++i) {
            out->exec_histogram[i] = atomic_load_explicit(&record->histogram[i], memory_order_relaxed);
        }
        for (uint32_t i = 0; i < RUNTIME_EVENT_COUNT; ++i) {
            out->events[i] = atomic_load_explicit(&record->events[i], memory_order_relaxed);
        }
        after = atomic_load_explicit(&record->sequence, memory_order_acquire);
    } while (before != after || (after & 1U));
}

bool runtime_metrics_core_attribution_complete(const uint8_t *affinity_masks,
                                               uint32_t task_count)
{
    if (!affinity_masks || task_count == 0) return false;
    for (uint32_t i = 0; i < task_count; ++i) {
        if (affinity_masks[i] != 1U && affinity_masks[i] != 2U) return false;
    }
    return true;
}

#ifdef ESP_PLATFORM
#define RUNTIME_METRICS_SYSTEM_TASK_CAPACITY 32

void runtime_metrics_register_handle(runtime_task_id_t id, TaskHandle_t handle)
{
    if (!valid_id(id)) return;
    lock_record(&s_records[id]);
    s_records[id].handle = handle;
    unlock_record(&s_records[id]);
}

void task_runtime_diagnostics(void *arg)
{
    (void)arg;
    static const char *tag = "RTM";
    TickType_t next_report = xTaskGetTickCount();
    uint32_t published_motor_generation = UINT32_MAX;
    /* 0 = nothing published yet; the control task never emits generation 0. */
    uint32_t published_calibrate_generation = 0;
    uint32_t published_bench_generation = 0;
    unsigned bench_repeats_left = 0;   /* re-sends of the terminal bench status */
#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1)
    static TaskStatus_t task_status[RUNTIME_METRICS_SYSTEM_TASK_CAPACITY];
    static uint8_t affinity_masks[RUNTIME_METRICS_SYSTEM_TASK_CAPACITY];
#endif
    while (true) {
        TickType_t now = xTaskGetTickCount();
        bool periodic_report = now >= next_report;
        boat_MotorStatus motor_status;
        uint32_t motor_generation = motor_control_get_status(&motor_status);
        if (periodic_report || motor_generation != published_motor_generation) {
            pipeline_publish_motor_status(&motor_status);
            published_motor_generation = motor_generation;
        }

        /* ESC-trim calibration progress: publish only on a fresh update (never
         * periodically -- an idle boat's generation stays put and nothing is
         * sent). generation 0 = never calibrated this boot. */
        boat_CalibrateStatus calibrate_status;
        uint32_t calibrate_generation = motor_control_get_calibrate_status(&calibrate_status);
        if (calibrate_generation != 0 && calibrate_generation != published_calibrate_generation) {
            pipeline_publish_calibrate_status(&calibrate_status);
            published_calibrate_generation = calibrate_generation;
        }

        /* Bench throttle-mismatch run: same fresh-update-only rule. The file on
         * the SD card is the real product; this is just so the operator can see
         * the run progress and which file number it saved as. */
        motor_control_bench_flush();   /* slow SD write, off the control loop */
        motor_control_trimlearn_log();  /* logs; must stay off the control loop */
        boat_BenchStatus bench_status;
        uint32_t bench_generation = motor_control_get_bench_status(&bench_status);
        bool bench_terminal = (bench_status.state == 4u || bench_status.state == 5u);
        if (bench_generation != 0 && bench_generation != published_bench_generation) {
            published_bench_generation = bench_generation;
            /* The terminal state is the one the operator actually needs (it
             * carries the saved file number) and it would otherwise be sent
             * exactly once -- on a lossy link that single packet going missing
             * leaves the tool believing the run is still going. Repeat it. */
            bench_repeats_left = bench_terminal ? 4u : 0u;
            pipeline_publish_bench_status(&bench_status);
        } else if (bench_repeats_left > 0u && periodic_report) {
            --bench_repeats_left;
            pipeline_publish_bench_status(&bench_status);
        }

        if (periodic_report) {
            gps_runtime_status_t gps_status;
            if (gps_driver_get_runtime_status(&gps_status) == ESP_OK) {
                ESP_LOGD(tag, "gps fifo_ovf=%u buffer_full=%u line_ovf=%u parse_errors=%u protocol=%u last_frame_us=%lld fix_age_us=%lld",
                         (unsigned)gps_status.uart_fifo_overflows,
                         (unsigned)gps_status.uart_buffer_full_events,
                         (unsigned)gps_status.parser_line_overflows,
                         (unsigned)gps_status.parse_errors,
                         (unsigned)gps_status.protocol_authority,
                         (long long)gps_status.last_frame_us,
                         (long long)gps_status.fix_age_us);
            }
            for (runtime_task_id_t id = RUNTIME_TASK_CONTROL; id < RUNTIME_TASK_COUNT; ++id) {
                runtime_metric_record_t *record = &s_records[id];
                const runtime_task_spec_t *spec = runtime_schedule_get(id);
                runtime_metric_snapshot_t metrics;
                TaskHandle_t handle;
                lock_record(record);
                handle = record->handle;
                unlock_record(record);
                if (handle) {
                    runtime_metrics_set_stack(id, uxTaskGetStackHighWaterMark(handle));
                }
                runtime_metrics_snapshot(id, &metrics);
                ESP_LOGD(tag, "task=%s core=%d prio=%u runs=%u max_exec_us=%u max_gap_us=%u max_jitter_us=%u misses=%u stack_free_words=%u skips=%u errors=%u",
                         spec->name, spec->core, (unsigned)spec->priority,
                         (unsigned)metrics.runs, (unsigned)metrics.max_exec_us,
                         (unsigned)metrics.max_gap_us, (unsigned)metrics.max_jitter_us,
                         (unsigned)metrics.deadline_misses, (unsigned)metrics.stack_free_words,
                         (unsigned)metrics.events[RUNTIME_EVENT_SENSOR_SKIP],
                         (unsigned)metrics.events[RUNTIME_EVENT_SENSOR_ERROR]);
            }
#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1) && \
    (configUSE_CORE_AFFINITY == 1) && (configNUMBER_OF_CORES > 1)
        configRUN_TIME_COUNTER_TYPE total_runtime = 0;
        UBaseType_t task_count = uxTaskGetSystemState(task_status,
                                                      RUNTIME_METRICS_SYSTEM_TASK_CAPACITY,
                                                      &total_runtime);
        for (UBaseType_t i = 0; i < task_count; ++i) {
            affinity_masks[i] = (uint8_t)task_status[i].uxCoreAffinityMask;
        }
        if (total_runtime > 0 &&
            runtime_metrics_core_attribution_complete(affinity_masks, task_count)) {
            configRUN_TIME_COUNTER_TYPE core_runtime[2] = {0, 0};
            configRUN_TIME_COUNTER_TYPE idle_runtime[2] = {0, 0};
            for (UBaseType_t i = 0; i < task_count; ++i) {
                unsigned core = affinity_masks[i] == 1U ? 0U : 1U;
                core_runtime[core] += task_status[i].ulRunTimeCounter;
                if (strncmp(task_status[i].pcTaskName, "IDLE", 4) == 0) {
                    idle_runtime[core] += task_status[i].ulRunTimeCounter;
                }
            }
            unsigned core0_load = core_runtime[0] == 0 ? 0 :
                (unsigned)(100ULL * (core_runtime[0] - idle_runtime[0]) / core_runtime[0]);
            unsigned core1_load = core_runtime[1] == 0 ? 0 :
                (unsigned)(100ULL * (core_runtime[1] - idle_runtime[1]) / core_runtime[1]);
            ESP_LOGD(tag, "core_load_supported=1 core0_load_pct=%u core1_load_pct=%u",
                     core0_load, core1_load);
        } else {
            ESP_LOGD(tag, "core_load_supported=0");
        }
#else
            ESP_LOGD(tag, "core_load_supported=0");
#endif
            next_report += pdMS_TO_TICKS(1000);
            if (next_report <= now) next_report = now + pdMS_TO_TICKS(1000);
        }

        now = xTaskGetTickCount();
        TickType_t wait = next_report > now ? next_report - now : 0;
        ulTaskNotifyTake(pdTRUE, wait);
    }
}
#endif
