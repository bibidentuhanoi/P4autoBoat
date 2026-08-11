/* ═══════════════════════════════════════════════════════════════
 * CALIBRATION CHECKLIST — When changing housing/lens:
 * ───────────────────────────────────────────────────────────────
 * 1. Mount on flat wall (distance 1–3 m)
 * 2. Run IMU calibration (GPIO 35 at boot)
 * 3. Check 3D Threshold overlay alignment (dashboard)
 * 4. Adjust FX (focal length) if FOV is off
 * 5. Adjust CX/CY (principal point) for center offset
 * 6. Fine-tune AZ/EL offsets via dashboard sliders
 * 7. Verify grid scale GRID_SCALE ≈ 1.0
 * 8. Test drag offset (should be near 0,0)
 * ═══════════════════════════════════════════════════════════════ */

#include "sensor_task.h"
#include "detect_task.h"
#include "pipeline.h"
#include "training_log_task.h"
#include "sensor_fusion.h"
#include "drivers/gps_driver.h"
#include "drivers/imu_driver.h"
#include "transports/espnow_transport.h"
#include "common.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "runtime_metrics.h"
#include "sensor_schedule.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l5cx_api.h"
#include <stdatomic.h>
#include <string.h>

static const char *TAG = "SENSOR_TASK";

static inline int16_t median3(int16_t a, int16_t b, int16_t c) {
    return (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                   : ((a > c) ? a : ((b > c) ? c : b));
}

#define SNAPSHOT_INTERVAL_MS  50   /* 20 Hz sensor publish */
#define TOF_EVERY_N           2    /* ToF at every 2nd tick = 10 Hz, matching
                                     * SENSOR_TOF_PERIOD_US in sensor_schedule.c
                                     * -- no point publishing faster than the
                                     * cache actually refreshes, or slower and
                                     * sitting on fresher data than we send. */
#define STATUS_EVERY_N        20   /* SystemStatus every 20th iteration (~1Hz) */

/* ─── ToF acquisition and processing ────────────────────────────────────────
 * ToF used to be read inline by the snapshot loop, but tof_read_grid() is a
 * single non-blocking poll: if the sensor has no fresh frame at that instant it
 * returns ESP_FAIL and the snapshot ships with NO ToF at all. The sensor ranges
 * on its own ~10 Hz clock while the snapshot loop free-runs, so the two rates
 * beat against each other and most polls missed — measured ~2 of 5.5 expected
 * ToF frames/s reaching the ground station.
 *
 * SensorBusTask used to own all runtime I2C (IMU + ToF) on one 20ms tick, but
 * a single VL53L5CX read genuinely takes ~33ms (confirmed on hardware
 * 2026-08-11 -- transferring the enabled result fields at NB_TARGET_PER_ZONE=4
 * over I2C, not a fault) -- longer than SensorBus's own period, so
 * interleaving it there guaranteed a deadline miss on every cycle ToF was
 * selected (~38% sustained SensorBus miss rate). Split into task_tof_read(),
 * its own lower-priority task (see runtime_schedule.c) -- IMU keeps its tight
 * undisturbed 20ms cadence, ToF gets the ~100ms-per-sensor room it actually
 * needs. Both still publish/consume through the same lock-free result
 * channels to ToFProc on Core 1; that part is unchanged. Benefits:
 *   - reads happen when data actually is ready, so they succeed
 *   - the median filter runs per SENSOR frame, not per publish (feeding it the
 *     same cached frame repeatedly would make median3 a no-op)
 *   - the snapshot loop does no I2C at all
 *   - IMU acquisition is no longer delayed by ToF's much longer read time
 * ───────────────────────────────────────────────────────────────────────────*/
#define SENSOR_BUS_INTERVAL_MS 20
/* Log any single IMU I2C op slower than this -- normal accel/gyro/mag reads
 * are ~0.6-2ms at 100kHz, so 5ms is well above jitter but well below the
 * 20ms cycle budget. Now that ToF no longer shares this task, any op
 * crossing this threshold is a genuine anomaly, not routine ToF timing. */
#define SENSOR_BUS_SLOW_OP_US  5000U
/* task_tof_read()'s own poll tick -- fine enough to hit each sensor's
 * ~100ms-per-sensor due time (50ms A/B offset, see SENSOR_TOF_PERIOD_US /
 * SENSOR_TOF_B_OFFSET_US in sensor_schedule.c) precisely, without the tight
 * coupling to IMU timing SensorBus used to need. A no-op tick (nothing due)
 * costs microseconds, so ticking this often is cheap. */
#define TOF_READ_POLL_MS       20
/* A single ToF read taking ~33ms is now expected, not diagnostic-worthy --
 * logging every one would just be the same noise, differently attributed.
 * This threshold is for genuine anomalies: something well beyond the normal
 * ~33ms (e.g. a real bus fault), worth a look. */
#define TOF_READ_ANOMALY_US    50000U
#define TOF_STALE_US   (1000 * 1000)  /* cached grid older than this = not valid */

#define TOF_NVALS  (64 * VL53L5CX_NB_TARGET_PER_ZONE)

typedef struct {
    int16_t  distances[TOF_NVALS];
    uint16_t sigma[TOF_NVALS];
    uint8_t  status[TOF_NVALS];
    uint8_t  nb_target[64];
    uint32_t generation;     /* verified (torn-frame-excluded) generation number */
    int64_t  ts_us;          /* 0 = never populated */
} tof_grid_cache_t;

typedef struct {
    VL53L5CX_ResultsData buffers[2];
    atomic_uint slot_state[2];
    atomic_uint generation;
} tof_result_channel_t;

enum {
    TOF_SLOT_FREE,
    TOF_SLOT_WRITING,
    TOF_SLOT_READING,
};

_Static_assert(sizeof(unsigned int) == 4 && ATOMIC_INT_LOCK_FREE == 2,
               "ToF cross-core slot claims require lock-free 32-bit atomics");

static tof_grid_cache_t  s_cache_a, s_cache_b;
static SemaphoreHandle_t s_cache_mutex = NULL;
static sample_snapshot_t s_imu_samples;
static tof_result_channel_t s_tof_results_a, s_tof_results_b;
static _Atomic(TaskHandle_t) s_fusion_task;
static _Atomic(TaskHandle_t) s_tof_processor_task;
static uint32_t s_imu_sequence = 0;

/* Apply per-slot gating + 3-frame median, then publish into the cache. */
static void tof_cache_store(tof_grid_cache_t *cache,
                            const VL53L5CX_ResultsData *res,
                            int16_t med[][3], uint8_t *med_idx,
                            uint32_t generation)
{
    int16_t  dist[TOF_NVALS];
    uint16_t sig[TOF_NVALS];
    uint8_t  st[TOF_NVALS];
    uint8_t  nbt[64];

    for (int i = 0; i < 64; i++) {
        uint8_t nb = res->nb_target_detected[i];
        for (int j = 0; j < VL53L5CX_NB_TARGET_PER_ZONE; j++) {
            int idx = i * VL53L5CX_NB_TARGET_PER_ZONE + j;
            uint8_t status = res->target_status[idx];
            /* Slots beyond nb_target_detected hold stale driver buffer data. */
            bool slot_valid = (j < nb) && (status == 5 || status == 9);
            med[idx][*med_idx] = slot_valid ? res->distance_mm[idx] : 0;
            dist[idx] = median3(med[idx][0], med[idx][1], med[idx][2]);
            sig[idx]  = slot_valid ? res->range_sigma_mm[idx] : 0;
            st[idx]   = slot_valid ? status : 0;
        }
        nbt[i] = nb;
    }
    *med_idx = (*med_idx + 1) % 3;

    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    memcpy(cache->distances, dist, sizeof(dist));
    memcpy(cache->sigma,     sig,  sizeof(sig));
    memcpy(cache->status,    st,   sizeof(st));
    memcpy(cache->nb_target, nbt,  sizeof(nbt));
    cache->generation = generation;
    cache->ts_us = esp_timer_get_time();
    xSemaphoreGive(s_cache_mutex);
}

esp_err_t sensor_task_init(void)
{
    if (!s_cache_mutex) {
        s_cache_mutex = xSemaphoreCreateMutex();
        if (!s_cache_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }
    sample_snapshot_init(&s_imu_samples);
    return ESP_OK;
}

sample_snapshot_t *sensor_imu_sample_snapshot(void)
{
    return &s_imu_samples;
}

void sensor_task_register_fusion_task(TaskHandle_t task)
{
    atomic_store_explicit(&s_fusion_task, task, memory_order_release);
}

bool sensor_read_imu_sample(imu_sample_t *sample)
{
    static uint32_t ag_fails;
    static uint32_t mag_fails;
    static bool bus_scanned;
    static int16_t last_mx, last_my, last_mz;
    static uint32_t mag_static, mag_rearm, mag_diag;
    static bool mag_frozen;

    if (!sample) {
        return false;
    }

    *sample = (imu_sample_t){0};

    int64_t ag_t0 = esp_timer_get_time();
    esp_err_t ag_err = imu_read_accel_gyro(&sample->ax, &sample->ay, &sample->az,
                                            &sample->gx, &sample->gy, &sample->gz);
    int64_t ag_read_us = esp_timer_get_time() - ag_t0;

    int64_t mag_t0 = esp_timer_get_time();
    esp_err_t mag_err = imu_read_mag(&sample->mx, &sample->my, &sample->mz);
    int64_t mag_read_us = esp_timer_get_time() - mag_t0;

    if (ag_read_us > SENSOR_BUS_SLOW_OP_US || mag_read_us > SENSOR_BUS_SLOW_OP_US) {
        /* One combined line, not two separate ones: shows both durations
         * together so a single cycle where accel/gyro AND mag each collide
         * with a ToF chunk (stacking toward a much larger total) is visible
         * directly, instead of having to correlate two disjoint log lines. */
        ESP_LOGW(TAG, "SensorBus: IMU read ag=%lldus(%s) mag=%lldus(%s) total=%lldus",
                 (long long)ag_read_us, esp_err_to_name(ag_err),
                 (long long)mag_read_us, esp_err_to_name(mag_err),
                 (long long)(ag_read_us + mag_read_us));
    }

    sample->accel_gyro_valid = (ag_err == ESP_OK);
    sample->mag_valid = (mag_err == ESP_OK);
    sample->captured_us = (uint64_t)esp_timer_get_time();

    if (!sample->mag_valid) {
        ++mag_fails;
        if (mag_fails == 50) imu_set_mag_ok(false);
        if (mag_fails == 50 && !bus_scanned) {
            bus_scanned = true;
            imu_bus_scan();
        }
        if ((mag_fails % 100) == 1) {
            ESP_LOGW(TAG, "QMC5883L mag read failing (%lu fails, %s) — heading frozen",
                     (unsigned long)mag_fails, esp_err_to_name(mag_err));
        }
        if ((mag_fails % 100) == 0) {
            int64_t rec_t0 = esp_timer_get_time();
            esp_err_t rec_err = imu_recover_mag();
            int64_t rec_us = esp_timer_get_time() - rec_t0;
            if (rec_us > SENSOR_BUS_SLOW_OP_US) {
                ESP_LOGW(TAG, "SensorBus: imu_recover_mag() took %lldus (%s)",
                         (long long)rec_us, esp_err_to_name(rec_err));
            }
            if (rec_err == ESP_OK) {
                ESP_LOGI(TAG, "mag recovered after %lu fails — re-added + reconfigured",
                         (unsigned long)mag_fails);
                imu_set_mag_ok(true);
                mag_fails = 0;
            }
        }
    } else {
        if (sample->mx == last_mx && sample->my == last_my && sample->mz == last_mz) {
            if (++mag_static >= 50) {
                mag_static = 0;
                if (!mag_frozen) {
                    imu_set_mag_ok(false);
                    mag_frozen = true;
                }
                if (++mag_rearm >= 3) {
                    imu_reinit_mag();
                    mag_rearm = 0;
                    ESP_LOGW(TAG, "mag still frozen after re-arm — full reset");
                } else {
                    imu_mag_ensure_continuous();
                    ESP_LOGW(TAG, "mag data frozen — re-asserted continuous mode");
                }
            }
        } else {
            if (mag_frozen) {
                imu_set_mag_ok(true);
                mag_frozen = false;
            }
            mag_static = 0;
            mag_rearm = 0;
            last_mx = sample->mx;
            last_my = sample->my;
            last_mz = sample->mz;
        }
        if (++mag_diag >= 1500) {
            uint8_t mag_ctrl = 0xFF;
            mag_diag = 0;
            imu_mag_ensure_continuous();
            imu_mag_read_ctrl(&mag_ctrl);
            ESP_LOGI(TAG, "MAG DIAG raw=(%d,%d,%d) ctrl09=0x%02X",
                     sample->mx, sample->my, sample->mz, mag_ctrl);
        }
        if (mag_fails) {
            if (mag_fails >= 50) {
                imu_reinit_mag();
                ESP_LOGI(TAG, "mag recovered after %lu fails — reconfigured",
                         (unsigned long)mag_fails);
                imu_set_mag_ok(true);
            }
            mag_fails = 0;
        }
    }

    if (!sample->accel_gyro_valid) {
        ++ag_fails;
        if (ag_fails == 50) imu_set_icm_ok(false);
        if (ag_fails == 50 && !bus_scanned) {
            bus_scanned = true;
            imu_bus_scan();
        }
        if ((ag_fails % 100) == 1) {
            ESP_LOGW(TAG, "ICM20948 accel/gyro read failing (%lu fails, %s) — pitch/roll frozen",
                     (unsigned long)ag_fails, esp_err_to_name(ag_err));
        }
        if ((ag_fails % 100) == 0) {
            int64_t rec_t0 = esp_timer_get_time();
            esp_err_t rec_err = imu_recover_accel_gyro();
            int64_t rec_us = esp_timer_get_time() - rec_t0;
            if (rec_us > SENSOR_BUS_SLOW_OP_US) {
                ESP_LOGW(TAG, "SensorBus: imu_recover_accel_gyro() took %lldus (%s)",
                         (long long)rec_us, esp_err_to_name(rec_err));
            }
            if (rec_err == ESP_OK) {
                ESP_LOGI(TAG, "accel/gyro recovered after %lu fails — reprobed + reconfigured",
                         (unsigned long)ag_fails);
                imu_set_icm_ok(true);
                ag_fails = 0;
            }
        }
    } else if (ag_fails) {
        if (ag_fails >= 50) {
            imu_reinit_accel_gyro();
            ESP_LOGI(TAG, "accel/gyro recovered after %lu fails — reconfigured",
                     (unsigned long)ag_fails);
            imu_set_icm_ok(true);
        }
        ag_fails = 0;
    }

    if (!sample->accel_gyro_valid && !sample->mag_valid) {
        return false;
    }
    if (++s_imu_sequence == 0) ++s_imu_sequence;
    sample->sequence = s_imu_sequence;
    return true;
}

static sensor_tof_id_t tof_oldest_due(const sensor_schedule_t *schedule,
                                      uint64_t now_us,
                                      bool tof_a_available,
                                      bool tof_b_available)
{
    bool a_due = tof_a_available && schedule->next_due_a_us <= now_us;
    bool b_due = tof_b_available && schedule->next_due_b_us <= now_us;
    if (!a_due && !b_due) return SENSOR_TOF_NONE;
    if (!b_due || (a_due && schedule->next_due_a_us <= schedule->next_due_b_us)) {
        return SENSOR_TOF_A;
    }
    return SENSOR_TOF_B;
}

static uint32_t tof_next_generation(const tof_result_channel_t *channel)
{
    uint32_t next = atomic_load_explicit(&channel->generation, memory_order_relaxed) + 1U;
    return next == 0 ? 1U : next;
}

static bool tof_result_copy(tof_result_channel_t *channel,
                            uint32_t last_generation,
                            VL53L5CX_ResultsData *result,
                            uint32_t *generation)
{
    for (;;) {
        uint32_t published = atomic_load_explicit(&channel->generation, memory_order_acquire);
        if (published == 0 || published == last_generation) return false;
        uint32_t slot = published & 1U;
        uint32_t expected = TOF_SLOT_FREE;
        if (!atomic_compare_exchange_strong_explicit(
                &channel->slot_state[slot], &expected, TOF_SLOT_READING,
                memory_order_acq_rel, memory_order_acquire)) {
            continue;
        }

        uint32_t stable = atomic_load_explicit(&channel->generation, memory_order_acquire);
        if (stable != published) {
            atomic_store_explicit(&channel->slot_state[slot], TOF_SLOT_FREE,
                                  memory_order_release);
            continue;
        }

        memcpy(result, &channel->buffers[slot], sizeof(*result));
        atomic_store_explicit(&channel->slot_state[slot], TOF_SLOT_FREE,
                              memory_order_release);
        *generation = published;
        return true;
    }
}

static void tof_count_overwrites(uint32_t last_generation, uint32_t generation)
{
    uint32_t missing = generation - last_generation;
    while (missing > 1U) {
        runtime_metrics_count(RUNTIME_TASK_TOF_PROCESS, RUNTIME_EVENT_SENSOR_SKIP);
        --missing;
    }
}

static void tof_process_latest(tof_result_channel_t *channel,
                               tof_grid_cache_t *cache,
                               int16_t median[][3],
                               uint8_t *median_index,
                               uint32_t *last_generation,
                               VL53L5CX_ResultsData *result)
{
    uint32_t generation;
    while (tof_result_copy(channel, *last_generation, result, &generation)) {
        tof_count_overwrites(*last_generation, generation);
        uint64_t started = esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_TOF_PROCESS, started, started);
        tof_cache_store(cache, result, median, median_index, generation);
        runtime_metrics_cycle_end(RUNTIME_TASK_TOF_PROCESS, esp_timer_get_time());
        *last_generation = generation;
    }
}

void task_sensor_bus(void *pvParameters)
{
    (void)pvParameters;  /* ToF moved to task_tof_read(); this task is IMU-only now. */

    TickType_t last_wake = xTaskGetTickCount();
    uint64_t metric_scheduled = (uint64_t)esp_timer_get_time();

    ESP_LOGI(TAG, "SensorBus task started (IMU-only, %d Hz)",
             1000 / SENSOR_BUS_INTERVAL_MS);

    for (;;) {
        uint64_t metric_started = (uint64_t)esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_SENSOR_BUS,
                                    metric_scheduled, metric_started);

        imu_sample_t raw_imu;
        if (sensor_read_imu_sample(&raw_imu)) {
            sample_snapshot_publish(&s_imu_samples, &raw_imu);
            TaskHandle_t fusion_task =
                atomic_load_explicit(&s_fusion_task, memory_order_acquire);
            if (fusion_task) xTaskNotifyGive(fusion_task);
        } else {
            runtime_metrics_count(RUNTIME_TASK_SENSOR_BUS, RUNTIME_EVENT_SENSOR_ERROR);
        }

        runtime_metrics_cycle_end(RUNTIME_TASK_SENSOR_BUS, esp_timer_get_time());
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_BUS_INTERVAL_MS));
        metric_scheduled += (uint64_t)SENSOR_BUS_INTERVAL_MS * 1000U;
    }
}

/* Independent ToF acquisition task (2026-08-11 SensorBus/ToF split -- see the
 * file header comment above for why). Runs at its own pace, decoupled from
 * SensorBus's IMU timing entirely. Lower priority than SensorBus/Fusion (see
 * runtime_schedule.c) so IMU always wins any I2C scheduling contention; the
 * bus itself is still shared and serialized by ESP-IDF's own per-transaction
 * lock, not an application mutex around the whole ~33ms read -- wrapping the
 * whole read would just relocate the same stall onto whichever task loses
 * that lock instead of removing it. Publishes through the same lock-free
 * result channels ToFProc already consumes; that hand-off is unchanged. */
void task_tof_read(void *pvParameters)
{
    static uint32_t tof_torn_frames;

    tof_devices_t *devs = (tof_devices_t *)pvParameters;
    sensor_schedule_t schedule;
    uint64_t started_us = (uint64_t)esp_timer_get_time();
    sensor_schedule_init(&schedule, started_us);

    TickType_t last_wake = xTaskGetTickCount();
    uint64_t metric_scheduled = started_us;

    ESP_LOGI(TAG, "ToFRead task started (polling every %d ms, alternating ToF ~10 Hz each)",
             TOF_READ_POLL_MS);

    for (;;) {
        uint64_t metric_started = (uint64_t)esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_TOF_READ,
                                    metric_scheduled, metric_started);

        uint64_t now_us = (uint64_t)esp_timer_get_time();
        sensor_tof_id_t due = tof_oldest_due(&schedule, now_us,
                                             devs->a_ok, devs->b_ok);
        /* No IMU deadline to protect anymore -- ToF has its own task now.
         * UINT64_MAX/0 disables sensor_schedule_choose_tof()'s
         * budget-protection branch (built for the old SensorBus-shared-tick
         * design) without touching that function's tested logic, so this
         * always returns whatever's actually due by pure due-time. */
        sensor_tof_id_t selected = sensor_schedule_choose_tof(
            &schedule, now_us, UINT64_MAX, 0, devs->a_ok, devs->b_ok);

        if (selected == SENSOR_TOF_NONE) {
            if (due != SENSOR_TOF_NONE) {
                sensor_schedule_note_skip(&schedule, due);
                runtime_metrics_count(RUNTIME_TASK_TOF_READ, RUNTIME_EVENT_SENSOR_SKIP);
            }
        } else {
            tof_result_channel_t *channel =
                selected == SENSOR_TOF_A ? &s_tof_results_a : &s_tof_results_b;
            VL53L5CX_Configuration *device =
                selected == SENSOR_TOF_A ? &devs->dev_a : &devs->dev_b;
            uint32_t generation = tof_next_generation(channel);
            uint32_t slot = generation & 1U;
            uint32_t expected = TOF_SLOT_FREE;
            if (!atomic_compare_exchange_strong_explicit(
                    &channel->slot_state[slot], &expected, TOF_SLOT_WRITING,
                    memory_order_acq_rel, memory_order_acquire)) {
                sensor_schedule_note_skip(&schedule, selected);
                runtime_metrics_count(RUNTIME_TASK_TOF_READ,
                                      RUNTIME_EVENT_SENSOR_SKIP);
            } else {
                const char *tof_label = selected == SENSOR_TOF_A ? "A" : "B";
                int64_t read_started = esp_timer_get_time();
                esp_err_t result =
                    tof_read_grid(device, &channel->buffers[slot], tof_label);
                uint32_t read_us = (uint32_t)(esp_timer_get_time() - read_started);
                if (read_us > TOF_READ_ANOMALY_US) {
                    ESP_LOGW(TAG, "ToFRead: ToF-%s read took %luus (%s) -- well beyond the normal ~33ms",
                             tof_label, (unsigned long)read_us, esp_err_to_name(result));
                }
                if (result == ESP_OK) {
                    atomic_store_explicit(&channel->generation, generation,
                                          memory_order_release);
                }
                atomic_store_explicit(&channel->slot_state[slot], TOF_SLOT_FREE,
                                      memory_order_release);

                sensor_schedule_note_tof_result(&schedule, selected, now_us,
                                                result == ESP_OK);
                if (result == ESP_OK) {
                    TaskHandle_t processor =
                        atomic_load_explicit(&s_tof_processor_task,
                                             memory_order_acquire);
                    if (processor) xTaskNotifyGive(processor);
                } else if (result == ESP_ERR_NOT_FINISHED) {
                    /* Schedule polled before this sensor's ranging engine had
                     * a new frame ready -- expected occasionally, not a
                     * fault. Next ~20ms poll tick will normally catch it. */
                    sensor_schedule_note_skip(&schedule, selected);
                    runtime_metrics_count(RUNTIME_TASK_TOF_READ,
                                          RUNTIME_EVENT_SENSOR_SKIP);
                } else if (result == ESP_ERR_INVALID_CRC) {
                    /* Possible chunk-boundary tear -- tof_read_grid() already
                     * logged the streamcount/device detail. Do not publish:
                     * channel->generation is only stored in the ESP_OK branch
                     * above, so this frame structurally cannot reach ToFProc,
                     * the dashboard, or dataset storage. Counted as a skip,
                     * not an I2C error, since nothing on the bus failed --
                     * the next poll acquires a clean frame instead. */
                    ++tof_torn_frames;
                    ESP_LOGW(TAG, "ToFRead: ToF-%s frame rejected (torn), total=%lu",
                             tof_label, (unsigned long)tof_torn_frames);
                    sensor_schedule_note_skip(&schedule, selected);
                    runtime_metrics_count(RUNTIME_TASK_TOF_READ,
                                          RUNTIME_EVENT_SENSOR_SKIP);
                } else {
                    ESP_LOGW(TAG, "ToFRead: ToF-%s read failed: %s",
                             tof_label, esp_err_to_name(result));
                    runtime_metrics_count(RUNTIME_TASK_TOF_READ,
                                          RUNTIME_EVENT_SENSOR_ERROR);
                }
            }
        }

        runtime_metrics_cycle_end(RUNTIME_TASK_TOF_READ, esp_timer_get_time());
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TOF_READ_POLL_MS));
        metric_scheduled += (uint64_t)TOF_READ_POLL_MS * 1000U;
    }
}

void task_tof_processor(void *pvParameters)
{
    (void)pvParameters;
    static VL53L5CX_ResultsData result;
    static int16_t med_a[TOF_NVALS][3];
    static int16_t med_b[TOF_NVALS][3];
    uint8_t med_idx_a = 0;
    uint8_t med_idx_b = 0;
    uint32_t generation_a = 0;
    uint32_t generation_b = 0;
    uint64_t last_gen_log_us = 0;

    atomic_store_explicit(&s_tof_processor_task, xTaskGetCurrentTaskHandle(),
                          memory_order_release);
    ESP_LOGI(TAG, "ToF processor task started");

    for (;;) {
        tof_process_latest(&s_tof_results_a, &s_cache_a, med_a, &med_idx_a,
                           &generation_a, &result);
        tof_process_latest(&s_tof_results_b, &s_cache_b, med_b, &med_idx_b,
                           &generation_b, &result);

        /* Throttled per-sensor generation counters -- ToFProc's own RTM
         * "runs=" is aggregate across both channels, so a stalled ToF-A with
         * a healthy ToF-B would still show runs= climbing. This is the only
         * per-sensor liveness signal. */
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        if (now_us - last_gen_log_us >= 1000000ULL) {
            ESP_LOGI(TAG, "ToFProc: gen_a=%lu gen_b=%lu",
                     (unsigned long)generation_a, (unsigned long)generation_b);
            last_gen_log_us = now_us;
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

/* Copy a cached grid out. Returns false when nothing fresh has been
 * published yet or the cache has exceeded TOF_STALE_US. generation/age_us
 * are optional (NULL-able, matching camera_capture_frame()'s convention) --
 * task_sensor_snapshot's WS telemetry path doesn't need them, TrainingLog's
 * dataset sidecar does, for honest per-field freshness disclosure. age_us
 * measures time since this generation was stored into the cache, not since
 * the sensor produced it -- ToFProc publishes promptly after ToFRead, so
 * that gap is small, but it is what's actually being measured. */
bool sensor_tof_cache_load(sensor_tof_id_t which, boat_ToFGrid *out,
                           uint32_t *generation, int64_t *age_us)
{
    const tof_grid_cache_t *cache = (which == SENSOR_TOF_A) ? &s_cache_a
                                    : (which == SENSOR_TOF_B) ? &s_cache_b
                                    : NULL;
    if (!cache || !out) return false;

    bool ok = false;
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    if (cache->ts_us != 0 && (esp_timer_get_time() - cache->ts_us) < TOF_STALE_US) {
        out->valid = true;
        out->distances_count = TOF_NVALS;
        out->sigma_count = TOF_NVALS;
        out->target_status_count = TOF_NVALS;
        out->nb_target_detected_count = 64;
        memcpy(out->distances, cache->distances, sizeof(cache->distances));
        memcpy(out->sigma, cache->sigma, sizeof(cache->sigma));
        memcpy(out->target_status, cache->status, sizeof(cache->status));
        memcpy(out->nb_target_detected, cache->nb_target, sizeof(cache->nb_target));
        if (generation) *generation = cache->generation;
        if (age_us) *age_us = esp_timer_get_time() - cache->ts_us;
        ok = true;
    }
    xSemaphoreGive(s_cache_mutex);
    return ok;
}

void task_sensor_snapshot(void *pvParameters)
{
    tof_devices_t *devs = (tof_devices_t *)pvParameters;

    /* Static allocation avoids stack overflow risk. */
    static boat_SensorSnapshot snap;

    uint32_t iteration = 0;

    ESP_LOGI(TAG, "Sensor snapshot task started (%d Hz, ToF %d Hz)",
             1000 / SNAPSHOT_INTERVAL_MS,
             1000 / SNAPSHOT_INTERVAL_MS / TOF_EVERY_N);

    /* Fixed period, not fixed delay. This task only assembles cached data. */
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t overruns = 0;
    uint64_t metric_scheduled = esp_timer_get_time();

    while (true) {
        uint64_t metric_started = esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_SNAPSHOT, metric_scheduled, metric_started);

        /* Per-phase timing, cheap to capture unconditionally (esp_timer_get_time()
         * is microsecond-scale overhead) but only logged on an actual overrun --
         * see the xTaskDelayUntil() check at the bottom of the loop. Chasing a
         * real, hw-observed correlation between TrainingLog SD writes and this
         * task overrunning; esp_wifi_sta_get_ap_info() below is the prime
         * suspect (RPC over the same SDIO transport SD writes contend for),
         * but this covers every phase so the log proves it either way instead
         * of assuming it. */
        uint64_t t_tof_us = 0, t_publish_us = 0, t_wifi_status_us = 0;
        bool sd_active_at_tof = false;
        bool sd_active_at_publish = false, sd_active_at_wifi_status = false;

        /* IMU — always available, 20 Hz. GPS — whatever the driver currently
         * has cached. Both modes need these; ToF/detections/full-snapshot
         * assembly below is bench-mode only. */
        uint64_t t_sensors_start = esp_timer_get_time();
        FusionResult imu;
        fusion_get_result(&imu);

        gps_fix_t gps;
        bool have_gps = (gps_driver_get_fix(&gps) == ESP_OK && gps.last_update_us != 0);
        uint64_t t_sensors_us = esp_timer_get_time() - t_sensors_start;

        if (g_field_mode) {
            /* ESP-NOW: compact struct, IMU+GPS only -- see espnow_telemetry_t
             * in espnow_protocol.h for why (no ToF, no detections, no
             * boat.proto at all on this path). WiFi/WS mode never reaches
             * this branch, so its full-fidelity snapshot is untouched. */
            espnow_telemetry_t tel = {
                .pitch   = imu.pitch,
                .roll    = imu.roll,
                .heading = imu.heading,
            };
            if (have_gps) {
                tel.gps_valid  = gps.valid;
                tel.latitude   = gps.latitude;
                tel.longitude  = gps.longitude;
                tel.speed_mps  = gps.speed_mps;
                tel.course_deg = gps.course_deg;
                tel.satellites = (uint8_t)gps.satellites;
                tel.hdop       = gps.hdop;
            }
            espnow_transport_send_telemetry(&tel);
        } else {
            /* WiFi/WS bench mode: full snapshot, exactly as always. */
            snap = (boat_SensorSnapshot)boat_SensorSnapshot_init_zero;
            snap.timestamp_us = (uint64_t)esp_timer_get_time();
            snap.has_imu     = true;
            snap.imu.pitch   = imu.pitch;
            snap.imu.roll    = imu.roll;
            snap.imu.heading = imu.heading;

            /* ToF — copied from ToFProc's processed cache (5 Hz publish). */
            uint64_t t_tof_start = esp_timer_get_time();
            bool tof_tick = (iteration % TOF_EVERY_N == 0);
            if (tof_tick) {
                if (devs->a_ok) {
                    snap.has_tof_a = sensor_tof_cache_load(SENSOR_TOF_A, &snap.tof_a, NULL, NULL);
                }
                if (devs->b_ok) {
                    snap.has_tof_b = sensor_tof_cache_load(SENSOR_TOF_B, &snap.tof_b, NULL, NULL);
                }
            }
            sd_active_at_tof = g_training_log_sd_active;
            t_tof_us = esp_timer_get_time() - t_tof_start;

            /* Re-emit cached detections for up to 30s so the browser gets
             * them after WS reconnect — was 15s but post-detect WS reconnect
             * can drag 15-20s on slow TCP timeouts, dropping the cached
             * result on the floor. */
            detect_get_cached_results(snap.detections, &snap.detections_count, 30000);

            if (have_gps) {
                snap.has_gps         = true;
                snap.gps.valid       = gps.valid;
                snap.gps.latitude    = gps.latitude;
                snap.gps.longitude   = gps.longitude;
                snap.gps.altitude_m  = gps.altitude_m;
                snap.gps.speed_mps   = gps.speed_mps;
                snap.gps.course_deg  = gps.course_deg;
                snap.gps.fix_quality = gps.fix_quality;
                snap.gps.satellites  = gps.satellites;
                snap.gps.hdop        = gps.hdop;
                snap.gps.utc_ms      = gps.utc_ms;
            }

            uint64_t t_pub_start = esp_timer_get_time();
            pipeline_publish_sensors(&snap);
            sd_active_at_publish = g_training_log_sd_active;
            t_publish_us = esp_timer_get_time() - t_pub_start;
        }

        /* SystemStatus at ~1Hz */
        if (++iteration % STATUS_EVERY_N == 0) {
            boat_SystemStatus sys = boat_SystemStatus_init_zero;
            sys.heap_free = (uint32_t)esp_get_free_heap_size();
            sys.uptime_us = (uint64_t)esp_timer_get_time();

            /* Field mode never joins a WiFi AP (ESP-NOW only) -- querying AP
             * info while connected to none goes through esp_wifi_remote's own
             * RPC path to the C6 (separate from PEER_MSG_VIDEO, uninstrumented
             * here) for a value that's always "not connected" anyway, once a
             * second, forever. Correlates with an rpc_rsp "resp code 12303"
             * log at the exact same 1Hz cadence and lines up with the one
             * "snapshot loop overrun" seen after the ToF/struct fixes landed.
             * espnow_drive.py doesn't even decode MSG_STATUS right now, so
             * this was costing RPC time for a field nothing reads. */
            uint64_t t_wifi_start = esp_timer_get_time();
            if (!g_field_mode) {
                wifi_ap_record_t ap;
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                    sys.wifi_rssi = ap.rssi;
                }
            }
            sd_active_at_wifi_status = g_training_log_sd_active;
            t_wifi_status_us = esp_timer_get_time() - t_wifi_start;

            sys.camera_ok = g_camera_ok;
            sys.tof_a_ok  = devs->a_ok;
            sys.tof_b_ok  = devs->b_ok;
            sys.imu_ok    = imu_icm_ok();
            sys.mag_ok    = imu_mag_ok();
            sys.gps_ok    = gps_driver_is_alive();
            sys.gps_detected_baud = gps_driver_get_detected_baud(&sys.gps_baud_confirmed);

            pipeline_publish_status(&sys);
        }

        /* Sleep until the NEXT period boundary, so loop time is 50 ms total
         * rather than 50 ms on top of the work. Returns pdFALSE when the
         * deadline had already passed, i.e. the work itself overran — surfaced
         * here so a slow loop is visible instead of silently halving the rate
         * the way the old vTaskDelay() did. */
        runtime_metrics_cycle_end(RUNTIME_TASK_SNAPSHOT, esp_timer_get_time());
        metric_scheduled += (uint64_t)SNAPSHOT_INTERVAL_MS * 1000U;
        if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SNAPSHOT_INTERVAL_MS)) == pdFALSE) {
            if ((++overruns % 50) == 1) {
                ESP_LOGW(TAG, "snapshot loop overrun (#%u): work exceeded %d ms "
                              "(sensors=%lldus tof=%lldus(sd_active=%d) publish=%lldus(sd_active=%d) "
                              "wifi_status=%lldus(sd_active=%d))",
                         (unsigned)overruns, SNAPSHOT_INTERVAL_MS,
                         (long long)t_sensors_us,
                         (long long)t_tof_us, (int)sd_active_at_tof,
                         (long long)t_publish_us, (int)sd_active_at_publish,
                         (long long)t_wifi_status_us, (int)sd_active_at_wifi_status);
            }
        }
    }
}
