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
#include "sensor_fusion.h"
#include "drivers/gps_driver.h"
#include "drivers/imu_driver.h"
#include "transports/espnow_transport.h"
#include "common.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "runtime_metrics.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l5cx_api.h"
#include <string.h>

static const char *TAG = "SENSOR_TASK";

static inline int16_t median3(int16_t a, int16_t b, int16_t c) {
    return (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                   : ((a > c) ? a : ((b > c) ? c : b));
}

#define SNAPSHOT_INTERVAL_MS  50   /* 20 Hz sensor publish */
#define TOF_EVERY_N           4    /* ToF at every 4th tick = 5 Hz (keeps WS traffic low) */
#define STATUS_EVERY_N        20   /* SystemStatus every 20th iteration (~1Hz) */

/* ─── ToF reader task ────────────────────────────────────────────────────────
 * ToF used to be read inline by the snapshot loop, but tof_read_grid() is a
 * single non-blocking poll: if the sensor has no fresh frame at that instant it
 * returns ESP_FAIL and the snapshot ships with NO ToF at all. The sensor ranges
 * on its own ~10 Hz clock while the snapshot loop free-runs, so the two rates
 * beat against each other and most polls missed — measured ~2 of 5.5 expected
 * ToF frames/s reaching the ground station.
 *
 * Now a dedicated task polls at the sensor's own rate and caches the PROCESSED
 * grid; the snapshot just copies the cache. Benefits:
 *   - reads happen when data actually is ready, so they succeed
 *   - the median filter runs per SENSOR frame, not per publish (feeding it the
 *     same cached frame repeatedly would make median3 a no-op)
 *   - the snapshot loop does no I2C at all, so it stops overrunning its 50 ms
 *     period and no longer contends with the IMU task for g_i2c_mutex
 * ───────────────────────────────────────────────────────────────────────────*/
#define TOF_POLL_INTERVAL_MS  25   /* 40 Hz poll of a ~10 Hz sensor: always catches
                                    * a fresh frame without busy-waiting */
#define TOF_STALE_US   (1000 * 1000)  /* cached grid older than this = not valid */

#define TOF_NVALS  (64 * VL53L5CX_NB_TARGET_PER_ZONE)

typedef struct {
    int16_t  distances[TOF_NVALS];
    uint16_t sigma[TOF_NVALS];
    uint8_t  status[TOF_NVALS];
    uint8_t  nb_target[64];
    int64_t  ts_us;          /* 0 = never populated */
} tof_grid_cache_t;

static tof_grid_cache_t  s_cache_a, s_cache_b;
static SemaphoreHandle_t s_cache_mutex = NULL;

/* Apply per-slot gating + 3-frame median, then publish into the cache. */
static void tof_cache_store(tof_grid_cache_t *cache,
                            const VL53L5CX_ResultsData *res,
                            int16_t med[][3], uint8_t *med_idx)
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
    return ESP_OK;
}

void task_tof_reader(void *pvParameters)
{
    tof_devices_t *devs = (tof_devices_t *)pvParameters;

    static VL53L5CX_ResultsData res;
    static int16_t med_a[TOF_NVALS][3];
    static int16_t med_b[TOF_NVALS][3];
    static uint8_t med_idx_a = 0, med_idx_b = 0;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t ok_a = 0, notready_a = 0, mutexfail_a = 0;
    uint32_t ok_b = 0, notready_b = 0, mutexfail_b = 0;
    uint32_t report_div = 0;

    /* Diagnostic: how long tof_read_grid() actually holds g_i2c_mutex, split
     * by outcome. task_imu_fusion is failing to get this same mutex ~96% of
     * the time (its own "fusion 5s: mutex_fails=" log) despite ToF never
     * reporting a mutexfail itself -- this measures the side of that
     * asymmetry that was never actually timed, just described as "slow" in
     * a comment. The ok/notready split matters for deciding whether an
     * interrupt-driven redesign (INT pins are wired -- ToF-A GPIO50,
     * ToF-B GPIO1) is worth building: if notready time dominates, it would
     * remove most of this task's I2C usage (no more blind polling of a
     * sensor with nothing ready, most polls). If ok time dominates, the real
     * grid reads are the cost and still have to happen regardless of what
     * triggers them -- interrupts would only help by cutting the poll count,
     * not the per-read cost. */
    int64_t held_us_a_ok = 0, held_us_a_notready = 0;
    int64_t held_us_b_ok = 0, held_us_b_notready = 0;
    uint32_t max_read_us_a = 0, max_read_us_b = 0;
    uint64_t metric_scheduled = esp_timer_get_time();

    ESP_LOGI(TAG, "ToF reader task started (polling every %d ms)", TOF_POLL_INTERVAL_MS);

    while (true) {
        if (g_inference_active) {
            while (g_inference_active) vTaskDelay(pdMS_TO_TICKS(10));
            last_wake = xTaskGetTickCount();
            metric_scheduled = esp_timer_get_time();
        }
        uint64_t metric_started = esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_SENSOR_BUS, metric_scheduled, metric_started);

        /* One sensor per mutex acquisition, so the IMU keeps a read window. */
        if (devs->a_ok) {
            if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                int64_t t0 = esp_timer_get_time();
                esp_err_t r = tof_read_grid(&devs->dev_a, &res);
                uint32_t held = (uint32_t)(esp_timer_get_time() - t0);
                xSemaphoreGive(g_i2c_mutex);
                if (held > max_read_us_a) max_read_us_a = held;
                if (r == ESP_OK) { ok_a++;   held_us_a_ok += held; tof_cache_store(&s_cache_a, &res, med_a, &med_idx_a); }
                else             { notready_a++; held_us_a_notready += held; runtime_metrics_count(RUNTIME_TASK_SENSOR_BUS, RUNTIME_EVENT_SENSOR_ERROR); }
            } else {
                mutexfail_a++;
                runtime_metrics_count(RUNTIME_TASK_SENSOR_BUS, RUNTIME_EVENT_SENSOR_SKIP);
            }
        }

        if (devs->b_ok) {
            if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                int64_t t0 = esp_timer_get_time();
                esp_err_t r = tof_read_grid(&devs->dev_b, &res);
                uint32_t held = (uint32_t)(esp_timer_get_time() - t0);
                xSemaphoreGive(g_i2c_mutex);
                if (held > max_read_us_b) max_read_us_b = held;
                if (r == ESP_OK) { ok_b++;   held_us_b_ok += held; tof_cache_store(&s_cache_b, &res, med_b, &med_idx_b); }
                else             { notready_b++; held_us_b_notready += held; runtime_metrics_count(RUNTIME_TASK_SENSOR_BUS, RUNTIME_EVENT_SENSOR_ERROR); }
            } else {
                mutexfail_b++;
                runtime_metrics_count(RUNTIME_TASK_SENSOR_BUS, RUNTIME_EVENT_SENSOR_SKIP);
            }
        }

        /* Report every ~5 s. Separates the two failure modes: "sensor had no
         * frame ready" (notready) vs "could not get the I2C bus" (mutexfail).
         * ok/s should land near the sensor's ranging rate (~10 Hz). */
        if (++report_div >= (5000 / TOF_POLL_INTERVAL_MS)) {
            ESP_LOGI(TAG,
                     "ToF 5s: A ok=%u(%lldms) notready=%u(%lldms) mutexfail=%u max=%ums | "
                     "B ok=%u(%lldms) notready=%u(%lldms) mutexfail=%u max=%ums",
                     (unsigned)ok_a, (long long)(held_us_a_ok / 1000),
                     (unsigned)notready_a, (long long)(held_us_a_notready / 1000),
                     (unsigned)mutexfail_a, (unsigned)(max_read_us_a / 1000),
                     (unsigned)ok_b, (long long)(held_us_b_ok / 1000),
                     (unsigned)notready_b, (long long)(held_us_b_notready / 1000),
                     (unsigned)mutexfail_b, (unsigned)(max_read_us_b / 1000));
            ok_a = notready_a = mutexfail_a = 0;
            ok_b = notready_b = mutexfail_b = 0;
            held_us_a_ok = held_us_a_notready = 0;
            held_us_b_ok = held_us_b_notready = 0;
            max_read_us_a = max_read_us_b = 0;
            report_div = 0;
        }

        runtime_metrics_cycle_end(RUNTIME_TASK_SENSOR_BUS, esp_timer_get_time());
        metric_scheduled += (uint64_t)TOF_POLL_INTERVAL_MS * 1000U;
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TOF_POLL_INTERVAL_MS));
    }
}

/* Copy a cached grid into the snapshot. Returns false when nothing fresh.
 * Only ever called from the non-field-mode branch of task_sensor_snapshot
 * below -- field mode sends espnow_telemetry_t instead and never reaches
 * this function at all, so there is no g_field_mode check needed here. */
static bool tof_cache_load(const tof_grid_cache_t *cache, boat_ToFGrid *out)
{
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
        ok = true;
    }
    xSemaphoreGive(s_cache_mutex);
    return ok;
}

void task_sensor_snapshot(void *pvParameters)
{
    tof_devices_t *devs = (tof_devices_t *)pvParameters;

    /* Static allocation — avoids stack overflow risk.
     * ToF buffers and the median filter now live in task_tof_reader. */
    static boat_SensorSnapshot snap;

    uint32_t iteration = 0;

    ESP_LOGI(TAG, "Sensor snapshot task started (%d Hz, ToF %d Hz)",
             1000 / SNAPSHOT_INTERVAL_MS,
             1000 / SNAPSHOT_INTERVAL_MS / TOF_EVERY_N);

    /* Fixed PERIOD, not fixed delay. vTaskDelay() after the work made the loop
     * run at (work + 50 ms): the ToF ticks take ~25 ms of I2C (two grid reads,
     * contending with the 50 Hz IMU task for g_i2c_mutex), so the real rate was
     * ~13 Hz rather than the advertised 20 Hz. That was the whole "missing ToF
     * frames" mystery — the radio was delivering everything it was given. */
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t overruns = 0;
    uint64_t metric_scheduled = esp_timer_get_time();

    while (true) {
        /* Yield while inference is running — avoid DMA/PSRAM contention */
        if (g_inference_active) {
            while (g_inference_active) vTaskDelay(pdMS_TO_TICKS(10));
            /* Re-baseline after the pause, or xTaskDelayUntil would fire with
             * no delay repeatedly trying to "catch up" the inference stall. */
            last_wake = xTaskGetTickCount();
            metric_scheduled = esp_timer_get_time();
        }
        uint64_t metric_started = esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_SNAPSHOT, metric_scheduled, metric_started);

        /* IMU — always available, 20 Hz. GPS — whatever the driver currently
         * has cached. Both modes need these; ToF/detections/full-snapshot
         * assembly below is bench-mode only. */
        FusionResult imu;
        fusion_get_result(&imu);

        gps_fix_t gps;
        bool have_gps = (gps_driver_get_fix(&gps) == ESP_OK && gps.last_update_us != 0);

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

            /* ToF — copied from the reader task's cache (5 Hz publish).
             * No I2C here any more: the cache is filled by task_tof_reader at
             * the sensor's own rate, so a snapshot no longer misses ToF just
             * because the sensor had nothing ready at this exact instant. */
            bool tof_tick = (iteration % TOF_EVERY_N == 0);
            if (tof_tick) {
                if (devs->a_ok) {
                    snap.has_tof_a = tof_cache_load(&s_cache_a, &snap.tof_a);
                }
                if (devs->b_ok) {
                    snap.has_tof_b = tof_cache_load(&s_cache_b, &snap.tof_b);
                }
            }

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

            pipeline_publish_sensors(&snap);
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
            if (!g_field_mode) {
                wifi_ap_record_t ap;
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                    sys.wifi_rssi = ap.rssi;
                }
            }

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
                ESP_LOGW(TAG, "snapshot loop overrun (#%u): work exceeded %d ms",
                         (unsigned)overruns, SNAPSHOT_INTERVAL_MS);
            }
        }
    }
}
