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
#include "common.h"
#include "esp_log.h"
#include "esp_timer.h"
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

    ESP_LOGI(TAG, "ToF reader task started (polling every %d ms)", TOF_POLL_INTERVAL_MS);

    while (true) {
        if (g_inference_active) {
            while (g_inference_active) vTaskDelay(pdMS_TO_TICKS(10));
            last_wake = xTaskGetTickCount();
        }

        /* One sensor per mutex acquisition, so the IMU keeps a read window. */
        if (devs->a_ok) {
            if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                esp_err_t r = tof_read_grid(&devs->dev_a, &res);
                xSemaphoreGive(g_i2c_mutex);
                if (r == ESP_OK) { ok_a++;   tof_cache_store(&s_cache_a, &res, med_a, &med_idx_a); }
                else             { notready_a++; }
            } else {
                mutexfail_a++;
            }
        }

        if (devs->b_ok) {
            if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                esp_err_t r = tof_read_grid(&devs->dev_b, &res);
                xSemaphoreGive(g_i2c_mutex);
                if (r == ESP_OK) { ok_b++;   tof_cache_store(&s_cache_b, &res, med_b, &med_idx_b); }
                else             { notready_b++; }
            } else {
                mutexfail_b++;
            }
        }

        /* Report every ~5 s. Separates the two failure modes: "sensor had no
         * frame ready" (notready) vs "could not get the I2C bus" (mutexfail).
         * ok/s should land near the sensor's ranging rate (~10 Hz). */
        if (++report_div >= (5000 / TOF_POLL_INTERVAL_MS)) {
            ESP_LOGI(TAG,
                     "ToF 5s: A ok=%u notready=%u mutexfail=%u | B ok=%u notready=%u mutexfail=%u",
                     (unsigned)ok_a, (unsigned)notready_a, (unsigned)mutexfail_a,
                     (unsigned)ok_b, (unsigned)notready_b, (unsigned)mutexfail_b);
            ok_a = notready_a = mutexfail_a = 0;
            ok_b = notready_b = mutexfail_b = 0;
            report_div = 0;
        }

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TOF_POLL_INTERVAL_MS));
    }
}

/* Copy a cached grid into the snapshot. Returns false when nothing fresh. */
static bool tof_cache_load(const tof_grid_cache_t *cache, boat_ToFGrid *out)
{
    bool ok = false;
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    if (cache->ts_us != 0 && (esp_timer_get_time() - cache->ts_us) < TOF_STALE_US) {
        out->valid = true;
        out->distances_count = TOF_NVALS;
        memcpy(out->distances, cache->distances, sizeof(cache->distances));
        /* Field mode omits the diagnostics: proto3 encodes an empty repeated
         * field as zero bytes, halving the snapshot for the ESP-NOW link. */
        if (!g_field_mode) {
            out->sigma_count = TOF_NVALS;
            out->target_status_count = TOF_NVALS;
            out->nb_target_detected_count = 64;
            memcpy(out->sigma, cache->sigma, sizeof(cache->sigma));
            memcpy(out->target_status, cache->status, sizeof(cache->status));
            memcpy(out->nb_target_detected, cache->nb_target, sizeof(cache->nb_target));
        }
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

    while (true) {
        /* Yield while inference is running — avoid DMA/PSRAM contention */
        if (g_inference_active) {
            while (g_inference_active) vTaskDelay(pdMS_TO_TICKS(10));
            /* Re-baseline after the pause, or xTaskDelayUntil would fire with
             * no delay repeatedly trying to "catch up" the inference stall. */
            last_wake = xTaskGetTickCount();
        }

        snap = (boat_SensorSnapshot)boat_SensorSnapshot_init_zero;
        snap.timestamp_us = (uint64_t)esp_timer_get_time();

        /* IMU — always available, 20 Hz */
        FusionResult imu;
        fusion_get_result(&imu);
        snap.has_imu     = true;
        snap.imu.pitch   = imu.pitch;
        snap.imu.roll    = imu.roll;
        snap.imu.heading = imu.heading;

        /* ToF — copied from the reader task's cache (5 Hz publish).
         * No I2C here any more: the cache is filled by task_tof_reader at the
         * sensor's own rate, so a snapshot no longer misses ToF just because
         * the sensor had nothing ready at this exact instant. */
        bool tof_tick = (iteration % TOF_EVERY_N == 0);

        if (tof_tick) {
            if (devs->a_ok) {
                snap.has_tof_a = tof_cache_load(&s_cache_a, &snap.tof_a);
            }
            if (devs->b_ok) {
                snap.has_tof_b = tof_cache_load(&s_cache_b, &snap.tof_b);
            }
        }

        /* Re-emit cached detections for up to 30s so the browser gets them
         * after WS reconnect — was 15s but post-detect WS reconnect can drag
         * 15-20s on slow TCP timeouts, dropping the cached result on the floor. */
        detect_get_cached_results(snap.detections, &snap.detections_count, 30000);

        gps_fix_t gps;
        if (gps_driver_get_fix(&gps) == ESP_OK && gps.last_update_us != 0) {
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

        /* SystemStatus at ~1Hz */
        if (++iteration % STATUS_EVERY_N == 0) {
            boat_SystemStatus sys = boat_SystemStatus_init_zero;
            sys.heap_free = (uint32_t)esp_get_free_heap_size();
            sys.uptime_us = (uint64_t)esp_timer_get_time();

            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                sys.wifi_rssi = ap.rssi;
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
        if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SNAPSHOT_INTERVAL_MS)) == pdFALSE) {
            if ((++overruns % 50) == 1) {
                ESP_LOGW(TAG, "snapshot loop overrun (#%u): work exceeded %d ms",
                         (unsigned)overruns, SNAPSHOT_INTERVAL_MS);
            }
        }
    }
}
