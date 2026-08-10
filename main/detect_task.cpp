extern "C" {
#include "detect_task.h"
#include "drivers/camera_driver.h"
#include "camera_stream.h"
#include "pipeline.h"
#include "proto/boat.pb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "runtime_metrics.h"
#include "runtime_startup.h"
#include "runtime_task.h"
#include <string.h>
}

/* Cache the most recent detection result so sensor_task can re-emit it
 * after WS reconnect. Without this, detections sent during the WiFi
 * teardown window are lost to all clients. */
#define DETECT_CACHE_MAX 10
static SemaphoreHandle_t s_cache_mutex = NULL;
static boat_Detection   s_cache[DETECT_CACHE_MAX];
static pb_size_t        s_cache_count = 0;
static int64_t          s_cache_ts_us = 0;

#define LOG_HEAP() ESP_LOGI(TAG, "  heap: internal=%u PSRAM=%u", \
    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), \
    (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM))

#include "cat_detect.hpp"
#include "dl_image_define.hpp"
#include "dl_image_jpeg.hpp"

volatile bool g_inference_active = false;
static const char *TAG = "DETECT";

static TaskHandle_t s_detect_task = NULL;
static CatDetect *s_detector = NULL;

static void detect_task_fn(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t metric_started = esp_timer_get_time();
        runtime_metrics_cycle_begin(RUNTIME_TASK_DETECT, metric_started, metric_started);

        int64_t t0 = esp_timer_get_time();

        ESP_LOGI(TAG, "=== Step 0: trigger received ===");
        LOG_HEAP();

        /* 1. Capture JPEG frame */
        ESP_LOGI(TAG, "=== Step 2: capturing frame ===");
        void *jpeg_buf = NULL;
        size_t jpeg_len = 0;
        uint32_t width = 0, height = 0, pix_fmt = 0;
        esp_err_t cap_ret = ESP_FAIL;

        for (int attempt = 0; attempt < 10; attempt++) {
            cap_ret = camera_capture_frame(&jpeg_buf, &jpeg_len, &width, &height, &pix_fmt);
            if (cap_ret == ESP_OK) break;
            ESP_LOGW(TAG, "  capture attempt %d failed", attempt);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (cap_ret != ESP_OK) {
            ESP_LOGE(TAG, "Frame capture failed after retries");
            runtime_metrics_count(RUNTIME_TASK_DETECT, RUNTIME_EVENT_SENSOR_ERROR);
            runtime_metrics_cycle_end(RUNTIME_TASK_DETECT, esp_timer_get_time());
            continue;
        }

        ESP_LOGI(TAG, "=== Step 2: captured %" PRIu32 "x%" PRIu32 " JPEG (%u bytes) ===",
                 width, height, (unsigned)jpeg_len);
        LOG_HEAP();

        /* 2. Decode JPEG → RGB888 */
        ESP_LOGI(TAG, "=== Step 3: decoding JPEG ===");
        dl::image::jpeg_img_t jpeg_img = {
            .data = jpeg_buf,
            .data_len = jpeg_len,
        };
        auto img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
        camera_release_frame();

        if (!img.data) {
            ESP_LOGE(TAG, "JPEG decode returned NULL");
            runtime_metrics_count(RUNTIME_TASK_DETECT, RUNTIME_EVENT_SENSOR_ERROR);
            runtime_metrics_cycle_end(RUNTIME_TASK_DETECT, esp_timer_get_time());
            continue;
        }

        ESP_LOGI(TAG, "=== Step 3: decoded %dx%d RGB888 ===", img.width, img.height);

        /* 3a. Rotate 180° — camera is physically inverted (PPA removed).
         *     Reverse pixel order in-place: swap pixel[i] with pixel[N-1-i]. */
        {
            uint8_t *px = (uint8_t *)img.data;
            int npix = img.width * img.height;
            for (int i = 0; i < npix / 2; i++) {
                int j = npix - 1 - i;
                uint8_t r = px[i*3]; uint8_t g = px[i*3+1]; uint8_t b = px[i*3+2];
                px[i*3] = px[j*3]; px[i*3+1] = px[j*3+1]; px[i*3+2] = px[j*3+2];
                px[j*3] = r; px[j*3+1] = g; px[j*3+2] = b;
            }
            ESP_LOGI(TAG, "=== Step 3a: rotated 180 ===");
        }
        LOG_HEAP();

        /* 3b. Stop the world — freeze all DMA-touching tasks + WiFi.
         *    SDIO/ISP/I2C DMA corrupts PSRAM heap metadata (TLSF free list)
         *    when esp-dl allocates scratch buffers concurrently. */
        ESP_LOGI(TAG, "=== Step 4: freezing tasks + disconnecting WiFi ===");
        g_inference_active = true;
        esp_wifi_disconnect();  /* stops SDIO data DMA but keeps TCP stack alive */
        vTaskDelay(pdMS_TO_TICKS(500)); /* let SDIO DMA drain */

        ESP_LOGI(TAG, "=== Step 4: running inference ===");
        LOG_HEAP();
        int64_t infer_start = esp_timer_get_time();
        auto &results = s_detector->run(img);
        int64_t infer_ms = (esp_timer_get_time() - infer_start) / 1000;
        ESP_LOGI(TAG, "=== Step 4: inference done in %lld ms ===", infer_ms);

        heap_caps_free(img.data);

        /* Resume — wifi_manager's STA_DISCONNECTED handler owns reconnect.
         * Calling esp_wifi_connect() here races the handler and produces
         * ESP_ERR_WIFI_NOT_CONNECT (RPC code 12303) → ~60s stall. */
        ESP_LOGI(TAG, "=== Step 5: resuming tasks, wifi_manager will reconnect ===");
        g_inference_active = false;

        ESP_LOGI(TAG, "Inference done in %lld ms: %d detections",
                 infer_ms, (int)results.size());

        /* 4. Package results into protobuf */
        static boat_SensorSnapshot snap;
        memset(&snap, 0, sizeof(snap));
        snap.timestamp_us = (uint64_t)esp_timer_get_time();
        snap.detections_count = 0;

        for (const auto &r : results) {
            if (snap.detections_count >= 10) break;
            boat_Detection *d = &snap.detections[snap.detections_count];
            d->category = r.category;
            d->score    = r.score;
            d->x1       = r.box[0];
            d->y1       = r.box[1];
            d->x2       = r.box[2];
            d->y2       = r.box[3];
            snap.detections_count++;
            ESP_LOGI(TAG, "  [cat:%d score:%.2f box:(%d,%d)-(%d,%d)]",
                     r.category, r.score, r.box[0], r.box[1], r.box[2], r.box[3]);
        }

        /* 5. Cache results so sensor_task can re-emit them after reconnect,
         *    then publish once directly for clients already connected. */
        if (s_cache_mutex) {
            xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
            memcpy(s_cache, snap.detections,
                   snap.detections_count * sizeof(boat_Detection));
            s_cache_count = snap.detections_count;
            s_cache_ts_us = esp_timer_get_time();
            xSemaphoreGive(s_cache_mutex);
        }
        pipeline_publish_sensors(&snap);

        int64_t total_ms = (esp_timer_get_time() - t0) / 1000;
        ESP_LOGI(TAG, "Detection cycle complete in %lld ms", total_ms);
        runtime_metrics_cycle_end(RUNTIME_TASK_DETECT, esp_timer_get_time());
    }
}

extern "C" void detect_get_cached_results(boat_Detection *out, pb_size_t *out_count,
                                           uint32_t max_age_ms)
{
    *out_count = 0;
    if (!s_cache_mutex || !out) return;
    xSemaphoreTake(s_cache_mutex, portMAX_DELAY);
    int64_t age_us = esp_timer_get_time() - s_cache_ts_us;
    if (s_cache_count > 0 && age_us < (int64_t)max_age_ms * 1000) {
        memcpy(out, s_cache, s_cache_count * sizeof(boat_Detection));
        *out_count = s_cache_count;
    }
    xSemaphoreGive(s_cache_mutex);
}

extern "C" esp_err_t detect_init(void)
{
    s_cache_mutex = xSemaphoreCreateMutex();
    if (!s_cache_mutex) {
        ESP_LOGE(TAG, "Failed to create detect cache mutex");
        return runtime_startup_handle_task_failure(RUNTIME_TASK_DETECT,
                                                   ESP_ERR_NO_MEM);
    }

    /* Create the task BEFORE the model preload: its 32KB stack needs one
     * contiguous internal block, and the esp-dl preload fragments the heap
     * badly enough that grabbing it afterwards fails (seen on hardware with
     * 214KB free but no 32KB block). The task parks on ulTaskNotifyTake and
     * cannot run until a WS trigger — long after this function returns. */
    esp_err_t task_error = runtime_task_create(RUNTIME_TASK_DETECT, detect_task_fn,
                                               NULL, &s_detect_task);
    if (task_error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create detect task (largest internal block: %u B)",
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        return runtime_startup_handle_task_failure(RUNTIME_TASK_DETECT,
                                                   task_error);
    }

    /* Preload model NOW (before WiFi starts) — allocates PSRAM bulk
     * blocks without SDIO DMA interference. */
    ESP_LOGI(TAG, "Preloading cat_detect model...");
    LOG_HEAP();
    s_detector = new CatDetect();
    s_detector->set_score_thr(0.3f);

    /* Force actual model load with a tiny dummy inference */
    uint8_t dummy_pixel[3] = {0, 0, 0};
    dl::image::img_t dummy_img = {
        .data = dummy_pixel,
        .width = 1,
        .height = 1,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888,
    };
    s_detector->run(dummy_img);
    ESP_LOGI(TAG, "Model preloaded and warm");
    LOG_HEAP();

    ESP_LOGI(TAG, "Detect task ready (trigger via WS command)");
    return ESP_OK;
}

extern "C" void detect_trigger(void)
{
    if (s_detect_task) {
        ESP_LOGI(TAG, "Inference triggered");
        xTaskNotifyGive(s_detect_task);
    }
}
