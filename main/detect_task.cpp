extern "C" {
#include "detect_task.h"
#include "drivers/camera_driver.h"
#include "camera_stream.h"
#include "pipeline.h"
#include "proto/boat.pb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#include "cat_detect.hpp"
#include "dl_image_define.hpp"

static const char *TAG = "DETECT";

static TaskHandle_t s_detect_task = NULL;
static CatDetect *s_detector = NULL;

static void detect_task_fn(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t t0 = esp_timer_get_time();

        /* Lazy-load model on first trigger */
        if (!s_detector) {
            ESP_LOGI(TAG, "Loading cat_detect model (first trigger)...");
            int64_t load_start = esp_timer_get_time();
            s_detector = new CatDetect();
            ESP_LOGI(TAG, "Model loaded in %lld ms",
                     (esp_timer_get_time() - load_start) / 1000);
        }

        /* 1. Pause MJPEG streaming */
        camera_stop_streaming();
        vTaskDelay(pdMS_TO_TICKS(50));

        /* 2. Restart streaming briefly to get a fresh frame, then capture raw */
        camera_start_streaming();
        vTaskDelay(pdMS_TO_TICKS(100));

        void *raw_buf = NULL;
        size_t raw_len = 0;
        uint32_t width = 0, height = 0;

        esp_err_t cap_ret = camera_capture_raw(&raw_buf, &raw_len, &width, &height);
        if (cap_ret != ESP_OK) {
            ESP_LOGE(TAG, "Raw frame capture failed: %s", esp_err_to_name(cap_ret));
            camera_start_streaming();
            continue;
        }

        ESP_LOGI(TAG, "Captured raw %" PRIu32 "x%" PRIu32 " RGB565 (%u bytes)",
                 width, height, (unsigned)raw_len);

        /* 3. Run inference */
        dl::image::img_t img = {
            .data     = raw_buf,
            .width    = (uint16_t)width,
            .height   = (uint16_t)height,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE,
        };

        int64_t infer_start = esp_timer_get_time();
        auto &results = s_detector->run(img);
        int64_t infer_ms = (esp_timer_get_time() - infer_start) / 1000;

        ESP_LOGI(TAG, "Inference done in %lld ms: %d detections",
                 infer_ms, (int)results.size());

        /* 4. Package results into protobuf */
        boat_SensorSnapshot snap = boat_SensorSnapshot_init_zero;
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

        /* 5. Release frame and resume streaming */
        camera_release_frame();
        camera_start_streaming();

        /* 6. Publish results */
        pipeline_publish_sensors(&snap);

        int64_t total_ms = (esp_timer_get_time() - t0) / 1000;
        ESP_LOGI(TAG, "Detection cycle complete in %lld ms", total_ms);
    }
}

extern "C" esp_err_t detect_init(void)
{
    BaseType_t ret = xTaskCreate(detect_task_fn, "Detect", 8192,
                                  NULL, 5, &s_detect_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create detect task");
        return ESP_ERR_NO_MEM;
    }
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
