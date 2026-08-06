/*
 * espnow_video.c — periodic JPEG stills over the ESP-NOW field link.
 *
 * The camera is already streaming by the time we get here: camera_init() issues
 * VIDIOC_STREAMON, so frames can be captured in field mode even though the
 * MJPEG server (which only starts on the WiFi branch) is not running.
 *
 * Kept as its own module rather than living in espnow_transport.c so the
 * transport stays a transport and does not acquire a camera dependency.
 */
#include "espnow_video.h"
#include "common.h"
#include "drivers/camera_driver.h"
#include "transports/espnow_transport.h"

#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

static const char *TAG = "ESPNOW_VIDEO";

/* Set by detect_task while ML inference owns the camera. */
extern volatile bool g_inference_active;

#if CONFIG_ESPNOW_JPEG_INTERVAL_MS > 0

static void espnow_video_task(void *arg)
{
    (void)arg;

    /* Let telemetry establish itself before competing for airtime. */
    vTaskDelay(pdMS_TO_TICKS(3000));

    uint32_t sent = 0, skipped = 0;
    bool warned_nocam = false;
    uint32_t cap_fail = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_JPEG_INTERVAL_MS));

        if (!g_camera_ok) {
            /* Say so ONCE. Silently spinning here is how a dead feature looks
             * identical to a disabled one. */
            if (!warned_nocam) {
                warned_nocam = true;
                ESP_LOGE(TAG, "camera not available (init failed at boot) — no video");
            }
            continue;
        }

        /* detect_task holds a frame while inferring; camera_capture_frame()
         * returns ESP_ERR_INVALID_STATE if a frame is already held, so back off
         * rather than fight it. */
        if (g_inference_active) {
            skipped++;
            continue;
        }

        void    *buf = NULL;
        size_t   len = 0;
        uint32_t w = 0, h = 0, fmt = 0;

        esp_err_t ret = camera_capture_frame(&buf, &len, &w, &h, &fmt);
        if (ret != ESP_OK) {
            skipped++;
            /* Was ESP_LOGD, i.e. invisible at default log level — a capture
             * that always fails looked exactly like video never running.
             * Shout for the first few, then throttle. */
            if (++cap_fail <= 3 || (cap_fail % 20) == 0) {
                ESP_LOGW(TAG, "capture failed (#%u): %s — nothing drains the "
                              "camera in field mode, so buffers may be starved",
                         (unsigned)cap_fail, esp_err_to_name(ret));
            }
            continue;
        }

        if (buf && len > 0) {
            ret = espnow_transport_send_jpeg((const uint8_t *)buf, len);
            if (ret == ESP_OK) {
                sent++;
                ESP_LOGI(TAG, "JPEG sent: %ux%u, %u bytes (sent=%u skipped=%u)",
                         (unsigned)w, (unsigned)h, (unsigned)len,
                         (unsigned)sent, (unsigned)skipped);
            } else {
                skipped++;
                ESP_LOGW(TAG, "JPEG send failed (%s) — %u bytes",
                         esp_err_to_name(ret), (unsigned)len);
            }
        }

        /* MUST release, or the next capture returns ESP_ERR_INVALID_STATE
         * forever (s_frame_held stays true and the V4L2 buffer is never
         * requeued). */
        camera_release_frame();
    }
}

esp_err_t espnow_video_start(void)
{
    if (!g_field_mode) {
        ESP_LOGW(TAG, "not in field mode — video not started");
        return ESP_ERR_INVALID_STATE;
    }

    if (xTaskCreate(espnow_video_task, "espnow_video", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create video task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Field-mode video enabled: 1 JPEG every %d ms",
             CONFIG_ESPNOW_JPEG_INTERVAL_MS);
    return ESP_OK;
}

#else  /* CONFIG_ESPNOW_JPEG_INTERVAL_MS == 0 */

esp_err_t espnow_video_start(void)
{
    ESP_LOGI(TAG, "Field-mode video disabled (ESPNOW_JPEG_INTERVAL_MS = 0)");
    return ESP_OK;
}

#endif
