#include "camera_stream.h"
#include "drivers/camera_driver.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "linux/videodev2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "CAM_STREAM";

/* Multipart MIME — reference (simple_video_server_example.c line 37):
 *   STREAM_BOUNDARY = "\r\n--" EXAMPLE_PART_BOUNDARY "\r\n"
 * Leading \r\n is required by RFC 2046 for all parts after the first. */
#define BOUNDARY      "frame"
#define PART_BOUNDARY "\r\n--" BOUNDARY "\r\n"
#define PART_HEADER   "Content-Type: image/jpeg\r\nContent-Length: %"PRIu32"\r\n\r\n"

/* ---- Frame drain ----
 * The ISP pipeline runs continuously after VIDIOC_STREAMON.
 * If nobody calls VIDIOC_DQBUF the buffer queue fills up,
 * ISP metadata indices corrupt → crash.
 * This task does lightweight drain (DQBUF+QBUF, no PPA/JPEG)
 * when no MJPEG client is connected. Near-zero CPU cost. */
static volatile bool s_client_streaming = false;

static void camera_drain_task(void *pvParameters)
{
    while (true) {
        if (!s_client_streaming) {
            camera_drain_frame();
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    void     *frame_buf;
    size_t    frame_len;
    char      hdr[96];
    esp_err_t ret;

    s_client_streaming = true;

    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        ret = camera_capture_frame(&frame_buf, &frame_len, NULL, NULL, NULL);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* Boundary */
        ret = httpd_resp_send_chunk(req, PART_BOUNDARY, strlen(PART_BOUNDARY));
        if (ret != ESP_OK) { camera_release_frame(); break; }

        /* Content-Type + Content-Length */
        int hlen = snprintf(hdr, sizeof(hdr), PART_HEADER, (uint32_t)frame_len);
        ret = httpd_resp_send_chunk(req, hdr, hlen);
        if (ret != ESP_OK) { camera_release_frame(); break; }

        /* JPEG payload — bytesused bytes only, not the full mmap allocation */
        ret = httpd_resp_send_chunk(req, (const char *)frame_buf, (ssize_t)frame_len);
        camera_release_frame();
        if (ret != ESP_OK) break;

        /* Throttle to ~15fps — with PPA removed, per-frame DMA cost is halved.
         * 800x640 JPEG at quality 60 ~40-50KB/frame, 15fps = ~5 Mbps.
         * SDIO link is ~3.3 Mbps so some frames will get EAGAIN backpressure
         * (handled gracefully by the WS transport). */
        vTaskDelay(pdMS_TO_TICKS(66));
    }

    s_client_streaming = false;

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t s_stream_uri = {
    .uri      = "/stream",
    .method   = HTTP_GET,
    .handler  = stream_handler,
    .user_ctx = NULL,
};

esp_err_t camera_stream_server_start(void)
{
    uint32_t width, height, pixel_fmt;
    camera_get_frame_info(&width, &height, &pixel_fmt);

    if (pixel_fmt != V4L2_PIX_FMT_JPEG) {
        ESP_LOGE(TAG, "Camera not in JPEG mode (0x%08"PRIx32") — stream aborted",
                 pixel_fmt);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "MJPEG stream: %"PRIu32"x%"PRIu32" -> http://<ip>:%d/stream",
             width, height, CONFIG_HTTP_STREAM_PORT);

    /* Dedicated server for MJPEG only — isolated from API server on port 80.
     * The stream handler is a blocking while(1) loop; keeping it separate
     * prevents it from starving the API/dashboard httpd task. */
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = CONFIG_HTTP_STREAM_PORT;
    cfg.ctrl_port        = 32769;  /* must differ from port-80 server (32768) */
    cfg.stack_size       = 8192;
    cfg.max_open_sockets = 4;
    cfg.max_uri_handlers = 2;

    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &cfg), TAG, "httpd_start failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &s_stream_uri),
                        TAG, "register /stream failed");

    /* Lightweight drain — keeps ISP pipeline alive when no MJPEG client */
    xTaskCreate(camera_drain_task, "CamDrain", 2048, NULL, 2, NULL);

    return ESP_OK;
}
