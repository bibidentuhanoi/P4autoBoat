#include "camera_stream.h"
#include "drivers/camera_driver.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_jpeg_enc.h"
#include "linux/videodev2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

static const char *TAG = "CAM_STREAM";

/* MJPEG multipart boundary */
#define BOUNDARY        "frame"
#define PART_BOUNDARY   "--" BOUNDARY "\r\n"
#define PART_HEADER     "Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
#define PART_TAIL       "\r\n"

/* JPEG encode quality (1-100) */
#define JPEG_QUALITY    60

static jpeg_enc_handle_t s_jpeg_enc;
static uint8_t          *s_jpeg_out_buf;
static size_t            s_jpeg_out_size;

/* Map V4L2 pixelformat → jpeg_pixel_format_t + subsampling.
 * The FOURCC values are identical between V4L2 and esp_new_jpeg. */
static esp_err_t map_pixel_format(uint32_t v4l2_fmt,
                                   jpeg_pixel_format_t *src_type,
                                   jpeg_subsampling_t  *subsample)
{
    switch (v4l2_fmt) {
    case V4L2_PIX_FMT_YUYV:   /* YUYV 4:2:2  */
        *src_type  = JPEG_PIXEL_FORMAT_YCbYCr;
        *subsample = JPEG_SUBSAMPLE_422;
        return ESP_OK;
    case V4L2_PIX_FMT_UYVY:   /* UYVY 4:2:2  */
        *src_type  = JPEG_PIXEL_FORMAT_CbYCrY;
        *subsample = JPEG_SUBSAMPLE_422;
        return ESP_OK;
    case V4L2_PIX_FMT_RGB565:
        *src_type  = JPEG_PIXEL_FORMAT_RGB565_LE;
        *subsample = JPEG_SUBSAMPLE_444;
        return ESP_OK;
    case V4L2_PIX_FMT_RGB24:
        *src_type  = JPEG_PIXEL_FORMAT_RGB888;
        *subsample = JPEG_SUBSAMPLE_444;
        return ESP_OK;
    case V4L2_PIX_FMT_GREY:
        *src_type  = JPEG_PIXEL_FORMAT_GRAY;
        *subsample = JPEG_SUBSAMPLE_GRAY;
        return ESP_OK;
    default:
        ESP_LOGE(TAG, "Unsupported pixel format 0x%08" PRIx32, v4l2_fmt);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

/* HTTP handler: streams MJPEG to a single connected client */
static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t ret;
    void     *frame_buf;
    size_t    frame_len;
    int       out_size;
    char      header[64];

    ret = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
    if (ret != ESP_OK) {
        return ret;
    }

    while (true) {
        ret = camera_capture_frame(&frame_buf, &frame_len, NULL, NULL, NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "capture_frame failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        jpeg_error_t jerr = jpeg_enc_process(s_jpeg_enc,
                                              (const uint8_t *)frame_buf, (int)frame_len,
                                              s_jpeg_out_buf, (int)s_jpeg_out_size,
                                              &out_size);
        camera_release_frame();

        if (jerr != JPEG_ERR_OK) {
            ESP_LOGW(TAG, "jpeg_enc_process error %d", jerr);
            continue;
        }

        /* --frame\r\n */
        ret = httpd_resp_send_chunk(req, PART_BOUNDARY, strlen(PART_BOUNDARY));
        if (ret != ESP_OK) break;

        /* X-Timestamp + Content-Type + Content-Length header */
        int64_t ts = esp_timer_get_time();
        char ts_header[32];
        snprintf(ts_header, sizeof(ts_header), "X-Timestamp: %" PRId64 "\r\n", ts);
        ret = httpd_resp_send_chunk(req, ts_header, strlen(ts_header));
        if (ret != ESP_OK) break;

        /* Content-Type + Content-Length header */
        int hlen = snprintf(header, sizeof(header), PART_HEADER, out_size);
        ret = httpd_resp_send_chunk(req, header, hlen);
        if (ret != ESP_OK) break;

        /* JPEG payload */
        ret = httpd_resp_send_chunk(req, (const char *)s_jpeg_out_buf, out_size);
        if (ret != ESP_OK) break;

        /* trailing \r\n */
        ret = httpd_resp_send_chunk(req, PART_TAIL, strlen(PART_TAIL));
        if (ret != ESP_OK) break;
    }

    /* Signal end of chunked response on disconnect */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t s_stream_uri = {
    .uri      = "/stream",
    .method   = HTTP_GET,
    .handler  = stream_handler,
    .user_ctx = NULL,
};

esp_err_t camera_stream_server_start(httpd_handle_t *out_handle)
{
    /* Query camera format to configure the encoder */
    uint32_t width, height, pixel_fmt;
    camera_get_frame_info(&width, &height, &pixel_fmt);

    jpeg_pixel_format_t src_type;
    jpeg_subsampling_t  subsample;
    ESP_RETURN_ON_ERROR(map_pixel_format(pixel_fmt, &src_type, &subsample),
                        TAG, "Cannot map pixel format");

    /* Allocate JPEG output buffer — worst case ~2 bytes/pixel */
    s_jpeg_out_size = (size_t)(width * height * 2);
    s_jpeg_out_buf  = (uint8_t *)malloc(s_jpeg_out_size);
    if (!s_jpeg_out_buf) {
        return ESP_ERR_NO_MEM;
    }

    /* Open software JPEG encoder */
    jpeg_enc_config_t enc_cfg = {
        .width             = (int)width,
        .height            = (int)height,
        .src_type          = src_type,
        .subsampling       = subsample,
        .quality           = JPEG_QUALITY,
        .rotate            = JPEG_ROTATE_0D,
        .task_enable       = false,
    };
    jpeg_error_t jerr = jpeg_enc_open(&enc_cfg, &s_jpeg_enc);
    if (jerr != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "jpeg_enc_open failed: %d", jerr);
        free(s_jpeg_out_buf);
        s_jpeg_out_buf = NULL;
        return ESP_FAIL;
    }

    /* Start HTTP server */
    httpd_handle_t server = NULL;
    httpd_config_t httpd_cfg = HTTPD_DEFAULT_CONFIG();
    httpd_cfg.server_port   = CONFIG_HTTP_STREAM_PORT;
    httpd_cfg.stack_size    = 8192;

    ESP_RETURN_ON_ERROR(httpd_start(&server, &httpd_cfg),
                        TAG, "httpd_start failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &s_stream_uri),
                        TAG, "register /stream failed");

    if (out_handle) {
        *out_handle = server;
    }

    ESP_LOGI(TAG, "MJPEG stream at http://<ip>:%d/stream  (%"PRIu32"x%"PRIu32")",
             CONFIG_HTTP_STREAM_PORT, width, height);
    return ESP_OK;
}
