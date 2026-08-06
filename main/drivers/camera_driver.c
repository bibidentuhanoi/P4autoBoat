#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "driver/jpeg_encode.h"
#include "driver/jpeg_types.h"
/* PPA removed — eliminates 2 DMA hops per frame (PPA read + PPA write).
 * Rotation handled in dashboard CSS instead. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_driver.h"

static const char *TAG = "CAM_DRV";

/* Encode quality, applied per frame. Runtime-settable because ESP-NOW field
 * mode needs far smaller images than the WiFi MJPEG stream: one lost chunk
 * loses a whole image (there is no retransmission), so fewer chunks means a
 * far higher chance of a complete frame. */
static int s_jpeg_quality = CONFIG_CAM_JPEG_QUALITY;

void camera_set_jpeg_quality(int quality)
{
    if (quality < 1)   quality = 1;
    if (quality > 100) quality = 100;
    s_jpeg_quality = quality;
    ESP_LOGI(TAG, "JPEG encode quality set to %d", quality);
}

#define CAM_BUF_COUNT  2

static int                    s_cam_fd = -1;
static uint8_t               *s_cam_buf[CAM_BUF_COUNT];
static uint32_t               s_cam_buf_size;
static uint32_t               s_cam_width;
static uint32_t               s_cam_height;
static uint32_t               s_cam_pixel_format;
static struct v4l2_buffer     s_current_buf;
static bool                   s_frame_held;
static jpeg_encoder_handle_t  s_jpeg_enc;
static uint8_t               *s_jpeg_buf;
static uint32_t               s_jpeg_buf_size;
static uint32_t               s_jpeg_out_len;
static bool                   s_streaming;

esp_err_t camera_init(i2c_master_bus_handle_t sccb_handle)
{
    esp_err_t ret = ESP_OK;
    bool video_inited = false;

    /* 1. Init esp_video — CSI config only, NO .jpeg field.
     *    Reference: example_init_video.c, SCCB_I2C_INIT_BY_APP branch (lines 185–268).
     *    init_sccb=false: we pass the pre-created bus. Caller deletes it after we return.
     *    The .jpeg field is intentionally absent — it corrupts the ISP→HW-JPEG pipeline. */
    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = sccb_handle,
            .freq = 400000,
        },
        .reset_pin = CONFIG_CAM_RESET_PIN,
        .pwdn_pin  = CONFIG_CAM_PWDN_PIN,
    };
    esp_video_init_config_t cam_cfg = {
        .csi = &csi_cfg,
        /* NO .jpeg — reference never uses it. Setting it = wrong colors + lines. */
    };

    ret = esp_video_init(&cam_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed: %s. "
                 "Check sdkconfig: CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER=y "
                 "and CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE=y must both be set",
                 esp_err_to_name(ret));
        return ret;
    }
    video_inited = true;

    /* 2. Open V4L2 device */
    s_cam_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    if (s_cam_fd < 0) {
        ESP_LOGE(TAG, "Open %s failed", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    /* 3. Flip — sensor-level VFLIP/HFLIP shifts the Bayer pattern and breaks ISP colors.
     *    PPA hardware 180° rotation is applied on the RGB565 buffer in camera_capture_frame(). */

    /* 4. Read actual output format — always log FourCC.
     *    ISP pipeline outputs RGB565 (RGBP). That is expected — we encode to JPEG below.
     *    V4L2_PIX_FMT_RGB565X (big-endian) requires CONFIG_ESP_VIDEO_ENABLE_SWAP_BYTE. */
    {
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_G_FMT, &fmt),
                          cleanup, TAG, "VIDIOC_G_FMT failed");

        s_cam_width  = fmt.fmt.pix.width;
        s_cam_height = fmt.fmt.pix.height;
        uint32_t raw_fmt = fmt.fmt.pix.pixelformat;

        char fourcc[5] = {
            (char)( raw_fmt        & 0xFF),
            (char)((raw_fmt >>  8) & 0xFF),
            (char)((raw_fmt >> 16) & 0xFF),
            (char)((raw_fmt >> 24) & 0xFF),
            '\0'
        };
        ESP_LOGI(TAG, "CSI format: %"PRIu32"x%"PRIu32" raw=%s (0x%08"PRIx32") — will encode to JPEG",
                 s_cam_width, s_cam_height, fourcc, raw_fmt);

        if (raw_fmt != V4L2_PIX_FMT_RGB565) {
            ESP_LOGE(TAG, "Expected RGB565 from ISP pipeline but got %s. "
                     "Check CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER=y in sdkconfig.", fourcc);
            ret = ESP_ERR_NOT_SUPPORTED;
            goto cleanup;
        }
    }

    /* 5. Init HW JPEG encoder (esp_driver_jpeg).
     *    Matches example_encoder.c with CONFIG_EXAMPLE_SELECT_JPEG_HW_DRIVER=y.
     *    Encodes RGB565 → JPEG in camera_capture_frame() using this engine.
     *    Output buffer: width*height*2 bytes — generous upper bound for any quality. */
    {
        jpeg_encode_engine_cfg_t eng_cfg = { .timeout_ms = 40 };
        ESP_GOTO_ON_ERROR(jpeg_new_encoder_engine(&eng_cfg, &s_jpeg_enc),
                          cleanup, TAG, "jpeg_new_encoder_engine failed");

        /* DMA-aware allocation — jpeg_alloc_encoder_mem returns properly
         * aligned memory for the JPEG DMA engine (matches simple_video_server). */
        jpeg_encode_memory_alloc_cfg_t jpeg_mem_cfg = {
            .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
        };
        size_t jpeg_alloc_size = 0;
        s_jpeg_buf = (uint8_t *)jpeg_alloc_encoder_mem(
            s_cam_width * s_cam_height * 2, &jpeg_mem_cfg, &jpeg_alloc_size);
        if (!s_jpeg_buf) {
            ESP_LOGE(TAG, "JPEG output buffer alloc failed");
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
        s_jpeg_buf_size = jpeg_alloc_size;

        /* Report format as JPEG to upper layers — stream code checks this. */
        s_cam_pixel_format = V4L2_PIX_FMT_JPEG;
        ESP_LOGI(TAG, "JPEG encoder ready: quality=%d buf=%"PRIu32" bytes",
                 CONFIG_CAM_JPEG_QUALITY, s_jpeg_buf_size);
    }

    /* 6. Request mmap buffers */
    {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count  = CAM_BUF_COUNT;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_REQBUFS, &req),
                          cleanup, TAG, "VIDIOC_REQBUFS failed");
    }

    /* 8. Query, mmap, and queue each buffer */
    for (int i = 0; i < CAM_BUF_COUNT; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_QUERYBUF, &buf),
                          cleanup, TAG, "VIDIOC_QUERYBUF[%d] failed", i);

        s_cam_buf[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, s_cam_fd, buf.m.offset);
        if (s_cam_buf[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap[%d] failed", i);
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
        s_cam_buf_size = buf.length;

        ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_QBUF, &buf),
                          cleanup, TAG, "VIDIOC_QBUF[%d] failed", i);
    }

    /* 9. Start streaming — ISP pipeline stays alive.
     *    A drain task in camera_stream.c keeps buffers cycling
     *    when no MJPEG client is connected (prevents ISP crash). */
    {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_STREAMON, &type),
                          cleanup, TAG, "VIDIOC_STREAMON failed");
        s_streaming = true;
    }

    ESP_LOGI(TAG, "Camera ready: %"PRIu32"x%"PRIu32" JPEG", s_cam_width, s_cam_height);
    return ESP_OK;

cleanup:
    if (s_jpeg_buf) {
        free(s_jpeg_buf);
        s_jpeg_buf = NULL;
    }
    if (s_jpeg_enc) {
        jpeg_del_encoder_engine(s_jpeg_enc);
        s_jpeg_enc = NULL;
    }
    if (s_cam_fd >= 0) {
        close(s_cam_fd);
        s_cam_fd = -1;
    }
    if (video_inited) {
        esp_video_deinit();
    }
    return ret;
}

esp_err_t camera_capture_frame(void **buf, size_t *len,
                                uint32_t *width, uint32_t *height,
                                uint32_t *pixel_fmt)
{
    if (s_cam_fd < 0 || s_frame_held) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_current_buf, 0, sizeof(s_current_buf));
    s_current_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    s_current_buf.memory = V4L2_MEMORY_MMAP;

    ESP_RETURN_ON_ERROR(ioctl(s_cam_fd, VIDIOC_DQBUF, &s_current_buf),
                        TAG, "VIDIOC_DQBUF failed");

    if (!(s_current_buf.flags & V4L2_BUF_FLAG_DONE)) {
        ioctl(s_cam_fd, VIDIOC_QBUF, &s_current_buf);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* ISP buffer → JPEG directly (no PPA rotation — handled in dashboard CSS).
     * This eliminates 2 DMA hops per frame vs the old PPA path. */
    uint8_t *rgb_buf = s_cam_buf[s_current_buf.index];
    uint32_t rgb_len = s_current_buf.bytesused ? s_current_buf.bytesused : s_cam_buf_size;

    /* Encode RGB565 → JPEG using HW encoder. */
    jpeg_encode_cfg_t enc_cfg = {
        .src_type      = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample    = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = s_jpeg_quality,
        .width         = s_cam_width,
        .height        = s_cam_height,
    };
    s_jpeg_out_len = 0;

    esp_err_t enc_ret = jpeg_encoder_process(s_jpeg_enc, &enc_cfg,
                                              rgb_buf, rgb_len,
                                              s_jpeg_buf, s_jpeg_buf_size,
                                              &s_jpeg_out_len);
    if (enc_ret != ESP_OK || s_jpeg_out_len == 0) {
        ioctl(s_cam_fd, VIDIOC_QBUF, &s_current_buf);
        ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(enc_ret));
        return ESP_ERR_INVALID_RESPONSE;
    }

    *buf = s_jpeg_buf;
    *len = s_jpeg_out_len;
    if (width)     *width     = s_cam_width;
    if (height)    *height    = s_cam_height;
    if (pixel_fmt) *pixel_fmt = s_cam_pixel_format; /* V4L2_PIX_FMT_JPEG */

    s_frame_held = true;
    return ESP_OK;
}

esp_err_t camera_capture_raw(void **buf, size_t *len,
                              uint32_t *width, uint32_t *height)
{
    if (s_cam_fd < 0 || s_frame_held) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_current_buf, 0, sizeof(s_current_buf));
    s_current_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    s_current_buf.memory = V4L2_MEMORY_MMAP;

    ESP_RETURN_ON_ERROR(ioctl(s_cam_fd, VIDIOC_DQBUF, &s_current_buf),
                        TAG, "VIDIOC_DQBUF failed");

    if (!(s_current_buf.flags & V4L2_BUF_FLAG_DONE)) {
        ioctl(s_cam_fd, VIDIOC_QBUF, &s_current_buf);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *buf = s_cam_buf[s_current_buf.index];
    *len = s_current_buf.bytesused ? s_current_buf.bytesused : s_cam_buf_size;
    if (width)  *width  = s_cam_width;
    if (height) *height = s_cam_height;

    s_frame_held = true;
    return ESP_OK;
}

void camera_release_frame(void)
{
    if (!s_frame_held || s_cam_fd < 0) return;
    ioctl(s_cam_fd, VIDIOC_QBUF, &s_current_buf);
    s_frame_held = false;
}

esp_err_t camera_start_streaming(void)
{
    if (s_cam_fd < 0) return ESP_ERR_INVALID_STATE;
    if (s_streaming) return ESP_OK;

    /* Re-queue all buffers — STREAMOFF dequeues them */
    for (int i = 0; i < CAM_BUF_COUNT; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ioctl(s_cam_fd, VIDIOC_QBUF, &buf);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_ERROR(ioctl(s_cam_fd, VIDIOC_STREAMON, &type),
                        TAG, "VIDIOC_STREAMON failed");
    s_streaming = true;
    ESP_LOGI(TAG, "Camera streaming started");
    return ESP_OK;
}

esp_err_t camera_stop_streaming(void)
{
    if (s_cam_fd < 0) return ESP_ERR_INVALID_STATE;
    if (!s_streaming) return ESP_OK;

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_RETURN_ON_ERROR(ioctl(s_cam_fd, VIDIOC_STREAMOFF, &type),
                        TAG, "VIDIOC_STREAMOFF failed");
    s_streaming = false;
    s_frame_held = false;
    ESP_LOGI(TAG, "Camera streaming stopped");
    return ESP_OK;
}

void camera_drain_frame(void)
{
    if (s_cam_fd < 0 || s_frame_held) return;

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(s_cam_fd, VIDIOC_DQBUF, &buf) == 0) {
        ioctl(s_cam_fd, VIDIOC_QBUF, &buf);
    }
}

void camera_get_frame_info(uint32_t *width, uint32_t *height, uint32_t *pixel_fmt)
{
    if (width)     *width     = s_cam_width;
    if (height)    *height    = s_cam_height;
    if (pixel_fmt) *pixel_fmt = s_cam_pixel_format;
}
