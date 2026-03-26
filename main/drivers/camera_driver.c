#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "camera_driver.h"

static const char *TAG = "CAM_DRV";

#define CAM_BUF_COUNT  2

static int                s_cam_fd = -1;
static uint8_t           *s_cam_buf[CAM_BUF_COUNT];
static uint32_t           s_cam_buf_size;
static uint32_t           s_cam_width;
static uint32_t           s_cam_height;
static uint32_t           s_cam_pixel_format;
static struct v4l2_buffer s_current_buf;
static bool               s_frame_held;

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

    /* 3. Flip — one VIDIOC_S_CTRL call per control (reference pattern, not EXT_CTRLS).
     *    Warn but do not fail — some sensor builds may not support flip. */
    {
        struct v4l2_control ctrl;

        ctrl.id    = V4L2_CID_VFLIP;
        ctrl.value = CONFIG_CAM_VFLIP;
        if (ioctl(s_cam_fd, VIDIOC_S_CTRL, &ctrl) != 0) {
            ESP_LOGW(TAG, "VFLIP ioctl failed (non-fatal)");
        }

        ctrl.id    = V4L2_CID_HFLIP;
        ctrl.value = CONFIG_CAM_HFLIP;
        if (ioctl(s_cam_fd, VIDIOC_S_CTRL, &ctrl) != 0) {
            ESP_LOGW(TAG, "HFLIP ioctl failed (non-fatal)");
        }

        ESP_LOGI(TAG, "Flip: VFLIP=%d HFLIP=%d", CONFIG_CAM_VFLIP, CONFIG_CAM_HFLIP);
    }

    /* 4. Read actual output format — always log FourCC.
     *    ISP pipeline + HW JPEG device → must be V4L2_PIX_FMT_JPEG. Fail loudly if not. */
    {
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_G_FMT, &fmt),
                          cleanup, TAG, "VIDIOC_G_FMT failed");

        s_cam_width        = fmt.fmt.pix.width;
        s_cam_height       = fmt.fmt.pix.height;
        s_cam_pixel_format = fmt.fmt.pix.pixelformat;

        char fourcc[5] = {
            (char)( s_cam_pixel_format        & 0xFF),
            (char)((s_cam_pixel_format >>  8) & 0xFF),
            (char)((s_cam_pixel_format >> 16) & 0xFF),
            (char)((s_cam_pixel_format >> 24) & 0xFF),
            '\0'
        };
        ESP_LOGI(TAG, "Format: %"PRIu32"x%"PRIu32" fmt=%s (0x%08"PRIx32")",
                 s_cam_width, s_cam_height, fourcc, s_cam_pixel_format);

        if (s_cam_pixel_format != V4L2_PIX_FMT_JPEG) {
            ESP_LOGE(TAG, "Expected JPEG (ISP+HW-JPEG pipeline) but got fmt=%s. "
                     "sdkconfig must have CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER=y "
                     "and CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE=y", fourcc);
            ret = ESP_ERR_NOT_SUPPORTED;
            goto cleanup;
        }
    }

    /* 5. Set JPEG quality.
     *    Reference: set_camera_jpeg_quality() in example_encoder.c (lines 243–294).
     *    Query valid range → clamp desired value → set via V4L2_CID_JPEG_CLASS ctrl_class. */
    {
        struct v4l2_query_ext_ctrl qctrl = {0};
        qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;

        if (ioctl(s_cam_fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
            int quality = CONFIG_CAM_JPEG_QUALITY;
            if      (quality > (int)qctrl.maximum) quality = (int)qctrl.maximum;
            else if (quality < (int)qctrl.minimum) quality = (int)qctrl.minimum;
            else quality = (int)qctrl.minimum +
                           ((quality - (int)qctrl.minimum) / (int)qctrl.step) * (int)qctrl.step;

            struct v4l2_ext_controls controls = {0};
            struct v4l2_ext_control  control[1];
            controls.ctrl_class = V4L2_CID_JPEG_CLASS;
            controls.count      = 1;
            controls.controls   = control;
            control[0].id       = V4L2_CID_JPEG_COMPRESSION_QUALITY;
            control[0].value    = quality;

            if (ioctl(s_cam_fd, VIDIOC_S_EXT_CTRLS, &controls) == 0) {
                ESP_LOGI(TAG, "JPEG quality set to %d (range %lld-%lld step %lld)",
                         quality, qctrl.minimum, qctrl.maximum, qctrl.step);
            } else {
                ESP_LOGW(TAG, "JPEG quality ioctl failed — device default used");
            }
        } else {
            ESP_LOGW(TAG, "JPEG quality not queryable — device default used");
        }
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

    /* 7. Query, mmap, and queue each buffer */
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

    /* 8. Start streaming */
    {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_STREAMON, &type),
                          cleanup, TAG, "VIDIOC_STREAMON failed");
    }

    ESP_LOGI(TAG, "Camera ready: %"PRIu32"x%"PRIu32" JPEG", s_cam_width, s_cam_height);
    return ESP_OK;

cleanup:
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

    *buf = s_cam_buf[s_current_buf.index];
    /* bytesused = actual JPEG payload. Never use full buf.length — that includes
     * uninitialised memory past the JPEG end marker (causes browser decode errors). */
    *len = s_current_buf.bytesused ? s_current_buf.bytesused : s_cam_buf_size;
    if (width)     *width     = s_cam_width;
    if (height)    *height    = s_cam_height;
    if (pixel_fmt) *pixel_fmt = s_cam_pixel_format;

    s_frame_held = true;
    return ESP_OK;
}

void camera_release_frame(void)
{
    if (!s_frame_held || s_cam_fd < 0) return;
    ioctl(s_cam_fd, VIDIOC_QBUF, &s_current_buf);
    s_frame_held = false;
}

void camera_get_frame_info(uint32_t *width, uint32_t *height, uint32_t *pixel_fmt)
{
    if (width)     *width     = s_cam_width;
    if (height)    *height    = s_cam_height;
    if (pixel_fmt) *pixel_fmt = s_cam_pixel_format;
}
