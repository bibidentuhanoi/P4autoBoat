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
#include "esp_cam_sensor_xclk.h"
#include "camera_driver.h"

static const char *TAG = "CAM_DRV";

#define CAM_BUF_COUNT  2

static int                        s_cam_fd = -1;
static uint8_t                   *s_cam_buf[CAM_BUF_COUNT];
static uint32_t                   s_cam_buf_size;
static uint32_t                   s_cam_width;
static uint32_t                   s_cam_height;
static uint32_t                   s_cam_pixel_format;
static struct v4l2_buffer         s_current_buf;
static bool                       s_frame_held;
static esp_cam_sensor_xclk_handle_t s_xclk_handle;

esp_err_t camera_init(void)
{
    esp_err_t ret = ESP_OK;
    bool xclk_started  = false;
    bool video_inited  = false;

    // 1. Allocate + start XCLK (only if pin > 0; example uses -1 = not needed)
#if CONFIG_CAM_XCLK_PIN > 0
    esp_cam_sensor_xclk_config_t xclk_cfg = {
        .esp_clock_router_cfg = {
            .xclk_pin     = CONFIG_CAM_XCLK_PIN,
            .xclk_freq_hz = CONFIG_CAM_XCLK_FREQ_HZ,
        },
    };
    ESP_LOGI(TAG, "Starting XCLK: GPIO%d @ %dHz",
             CONFIG_CAM_XCLK_PIN, CONFIG_CAM_XCLK_FREQ_HZ);
    ESP_GOTO_ON_ERROR(
        esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &s_xclk_handle),
        cleanup, TAG, "XCLK allocate failed");
    ESP_GOTO_ON_ERROR(
        esp_cam_sensor_xclk_start(s_xclk_handle, &xclk_cfg),
        cleanup, TAG, "XCLK start failed");
    xclk_started = true;
#else
    ESP_LOGI(TAG, "XCLK pin not configured (using sensor internal clock)");
#endif

    // 2. Init esp_video with MIPI-CSI config.
    //    Uses I2C_NUM_0 (SCL=CAM_SCCB_SCL_PIN=8, SDA=CAM_SCCB_SDA_PIN=7) —
    //    pins reversed from sensor bus (I2C_NUM_1, SCL=7, SDA=8).
    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port    = 0,  // I2C_NUM_0
                .scl_pin = CONFIG_CAM_SCCB_SCL_PIN,
                .sda_pin = CONFIG_CAM_SCCB_SDA_PIN,
            },
            .freq = 400000,  // 400 kHz SCCB (matching example)
        },
        .reset_pin = CONFIG_CAM_RESET_PIN,
        .pwdn_pin  = CONFIG_CAM_PWDN_PIN,
    };
    ESP_LOGI(TAG, "CSI SCCB: I2C0 SCL=%d SDA=%d, RESET=%d, PWDN=%d",
             CONFIG_CAM_SCCB_SCL_PIN, CONFIG_CAM_SCCB_SDA_PIN,
             CONFIG_CAM_RESET_PIN, CONFIG_CAM_PWDN_PIN);

    esp_video_init_config_t cam_cfg = { .csi = &csi_cfg };
    ESP_GOTO_ON_ERROR(esp_video_init(&cam_cfg),
                      cleanup, TAG, "esp_video_init failed");
    video_inited = true;

    // 3. Open V4L2 device
    s_cam_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    ESP_GOTO_ON_FALSE(s_cam_fd >= 0, ESP_ERR_NOT_FOUND,
                      cleanup, TAG, "Open %s failed", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);

    // 4. Query current format (set by sensor driver after init)
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_G_FMT, &fmt),
                      cleanup, TAG, "VIDIOC_G_FMT failed");
    s_cam_width        = fmt.fmt.pix.width;
    s_cam_height       = fmt.fmt.pix.height;
    s_cam_pixel_format = fmt.fmt.pix.pixelformat;
    ESP_LOGI(TAG, "Format: %" PRIu32 "x%" PRIu32 " pix_fmt=0x%08" PRIx32,
             s_cam_width, s_cam_height, s_cam_pixel_format);

    // 5. Request mmap buffers
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = CAM_BUF_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_REQBUFS, &req),
                      cleanup, TAG, "VIDIOC_REQBUFS failed");

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
        ESP_GOTO_ON_FALSE(s_cam_buf[i] != MAP_FAILED, ESP_ERR_NO_MEM,
                          cleanup, TAG, "mmap[%d] failed", i);
        s_cam_buf_size = buf.length;

        ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_QBUF, &buf),
                          cleanup, TAG, "VIDIOC_QBUF[%d] failed", i);
    }

    // 6. Start streaming
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(s_cam_fd, VIDIOC_STREAMON, &type),
                      cleanup, TAG, "VIDIOC_STREAMON failed");

    ESP_LOGI(TAG, "Camera ready: %" PRIu32 "x%" PRIu32, s_cam_width, s_cam_height);
    return ESP_OK;

cleanup:
    if (s_cam_fd >= 0) {
        close(s_cam_fd);
        s_cam_fd = -1;
    }
    if (video_inited) {
        esp_video_deinit();
    }
#if CONFIG_CAM_XCLK_PIN > 0
    if (xclk_started) {
        esp_cam_sensor_xclk_stop(s_xclk_handle);
    }
    if (s_xclk_handle) {
        esp_cam_sensor_xclk_free(s_xclk_handle);
        s_xclk_handle = NULL;
    }
#endif
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

    *buf = s_cam_buf[s_current_buf.index];
    *len = s_current_buf.bytesused ? s_current_buf.bytesused : s_cam_buf_size;
    if (width)     *width     = s_cam_width;
    if (height)    *height    = s_cam_height;
    if (pixel_fmt) *pixel_fmt = s_cam_pixel_format;

    s_frame_held = true;
    return ESP_OK;
}

void camera_release_frame(void)
{
    if (!s_frame_held || s_cam_fd < 0) {
        return;
    }
    ioctl(s_cam_fd, VIDIOC_QBUF, &s_current_buf);
    s_frame_held = false;
}

void camera_get_frame_info(uint32_t *width, uint32_t *height, uint32_t *pixel_fmt)
{
    if (width)     *width     = s_cam_width;
    if (height)    *height    = s_cam_height;
    if (pixel_fmt) *pixel_fmt = s_cam_pixel_format;
}
