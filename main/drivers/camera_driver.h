#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize camera using a pre-created SCCB I2C bus handle.
 *
 * Uses init_sccb=false (SCCB_I2C_INIT_BY_APP pattern). The caller creates
 * the SCCB I2C bus, passes it here, then deletes it after this call returns
 * (SCCB is only needed during sensor register init — camera streams via MIPI CSI).
 */
esp_err_t camera_init(i2c_master_bus_handle_t sccb_handle);

/** @brief Capture one frame. Caller MUST call camera_release_frame() after use. */
esp_err_t camera_capture_frame(void **buf, size_t *len,
                                uint32_t *width, uint32_t *height,
                                uint32_t *pixel_fmt);

/** Capture raw RGB565 frame from ISP (no JPEG encode). Call camera_release_frame() when done. */
esp_err_t camera_capture_raw(void **buf, size_t *len, uint32_t *width, uint32_t *height);

/** @brief Return the held frame buffer back to the driver. */
void camera_release_frame(void);

/** @brief Set JPEG encode quality (1-100, higher = better/larger). Applies to
 *  the next captured frame. Used to shrink images for the ESP-NOW field link. */
void camera_set_jpeg_quality(int quality);

/** @brief Start V4L2 streaming (VIDIOC_STREAMON). Call before capturing frames. Idempotent. */
esp_err_t camera_start_streaming(void);

/** @brief Stop V4L2 streaming (VIDIOC_STREAMOFF). Stops ISP pipeline. Idempotent. */
esp_err_t camera_stop_streaming(void);

/** @brief Lightweight drain — DQBUF+QBUF only, no PPA/JPEG. Keeps ISP pipeline alive. */
void camera_drain_frame(void);

/** @brief Query resolution and pixel format without capturing. */
void camera_get_frame_info(uint32_t *width, uint32_t *height, uint32_t *pixel_fmt);
