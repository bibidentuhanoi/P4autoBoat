#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize the OV5647 camera driver.
 *
 * MUST be called early in app_main (right after NVS init) so XCLK is
 * provided to the OV5647 before SCCB access begins.
 *
 * Creates I2C_NUM_0 (SCL=CONFIG_CAM_SCCB_SCL_PIN, SDA=CONFIG_CAM_SCCB_SDA_PIN)
 * internally via esp_video — independent of the sensor bus on I2C_NUM_1.
 */
esp_err_t camera_init(void);

/**
 * @brief Capture one frame from the camera.
 *
 * The returned buffer is owned by the driver until camera_release_frame()
 * is called. Do NOT free or modify the pointer.
 *
 * @param[out] buf        Pointer to frame data
 * @param[out] len        Frame data length in bytes
 * @param[out] width      Frame width in pixels  (may be NULL)
 * @param[out] height     Frame height in pixels (may be NULL)
 * @param[out] pixel_fmt  V4L2 pixel format      (may be NULL)
 */
esp_err_t camera_capture_frame(void **buf, size_t *len,
                                uint32_t *width, uint32_t *height,
                                uint32_t *pixel_fmt);

/**
 * @brief Release the current frame back to the driver.
 *
 * Must be called after camera_capture_frame() before calling it again.
 */
void camera_release_frame(void);

/**
 * @brief Query frame dimensions and pixel format without capturing a frame.
 * All output parameters may be NULL.
 */
void camera_get_frame_info(uint32_t *width, uint32_t *height, uint32_t *pixel_fmt);
