#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Shared I2C bus mutex — IMU and ToF tasks must hold this during transactions
extern SemaphoreHandle_t g_i2c_mutex;

// Camera presence flag — set once at boot (false if camera_init failed).
// Read by sensor_task to report status; dashboard shows a warning when false.
extern volatile bool g_camera_ok;

/**
 * @brief True when running in ESP-NOW field mode (set once at boot, never
 *        changes afterwards).
 *
 * Consumers trim what they put on the wire: ESP-NOW has ~1/10th the usable
 * bandwidth of the WiFi link. Defaults to false so the full payload — what the
 * dashboard's ToF overlay needs — is the safe fallback.
 */
extern volatile bool g_field_mode;

// Define constants
#ifndef PI
#define PI                  3.14159265358979323846f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG          57.2957795131f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD          0.01745329251f
#endif

#define CALIB_MAGIC_WORD    0xCA11BEEF

// Calibration Data Structure (Strictly packed for NVS safety)
typedef struct __attribute__((packed)) {
    uint32_t magic_word;
    float g_bias[3];
    float m_bias[3];
    float m_scale[3];
    float pitch_tare;
    float roll_tare;
    float heading_tare;
} CalibrationData;

#endif // COMMON_H
