#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include "esp_err.h"

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
