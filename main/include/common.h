#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

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

/* v2 (2026-09-24): flat compass calibration.  A new value, not just a new
 * size, so an old record can never be read as this one.  v3: + tolerance. */
#define CALIB_MAGIC_WORD    0xCA11B003   /* v3: + gyro disagreement and tolerance */

// Calibration Data Structure (Strictly packed for NVS safety)
typedef struct __attribute__((packed)) {
    uint32_t magic_word;
    float g_bias[3];        /* gyro drift, raw counts */
    float pitch_tare;       /* level reference, deg */
    float roll_tare;
    /* Compass: corrected = soft * (raw_xy - center), heading = atan2(y, x).
     * No heading offset: north comes from the magnetic field itself. */
    float mag_center[2];    /* hard iron, raw counts */
    float mag_soft[4];      /* soft iron, row-major 2x2 */
    float mag_radius;       /* corrected field strength, raw counts; 0 = never calibrated */
    uint32_t mag_calibrated;/* 1 once a compass calibration has PASSED */
    float mag_gyro_dev_deg;   /* its worst compass-vs-gyro disagreement, deg */
    float mag_tolerance_deg;  /* the limit it passed with (5 strict .. 25 relaxed) */
} CalibrationData;

#endif // COMMON_H
