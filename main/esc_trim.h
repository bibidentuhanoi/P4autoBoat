#pragma once
#include <stdint.h>
#include <stdbool.h>

#define ESC_TRIM_MAX_POINTS 8
#define ESC_TRIM_NVS_MAGIC 0x54524931u   /* "TRI1" -- bump if the on-disk layout ever changes */

typedef struct __attribute__((packed)) {
    float throttle_frac;   /* common throttle 0..1 this point was learned at */
    float trim_diff;       /* signed differential: left = T - trim/2, right = T + trim/2 */
} EscTrimPoint;

/* The whole persisted record -- one NVS blob, one key ("esc_trim"), entirely
 * independent of CalibrationData/"imu_cal". magic_word guards against a
 * corrupted or absent blob; count==0 is always a safe "no trim" state. */
typedef struct __attribute__((packed)) {
    uint32_t magic_word;
    uint8_t  count;
    EscTrimPoint points[ESC_TRIM_MAX_POINTS];
} EscTrimNvsBlob;

/* Interpolate the learned differential trim for a given common throttle
 * (0..1). Points need not be pre-sorted. count==0 -> 0. Outside the
 * calibrated range, clamp to the nearest endpoint's trim (do NOT extrapolate
 * -- a linear extrapolation off the ends can command large differentials the
 * calibration never validated). */
float esc_trim_lookup(const EscTrimPoint *pts, uint8_t count, float common_throttle);

/* Apply the looked-up trim symmetrically about the common throttle, preserving
 * the pilot's turn differential already present in *left / *right:
 *   common = (L+R)/2 ; L -= trim/2 ; R += trim/2.
 * This is the exact "trim on common, pilot turn intentional" ordering; the
 * per-ESC floor/curve (esc_map) still runs AFTER this, in esc_driver. */
void esc_trim_apply(float *left, float *right, const EscTrimPoint *pts, uint8_t count);

/* Mix an original normalized throttle/rudder command, applying static trim
 * before saturation. Pilot steering is preserved first; trim is reduced when
 * there is no remaining actuator headroom. */
void esc_trim_mix(float throttle, float rudder, const EscTrimPoint *pts,
                  uint8_t count, float *left, float *right);
