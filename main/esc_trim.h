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
/* Apply a trim to an already-mixed (left,right) pair, the same way
 * esc_trim_mix does: left -= trim/2, right += trim/2, clamped to [0,1].
 *
 * Stopped stays stopped: if BOTH inputs are zero the pair is returned
 * untouched. Without that guard a trim would spin one motor up while the
 * caller believes the motors are off -- which silently ruins the bench run's
 * motors-off baseline, the very measurement the trim is derived from. */
void esc_trim_apply_pair(float trim, float *left, float *right);

/* Build the table that makes the trim PROPORTIONAL to throttle:
 *
 *     trim(T) = 2*c*T   ->   left = T*(1-c),  right = T*(1+c)
 *
 * One dimensionless number covers the whole throttle range. That is what the
 * bench measured: the split that makes the boat go straight is 2.3% at T10,
 * 6.5% at T30, 7.0% at T35 and 7.5% at T40 -- not a constant, but a constant
 * FRACTION (c ~ 0.20) of throttle. A flat trim tuned at T40 over-corrects
 * badly at T10 (measured +8.4 deg/s), which is exactly the failure this fixes.
 *
 * Two points, (0,0) and (1, 2c), because esc_trim_lookup interpolates linearly
 * between them -- so the straight line through the origin IS the model, and no
 * per-throttle table has to be filled in.
 *
 * `out` must have room for 2 points. Returns the point count, or 0 when c is
 * unusable (NaN or zero), which the caller reads as "no trim". */
uint8_t esc_trim_build_proportional(float c, EscTrimPoint *out);

void esc_trim_mix(float throttle, float rudder, const EscTrimPoint *pts,
                  uint8_t count, float *left, float *right);
