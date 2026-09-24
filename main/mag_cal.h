#pragma once

/* Compass (magnetometer) calibration maths.  Pure C, no ESP-IDF, so every
 * piece can be tested on the host.
 *
 * The boat never tilts more than a few degrees, so the calibration is the
 * flat ("2-D") kind marine autopilots use: spin the boat level through a
 * couple of full circles, fit an ellipse to the horizontal compass readings,
 * and store the transform that turns that ellipse back into a circle:
 *
 *     corrected = soft * (raw - center)      heading = atan2(y, x)
 *
 *   center  hard iron: the boat's own magnets shift the circle
 *   soft    soft iron: nearby steel squashes it into an ellipse
 *
 * Nothing here needs to know which way north is.  A correct circle gives the
 * magnetic heading directly; no "point the bow and set zero" step exists.
 *
 * The calibration runs in two steps, both judged on the data, not on a timer:
 *   1. hold still  -> gyro drift + level reference (mag_still_*)
 *   2. spin flat   -> ellipse fit + quality gate (mag_circle_*, mag_cal_*) */

#include <stdbool.h>
#include <stdint.h>

/* Coverage is tracked in 32 slices of 11.25 deg so one uint32 holds it. */
#define MAG_CAL_BINS 32

/* Quality gate.  A calibration that fails any of these is not saved. */
#define MAG_CAL_MIN_SAMPLES        200
#define MAG_CAL_MIN_TURN_DEG       540.0f  /* 1.5 turns, measured by the gyro */
#define MAG_CAL_MIN_BINS           30      /* of 32 slices around the fitted centre */
#define MAG_CAL_MAX_AXIS_RATIO     2.0f    /* ellipse long/short axis */
#define MAG_CAL_MAX_RMS            0.05f   /* spread of |corrected| around the circle */
#define MAG_CAL_RADIUS_MIN_LSB     800.0f  /* horizontal field, raw QMC5883L counts */
#define MAG_CAL_RADIUS_MAX_LSB     20000.0f
#define MAG_CAL_GYRO_SCALE_MIN     0.90f   /* compass turn / gyro turn */
#define MAG_CAL_GYRO_SCALE_MAX     1.10f
#define MAG_CAL_MAX_GYRO_DEV_DEG   5.0f    /* worst heading disagreement with the gyro */

typedef struct {
    float center[2];   /* hard iron, raw counts (x, y) */
    float soft[4];     /* row-major 2x2 soft-iron correction */
    float radius;      /* corrected horizontal field strength, raw counts */
} mag_cal_2d_t;

void mag_cal_identity(mag_cal_2d_t *cal);
/* Finite, positive-definite and inside sane bounds -- safe to apply. */
bool mag_cal_valid(const mag_cal_2d_t *cal);
void mag_cal_apply(const mag_cal_2d_t *cal, float mx, float my,
                   float *out_x, float *out_y);
/* atan2(y, x) in degrees, wrapped to [0, 360). */
float mag_cal_heading_deg(float x, float y);

/* ---- Step 1: hold still ------------------------------------------------ */

typedef enum {
    MAG_STILL_WAITING,      /* moving, or not enough samples yet */
    MAG_STILL_COLLECTING,   /* still; averaging */
    MAG_STILL_DONE,
} mag_still_state_t;

typedef struct {
    int window;             /* samples per stillness decision */
    int collect;            /* still samples to average */
    float accel_std_max;    /* raw counts, per axis */
    float gyro_std_max;     /* raw counts, per axis */
    int n_win;
    double win_a[3], win_aa[3], win_g[3], win_gg[3];
    int n_col;
    double col_a[3], col_g[3];
    mag_still_state_t state;
} mag_still_t;

void mag_still_init(mag_still_t *s, int window, int collect,
                    float accel_std_max, float gyro_std_max);
mag_still_state_t mag_still_add(mag_still_t *s, const int16_t accel[3],
                                const int16_t gyro[3]);
float mag_still_progress(const mag_still_t *s);   /* 0..1 */
/* Means over the still samples.  Valid once DONE. */
void mag_still_result(const mag_still_t *s, float gyro_mean[3],
                      float accel_mean[3]);

/* ---- Step 2: spin flat ------------------------------------------------- */

typedef struct {
    float *x, *y, *turn;    /* caller-owned buffers, cap entries each */
    int cap;
    int n;
    int keep_every;         /* thinning factor once the buffer has filled */
    int skip;
    float min[2], max[2];
    float turn_deg;         /* gyro-integrated rotation so far, signed */
    int total;              /* samples offered */
} mag_circle_t;

void mag_circle_init(mag_circle_t *c, float *x, float *y, float *turn, int cap);
/* yaw_rate_dps: bias-corrected gyro rate about the vertical, + = left.
 * Pass NAN for mx/my when the compass reading is missing: the gyro rotation
 * is still counted, nothing is stored. */
void mag_circle_add(mag_circle_t *c, float mx, float my,
                    float yaw_rate_dps, float dt_s);
/* Slices covered around the current min/max centre (live progress). */
uint32_t mag_circle_live_mask(const mag_circle_t *c);
bool mag_circle_complete(const mag_circle_t *c);

/* ---- Fit and judge ----------------------------------------------------- */

typedef enum {
    MAG_CAL_PASS = 0,
    MAG_CAL_FAIL_TOO_FEW,       /* not enough samples */
    MAG_CAL_FAIL_FIT,           /* points do not form an ellipse */
    MAG_CAL_FAIL_DISTORTED,     /* ellipse too squashed: metal too close */
    MAG_CAL_FAIL_NOISY,         /* readings scattered off the ellipse */
    MAG_CAL_FAIL_FIELD,         /* field strength not Earth-like */
    MAG_CAL_FAIL_COVERAGE,      /* part of the circle missing */
    MAG_CAL_FAIL_GYRO,          /* compass turn disagrees with the gyro */
} mag_cal_verdict_t;

typedef struct {
    mag_cal_2d_t cal;
    float axis_ratio;       /* long / short axis of the raw ellipse */
    float rms;              /* relative spread of |corrected| around radius */
    uint32_t mask;          /* slices covered around the fitted centre */
    int bins;
    float gyro_scale;       /* compass turn / gyro turn (least squares) */
    float gyro_dev_deg;     /* worst heading disagreement after that fit */
} mag_fit_t;

/* Fits the ellipse and fills every metric.  Returns PASS or the first gate
 * that failed; `out` is filled as far as the fit got. */
mag_cal_verdict_t mag_cal_fit_and_judge(const float *x, const float *y,
                                        const float *turn, int n,
                                        mag_fit_t *out);
const char *mag_cal_verdict_text(mag_cal_verdict_t v);
