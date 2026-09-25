#pragma once

#include <stdbool.h>
#include <stdint.h>

/* beta: GPS course minus fused compass heading = compass error + crab angle.
 *
 *   OUTBOUND  equal-weight circular mean of the samples that pass the gates.
 *   RETURN    starts from the outbound value and follows new samples with a
 *             first-order (circular) filter: the crab from wind or current
 *             flips at the turnaround and hard-iron compass error changes with
 *             heading (2026-09-20: +21 deg at heading 122, +53 deg at 225).
 *
 * No ESP-IDF dependencies. */

typedef struct {
    float min_speed_mps;       /* GPS course is meaningless when crawling */
    float max_course_err_deg;  /* asin(sAcc / speed) must be below this */
    float max_yaw_dps;         /* course lags heading while turning */
    uint32_t min_samples;      /* outbound mean needs at least this many */
    float min_travel_m;        /* ...spread over at least this much travel */
    float max_std_deg;         /* ...and this consistent */
    float clamp_deg;           /* |beta| is never used beyond this */
    float bad_deg;             /* |outbound beta| above this = compass bad */
    float follow_tau_s;        /* RETURN filter time constant */
} course_err_cfg_t;

typedef struct {
    double sum_sin;
    double sum_cos;
    uint32_t n;
    float travel_m;
    bool have_last;
    float last_e;
    float last_n;
} course_err_mean_t;

typedef struct {
    bool valid;
    bool compass_bad;
    float mean_deg;
    float std_deg;
    uint32_t n;
    float travel_m;
} course_err_result_t;

course_err_cfg_t course_err_cfg_default(void);

/* Gate for one GPS fix. sAcc <= 0 means unknown and is rejected: a correction
 * of unknown quality is worse than none (the LOS still gets the boat home). */
bool course_err_sample_ok(const course_err_cfg_t *cfg, float speed_mps,
                          float speed_acc_mps, float yaw_dps);
/* wrap180(course - heading) */
float course_err_sample(float course_deg, float heading_deg);

void course_err_mean_reset(course_err_mean_t *m);
/* e/n: the boat's local position for this sample (for the travel check). */
void course_err_mean_add(course_err_mean_t *m, float sample_deg, float e, float n);
course_err_result_t course_err_mean_result(const course_err_mean_t *m,
                                           const course_err_cfg_t *cfg);

/* RETURN: move `current_deg` toward `sample_deg` along the shorter way round,
 * by the first-order fraction dt/(tau+dt); clamped to +-clamp_deg. */
float course_err_follow(float current_deg, float sample_deg, float dt_s,
                        const course_err_cfg_t *cfg);
