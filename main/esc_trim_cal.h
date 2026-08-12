#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esc_trim.h"

typedef enum {
    ETC_IDLE, ETC_MEASURE_NOISE, ETC_RAMP, ETC_SETTLE, ETC_DONE, ETC_ABORTED
} etc_state_t;

typedef struct {
    float ki;
    float trim_clamp;
    float accept_k;
    float excessive_yaw_dps;
    float min_speed_mps;
    uint32_t window_ticks;
    uint32_t noise_ticks;
    int64_t settle_in_us;
    int64_t level_timeout_us;
    uint8_t level_count;
    float levels[ESC_TRIM_MAX_POINTS];
    bool average_into_existing;
} etc_cfg_t;

typedef struct {
    etc_state_t state;
    uint8_t level_idx;
    float trim_diff;
    float b0, sigma;
    double acc_sum, acc_sumsq;
    uint32_t acc_n;
    int64_t phase_start_us;
    int64_t last_tick_us;
    EscTrimPoint out_table[ESC_TRIM_MAX_POINTS];
    uint8_t out_count;
} etc_t;

typedef struct {
    bool active;
    float left_cmd;
    float right_cmd;
    bool done;
    bool aborted;
    const char *reason;
} etc_out_t;

void esc_trim_cal_init(etc_t *s, const etc_cfg_t *cfg);
void esc_trim_cal_start(etc_t *s, const etc_cfg_t *cfg);
void esc_trim_cal_abort(etc_t *s);
etc_out_t esc_trim_cal_step(etc_t *s, const etc_cfg_t *cfg, int64_t now_us,
                            float yaw_rate_dps, float gps_speed_mps,
                            bool armed, bool link_alive, bool imu_ok,
                            bool manual_override);
