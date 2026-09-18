#pragma once

#include <stdbool.h>

/* Pure cascaded heading / yaw-rate controller.  No ESP-IDF dependencies. */

typedef struct {
    float yaw_tau_s;
    float rate_kp;
    float rate_ki;
    float heading_tau_s;
    float heading_kp;
    float max_yaw_target_dps;
    float min_throttle;
    float steering_deadband;
    float recapture_delay_s;
} yaw_heading_cfg_t;

typedef struct {
    float dt_s;
    float yaw_rate_dps;
    float heading_deg;
    float throttle;
    float feedforward_c;
    float steering;
    bool enabled;
    bool driving;
    bool gyro_fresh;
    bool heading_valid;
    bool base_capture_now;
} yaw_heading_input_t;

typedef struct {
    bool active;
    bool heading_hold;
    bool saturated;
    float heading_target_deg;
    float heading_error_deg;
    float yaw_target_dps;
    float yaw_filt_dps;
    float rate_error_dps;
    float p_term;
    float i_term;
    float dynamic_c;
    float effective_c;
    float c_limit;
} yaw_heading_output_t;

typedef struct {
    bool initialized;
    bool heading_hold;
    bool steering_suspended;
    float yaw_filt;
    float heading_target;
    float heading_error_filt;
    float integral;
    float recapture_elapsed_s;
} yaw_heading_control_t;

void yaw_heading_control_init(yaw_heading_control_t *ctl);
void yaw_heading_control_reset(yaw_heading_control_t *ctl);

/* Update once for each fresh fusion sample. */
yaw_heading_output_t yaw_heading_control_update(
    yaw_heading_control_t *ctl,
    const yaw_heading_cfg_t *cfg,
    const yaw_heading_input_t *in);

float yaw_heading_wrap_180(float angle_deg);

