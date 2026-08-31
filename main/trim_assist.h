#pragma once

/* TEMPORARY proportional yaw correction, added on top of the learned trim.
 *
 * The learner (trim_learn) is a slow integrator: it finds the boat's permanent
 * motor imbalance and holds it. This is the fast half -- it reacts to yaw the
 * learner has not had time to absorb, and it is deliberately FORGOTTEN the
 * moment conditions stop being valid. Nothing here is ever learned or stored.
 *
 *     effective_c = clamp(learned_c + p_correction, c_min, c_max)
 *     p_correction = clamp(-kp * yaw_filtered, -cap, +cap)
 *
 * Sign: yaw NEGATIVE means the boat is turning the way too-little-trim turns
 * it, so -kp*yaw is POSITIVE, which raises c, which gives the RIGHT motor
 * more. That matches 221 recorded runs where too little trim read negative.
 *
 * This is the same maths as stability_control.c's P path, kept as separate
 * state and config on purpose: that one drives the RUDDER, this one drives
 * DIFFERENTIAL THRUST. Sharing state between them would couple two loops that
 * must stay independent.
 *
 * Filter speed is set by the HULL, not by taste. The measured yaw response to
 * a motor step is a 1.10 s time constant with ~0.3 s of dead time (untrimmed
 * T40 runs, n>=8 per sample point). A filter faster than that chases gyro
 * noise the boat cannot physically follow: at tau=0.15 s the correction sits
 * at its cap 30% of the time on noise alone.
 *
 * The SHIPPED values live in Kconfig and are kp=0.035, tau=0.25 s, cap=0.075
 * (CONFIG_STABILITY_PASSIST_*); see their help text for how each was picked.
 * tau landed at 0.25 rather than nearer the hull constant only because a pool
 * run is capped at 3 s, and P's output reaches the yaw a filter-length plus
 * 1.10 s later -- 0.5 s left too little of the run to act in. The literal
 * defaults in motor_control.c's parse_cfg_float() calls are a defensive
 * fallback for an unparseable config string, NOT the shipped tuning.
 *
 * MEASURED LIMIT (THEPTESTV3, 10 runs per arm at T40, c reset to 0.120 every
 * run): loop gain is kp * plant = 0.035 * 16.1 = 0.56, and a proportional term
 * can only remove gain/(1+gain) = 36% of a CONSTANT lean. 36% of the 0.47
 * deg/s bias that mistrim produced is below the 0.25 deg/s run-to-run scatter,
 * so P did not improve the whole-run heading error there (+2%, n.s.) even
 * though its correction matched theory to 15% (+0.0183 measured vs +0.0159
 * predicted). A steady mismatch is the integrator's job; P is for the fast
 * disturbances a 3 s flat-water run does not contain. Hence: default OFF.
 */

#include <stdbool.h>

typedef struct {
    float kp;      /* c per (deg/s) of yaw. Shipped 0.035 -- ~56% of unity loop
                    * gain, since the measured plant is 16.1 deg/s per unit c. */
    float tau_s;   /* fast low-pass on measured yaw (seconds). Shipped 0.25. */
    float cap;     /* max |correction|, in c units. Shipped 0.075, which moves
                    * each motor by at most 3.0 percentage points at T40. */
} trim_assist_cfg_t;

typedef struct {
    float yaw_filt;
    bool initialized;
    float correction;   /* last value applied, in c units */
    bool at_cap;        /* the cap bound the last correction */
} trim_assist_t;

/* Forget everything. Called whenever the correction must not persist:
 * disabled, steering, low throttle, stale gyro, impact rejected, a bench
 * LEFT/RIGHT or calibration owning the motors, or the learner being reset. */
void trim_assist_reset(trim_assist_t *s);

/* One fusion sample. `gate` false resets and returns 0 -- there is no decay,
 * no memory and no carry-over, which is what makes this correction temporary
 * by construction rather than by convention.
 *
 * Returns the correction in c units, already capped. */
float trim_assist_update(trim_assist_t *s, const trim_assist_cfg_t *cfg,
                         float dt_s, float yaw_rate_dps, bool gate);

/* learned_c + correction, held inside the learner's own bounds. The learned
 * value itself is never modified -- only this sum is what the mixer sees. */
float trim_assist_effective_c(float learned_c, float correction,
                              float c_min, float c_max);
