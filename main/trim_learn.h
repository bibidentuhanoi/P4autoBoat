#pragma once

/* On-the-fly ESC trim learner -- pure, no ESP dependencies.
 *
 * Learns ONE dimensionless number `c`, the differential split expressed as a
 * FRACTION of throttle:
 *
 *     split = c * throttle
 *     left  = throttle - split      right = throttle + split
 *
 * Measured on this boat (~200 bench runs): c ~ 0.20 across 10-45% throttle.
 * A fixed absolute trim was wrong precisely because it does not scale -- 7.6%
 * is right at 40% throttle and nearly 4x too much at 10%.
 *
 * It is an INTEGRATOR only (the "I" of PID). No P, no D: those would make the
 * motor balance twitch on every wave. P/D belong to the rudder loop.
 *
 * Direction-only ("half-I"): measured in-run gyro noise (sd ~2.3 deg/s) is the
 * same size as the bias being corrected (2-8 deg/s), so WHICH WAY is
 * trustworthy and HOW MUCH is not. A deadband on the filtered rate stops it
 * random-walking once the boat is already straight.
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float c_init;          /* seeded from the bench measurement (~0.20) */
    float c_min, c_max;    /* clamp; a hit is treated as a FAULT, not a limit */
    float deadband_dps;    /* below this filtered drift, change nothing */
    float step_per_s;      /* how fast c may move (units of c per second) */
    float yaw_tau_s;       /* low-pass time constant on the measured yaw rate */
    float min_throttle;    /* freeze below this -- dead-band floors dominate */
    float reject_dps;      /* above this the boat is being hit, not mistrimmed;
                            * 0 disables. See TRIM_LEARN_REJECT_DPS. */
} trim_learn_cfg_t;

typedef struct {
    float c;
    float yaw_filt;
    bool initialized;
    uint32_t last_seq;     /* fusion sample already consumed */
    bool faulted;          /* clamp was hit: stop adapting, flag it */
} trim_learn_t;

/* Longest gap between fusion samples that still counts as continuous. Fusion
 * publishes at 50 Hz (20 ms), so this allows a dozen missed samples. A gap
 * longer than this means the learner was not running -- frozen through a bench
 * run or an ESC calibration -- and the elapsed wall time is NOT integration
 * time. Without this bound the step (step_per_s * dt) scales with however long
 * the pause lasted: a 30 s calibration would move c by 0.15 in a single
 * sample, slamming it into a clamp and latching the fault. */
#define TRIM_LEARN_MAX_DT_S 0.25f

/* Yaw above this is not a trim error -- it is the hull hitting something.
 * In a 1.2 m pool a 50 cm boat reaches the wall in a couple of seconds and
 * then bounces, and the bounce is far larger than the imbalance being
 * measured. Across 63 recorded runs, 12 carried a collision (first spike
 * 1.0-6.8 s in, peaks 10-33 deg/s) while the 51 clean ones peaked at a median
 * of 6.8 and a maximum of 10.2. So 10 deg/s separates them: it sits above 98%
 * of clean runs' worst sample and below every collision.
 *
 * A trim error of the size actually seen here moves the boat 0.1-2 deg/s. */
#define TRIM_LEARN_REJECT_DPS 10.0f

void trim_learn_init(trim_learn_t *s, const trim_learn_cfg_t *cfg);

/* Put the learner at a known c and wipe what it thought it knew: the yaw
 * estimate is dropped (it described a different trim) and any latched clamp
 * fault is cleared.
 *
 * This exists for ONE experiment: start low, start high, and see whether both
 * ends walk to the same place. Nothing else may call it -- c carrying over
 * between runs is what makes convergence observable at all, so a reset that
 * fired on every run would destroy the very thing being measured.
 *
 * Refuses, changing nothing, unless c is finite and inside [c_min, c_max].
 * last_seq is deliberately left alone so the sample in flight is not
 * reprocessed. Returns true if the reset was applied. */
bool trim_learn_reset(trim_learn_t *s, const trim_learn_cfg_t *cfg, float c);

/* Feed one FUSION SAMPLE. Returns true when `c` actually changed, so the caller
 * can push a new ESC command -- the existing drive path only refreshes the ESCs
 * when a pilot command arrives, so a silently-changing c would never reach the
 * motors.
 *
 * `seq` is FusionResult.sequence: a repeated sequence is ignored, because the
 * control loop runs at 100 Hz while fusion updates at 50 Hz and integrating
 * every tick would count each sample twice. */
bool trim_learn_update(trim_learn_t *s, const trim_learn_cfg_t *cfg,
                       uint32_t seq, float dt_s, float yaw_rate_dps,
                       float throttle, bool steering, bool healthy);

/* split = c * throttle, 0 when stopped. */
float trim_learn_split(const trim_learn_t *s, float throttle);
