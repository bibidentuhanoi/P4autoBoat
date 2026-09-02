#pragma once

#include <stdbool.h>

/* Rudder yaw-rate stabilizer -- pure, no ESP dependencies.
 *
 * THE SIGN CONVENTION, MEASURED ON THE BOAT (dataout/RUD_T20_*.csv, 21 runs):
 *
 *     steering command -1  =  1805 us  =  physical LEFT   ->  POSITIVE yaw
 *     steering command +1  =  1195 us  =  physical RIGHT  ->  NEGATIVE yaw
 *
 * Confirmed, not assumed: commanding -0.80 gave a median +2.30 deg/s over
 * seven runs, and +0.80 gave -1.82 deg/s over nine. Rudder and yaw carry
 * OPPOSITE signs.
 *
 * That is what the previous version of this file got wrong. It computed
 *
 *     rudder = k_r * (rate_command - yaw_filt)
 *
 * treating stick sign and yaw sign as the same quantity. Ask it for +2 deg/s
 * from rest and it commanded POSITIVE rudder -- physical RIGHT -- which drives
 * yaw further NEGATIVE, which grows the error, which commands more right
 * rudder. Positive feedback, hard over in a couple of seconds. The loop was
 * never enabled (CONFIG_STABILITY_SAS_ENABLE defaulted n), which is the only
 * reason it never did that on the water.
 *
 * The corrected law, with directional feedforward:
 *
 *     r      = -stick * r_max_dps          physical yaw-rate TARGET
 *     ff     = (r >= 0) ? ff_left : ff_right
 *     rudder = -ff * r  -  k_p * (r - y)
 *
 * Both terms carry the leading minus, for the same physical reason: to make
 * the boat rotate one way you push the rudder the other. Then cap, then slew.
 *
 * Two feedforward gains because the hull is NOT symmetric -- the same |0.80|
 * of rudder yields a different rate each way, so one gain would under-drive
 * one direction and over-drive the other.
 *
 * No I and no D. This is the first on-water experiment with the loop actually
 * live; adding integral wind-up or derivative noise-gain before P has been
 * seen to work would make a bad result impossible to attribute.
 */

typedef struct {
    float r_max_dps;    /* |stick| = 1 commands this yaw-rate MAGNITUDE (deg/s) */
    float k_p;          /* rudder per (deg/s) of rate error */
    float ff_left;      /* rudder per (deg/s) for POSITIVE yaw targets (left) */
    float ff_right;     /* rudder per (deg/s) for NEGATIVE yaw targets (right) */
    float yaw_tau_s;    /* low-pass TIME CONSTANT (seconds) on measured yaw --
                         * not a per-call blend fraction, so the effective
                         * filter does not drift if the tick rate changes */
    float out_cap;      /* max |rudder| the loop may command (0..1) */
    float slew_per_s;   /* max |change in rudder| per second; <= 0 disables.
                         * The servo cannot step instantly and a step command
                         * just slams it; this also keeps a filter transient
                         * from becoming a bang-bang input. */
} stab_cfg_t;

typedef struct {
    float yaw_filt;
    float rudder;       /* last commanded value, for the slew limiter */
    bool initialized;
} stab_state_t;

/* Optional diagnostic breakdown of one update. Everything the CSV and the
 * telemetry need, so neither has to recompute the controller and risk
 * disagreeing with it. */
typedef struct {
    float target_dps;   /* the physical yaw-rate target the stick asked for */
    float yaw_filt;     /* filtered measured yaw */
    float ff_term;      /* feedforward contribution, before cap/slew */
    float p_term;       /* proportional contribution, before cap/slew */
    float raw;          /* ff_term + p_term, before cap/slew */
    bool saturated;     /* the CAP bound this output (not the slew limiter) */
    bool slewed;        /* the slew limiter bound this output */
} stab_debug_t;

void stab_reset(stab_state_t *state);

/* Physical yaw-rate target for a stick demand. Exposed because the assisted
 * pool tests command a target directly and must agree with the loop about
 * what "+2 deg/s" means. */
float stab_target_dps(const stab_cfg_t *cfg, float steer_cmd_norm);

/* dt_s is real elapsed seconds since the previous call for this state
 * (measured and clamped by the caller). Pass 0.0f on the first call after a
 * reset to snap the filter straight to the measurement instead of blending
 * from zero. steer_cmd_norm is the pilot's [-1, 1] demand in the CANONICAL
 * steering convention (-1 = left). `dbg` may be NULL.
 *
 * Returns the rudder command, capped and slew-limited. */
float stab_rudder_update(stab_state_t *state, const stab_cfg_t *cfg,
                         float dt_s, float steer_cmd_norm, float yaw_rate_dps,
                         stab_debug_t *dbg);

/* Same loop, given the physical yaw-rate target DIRECTLY in deg/s rather than
 * as a stick position. This is what an explicit SteerRateCommand feeds: a rate
 * is what the controller actually wants, and passing one avoids a round trip
 * through a stick value that means something else entirely when the loop is
 * off. stab_rudder_update() is this with the stick conversion in front. */
float stab_rudder_update_dps(stab_state_t *state, const stab_cfg_t *cfg,
                             float dt_s, float target_dps, float yaw_rate_dps,
                             stab_debug_t *dbg);
