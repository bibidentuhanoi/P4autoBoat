#pragma once

/* Bench throttle-mismatch test -- pure state machine, no ESP dependencies.
 *
 * The BOAT owns the whole run: it drives the profile and buffers every sample
 * itself, so a radio dropout cannot spoil the measurement. The link is only
 * used to press the button. motor_control writes the buffer to the SD card
 * when the run finishes.
 *
 * Profile (wall clock, never sample-driven -- data gathering is NEVER cut
 * short by missing data):
 *   baseline  motors OFF   gyro drift + initial state
 *   run       the split    the actual test
 *   coast     motors OFF   still recording; the spin-down shows how fast the
 *                          boat stops turning
 */

#include <stdbool.h>
#include <stdint.h>

/* 6 s at the 100 Hz control tick -- comfortably covers the 4.5 s profile. */
#define BENCH_MAX_SAMPLES 600u

typedef enum {
    BENCH_IDLE = 0,
    BENCH_BASELINE = 1,
    BENCH_RUN = 2,
    BENCH_COAST = 3,
    BENCH_SAVED = 4,
    BENCH_FAILED = 5,
} bench_state_t;

typedef enum {
    BENCH_KIND_BASE = 0,   /* both equal -- any turn IS the mismatch */
    BENCH_KIND_LEFT = 1,   /* left stronger, right weaker */
    BENCH_KIND_RIGHT = 2,  /* right stronger, left weaker */
} bench_kind_t;

typedef struct {
    int64_t baseline_us;
    int64_t run_us;
    int64_t coast_us;
    float   max_yaw_dps;   /* safety abort only; 0 disables */
} bench_cfg_t;

/* What the fast P assist was doing on this sample. Recorded so an A/B run is
 * self-describing: the file says whether P was on, what it saw, what it did
 * and whether it was against its cap -- none of which is recoverable from the
 * motor commands alone. NULL to bench_step means "no assist compiled/active",
 * which records as p_on=0 and zeroes. */
typedef struct {
    float yaw_filt;      /* the P loop's own fast-filtered yaw (deg/s) */
    float correction;    /* c units, already capped */
    float learned_c;     /* the I learner's c, before P is added */
    bool on;
    bool at_cap;
} bench_assist_t;

typedef struct {
    float t_s;
    float yaw_rate_dps;
    float left;
    float right;
    float p_yaw;         /* P's filtered yaw at this sample */
    float p_corr;        /* P's contribution, in c units */
    float c_learn;       /* the I learner's own c, logged directly rather than
                          * derived: c - p_corr only holds while neither hits a
                          * clamp, and both can. */
    uint8_t p_flags;     /* bit0 = P on, bit1 = at cap */
    float c;               /* the trim in force AT THIS SAMPLE, as a fraction
                            * of throttle. Recorded per sample, not per run:
                            * during a BASE run the learner is live, so this is
                            * the whole record of it working. */
} bench_sample_t;

typedef struct {
    bench_state_t state;
    bench_kind_t  kind;
    float base, delta;
    int64_t start_us;
    float elapsed_s;
    bench_sample_t samples[BENCH_MAX_SAMPLES];
    uint16_t count;
    bool overflow;         /* ran out of buffer; earlier samples are still good */
    /* Mean yaw over the motors-off baseline: the gyro's zero PLUS whatever the
     * hands holding the boat are doing. Measured fresh every run because the
     * second part changes every time the boat is picked up. */
    float baseline_sum;
    uint16_t baseline_n;
} bench_t;

typedef struct {
    bool active;           /* the run owns the ESCs this tick */
    float left_cmd, right_cmd;
    bool finished;         /* true on the single tick the run completes */
    bool aborted;
    const char *reason;
} bench_out_t;

void bench_init(bench_t *b);
bool bench_start(bench_t *b, bench_kind_t kind, float base, float delta,
                 int64_t now_us);
void bench_abort(bench_t *b);

/* One control tick. `armed` false aborts (the motors must stop); everything
 * else runs to completion on the clock.
 *
 * `trim` is the stored L/R correction, applied here so the commands that are
 * RECORDED are the ones actually sent. Recording the pre-trim values instead
 * would make every later analysis read the split as zero while the boat was
 * really running trimmed. */
bench_out_t bench_step(bench_t *b, const bench_cfg_t *cfg, int64_t now_us,
                       float yaw_rate_dps, bool armed, float trim,
                       const bench_assist_t *assist);

#define BENCH_P_ON     0x01u
#define BENCH_P_AT_CAP 0x02u

/* (left, right) commands for a kind -- clamped at 0, the jets never reverse. */
void bench_commands(bench_kind_t kind, float base, float delta,
                    float *left, float *right);

/* The motors-off zero for THIS run, or 0 before enough of it has been
 * measured. Subtracting it is what separates "the motors are unbalanced" from
 * "the person holding the boat is turning it" -- on a hand-held bench those
 * are the same size, and without this the learner cheerfully corrects the
 * hands. */
float bench_baseline_yaw(const bench_t *b);

/* Below this many baseline samples the zero is not worth trusting, so nothing
 * is subtracted. 0.5 s of a 50 Hz gyro is ~25. */
#define BENCH_BASELINE_MIN_N 10u
