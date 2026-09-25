/* Out-and-back mission simulator (spec: docs/superpowers/specs/
 * 2026-09-24-out-and-back-mission-design.md, section 9.2).
 *
 * Build (from the repository root):
 *   cc -std=c11 -O2 -I main tools/mission_sim/mission_sim.c main/mission.c \
 *      main/nav_geo.c main/course_error.c main/path_follow.c main/planner.c \
 *      main/auto_drive.c main/esc_trim.c main/yaw_heading_control.c -lm -o mission_sim
 *
 * The REAL mission, AUTO owner, heading hold and mixer (the main/ sources) fly
 * a boat model fitted to the 2026-09-20/21 lake runs (the stable current
 * boat; tools/mission_sim/model_from_dataout.py extracts the reference numbers
 * and the replay segments into model_ref.json, and tests/test_mission_sim.py
 * checks this model against them):
 *   yaw:   r' = (S*tanh(K*u/S) + wave - r)/tau, u = c(t - delay) - c_bal,
 *          S = S_left for u >= 0 (left turn), S_right otherwise;
 *          c = (R - L)/(R + L) as written to the ESCs;
 *          c_bal = the c that goes straight: per run + a slow random walk
 *          (wind and waves move it; the P-ON runs held c from -0.33 to +0.48);
 *          waves: slow yaw wander + fast chop;
 *   speed: table from the straight runs, 5-6 s to speed, glide half-life ~3.3 s;
 *   compass: constant + heading-dependent + throttle-dependent error;
 *   GPS:   white noise, slow wander, 50-200 ms lag, dropouts, jumps,
 *          reported speed accuracy.
 * Firmware timing: control 100 Hz, fusion 50 Hz, autonomy 20 Hz, GPS 10 Hz.
 * The control-task glue (AUTO owner -> heading hold -> mixer) is replicated in
 * the control step below.
 *
 * Modes:
 *   mission_sim [--runs N] [--seed S] [options]      Monte Carlo summary (JSON)
 *   mission_sim --catalogue [--runs N]               every scenario, checked
 *   mission_sim --scenario NAME [--runs N] [-v]      one scenario
 *   mission_sim --sweep NAME|all [--runs N]          one parameter at a time
 *   mission_sim --plant [--runs N]                   model metrics, JSON lines
 *   mission_sim --replay FILE [--runs N]             recorded commands -> model yaw
 *   mission_sim --list                               scenario names
 * Everything is judged on the TRUE position. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_drive.h"
#include "mission.h"
#include "yaw_heading_control.h"

#define LAT0 21.0417483
#define LON0 105.8862132
#define DT 0.01f
#define D2R 0.017453292519943295f
#define HIST 32            /* GPS lag history, 10 ms steps: up to 310 ms */
#define CHIST 128          /* actuator delay history, 10 ms steps */

/* ---- random numbers (reproducible per seed) --------------------------- */
static uint64_t rng_state;
static double urand(void)
{
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
    return (double)(rng_state >> 11) * (1.0 / 9007199254740992.0);
}
static float uni(float a, float b) { return a + (b - a) * (float)urand(); }
static float gauss(void)
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return (float)(sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2));
}
static float sign_rand(void) { return urand() < 0.5 ? -1.0f : 1.0f; }
static void seed_rng(uint64_t seed)
{
    rng_state = seed * 2654435761ull + 88172645463325252ull;
    for (int i = 0; i < 10; ++i) (void)urand();
}

/* ---- the envelope (random ranges per run) ------------------------------ */
typedef struct {
    /* boat (fitted to the 09-20/21 runs; see the plant check) */
    float K_min, K_max;                  /* small-signal yaw gain, deg/s per unit c */
    float SL_min, SL_max, SR_min, SR_max;/* saturation, left / right turns, deg/s */
    float tau_min, tau_max;              /* yaw lag, s */
    float delay_min, delay_max;          /* command -> yaw delay, s */
    float cbal_min, cbal_max;            /* per-run straight-running c */
    float cbal_hdg_max;                  /* + up to this * sin(heading - wind): 09-20 swung
                                            -0.34 (hdg 122) .. +0.40 (hdg 226); 09-21 ~0 */
    float cbal_walk;                     /* + a slow random walk (sigma, 20 s) */
    float wave_min, wave_max;            /* slow yaw wander, deg/s rms (1.5 s) */
    float chop_min, chop_max;            /* fast yaw chop, deg/s rms (0.15 s) */
    /* water */
    float drift_min, drift_max;          /* magnitude uniform in [min, max], any direction */
    /* compass */
    float bias_max; int bias_fixed;      /* fixed: |bias| = bias_max, random sign */
    float hdg_err_max, thr_err_max;
    /* GPS */
    float gps_white, gps_wander, gps_drop_per_s, gps_jump_per_s;
    float gps_lag_min_s, gps_lag_max_s;
    float sacc_mean;                     /* reported speed accuracy (m/s) */
    float vel_noise;                     /* actual GPS velocity noise per axis (m/s) */
    /* radio */
    float link_cut_start, link_cut_len;
    mission_settings_t settings;
} envelope_t;

/* ---- one scenario = envelope tweaks + scripted faults + expected outcome - */
typedef enum {
    EXPECT_IN_ZONE,       /* DONE by an arrival rule, TRULY within radius + 0.5 m */
    EXPECT_DONE_OUT,      /* stage 1 ended at the out distance */
    EXPECT_DONE_TURNED,   /* stage 2 ended facing home */
    EXPECT_ABORT,         /* aborted with abort_reason, within max_latency_s of the fault */
    EXPECT_SAFE,          /* beyond what the boat can do: DONE, or the spin guard, or it
                             waits (no spin) with the laptop's no-progress warning up */
    EXPECT_DONE_ANY,
} expect_t;

typedef struct scenario scenario_t;
struct scenario {
    const char *name;
    const char *what;
    expect_t expect;
    uint8_t abort_reason;
    float max_latency_s;           /* EXPECT_ABORT: fault -> ABORTED at most this */
    float min_pass;                /* fraction of runs that must meet the expectation */
    int event_state;               /* 0: event times count from START; else from the
                                      first entry into this mission state */
    float stop_at, manual_at, disarm_at, other_owner_at, rail_cut_at;
    float gps_drop_at, gps_drop_len;
    float gps_jump_at, gps_jump_len, gps_jump_m; int gps_jump_toward_home;
    float gps_frozen_at, gps_frozen_len;
    float gps_sats_at, gps_sats_len; int gps_sats;
    float gps_2d_at, gps_2d_len;
    float compass_step_at, compass_step_deg;
    float compass_thr_amp;
    float compass_hdg_amp;
    float gyro_scale, gyro_bias;
    float imu_frozen_at, imu_frozen_len;
    float jet_fault_at, jet_right_factor;
    float gust_amp, gust_period;
    int current_fixed; float current_mps, current_dir;
    float stall_at, stall_len;
    float start_drift;
    float home_jump_m;
    float link_flap_period;
    int dry_run_walk;
    int start_hdg_fixed; float start_hdg_min, start_hdg_max;
    int start_spam;                /* START repeated (same id) for 3 s, a new id at 30 s */
    float restart_at;              /* a second START (new id) this long after the first */
    int need_motors_never;
    int need_compass_bad;
    int need_beta_invalid;
    int need_runs;
    void (*env_fn)(envelope_t *e);
};

static scenario_t scenario_defaults(void)
{
    scenario_t s;
    memset(&s, 0, sizeof(s));
    s.expect = EXPECT_IN_ZONE; s.min_pass = 0.95f; s.max_latency_s = 3.0f;
    s.stop_at = s.manual_at = s.disarm_at = s.other_owner_at = s.rail_cut_at = -1.0f;
    s.gps_drop_at = s.gps_jump_at = s.gps_frozen_at = s.gps_sats_at = s.gps_2d_at = -1.0f;
    s.compass_step_at = s.imu_frozen_at = s.jet_fault_at = s.stall_at = -1.0f;
    s.gyro_scale = 1.0f; s.jet_right_factor = 1.0f;
    return s;
}

/* ---- the boat ------------------------------------------------------------ */
typedef struct {
    float K, S_left, S_right, tau, delay, speed_scale, tau_v, tau_glide, floor_;
    float cbal0, cbal_amp, cbal_dir, cbal_walk;
    float drift_e, drift_n;
    float bias, hdg_amp, hdg_phase, thr_err;
    float wave_sigma, chop_sigma;
    float gyro_bias;
    float start_heading;
    int gps_lag_steps;
    float sacc_base;
} boat_params_t;

typedef struct {
    float e, n, psi, r, v, T;
    float c_hist[CHIST]; int c_i;
    float cbal;            /* the c that goes straight right now */
    float cbal_gm;         /* its random-walk part */
    float wave, chop;
    float yaw_bias;        /* replay only: the yaw rate held before a segment */
} boat_t;

static float speed_table(float T)
{
    static const float t[] = {0.0f, 0.05f, 0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.40f, 0.45f, 0.60f, 1.0f};
    static const float v[] = {0.0f, 0.02f, 0.10f, 0.17f, 0.25f, 0.29f, 0.32f, 0.38f, 0.42f, 0.50f, 0.70f};
    if (T <= 0.0f) return 0.0f;
    for (int i = 1; i < 11; ++i) {
        if (T <= t[i]) return v[i - 1] + (v[i] - v[i - 1]) * (T - t[i - 1]) / (t[i] - t[i - 1]);
    }
    return 0.70f;
}

/* Every random draw happens in the same order whatever the scenario, so a
 * seed is the same boat in every scenario (common random numbers). */
static void boat_draw(boat_params_t *p, const envelope_t *env, const scenario_t *sc)
{
    p->K = uni(env->K_min, env->K_max);
    p->S_left = uni(env->SL_min, env->SL_max);
    p->S_right = uni(env->SR_min, env->SR_max);
    p->tau = uni(env->tau_min, env->tau_max);
    p->delay = uni(env->delay_min, env->delay_max);
    p->speed_scale = uni(0.85f, 1.15f);
    p->tau_v = uni(1.8f, 3.0f);
    p->tau_glide = uni(4.0f, 5.5f);
    p->floor_ = uni(0.0f, 0.0f);   /* the speed table is per COMMANDED throttle: no second floor */
    p->cbal0 = uni(env->cbal_min, env->cbal_max);
    p->cbal_amp = uni(0.0f, env->cbal_hdg_max);
    p->cbal_dir = uni(0.0f, 360.0f);
    p->cbal_walk = env->cbal_walk;
    float dmag = uni(env->drift_min, env->drift_max);
    float ddir = uni(0.0f, 360.0f);
    if (sc && sc->current_fixed) { dmag = sc->current_mps; ddir = sc->current_dir; }
    p->drift_e = dmag * sinf(ddir * D2R); p->drift_n = dmag * cosf(ddir * D2R);
    const float bsign = sign_rand();
    const float bmag = uni(0.0f, env->bias_max);
    p->bias = env->bias_fixed ? bsign * env->bias_max : bsign * bmag;
    p->hdg_amp = uni(0.0f, (sc && sc->compass_hdg_amp > 0.0f) ? sc->compass_hdg_amp : env->hdg_err_max);
    p->hdg_phase = uni(0.0f, 360.0f);
    p->thr_err = uni(-env->thr_err_max, env->thr_err_max) + (sc ? sc->compass_thr_amp : 0.0f);
    p->wave_sigma = uni(env->wave_min, env->wave_max);
    p->chop_sigma = uni(env->chop_min, env->chop_max);
    const float gsign = sign_rand();
    p->gyro_bias = uni(-0.3f, 0.3f) + gsign * (sc ? sc->gyro_bias : 0.0f);
    const float h0 = uni(0.0f, 360.0f);
    const float h1 = uni(sc ? sc->start_hdg_min : 0.0f, sc ? sc->start_hdg_max : 0.0f);
    p->start_heading = (sc && sc->start_hdg_fixed) ? nav_wrap_360(h1) : h0;
    int lag = (int)(uni(env->gps_lag_min_s, env->gps_lag_max_s) / DT + 0.5f);
    if (lag < 0) lag = 0;
    if (lag > HIST - 2) lag = HIST - 2;
    p->gps_lag_steps = lag;
    p->sacc_base = env->sacc_mean * uni(0.7f, 1.3f);
}

static void boat_init(boat_t *b, const boat_params_t *p)
{
    memset(b, 0, sizeof(*b));
    b->psi = p->start_heading;
    b->cbal = p->cbal0 + p->cbal_amp * sinf((b->psi - p->cbal_dir) * D2R);
    for (int i = 0; i < CHIST; ++i) b->c_hist[i] = b->cbal;
}

/* One 10 ms step.  L, R: jet commands 0..1 as written to the ESCs (a jet
 * fault is applied by the caller).  walk: the operator carries the boat. */
static void boat_step(boat_t *b, const boat_params_t *p, float L, float R, float extra_ve, float extra_vn,
                      int walk)
{
    const float Lf = L > p->floor_ ? (L - p->floor_) / (1.0f - p->floor_) : 0.0f;
    const float Rf = R > p->floor_ ? (R - p->floor_) / (1.0f - p->floor_) : 0.0f;
    const float T = 0.5f * (Lf + Rf);
    b->T = T;
    /* the balance point: per run, with heading (wind), wandering */
    if (p->cbal_walk > 0.0f) {
        b->cbal_gm += (-b->cbal_gm / 20.0f) * DT + p->cbal_walk * sqrtf(2.0f * DT / 20.0f) * gauss();
    }
    b->cbal = p->cbal0 + p->cbal_amp * sinf((b->psi - p->cbal_dir) * D2R) + b->cbal_gm;
    const float c_now = (Lf + Rf) > 0.02f ? (Rf - Lf) / (Rf + Lf) : b->cbal;
    b->c_hist[b->c_i] = c_now; b->c_i = (b->c_i + 1) % CHIST;
    int lag = (int)(p->delay / DT + 0.5f);
    if (lag > CHIST - 2) lag = CHIST - 2;
    const float c_del = b->c_hist[(b->c_i - 1 - lag + 2 * CHIST) % CHIST];
    const float u = c_del - b->cbal;
    const float S = (u >= 0.0f) ? p->S_left : p->S_right;
    float r_ss = (T > 0.02f) ? S * tanhf(p->K * u / S) : 0.0f;
    if (p->wave_sigma > 0.0f) {
        b->wave += (-b->wave / 1.5f) * DT + p->wave_sigma * sqrtf(2.0f * DT / 1.5f) * gauss();
    }
    if (p->chop_sigma > 0.0f) {
        b->chop += (-b->chop / 0.15f) * DT + p->chop_sigma * sqrtf(2.0f * DT / 0.15f) * gauss();
    }
    r_ss += b->wave + b->yaw_bias;
    b->r += (r_ss - b->r) * (DT / p->tau);
    const float r_true = b->r + b->chop;
    if (!walk) {
        b->psi = nav_wrap_360(b->psi - r_true * DT);
        const float v_ss = speed_table(T) * p->speed_scale * (1.0f - 0.2f * fminf(fabsf(b->r) / 20.0f, 1.0f));
        if (T > 0.02f) b->v += (v_ss - b->v) * (DT / p->tau_v);
        else b->v -= b->v * (DT / p->tau_glide);
    }
    const float ve = b->v * sinf(b->psi * D2R) + p->drift_e + extra_ve;
    const float vn = b->v * cosf(b->psi * D2R) + p->drift_n + extra_vn;
    b->e += ve * DT; b->n += vn * DT;
}

/* the yaw rate the gyro sees (deg/s, + = left) */
static float boat_yaw(const boat_t *b) { return b->r + b->chop; }

static float wrap180(float a) { return nav_wrap_180(a); }

static const yaw_heading_cfg_t HCFG = {
    .yaw_tau_s = 0.08f, .rate_kp = 0.100f, .rate_ki = 0.080f,
    .heading_tau_s = 0.10f, .heading_kp = 1.50f, .max_yaw_target_dps = 15.0f,
    .min_throttle = 0.15f, .steering_deadband = 0.02f, .recapture_delay_s = 0.50f };
#define LEARNED_C 0.21f

typedef struct {
    int reason;          /* mission reason; -1 = never ended */
    int state;           /* final mission state */
    float t_done, d_done, d_final, closest_true;
    float turned, max_xtrack, heading_err_at_turn_end;
    int outages, false_arrival, spun, motors_after_stop, refused;
    float max_abs_turn;  /* largest |leg turn| seen (true, deg) */
    int warned;          /* the laptop's no-progress warning would be up */
    int compass_bad, beta_valid_ever, runs_started;
    float max_cmd;       /* largest jet command written by the mission */
    float latency;       /* first scripted fault -> terminal state (s); -1 = n/a */
    float beta_final;
} result_t;

static bool active_window(float t, float at, float len) { return at >= 0.0f && t >= at && t < at + len; }
static bool after(float t, float at) { return at >= 0.0f && t >= at; }

static float g_course_gate_deg = -1.0f;   /* --course-gate: experiment with the beta sample gate */
static float g_approach = -1.0f;          /* --approach: approach throttle for sweeps */

static result_t run_one(const envelope_t *env, const scenario_t *sc, uint64_t seed, FILE *trace)
{
    seed_rng(seed);
    boat_params_t p;
    boat_draw(&p, env, sc);
    boat_t b;
    boat_init(&b, &p);

    nav_origin_t o; nav_origin_init(&o, LAT0, LON0);
    mission_t m; mission_init(&m);
    mission_cfg_t cfg = mission_cfg_default();
    if (g_course_gate_deg > 0.0f) cfg.course.max_course_err_deg = g_course_gate_deg;
    auto_drive_t ad; auto_drive_init(&ad);
    yaw_heading_control_t ctl; yaw_heading_control_init(&ctl);

    float compass_step = 0.0f;
    float fused = p.start_heading + p.bias;
    float r_meas = 0.0f, last_fusion_t = 0.0f;
    yaw_heading_output_t hout; memset(&hout, 0, sizeof(hout));
    auto_setpoint_t sp; memset(&sp, 0, sizeof(sp));
    mission_output_t mout; memset(&mout, 0, sizeof(mout));
    float L = 0.0f, R = 0.0f;
    float last_fix_t = -1.0f, last_fix_seen = -1.0f;
    double fix_lat = LAT0, fix_lon = LON0;
    float fix_speed = 0.0f, fix_course = 0.0f, fix_sacc = 0.05f;
    uint8_t fix_type = 3, fix_sats = 12;
    float frozen_e = 0.0f, frozen_n = 0.0f; bool frozen_set = false;
    float hist_e[HIST] = {0}, hist_n[HIST] = {0}, hist_ve[HIST] = {0}, hist_vn[HIST] = {0}; int hi = 0;
    float gm_e = 0.0f, gm_n = 0.0f, jump_e = 0.0f, jump_n = 0.0f, jump_until = -1.0f, drop_until = -1.0f;

    result_t res; memset(&res, 0, sizeof(res));
    res.closest_true = 1e9f; res.reason = -1; res.latency = -1.0f;
    float t = 0.0f, t_done = -1.0f, t_start = -1.0f, t_event = -1.0f;
    float t_state_first[8]; for (int i = 0; i < 8; ++i) t_state_first[i] = -1.0f;
    const float t_max = 400.0f;
    int step = 0;
    bool started = false, manual_driving = false, armed = true, rail_cut = false;
    bool spam_new_sent = false, restart_issued = false, stop_to_mission = false;
    int retries = 0; float t_retry = -1.0f;
    float leg_psi_turn = 0.0f; int leg_state = -1;
    float jet_right = 1.0f;
    /* the true home: the mean true position while the mission took home */
    double th_e = 0.0, th_n = 0.0; int th_k = 0; bool in_home = false;
    float true_home_e = 0.0f, true_home_n = 0.0f;
    float p0_e = 0.0f, p0_n = 0.0f; bool have_p0 = false;
    uint32_t last_run = 0;
    /* the laptop's no-progress warning (reference rule for the tool):
     * OUTBOUND must get >= 1 m farther from home, RETURN >= 1 m nearer, within
     * every 30 s; a TURN longer than 30 s also warns */
    float np_best = 0.0f, np_t = 0.0f; int np_state = -1;

    while (t < t_max) {
        const float ts = (t_start >= 0.0f) ? t - t_start : -1.0f;
        float te = ts;
        if (sc->event_state > 0) {
            const float t0 = t_state_first[sc->event_state];
            te = t0 >= 0.0f ? t - t0 : -1.0f;
        }
        /* ---- scripted faults -------------------------------------------- */
        if (after(te, sc->jet_fault_at)) jet_right = sc->jet_right_factor;
        if (after(te, sc->compass_step_at)) compass_step = sc->compass_step_deg;
        const float gust = sc->gust_amp > 0.0f ? sc->gust_amp * sinf(6.2831853f * t / sc->gust_period) : 0.0f;
        const bool ev_stop = sc->stop_at >= 0.0f && te >= sc->stop_at && te < sc->stop_at + 0.01f;
        const bool ev_manual = sc->manual_at >= 0.0f && te >= sc->manual_at && te < sc->manual_at + 0.01f;
        const bool fault_now =
            after(te, sc->jet_fault_at) || after(te, sc->compass_step_at) || ev_stop || ev_manual ||
            after(te, sc->disarm_at) || after(te, sc->rail_cut_at) || after(te, sc->other_owner_at) ||
            active_window(te, sc->gps_drop_at, sc->gps_drop_len) ||
            active_window(te, sc->gps_jump_at, sc->gps_jump_len) ||
            active_window(te, sc->gps_frozen_at, sc->gps_frozen_len) ||
            active_window(te, sc->gps_sats_at, sc->gps_sats_len) ||
            active_window(te, sc->gps_2d_at, sc->gps_2d_len) ||
            active_window(te, sc->imu_frozen_at, sc->imu_frozen_len) ||
            active_window(te, sc->stall_at, sc->stall_len) ||
            (sc->home_jump_m > 0.0f && m.state == MISSION_HOME);
        if (fault_now && t_event < 0.0f) t_event = t;

        /* ---- plant (10 ms) ---------------------------------------------- */
        const bool walking = sc->dry_run_walk && (m.state == MISSION_OUTBOUND || m.state == MISSION_TURN ||
                                                  m.state == MISSION_RETURN);
        const float start_drift = (sc->start_drift > 0.0f && m.state <= MISSION_HOME) ? sc->start_drift : 0.0f;
        if (walking) {
            /* The operator carries the boat along the wanted heading at 0.4 m/s. */
            const float err = wrap180(mout.heading_deg - fused);
            const float rate = fmaxf(-20.0f, fminf(20.0f, 1.5f * err));
            b.r = -rate; b.chop = 0.0f; b.psi = nav_wrap_360(b.psi + rate * DT); b.v = 0.4f;
        }
        boat_step(&b, &p, L, R * jet_right, gust + start_drift, 0.0f, walking);
        const float ve = b.v * sinf(b.psi * D2R) + p.drift_e + gust + start_drift;
        const float vn = b.v * cosf(b.psi * D2R) + p.drift_n;
        hist_e[hi] = b.e; hist_n[hi] = b.n; hist_ve[hi] = ve; hist_vn[hi] = vn; hi = (hi + 1) % HIST;

        /* ---- GPS at 10 Hz, late, with scripted and random faults -------- */
        if (step % 10 == 0) {
            gm_e += (-gm_e / 60.0f) * 0.1f + env->gps_wander * sqrtf(2.0f * 0.1f / 60.0f) * gauss();
            gm_n += (-gm_n / 60.0f) * 0.1f + env->gps_wander * sqrtf(2.0f * 0.1f / 60.0f) * gauss();
            if (t > drop_until && urand() < env->gps_drop_per_s * 0.1) drop_until = t + uni(0.5f, 1.5f);
            if (t > jump_until && urand() < env->gps_jump_per_s * 0.1) {
                jump_until = t + uni(1.0f, 2.0f);
                const float a = uni(0.0f, 360.0f), mag = uni(1.0f, 2.5f);
                jump_e = mag * sinf(a * D2R); jump_n = mag * cosf(a * D2R);
            }
            const bool dropped = (t < drop_until) || active_window(te, sc->gps_drop_at, sc->gps_drop_len);
            if (!dropped) {
                const int k = (hi - 1 - p.gps_lag_steps + 2 * HIST) % HIST;
                float ge = hist_e[k] + gm_e + env->gps_white * gauss();
                float gn = hist_n[k] + gm_n + env->gps_white * gauss();
                if (t < jump_until) { ge += jump_e; gn += jump_n; }
                if (active_window(te, sc->gps_jump_at, sc->gps_jump_len)) {
                    const float dx = b.e - true_home_e, dy = b.n - true_home_n;
                    const float d = hypotf(dx, dy);
                    if (sc->gps_jump_toward_home && d > 0.1f) {
                        ge -= sc->gps_jump_m * dx / d; gn -= sc->gps_jump_m * dy / d;
                    } else { ge += sc->gps_jump_m; }
                }
                if (m.state == MISSION_HOME && sc->home_jump_m > 0.0f && (step / 10) % 7 == 3) ge += sc->home_jump_m;
                if (active_window(te, sc->gps_frozen_at, sc->gps_frozen_len)) {
                    if (!frozen_set) { frozen_e = ge; frozen_n = gn; frozen_set = true; }
                    ge = frozen_e; gn = frozen_n;
                } else frozen_set = false;
                nav_to_latlon(&o, (nav_en_t){ge, gn}, &fix_lat, &fix_lon);
                const float gve = hist_ve[k] + env->vel_noise * gauss(), gvn = hist_vn[k] + env->vel_noise * gauss();
                fix_speed = hypotf(gve, gvn);
                fix_course = nav_wrap_360(atan2f(gve, gvn) / D2R);
                fix_sacc = p.sacc_base * (1.0f + 0.3f * fabsf(gauss()));
                fix_sats = active_window(te, sc->gps_sats_at, sc->gps_sats_len) ? (uint8_t)sc->gps_sats : 12;
                fix_type = active_window(te, sc->gps_2d_at, sc->gps_2d_len) ? 2 : 3;
                last_fix_t = t;
            }
        }

        /* ---- fusion at 50 Hz (sensor_fusion.c's complementary filter) ---- */
        const bool fusion_tick = (step % 2) == 0;
        const bool imu_frozen = active_window(te, sc->imu_frozen_at, sc->imu_frozen_len);
        if (fusion_tick && !imu_frozen) {
            r_meas = boat_yaw(&b) * sc->gyro_scale + p.gyro_bias + 0.3f * gauss();
            const float cerr = p.bias + p.hdg_amp * sinf((b.psi + p.hdg_phase) * D2R) +
                               p.thr_err * (b.T / 0.4f) + compass_step;
            const float mag = b.psi + cerr + 1.0f * gauss();
            fused = nav_wrap_360(fused - r_meas * 0.02f);
            fused = nav_wrap_360(fused + 0.02f * wrap180(mag - fused));
            last_fusion_t = t;
        }

        /* ---- autonomy at 20 Hz (it can be made to stall) ----------------- */
        const bool stalled = active_window(te, sc->stall_at, sc->stall_len);
        if (step % 5 == 0 && !stalled) {
            mission_input_t in; memset(&in, 0, sizeof(in));
            in.t_s = t;
            in.settings = env->settings;
            if (!started && t >= 1.0f) {
                in.start = true; in.request_id = 7u; started = true; t_start = t;
            } else if (sc->start_spam && ts >= 0.0f && ts <= 3.0f) {
                in.start = true; in.request_id = 7u;              /* the laptop repeating START */
            } else if (sc->start_spam && ts >= 30.0f && !spam_new_sent) {
                in.start = true; in.request_id = 99u; spam_new_sent = true;   /* a second START mid-run */
            }
            if (sc->restart_at > 0.0f && !restart_issued && ts >= sc->restart_at) {
                in.start = true; in.request_id = 8u; restart_issued = true;
            }
            if (t_retry >= 0.0f && t >= t_retry) {
                in.start = true; in.request_id = 20u + (uint32_t)retries; retries++; t_retry = -1.0f;
            }
            if (stop_to_mission) { in.stop = true; stop_to_mission = false; }
            in.gps_new = last_fix_t >= 0.0f && last_fix_t != last_fix_seen;
            last_fix_seen = last_fix_t;
            in.gps_ubx = true; in.fix_type = fix_type; in.sats = fix_sats; in.pdop = 1.5f;
            in.fix_age_s = last_fix_t >= 0.0f ? t - last_fix_t : 99.0f;
            in.lat_deg = fix_lat; in.lon_deg = fix_lon;
            in.speed_mps = fix_speed; in.course_deg = fix_course; in.speed_acc_mps = fix_sacc;
            in.heading_valid = true; in.heading_deg = fused; in.yaw_rate_dps = r_meas;
            in.fusion_age_s = t - last_fusion_t; in.imu_ok = true;
            in.compass_calibrated = true; in.field_ratio = 1.0f;
            in.armed = armed || env->settings.dry_run; in.heading_hold_built = true;
            in.link_alive = !(t >= env->link_cut_start && t < env->link_cut_start + env->link_cut_len);
            if (sc->link_flap_period > 0.0f) in.link_alive = fmodf(t, sc->link_flap_period) > 0.3f * sc->link_flap_period;
            in.control_abort_run = ad.aborted_run_id;
            in.control_abort_reason = (uint8_t)ad.abort_reason;
            mout = mission_step(&m, &cfg, &in);
            /* The operator reads "refused: GPS ..." on the laptop and presses START again. */
            if (in.start && m.state == MISSION_REFUSED && retries < 3 &&
                (m.reason == MISSION_REFUSE_FIX_OLD || m.reason == MISSION_REFUSE_FEW_SATS ||
                 m.reason == MISSION_REFUSE_NO_3D_FIX || m.reason == MISSION_REFUSE_PDOP)) {
                t_retry = t + 2.0f;
            }
            sp.run_id = m.run_id; sp.active = mout.active; sp.drive = mout.drive;
            sp.dry_run = mout.dry_run; sp.heading_deg = mout.heading_deg;
            sp.throttle = mout.throttle; sp.stamp_us = (int64_t)(t * 1e6f);
            if (m.run_id != last_run && m.state == MISSION_HOME) { last_run = m.run_id; res.runs_started++; }
            if (m.beta_valid) res.beta_valid_ever = 1;
            if (m.compass_bad) res.compass_bad = 1;

            /* laptop no-progress warning (what the tool will show) */
            if ((int)m.state != np_state) {
                np_state = (int)m.state; np_t = t;
                np_best = m.state == MISSION_OUTBOUND ? -m.dist_home_m : m.dist_home_m;
            }
            if (m.state == MISSION_OUTBOUND || m.state == MISSION_RETURN) {
                const float metric = m.state == MISSION_OUTBOUND ? -m.dist_home_m : m.dist_home_m;
                if (metric < np_best - 1.0f) { np_best = metric; np_t = t; }
                else if (t - np_t > 30.0f) res.warned = 1;
            } else if (m.state == MISSION_TURN && t - np_t > 30.0f) {
                res.warned = 1;
            }
        }

        /* ---- control at 100 Hz: operator events, AUTO owner, hold ------- */
        if (ev_stop) { stop_to_mission = true; t_retry = -1.0f; }   /* reaches both tasks; no more retries */
        if (after(te, sc->disarm_at)) armed = false;
        if (after(te, sc->rail_cut_at)) rail_cut = true;
        const bool other = after(te, sc->other_owner_at);
        const auto_inputs_t ain = { .now_us = (int64_t)(t * 1e6f), .armed = armed,
                                    .rail_cut = rail_cut, .other_owner = other,
                                    .manual_input = ev_manual, .stop_request = ev_stop };
        const bool owned_before = sp.run_id != 0u && sp.run_id != ad.aborted_run_id && sp.active;
        const auto_out_t aout = auto_drive_step(&ad, &sp, &ain);
        if (ev_manual) manual_driving = true;
        if (fusion_tick) {
            yaw_heading_input_t hin = { .dt_s = 0.02f, .yaw_rate_dps = r_meas, .heading_deg = fused,
                .throttle = 0.0f, .feedforward_c = LEARNED_C, .steering = 0.0f, .enabled = false,
                .driving = armed, .gyro_fresh = (t - last_fusion_t) < 0.2f, .heading_valid = true,
                .base_capture_now = false };
            if (aout.own) auto_drive_hold_input(&aout, &hin);
            hout = yaw_heading_control_update(&ctl, &HCFG, &hin);
        }
        if (aout.own && aout.drive) {
            auto_drive_mix(aout.throttle, LEARNED_C, hout.dynamic_c, hout.active, &L, &R);
        } else if (manual_driving) {
            L = 0.3f; R = 0.3f;       /* the operator drives on */
        } else {
            L = R = 0.0f;
        }
        if (!armed) L = R = 0.0f;
        if (!manual_driving) res.max_cmd = fmaxf(res.max_cmd, fmaxf(L, R));
        if (owned_before && !aout.own && !manual_driving && (L != 0.0f || R != 0.0f)) res.motors_after_stop++;

        /* ---- bookkeeping ------------------------------------------------ */
        if (t_state_first[m.state] < 0.0f) t_state_first[m.state] = t;
        if (m.state == MISSION_HOME) {
            if (!in_home) { in_home = true; th_e = th_n = 0.0; th_k = 0; have_p0 = false; res.closest_true = 1e9f; }
            th_e += b.e; th_n += b.n; th_k++;
            true_home_e = (float)(th_e / th_k); true_home_n = (float)(th_n / th_k);
        } else {
            in_home = false;
        }
        if ((int)m.state != leg_state) { leg_state = (int)m.state; leg_psi_turn = 0.0f; }
        leg_psi_turn += -boat_yaw(&b) * DT;
        if (mission_is_active(&m)) res.max_abs_turn = fmaxf(res.max_abs_turn, fabsf(leg_psi_turn));
        const float d_true = hypotf(b.e - true_home_e, b.n - true_home_n);
        if (m.state == MISSION_RETURN) {
            if (!have_p0) { p0_e = b.e; p0_n = b.n; have_p0 = true; }
            res.closest_true = fminf(res.closest_true, d_true);
            const nav_line_pos_t lp = nav_line_position((nav_en_t){p0_e, p0_n},
                                                        (nav_en_t){true_home_e, true_home_n},
                                                        (nav_en_t){b.e, b.n});
            res.max_xtrack = fmaxf(res.max_xtrack, fabsf(lp.cross_m));
        }
        if (trace && step % 10 == 0) {
            fprintf(trace, "%.2f,%d,%.3f,%.3f,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f\n", t, m.state, b.e, b.n,
                    b.psi, fused, L, R, b.v, mout.heading_deg, m.beta_deg, b.cbal);
        }
        const bool terminal = m.state == MISSION_DONE || m.state == MISSION_ABORTED || m.state == MISSION_REFUSED;
        const bool retry_pending = m.state == MISSION_REFUSED && t_retry >= 0.0f;
        const bool restart_pending = (sc->restart_at > 0.0f && !restart_issued) || retry_pending;
        if (terminal && t_done < 0.0f && !restart_pending) {
            t_done = t;
            res.reason = m.reason;
            res.state = (int)m.state;
            res.refused = m.state == MISSION_REFUSED;
            res.t_done = t - (t_start >= 0.0f ? t_start : 0.0f);
            res.d_done = d_true;
            res.turned = m.turned_deg;
            res.outages = (int)m.outages;
            res.beta_final = m.beta_deg;
            res.latency = t_event >= 0.0f ? t - t_event : -1.0f;
            /* An arrival declared where the boat truly never was: GPS error
             * allowance 1.5 m for "in zone"; "passed"/"near" need the true
             * closest approach inside the approach radius + 1 m. */
            if (m.state == MISSION_DONE && m.reason == MISSION_DONE_IN_ZONE &&
                d_true > env->settings.home_radius_m + 1.5f) res.false_arrival = 1;
            if (m.state == MISSION_DONE && (m.reason == MISSION_DONE_PASSED || m.reason == MISSION_DONE_NEAR) &&
                res.closest_true > 2.0f * env->settings.home_radius_m + 1.0f) res.false_arrival = 1;
            if (m.reason == MISSION_DONE_TURNED) {
                res.heading_err_at_turn_end = wrap180(atan2f(true_home_e - b.e, true_home_n - b.n) / D2R - b.psi);
            }
        }
        if (t_done >= 0.0f && t > t_done + 6.0f) break;
        t += DT; step++;
    }
    const float d_now = hypotf(b.e - true_home_e, b.n - true_home_n);
    if (t_done < 0.0f) { res.reason = -1; res.state = (int)m.state; res.t_done = t_max; res.d_done = d_now; }
    res.d_final = d_now;
    res.spun = res.max_abs_turn > 400.0f;
    return res;
}

static envelope_t nominal_envelope(void)
{
    envelope_t e = {
        /* fitted to the 09-21 turns/pulses (K 56-58, S 14.0/19.3, tau 1.15-1.3,
         * delay 0.05-0.15 s once the logged commands are re-timed) and to the
         * P-ON straight runs (yaw std ~1.4 deg/s, c std ~0.15) */
        .K_min = 50.0f, .K_max = 65.0f, .SL_min = 12.5f, .SL_max = 15.5f, .SR_min = 17.5f, .SR_max = 21.0f,
        .tau_min = 1.0f, .tau_max = 1.4f, .delay_min = 0.05f, .delay_max = 0.18f,
        .cbal_min = 0.05f, .cbal_max = 0.40f, .cbal_hdg_max = 0.40f, .cbal_walk = 0.12f,
        .wave_min = 1.0f, .wave_max = 2.5f, .chop_min = 0.6f, .chop_max = 1.2f,
        .drift_min = 0.0f, .drift_max = 0.10f,
        .bias_max = 45.0f, .bias_fixed = 0, .hdg_err_max = 8.0f, .thr_err_max = 5.0f,
        .gps_white = 0.12f, .gps_wander = 0.3f, .gps_drop_per_s = 0.01f, .gps_jump_per_s = 0.003f,
        .gps_lag_min_s = 0.05f, .gps_lag_max_s = 0.20f, .sacc_mean = 0.06f, .vel_noise = 0.04f,
        .link_cut_start = 20.0f, .link_cut_len = 30.0f };
    e.settings = mission_settings_default();
    return e;
}

static bool in_zone_true(const envelope_t *env, const result_t *r)
{
    return r->state == MISSION_DONE && r->reason >= MISSION_DONE_IN_ZONE &&
           r->d_final <= env->settings.home_radius_m + 0.5f && !r->spun;
}

/* Does one run meet its scenario's expectation?  Never if it declared an
 * arrival far from home or left the jets running after a stop. */
static bool meets(const scenario_t *sc, const envelope_t *env, const result_t *r)
{
    if (r->false_arrival || r->motors_after_stop) return false;
    if (sc->need_motors_never && r->max_cmd > 0.0f) return false;
    if (sc->need_compass_bad && !r->compass_bad) return false;
    if (sc->need_beta_invalid && r->beta_valid_ever) return false;
    if (sc->need_runs && r->runs_started != sc->need_runs) return false;
    switch (sc->expect) {
    case EXPECT_IN_ZONE:
        return in_zone_true(env, r);
    case EXPECT_DONE_OUT:
        return r->state == MISSION_DONE && r->reason == MISSION_DONE_OUT &&
               r->d_done >= env->settings.out_distance_m - 1.0f;
    case EXPECT_DONE_TURNED:
        return r->state == MISSION_DONE && r->reason == MISSION_DONE_TURNED &&
               fabsf(r->heading_err_at_turn_end) < 60.0f;
    case EXPECT_ABORT:
        return r->state == MISSION_ABORTED && r->reason == sc->abort_reason &&
               r->latency >= 0.0f && r->latency <= sc->max_latency_s;
    case EXPECT_SAFE:
        if (r->state == MISSION_DONE) return !r->spun;
        if (r->state == MISSION_ABORTED) return r->reason == MISSION_ABORT_SPIN;
        return r->reason == -1 && r->warned && !r->spun;
    case EXPECT_DONE_ANY:
        return r->state == MISSION_DONE && !r->spun;
    }
    return false;
}

/* ---- envelope tweaks per scenario ------------------------------------------ */
static void env_out_only(envelope_t *e) { e->settings.stage = MISSION_STAGE_OUT; }
static void env_out_turn(envelope_t *e) { e->settings.stage = MISSION_STAGE_OUT_TURN; }
static void env_out_turn_left(envelope_t *e) { e->settings.stage = MISSION_STAGE_OUT_TURN; e->settings.turn_right = false; }
static void env_left(envelope_t *e) { e->settings.turn_right = false; }
static void env_dry(envelope_t *e) { e->settings.dry_run = true; }
static void env_radio_gone(envelope_t *e) { e->link_cut_start = 5.0f; e->link_cut_len = 1e9f; }
static void env_small_zone(envelope_t *e) { e->settings.home_radius_m = 1.5f; e->drift_max = 0.05f; }
static void env_zone_5(envelope_t *e) { e->settings.home_radius_m = 5.0f; }
static void env_far_30(envelope_t *e) { e->settings.out_distance_m = 30.0f; }
static void env_far_50(envelope_t *e) { e->settings.out_distance_m = 50.0f; }
static void env_out_5(envelope_t *e) { e->settings.out_distance_m = 5.0f; }
static void env_thr_20(envelope_t *e) { e->settings.throttle = 0.20f; e->settings.approach_throttle = 0.20f; }
static void env_thr_60(envelope_t *e) { e->settings.throttle = 0.60f; }
static void env_bias_45(envelope_t *e) { e->bias_max = 45.0f; e->bias_fixed = 1; }
static void env_bias_55(envelope_t *e) { e->bias_max = 55.0f; e->bias_fixed = 1; }
static void env_bias_70(envelope_t *e) { e->bias_max = 70.0f; e->bias_fixed = 1; }
static void env_lag_200(envelope_t *e) { e->gps_lag_min_s = e->gps_lag_max_s = 0.20f; }
static void env_sacc_poor(envelope_t *e) { e->sacc_mean = 0.60f; e->bias_max = 10.0f; }
static void env_sacc_poor_b30(envelope_t *e) { e->sacc_mean = 0.60f; e->bias_max = 30.0f; e->bias_fixed = 1; }
static void env_cbal_wide(envelope_t *e) { e->cbal_min = -0.35f; e->cbal_max = 0.60f; e->cbal_walk = 0.25f; }
static void env_windy(envelope_t *e) { e->cbal_hdg_max = 0.60f; e->cbal_walk = 0.20f; }
static void env_old_compass(envelope_t *e) { e->bias_max = 45.0f; e->hdg_err_max = 22.0f; }
static void env_waves_x2(envelope_t *e) { e->wave_min *= 2.0f; e->wave_max *= 2.0f; e->chop_min *= 2.0f; e->chop_max *= 2.0f; }
static void env_drift_015(envelope_t *e) { e->drift_min = 0.10f; e->drift_max = 0.15f; }
static void env_gps_noisy(envelope_t *e) { e->gps_white = 0.30f; e->gps_jump_per_s = 0.02f; }
static void env_new_compass(envelope_t *e) { e->bias_max = 5.0f; e->hdg_err_max = 5.0f; }
static void env_slow_boat(envelope_t *e) { e->K_min = 20.0f; e->K_max = 30.0f; e->tau_min = 1.1f; e->tau_max = 1.5f; e->delay_min = 0.4f; e->delay_max = 0.5f; }

#define NSC 72
static int build_catalogue(scenario_t *out)
{
    int n = 0;
    scenario_t s;
#define ADD() do { if (n < NSC) out[n++] = s; } while (0)
    /* -- the missions as Kiet will fly them ----------------------------- */
    s = scenario_defaults(); s.name = "nominal"; s.what = "drift<=0.1, compass<=45 deg, GPS noise/jumps/lag, 30 s radio cut"; ADD();
    s = scenario_defaults(); s.name = "new_compass"; s.what = "flat calibration as checked indoors (<=5 deg)"; s.env_fn = env_new_compass; ADD();
    s = scenario_defaults(); s.name = "out_only"; s.what = "stage OUT ONLY"; s.expect = EXPECT_DONE_OUT; s.env_fn = env_out_only; ADD();
    s = scenario_defaults(); s.name = "out_turn"; s.what = "stage OUT + TURN: ends facing home"; s.expect = EXPECT_DONE_TURNED; s.env_fn = env_out_turn; ADD();
    s = scenario_defaults(); s.name = "out_turn_left"; s.what = "stage OUT + TURN, turning left"; s.expect = EXPECT_DONE_TURNED; s.env_fn = env_out_turn_left; ADD();
    s = scenario_defaults(); s.name = "left_turn"; s.what = "FULL, turn left (the weak side)"; s.env_fn = env_left; ADD();
    s = scenario_defaults(); s.name = "dry_run"; s.what = "DRY RUN: carried along the wanted heading, jets never run"; s.expect = EXPECT_DONE_ANY; s.dry_run_walk = 1; s.need_motors_never = 1; s.env_fn = env_dry; ADD();
    s = scenario_defaults(); s.name = "north_wrap"; s.what = "pointed within 5 deg of north (0/360 wrap)"; s.start_hdg_fixed = 1; s.start_hdg_min = -5.0f; s.start_hdg_max = 5.0f; ADD();
    s = scenario_defaults(); s.name = "out_5m"; s.what = "out 5 m (the shortest allowed)"; s.env_fn = env_out_5; ADD();
    s = scenario_defaults(); s.name = "far_30m"; s.what = "out 30 m"; s.env_fn = env_far_30; ADD();
    s = scenario_defaults(); s.name = "far_50m"; s.what = "out 50 m (the longest allowed)"; s.env_fn = env_far_50; ADD();
    s = scenario_defaults(); s.name = "zone_5m"; s.what = "home radius 5 m"; s.env_fn = env_zone_5; ADD();
    s = scenario_defaults(); s.name = "calm_small_zone"; s.what = "home radius 1.5 m, drift<=0.05"; s.env_fn = env_small_zone; s.min_pass = 0.80f; ADD();
    s = scenario_defaults(); s.name = "throttle_20"; s.what = "mission throttle 20 %"; s.env_fn = env_thr_20; ADD();
    s = scenario_defaults(); s.name = "throttle_60"; s.what = "mission throttle 60 %"; s.env_fn = env_thr_60; ADD();
    /* -- the radio (must never stop the mission) ------------------------- */
    s = scenario_defaults(); s.name = "radio_gone"; s.what = "radio lost 5 s after START, for good"; s.env_fn = env_radio_gone; ADD();
    s = scenario_defaults(); s.name = "radio_flapping"; s.what = "radio drops 30 % of every second"; s.link_flap_period = 1.0f; ADD();
    /* -- the operator ---------------------------------------------------- */
    s = scenario_defaults(); s.name = "start_spam"; s.what = "START repeated 3 s + a new START mid-run: one run only"; s.start_spam = 1; s.need_runs = 1; ADD();
    s = scenario_defaults(); s.name = "stop_in_home"; s.what = "STOP while home is taken: jets never run"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_STOP; s.event_state = MISSION_HOME; s.stop_at = 0.5f; s.max_latency_s = 0.1f; s.need_motors_never = 1; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "stop_mid"; s.what = "STOP 20 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_STOP; s.stop_at = 20.0f; s.max_latency_s = 0.1f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "stop_in_return"; s.what = "STOP 3 s into the return"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_STOP; s.event_state = MISSION_RETURN; s.stop_at = 3.0f; s.max_latency_s = 0.1f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "manual_nudge"; s.what = "stick touched 25 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_MANUAL; s.manual_at = 25.0f; s.max_latency_s = 0.1f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "manual_in_turn"; s.what = "stick touched 1 s into the turn"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_MANUAL; s.event_state = MISSION_TURN; s.manual_at = 1.0f; s.max_latency_s = 0.1f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "restart_after_stop"; s.what = "STOP 10 s in, START again 20 s in: second run home"; s.stop_at = 10.0f; s.restart_at = 20.0f; s.need_runs = 2; ADD();
    s = scenario_defaults(); s.name = "disarm_mid"; s.what = "disarmed 30 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_DISARMED; s.disarm_at = 30.0f; s.max_latency_s = 0.1f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "rail_cut"; s.what = "explicit servo PWR-OFF 15 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_RAIL_CUT; s.rail_cut_at = 15.0f; s.max_latency_s = 0.1f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "calibration_start"; s.what = "a calibration/bench takes the jets 12 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_OTHER_OWNER; s.other_owner_at = 12.0f; s.max_latency_s = 0.1f; s.min_pass = 1.0f; ADD();
    /* -- GPS ------------------------------------------------------------- */
    s = scenario_defaults(); s.name = "gps_dropout_3s"; s.what = "no fix for 3 s, 15 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_GPS_LOST; s.gps_drop_at = 15.0f; s.gps_drop_len = 3.0f; s.max_latency_s = 2.3f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "gps_dropout_1s"; s.what = "no fix for 1 s in the return (rides through)"; s.event_state = MISSION_RETURN; s.gps_drop_at = 5.0f; s.gps_drop_len = 1.0f; ADD();
    s = scenario_defaults(); s.name = "gps_dropout_1_5s"; s.what = "no fix for 1.5 s, 15 s in (rides through)"; s.gps_drop_at = 15.0f; s.gps_drop_len = 1.5f; ADD();
    s = scenario_defaults(); s.name = "gps_jump_home"; s.what = "3 m jump TOWARD home for 1.5 s in the return"; s.event_state = MISSION_RETURN; s.gps_jump_at = 8.0f; s.gps_jump_len = 1.5f; s.gps_jump_m = 3.0f; s.gps_jump_toward_home = 1; ADD();
    s = scenario_defaults(); s.name = "gps_jump_side"; s.what = "5 m sideways jump for 1.5 s in the return"; s.event_state = MISSION_RETURN; s.gps_jump_at = 5.0f; s.gps_jump_len = 1.5f; s.gps_jump_m = 5.0f; ADD();
    s = scenario_defaults(); s.name = "gps_jump_near_home"; s.what = "3 m jump toward home on the last metres"; s.event_state = MISSION_RETURN; s.gps_jump_at = 16.0f; s.gps_jump_len = 1.5f; s.gps_jump_m = 3.0f; s.gps_jump_toward_home = 1; ADD();
    s = scenario_defaults(); s.name = "gps_frozen"; s.what = "position stuck but fixes fresh, 20 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_GPS_FROZEN; s.gps_frozen_at = 20.0f; s.gps_frozen_len = 30.0f; s.max_latency_s = 4.0f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "gps_few_sats"; s.what = "satellites drop to 5, 18 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_GPS_LOST; s.gps_sats_at = 18.0f; s.gps_sats_len = 5.0f; s.gps_sats = 5; s.max_latency_s = 2.3f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "gps_6_sats"; s.what = "6 satellites after 5 s (still enough)"; s.gps_sats_at = 5.0f; s.gps_sats_len = 1e9f; s.gps_sats = 6; ADD();
    s = scenario_defaults(); s.name = "gps_2d"; s.what = "fix drops to 2D, 18 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_GPS_LOST; s.gps_2d_at = 18.0f; s.gps_2d_len = 5.0f; s.max_latency_s = 2.3f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "gps_lag_200ms"; s.what = "fixes 200 ms late"; s.env_fn = env_lag_200; ADD();
    s = scenario_defaults(); s.name = "gps_noisy"; s.what = "0.3 m noise + a jump every ~50 s"; s.env_fn = env_gps_noisy; s.min_pass = 0.90f; ADD();
    s = scenario_defaults(); s.name = "gps_sacc_poor"; s.what = "speed accuracy 0.6 m/s: beta never learnt; compass<=10 deg"; s.env_fn = env_sacc_poor; s.need_beta_invalid = 1; ADD();
    s = scenario_defaults(); s.name = "gps_sacc_poor_b30"; s.what = "speed accuracy 0.6 m/s and a 30 deg compass error"; s.env_fn = env_sacc_poor_b30; s.expect = EXPECT_SAFE; s.need_beta_invalid = 1; ADD();
    s = scenario_defaults(); s.name = "home_gps_jumpy"; s.what = "GPS jumps 3 m while home is taken: stops, jets never ran"; s.home_jump_m = 3.0f; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_HOME_NOT_STILL; s.max_latency_s = 5.2f; s.need_motors_never = 1; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "home_gps_wobble"; s.what = "GPS wobbles 1 m while home is taken (accepted)"; s.home_jump_m = 1.0f; ADD();
    /* -- compass / IMU --------------------------------------------------- */
    s = scenario_defaults(); s.name = "compass_45"; s.what = "compass off by exactly 45 deg"; s.env_fn = env_bias_45; ADD();
    s = scenario_defaults(); s.name = "compass_55"; s.what = "compass off by 55 deg (crab can hide the flag), still home"; s.env_fn = env_bias_55; ADD();
    s = scenario_defaults(); s.name = "compass_70"; s.what = "compass off by 70 deg: compass-bad flag, beta clamped at 60"; s.env_fn = env_bias_70; s.need_compass_bad = 1; s.min_pass = 0.90f; ADD();
    s = scenario_defaults(); s.name = "compass_step"; s.what = "compass jumps +30 deg in the return"; s.event_state = MISSION_RETURN; s.compass_step_at = 4.0f; s.compass_step_deg = 30.0f; ADD();
    s = scenario_defaults(); s.name = "compass_motor"; s.what = "compass error grows 12 deg with throttle"; s.compass_thr_amp = 12.0f; ADD();
    s = scenario_defaults(); s.name = "compass_hard_iron"; s.what = "heading-dependent compass error up to 15 deg"; s.compass_hdg_amp = 15.0f; s.min_pass = 0.90f; ADD();
    s = scenario_defaults(); s.name = "old_compass"; s.what = "the 09-20/21 calibration: <=45 deg + 22 deg with heading"; s.env_fn = env_old_compass; s.min_pass = 0.90f; ADD();
    s = scenario_defaults(); s.name = "gyro_scale"; s.what = "gyro reads 6 % high"; s.gyro_scale = 1.06f; ADD();
    s = scenario_defaults(); s.name = "gyro_bias_1dps"; s.what = "gyro bias 1 deg/s"; s.gyro_bias = 1.0f; ADD();
    s = scenario_defaults(); s.name = "imu_frozen"; s.what = "IMU freezes 22 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_HEADING_LOST; s.imu_frozen_at = 22.0f; s.imu_frozen_len = 20.0f; s.max_latency_s = 0.7f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "imu_hiccup"; s.what = "IMU pauses 0.3 s, 22 s in (rides through)"; s.imu_frozen_at = 22.0f; s.imu_frozen_len = 0.3f; ADD();
    /* -- jets ------------------------------------------------------------ */
    s = scenario_defaults(); s.name = "balance_wide"; s.what = "straight-running c -0.35..0.60, walking faster"; s.env_fn = env_cbal_wide; s.min_pass = 0.90f; ADD();
    s = scenario_defaults(); s.name = "windy_balance"; s.what = "straight-running c swings +-0.6 with heading (1.5x 09-20)"; s.env_fn = env_windy; s.min_pass = 0.90f; ADD();
    s = scenario_defaults(); s.name = "slow_boat"; s.what = "weaker, slower, later yaw response than fitted"; s.env_fn = env_slow_boat; s.min_pass = 0.90f; ADD();
    s = scenario_defaults(); s.name = "jet_weakens"; s.what = "right jet loses half its thrust 20 s in"; s.jet_fault_at = 20.0f; s.jet_right_factor = 0.5f; s.expect = EXPECT_SAFE; ADD();
    s = scenario_defaults(); s.name = "jet_dies"; s.what = "right jet dies 20 s in: no spin, laptop warns"; s.jet_fault_at = 20.0f; s.jet_right_factor = 0.0f; s.expect = EXPECT_SAFE; ADD();
    /* -- water ----------------------------------------------------------- */
    s = scenario_defaults(); s.name = "drift_0.15"; s.what = "drift 0.10-0.15 m/s (past the pass gate)"; s.env_fn = env_drift_015; s.expect = EXPECT_SAFE; ADD();
    s = scenario_defaults(); s.name = "gusty"; s.what = "wind gusts +-0.1 m/s over 10 s"; s.gust_amp = 0.1f; s.gust_period = 10.0f; ADD();
    s = scenario_defaults(); s.name = "strong_current"; s.what = "0.35 m/s current (about the boat's speed)"; s.current_fixed = 1; s.current_mps = 0.35f; s.current_dir = 45.0f; s.expect = EXPECT_SAFE; ADD();
    s = scenario_defaults(); s.name = "current_faster"; s.what = "0.5 m/s current (faster than the boat)"; s.current_fixed = 1; s.current_mps = 0.5f; s.current_dir = 100.0f; s.expect = EXPECT_SAFE; ADD();
    s = scenario_defaults(); s.name = "waves_x2"; s.what = "twice the recorded wave yaw"; s.env_fn = env_waves_x2; ADD();
    s = scenario_defaults(); s.name = "start_drifting"; s.what = "boat drifting 0.15 m/s when START is pressed"; s.start_drift = 0.15f; ADD();
    /* -- the boat's own tasks --------------------------------------------- */
    s = scenario_defaults(); s.name = "autonomy_stall"; s.what = "mission task stalls 1 s, 20 s in"; s.expect = EXPECT_ABORT; s.abort_reason = MISSION_ABORT_STALE; s.stall_at = 20.0f; s.stall_len = 1.0f; s.max_latency_s = 1.1f; s.min_pass = 1.0f; ADD();
    s = scenario_defaults(); s.name = "autonomy_hiccup"; s.what = "mission task stalls 0.3 s (rides through)"; s.stall_at = 20.0f; s.stall_len = 0.3f; ADD();
#undef ADD
    return n;
}

static int cmpf(const void *a, const void *b)
{
    const float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

typedef struct {
    int n, meet, inzone, done, aborted, refused, never, spins, false_arr, warned, beta_valid;
    float d_med, d_p95, t_med;
} stats_t;

static stats_t run_set(const envelope_t *env, const scenario_t *sc, int runs, uint64_t seed, int verbose)
{
    stats_t s; memset(&s, 0, sizeof(s));
    float *d = malloc(sizeof(float) * (size_t)runs), *tt = malloc(sizeof(float) * (size_t)runs);
    if (!d || !tt) { free(d); free(tt); return s; }
    for (int i = 0; i < runs; ++i) {
        const result_t r = run_one(env, sc, seed + (uint64_t)i, NULL);
        const bool good = meets(sc, env, &r);
        s.n++; s.meet += good; s.inzone += in_zone_true(env, &r);
        s.done += r.state == MISSION_DONE; s.aborted += r.state == MISSION_ABORTED;
        s.refused += r.state == MISSION_REFUSED; s.never += r.reason == -1;
        s.spins += r.spun; s.false_arr += r.false_arrival; s.warned += r.warned;
        s.beta_valid += r.beta_valid_ever;
        d[i] = r.d_final; tt[i] = r.t_done;
        if (verbose && !good) {
            printf("   miss run %d: state=%d reason=%d t=%.1f d_done=%.2f d_final=%.2f closest=%.2f false=%d "
                   "motors_after=%d spun=%d(%.0f) warned=%d runs=%d max_cmd=%.2f latency=%.2f turn_err=%.0f beta=%.1f\n",
                   i, r.state, r.reason, r.t_done, r.d_done, r.d_final, r.closest_true, r.false_arrival,
                   r.motors_after_stop, r.spun, r.max_abs_turn, r.warned, r.runs_started, r.max_cmd, r.latency,
                   r.heading_err_at_turn_end, r.beta_final);
        }
    }
    qsort(d, (size_t)runs, sizeof(float), cmpf);
    qsort(tt, (size_t)runs, sizeof(float), cmpf);
    s.d_med = d[runs / 2]; s.d_p95 = d[(int)((float)runs * 0.95f)]; s.t_med = tt[runs / 2];
    free(d); free(tt);
    return s;
}

/* ---- sweeps: one parameter at a time around the nominal envelope --------- */
typedef struct { const char *name; const char *unit; float v[10]; int n; } sweep_t;
static const sweep_t SWEEPS[] = {
    {"drift", "m/s", {0.0f, 0.05f, 0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.35f, 0.40f}, 9},
    {"compass", "deg", {0.0f, 15.0f, 30.0f, 45.0f, 55.0f, 60.0f, 70.0f}, 7},
    {"gps_white", "m", {0.05f, 0.12f, 0.25f, 0.40f, 0.60f}, 5},
    {"gps_wander", "m", {0.1f, 0.3f, 0.6f, 1.0f, 1.5f}, 5},
    {"gps_lag", "s", {0.05f, 0.10f, 0.20f, 0.25f, 0.30f}, 5},
    {"sacc", "m/s", {0.03f, 0.06f, 0.10f, 0.20f, 0.30f}, 5},
    {"sacc_honest", "m/s", {0.03f, 0.06f, 0.10f, 0.20f, 0.30f}, 5},
    {"throttle", "", {0.20f, 0.30f, 0.40f, 0.50f, 0.60f}, 5},
    {"radius", "m", {1.5f, 2.0f, 2.5f, 3.0f, 4.0f, 5.0f}, 6},
    {"out", "m", {5.0f, 10.0f, 20.0f, 30.0f, 50.0f}, 5},
    {"cbal", "", {-0.35f, -0.1f, 0.1f, 0.25f, 0.4f, 0.55f, 0.7f}, 7},
    {"K", "", {15.0f, 25.0f, 35.0f, 45.0f, 55.0f, 65.0f, 80.0f}, 7},
    {"delay", "s", {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f}, 6},
    {"turn_right", "", {1.0f, 0.0f}, 2},
    {"waves", "x", {0.5f, 1.0f, 2.0f, 3.0f}, 4},
};
#define NSWEEPS ((int)(sizeof(SWEEPS) / sizeof(SWEEPS[0])))

static void sweep_apply(envelope_t *e, const char *name, float v)
{
    if (!strcmp(name, "drift")) { e->drift_min = e->drift_max = v; }
    else if (!strcmp(name, "compass")) { e->bias_max = v; e->bias_fixed = 1; }
    else if (!strcmp(name, "gps_white")) e->gps_white = v;
    else if (!strcmp(name, "gps_wander")) e->gps_wander = v;
    else if (!strcmp(name, "gps_lag")) e->gps_lag_min_s = e->gps_lag_max_s = v;
    else if (!strcmp(name, "sacc")) e->sacc_mean = v;
    else if (!strcmp(name, "sacc_honest")) { e->sacc_mean = v; e->vel_noise = v > 0.04f ? v : 0.04f; }
    else if (!strcmp(name, "throttle")) {
        e->settings.throttle = v;
        if (e->settings.approach_throttle > v) e->settings.approach_throttle = v;
    }
    else if (!strcmp(name, "radius")) e->settings.home_radius_m = v;
    else if (!strcmp(name, "out")) e->settings.out_distance_m = v;
    else if (!strcmp(name, "cbal")) { e->cbal_min = e->cbal_max = v; }
    else if (!strcmp(name, "K")) { e->K_min = e->K_max = v; }
    else if (!strcmp(name, "delay")) { e->delay_min = e->delay_max = v; }
    else if (!strcmp(name, "turn_right")) e->settings.turn_right = v > 0.5f;
    else if (!strcmp(name, "waves")) { e->wave_min *= v; e->wave_max *= v; e->chop_min *= v; e->chop_max *= v; }
}

static void run_sweep(const sweep_t *sw, int runs, uint64_t seed)
{
    scenario_t nom = scenario_defaults(); nom.name = "nominal";
    if (g_approach > 0.0f) printf("(approach throttle %.2f)\n", g_approach);
    printf("sweep %s (%d runs each; in zone = DONE and truly within radius + 0.5 m)\n", sw->name, runs);
    printf("  %9s %8s %7s %7s %6s %6s %5s %5s %5s %6s %6s\n", "value", "in zone", "d_med", "d_p95",
           "never", "abort", "spins", "false", "warn", "beta", "t_med");
    for (int i = 0; i < sw->n; ++i) {
        envelope_t e = nominal_envelope();
        if (g_approach > 0.0f) e.settings.approach_throttle = g_approach;
        sweep_apply(&e, sw->name, sw->v[i]);
        const stats_t s = run_set(&e, &nom, runs, seed, 0);
        printf("  %6.2f%-3s %7.1f%% %7.2f %7.2f %6d %6d %5d %5d %5d %5.0f%% %6.0f\n", sw->v[i], sw->unit,
               100.0f * (float)s.inzone / (float)s.n, s.d_med, s.d_p95, s.never, s.aborted, s.spins,
               s.false_arr, s.warned, 100.0f * (float)s.beta_valid / (float)s.n, s.t_med);
    }
}

/* ---- plant check: the model's own numbers, to compare with the lake data --
 * One JSON line per random boat (nominal envelope):
 *   v15/v20/v40/v45: speed after 20 s held straight (c = its balance, no waves);
 *   t90_40: time to 90 % of the T40 speed;
 *   glide3_40/half_40, glide3_20/half_20: glide in 3 s and speed half-life
 *   after motors off from T40 / T20;
 *   pon_T: 25 s straight with the heading hold (closed loop, waves on) at
 *   T20 and T40: gyro yaw std, applied-c std and mean, heading change over
 *   the last 20 s, largest distance off the start line;
 *   poff_T40: the same with the hold off (learned c only). */
typedef struct { float yaw_std, c_std, c_mean, hdg_change, max_off; } straight_stats_t;

static straight_stats_t plant_straight(const boat_params_t *p, float T, int hold_on)
{
    boat_t b; boat_init(&b, p);
    yaw_heading_control_t ctl; yaw_heading_control_init(&ctl);
    yaw_heading_output_t ho; memset(&ho, 0, sizeof(ho));
    float fused = p->start_heading, r_meas = 0.0f, Lc = 0.0f, Rc = 0.0f;
    double ys = 0.0, yss = 0.0, cs = 0.0, css = 0.0; int ny = 0, nc = 0;
    float max_off = 0.0f, psi0 = 0.0f, e0 = 0.0f, n0 = 0.0f;
    for (int i = 0; i < 2500; ++i) {
        boat_step(&b, p, Lc, Rc, -p->drift_e, -p->drift_n, 0);
        if (i % 2 == 0) {
            r_meas = boat_yaw(&b) + p->gyro_bias + 0.3f * gauss();
            fused = nav_wrap_360(fused - r_meas * 0.02f);
            fused = nav_wrap_360(fused + 0.02f * wrap180(b.psi + 1.0f * gauss() - fused));
            const yaw_heading_input_t hin = { .dt_s = 0.02f, .yaw_rate_dps = r_meas, .heading_deg = fused,
                .throttle = T, .feedforward_c = LEARNED_C, .steering = 0.0f, .enabled = hold_on != 0,
                .driving = true, .gyro_fresh = true, .heading_valid = true, .base_capture_now = false };
            ho = yaw_heading_control_update(&ctl, &HCFG, &hin);
        }
        auto_drive_mix(T, LEARNED_C, ho.dynamic_c, ho.active, &Lc, &Rc);
        if (i == 500) { psi0 = b.psi; e0 = b.e; n0 = b.n; }
        if (i >= 500 && i % 5 == 0) {           /* 20 Hz, like the telemetry */
            ys += r_meas; yss += (double)r_meas * r_meas; ny++;
            const float c = (Lc + Rc) > 0.0f ? (Rc - Lc) / (Rc + Lc) : 0.0f;
            cs += c; css += (double)c * c; nc++;
            const float off = (b.e - e0) * cosf(psi0 * D2R) - (b.n - n0) * sinf(psi0 * D2R);
            max_off = fmaxf(max_off, fabsf(off));
        }
    }
    straight_stats_t s;
    s.yaw_std = (float)sqrt(fmax(0.0, yss / ny - (ys / ny) * (ys / ny)));
    s.c_mean = (float)(cs / nc);
    s.c_std = (float)sqrt(fmax(0.0, css / nc - (cs / nc) * (cs / nc)));
    s.hdg_change = wrap180(b.psi - psi0);
    s.max_off = max_off;
    return s;
}

static float plant_speed(const boat_params_t *p0, float T)
{
    boat_params_t p = *p0; p.wave_sigma = 0.0f; p.chop_sigma = 0.0f; p.cbal_walk = 0.0f; p.cbal_amp = 0.0f;
    boat_t b; boat_init(&b, &p);
    const float L = T * (1.0f - p.cbal0), R = T * (1.0f + p.cbal0);
    for (int i = 0; i < 2000; ++i) boat_step(&b, &p, L, R, -p.drift_e, -p.drift_n, 0);
    return b.v;
}

static void plant_check(const envelope_t *env, int runs, uint64_t seed)
{
    scenario_t nom = scenario_defaults();
    for (int k = 0; k < runs; ++k) {
        seed_rng(seed + (uint64_t)k);
        boat_params_t p; boat_draw(&p, env, &nom);
        const float v15 = plant_speed(&p, 0.15f), v20 = plant_speed(&p, 0.20f);
        const float v40 = plant_speed(&p, 0.40f), v45 = plant_speed(&p, 0.45f);
        boat_params_t pq = p; pq.wave_sigma = 0.0f; pq.chop_sigma = 0.0f; pq.cbal_walk = 0.0f; pq.cbal_amp = 0.0f;
        boat_t b; boat_init(&b, &pq);
        float t90 = -1.0f;
        for (int i = 0; i < 3000; ++i) {
            boat_step(&b, &pq, 0.40f * (1.0f - pq.cbal0), 0.40f * (1.0f + pq.cbal0), -pq.drift_e, -pq.drift_n, 0);
            if (t90 < 0.0f && b.v >= 0.9f * v40) t90 = (float)(i + 1) * DT;
        }
        float glide[2] = {0.0f, 0.0f}, half[2] = {-1.0f, -1.0f};
        const float Ts[2] = {0.40f, 0.20f};
        for (int g = 0; g < 2; ++g) {
            boat_init(&b, &pq);
            for (int i = 0; i < 2000; ++i)
                boat_step(&b, &pq, Ts[g] * (1.0f - pq.cbal0), Ts[g] * (1.0f + pq.cbal0), -pq.drift_e, -pq.drift_n, 0);
            const float e0 = b.e, n0 = b.n, v0 = b.v;
            for (int i = 0; i < 1500; ++i) {
                boat_step(&b, &pq, 0.0f, 0.0f, -pq.drift_e, -pq.drift_n, 0);
                if (i == 299) glide[g] = hypotf(b.e - e0, b.n - n0);
                if (half[g] < 0.0f && b.v <= 0.5f * v0) half[g] = (float)(i + 1) * DT;
            }
        }
        const straight_stats_t on20 = plant_straight(&p, 0.20f, 1);
        const straight_stats_t on40 = plant_straight(&p, 0.40f, 1);
        const straight_stats_t off40 = plant_straight(&p, 0.40f, 0);
        printf("{\"boat\":%d,\"K\":%.1f,\"tau\":%.2f,\"delay\":%.2f,\"cbal\":%.3f,\"v15\":%.3f,\"v20\":%.3f,"
               "\"v40\":%.3f,\"v45\":%.3f,\"t90_40\":%.2f,\"glide3_40\":%.3f,\"half_40\":%.2f,\"glide3_20\":%.3f,"
               "\"half_20\":%.2f,"
               "\"pon20_yaw_std\":%.2f,\"pon20_c_std\":%.3f,\"pon20_c_mean\":%.3f,\"pon20_hdg\":%.2f,\"pon20_off\":%.2f,"
               "\"pon40_yaw_std\":%.2f,\"pon40_c_std\":%.3f,\"pon40_c_mean\":%.3f,\"pon40_hdg\":%.2f,\"pon40_off\":%.2f,"
               "\"poff40_yaw_std\":%.2f,\"poff40_hdg\":%.2f}\n",
               k, p.K, p.tau, p.delay, p.cbal0, v15, v20, v40, v45, t90, glide[0], half[0], glide[1], half[1],
               on20.yaw_std, on20.c_std, on20.c_mean, on20.hdg_change, on20.max_off,
               on40.yaw_std, on40.c_std, on40.c_mean, on40.hdg_change, on40.max_off,
               off40.yaw_std, off40.hdg_change);
    }
}

/* ---- replay: recorded jet commands -> model yaw ---------------------------
 * FILE lines:  S,<id>,<c_pre>,<yaw_pre>   starts a segment (the boat was going
 *                                         straight at c_pre, turning at yaw_pre)
 *              P,<t>,<L>,<R>              recorded commands (held until the next P)
 * Output, for each boat and each P row:  <id>,<boat>,<t>,<model yaw deg/s>
 * Waves are off: the recorded yaw carries its own noise, the comparison
 * allows for it. */
#define REPLAY_MAX 4096
static int replay(const envelope_t *env, const char *path, int runs, uint64_t seed)
{
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 2; }
    static float tt[REPLAY_MAX], LL[REPLAY_MAX], RR[REPLAY_MAX];
    char line[256];
    int id = -1, n = 0; float c_pre = 0.0f, yaw_pre = 0.0f;
    scenario_t nom = scenario_defaults();
    int more = 1;
    long seg_count = 0;
    while (more) {
        char *got = fgets(line, sizeof(line), f);
        const bool seg_start = got && line[0] == 'S';
        if (!got || seg_start) {
            if (id >= 0 && n > 0) {
                seg_count++;
                for (int k = 0; k < runs; ++k) {
                    seed_rng(seed + (uint64_t)k);
                    boat_params_t p; boat_draw(&p, env, &nom);
                    p.wave_sigma = 0.0f; p.chop_sigma = 0.0f; p.cbal_walk = 0.0f; p.cbal_amp = 0.0f;
                    p.cbal0 = c_pre; p.floor_ = 0.0f;
                    boat_t b; boat_init(&b, &p);
                    b.r = yaw_pre; b.yaw_bias = yaw_pre;
                    float t = tt[0];
                    int j = 0;
                    for (int i = 0; i < n; ++i) {
                        while (t < tt[i]) {
                            boat_step(&b, &p, LL[j], RR[j], 0.0f, 0.0f, 0);
                            t += DT;
                        }
                        j = i;
                        printf("%d,%d,%.3f,%.3f\n", id, k, tt[i], boat_yaw(&b));
                    }
                }
            }
            if (!got) { more = 0; break; }
            if (sscanf(line, "S,%d,%f,%f", &id, &c_pre, &yaw_pre) != 3) { fclose(f); return 2; }
            n = 0;
        } else if (line[0] == 'P' && n < REPLAY_MAX) {
            if (sscanf(line, "P,%f,%f,%f", &tt[n], &LL[n], &RR[n]) == 3) n++;
        }
    }
    fclose(f);
    return seg_count > 0 ? 0 : 2;
}

int main(int argc, char **argv)
{
    envelope_t env = nominal_envelope();
    int runs = 200; uint64_t seed = 1; const char *trace_path = NULL; int verbose = 0;
    const char *only = NULL, *sweep = NULL, *replay_path = NULL; int catalogue = 0, plant = 0, list = 0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--runs") && i + 1 < argc) runs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = strtoull(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "--drift") && i + 1 < argc) env.drift_max = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--bias") && i + 1 < argc) env.bias_max = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--gps-white") && i + 1 < argc) env.gps_white = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--gps-wander") && i + 1 < argc) env.gps_wander = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--radius") && i + 1 < argc) env.settings.home_radius_m = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--throttle") && i + 1 < argc) env.settings.throttle = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) env.settings.out_distance_m = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--K") && i + 2 < argc) { env.K_min = strtof(argv[++i], NULL); env.K_max = strtof(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--tau") && i + 2 < argc) { env.tau_min = strtof(argv[++i], NULL); env.tau_max = strtof(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--delay") && i + 2 < argc) { env.delay_min = strtof(argv[++i], NULL); env.delay_max = strtof(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--chop") && i + 2 < argc) { env.chop_min = strtof(argv[++i], NULL); env.chop_max = strtof(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--wave") && i + 2 < argc) { env.wave_min = strtof(argv[++i], NULL); env.wave_max = strtof(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--left")) env.settings.turn_right = false;
        else if (!strcmp(argv[i], "--course-gate") && i + 1 < argc) g_course_gate_deg = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--approach") && i + 1 < argc) g_approach = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--sacc") && i + 1 < argc) env.sacc_mean = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--vel-noise") && i + 1 < argc) env.vel_noise = strtof(argv[++i], NULL);
        else if (!strcmp(argv[i], "--trace") && i + 1 < argc) trace_path = argv[++i];
        else if (!strcmp(argv[i], "--scenario") && i + 1 < argc) only = argv[++i];
        else if (!strcmp(argv[i], "--sweep") && i + 1 < argc) sweep = argv[++i];
        else if (!strcmp(argv[i], "--replay") && i + 1 < argc) replay_path = argv[++i];
        else if (!strcmp(argv[i], "--catalogue")) catalogue = 1;
        else if (!strcmp(argv[i], "--plant")) plant = 1;
        else if (!strcmp(argv[i], "--list")) list = 1;
        else if (!strcmp(argv[i], "-v")) verbose = 1;
        else { fprintf(stderr, "unknown option %s\n", argv[i]); return 2; }
    }
    if (runs < 1) runs = 1;
    static scenario_t cat[NSC];
    const int ncat = build_catalogue(cat);
    if (list) {
        for (int k = 0; k < ncat; ++k) printf("%s\n", cat[k].name);
        return 0;
    }
    if (plant) { plant_check(&env, runs, seed); return 0; }
    if (replay_path) return replay(&env, replay_path, runs, seed);
    if (sweep) {
        int found = 0;
        for (int k = 0; k < NSWEEPS; ++k) {
            if (!strcmp(sweep, "all") || !strcmp(sweep, SWEEPS[k].name)) { run_sweep(&SWEEPS[k], runs, seed); found = 1; }
        }
        if (!found) { fprintf(stderr, "unknown sweep %s\n", sweep); return 2; }
        return 0;
    }
    if (catalogue || only) {
        int failed = 0, found = 0;
        printf("%-20s %6s %5s %8s %6s  %s\n", "scenario", "pass", "need", "in zone", "d_med", "what");
        for (int k = 0; k < ncat; ++k) {
            if (only && strcmp(only, cat[k].name)) continue;
            found = 1;
            envelope_t e = env;
            if (cat[k].env_fn) cat[k].env_fn(&e);
            const stats_t s = run_set(&e, &cat[k], runs, seed, verbose);
            const float frac = (float)s.meet / (float)s.n;
            const bool pass = frac + 1e-6f >= cat[k].min_pass;
            failed += !pass;
            printf("%-20s %5.1f%% %4.0f%% %7.1f%% %6.2f  %s%s\n", cat[k].name, 100.0f * frac,
                   100.0f * cat[k].min_pass, 100.0f * (float)s.inzone / (float)s.n, s.d_med, cat[k].what,
                   pass ? "" : "   <-- FAILS");
        }
        if (!found) { fprintf(stderr, "unknown scenario %s\n", only ? only : ""); return 2; }
        printf("CATALOGUE %s (%d failing)\n", failed ? "FAILED" : "PASSED", failed);
        return failed ? 1 : 0;
    }

    scenario_t nom = scenario_defaults(); nom.name = "nominal";
    if (trace_path) {
        FILE *tr = fopen(trace_path, "w");
        if (tr) { (void)run_one(&env, &nom, seed, tr); fclose(tr); }
    }
    const stats_t s = run_set(&env, &nom, runs, seed, verbose);
    printf("{\"runs\":%d,\"drift_max\":%.2f,\"bias_max\":%.0f,\"radius\":%.1f,\"in_zone\":%.3f,\"done\":%d,"
           "\"refused\":%d,\"aborted\":%d,\"never_ended\":%d,\"spins\":%d,\"false_arrivals\":%d,\"warned\":%d,"
           "\"d_final_median\":%.2f,\"d_final_p95\":%.2f,\"time_median\":%.0f}\n",
           s.n, env.drift_max, env.bias_max, env.settings.home_radius_m, (double)s.inzone / s.n, s.done,
           s.refused, s.aborted, s.never, s.spins, s.false_arr, s.warned, s.d_med, s.d_p95, s.t_med);
    return 0;
}
