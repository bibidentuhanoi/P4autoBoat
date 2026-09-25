#include "mission.h"

#include "planner.h"

#include <math.h>
#include <string.h>

#define M_DEG2RAD 0.017453292519943295
#define M_RAD2DEG 57.29577951308232

/* auto_drive.h's auto_abort_t, repeated as numbers so this module stays free
 * of the control-side header.  tests/test_mission.c pins the two together. */
#define CTRL_ABORT_STOP 1
#define CTRL_ABORT_MANUAL 2
#define CTRL_ABORT_DISARMED 3
#define CTRL_ABORT_RAIL_CUT 4
#define CTRL_ABORT_OTHER_OWNER 5
#define CTRL_ABORT_STALE 6

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

mission_cfg_t mission_cfg_default(void)
{
    mission_cfg_t c = {
        .home_fixes = 15u,
        .home_spread_m = 1.5f,
        .home_timeout_s = 5.0f,
        .start_min_sats = 8u,
        .start_max_pdop = 2.5f,
        .start_max_fix_age_s = 0.5f,
        .run_min_sats = 6u,
        .run_max_fix_age_s = 2.0f,
        .max_fusion_age_s = 0.5f,
        .field_ratio_min = 0.6f,
        .field_ratio_max = 1.4f,
        .turn_force_deg = 90.0f,
        .turn_lead_deg = 60.0f,
        .turn_done_deg = 20.0f,
        .approach_factor = 2.0f,
        .lookahead_m = 3.0f,
        .arrive_fixes = 3u,
        .slip_margin_m = 1.0f,
        .spin_limit_deg = 360.0f,
        .outbound_settle_s = 3.0f,
        .frozen_speed_mps = 0.15f,
        .frozen_fixes = 20u,
        .frozen_move_m = 0.02f,
        .jump_gate_m = 1.0f,
        .jump_accept_fixes = 20u,
        .course = course_err_cfg_default(),
    };
    return c;
}

mission_settings_t mission_settings_default(void)
{
    mission_settings_t s = {
        .stage = MISSION_STAGE_FULL,
        .out_distance_m = 10.0f,
        .home_radius_m = 2.5f,         /* motors off here: truly inside 3 m (sim) */
        .throttle = 0.40f,
        .approach_throttle = 0.20f,
        .turn_right = true,
        .dry_run = false,
    };
    return s;
}

bool mission_settings_valid(const mission_settings_t *s)
{
    if (!s) return false;
    if (s->stage < MISSION_STAGE_OUT || s->stage > MISSION_STAGE_FULL) return false;
    if (!isfinite(s->out_distance_m) || s->out_distance_m < 3.0f || s->out_distance_m > 50.0f) return false;
    if (!isfinite(s->home_radius_m) || s->home_radius_m < 1.5f || s->home_radius_m > 10.0f) return false;
    if (!isfinite(s->throttle) || s->throttle < 0.20f || s->throttle > 0.60f) return false;
    /* 0.15 is the heading hold's own floor: below it the hold switches off. */
    if (!isfinite(s->approach_throttle) || s->approach_throttle < 0.15f ||
        s->approach_throttle > s->throttle) return false;
    /* The home zone must fit well inside the trip, or HOME and the finish
     * would overlap. */
    if (s->out_distance_m < 2.0f * s->home_radius_m) return false;
    return true;
}

void mission_init(mission_t *m)
{
    if (m) memset(m, 0, sizeof(*m));
}

bool mission_is_active(const mission_t *m)
{
    return m && (m->state == MISSION_HOME || m->state == MISSION_OUTBOUND ||
                 m->state == MISSION_TURN || m->state == MISSION_RETURN);
}

uint8_t mission_reason_from_control(uint8_t auto_abort)
{
    switch (auto_abort) {
    case CTRL_ABORT_STOP:        return MISSION_ABORT_STOP;
    case CTRL_ABORT_MANUAL:      return MISSION_ABORT_MANUAL;
    case CTRL_ABORT_DISARMED:    return MISSION_ABORT_DISARMED;
    case CTRL_ABORT_RAIL_CUT:    return MISSION_ABORT_RAIL_CUT;
    case CTRL_ABORT_OTHER_OWNER: return MISSION_ABORT_OTHER_OWNER;
    case CTRL_ABORT_STALE:       return MISSION_ABORT_STALE;
    default:                     return MISSION_ABORT_STALE;
    }
}

static void enter(mission_t *m, mission_state_t s, float t_s)
{
    m->state = s;
    m->t_state_s = t_s;
    m->leg_turn_deg = 0.0f;
}

static void finish(mission_t *m, mission_state_t s, uint8_t reason, float t_s)
{
    enter(m, s, t_s);
    m->reason = reason;
    m->approach = false;
}

static uint8_t refusal(const mission_cfg_t *cfg, const mission_input_t *in)
{
    if (!mission_settings_valid(&in->settings)) return MISSION_REFUSE_SETTINGS;
    if (!in->heading_hold_built) return MISSION_REFUSE_NO_HOLD;
    if (in->busy) return MISSION_REFUSE_BUSY;
    if (!in->armed && !in->settings.dry_run) return MISSION_REFUSE_NOT_ARMED;
    if (!in->gps_ubx) return MISSION_REFUSE_GPS_NOT_UBX;
    if (in->fix_type < 3u) return MISSION_REFUSE_NO_3D_FIX;
    if (in->sats < cfg->start_min_sats) return MISSION_REFUSE_FEW_SATS;
    if (!(in->pdop > 0.0f) || in->pdop > cfg->start_max_pdop) return MISSION_REFUSE_PDOP;
    if (!(in->fix_age_s >= 0.0f) || in->fix_age_s > cfg->start_max_fix_age_s) return MISSION_REFUSE_FIX_OLD;
    if (!in->compass_calibrated) return MISSION_REFUSE_COMPASS;
    if (!(in->field_ratio >= cfg->field_ratio_min && in->field_ratio <= cfg->field_ratio_max)) {
        return MISSION_REFUSE_FIELD;
    }
    if (!in->imu_ok) return MISSION_REFUSE_IMU;
    if (!in->heading_valid || !(in->fusion_age_s <= cfg->max_fusion_age_s)) return MISSION_REFUSE_HEADING;
    return MISSION_REASON_NONE;
}

static void reset_run(mission_t *m)
{
    const uint32_t run_id = m->run_id, request_id = m->request_id;
    const bool any = m->any_request;
    const float t_last = m->t_last_s;
    const bool have_t = m->have_t;
    memset(m, 0, sizeof(*m));
    m->run_id = run_id;
    m->request_id = request_id;
    m->any_request = any;
    m->t_last_s = t_last;
    m->have_t = have_t;
}

static void handle_start(mission_t *m, const mission_cfg_t *cfg, const mission_input_t *in)
{
    if (mission_is_active(m)) return;                        /* one mission at a time */
    if (m->any_request && in->request_id == m->request_id) return;   /* a repeat */
    m->run_id++;
    m->request_id = in->request_id;
    m->any_request = true;
    reset_run(m);
    m->settings = in->settings;
    m->t_start_s = in->t_s;
    const uint8_t why = refusal(cfg, in);
    if (why != MISSION_REASON_NONE) {
        finish(m, MISSION_REFUSED, why, in->t_s);
        return;
    }
    enter(m, MISSION_HOME, in->t_s);
    m->t_home_start_s = in->t_s;
    m->reason = MISSION_REASON_NONE;
}

static bool gps_lost(const mission_cfg_t *cfg, const mission_input_t *in)
{
    return in->fix_type < 3u || in->sats < cfg->run_min_sats ||
           !(in->fix_age_s <= cfg->run_max_fix_age_s);
}

static bool heading_lost(const mission_cfg_t *cfg, const mission_input_t *in)
{
    return !in->heading_valid || !in->imu_ok ||
           !(in->fusion_age_s <= cfg->max_fusion_age_s) || !isfinite(in->heading_deg);
}

/* One new fix.  A fix more than jump_gate_m from where the last one and its
 * velocity say the boat is now is a GPS jump: it is not used, the position is
 * dead-reckoned instead, and it does not count toward any arrival rule.  A
 * jump that persists jump_accept_fixes is accepted as real.  Returns true
 * when the fix was accepted. */
static bool update_position(mission_t *m, const mission_cfg_t *cfg, const mission_input_t *in)
{
    const nav_en_t fix = nav_to_local(&m->origin, in->lat_deg, in->lon_deg);
    /* A GPS that keeps reporting speed while its position never changes is
     * stuck: without a position the mission cannot navigate (and with no
     * time limit it would never notice on its own). */
    if (m->have_pos && in->speed_mps > cfg->frozen_speed_mps &&
        nav_distance_m(fix, m->frozen_ref) < cfg->frozen_move_m) {
        m->frozen_count++;
    } else {
        m->frozen_count = 0u;
        m->frozen_ref = fix;
    }
    bool accepted = true;
    if (m->have_pos && cfg->jump_gate_m > 0.0f) {
        const float dt = clampf(in->t_s - m->t_fix_s, 0.0f, 1.0f);
        const nav_en_t pred = { m->pos.e + m->vel_e * dt, m->pos.n + m->vel_n * dt };
        if (nav_distance_m(fix, pred) > cfg->jump_gate_m &&
            m->outlier_run < cfg->jump_accept_fixes) {
            m->outlier_run++;
            m->outliers++;
            m->pos = pred;
            accepted = false;
        }
    }
    if (accepted) {
        m->pos = fix;
        m->outlier_run = 0u;
    }
    m->t_fix_s = in->t_s;
    if (isfinite(in->speed_mps) && isfinite(in->course_deg)) {
        m->vel_e = in->speed_mps * sinf(in->course_deg * 0.017453292f);
        m->vel_n = in->speed_mps * cosf(in->course_deg * 0.017453292f);
    }
    m->have_pos = true;
    m->dist_home_m = hypotf(m->pos.e, m->pos.n);
    return accepted;
}

/* ---- HOME: average 15 still fixes; restart if they are spread out ------- */
static void step_home(mission_t *m, const mission_cfg_t *cfg, const mission_input_t *in)
{
    m->wanted_heading_deg = in->heading_deg;       /* shown only; motors are 0 */
    m->wanted_course_deg = in->heading_deg;
    if (in->heading_valid && isfinite(in->heading_deg)) {
        m->hdg_sin += sin((double)in->heading_deg * M_DEG2RAD);
        m->hdg_cos += cos((double)in->heading_deg * M_DEG2RAD);
    }
    if (in->gps_new) {
        if (m->home_n == 0) nav_origin_init(&m->home_try_origin, in->lat_deg, in->lon_deg);
        if (m->home_n < MISSION_HOME_MAX) {
            m->home_samples[m->home_n++] = nav_to_local(&m->home_try_origin, in->lat_deg, in->lon_deg);
        }
        if (m->home_n >= cfg->home_fixes) {
            nav_en_t mean = {0.0f, 0.0f};
            for (uint32_t i = 0; i < m->home_n; ++i) {
                mean.e += m->home_samples[i].e;
                mean.n += m->home_samples[i].n;
            }
            mean.e /= (float)m->home_n;
            mean.n /= (float)m->home_n;
            float spread = 0.0f;
            for (uint32_t i = 0; i < m->home_n; ++i) {
                spread = fmaxf(spread, nav_distance_m(mean, m->home_samples[i]));
            }
            if (spread <= cfg->home_spread_m && (m->hdg_sin != 0.0 || m->hdg_cos != 0.0)) {
                nav_to_latlon(&m->home_try_origin, mean, &m->home_lat_deg, &m->home_lon_deg);
                nav_origin_init(&m->origin, m->home_lat_deg, m->home_lon_deg);
                m->have_origin = true;
                m->start_heading_deg = nav_wrap_360((float)(atan2(m->hdg_sin, m->hdg_cos) * M_RAD2DEG));
                const nav_en_t home = {0.0f, 0.0f};
                m->out_point = nav_offset(home, m->start_heading_deg, m->settings.out_distance_m);
                (void)update_position(m, cfg, in);
                course_err_mean_reset(&m->out_mean);
                enter(m, MISSION_OUTBOUND, in->t_s);
                return;
            }
            /* The boat moved (or the GPS jumped) while home was being taken:
             * start the average again, heading included. */
            m->home_n = 0;
            m->hdg_sin = m->hdg_cos = 0.0;
        }
    }
    if (in->t_s - m->t_home_start_s > cfg->home_timeout_s) {
        finish(m, MISSION_ABORTED, MISSION_ABORT_HOME_NOT_STILL, in->t_s);
    }
}

/* ---- OUTBOUND: hold the start heading; learn beta; stop at out distance -- */
static void step_outbound(mission_t *m, const mission_cfg_t *cfg, const mission_input_t *in)
{
    m->wanted_heading_deg = m->start_heading_deg;
    m->wanted_course_deg = m->start_heading_deg;
    if (!in->gps_new) return;
    const bool good_fix = update_position(m, cfg, in);
    m->dist_target_m = nav_distance_m(m->pos, m->out_point);
    m->bearing_target_deg = nav_bearing_deg(m->pos, m->out_point);
    if (good_fix && in->t_s - m->t_state_s >= cfg->outbound_settle_s &&
        course_err_sample_ok(&cfg->course, in->speed_mps, in->speed_acc_mps, in->yaw_rate_dps)) {
        course_err_mean_add(&m->out_mean, course_err_sample(in->course_deg, in->heading_deg),
                            m->pos.e, m->pos.n);
    }
    if (good_fix && m->dist_home_m >= m->settings.out_distance_m) {
        m->out_result = course_err_mean_result(&m->out_mean, &cfg->course);
        m->beta_valid = m->out_result.valid;
        m->beta_deg = m->out_result.valid ? m->out_result.mean_deg : 0.0f;
        m->compass_bad = m->out_result.compass_bad;
        if (m->settings.stage == MISSION_STAGE_OUT) {
            finish(m, MISSION_DONE, MISSION_DONE_OUT, in->t_s);
            return;
        }
        m->turn_start = m->pos;
        m->turned_deg = 0.0f;
        m->turn_peak_dps = 0.0f;
        m->t_turn_start_s = in->t_s;
        enter(m, MISSION_TURN, in->t_s);
    }
}

/* ---- TURN: pivot-style, forced direction until within turn_force_deg ---- */
static void step_turn(mission_t *m, const mission_cfg_t *cfg, const mission_input_t *in, float dt)
{
    m->turned_deg += -in->yaw_rate_dps * dt;          /* + = right */
    m->turn_peak_dps = fmaxf(m->turn_peak_dps, fabsf(in->yaw_rate_dps));
    if (in->gps_new) (void)update_position(m, cfg, in);
    const nav_en_t home = {0.0f, 0.0f};
    m->wanted_course_deg = nav_bearing_deg(m->pos, home);
    m->dist_target_m = m->dist_home_m;
    m->bearing_target_deg = m->wanted_course_deg;
    const float target = nav_wrap_360(m->wanted_course_deg - m->beta_deg);
    const float err = nav_wrap_180(target - in->heading_deg);
    if (fabsf(err) > cfg->turn_force_deg) {
        const float lead = m->settings.turn_right ? cfg->turn_lead_deg : -cfg->turn_lead_deg;
        m->wanted_heading_deg = nav_wrap_360(in->heading_deg + lead);
    } else {
        m->wanted_heading_deg = target;
    }
    if (fabsf(err) < cfg->turn_done_deg) {
        if (m->settings.stage == MISSION_STAGE_OUT_TURN) {
            finish(m, MISSION_DONE, MISSION_DONE_TURNED, in->t_s);
            return;
        }
        m->return_start = m->pos;
        /* The Planner slot: today the straight line home. */
        m->path = planner_straight(m->return_start, (nav_en_t){0.0f, 0.0f});
        path_follow_reset(&m->follow);
        m->closest_m = m->dist_home_m;
        m->zone_count = m->pass_count = m->slip_count = 0;
        m->t_beta_last_s = in->t_s;
        enter(m, MISSION_RETURN, in->t_s);
    }
}

/* ---- RETURN: follow the path turn-end -> home; slow approach; arrive ----- */
static void step_return(mission_t *m, const mission_cfg_t *cfg, const mission_input_t *in)
{
    const float approach_r = cfg->approach_factor * m->settings.home_radius_m;
    if (in->gps_new) {
        const bool good_fix = update_position(m, cfg, in);
        /* Guidance: follow the planner's path home (path_follow.c -- past
         * its end without arriving, it points at home instead). */
        const path_follow_out_t pf = path_follow_step(&m->follow, &m->path, m->pos,
                                                      cfg->lookahead_m);
        m->line = pf.line;
        m->dist_target_m = m->dist_home_m;
        m->bearing_target_deg = pf.bearing_to_end_deg;
        m->wanted_course_deg = pf.course_deg;
        m->approach = m->dist_home_m <= approach_r;
        if (!good_fix) {
            m->wanted_heading_deg = nav_wrap_360(m->wanted_course_deg - m->beta_deg);
            return;                       /* a jump never counts toward arriving */
        }

        /* beta keeps learning on the way back -- until the last metres, where
         * the course bends toward home and says nothing about the compass. */
        if (!m->approach &&
            course_err_sample_ok(&cfg->course, in->speed_mps, in->speed_acc_mps, in->yaw_rate_dps)) {
            /* Always a gradual follow from where it stands (the outbound
             * value, or 0): one noisy fix must never set the correction.  The
             * step is capped as if at most 0.2 s had passed, so the first
             * sample after a gated-out stretch cannot jump either. */
            const float sample = course_err_sample(in->course_deg, in->heading_deg);
            const float dt_used = clampf(in->t_s - m->t_beta_last_s, 0.0f, 0.2f);
            m->beta_deg = course_err_follow(m->beta_deg, sample, dt_used, &cfg->course);
            m->t_beta_last_s = in->t_s;
            m->beta_return_n++;
            if (m->beta_return_n >= 20u) m->beta_valid = true;   /* ~2 s of samples */
        }

        /* Arrival: three rules, each confirmed on consecutive fixes. */
        const bool in_zone = m->dist_home_m <= m->settings.home_radius_m;
        const bool passed = m->line.progress >= 1.0f && m->dist_home_m <= approach_r;
        if (m->dist_home_m < m->closest_m) m->closest_m = m->dist_home_m;
        const bool slipping = m->closest_m <= approach_r &&
                              m->dist_home_m >= m->closest_m + cfg->slip_margin_m;
        m->zone_count = in_zone ? m->zone_count + 1u : 0u;
        m->pass_count = passed ? m->pass_count + 1u : 0u;
        m->slip_count = slipping ? m->slip_count + 1u : 0u;
        if (m->zone_count >= cfg->arrive_fixes) {
            finish(m, MISSION_DONE, MISSION_DONE_IN_ZONE, in->t_s);
            return;
        }
        if (m->pass_count >= cfg->arrive_fixes) {
            finish(m, MISSION_DONE, MISSION_DONE_PASSED, in->t_s);
            return;
        }
        if (m->slip_count >= cfg->arrive_fixes) {
            finish(m, MISSION_DONE, MISSION_DONE_NEAR, in->t_s);
            return;
        }
    }
    m->wanted_heading_deg = nav_wrap_360(m->wanted_course_deg - m->beta_deg);
}

mission_output_t mission_step(mission_t *m, const mission_cfg_t *cfg, const mission_input_t *in)
{
    mission_output_t out = {0};
    if (!m || !cfg || !in) return out;
    const float dt = m->have_t ? clampf(in->t_s - m->t_last_s, 0.0f, 0.5f) : 0.0f;
    m->t_last_s = in->t_s;
    m->have_t = true;

    if (in->start) handle_start(m, cfg, in);

    if (mission_is_active(m)) {
        /* Radio outages never stop the mission; they are only counted. */
        if (!in->link_alive) {
            if (!m->in_outage) {
                m->in_outage = true;
                m->t_outage_start_s = in->t_s;
                m->outages++;
            }
            m->longest_outage_s = fmaxf(m->longest_outage_s, in->t_s - m->t_outage_start_s);
        } else {
            m->in_outage = false;
        }

        if (in->control_abort_run == m->run_id && in->control_abort_reason != 0u) {
            finish(m, MISSION_ABORTED, mission_reason_from_control(in->control_abort_reason), in->t_s);
        } else if (in->stop) {
            finish(m, MISSION_ABORTED, MISSION_ABORT_STOP, in->t_s);
        } else if (gps_lost(cfg, in)) {
            finish(m, MISSION_ABORTED, MISSION_ABORT_GPS_LOST, in->t_s);
        } else if (heading_lost(cfg, in)) {
            finish(m, MISSION_ABORTED, MISSION_ABORT_HEADING_LOST, in->t_s);
        } else if (m->frozen_count >= cfg->frozen_fixes) {
            finish(m, MISSION_ABORTED, MISSION_ABORT_GPS_FROZEN, in->t_s);
        } else {
            if (m->state != MISSION_HOME) {
                m->leg_turn_deg += -in->yaw_rate_dps * dt;
                if (fabsf(m->leg_turn_deg) > cfg->spin_limit_deg) {
                    finish(m, MISSION_ABORTED, MISSION_ABORT_SPIN, in->t_s);
                }
            }
            switch (m->state) {
            case MISSION_HOME:     step_home(m, cfg, in); break;
            case MISSION_OUTBOUND: step_outbound(m, cfg, in); break;
            case MISSION_TURN:     step_turn(m, cfg, in, dt); break;
            case MISSION_RETURN:   step_return(m, cfg, in); break;
            default: break;
            }
        }
    }

    out.active = mission_is_active(m);
    out.dry_run = m->settings.dry_run;
    out.drive = out.active && m->state != MISSION_HOME && !m->settings.dry_run;
    out.heading_deg = m->wanted_heading_deg;
    out.throttle = out.drive ? (m->approach ? m->settings.approach_throttle : m->settings.throttle) : 0.0f;
    return out;
}
