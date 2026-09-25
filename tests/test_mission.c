/* The out-and-back sequencer, driven by a small ideal boat: it turns toward
 * the wanted heading at up to 15 deg/s and moves at a speed set by throttle.
 * GPS fixes arrive every 0.1 s; the mission steps at 20 Hz. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "auto_drive.h"
#include "mission.h"

#define LAT0 21.0417483
#define LON0 105.8862132

typedef struct {
    nav_origin_t o;
    float e, n;          /* true position */
    float heading;       /* true = measured (ideal compass) */
    float yaw;           /* deg/s, + = left */
    float t;
    float drift_e, drift_n;
    float compass_bias;  /* measured - true */
    float gps_off_e, gps_off_n;   /* a GPS error on top of the true position */
    int tick;
} boat_t;

static mission_cfg_t cfg;

static void boat_init(boat_t *b, float heading)
{
    memset(b, 0, sizeof(*b));
    nav_origin_init(&b->o, LAT0, LON0);
    b->heading = heading;
}

static mission_input_t base_input(const boat_t *b)
{
    mission_input_t in;
    memset(&in, 0, sizeof(in));
    in.t_s = b->t;
    in.gps_ubx = true;
    in.fix_type = 3;
    in.sats = 12;
    in.pdop = 1.5f;
    in.fix_age_s = 0.05f;
    nav_to_latlon(&b->o, (nav_en_t){b->e, b->n}, &in.lat_deg, &in.lon_deg);
    in.speed_acc_mps = 0.05f;
    in.heading_valid = true;
    in.heading_deg = nav_wrap_360(b->heading + b->compass_bias);
    in.yaw_rate_dps = b->yaw;
    in.fusion_age_s = 0.02f;
    in.imu_ok = true;
    in.compass_calibrated = true;
    in.field_ratio = 1.0f;
    in.armed = true;
    in.heading_hold_built = true;
    in.link_alive = true;
    return in;
}

static float speed_for(float thr) { return thr <= 0.0f ? 0.0f : 0.09f + 0.55f * thr; }

/* One 50 ms tick: step the mission, then move the ideal boat. */
static mission_output_t tick(mission_t *m, boat_t *b, mission_input_t *in)
{
    in->t_s = b->t;
    in->gps_new = (b->tick % 2) == 0;
    nav_to_latlon(&b->o, (nav_en_t){b->e + b->gps_off_e, b->n + b->gps_off_n}, &in->lat_deg, &in->lon_deg);
    in->heading_deg = nav_wrap_360(b->heading + b->compass_bias);
    in->yaw_rate_dps = b->yaw;
    const mission_output_t out = mission_step(m, &cfg, in);
    const float dt = 0.05f;
    const float v = out.drive ? speed_for(out.throttle) : 0.0f;
    if (out.active && !out.dry_run) {
        const float want_true = nav_wrap_360(out.heading_deg - b->compass_bias);
        const float err = nav_wrap_180(want_true - b->heading);
        const float rate = fmaxf(-15.0f, fminf(15.0f, 1.5f * err));   /* + = clockwise */
        b->yaw = out.drive ? -rate : 0.0f;
        b->heading = nav_wrap_360(b->heading - b->yaw * dt);
    } else {
        b->yaw = 0.0f;
    }
    const float h = b->heading * 0.017453292f;
    b->e += (v * sinf(h) + b->drift_e) * dt;
    b->n += (v * cosf(h) + b->drift_n) * dt;
    in->speed_mps = hypotf(v * sinf(h) + b->drift_e, v * cosf(h) + b->drift_n);
    in->course_deg = nav_wrap_360(atan2f(v * sinf(h) + b->drift_e, v * cosf(h) + b->drift_n) * 57.2957795f);
    b->t += dt;
    b->tick++;
    in->start = false;
    in->stop = false;
    return out;
}

static void start(mission_t *m, boat_t *b, mission_input_t *in, uint32_t req, mission_settings_t s)
{
    (void)m;
    *in = base_input(b);
    in->start = true;
    in->request_id = req;
    in->settings = s;
}

static int run_until_terminal(mission_t *m, boat_t *b, mission_input_t *in, float max_s)
{
    while (b->t < max_s) {
        tick(m, b, in);
        if (m->state == MISSION_DONE || m->state == MISSION_ABORTED || m->state == MISSION_REFUSED) return 1;
    }
    return 0;
}

static void settings_are_validated(void)
{
    mission_settings_t s = mission_settings_default();
    assert(mission_settings_valid(&s));
    s.approach_throttle = 0.10f; assert(!mission_settings_valid(&s));
    s = mission_settings_default(); s.approach_throttle = 0.50f; assert(!mission_settings_valid(&s));
    s = mission_settings_default(); s.stage = 4; assert(!mission_settings_valid(&s));
    s = mission_settings_default(); s.out_distance_m = 5.0f; s.home_radius_m = 3.0f;
    assert(!mission_settings_valid(&s));                         /* zone does not fit */
    s = mission_settings_default(); s.throttle = NAN; assert(!mission_settings_valid(&s));
}

static void each_start_check_refuses_with_its_reason(void)
{
    boat_t b; mission_t m; mission_input_t in;
    const uint8_t want[] = { MISSION_REFUSE_NO_HOLD, MISSION_REFUSE_BUSY, MISSION_REFUSE_NOT_ARMED,
                             MISSION_REFUSE_GPS_NOT_UBX, MISSION_REFUSE_NO_3D_FIX, MISSION_REFUSE_FEW_SATS,
                             MISSION_REFUSE_PDOP, MISSION_REFUSE_FIX_OLD, MISSION_REFUSE_COMPASS,
                             MISSION_REFUSE_FIELD, MISSION_REFUSE_IMU, MISSION_REFUSE_HEADING };
    for (unsigned i = 0; i < sizeof(want); ++i) {
        boat_init(&b, 0.0f); mission_init(&m);
        start(&m, &b, &in, 100u + i, mission_settings_default());
        switch (i) {
        case 0: in.heading_hold_built = false; break;
        case 1: in.busy = true; break;
        case 2: in.armed = false; break;
        case 3: in.gps_ubx = false; break;
        case 4: in.fix_type = 2; break;
        case 5: in.sats = 7; break;
        case 6: in.pdop = 2.6f; break;
        case 7: in.fix_age_s = 0.6f; break;
        case 8: in.compass_calibrated = false; break;
        case 9: in.field_ratio = 1.5f; break;
        case 10: in.imu_ok = false; break;
        case 11: in.fusion_age_s = 0.6f; break;
        }
        mission_step(&m, &cfg, &in);
        assert(m.state == MISSION_REFUSED && m.reason == want[i] && m.run_id == 1u);
    }
    /* Bad settings come first. */
    boat_init(&b, 0.0f); mission_init(&m);
    mission_settings_t s = mission_settings_default(); s.throttle = 0.9f;
    start(&m, &b, &in, 9u, s);
    mission_step(&m, &cfg, &in);
    assert(m.state == MISSION_REFUSED && m.reason == MISSION_REFUSE_SETTINGS);
    /* A dry run does not need arming. */
    boat_init(&b, 0.0f); mission_init(&m);
    s = mission_settings_default(); s.dry_run = true;
    start(&m, &b, &in, 10u, s); in.armed = false;
    mission_step(&m, &cfg, &in);
    assert(m.state == MISSION_HOME);
}

static void repeats_and_second_starts_are_ignored(void)
{
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); mission_init(&m);
    start(&m, &b, &in, 42u, mission_settings_default());
    tick(&m, &b, &in);
    assert(m.state == MISSION_HOME && m.run_id == 1u);
    in.start = true; in.request_id = 42u; tick(&m, &b, &in);      /* the same press again */
    assert(m.run_id == 1u);
    in.start = true; in.request_id = 43u; tick(&m, &b, &in);      /* a new press mid-run */
    assert(m.run_id == 1u && m.request_id == 42u && m.state == MISSION_HOME);
}

static void a_full_mission_goes_out_turns_and_arrives_in_the_zone(void)
{
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 30.0f); mission_init(&m);
    start(&m, &b, &in, 1u, mission_settings_default());
    int seen_out = 0, seen_turn = 0, seen_return = 0, seen_approach = 0;
    float max_dist = 0.0f;
    while (b.t < 200.0f && m.state != MISSION_DONE && m.state != MISSION_ABORTED) {
        const mission_output_t o = tick(&m, &b, &in);
        seen_out |= m.state == MISSION_OUTBOUND;
        seen_turn |= m.state == MISSION_TURN;
        seen_return |= m.state == MISSION_RETURN;
        if (m.state == MISSION_RETURN && m.approach) {
            seen_approach = 1;
            assert(fabsf(o.throttle - 0.20f) < 1e-6f);
        }
        if (m.state == MISSION_OUTBOUND) assert(fabsf(o.heading_deg - m.start_heading_deg) < 1e-4f);
        max_dist = fmaxf(max_dist, hypotf(b.e, b.n));
    }
    assert(seen_out && seen_turn && seen_return && seen_approach);
    assert(m.state == MISSION_DONE && m.reason == MISSION_DONE_IN_ZONE);
    assert(fabsf(m.start_heading_deg - 30.0f) < 0.5f);
    assert(max_dist >= 10.0f && max_dist < 13.0f);
    assert(hypotf(b.e, b.n) <= 3.2f);
    assert(m.turned_deg > 150.0f && m.turned_deg < 230.0f);      /* turned RIGHT */
    assert(m.outages == 0u);
    const mission_output_t o = tick(&m, &b, &in);
    assert(!o.active && !o.drive && o.throttle == 0.0f);          /* motors off */
}

static void a_left_turn_turns_left(void)
{
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); mission_init(&m);
    mission_settings_t s = mission_settings_default(); s.turn_right = false;
    start(&m, &b, &in, 1u, s);
    assert(run_until_terminal(&m, &b, &in, 200.0f));
    assert(m.state == MISSION_DONE && m.turned_deg < -150.0f);
}

static void stages_stop_where_they_should(void)
{
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); mission_init(&m);
    mission_settings_t s = mission_settings_default(); s.stage = MISSION_STAGE_OUT;
    start(&m, &b, &in, 1u, s);
    assert(run_until_terminal(&m, &b, &in, 100.0f));
    assert(m.state == MISSION_DONE && m.reason == MISSION_DONE_OUT && m.dist_home_m >= 10.0f);
    boat_init(&b, 0.0f); mission_init(&m);
    s.stage = MISSION_STAGE_OUT_TURN;
    start(&m, &b, &in, 1u, s);
    assert(run_until_terminal(&m, &b, &in, 100.0f));
    assert(m.state == MISSION_DONE && m.reason == MISSION_DONE_TURNED);
    assert(fabsf(nav_wrap_180(in.heading_deg - nav_bearing_deg(m.pos, (nav_en_t){0, 0}))) < 25.0f);
}

static void a_compass_bias_is_learned_out_and_corrected_back(void)
{
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); b.compass_bias = -40.0f;           /* the old calibration */
    mission_init(&m);
    start(&m, &b, &in, 1u, mission_settings_default());
    int checked = 0;
    while (b.t < 200.0f && m.state != MISSION_DONE && m.state != MISSION_ABORTED) {
        tick(&m, &b, &in);
        if (m.state == MISSION_TURN && !checked) {
            checked = 1;
            assert(m.out_result.valid && fabsf(m.beta_deg - 40.0f) < 2.0f);   /* course - heading */
            assert(!m.compass_bad);                                         /* 40 < 45 */
        }
    }
    assert(checked && m.state == MISSION_DONE && m.reason == MISSION_DONE_IN_ZONE);
    assert(hypotf(b.e, b.n) <= 3.2f);
}

static void a_near_miss_ends_as_passed_or_near_never_circles(void)
{
    /* A 0.12 m/s cross current at T20-ish speeds pushes the boat past home
     * off the line; it must end, not orbit. */
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); b.drift_e = 0.12f;
    mission_init(&m);
    mission_settings_t s = mission_settings_default(); s.home_radius_m = 1.5f;
    start(&m, &b, &in, 1u, s);
    assert(run_until_terminal(&m, &b, &in, 300.0f));
    assert(m.state == MISSION_DONE);
    assert(m.reason == MISSION_DONE_IN_ZONE || m.reason == MISSION_DONE_PASSED ||
           m.reason == MISSION_DONE_NEAR);
    assert(m.closest_m <= 3.0f);
}

static void every_abort_reason_stops_the_run(void)
{
    for (int k = 0; k < 7; ++k) {
        boat_t b; mission_t m; mission_input_t in;
        boat_init(&b, 0.0f); mission_init(&m);
        start(&m, &b, &in, 1u, mission_settings_default());
        while (m.state != MISSION_OUTBOUND) tick(&m, &b, &in);
        for (int i = 0; i < 40; ++i) tick(&m, &b, &in);             /* 2 s out */
        switch (k) {
        case 0: in.stop = true; break;
        case 1: in.control_abort_run = m.run_id; in.control_abort_reason = AUTO_ABORT_MANUAL; break;
        case 2: in.fix_age_s = 2.5f; break;
        case 3: in.sats = 5; break;
        case 4: in.heading_valid = false; break;
        case 5: in.fusion_age_s = 0.8f; break;
        case 6: in.control_abort_run = m.run_id + 7u; in.control_abort_reason = AUTO_ABORT_STOP; break;
        }
        tick(&m, &b, &in);
        const uint8_t want[] = { MISSION_ABORT_STOP, MISSION_ABORT_MANUAL, MISSION_ABORT_GPS_LOST,
                                 MISSION_ABORT_GPS_LOST, MISSION_ABORT_HEADING_LOST,
                                 MISSION_ABORT_HEADING_LOST, 0 };
        if (k == 6) assert(m.state == MISSION_OUTBOUND);             /* another run's report */
        else assert(m.state == MISSION_ABORTED && m.reason == want[k]);
    }
}

static void radio_loss_is_counted_never_stopping(void)
{
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); mission_init(&m);
    start(&m, &b, &in, 1u, mission_settings_default());
    while (b.t < 200.0f && m.state != MISSION_DONE && m.state != MISSION_ABORTED) {
        in.link_alive = !(b.t > 5.0f && b.t < 25.0f) && !(b.t > 40.0f && b.t < 41.0f);
        tick(&m, &b, &in);
    }
    assert(m.state == MISSION_DONE && m.outages == 2u);
    assert(m.longest_outage_s > 19.0f && m.longest_outage_s < 20.5f);
}

static void a_spinning_boat_is_stopped(void)
{
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); mission_init(&m);
    start(&m, &b, &in, 1u, mission_settings_default());
    while (m.state != MISSION_OUTBOUND) tick(&m, &b, &in);
    /* Force a steady 30 deg/s rotation the mission did not ask for. */
    for (int i = 0; i < 400 && m.state == MISSION_OUTBOUND; ++i) {
        in.t_s = b.t; in.gps_new = false; in.yaw_rate_dps = 30.0f;
        mission_step(&m, &cfg, &in);
        b.t += 0.05f;
    }
    assert(m.state == MISSION_ABORTED && m.reason == MISSION_ABORT_SPIN);
}

static void a_dry_run_never_drives_but_walks_through_every_step(void)
{
    /* Carried at walking pace: the test boat moves on its own while the
     * mission holds drive = false. */
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 90.0f); mission_init(&m);
    mission_settings_t s = mission_settings_default(); s.dry_run = true;
    start(&m, &b, &in, 1u, s); in.armed = false;
    int states = 0;
    while (b.t < 300.0f && m.state != MISSION_DONE && m.state != MISSION_ABORTED) {
        in.t_s = b.t; in.gps_new = (b.tick % 2) == 0;
        nav_to_latlon(&b.o, (nav_en_t){b.e, b.n}, &in.lat_deg, &in.lon_deg);
        in.heading_deg = b.heading; in.yaw_rate_dps = b.yaw;
        const mission_output_t o = mission_step(&m, &cfg, &in);
        in.start = false;
        assert(!o.drive && o.throttle == 0.0f);
        states |= 1 << m.state;
        /* The operator walks: follow the wanted heading at 0.4 m/s once out of HOME. */
        if (m.state != MISSION_HOME && m.state != MISSION_DONE) {
            const float err = nav_wrap_180(o.heading_deg - b.heading);
            const float rate = fmaxf(-20.0f, fminf(20.0f, 1.5f * err));
            b.yaw = -rate; b.heading = nav_wrap_360(b.heading + rate * 0.05f);
            const float h = b.heading * 0.017453292f;
            b.e += 0.4f * sinf(h) * 0.05f; b.n += 0.4f * cosf(h) * 0.05f;
            in.speed_mps = 0.4f; in.course_deg = b.heading;
        }
        b.t += 0.05f; b.tick++;
    }
    assert(m.state == MISSION_DONE);
    assert(states & (1 << MISSION_OUTBOUND) && states & (1 << MISSION_TURN) && states & (1 << MISSION_RETURN));
}

static void the_control_reason_codes_match_auto_drive(void)
{
    assert(mission_reason_from_control(AUTO_ABORT_STOP) == MISSION_ABORT_STOP);
    assert(mission_reason_from_control(AUTO_ABORT_MANUAL) == MISSION_ABORT_MANUAL);
    assert(mission_reason_from_control(AUTO_ABORT_DISARMED) == MISSION_ABORT_DISARMED);
    assert(mission_reason_from_control(AUTO_ABORT_RAIL_CUT) == MISSION_ABORT_RAIL_CUT);
    assert(mission_reason_from_control(AUTO_ABORT_OTHER_OWNER) == MISSION_ABORT_OTHER_OWNER);
    assert(mission_reason_from_control(AUTO_ABORT_STALE) == MISSION_ABORT_STALE);
}

static void home_restarts_when_the_boat_moves_and_gives_up_after_5_s(void)
{
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); mission_init(&m);
    start(&m, &b, &in, 1u, mission_settings_default());
    tick(&m, &b, &in);                                   /* START lands: HOME */
    assert(m.state == MISSION_HOME);
    /* The GPS jumps between two spots 4 m apart (spread 2 m > 1.5 m): never still. */
    while (b.t < 6.0f && m.state == MISSION_HOME) {
        b.e = (b.tick % 4 < 2) ? 0.0f : 4.0f;
        tick(&m, &b, &in);
    }
    assert(m.state == MISSION_ABORTED && m.reason == MISSION_ABORT_HOME_NOT_STILL);
}

static void a_frozen_gps_stops_the_run(void)
{
    /* A stuck receiver (or u-blox static hold): fixes keep coming, fresh and
     * with speed, but the position never changes.  With no time limit the
     * mission would never notice by itself. */
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); mission_init(&m);
    start(&m, &b, &in, 1u, mission_settings_default());
    while (m.state != MISSION_OUTBOUND) tick(&m, &b, &in);
    for (int i = 0; i < 40; ++i) tick(&m, &b, &in);              /* 2 s out, moving */
    const double lat = in.lat_deg, lon = in.lon_deg;             /* the last fix, stuck */
    const float t_frozen = b.t;
    while (m.state == MISSION_OUTBOUND && b.t < t_frozen + 5.0f) {
        in.t_s = b.t; in.gps_new = (b.tick % 2) == 0;
        in.lat_deg = lat; in.lon_deg = lon; in.speed_mps = 0.35f;
        mission_step(&m, &cfg, &in);
        b.t += 0.05f; b.tick++;
    }
    assert(m.state == MISSION_ABORTED && m.reason == MISSION_ABORT_GPS_FROZEN);
    assert(b.t - t_frozen <= 2.2f);                              /* 20 fixes */
    /* A boat that really stands still (GPS speed under 0.15 m/s) is not frozen. */
    boat_init(&b, 0.0f); mission_init(&m);
    start(&m, &b, &in, 2u, mission_settings_default());
    while (m.state != MISSION_OUTBOUND) tick(&m, &b, &in);
    const float t_still = b.t;
    while (m.state == MISSION_OUTBOUND && b.t < t_still + 5.0f) {
        in.t_s = b.t; in.gps_new = (b.tick % 2) == 0; in.speed_mps = 0.05f;
        mission_step(&m, &cfg, &in);
        b.t += 0.05f; b.tick++;
    }
    assert(m.state == MISSION_OUTBOUND);
}

static void a_gps_jump_toward_home_is_not_an_arrival(void)
{
    /* In the return, ~5.5 m out, the GPS jumps 4 m toward home for 1 s: it
     * would read "in the zone" three fixes running.  The jump is not used
     * (the position is dead-reckoned), so it cannot end the mission. */
    boat_t b; mission_t m; mission_input_t in;
    boat_init(&b, 0.0f); mission_init(&m);
    start(&m, &b, &in, 1u, mission_settings_default());
    while (m.state != MISSION_RETURN) tick(&m, &b, &in);
    while (m.state == MISSION_RETURN && hypotf(b.e, b.n) > 5.5f) tick(&m, &b, &in);
    assert(m.state == MISSION_RETURN);
    const float d = hypotf(b.e, b.n);
    b.gps_off_e = -4.0f * b.e / d; b.gps_off_n = -4.0f * b.n / d;
    const uint32_t before = m.outliers;
    for (int i = 0; i < 20; ++i) tick(&m, &b, &in);              /* 1 s = 10 fixes */
    assert(m.state == MISSION_RETURN);
    assert(m.outliers >= before + 8u);
    b.gps_off_e = b.gps_off_n = 0.0f;
    assert(run_until_terminal(&m, &b, &in, 300.0f));
    assert(m.state == MISSION_DONE && hypotf(b.e, b.n) <= 3.2f);
}

int main(void)
{
    cfg = mission_cfg_default();
    settings_are_validated();
    each_start_check_refuses_with_its_reason();
    repeats_and_second_starts_are_ignored();
    a_full_mission_goes_out_turns_and_arrives_in_the_zone();
    a_left_turn_turns_left();
    stages_stop_where_they_should();
    a_compass_bias_is_learned_out_and_corrected_back();
    a_near_miss_ends_as_passed_or_near_never_circles();
    every_abort_reason_stops_the_run();
    radio_loss_is_counted_never_stopping();
    a_spinning_boat_is_stopped();
    a_dry_run_never_drives_but_walks_through_every_step();
    the_control_reason_codes_match_auto_drive();
    home_restarts_when_the_boat_moves_and_gives_up_after_5_s();
    a_frozen_gps_stops_the_run();
    a_gps_jump_toward_home_is_not_an_arrival();
    printf("mission tests passed\n");
    return 0;
}
