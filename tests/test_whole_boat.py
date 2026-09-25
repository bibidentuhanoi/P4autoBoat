"""The whole boat on the host: the REAL control task and the REAL mission task
together, built with the REAL sdkconfig.h of the firmware that gets flashed,
driving a simulated boat, commanded the way the laptop tool commands it.

Kiet, 2026-09-25: "trace from boot to hit mission 10m run, after that if i hit
base test or lake test, would that work or if i do base test, lake test, motor
mismatch, manual drive then i start the mission would that break anything" --
"test it extensively i want it to work".

Two compilation units, linked into one program, because the two halves were
written against different stub worlds:
  * CONTROL: main/motor_control.c + everything it links (arbiter, arm sequence,
    ESC trim, bench, trim learner, heading hold, AUTO owner, SAS, the real
    runtime_metrics.c) on the shared harness stubs of
    tests/test_runtime_architecture.py -- the proven way to run it -- plus
    this file's plant (the boat), the laptop's 15 Hz stream and the scheduler;
  * MISSION: main/autonomy.c + mission / navigation / planner / guidance on the
    stubs of tests/test_autonomy_task.py (the real generated protobuf types).
They meet only where the firmware's two tasks meet: motor_control_* (the
AUTO owner's API), the GPS/fusion getters and the SD card -- all with the REAL
struct layouts (the real gps_driver.h in both units).

The plant is the boat as the 09-20/21 lake data describe it: yaw ~57 deg/s per
unit split c, a first-order lag of 1.2 s, the right jet the weak one (equal
commands turn it right; about c = 0.20 goes straight), ~0.38 m/s at 40 %.
Right jet stronger = turn left (Kiet's rule; + yaw = left on this boat).

Built with AddressSanitizer + UndefinedBehaviorSanitizer: a memory error or
undefined behaviour anywhere in the real firmware code fails the run.
"""
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from tests.test_heading_hold_boot import FUSION_H, heading_hold_stubs
from tests.test_runtime_architecture import STUB_HEADERS
from tests.test_autonomy_task import HEADERS as MISSION_HEADERS, NANOPB

ROOT = Path(__file__).resolve().parents[1]
SDKCONFIG_H = ROOT / "build" / "config" / "sdkconfig.h"

# ---- the CONTROL unit's stubs: the shared ones, with the parts the whole boat
# needs made real. Each original must occur exactly once. ----------------------
CONTROL_SWAPS = (
    ("esp_err_t gps_driver_get_fix(gps_fix_t *fix) { (void)fix; return ESP_OK; }",
     "static gps_fix_t gps_now;\n"
     "static gps_protocol_authority_t gps_protocol = GPS_PROTOCOL_UBX;\n"
     "esp_err_t gps_driver_get_fix(gps_fix_t *fix) { *fix = gps_now; return ESP_OK; }\n"
     "esp_err_t gps_driver_get_runtime_status(gps_runtime_status_t *out) {\n"
     "    memset(out, 0, sizeof(*out)); out->protocol_authority = gps_protocol;\n"
     "    out->last_frame_us = gps_now.last_update_us; return ESP_OK;\n"
     "}"),
    ("    } else {\n"
     "        assert(id == RUNTIME_TASK_ARM_SEQUENCE);\n"
     "        arm_sequence_fn = fn;\n",
     "    } else if (id == RUNTIME_TASK_AUTONOMY) {\n"
     "        autonomy_fn = fn;               /* created; its loop is driven below */\n"
     "        if (out) *out = (TaskHandle_t)4;\n"
     "    } else {\n"
     "        assert(id == RUNTIME_TASK_ARM_SEQUENCE);\n"
     "        arm_sequence_fn = fn;\n"),
    ("static TaskFunction_t arm_sequence_fn;\n",
     "static TaskFunction_t arm_sequence_fn;\nstatic TaskFunction_t autonomy_fn;\n"),
    # main/drivers/esc_driver.c: a write while not ARMED is refused, the pulse unchanged
    ("esp_err_t esc_driver_set_throttle(float left, float right) { esc_left = left; esc_right = right; ++throttle_writes; return ESP_OK; }",
     "static unsigned refused_disarmed_writes, refused_nonzero_writes;\n"
     "static unsigned cycle_writes; static bool cycle_zero_then_drive;\n"
     "esp_err_t esc_driver_set_throttle(float left, float right) {\n"
     "    if (esc_state != ESC_STATE_ARMED) {\n"
     "        ++refused_disarmed_writes; if (left != 0.0f || right != 0.0f) ++refused_nonzero_writes;\n"
     "        return ESP_ERR_INVALID_STATE;\n"
     "    }\n"
     "    /* a zero, then a drive, inside one control cycle: the MCPWM latches the\n"
     "     * compare value at each period start, so a period starting between the\n"
     "     * two writes puts out one 20 ms stop pulse */\n"
     "    if (cycle_writes > 0u && esc_left == 0.0f && esc_right == 0.0f && (left != 0.0f || right != 0.0f))\n"
     "        cycle_zero_then_drive = true;\n"
     "    ++cycle_writes;\n"
     "    esc_left = left; esc_right = right; ++throttle_writes; return ESP_OK;\n"
     "}"),
    # the real runtime_metrics.c is linked: its timing feeds the mission record
    ("void runtime_metrics_count(runtime_task_id_t id, runtime_metric_event_t event) { (void)id; (void)event; }\n", ""),
    ("void runtime_metrics_cycle_begin(runtime_task_id_t id, uint64_t scheduled, uint64_t started) { (void)id; (void)scheduled; (void)started; }\n", ""),
    ("void runtime_metrics_cycle_end(runtime_task_id_t id, uint64_t finished) { (void)id; (void)finished; }\n", ""),
    # an SD card that remembers its files: record names must not repeat
    ("esp_err_t fs_sdcard_read(const char *p, void *b, size_t c, size_t *o) { (void)p; (void)b; (void)c; if (o) *o = 0; return ESP_ERR_NOT_FOUND; }\n"
     "esp_err_t fs_sdcard_write(const char *p, const void *d, size_t l) { (void)p; (void)d; (void)l; return ESP_OK; }\n",
     "static char card_name[256][32]; static size_t card_size[256]; static unsigned card_n;\n"
     "static int card_find(const char *p) {\n"
     "    for (unsigned i = 0; i < card_n; ++i) if (!strcmp(card_name[i], p)) return (int)i;\n"
     "    return -1;\n"
     "}\n"
     "esp_err_t fs_sdcard_read(const char *p, void *b, size_t c, size_t *o) {\n"
     "    (void)b; (void)c; if (o) *o = 0; return card_find(p) < 0 ? ESP_ERR_NOT_FOUND : ESP_OK;\n"
     "}\n"
     "esp_err_t fs_sdcard_write(const char *p, const void *d, size_t l) {\n"
     "    (void)d; assert(strlen(p) < 32);\n"
     "    int i = card_find(p);\n"
     "    if (i < 0) { assert(card_n < 256); i = (int)card_n++; snprintf(card_name[i], 32, \"%s\", p); }\n"
     "    card_size[i] = l; return ESP_OK;\n"
     "}\n"),
    ("    unsigned idx = sd_append_calls++;\n",
     "    unsigned idx = sd_append_calls++;\n"
     "    { int ci = card_find(p); assert(ci >= 0); card_size[ci] += l; }\n"),
)

MISSION_HEADERS_WB = {k: v for k, v in MISSION_HEADERS.items() if k != "runtime_metrics.h"}

MISSION_UNIT = r"""
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "autonomy.c"

/* The pipeline end of the mission task: its handler and its status. */
static mission_command_handler_fn s_wb_handler = NULL;
void pipeline_register_mission_handler(mission_command_handler_fn h) { s_wb_handler = h; }
static boat_MissionStatus s_wb_last;
static unsigned s_wb_publishes;
void pipeline_publish_mission_status(const boat_MissionStatus *s) { s_wb_last = *s; ++s_wb_publishes; }

void wb_autonomy_init(void) { assert(autonomy_init() == ESP_OK); assert(s_wb_handler); }
void wb_autonomy_step(int64_t now) { autonomy_step(now); }
void wb_autonomy_diag(void) { autonomy_diagnostics_tick(false); }
void wb_mission_start(uint32_t request_id, uint32_t stage, float out, float radius,
                      float thr, float appr, bool right, bool dry) {
    boat_MissionCommand c = boat_MissionCommand_init_zero;
    c.start = true; c.request_id = request_id; c.stage = stage; c.out_distance_m = out;
    c.home_radius_m = radius; c.throttle = thr; c.approach_throttle = appr;
    c.turn_right = right; c.dry_run = dry;
    s_wb_handler(&c);                       /* as the RX task calls it */
}
void wb_mission_stop(void) {
    boat_MissionCommand c = boat_MissionCommand_init_zero;
    c.stop = true;
    s_wb_handler(&c);
}
int wb_state(void) { return (int)s_m.state; }
int wb_reason(void) { return (int)s_m.reason; }
const char *wb_reason_text(void) { return reason_text(s_m.reason); }
uint32_t wb_run(void) { return s_m.run_id; }
float wb_closest(void) { return s_m.closest_m; }
float wb_dist_home(void) { return s_m.have_pos ? s_m.dist_home_m : -1.0f; }
uint32_t wb_outages(void) { return s_m.outages; }
uint32_t wb_record_state(void) { return atomic_load(&s_rec_state); }
uint32_t wb_file_index(void) { return atomic_load(&s_rec_file_index); }
bool wb_save_pending(void) { return atomic_load(&s_save_pending); }
unsigned wb_publishes(void) { return s_wb_publishes; }
int wb_last_status_state(void) { return (int)s_wb_last.state; }
bool wb_active(void) { return mission_is_active(&s_m); }
bool wb_dry(void) { return s_m.settings.dry_run; }
float wb_wanted_heading(void) { return s_m.wanted_heading_deg; }
"""

CONTROL_MAIN = r"""
/* ======================= the whole boat, on the host ======================= */
#include <stdlib.h>
#include "bench_run.h"

void wb_autonomy_init(void);
void wb_autonomy_step(int64_t now);
void wb_autonomy_diag(void);
void wb_mission_start(uint32_t request_id, uint32_t stage, float out, float radius,
                      float thr, float appr, bool right, bool dry);
void wb_mission_stop(void);
int wb_state(void); int wb_reason(void); const char *wb_reason_text(void);
uint32_t wb_run(void); float wb_closest(void); float wb_dist_home(void);
uint32_t wb_outages(void); uint32_t wb_record_state(void); uint32_t wb_file_index(void);
bool wb_save_pending(void); unsigned wb_publishes(void); int wb_last_status_state(void);
bool wb_active(void); bool wb_dry(void); float wb_wanted_heading(void);

enum { M_IDLE, M_HOME, M_OUT, M_TURN, M_RETURN, M_DONE, M_ABORTED, M_REFUSED };
enum { R_OUT = 1, R_TURNED = 2, R_IN_ZONE = 3, R_PASSED = 4, R_NEAR = 5,
       R_STOP = 10, R_MANUAL = 11, R_DISARMED = 12, R_RAIL = 13,
       R_BUSY = 32, R_NOT_ARMED = 33 };
#define TERMINAL(s) ((s) == M_DONE || (s) == M_ABORTED || (s) == M_REFUSED)
static const double WB_PI = 3.14159265358979323846;

float fusion_get_field_ratio(void) { return 1.0f; }
void vTaskDelayUntil(TickType_t *prev, TickType_t period) { (void)prev; (void)period; }
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t t) { (void)t; return 1024u; }

/* ---- the boat on the water ---- */
static double pn = 0.0, pe = 0.0;             /* metres from the origin */
static float hdg = 30.0f, yaw = 0.0f, spd = 0.0f;
static float drift_n = 0.0f, drift_e = 0.0f;  /* wind / current, m/s */
static float c_bal = 0.20f;                    /* the split that goes straight */
static uint32_t rng = 12345u;
static float noise(float a) {
    rng = rng * 1664525u + 1013904223u;
    return a * ((float)((rng >> 8) & 0xFFFFu) / 32768.0f - 1.0f);
}
static void carry_step(float dt) {
    /* a DRY RUN: the jets stay off and a person walks the boat along the
     * heading the mission wants, turning with it (at most 20 deg/s) */
    const int st = wb_state();
    const float want = wb_wanted_heading();
    float err = fmodf(want - hdg + 540.0f, 360.0f) - 180.0f;
    const float turn = fmaxf(-20.0f * dt, fminf(20.0f * dt, err));
    hdg = fmodf(hdg + turn + 360.0f, 360.0f);
    yaw = -turn / dt;                                        /* + = left */
    spd = (st == M_OUT || st == M_RETURN) ? 0.5f : 0.0f;
    const double h = hdg * WB_PI / 180.0;
    pn += spd * cos(h) * dt;
    pe += spd * sin(h) * dt;
}
static void plant_step(float dt) {
    if (wb_active() && wb_dry()) { carry_step(dt); return; }
    const float l = esc_left > 0.0f ? esc_left : 0.0f, r = esc_right > 0.0f ? esc_right : 0.0f;
    const float t = 0.5f * (l + r);
    const float c = (l + r) > 0.02f ? (r - l) / (l + r) : 0.0f;
    const float auth = fminf(1.0f, t / 0.40f);
    const float yaw_target = 57.0f * (c - c_bal) * auth;      /* + = left */
    yaw += (yaw_target - yaw) * dt / 1.2f;
    hdg = fmodf(hdg - yaw * dt + 360.0f, 360.0f);
    spd += (0.95f * t - spd) * dt / 2.5f;
    const double h = hdg * WB_PI / 180.0;
    pn += (spd * cos(h) + drift_n) * dt;
    pe += (spd * sin(h) + drift_e) * dt;
}

/* ---- its sensors ---- */
#define LAT0 21.0417483
#define LON0 105.8862132
static bool gps_alive = true, heading_alive = true;
static uint8_t gps_sats = 12;
static void fusion_tick(void) {
    if (!heading_alive) return;                  /* fusion stops publishing */
    fusion_now.heading = hdg; fusion_now.heading_valid = true; fusion_now.yaw_rate = yaw;
    fusion_now.sequence++; fusion_now.captured_us = (uint64_t)now_us;
}
static void gps_tick(void) {
    if (!gps_alive) return;                      /* the fix just ages */
    gps_now.valid = true; gps_now.fix_quality = 3; gps_now.satellites = gps_sats; gps_now.hdop = 1.4f;
    gps_now.latitude = LAT0 + (pn + noise(0.05f)) / 110540.0;
    gps_now.longitude = LON0 + (pe + noise(0.05f)) / (111320.0 * cos(LAT0 * WB_PI / 180.0));
    const double vn = spd * cos(hdg * WB_PI / 180.0) + drift_n, ve = spd * sin(hdg * WB_PI / 180.0) + drift_e;
    gps_now.speed_mps = (float)sqrt(vn * vn + ve * ve) + noise(0.01f);
    if (gps_now.speed_mps < 0.0f) gps_now.speed_mps = 0.0f;
    gps_now.course_deg = fmodf((float)(atan2(ve, vn) * 180.0 / WB_PI) + noise(2.0f) + 720.0f, 360.0f);
    gps_now.speed_acc_mps = 0.08f;
    gps_now.last_update_us = now_us;
}

/* ---- the laptop: its 15 Hz stream, exactly what the tool sends ---- */
static float st_l = 0.0f, st_r = 0.0f, st_rudder = 0.0f;
static bool link_up = true;       /* the radio */
static bool mission_mode = false; /* the tool streams motor zeros only during a mission */
static bool calib_mode = false;   /* ...and only CalibrateCommand keepalives during a calibration */
static bool autonomy_stalled = false;   /* the mission task starved (never happens, tested anyway) */
static int64_t calib_idle_since_us = 0;
static int64_t next_stream_us = 0;
static unsigned stream_frames = 0;
static int64_t last_nonzero_sent_us = -1000000000;
static void laptop_tick(void) {
    if (!link_up || now_us < next_stream_us) return;
    next_stream_us = now_us + 66667;
    if (calib_mode) { calibrate_handler(true, false); ++stream_frames; return; }
    if (st_l != 0.0f || st_r != 0.0f) last_nonzero_sent_us = now_us;
    motor_handler(&(boat_MotorCommand){.left = st_l, .right = st_r});
    if (!mission_mode) {
        steer_handler(&(boat_SteerCommand){.left = st_rudder, .right = st_rudder});
        winch_handler(&(boat_WinchCommand){.speed = 0.0f});
    }
    ++stream_frames;
}

/* ---- the invariants, checked every 10 ms ---- */
static unsigned violations = 0;
static bool bench_live(void) {
    boat_BenchStatus bs; motor_control_get_bench_status(&bs);
    return bs.state == BENCH_BASELINE || bs.state == BENCH_RUN || bs.state == BENCH_COAST;
}
static bool bench_live(void);
static void check(const char *what, bool ok) {
    if (!ok) {
        printf("VIOLATION t=%.2f: %s [jets %.3f/%.3f esc_state %d sticks %.2f/%.2f mission %d%s bench %d]\n",
               (double)now_us / 1e6, what, (double)esc_left, (double)esc_right, (int)esc_state,
               (double)st_l, (double)st_r, wb_state(), wb_dry() ? " dry" : "", (int)bench_live());
        ++violations;
    }
}
static bool p_changed_by_op;
static int64_t mission_idle_since_us = 0, bench_idle_since_us = 0;
#define GRACE_US 500000   /* a released stick or an ended run: zero within 0.5 s */
static bool c_tracking = false; static float c_at_start = 0.0f;
static unsigned last_pub = 0; static int64_t last_pub_us = 0;
static void invariants(void) {
    if (wb_active()) mission_idle_since_us = now_us + 1;
    if (bench_live()) bench_idle_since_us = now_us + 1;
    if (calib_mode) calib_idle_since_us = now_us + 1;
    const bool quiet = now_us - mission_idle_since_us > GRACE_US &&
                       now_us - bench_idle_since_us > GRACE_US &&
                       now_us - calib_idle_since_us > 5 * GRACE_US &&
                       now_us - last_nonzero_sent_us > GRACE_US;
    /* the learned trim never moves while a mission runs */
    if (wb_active()) {
        if (!c_tracking) { c_tracking = true; c_at_start = motor_control_trimlearn_c(); }
        check("the learned trim never changes during a mission", motor_control_trimlearn_c() == c_at_start);
    } else {
        c_tracking = false;
    }
    /* MissionStatus keeps coming: 1 Hz idle, 5 Hz running */
    if (wb_publishes() != last_pub) { last_pub = wb_publishes(); last_pub_us = now_us; }
    check("MissionStatus at least every 2 s", autonomy_stalled || now_us - last_pub_us < 2000000);
    check("ESC outputs finite and in [0,1]",
          isfinite(esc_left) && isfinite(esc_right) && esc_left >= 0.0f && esc_left <= 1.0f &&
          esc_right >= 0.0f && esc_right <= 1.0f);
    check("disarmed jets are zero",
          esc_state == ESC_STATE_ARMED || (esc_left == 0.0f && esc_right == 0.0f));
    check("jets at zero 0.5 s after the last stick, run and mission",
          !quiet || (esc_left == 0.0f && esc_right == 0.0f));
    check("a dry run keeps the jets at zero",
          !(wb_active() && wb_dry()) || (esc_left == 0.0f && esc_right == 0.0f) ||
          now_us - last_nonzero_sent_us <= GRACE_US);   /* a stick took the jets back */
    check("never a mission and a bench run at once", !(wb_active() && bench_live()));
    boat_CalibrateStatus cs; motor_control_get_calibrate_status(&cs);
    const bool cal_running = cs.state == 1u || cs.state == 2u || cs.state == 3u;   /* NOISE/RAMP/SETTLE */
    static int64_t both_since = 0;
    if (wb_active() && cal_running) { if (!both_since) both_since = now_us; } else both_since = 0;
    check("never a mission and a calibration at once", !both_since || now_us - both_since < 100000);
}

static unsigned tick_n = 0;
static void step_10ms(void) {
    now_us += 10000;
    plant_step(0.01f);
    if (tick_n % 2u == 0u) fusion_tick();
    if (tick_n % 10u == 0u) gps_tick();
    laptop_tick();
    cycle_writes = 0; cycle_zero_then_drive = false;
    const bool driving = wb_active() && !wb_dry() && wb_state() >= M_OUT && wb_state() <= M_RETURN;
    run_one_control_cycle();
    check("while a mission drives, the ESC never gets a zero before its value in one cycle",
          !(driving && cycle_zero_then_drive));
    if (tick_n % 5u == 0u && !autonomy_stalled) wb_autonomy_step(now_us);
    if (tick_n % 10u == 5u) { motor_control_bench_flush(); wb_autonomy_diag(); }
    invariants();
    ++tick_n;
}
static void run_s(float s) { const int n = (int)(s * 100.0f + 0.5f); for (int i = 0; i < n; ++i) step_10ms(); }

/* ---- the operator, as the tool does it ---- */
static uint32_t assist_id = 0, request_id = 1000;
static void op_p(bool on) { assist_handler(on, false, ++assist_id); p_changed_by_op = true; run_s(0.1f); }
static void op_arm(void) { esc_state = ESC_STATE_ARMED; servo_power = true; run_s(0.1f); }
static void op_disarm(void) { (void)esc_driver_disarm(); run_s(0.1f); }
static void op_sticks(float l, float r) { st_l = l; st_r = r; }
static void op_power(bool on) { power_handler(on); run_s(0.1f); }

static int bench_until_end(uint32_t kind, float base, float delta, float timeout_s) {
    boat_BenchStatus bs;
    const uint32_t files_before = card_n;
    op_sticks(0.0f, 0.0f);
    bench_handler(kind, base, delta, 0.0f, false);
    float max_jet = 0.0f;
    bool started = false;                          /* the status shows the OLD run until it does */
    for (int i = 0; i < 50 && !started; ++i) { step_10ms(); started = bench_live(); }
    if (!started) {
        printf("  bench kind %u base %.2f: NOT STARTED (refused)\n", (unsigned)kind, (double)base);
        return -1;
    }
    for (int i = 0; i < (int)(timeout_s * 100.0f); ++i) {
        step_10ms();
        max_jet = fmaxf(max_jet, fmaxf(esc_left, esc_right));
        motor_control_get_bench_status(&bs);
        if (bs.state == BENCH_SAVED || bs.state == BENCH_FAILED) break;
    }
    motor_control_get_bench_status(&bs);
    printf("  bench kind %u base %.2f: state %u file %u max jet %.2f (files %u -> %u)\n",
           (unsigned)kind, (double)base, (unsigned)bs.state, (unsigned)bs.file_index,
           (double)max_jet, (unsigned)files_before, (unsigned)card_n);
    if (bs.state == BENCH_SAVED) {
        assert(card_n == files_before + 1u);          /* one new file, never overwritten */
        assert(max_jet > 0.9f * base);                 /* it really drove */
    }
    return (int)bs.state;
}

/* A mission as the tool flies it: sticks to zero, START, presence stream. */
static int mission(uint32_t stage, float out_m, float radius, float thr, bool right, bool dry,
                   float timeout_s, int *reason_out) {
    const uint32_t run_before = wb_run();
    op_sticks(0.0f, 0.0f);
    mission_mode = true;
    const float c_before = motor_control_trimlearn_c();
    const bool p_before = motor_control_p_assist_on();
    wb_mission_start(++request_id, stage, out_m, radius, thr, 0.20f, right, dry);
    float max_jet = 0.0f;
    int state = M_IDLE;
    for (int i = 0; i < (int)(timeout_s * 100.0f); ++i) {
        step_10ms();
        max_jet = fmaxf(max_jet, fmaxf(esc_left, esc_right));
        state = wb_state();
        if (wb_run() != run_before && TERMINAL(state)) break;
    }
    mission_mode = false;
    run_s(0.05f);
    if (reason_out) *reason_out = wb_reason();
    printf("  mission stage %u out %.0f thr %.2f%s: %s reason %d (%s) closest %.2f dist %.2f "
           "outages %u max jet %.2f\n", (unsigned)stage, (double)out_m, (double)thr,
           dry ? " DRY" : "", state == M_DONE ? "DONE" : state == M_ABORTED ? "ABORTED" :
           state == M_REFUSED ? "REFUSED" : "STILL RUNNING", wb_reason(), wb_reason_text(),
           (double)wb_closest(), (double)wb_dist_home(), (unsigned)wb_outages(), (double)max_jet);
    if (state != M_REFUSED) {
        /* the mission never touches the operator's P switch or the learned trim */
        assert(motor_control_p_assist_on() == p_before);
        assert(motor_control_trimlearn_c() == c_before);
    }
    return state;
}

static void expect_record_saved(uint32_t index) {
    run_s(2.0f);
    printf("  record: state %u file MSN_%03u.CSV\n", (unsigned)wb_record_state(), (unsigned)wb_file_index());
    assert(wb_record_state() == 3u /* SAVED */);
    assert(wb_file_index() == index);
    char name[16]; snprintf(name, sizeof(name), "MSN_%03u.CSV", (unsigned)index);
    const int i = card_find(name);
    assert(i >= 0 && card_size[i] > 1000u);
}

static float heading_hold_drift(float thr, float seconds) {
    /* the lake test's straight leg: P ON, both sticks at thr */
    op_sticks(thr, thr);
    run_s(2.0f);                               /* capture, settle */
    const float h0 = hdg;
    run_s(seconds);
    float d = fmodf(hdg - h0 + 540.0f, 360.0f) - 180.0f;
    op_sticks(0.0f, 0.0f);
    return d;
}

static void lake_like(void) {
    /* tools/espnow_drive.py lake test: P ON, precheck zeros, straight, yaw pulses, stop */
    op_p(true);
    op_sticks(0.0f, 0.0f); run_s(2.0f);
    const float drift = heading_hold_drift(0.40f, 10.0f);
    printf("  lake straight T40 10 s with P: heading drift %+.2f deg\n", (double)drift);
    assert(fabsf(drift) < 5.0f);
    /* yaw pulses: each ends with the boat turning the way Kiet's rule says
     * (right jet stronger = turn LEFT = + yaw here), whatever it did before */
    op_sticks(0.20f, 0.60f); run_s(2.0f);                 /* right jet stronger */
    const float jl1 = esc_left, jr1 = esc_right, y1 = yaw;
    op_sticks(0.60f, 0.20f); run_s(2.0f);                 /* left jet stronger */
    const float jl2 = esc_left, jr2 = esc_right, y2 = yaw;
    op_sticks(0.0f, 0.0f); run_s(5.0f);
    printf("  lake pulses: 0.20/0.60 -> jets %.3f/%.3f yaw %+.1f (left); "
           "0.60/0.20 -> jets %.3f/%.3f yaw %+.1f (right)\n",
           (double)jl1, (double)jr1, (double)y1, (double)jl2, (double)jr2, (double)y2);
    assert(jr1 > jl1 && y1 > 5.0f);
    assert(jl2 > jr2 && y2 < -5.0f);
}

static void manual_drive(void) {
    op_sticks(0.30f, 0.30f); run_s(3.0f);
    printf("  manual T30: jets %.2f / %.2f\n", (double)esc_left, (double)esc_right);
    assert(esc_left > 0.10f && esc_right > 0.10f);
    op_sticks(0.0f, 0.0f); run_s(1.0f);
    assert(esc_left == 0.0f && esc_right == 0.0f);
}

static void boot(void) {
    runtime_metrics_init();
    assert(motor_control_init() == ESP_OK);
    wb_autonomy_init();
    assert(autonomy_fn != NULL);                       /* the mission task exists */
    assert(motor_control_p_assist_on());               /* Motor P is ON at boot */
    run_s(2.0f);
    assert(wb_publishes() >= 1u && wb_last_status_state() == M_IDLE);
}

static float start_hdg = 30.0f;     /* where Kiet points the boat before START */
static bool random_heading = false;
static uint32_t rnd(void);
static void home_boat(void) {
    /* carry the boat back to its start, pointing wherever the next run goes */
    op_sticks(0.0f, 0.0f); run_s(3.0f);
    if (random_heading) start_hdg = (float)(rnd() % 36000u) / 100.0f;
    pn = 0.0; pe = 0.0; spd = 0.0f; yaw = 0.0f; hdg = start_hdg;
    run_s(1.0f);
}

static unsigned msn_on_card(void) {
    unsigned n = 0;
    for (unsigned i = 0; i < card_n; ++i) n += !strncmp(card_name[i], "MSN_", 4);
    return n;
}
static bool turn_right_next = true;
static void full_mission_ok(float thr) {
    int reason = 0;
    home_boat();
    for (int i = 0; i < 2000 && wb_save_pending(); ++i) step_10ms();   /* the last run's file */
    const unsigned next_file = msn_on_card() + 1u;        /* every run that drove has one */
    const int s = mission(3, 10.0f, 2.5f, thr, turn_right_next, false, 240.0f, &reason);
    printf("    (pointed at %.1f deg, turn %s)\n", (double)start_hdg, turn_right_next ? "right" : "left");
    assert(s == M_DONE);
    assert(reason == R_IN_ZONE || reason == R_PASSED || reason == R_NEAR);
    assert(wb_closest() < 3.0f);
    run_s(0.5f);
    assert(esc_left == 0.0f && esc_right == 0.0f);    /* motors off after DONE */
    expect_record_saved(next_file);
}

/* ------------------------------ scenarios ------------------------------ */
static void scenario_mission_then_tests(void) {
    printf("mission, then BASE, lake, mismatch, BASE 10 s, manual, mission again\n");
    op_arm();
    full_mission_ok(0.40f);
    assert(bench_until_end(BENCH_KIND_BASE, 0.30f, 0.0f, 15.0f) == BENCH_SAVED);
    lake_like();
    assert(bench_until_end(BENCH_KIND_LEFT, 0.30f, 0.10f, 15.0f) == BENCH_SAVED);
    assert(bench_until_end(BENCH_KIND_RIGHT, 0.30f, 0.10f, 15.0f) == BENCH_SAVED);
    assert(bench_until_end(BENCH_KIND_BASE_LONG, 0.30f, 0.0f, 20.0f) == BENCH_SAVED);
    manual_drive();
    full_mission_ok(0.40f);
}

static void scenario_tests_then_mission(void) {
    printf("BASE, lake, mismatch, manual, then the mission (P OFF and ON)\n");
    op_arm();
    assert(bench_until_end(BENCH_KIND_BASE, 0.30f, 0.0f, 15.0f) == BENCH_SAVED);
    lake_like();
    assert(bench_until_end(BENCH_KIND_LEFT, 0.30f, 0.10f, 15.0f) == BENCH_SAVED);
    assert(bench_until_end(BENCH_KIND_RIGHT, 0.30f, 0.10f, 15.0f) == BENCH_SAVED);
    manual_drive();
    op_p(false);                                    /* the mission flies the same with P OFF */
    full_mission_ok(0.40f);
    assert(!motor_control_p_assist_on());
    op_p(true);
    full_mission_ok(0.30f);
    manual_drive();                                 /* and manual driving still works after */
}

static void scenario_busy_stop_rearm(void) {
    printf("START during a bench run, STOP, DISARM, re-ARM, fly\n");
    int reason = 0;
    op_arm();
    op_sticks(0.0f, 0.0f);
    bench_handler(BENCH_KIND_BASE_LONG, 0.30f, 0.0f, 0.0f, false);
    run_s(2.0f);
    assert(mission(3, 10.0f, 2.5f, 0.40f, true, false, 5.0f, &reason) == M_REFUSED && reason == R_BUSY);
    for (int i = 0; i < 2000 && bench_live(); ++i) step_10ms();
    run_s(0.5f);                                  /* its file is written right after */
    assert(!bench_live());
    home_boat();
    /* fly out, STOP after 8 s: jets stop at once; the tool also DISARMs */
    const uint32_t run_before = wb_run();
    mission_mode = true;
    wb_mission_start(++request_id, 3, 10.0f, 2.5f, 0.40f, 0.20f, true, false);
    run_s(8.0f);
    assert(wb_state() == M_OUT && esc_left + esc_right > 0.2f);
    wb_mission_stop();
    step_10ms();
    printf("  STOP: jets %.2f / %.2f in the next cycle, state %d reason %d\n",
           (double)esc_left, (double)esc_right, wb_state(), wb_reason());
    assert(esc_left == 0.0f && esc_right == 0.0f);   /* the same control cycle */
    run_s(0.2f);
    assert(wb_run() == run_before + 1u && wb_state() == M_ABORTED && wb_reason() == R_STOP);
    mission_mode = false;
    op_disarm();
    run_s(3.0f);
    assert(wb_record_state() == 3u);                 /* even a stopped run is recorded */
    assert(mission(3, 10.0f, 2.5f, 0.40f, true, false, 5.0f, &reason) == M_REFUSED && reason == R_NOT_ARMED);
    op_arm();
    full_mission_ok(0.40f);
}

static void scenario_stick_rail_disarm_end_it(void) {
    printf("a stick, PWR-OFF, DISARM each end a mission at once\n");
    int reason;
    op_arm();
    struct { const char *name; int expect; } ends[3] = {{"stick", R_MANUAL}, {"rail", R_RAIL}, {"disarm", R_DISARMED}};
    for (int k = 0; k < 3; ++k) {
        home_boat();
        mission_mode = true;
        wb_mission_start(++request_id, 3, 10.0f, 2.5f, 0.40f, 0.20f, true, false);
        run_s(6.0f);
        assert(wb_state() == M_OUT);
        if (k == 0) { op_sticks(0.25f, 0.25f); step_10ms(); step_10ms(); }
        if (k == 1) { power_handler(false); step_10ms(); step_10ms(); }
        if (k == 2) { (void)esc_driver_disarm(); step_10ms(); step_10ms(); }
        run_s(0.2f);
        reason = wb_reason();
        printf("  %s: state %d reason %d (%s), jets %.2f / %.2f\n", ends[k].name, wb_state(), reason,
               wb_reason_text(), (double)esc_left, (double)esc_right);
        assert(wb_state() == M_ABORTED && reason == ends[k].expect);
        if (k == 0) assert(esc_left > 0.10f && esc_right > 0.10f);   /* the stick's own command */
        else assert(esc_left == 0.0f && esc_right == 0.0f);
        mission_mode = false;
        op_sticks(0.0f, 0.0f);
        run_s(3.0f);
        if (k == 1) op_power(true);
        if (k == 2) op_arm();
    }
    full_mission_ok(0.40f);
}

static void scenario_radio_loss_and_sensors(void) {
    printf("radio loss mid-run (the boat carries on), GPS and heading loss (it stops)\n");
    int reason = 0;
    op_arm();
    home_boat();
    mission_mode = true;
    const uint32_t run_before = wb_run();
    wb_mission_start(++request_id, 3, 10.0f, 2.5f, 0.40f, 0.20f, true, false);
    run_s(10.0f);
    link_up = false;                                  /* the S3 unplugged for 12 s */
    run_s(12.0f);
    link_up = true;
    for (int i = 0; i < 24000 && !(wb_run() != run_before && TERMINAL(wb_state())); ++i) step_10ms();
    mission_mode = false;
    printf("  after a 12 s outage: state %d reason %d (%s), outages %u, closest %.2f\n", wb_state(),
           wb_reason(), wb_reason_text(), (unsigned)wb_outages(), (double)wb_closest());
    assert(wb_state() == M_DONE && wb_outages() >= 1u && wb_closest() < 3.0f);
    run_s(3.0f);

    home_boat();
    mission_mode = true;
    wb_mission_start(++request_id, 3, 10.0f, 2.5f, 0.40f, 0.20f, true, false);
    run_s(8.0f);
    gps_alive = false;
    run_s(4.0f);
    printf("  GPS lost: state %d reason %d (%s), jets %.2f / %.2f\n", wb_state(), wb_reason(),
           wb_reason_text(), (double)esc_left, (double)esc_right);
    assert(wb_state() == M_ABORTED && esc_left == 0.0f && esc_right == 0.0f);
    gps_alive = true; mission_mode = false; run_s(3.0f);

    home_boat();
    mission_mode = true;
    wb_mission_start(++request_id, 3, 10.0f, 2.5f, 0.40f, 0.20f, true, false);
    run_s(8.0f);
    heading_alive = false;
    run_s(2.0f);
    printf("  heading lost: state %d reason %d (%s), jets %.2f / %.2f\n", wb_state(), wb_reason(),
           wb_reason_text(), (double)esc_left, (double)esc_right);
    assert(wb_state() == M_ABORTED && esc_left == 0.0f && esc_right == 0.0f);
    heading_alive = true; mission_mode = false; run_s(3.0f);
    (void)reason;
    full_mission_ok(0.40f);
}

/* Random operator: every action the tool offers, in any order, any time. */
static uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng >> 8; }
static float rndf(void) { return (float)(rnd() & 0xFFFFu) / 65535.0f; }
static void start_run(uint32_t stage, float thr, bool right, bool dry) {
    home_boat(); op_sticks(0, 0); mission_mode = true;
    wb_mission_start(++request_id, stage, 10.0f, 2.5f, thr, 0.20f, right, dry);
}
static void end_run_cleanly(void) {
    for (int i = 0; i < 24000 && wb_active(); ++i) step_10ms();
    if (wb_active()) { wb_mission_stop(); run_s(0.5f); }
    mission_mode = false;
    for (int i = 0; i < 1000 && wb_save_pending(); ++i) step_10ms();
    run_s(1.0f);
    check("a finished run's record is never stuck saving", wb_record_state() != 2u);
}
static void scenario_random(unsigned seed, int actions) {
    printf("random operator, seed %u, %d actions\n", seed, actions);
    rng = seed * 2654435761u + 1u;
    /* every seed a different boat: the jets' mismatch across the 09-20/21
     * range, and a light wind */
    c_bal = 0.05f + 0.35f * rndf();
    drift_n = 0.10f * (rndf() - 0.5f);
    drift_e = 0.10f * (rndf() - 0.5f);
    printf("  boat: straight at c = %.2f, drift %.2f / %.2f m/s\n", (double)c_bal, (double)drift_n, (double)drift_e);
    random_heading = true;
    op_arm();
    unsigned done_ok = 0, started = 0, counts[22] = {0};
    for (int a = 0; a < actions; ++a) {
        const unsigned pick = rnd() % 22u;
        const float u = rndf();
        ++counts[pick];
        switch (pick) {
        case 0: op_sticks(0.1f + 0.4f * u, 0.1f + 0.4f * u); run_s(0.5f + 3.0f * u); op_sticks(0, 0); run_s(0.5f); break;
        case 1: op_sticks(0.6f * u, 0.6f * (1.0f - u)); run_s(1.0f + 2.0f * u); op_sticks(0, 0); run_s(0.5f); break;
        case 2: op_p(u < 0.5f); break;
        case 3: (void)bench_until_end(BENCH_KIND_BASE, 0.2f + 0.3f * u, 0.0f, 15.0f); break;
        case 4: (void)bench_until_end(u < 0.5f ? BENCH_KIND_LEFT : BENCH_KIND_RIGHT, 0.3f, 0.1f, 15.0f); break;
        case 5: (void)bench_until_end(BENCH_KIND_BASE_LONG, 0.3f, 0.0f, 20.0f); break;
        case 6: { int r; home_boat(); started++;
                  if (mission(1 + (unsigned)(u * 2.99f), 10.0f, 2.5f, 0.25f + 0.3f * u, u < 0.7f,
                              u > 0.85f, 240.0f, &r) == M_DONE) ++done_ok;
                  end_run_cleanly(); break; }
        case 7: /* START then STOP somewhere in the run; the tool also DISARMs */
                start_run(3, 0.4f, true, false); started++;
                run_s(1.0f + 30.0f * u); wb_mission_stop(); step_10ms();
                if (wb_active()) check("STOP: jets zero in the next cycle", esc_left == 0.0f && esc_right == 0.0f);
                op_disarm(); end_run_cleanly(); op_arm(); break;
        case 8: /* the throttle stick during a run */
                start_run(3, 0.4f, true, false); started++;
                run_s(2.0f + 20.0f * u); op_sticks(0.2f, 0.2f); run_s(0.5f); op_sticks(0, 0);
                end_run_cleanly(); break;
        case 9: op_disarm(); run_s(0.5f + u); op_arm(); break;
        case 10: op_power(false); run_s(0.5f + u); op_power(true); break;
        case 11: link_up = false; run_s(0.2f + 3.0f * u); link_up = true; break;
        case 12: lake_like(); break;
        case 13: { /* START while a bench run is going or saving */
                   op_sticks(0, 0); bench_handler(BENCH_KIND_BASE, 0.3f, 0.0f, 0.0f, false);
                   run_s(0.5f + 3.0f * u);
                   int r; (void)mission(3, 10.0f, 2.5f, 0.4f, true, false, 3.0f, &r);
                   end_run_cleanly();
                   for (int i = 0; i < 2000 && bench_live(); ++i) step_10ms();
                   run_s(1.0f); break; }
        case 14: { /* a second START (new request) while one runs: ignored */
                   start_run(3, 0.4f, true, false); started++;
                   run_s(3.0f + 10.0f * u);
                   const uint32_t run = wb_run(); const bool was_active = wb_active();
                   wb_mission_start(++request_id, 1, 20.0f, 3.0f, 0.3f, 0.2f, false, false);
                   run_s(0.3f);
                   if (was_active) check("a START during a run changes nothing", wb_run() == run);
                   end_run_cleanly(); break; }
        case 15: /* P flipped mid-run: the switch changes, the mission flies on */
                 start_run(3, 0.4f, true, false); started++;
                 run_s(3.0f + 10.0f * u); op_p(u < 0.5f);
                 end_run_cleanly(); break;
        case 16: /* the rudder or the winch touched mid-run: manual input ends it */
                 start_run(3, 0.4f, true, false); started++;
                 run_s(3.0f + 10.0f * u);
                 if (u < 0.5f) steer_handler(&(boat_SteerCommand){.left = 0.3f, .right = 0.3f});
                 else winch_handler(&(boat_WinchCommand){.speed = 0.5f});
                 run_s(0.3f);
                 steer_handler(&(boat_SteerCommand){.left = 0.0f, .right = 0.0f});
                 winch_handler(&(boat_WinchCommand){.speed = 0.0f});
                 end_run_cleanly(); break;
        case 17: /* the mission task starves for a second: control ends the run */
                 start_run(3, 0.4f, true, false); started++;
                 run_s(3.0f + 10.0f * u);
                 if (wb_active()) {
                     autonomy_stalled = true; run_s(1.0f);
                     check("a stale setpoint stops the jets", esc_left == 0.0f && esc_right == 0.0f);
                     autonomy_stalled = false; run_s(0.3f);
                 }
                 end_run_cleanly(); break;
        case 18: { /* compass calibration running: START refused busy */
                   compass_cal_running = true; run_s(0.5f);
                   int r; const int st = mission(3, 10.0f, 2.5f, 0.4f, true, false, 3.0f, &r);
                   check("START refused while the compass calibrates", st == M_REFUSED && r == R_BUSY);
                   compass_cal_running = false; end_run_cleanly(); break; }
        case 19: { /* ESC-trim calibration: keepalives only, START refused, then stop */
                   op_sticks(0, 0); run_s(0.3f);
                   calib_mode = true; run_s(1.0f + 4.0f * u);
                   int r; const int st = mission(3, 10.0f, 2.5f, 0.4f, true, false, 2.0f, &r);
                   boat_CalibrateStatus cs; motor_control_get_calibrate_status(&cs);
                   if (cs.state >= 1u && cs.state <= 3u)
                       check("START refused while calibrating", st == M_REFUSED && r == R_BUSY);
                   calib_mode = false; calibrate_handler(false, false);
                   end_run_cleanly(); run_s(2.0f); break; }
        case 20: { /* weak GPS: START refused, and a run that loses it stops */
                   gps_sats = 5; run_s(0.5f);
                   int r; const int st = mission(3, 10.0f, 2.5f, 0.4f, true, false, 3.0f, &r);
                   check("START refused on few satellites", st == M_REFUSED && r == 36);
                   end_run_cleanly();
                   gps_sats = 12; run_s(0.5f);
                   start_run(3, 0.4f, true, false); started++;
                   run_s(3.0f + 5.0f * u);
                   gps_sats = 4; run_s(3.0f);                  /* below the 6 a run needs */
                   check("a run that loses its GPS stops", !wb_active());
                   check("...with the jets at zero", esc_left == 0.0f && esc_right == 0.0f);
                   gps_sats = 12; end_run_cleanly(); break; }
        case 21: /* the radio drops for a long time mid-run: the boat carries on */
                 start_run(3, 0.4f, true, false); started++;
                 run_s(2.0f + 5.0f * u); link_up = false; run_s(5.0f + 15.0f * u); link_up = true;
                 end_run_cleanly(); break;
        }
        assert(violations == 0);
    }
    printf("  %u missions started, %u DONE; %u stream frames, %u files on the card; "
           "ESC writes refused while disarmed %u (non-zero %u)\n  actions:",
           started, done_ok, stream_frames, card_n, refused_disarmed_writes, refused_nonzero_writes);
    for (int k = 0; k < 22; ++k) printf(" %u", counts[k]);
    printf("\n");
    /* whatever happened before, a clean mission still flies */
    op_sticks(0, 0); op_power(true); op_arm(); link_up = true; calib_mode = false;
    compass_cal_running = false; autonomy_stalled = false; gps_sats = 12;
    turn_right_next = (rnd() & 1u) != 0u;
    for (int i = 0; i < 2000 && (bench_live() || wb_save_pending()); ++i) step_10ms();
    full_mission_ok(0.40f);
}

static void scenario_headings(void) {
    printf("full missions from every direction, both turns, the 0/360 wrap included\n");
    static const float heads[] = {0.0f, 0.1f, 1.0f, 30.0f, 45.0f, 89.9f, 90.0f, 135.0f, 179.0f, 180.0f,
                                  181.0f, 225.0f, 270.0f, 315.0f, 358.0f, 359.0f, 359.9f};
    op_arm();
    for (unsigned i = 0; i < sizeof(heads) / sizeof(heads[0]); ++i) {
        for (int t = 0; t < 2; ++t) {
            start_hdg = heads[i];
            turn_right_next = (t == 0);
            full_mission_ok(0.40f);
        }
    }
    random_heading = true;                  /* and 40 more, anywhere */
    rng = 777u;
    for (int k = 0; k < 40; ++k) { turn_right_next = (k & 1) == 0; full_mission_ok(0.25f + 0.3f * (float)(k % 5) / 4.0f); }
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);              /* every line survives an assert */
    assert(argc >= 2);
    boot();
    const char *which = argv[1];
    if (!strcmp(which, "mission_then_tests")) scenario_mission_then_tests();
    else if (!strcmp(which, "tests_then_mission")) scenario_tests_then_mission();
    else if (!strcmp(which, "busy_stop_rearm")) scenario_busy_stop_rearm();
    else if (!strcmp(which, "stick_rail_disarm")) scenario_stick_rail_disarm_end_it();
    else if (!strcmp(which, "radio_and_sensors")) scenario_radio_loss_and_sensors();
    else if (!strcmp(which, "random")) scenario_random((unsigned)atoi(argv[2]), atoi(argv[3]));
    else if (!strcmp(which, "headings")) scenario_headings();
    else { printf("unknown scenario %s\n", which); return 2; }
    assert(violations == 0);
    printf("simulated %.0f s, %u violations\nOK\n", (double)now_us / 1e6, violations);
    return 0;
}
"""


def _sanitizers():
    return ["-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-fno-omit-frame-pointer", "-g"]


_BINARY = None


def whole_boat_binary():
    """Build once per session (two units, one link); returns the path."""
    global _BINARY
    if _BINARY is not None:
        return _BINARY
    if not (NANOPB / "pb.h").exists():
        pytest.skip("nanopb headers absent (managed_components not fetched)")
    if not SDKCONFIG_H.exists():
        pytest.skip("build/config/sdkconfig.h absent: run idf.py reconfigure first")
    tmp = Path(tempfile.mkdtemp(prefix="whole_boat_"))
    cc = os.environ.get("CC", "cc")
    common = ["-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
              "-include", str(SDKCONFIG_H), *_sanitizers()]

    # CONTROL unit
    c_dir = tmp / "control"
    headers = dict(STUB_HEADERS)
    headers["sensor_fusion.h"] = FUSION_H + "float fusion_get_field_ratio(void);\n"
    headers["drivers/gps_driver.h"] = (ROOT / "main" / "drivers" / "gps_driver.h").read_text()
    for relative, content in headers.items():
        p = c_dir / relative
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content)
    stubs = heading_hold_stubs()
    for old, new in CONTROL_SWAPS:
        assert stubs.count(old) == 1, "shared harness changed: %r" % old[:70]
        stubs = stubs.replace(old, new)
    (c_dir / "control_main.c").write_text(stubs + CONTROL_MAIN)
    shutil.copy(ROOT / "main" / "motor_control.c", c_dir / "motor_control.c")
    control_sources = [c_dir / "motor_control.c"] + [ROOT / "main" / s for s in (
        "control_arbiter.c", "arm_sequence.c", "esc_trim.c", "esc_trim_cal.c", "bench_run.c",
        "trim_learn.c", "yaw_heading_control.c", "auto_drive.c", "stability_control.c",
        "runtime_metrics.c", "runtime_schedule.c")] + [c_dir / "control_main.c"]
    objects = []
    for src in control_sources:
        obj = tmp / ("c_" + Path(src).stem + ".o")
        subprocess.run([cc, *common, "-I", str(c_dir), "-I", str(ROOT / "main"),
                        "-c", str(src), "-o", str(obj)], check=True)
        objects.append(obj)

    # MISSION unit
    m_dir = tmp / "mission"
    for relative, content in MISSION_HEADERS_WB.items():
        p = m_dir / relative
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content)
    shutil.copy(ROOT / "main" / "autonomy.c", m_dir / "autonomy.c")
    (m_dir / "mission_unit.c").write_text(MISSION_UNIT)
    mission_sources = [m_dir / "mission_unit.c"] + [ROOT / "main" / s for s in (
        "mission.c", "nav_geo.c", "course_error.c", "path_follow.c", "planner.c")]
    for src in mission_sources:
        obj = tmp / ("m_" + Path(src).stem + ".o")
        subprocess.run([cc, *common, "-I", str(m_dir), "-I", str(ROOT / "main"),
                        "-I", str(ROOT / "main" / "drivers"), "-I", str(NANOPB),
                        "-c", str(src), "-o", str(obj)], check=True)
        objects.append(obj)

    binary = tmp / "whole_boat"
    subprocess.run([cc, *_sanitizers(), *[str(o) for o in objects], "-lm", "-o", str(binary)],
                   check=True)
    _BINARY = binary
    return binary


def run_scenario(*args, timeout=900):
    res = subprocess.run([str(whole_boat_binary()), *[str(a) for a in args]],
                         capture_output=True, text=True, timeout=timeout)
    out = res.stdout + res.stderr
    assert res.returncode == 0 and "OK" in res.stdout, out[-6000:]
    assert "VIOLATION" not in out, out[-6000:]
    return res.stdout


@pytest.mark.parametrize("scenario", [
    "headings",
    "mission_then_tests",
    "tests_then_mission",
    "busy_stop_rearm",
    "stick_rail_disarm",
    "radio_and_sensors",
])
def test_whole_boat_sequences(scenario):
    print(run_scenario(scenario))


@pytest.mark.parametrize("seed", range(1, 9))
def test_whole_boat_random_operator(seed):
    print(run_scenario("random", seed, 40))
