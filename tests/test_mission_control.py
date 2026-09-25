"""The mission's AUTO owner inside the REAL motor_control.c (spec 2026-09-24,
sections 3 and 5): built with the heading hold and CONFIG_MISSION_ENABLE, fed
setpoints the way the autonomy task feeds them, on the shared harness stubs.

What must hold on the boat:
  * a mission drives through a dead radio (the manual failsafe zeroes the jets
    every cycle; the mission's write is the last one and wins), rudder centred,
    servo rail kept on;
  * STOP, a non-zero stick, disarm, an explicit PWR-OFF and a stale setpoint
    each end it at once -- the jets at zero in that same cycle (the stick's own
    command instead, for the stick) -- and the control task reports which;
  * the laptop's zero presence stream does not end it;
  * the heading hold runs on the mission's target whatever the operator's P
    switch says, the switch itself is never changed, the trim learner does not
    learn meanwhile, and afterwards the operator's P captures a FRESH heading;
  * a bench run or an ESC-trim calibration cannot start while it owns the jets;
  * a dry run owns the jets at zero and survives being disarmed.
"""
import pytest

from tests.test_heading_hold_boot import build_and_run

ROOT_DEFINES = ["-DCONFIG_MISSION_ENABLE=1"]
EXTRA = []


def _sources():
    from tests.test_heading_hold_boot import ROOT
    return [ROOT / "main" / "auto_drive.c"]


PRELUDE = r"""
#include <math.h>
static float yaw_true = 0.0f, hdg = 90.0f;
static int cyc = 0;
static auto_setpoint_t sp;

static void plant_step(void) {
    const float split = esc_right - esc_left;
    const float target = 45.0f * (split - 0.08f);
    yaw_true += (target - yaw_true) * 0.01f / 0.8f;
    hdg = fmodf(hdg - yaw_true * 0.01f + 360.0f, 360.0f);
}

static void cycle(bool refresh_setpoint) {
    now_us += 10000;
    plant_step();
    if ((cyc % 2) == 0) {
        fusion_now.yaw_rate = yaw_true;
        fusion_now.heading = hdg;
        fusion_now.heading_valid = true;
        fusion_now.sequence++;
        fusion_now.captured_us = (uint64_t)now_us;
    }
    if (refresh_setpoint && (cyc % 5) == 0) {       /* the autonomy task: every 50 ms */
        sp.stamp_us = now_us;
        motor_control_set_auto_setpoint(&sp);
    }
    run_one_control_cycle();
    ++cyc;
}

static void run(int n) { for (int i = 0; i < n; ++i) cycle(true); }

static void start_mission(uint32_t run, float heading, float thr) {
    sp = (auto_setpoint_t){.run_id = run, .active = true, .drive = true, .dry_run = false,
                           .heading_deg = heading, .throttle = thr, .stamp_us = now_us};
    motor_control_set_auto_setpoint(&sp);
}

static void arm(void) { esc_state = ESC_STATE_ARMED; servo_power = true; }

static uint8_t abort_reason_of(uint32_t run) {
    uint32_t r = 0; uint8_t why = 0;
    motor_control_get_auto_abort(&r, &why);
    return r == run ? why : 0;
}

static boat_MotorStatus status(void) { boat_MotorStatus ms; motor_control_get_status(&ms); return ms; }
"""


def run_case(body, boot_on=True):
    main_c = PRELUDE + r"""
int main(void) {
    (void)run_scheduled_and_urgent_cycle; (void)close_enough; (void)imu_ok_flag;
    (void)arm; (void)start_mission; (void)status; (void)abort_reason_of; (void)run;
    assert(motor_control_init() == ESP_OK);
    assert(motor_control_mission_built());
""" + body + r"""
    printf("OK\n");
    return 0;
}
"""
    defines = ROOT_DEFINES + (["-DCONFIG_STABILITY_YAW_PI_DEFAULT_ON=1"] if boot_on else [])
    res = build_and_run(main_c, extra_defines=defines, extra_sources=_sources())
    assert res.returncode == 0 and "OK" in res.stdout, res.stdout[-3000:] + res.stderr[-3000:]


def test_a_mission_drives_through_a_dead_radio():
    run_case(r"""
    arm();
    motor_handler(&(boat_MotorCommand){0});           /* the laptop was there once */
    start_mission(1u, 90.0f, 0.40f);
    run(10);
    const unsigned power_writes_before = power_writes;
    run(300);                                          /* 3 s, not one manual command */
    /* the rail is never switched at all -- not cut by the failsafe and put
     * back by the mission 100 times a second */
    assert(power_writes == power_writes_before);
    assert(esc_left > 0.05f && esc_right > 0.05f);     /* both jets driving */
    assert(fabsf(0.5f * (esc_left + esc_right) - 0.40f) < 0.06f);
    assert(steer_value == 0.0f);                       /* rudder centred */
    assert(servo_power);                               /* rail NOT cut by the link failsafe */
    assert(abort_reason_of(1u) == 0);
    boat_MotorStatus ms = status();
    assert(ms.ctrl_active && ms.heading_hold);
    assert(fabsf(ms.heading_target_deg - 90.0f) < 0.01f);   /* the mission's target */
""")


def test_the_mission_target_is_followed_and_turns_the_boat():
    run_case(r"""
    arm();
    start_mission(1u, 90.0f, 0.40f);
    run(300);
    sp.heading_deg = 180.0f;                           /* the TURN step */
    run(1500);
    const float err = fmodf(hdg - 180.0f + 540.0f, 360.0f) - 180.0f;
    assert(fabsf(err) < 10.0f);                        /* it got there */
    assert(fabsf(status().heading_target_deg - 180.0f) < 0.01f);
""")


@pytest.mark.parametrize("name,action,reason,expect_zero", [
    ("stop", "motor_control_request_auto_stop();", 1, True),
    ("manual", "motor_handler(&(boat_MotorCommand){.throttle = 0.25f});", 2, False),
    ("disarm", "esc_state = ESC_STATE_DISARMED;", 3, True),
    ("rail", "power_handler(false);", 4, True),
    ("bench", "compass_cal_running = true;", 5, True),
])
def test_every_way_of_ending_a_mission_acts_in_the_same_cycle(name, action, reason, expect_zero):
    run_case(r"""
    arm();
    start_mission(7u, 90.0f, 0.40f);
    run(200);
    assert(esc_left > 0.05f || esc_right > 0.05f);
    """ + action + r"""
    cycle(true);
    assert(abort_reason_of(7u) == %d);
    if (%d) assert(esc_left == 0.0f && esc_right == 0.0f);
    else assert(fabsf(0.5f * (esc_left + esc_right) - 0.25f) < 0.05f);   /* the stick's command */
    run(50);                                           /* the run stays over */
    assert(abort_reason_of(7u) == %d);
    if (%d) assert(esc_left == 0.0f && esc_right == 0.0f);
""" % (reason, int(expect_zero), reason, int(expect_zero)))


def test_a_stale_setpoint_stops_the_jets():
    run_case(r"""
    arm();
    start_mission(3u, 90.0f, 0.40f);
    run(100);
    sp.stamp_us = now_us;                              /* the task's last setpoint */
    motor_control_set_auto_setpoint(&sp);
    for (int i = 0; i < 49; ++i) cycle(false);        /* 0.49 s: still driving */
    assert(abort_reason_of(3u) == 0 && esc_left + esc_right > 0.1f);
    for (int i = 0; i < 10; ++i) cycle(false);        /* past 0.5 s */
    assert(abort_reason_of(3u) == 6);
    assert(esc_left == 0.0f && esc_right == 0.0f);
""")


def test_the_laptops_zero_stream_does_not_end_a_mission():
    run_case(r"""
    arm();
    start_mission(4u, 90.0f, 0.40f);
    for (int i = 0; i < 300; ++i) {
        motor_handler(&(boat_MotorCommand){0});        /* presence, every cycle */
        steer_handler(&(boat_SteerCommand){0});
        cycle(true);
    }
    assert(abort_reason_of(4u) == 0);
    assert(fabsf(0.5f * (esc_left + esc_right) - 0.40f) < 0.06f);
""")


def test_the_operators_p_switch_is_untouched_and_recaptures_fresh():
    run_case(r"""
    arm();
    assist_handler(false, false, 1u);                  /* the operator turned P OFF */
    motor_handler(&(boat_MotorCommand){.throttle = 0.40f});
    run(50);
    assert(!motor_control_p_assist_on());
    start_mission(5u, 45.0f, 0.40f);
    run(300);
    assert(!motor_control_p_assist_on());              /* never changed by the mission */
    assert(status().ctrl_active);                      /* ...but the hold flies it */
    assert(fabsf(status().heading_target_deg - 45.0f) < 0.01f);
    sp.active = false; sp.drive = false;              /* DONE: the mission lets go */
    run(20);
    assert(esc_left == 0.0f && esc_right == 0.0f);
    assert(!status().ctrl_active);                     /* P OFF again: no hold */

    assist_handler(true, false, 2u);                   /* now with P ON */
    start_mission(6u, 200.0f, 0.40f);
    run(300);
    sp.active = false; sp.drive = false;
    run(5);
    for (int i = 0; i < 200; ++i) { motor_handler(&(boat_MotorCommand){.throttle = 0.40f}); cycle(true); }
    boat_MotorStatus ms = status();
    assert(ms.ctrl_active && ms.heading_hold);
    /* a FRESH capture near where the boat points, never the mission's 200 */
    const float from_mission = fabsf(fmodf(ms.heading_target_deg - 200.0f + 540.0f, 360.0f) - 180.0f);
    const float from_boat = fabsf(fmodf(ms.heading_target_deg - hdg + 540.0f, 360.0f) - 180.0f);
    assert(from_boat < 20.0f);
    assert(from_mission > 1.0f || from_boat < 1.0f);
""", boot_on=True)


def test_the_learner_does_not_learn_during_a_mission():
    run_case(r"""
    arm();
    assist_handler(false, false, 1u);                  /* P OFF: the learner would learn */
    start_mission(8u, 90.0f, 0.40f);
    const float c0 = motor_control_trimlearn_c();
    run(1500);
    assert(motor_control_trimlearn_c() == c0);
""")


def test_bench_and_calibration_cannot_start_under_a_mission():
    run_case(r"""
    arm();
    start_mission(9u, 90.0f, 0.40f);
    run(50);
    bench_handler(0u, 0.30f, 0.0f, 0.0f, false);
    calibrate_handler(true, false);
    run(100);
    assert(abort_reason_of(9u) == 0);                  /* nothing else took the jets */
    boat_BenchStatus bs;
    motor_control_get_bench_status(&bs);
    assert(bs.state == 0u);                            /* the bench never started */
    assert(fabsf(0.5f * (esc_left + esc_right) - 0.40f) < 0.06f);
""")


def test_a_dry_run_owns_the_jets_at_zero_and_survives_disarm():
    run_case(r"""
    esc_state = ESC_STATE_DISARMED;
    sp = (auto_setpoint_t){.run_id = 11u, .active = true, .drive = false, .dry_run = true,
                           .heading_deg = 90.0f, .throttle = 0.0f, .stamp_us = now_us};
    motor_control_set_auto_setpoint(&sp);
    run(300);
    assert(abort_reason_of(11u) == 0);
    assert(esc_left == 0.0f && esc_right == 0.0f);
""")


def test_right_jet_stronger_turns_left_and_left_jet_stronger_turns_right():
    """Kiet's rule, pinned on the real firmware path: to turn RIGHT (target
    clockwise of the bow) the mission makes the LEFT jet stronger; to turn
    LEFT, the RIGHT jet.  (09-21 lake: L0/R0.6 turned left at +12.7 deg/s.)"""
    run_case(r"""
    arm();
    hdg = 90.0f; yaw_true = 0.0f;
    start_mission(12u, 150.0f, 0.40f);                 /* 60 deg clockwise: turn RIGHT */
    run(40);
    assert(esc_left > esc_right);                      /* left stronger */
    assert(yaw_true < 0.0f);                           /* the boat turns right (negative yaw) */
    sp.heading_deg = fmodf(hdg - 60.0f + 360.0f, 360.0f);   /* 60 deg anticlockwise: turn LEFT */
    run(60);
    assert(esc_right > esc_left);                      /* right stronger */
    assert(yaw_true > 0.0f);                           /* the boat turns left */
""")


def test_turning_p_off_mid_mission_changes_the_switch_not_the_hold():
    """The operator's switch still flips (and applies once the mission lets
    go), but the mission's heading hold keeps its integral -- no reset."""
    run_case(r"""
    arm();
    start_mission(13u, 90.0f, 0.40f);
    run(1500);                                         /* the integral has absorbed the mismatch */
    const float i_before = status().i_term;
    assert(fabsf(i_before) > 0.02f);
    assist_handler(false, false, 3u);
    run(3);
    assert(!motor_control_p_assist_on());
    assert(status().ctrl_active);
    assert(fabsf(status().i_term - i_before) < 0.01f);   /* not reset */
""")


def test_the_drive_snapshot_is_this_cycles_jets_without_the_esc_driver():
    """What the mission's recorder reads every 50 ms: exactly what this control
    cycle wrote to the jets, with the heading hold's terms from the same cycle
    -- from the control task's own copy, never through the ESC driver, whose
    mutex (no timeout) is the control task's alone."""
    run_case(r"""
    arm();
    start_mission(5, 120.0f, 0.40f);
    run(400);
    const unsigned reads = esc_get_throttle_calls;
    motor_drive_snapshot_t d;
    motor_control_get_drive_snapshot(&d);
    assert(esc_get_throttle_calls == reads);             /* never the driver */
    assert(d.left == esc_left && d.right == esc_right);  /* this cycle's jets */
    assert(d.left > 0.0f && d.right > 0.0f);
    assert(d.hold_active);
    assert(fabsf(d.hold_target_deg - 120.0f) < 1e-3f);
    /* the hold's own terms: this plant needs a split of 0.08 to go straight,
     * so after 4 s of holding the integral carries it */
    assert(isfinite(d.p_term) && isfinite(d.i_term) && fabsf(d.i_term) > 0.01f);
    /* and it follows the jets cycle by cycle, not the rate-limited MotorStatus */
    for (int i = 0; i < 20; ++i) {
        cycle(true);
        motor_control_get_drive_snapshot(&d);
        assert(d.left == esc_left && d.right == esc_right);
    }
""")



def test_a_calibration_asked_for_during_a_mission_never_starts_by_itself_after():
    """The tool keeps sending CalibrateCommand start keepalives once a
    calibration is asked for. Refused while the mission owns the jets -- and
    it must stay refused when the mission ends: the operator is catching the
    boat then, and a sweep starting on its own would drive the jets at them.
    A new start needs an explicit stop first (the latch the tool already
    honours after every sweep)."""
    run_case(r"""
    arm();
    start_mission(12u, 90.0f, 0.40f);
    run(50);
    for (int i = 0; i < 30; ++i) { calibrate_handler(true, false); cycle(true); }   /* keepalives */
    boat_CalibrateStatus cs;
    motor_control_get_calibrate_status(&cs);
    assert(cs.state == 0u && "refused under the mission");
    /* the mission ends; the keepalives keep coming */
    sp.active = false; sp.drive = false;
    for (int i = 0; i < 200; ++i) {
        if ((i % 5) == 0) motor_handler(&(boat_MotorCommand){0});
        calibrate_handler(true, false);
        cycle(true);
    }
    motor_control_get_calibrate_status(&cs);
    assert(cs.state == 0u && "never starts by itself when the mission ends");
    assert(esc_left == 0.0f && esc_right == 0.0f);
    /* stop, then start: the deliberate way still works */
    calibrate_handler(false, false); cycle(true);
    for (int i = 0; i < 20; ++i) { calibrate_handler(true, false); cycle(true); }
    motor_control_get_calibrate_status(&cs);
    assert(cs.state != 0u);
""")


def test_the_published_motor_status_shows_the_jets_during_a_mission():
    """The tool's Throttle L/R comes from MotorStatus. It used to read 0/0 for
    the whole run: the manual path wrote the laptop's zero first in every
    cycle, and the first status commit of the cycle took that zero and the
    10 Hz publish slot before the mission's write. (Fixed with the zero ESC
    write, 53390f1.)"""
    run_case(r"""
    arm();
    start_mission(5, 120.0f, 0.40f);
    unsigned zero_reports = 0, reports = 0;
    uint32_t gen0 = 0;
    for (int i = 0; i < 600; ++i) {
        if ((i % 7) == 0) motor_handler(&(boat_MotorCommand){0});   /* the laptop's zero presence */
        cycle(true);
        boat_MotorStatus ms;
        const uint32_t g = motor_control_get_status(&ms);
        if (i > 100 && g != gen0) {
            ++reports;
            if (ms.left_throttle == 0.0f && ms.right_throttle == 0.0f) ++zero_reports;
        }
        gen0 = g;
    }
    assert(reports > 20 && zero_reports == 0);
""")
