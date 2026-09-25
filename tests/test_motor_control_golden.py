"""Driving WITHOUT a mission is exactly what it was.

Kiet, 2026-09-25: "do not break the auto trim, the heading holding and the P
algo that already work".  The mission's AUTO owner lives in motor_control.c,
and every line it touched must reduce to the old code while no mission runs.
This is checked on behaviour, not by reading the diff:

  * the motor_control.c of a baseline commit and the current one are built
    into the SAME harness -- the current one WITH the mission compiled in
    (CONFIG_MISSION_ENABLE=1) and idle;
  * both fly the same closed-loop boat through the same long script: ordinary
    driving at many throttles, turning by motor difference, by the rudder and
    by raw pulse, recapture, Motor P off (the learner learning) and on, link
    loss and return, IMU dropout, heading invalid, disarm with the slider up,
    servo PWR-OFF/ON, winch, BASE and LEFT bench runs, then a long random walk
    of stick commands and P toggles;
  * every 10 ms cycle prints both ESC commands, the learned c, P on/off,
    rudder, rail, and the heading hold's output (MotorStatus) as hex floats,
    and the two runs must be identical, line for line.

Baselines: ed20e29 (lake-id before the AUTO owner) booting with P ON, and
c8a4c5e (main, the proven boat firmware) booting with P OFF as it does -- the
boot state is the one approved change of Part 1.
"""
import subprocess
import tempfile
from pathlib import Path

import pytest

from tests.test_heading_hold_boot import FUSION_H, HEADING_HOLD_DEFINES, heading_hold_stubs
from tests.test_runtime_architecture import STUB_HEADERS

ROOT = Path(__file__).resolve().parents[1]
MISSION_DEFINE = "-DCONFIG_MISSION_ENABLE=1"
BOOT_ON_DEFINE = "-DCONFIG_STABILITY_YAW_PI_DEFAULT_ON=1"

SCRIPT = r"""
#include <math.h>

/* A small closed-loop boat, so the heading hold and the learner really act:
 * yaw answers the jet split with a lag, against a fixed mismatch, plus wave
 * noise; heading integrates it.  Deterministic: the same numbers every run. */
static uint32_t lcg = 20260925u;
static float frand(void) { lcg = lcg * 1664525u + 1013904223u; return (float)(lcg >> 8) / 16777216.0f; }
static float gauss(void) { float s = 0.0f; for (int i = 0; i < 12; ++i) s += frand(); return s - 6.0f; }
static float yaw_true = 0.0f, hdg = 90.0f, wave = 0.0f;
static bool heading_ok = true;
static int cyc = 0;

static void plant_step(void) {
    const float split = esc_right - esc_left;              /* + = right jet harder: turns LEFT (+) */
    wave += -wave * 0.01f / 1.5f + 0.25f * gauss();
    const float target = 45.0f * (split - 0.08f) + wave;
    yaw_true += (target - yaw_true) * 0.01f / 0.8f;
    hdg = fmodf(hdg - yaw_true * 0.01f + 360.0f, 360.0f);
}

static void cycle(void) {
    now_us += 10000;
    plant_step();
    if ((cyc % 2) == 0) {                                   /* fusion at 50 Hz */
        fusion_now.yaw_rate = yaw_true + 0.4f * gauss();
        fusion_now.heading = fmodf(hdg + 0.5f * gauss() + 360.0f, 360.0f);
        fusion_now.heading_valid = heading_ok;
        fusion_now.sequence++;
        fusion_now.captured_us = (uint64_t)now_us;
    }
    run_one_control_cycle();
    boat_MotorStatus ms;
    motor_control_get_status(&ms);
    printf("%d %a %a %a %d %a %d %d | %a %a %a %a %a %a %d %d %d\n", cyc,
           esc_left, esc_right, motor_control_trimlearn_c(), (int)motor_control_p_assist_on(),
           steer_value, (int)servo_power, (int)esc_state,
           ms.dynamic_c, ms.effective_c, ms.heading_target_deg, ms.heading_error_deg,
           ms.p_term, ms.i_term, (int)ms.ctrl_active, (int)ms.heading_hold, (int)ms.saturated);
    ++cyc;
}

static void pilot(float thr, float rud) {
    motor_handler(&(boat_MotorCommand){.throttle = thr, .rudder = rud});
}

static void drive(int n, float thr, float rud) {
    for (int i = 0; i < n; ++i) { pilot(thr, rud); cycle(); }
}

int main(void) {
    (void)run_scheduled_and_urgent_cycle; (void)close_enough;
    assert(motor_control_init() == ESP_OK);
    drive(50, 0.0f, 0.0f);                                   /* disarmed, laptop streaming zeros */
    esc_state = ESC_STATE_ARMED;
    servo_power = true;
    drive(1500, 0.40f, 0.0f);                                /* straight, boot P state */
    assist_handler(false, false, 1u);                        /* P OFF: the learner learns */
    drive(1500, 0.40f, 0.0f);
    assist_handler(true, false, 2u);                         /* P ON */
    drive(1000, 0.40f, 0.0f);
    drive(200, 0.40f, 0.30f);                                /* turn by motor difference */
    drive(400, 0.40f, 0.0f);                                 /* recapture */
    drive(200, 0.40f, -0.30f);
    drive(300, 0.40f, 0.0f);
    steer_handler(&(boat_SteerCommand){.left = 0.5f, .right = 0.5f});   /* rudder turn */
    drive(150, 0.30f, 0.0f);
    steer_handler(&(boat_SteerCommand){.left = 0.0f, .right = 0.0f});
    drive(300, 0.30f, 0.0f);
    raw_handler(&(boat_SteerRawCommand){.pulse_us = 1600u});            /* raw pulse */
    drive(60, 0.30f, 0.0f);
    steer_handler(&(boat_SteerCommand){.left = 0.0f, .right = 0.0f});
    drive(200, 0.30f, 0.0f);
    for (int i = 0; i < 120; ++i) cycle();                   /* link loss: nothing arrives */
    drive(300, 0.35f, 0.0f);                                 /* link back */
    imu_ok_flag = false; drive(40, 0.35f, 0.0f); imu_ok_flag = true;
    heading_ok = false; drive(60, 0.35f, 0.0f); heading_ok = true;
    drive(300, 0.35f, 0.0f);
    const float ramp[] = {0.10f, 0.15f, 0.20f, 0.25f, 0.45f, 0.60f, 1.0f, 0.30f};
    for (unsigned k = 0; k < sizeof(ramp) / sizeof(ramp[0]); ++k) drive(200, ramp[k], 0.0f);
    power_handler(false); drive(60, 0.30f, 0.0f);            /* explicit PWR-OFF */
    power_handler(true);  drive(120, 0.30f, 0.0f);
    winch_handler(&(boat_WinchCommand){.speed = 0.5f}); drive(50, 0.20f, 0.0f);
    winch_handler(&(boat_WinchCommand){.speed = 0.0f}); drive(50, 0.20f, 0.0f);
    esc_state = ESC_STATE_DISARMED; drive(100, 0.40f, 0.0f); /* slider up, jets dead */
    esc_state = ESC_STATE_ARMED; drive(300, 0.40f, 0.0f);
    drive(20, 0.0f, 0.0f);
    bench_handler(0u, 0.30f, 0.0f, 0.0f, false);             /* BASE run */
    drive(600, 0.0f, 0.0f);
    motor_control_bench_flush();
    bench_handler(1u, 0.30f, 0.10f, 0.0f, false);            /* LEFT run */
    drive(600, 0.0f, 0.0f);
    motor_control_bench_flush();
    drive(300, 0.40f, 0.0f);
    /* A long random walk: stick nudges, pauses, P toggles. */
    float thr = 0.35f, rud = 0.0f;
    for (int i = 0; i < 6000; ++i) {
        const float r = frand();
        if (r < 0.02f) thr = 0.10f + 0.50f * frand();
        else if (r < 0.04f) rud = (frand() < 0.6f) ? 0.0f : 0.4f * (frand() - 0.5f);
        else if (r < 0.045f) assist_handler(frand() < 0.5f, false, (uint32_t)(100 + i));
        if (frand() < 0.01f) { cycle(); continue; }         /* a dropped command */
        pilot(thr, rud);
        cycle();
    }
    return 0;
}
"""


def _source_at(rev):
    out = subprocess.run(["git", "-C", str(ROOT), "show", f"{rev}:main/motor_control.c"],
                         capture_output=True, text=True)
    if out.returncode != 0:
        pytest.skip(f"{rev} is not in this clone's history")
    return out.stdout


def _run(motor_control_src, defines):
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        headers = dict(STUB_HEADERS)
        headers["sensor_fusion.h"] = FUSION_H
        for relative, content in headers.items():
            path = tmpdir / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        (tmpdir / "motor_control.c").write_text(motor_control_src)
        (tmpdir / "harness.c").write_text(heading_hold_stubs() + SCRIPT)
        binary = tmpdir / "golden"
        subprocess.run(
            ["cc", "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", *HEADING_HOLD_DEFINES, *defines,
             "-I", str(tmpdir), "-I", str(ROOT / "main"),
             str(tmpdir / "motor_control.c"),
             *[str(ROOT / "main" / s) for s in ("control_arbiter.c", "arm_sequence.c", "esc_trim.c",
                                               "esc_trim_cal.c", "bench_run.c", "trim_learn.c",
                                               "yaw_heading_control.c", "auto_drive.c")],
             str(tmpdir / "harness.c"), "-lm", "-o", str(binary)],
            check=True)
        res = subprocess.run([str(binary)], capture_output=True, text=True)
        assert res.returncode == 0, res.stderr[-2000:]
        return res.stdout.splitlines()


def _compare(old, new):
    assert len(old) == len(new) > 10000, (len(old), len(new))
    first = next((i for i, (a, b) in enumerate(zip(old, new)) if a != b), None)
    assert first is None, "cycle %d differs:\n old %s\n new %s" % (first, old[first], new[first])


def _current():
    return (ROOT / "main" / "motor_control.c").read_text()


def test_driving_is_bit_identical_to_lake_id_before_the_auto_owner():
    old = _run(_source_at("ed20e29"), [BOOT_ON_DEFINE])
    new = _run(_current(), [BOOT_ON_DEFINE, MISSION_DEFINE])
    # the script really exercised the hold and the learner
    assert any(l.split(" | ")[1].split()[6] == "1" for l in new), "the heading hold never ran"
    assert len({l.split()[3] for l in new}) > 20, "the learned c never moved"
    _compare(old, new)


def test_driving_is_bit_identical_to_main_when_booted_the_same_way():
    old = _run(_source_at("c8a4c5e"), [])
    new = _run(_current(), [MISSION_DEFINE])        # P boots OFF, as on main
    _compare(old, new)


@pytest.mark.parametrize("needle,mutation", [
    # the learner keeps learning while P owns the trim
    ("healthy && !s_p_assist_on && !mission);", "healthy && !mission);"),
    # a manual turn no longer drops P at once (one 10 ms tick late)
    ("const bool hold_steering = steering && !mission;", "const bool hold_steering = false;"),
])
def test_the_golden_comparison_can_fail(needle, mutation):
    """Changes to what the no-mission path does must be caught -- including a
    one-tick one; a harness that always matches proves nothing."""
    src = _current()
    assert src.count(needle) == 1, needle
    old = _run(_source_at("ed20e29"), [BOOT_ON_DEFINE])
    new = _run(src.replace(needle, mutation), [BOOT_ON_DEFINE, MISSION_DEFINE])
    with pytest.raises(AssertionError):
        _compare(old, new)
