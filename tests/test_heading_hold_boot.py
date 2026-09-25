"""Motor P (heading hold + yaw-rate PI) is ON at every boot, and actually acts.

2026-09-25, out-and-back mission spec section 4: Kiet decided the boat boots
with Motor P ON. The laptop switch still turns it off until the next boot.

Built on the REAL motor_control.c WITH the heading hold compiled in. The older
runtime harness (tests/test_runtime_architecture.py) builds motor_control.c
with CONFIG_STABILITY_TRIMLEARN_ENABLE undefined, so it never executes a line
of the heading hold -- this file is where that code first runs on the host."""
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from tests.test_runtime_architecture import HARNESS, STUB_HEADERS

ROOT = Path(__file__).resolve().parents[1]

# The shared stub FusionResult predates heading_valid; the heading hold reads it.
FUSION_H = r"""
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { float pitch; float roll; float heading; bool heading_valid;
                 float yaw_rate; uint32_t sequence; uint64_t captured_us; } FusionResult;
void fusion_get_result(FusionResult *result);
"""

# The shared stubs with the IMU and fusion made controllable. Each original
# must occur exactly once, or the shared harness changed underneath this file.
STUB_SWAPS = (
    ("bool imu_icm_ok(void) { return false; }",
     "static bool imu_ok_flag = true;\n"
     "bool imu_icm_ok(void) { return imu_ok_flag; }"),
    ("void fusion_get_result(FusionResult *result) { (void)result; }",
     "static FusionResult fusion_now;\n"
     "void fusion_get_result(FusionResult *result) { *result = fusion_now; }"),
)

# The shipped Kconfig values (main/Kconfig.projbuild defaults == sdkconfig).
HEADING_HOLD_DEFINES = [
    "-DCONFIG_STABILITY_TRIMLEARN_ENABLE=1",
    '-DCONFIG_ESC_TRIM_C="0.21"',
    '-DCONFIG_STABILITY_TRIMLEARN_DEADBAND_DPS="0.5"',
    '-DCONFIG_STABILITY_TRIMLEARN_STEP_PER_S="0.005"',
    '-DCONFIG_STABILITY_TRIMLEARN_MIN_THROTTLE="0.15"',
    '-DCONFIG_STABILITY_YAW_PI_YAW_TAU_S="0.08"',
    '-DCONFIG_STABILITY_YAW_PI_RATE_KP="0.100"',
    '-DCONFIG_STABILITY_YAW_PI_RATE_KI="0.080"',
    '-DCONFIG_STABILITY_YAW_PI_HEADING_TAU_S="0.10"',
    '-DCONFIG_STABILITY_YAW_PI_HEADING_KP="1.50"',
    '-DCONFIG_STABILITY_YAW_PI_MAX_RATE_DPS="15.0"',
    '-DCONFIG_STABILITY_YAW_PI_RECAPTURE_S="0.50"',
]


def heading_hold_stubs():
    stubs = HARNESS[:HARNESS.index("int main(void) {")]
    for old, new in STUB_SWAPS:
        assert stubs.count(old) == 1, "shared harness changed: %r" % old
        stubs = stubs.replace(old, new)
    return stubs


def build_and_run(main_c, extra_defines=(), extra_sources=(), extra_headers=None):
    """Compile the real motor_control.c + the heading-hold sources against the
    shared stubs and run `main_c`. Returns the CompletedProcess."""
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        headers = dict(STUB_HEADERS)
        headers["sensor_fusion.h"] = FUSION_H
        headers.update(extra_headers or {})
        for relative, content in headers.items():
            path = tmpdir / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        shutil.copy(ROOT / "main" / "motor_control.c", tmpdir / "motor_control.c")
        (tmpdir / "harness.c").write_text(heading_hold_stubs() + main_c)
        binary = tmpdir / "heading_hold_test"
        subprocess.run(
            ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
             *HEADING_HOLD_DEFINES, *extra_defines,
             "-I", str(tmpdir), "-I", str(ROOT / "main"),
             str(tmpdir / "motor_control.c"),
             str(ROOT / "main" / "control_arbiter.c"),
             str(ROOT / "main" / "arm_sequence.c"),
             str(ROOT / "main" / "esc_trim.c"),
             str(ROOT / "main" / "esc_trim_cal.c"),
             str(ROOT / "main" / "bench_run.c"),
             str(ROOT / "main" / "trim_learn.c"),
             str(ROOT / "main" / "yaw_heading_control.c"),
             *[str(s) for s in extra_sources],
             str(tmpdir / "harness.c"), "-lm", "-o", str(binary)],
            check=True)
        return subprocess.run([str(binary)], capture_output=True, text=True)


BOOT_MAIN = r"""
#ifndef EXPECT_BOOT_ON
#error "EXPECT_BOOT_ON must be defined 0 or 1"
#endif

/* One fusion sample every 20 ms, like the real 50 Hz fusion task. */
static void fusion_tick(float yaw_dps, float heading_deg) {
    fusion_now.yaw_rate = yaw_dps;
    fusion_now.heading = heading_deg;
    fusion_now.heading_valid = true;
    fusion_now.sequence++;
    fusion_now.captured_us = (uint64_t)now_us;
}

/* 10 ms control ticks with the pilot holding `thr` straight ahead. */
static void drive(int ticks, float thr, float yaw_dps) {
    for (int i = 0; i < ticks; ++i) {
        now_us += 10000;
        if ((i % 2) == 0) fusion_tick(yaw_dps, 90.0f);
        motor_handler(&(boat_MotorCommand){.throttle = thr});
        run_one_control_cycle();
    }
}

int main(void) {
    (void)run_scheduled_and_urgent_cycle; (void)close_enough; (void)imu_ok_flag;
    assert(motor_control_init() == ESP_OK);

    /* 1. The boot state -- and the very first MotorStatus already carries it,
     *    so the laptop never sees a false OFF at connect. */
    assert(motor_control_p_assist_on() == (bool)EXPECT_BOOT_ON);
    boat_MotorStatus ms;
    assert(motor_control_get_status(&ms) != 0u);
    assert(ms.assist_motor_p == (bool)EXPECT_BOOT_ON);

    /* 2. Armed, 40 % straight ahead, hull yawing RIGHT at 20 deg/s (negative
     *    is right on this boat). Nobody has pressed anything. */
    esc_state = ESC_STATE_ARMED;
    servo_power = true;
    drive(40, 0.40f, -20.0f);
    const float plain_split = 2.0f * 0.21f * 0.40f;   /* right-left, learned c only */
    const float split_boot = esc_right - esc_left;
#if EXPECT_BOOT_ON
    /* The controller pushes the RIGHT jet up to stop the right turn, far
     * beyond the plain learned trim: it is running without being asked. */
    assert(split_boot > plain_split + 0.30f);
    assert(motor_control_get_status(&ms) != 0u);
    assert(ms.ctrl_active);
#else
    assert(fabsf(split_boot - plain_split) < 0.01f);
#endif

    /* 3. The switch still works. OFF returns to the plain trim at once ... */
    assist_handler(false, false, 7u);
    drive(1, 0.40f, -20.0f);
    assert(!motor_control_p_assist_on());
    assert(fabsf((esc_right - esc_left) - plain_split) < 0.01f);

    /* ... and ON brings the correction back. */
    assist_handler(true, false, 8u);
    drive(20, 0.40f, -20.0f);
    assert(motor_control_p_assist_on());
    assert(esc_right - esc_left > plain_split + 0.30f);

    printf("split_boot=%.3f plain=%.3f\n", (double)split_boot, (double)plain_split);
    return 0;
}
"""


def test_p_is_on_at_boot_and_acts_without_being_asked():
    result = build_and_run(BOOT_MAIN, ["-DCONFIG_STABILITY_YAW_PI_DEFAULT_ON=1",
                                       "-DEXPECT_BOOT_ON=1"])
    assert result.returncode == 0, result.stdout + result.stderr


def test_the_kconfig_option_really_controls_the_boot_state():
    """`n` in menuconfig means the symbol is ABSENT from sdkconfig.h. The boat
    must then boot with P OFF, exactly as before this change."""
    result = build_and_run(BOOT_MAIN, ["-DEXPECT_BOOT_ON=0"])
    assert result.returncode == 0, result.stdout + result.stderr


def test_the_shipped_default_is_on():
    kconfig = (ROOT / "main" / "Kconfig.projbuild").read_text()
    m = re.search(r"config STABILITY_YAW_PI_DEFAULT_ON\n\s+bool [^\n]*\n\s+default (\w)",
                  kconfig)
    assert m, "STABILITY_YAW_PI_DEFAULT_ON missing from main/Kconfig.projbuild"
    assert m.group(1) == "y"
    sdk = ROOT / "sdkconfig"          # gitignored, local build config
    if sdk.exists():
        text = sdk.read_text()
        assert "# CONFIG_STABILITY_YAW_PI_DEFAULT_ON is not set" not in text, (
            "the local sdkconfig switches Motor P OFF at boot -- the flashed boat "
            "would not match the tool's expectations")
