import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_both_four_target_tofs_use_fast_mode_plus_without_longer_bus_holds():
    kconfig = (ROOT / "main" / "Kconfig.projbuild").read_text()
    defaults = (ROOT / "sdkconfig.defaults").read_text()
    driver = (ROOT / "main" / "drivers" / "tof_driver.c").read_text()
    platform = (ROOT / "main" / "drivers" / "vl53l5cx_platform.c").read_text()

    assert re.search(
        r"config TOF_I2C_FREQ_HZ\s+int .*?\s+default 1000000\s+range 400000 1000000",
        kconfig,
        re.S,
    )
    assert "CONFIG_TOF_I2C_FREQ_HZ=1000000" in defaults
    assert ".scl_speed_hz    = CONFIG_TOF_I2C_FREQ_HZ" in driver

    # One probe function configures both device handles. Keep both calls pinned:
    # a fast A-only implementation would collapse when the second sensor lands.
    assert "&devices->dev_a" in driver
    assert "&devices->dev_b" in driver

    # Four returns per 8x8 zone are deliberate for vegetation/water scenes.
    assert "CONFIG_VL53L5CX_NB_TARGET_PER_ZONE=4" in defaults

    # 128 B at 1 MHz holds the bus for less time than the old 64 B at 400 kHz.
    # Keep a safe 64 B chunk if somebody deliberately lowers the configured bus.
    assert "CONFIG_TOF_I2C_FREQ_HZ >= 800000" in platform
    assert "#define VL53L5CX_I2C_READ_CHUNK_SIZE 128U" in platform
    assert "#define VL53L5CX_I2C_READ_CHUNK_SIZE 64U" in platform


def test_timing_log_gate_aggregates_repeated_slow_operations(tmp_path):
    source = tmp_path / "timing_log_gate_test.c"
    binary = tmp_path / "timing_log_gate_test"
    source.write_text(
        r'''
#include <assert.h>
#include <stdint.h>
#include "timing_log_gate.h"

int main(void) {
    timing_log_gate_t gate = {0};
    uint32_t count = 0, maximum = 0;

    assert(timing_log_gate_record(&gate, 1000000, 6000, &count, &maximum));
    assert(count == 1 && maximum == 6000);

    assert(!timing_log_gate_record(&gate, 1100000, 7000, &count, &maximum));
    assert(!timing_log_gate_record(&gate, 1500000, 6500, &count, &maximum));

    assert(timing_log_gate_record(&gate, 2000000, 8000, &count, &maximum));
    assert(count == 3 && maximum == 8000);

    assert(!timing_log_gate_record(&gate, 2500000, 9000, &count, &maximum));
    assert(timing_log_gate_record(&gate, 3000000, 7500, &count, &maximum));
    assert(count == 2 && maximum == 9000);
    return 0;
}
'''
    )
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "main" / "include"),
            str(source),
            "-o",
            str(binary),
        ],
        check=True,
    )
    subprocess.run([str(binary)], check=True)


def test_sensor_tasks_rate_limit_timing_warning_storms():
    source = (ROOT / "main" / "sensor_task.c").read_text()

    assert "timing_log_gate_record(&s_imu_timing_log" in source
    assert "timing_log_gate_record(&s_tof_timing_log[selected]" in source
    assert "slow cycles=%lu" in source
    assert "slow reads=%lu" in source
