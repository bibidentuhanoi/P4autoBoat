import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_critical_failure_disarms_while_optional_failure_only_disables_feature():
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        (tmpdir / "drivers").mkdir()
        (tmpdir / "esp_err.h").write_text(
            "#pragma once\ntypedef int esp_err_t;\n"
            "#define ESP_OK 0\n#define ESP_ERR_INVALID_ARG 0x102\n#define ESP_ERR_NO_MEM 0x101\n"
        )
        (tmpdir / "esp_log.h").write_text(
            "#pragma once\nvoid test_log(const char *, const char *, ...);\n"
            "#define ESP_LOGE(...) test_log(__VA_ARGS__)\n"
            "#define ESP_LOGW(...) test_log(__VA_ARGS__)\n"
        )
        (tmpdir / "motor_control.h").write_text(
            "#pragma once\nvoid motor_control_disarm(void);\n"
        )
        (tmpdir / "drivers" / "winch_driver.h").write_text(
            "#pragma once\n#include <stdbool.h>\n#include \"esp_err.h\"\n"
            "esp_err_t winch_driver_set_power(bool on);\n"
        )
        binary = tmpdir / "runtime_startup_policy_test"
        subprocess.run(
            [
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(tmpdir), "-I", str(ROOT / "main"),
                str(ROOT / "main" / "runtime_startup.c"),
                str(ROOT / "tests" / "test_runtime_startup_policy.c"),
                "-o", str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)
