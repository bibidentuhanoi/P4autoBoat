import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_uart_events_dispatch_exact_data_and_recover_overflows():
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        (tmpdir / "esp_err.h").write_text(
            "#pragma once\ntypedef int esp_err_t;\n"
            "#define ESP_OK 0\n#define ESP_ERR_INVALID_ARG 0x102\n"
        )
        binary = tmpdir / "gps_uart_events_test"
        subprocess.run(
            [
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(tmpdir), "-I", str(ROOT / "main"),
                str(ROOT / "main" / "gps_uart_events.c"),
                str(ROOT / "tests" / "test_gps_uart_events.c"),
                "-o", str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)
