import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_gps_snapshot_readers_never_observe_mixed_generations():
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        (tmpdir / "esp_err.h").write_text(
            "#pragma once\ntypedef int esp_err_t;\n"
            "#define ESP_OK 0\n#define ESP_FAIL -1\n#define ESP_ERR_INVALID_ARG 0x102\n"
        )
        binary = tmpdir / "gps_snapshot_test"
        subprocess.run(
            [
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-pthread",
                "-I", str(tmpdir), "-I", str(ROOT / "main"),
                str(ROOT / "main" / "gps_snapshot.c"),
                str(ROOT / "tests" / "test_gps_snapshot.c"),
                "-o", str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)
