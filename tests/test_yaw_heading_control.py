import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def test_yaw_heading_controller_host_binary():
    with tempfile.TemporaryDirectory() as td:
        binary = Path(td) / "test_yaw_heading_control"
        subprocess.run([
            os.environ.get("CC", "cc"),
            "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT / "main"),
            str(ROOT / "main" / "yaw_heading_control.c"),
            str(ROOT / "tests" / "test_yaw_heading_control.c"),
            "-lm", "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
