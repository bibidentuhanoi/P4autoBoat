"""Host test: the AUTO owner: when a mission may drive the jets, and how it stops."""
import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def test_auto_drive_host_binary():
    with tempfile.TemporaryDirectory() as td:
        binary = Path(td) / "test_auto_drive"
        subprocess.run([
            os.environ.get("CC", "cc"),
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wdouble-promotion",
            "-I", str(ROOT / "main"),
            str(ROOT / "main" / "auto_drive.c"),
            str(ROOT / "main" / "esc_trim.c"),
            str(ROOT / "main" / "yaw_heading_control.c"),
            str(ROOT / "tests" / "test_auto_drive.c"),
            "-lm", "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
