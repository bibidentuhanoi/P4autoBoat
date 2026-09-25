"""Host test: the out-and-back mission sequencer on an ideal boat."""
import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def test_mission_host_binary():
    with tempfile.TemporaryDirectory() as td:
        binary = Path(td) / "test_mission"
        subprocess.run([
            os.environ.get("CC", "cc"),
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wdouble-promotion",
            "-I", str(ROOT / "main"),
            str(ROOT / "main" / "mission.c"),
            str(ROOT / "main" / "nav_geo.c"),
            str(ROOT / "main" / "course_error.c"),
            str(ROOT / "tests" / "test_mission.c"),
            "-lm", "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
