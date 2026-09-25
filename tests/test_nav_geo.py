"""Host test: geo maths of the mission: local metres, bearings, the finish line and LOS."""
import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def test_nav_geo_host_binary():
    with tempfile.TemporaryDirectory() as td:
        binary = Path(td) / "test_nav_geo"
        subprocess.run([
            os.environ.get("CC", "cc"),
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wdouble-promotion",
            "-I", str(ROOT / "main"),
            str(ROOT / "main" / "nav_geo.c"),
            str(ROOT / "tests" / "test_nav_geo.c"),
            "-lm", "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
