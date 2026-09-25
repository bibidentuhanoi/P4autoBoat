"""Host test: the path follower and the planner slot (the ground for waypoint
missions and a SUSHI-style planner)."""
import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def test_path_follow_host_binary():
    with tempfile.TemporaryDirectory() as td:
        binary = Path(td) / "test_path_follow"
        subprocess.run([
            os.environ.get("CC", "cc"),
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wdouble-promotion",
            "-I", str(ROOT / "main"),
            str(ROOT / "main" / "path_follow.c"),
            str(ROOT / "main" / "planner.c"),
            str(ROOT / "main" / "nav_geo.c"),
            str(ROOT / "tests" / "test_path_follow.c"),
            "-lm", "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
