"""Host test: the GPS-course-minus-heading (beta) estimator of the mission."""
import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def test_course_error_host_binary():
    with tempfile.TemporaryDirectory() as td:
        binary = Path(td) / "test_course_error"
        subprocess.run([
            os.environ.get("CC", "cc"),
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wdouble-promotion",
            "-I", str(ROOT / "main"),
            str(ROOT / "main" / "course_error.c"),
            str(ROOT / "tests" / "test_course_error.c"),
            "-lm", "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
