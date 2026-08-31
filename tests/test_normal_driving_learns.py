import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class NormalDrivingLearnsTest(unittest.TestCase):
    """The learner has to work while the pilot just drives, not only inside a
    BASE bench run. Every other trim test goes through the bench, so without
    this one the learner could become bench-only and nothing would notice."""

    def test_ordinary_forward_driving_teaches_the_learner(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "normal_driving_test"
            subprocess.run([
                os.environ.get("CC", "cc"),
                "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "main"),
                str(ROOT / "main" / "trim_learn.c"),
                str(ROOT / "main" / "esc_trim.c"),
                str(ROOT / "tests" / "test_normal_driving_learns.c"),
                "-lm", "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
