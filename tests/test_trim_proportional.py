import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class TrimProportionalTest(unittest.TestCase):
    """split = c * throttle, on both the manual and the bench path."""

    def test_proportional_trim_matches_the_bench_measurements(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "trim_proportional_test"
            subprocess.run([
                os.environ.get("CC", "cc"),
                "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "main"),
                str(ROOT / "main" / "esc_trim.c"),
                str(ROOT / "tests" / "test_trim_proportional.c"),
                "-lm", "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
