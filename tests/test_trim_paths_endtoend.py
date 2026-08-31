import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class TrimPathsEndToEndTest(unittest.TestCase):
    """Bench run, linked slider and unlinked slider must all trim the same."""

    def test_all_three_paths_apply_the_same_trim(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "trim_paths_test"
            subprocess.run([
                os.environ.get("CC", "cc"),
                "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "main"),
                str(ROOT / "main" / "esc_trim.c"),
                str(ROOT / "main" / "bench_run.c"),
                str(ROOT / "main" / "trim_learn.c"),
                str(ROOT / "tests" / "test_trim_paths_endtoend.c"),
                "-lm", "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
