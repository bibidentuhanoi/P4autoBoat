import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class BenchRunStateMachineTest(unittest.TestCase):
    def test_profile_records_every_phase_and_never_stops_early(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "bench_run_test"
            subprocess.run([
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "main"),
                "-I", str(ROOT / "main" / "include"),
                str(ROOT / "main" / "bench_run.c"),
                str(ROOT / "main" / "esc_trim.c"),
                str(ROOT / "tests" / "test_bench_run.c"),
                "-lm", "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
