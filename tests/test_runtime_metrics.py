import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMPILE_COMMAND = [
    os.environ.get("CC", "cc"),
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I", str(ROOT / "main"),
    str(ROOT / "main" / "runtime_schedule.c"),
    str(ROOT / "main" / "runtime_metrics.c"),
    str(ROOT / "tests" / "test_runtime_metrics.c"),
]


class RuntimeMetricsTest(unittest.TestCase):
    def test_accumulates_cycle_timing_without_allocating(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "runtime_metrics_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)


if __name__ == "__main__":
    unittest.main()
