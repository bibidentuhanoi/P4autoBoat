import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class EscTrimCalTest(unittest.TestCase):
    def test_state_machine_converges_and_aborts_safely(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "esc_trim_cal_test"
            subprocess.run([
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "main"),
                "-I", str(ROOT / "main" / "include"),
                str(ROOT / "main" / "esc_trim_cal.c"),
                str(ROOT / "tests" / "test_esc_trim_cal.c"),
                "-lm", "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
