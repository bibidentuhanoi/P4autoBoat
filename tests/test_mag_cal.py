import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class MagCalTest(unittest.TestCase):
    def test_compass_calibration_maths(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "mag_cal_test"
            subprocess.run(
                [
                    os.environ.get("CC", "cc"),
                    "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1",
                    "-I", str(ROOT / "main"),
                    str(ROOT / "main" / "mag_cal.c"),
                    str(ROOT / "tests" / "test_mag_cal.c"),
                    "-lm", "-o", str(binary),
                ],
                check=True,
            )
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("all tests passed", result.stdout)


if __name__ == "__main__":
    unittest.main()
