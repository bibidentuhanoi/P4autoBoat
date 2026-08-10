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
    str(ROOT / "main" / "sensor_schedule.c"),
    str(ROOT / "tests" / "test_sensor_schedule.c"),
]


class SensorScheduleTest(unittest.TestCase):
    def test_imu_first_deadline_aware_tof_policy(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "sensor_schedule_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)


if __name__ == "__main__":
    unittest.main()
