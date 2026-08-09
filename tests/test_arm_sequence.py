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
    str(ROOT / "main" / "arm_sequence.c"),
    str(ROOT / "tests" / "test_arm_sequence.c"),
]


class ArmSequenceTest(unittest.TestCase):
    def test_gps_gate_deadline_and_urgent_disarm(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "arm_sequence_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)


if __name__ == "__main__":
    unittest.main()
