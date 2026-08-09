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
    str(ROOT / "main" / "control_arbiter.c"),
    str(ROOT / "tests" / "test_control_arbiter.c"),
]


class ControlArbiterTest(unittest.TestCase):
    def test_manual_only_commands_follow_the_control_policy(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "control_arbiter_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)


if __name__ == "__main__":
    unittest.main()
