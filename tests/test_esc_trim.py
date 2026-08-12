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
    str(ROOT / "main" / "esc_trim.c"),
    str(ROOT / "tests" / "test_esc_trim.c"),
]


class EscTrimTest(unittest.TestCase):
    def test_lookup_interpolates_and_apply_preserves_common_and_turn(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "esc_trim_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)


if __name__ == "__main__":
    unittest.main()
