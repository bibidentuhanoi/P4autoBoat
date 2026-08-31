import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class TrimAssistTest(unittest.TestCase):
    """The temporary P correction: both signs, the cap, the bounds, and every
    way it must forget itself."""

    def test_assist_rules(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "trim_assist_test"
            subprocess.run([
                os.environ.get("CC", "cc"),
                "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "main"),
                str(ROOT / "main" / "trim_assist.c"),
                str(ROOT / "tests" / "test_trim_assist.c"),
                "-lm", "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
