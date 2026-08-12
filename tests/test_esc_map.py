import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class EscMapTest(unittest.TestCase):
    def test_esc_map_preserves_off_full_and_skips_deadband(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "esc_map_test"
            subprocess.run(
                [
                    os.environ.get("CC", "cc"),
                    "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "main"),
                    str(ROOT / "main" / "esc_map.c"),
                    str(ROOT / "tests" / "test_esc_map.c"),
                    "-o", str(binary),
                ],
                check=True,
            )
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
