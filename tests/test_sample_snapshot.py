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
    "-pthread",
    "-I", str(ROOT / "main"),
    str(ROOT / "main" / "sample_snapshot.c"),
    str(ROOT / "tests" / "test_sample_snapshot.c"),
]


class SampleSnapshotTest(unittest.TestCase):
    def test_versioned_snapshot_never_returns_mixed_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "sample_snapshot_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)


if __name__ == "__main__":
    unittest.main()
