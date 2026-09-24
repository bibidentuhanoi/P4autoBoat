import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def test_frozen_imu_detector():
    with tempfile.TemporaryDirectory() as tmp:
        binary = Path(tmp) / "imu_freeze_test"
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(ROOT / "main"), str(ROOT / "tests" / "test_imu_freeze.c"),
                        "-o", str(binary)], check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        assert "all tests passed" in result.stdout
