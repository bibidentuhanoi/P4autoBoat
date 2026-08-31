import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class StabilityControlTest(unittest.TestCase):
    def test_yaw_rate_damper_tracks_pilot_demand_with_capped_authority(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "stability_control_test"
            subprocess.run(
                [
                    os.environ.get("CC", "cc"),
                    "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "main"),
                    str(ROOT / "main" / "stability_control.c"),
                    str(ROOT / "tests" / "test_stability_control.c"),
                    "-lm", "-o", str(binary),
                ],
                check=True,
            )
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
