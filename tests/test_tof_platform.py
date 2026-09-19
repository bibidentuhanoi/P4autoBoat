import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "managed_components" / "rjrp44__vl53l5cx"
PLATFORM_SOURCE = ROOT / "main" / "drivers" / "vl53l5cx_platform.c"
COMPILE_COMMAND = [
    os.environ.get("CC", "cc"),
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-Wno-unused-parameter",
    "-DCONFIG_VL53L5CX_RESET_PIN_LOW=1",
    "-DESP_OK=0",
    "-I", str(ROOT / "tests" / "tof_platform_stubs"),
    "-I", str(COMPONENT / "include"),
    str(PLATFORM_SOURCE),
    str(ROOT / "tests" / "test_tof_platform.c"),
]


class ToFPlatformTest(unittest.TestCase):
    def test_large_reads_release_the_shared_bus_in_bounded_chunks(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "tof_platform_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)


if __name__ == "__main__":
    unittest.main()
