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
    "-Werror",
    "-I", str(ROOT / "tests" / "wifi_fallback_stubs"),
    "-I", str(ROOT / "main"),
    str(ROOT / "main" / "wifi_manager.c"),
    str(ROOT / "tests" / "test_wifi_fallback.c"),
]


class WifiFallbackTest(unittest.TestCase):
    def test_fallback_gets_ip_without_stopping_hosted_netif(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "wifi_fallback_test"
            subprocess.run(COMPILE_COMMAND + ["-o", str(binary)], check=True)
            completed = subprocess.run([str(binary)], check=False)
            self.assertEqual(0, completed.returncode)


if __name__ == "__main__":
    unittest.main()
