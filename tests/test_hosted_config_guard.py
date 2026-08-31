"""The ESP32-C6 coprocessor configuration guard.

A corrupted sdkconfig builds a binary that links, boots, looks healthy and has
NO LINK TO THE C6 -- no WiFi, no ESP-NOW, no dashboard, no python tool.
Observed 2026-08-31 after `idf.py fullclean`: CP_TARGET collapsed to ESP32H2 (a
chip with no WiFi), board NONE, SDIO gone, every WIFI_RMT_* symbol absent, 182
settings lost -- and a subsequent build SUCCEEDED against that wrong target.

The guard runs at CMake configure time. These exercise its logic standalone,
because a guard nobody has watched fire is not a guard.
"""

import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GUARD = ROOT / "cmake" / "hosted_config_guard.cmake"

GOOD = {
    "CONFIG_SLAVE_IDF_TARGET_ESP32C6": "y",
    "CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6": "y",
    "CONFIG_ESP_HOSTED_P4_DEV_BOARD_FUNC_BOARD": "y",
    "CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE": "y",
    "CONFIG_WIFI_RMT_STATIC_RX_BUFFER_NUM": "16",
    "CONFIG_WIFI_RMT_DYNAMIC_RX_BUFFER_NUM": "64",
    "CONFIG_WIFI_RMT_TX_BUFFER_TYPE": "1",
    "CONFIG_WIFI_RMT_AMPDU_TX_ENABLED": "y",
    "CONFIG_WIFI_RMT_TX_BA_WIN": "32",
    "CONFIG_WIFI_RMT_AMPDU_RX_ENABLED": "y",
    "CONFIG_WIFI_RMT_RX_BA_WIN": "32",
    # ESP-IDF emits EVERY symbol; unset ones as an empty string. The guard has
    # to survive that, or it passes on any config at all.
    "CONFIG_ESP_HOSTED_CP_TARGET_ESP32H2": "",
    "CONFIG_ESP_HOSTED_P4_DEV_BOARD_NONE": "",
}


def run_guard(symbols):
    """Run the guard with these CONFIG_* values, exactly as ESP-IDF sets them."""
    if not shutil.which("cmake"):
        raise unittest.SkipTest("cmake not available")
    with tempfile.TemporaryDirectory() as tmp:
        script = Path(tmp) / "run.cmake"
        lines = ['set(%s "%s")' % (k, v) for k, v in symbols.items()]
        # the guard is written for a component context; nothing else is needed
        lines.append('include("%s")' % GUARD.as_posix())
        script.write_text("\n".join(lines) + "\n")
        p = subprocess.run(["cmake", "-P", str(script)],
                           capture_output=True, text=True)
        return p.returncode, p.stdout + p.stderr


class HostedConfigGuardTest(unittest.TestCase):
    def test_a_good_config_passes(self):
        rc, out = run_guard(GOOD)
        self.assertEqual(rc, 0, out)

    def test_every_required_symbol_is_actually_checked(self):
        """One at a time, so a symbol silently dropped from the list is caught
        rather than hidden behind another failure."""
        for key in GOOD:
            if GOOD[key] == "":
                continue                      # the forbidden ones, tested below
            broken = dict(GOOD)
            broken[key] = ""                  # how ESP-IDF renders "not set"
            rc, out = run_guard(broken)
            self.assertNotEqual(rc, 0, "guard did not catch missing %s" % key)
            self.assertIn(key, out)

    def test_a_missing_symbol_entirely_is_caught(self):
        """Not merely empty -- gone, which is what a lost component looks like."""
        broken = dict(GOOD)
        del broken["CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6"]
        rc, out = run_guard(broken)
        self.assertNotEqual(rc, 0, out)
        self.assertIn("CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6", out)

    def test_the_wrong_coprocessor_being_set_is_caught(self):
        """This is the exact observed failure: H2 selected, a chip with no
        WiFi. Having the wrong one set is as fatal as missing the right one."""
        for sym in ("CONFIG_ESP_HOSTED_CP_TARGET_ESP32H2",
                    "CONFIG_ESP_HOSTED_P4_DEV_BOARD_NONE"):
            broken = dict(GOOD)
            broken[sym] = "y"
            rc, out = run_guard(broken)
            self.assertNotEqual(rc, 0, "guard did not catch %s being set" % sym)
            self.assertIn(sym, out)

    def test_the_message_says_how_to_fix_it_and_not_to_hand_edit(self):
        broken = dict(GOOD)
        broken["CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE"] = ""
        _rc, out = run_guard(broken)
        self.assertIn("rm sdkconfig", out)
        self.assertIn("idf.py reconfigure", out)
        self.assertIn("never by hand-editing", out)


class TrackedDefaultsTest(unittest.TestCase):
    """sdkconfig.defaults is the tracked source of truth and self-heals a
    drifted sdkconfig on reconfigure. It must pin everything the guard needs."""

    def test_defaults_pins_every_symbol_the_guard_requires(self):
        guard = GUARD.read_text()
        req = re.search(r"set\(REQUIRED_HOSTED_CONFIG(.*?)\)", guard, re.S).group(1)
        required = re.findall(r"CONFIG_[A-Z0-9_]+", req)
        defaults = (ROOT / "sdkconfig.defaults").read_text()
        for sym in required:
            self.assertRegex(defaults, r"(?m)^%s=" % sym,
                             "%s is required by the guard but not pinned in "
                             "sdkconfig.defaults, so a regeneration would not "
                             "restore it" % sym)

    def test_the_live_sdkconfig_satisfies_the_guard(self):
        """Catches a stale build directory carrying a drifted config."""
        sdk = ROOT / "sdkconfig"
        if not sdk.exists():
            self.skipTest("no sdkconfig yet")
        text = sdk.read_text()
        guard = GUARD.read_text()
        req = re.search(r"set\(REQUIRED_HOSTED_CONFIG(.*?)\)", guard, re.S).group(1)
        for sym in re.findall(r"CONFIG_[A-Z0-9_]+", req):
            self.assertRegex(text, r"(?m)^%s=" % sym, "%s missing" % sym)
        for sym in ("CONFIG_ESP_HOSTED_CP_TARGET_ESP32H2",
                    "CONFIG_ESP_HOSTED_P4_DEV_BOARD_NONE"):
            self.assertNotRegex(text, r"(?m)^%s=y" % sym, "%s is set" % sym)


if __name__ == "__main__":
    unittest.main()
