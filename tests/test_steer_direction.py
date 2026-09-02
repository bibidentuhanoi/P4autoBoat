"""The rudder truth table, measured on the physical boat 2026-08-31.

Sighting from BEHIND the hull toward the bow:

    1805 us = full LEFT     1516 us = centre     1195 us = full RIGHT

and the canonical operator/internal convention everywhere above the driver is

    -100 / -1 = LEFT        0 = centre           +100 / +1 = RIGHT

These are not preferences. The servo genuinely runs the opposite way to the
raw MIN/NEUTRAL/MAX interpolation, so `steer_to_us()` alone would send a
"+1 = right" command hard over to the LEFT. CONFIG_STEER_REVERSE is what
reconciles them, and it must stay on for this board.

Why this file exists: the direction was wrong for months and nothing caught it,
because it was wrong *consistently*. steer_driver.c called 1805 us "full
right" (inferred from MAX_US being the larger number, never checked against
the hull), the Kconfig labels repeated it, and dashboard.html then negated its
slider to compensate -- which made the DASHBOARD behave correctly on the water
and hid the root error. tools/espnow_drive.py, which did the honest thing and
mapped right to +1, was the one that steered backwards.

So the end-to-end path is what gets pinned here, for BOTH UIs, all the way to
the microsecond. Checking any single layer in isolation would have passed
happily on the old code.
"""

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# The physical measurement. Everything else in this file is derived from it.
FULL_LEFT_US = 1805
CENTRE_US = 1516
FULL_RIGHT_US = 1195


def _sdkconfig_int(name):
    src = (ROOT / 'sdkconfig').read_text()
    m = re.search(r'^CONFIG_%s=(\d+)$' % name, src, re.M)
    return int(m.group(1)) if m else None


def _sdkconfig_bool(name):
    """True only for '=y'. A bool that is off is written as a comment line,
    so 'absent' and 'off' are the same thing and both return False."""
    src = (ROOT / 'sdkconfig').read_text()
    return re.search(r'^CONFIG_%s=y$' % name, src, re.M) is not None


def _kconfig_int_default(name):
    src = (ROOT / 'main' / 'Kconfig.projbuild').read_text()
    m = re.search(r'config %s\b.*?\n\s*default\s+(\d+)' % name, src, re.S)
    return int(m.group(1)) if m else None


def _kconfig_bool_default(name):
    src = (ROOT / 'main' / 'Kconfig.projbuild').read_text()
    m = re.search(r'config %s\b.*?\n\s*bool\s+"[^"]*"\s*\n\s*default\s+(\w+)' % name,
                  src, re.S)
    return m.group(1) if m else None


def steer_to_us(s, min_us, neutral_us, max_us):
    """Mirror of steer_driver.c's steer_to_us(): the RAW interpolation, which
    knows nothing about left or right. Pinned against the C source by
    test_the_mirror_still_matches_the_driver below."""
    s = max(-1.0, min(1.0, s))
    if s >= 0.0:
        return int(neutral_us + s * (max_us - neutral_us) + 0.5)
    return int(neutral_us + s * (neutral_us - min_us) + 0.5)


def commanded_us(canonical, min_us, neutral_us, max_us, reverse):
    """A canonical steer value (-1 left .. +1 right) -> the pulse the servo
    actually receives. Mirrors steer_driver_set(): the reverse flag is applied
    by the CALLER, outside steer_to_us()."""
    s = -canonical if reverse else canonical
    return steer_to_us(s, min_us, neutral_us, max_us)


class SteerDirectionTest(unittest.TestCase):

    def setUp(self):
        self.min_us = _sdkconfig_int('STEER_PULSE_MIN_US')
        self.neutral_us = _sdkconfig_int('STEER_PULSE_NEUTRAL_US')
        self.max_us = _sdkconfig_int('STEER_PULSE_MAX_US')
        self.reverse = _sdkconfig_bool('STEER_REVERSE')
        for n, v in (('MIN', self.min_us), ('NEUTRAL', self.neutral_us),
                     ('MAX', self.max_us)):
            self.assertIsNotNone(v, 'STEER_PULSE_%s_US missing from sdkconfig' % n)

    # ---- the configuration itself -------------------------------------

    def test_a_clean_generated_sdkconfig_has_steer_reverse_on(self):
        """The build that gets flashed must have it. A Kconfig 'default y' is
        NOT enough on its own: an sdkconfig that already contains
        '# CONFIG_STEER_REVERSE is not set' wins over a newly-changed default,
        which is exactly how the trim learner once shipped switched off."""
        self.assertTrue(self.reverse,
                        "CONFIG_STEER_REVERSE is not =y in the generated "
                        "sdkconfig -- every steering command points the wrong "
                        "way. Regenerate: rm sdkconfig && idf.py reconfigure")
        self.assertEqual(_kconfig_bool_default('STEER_REVERSE'), 'y',
                         "Kconfig default must be y too, or the next clean "
                         "regeneration silently turns it back off")

    def test_the_pulse_endpoints_are_the_measured_ones(self):
        self.assertEqual(self.min_us, FULL_RIGHT_US)
        self.assertEqual(self.neutral_us, CENTRE_US)
        self.assertEqual(self.max_us, FULL_LEFT_US)
        for name, want in (('STEER_PULSE_MIN_US', FULL_RIGHT_US),
                           ('STEER_PULSE_NEUTRAL_US', CENTRE_US),
                           ('STEER_PULSE_MAX_US', FULL_LEFT_US)):
            self.assertEqual(_kconfig_int_default(name), want,
                             '%s drifted from the tracked Kconfig default' % name)

    # ---- the truth table, canonical value -> microseconds ---------------

    def test_the_canonical_convention_reaches_the_right_pulse(self):
        """-1 = LEFT = 1805, 0 = centre = 1516, +1 = RIGHT = 1195."""
        table = ((-1.0, FULL_LEFT_US, 'full LEFT'),
                 (0.0, CENTRE_US, 'centre'),
                 (+1.0, FULL_RIGHT_US, 'full RIGHT'))
        for canonical, want_us, what in table:
            got = commanded_us(canonical, self.min_us, self.neutral_us,
                               self.max_us, self.reverse)
            self.assertEqual(got, want_us,
                             'steer %+.1f should be %s (%d us), got %d us'
                             % (canonical, what, want_us, got))

    def test_the_two_ends_are_not_the_same_way_round(self):
        """Guards the degenerate pass where MIN == MAX, or where a sign error
        maps both ends to the same place."""
        left = commanded_us(-1.0, self.min_us, self.neutral_us, self.max_us,
                            self.reverse)
        right = commanded_us(+1.0, self.min_us, self.neutral_us, self.max_us,
                             self.reverse)
        self.assertNotEqual(left, right)
        self.assertGreater(left, right,
                           'left must be the LONGER pulse on this board')

    # ---- both UIs, slider -> canonical value -> microseconds ------------

    def _dashboard_expression(self):
        src = (ROOT / 'main' / 'dashboard.html').read_text()
        m = re.search(r"const v = (.*?parseInt\(\$\('rudder'\)\.value\).*?);", src)
        self.assertIsNotNone(m, "dashboard.html's rudder send expression moved")
        return m.group(1).strip()

    def test_dashboard_sends_the_slider_straight_through(self):
        """The compensating negation is gone. It only ever existed to cancel
        the driver's wrong idea of which way 1805 us pointed; with the root
        fixed, negating here would steer the boat backwards."""
        expr = self._dashboard_expression()
        self.assertEqual(expr, "parseInt($('rudder').value) / 100")
        self.assertNotIn('-parseInt', expr)

    def test_python_tool_sends_the_slider_straight_through(self):
        """The slider now feeds the control heartbeat rather than posting a
        one-off state, but the MAPPING is what this pins: v / 100, no sign
        flip."""
        src = (ROOT / 'tools' / 'espnow_drive.py').read_text()
        self.assertIn('ctrl.rudder = v / 100;', src)
        self.assertNotIn('ctrl.rudder = -v / 100', src)
        self.assertNotIn("rudder: -v / 100", src)

    def test_both_uis_produce_the_same_truth_table(self):
        """The whole point of the change: one slider convention, one wire
        value, one physical direction -- for both UIs."""
        for slider, want_us, what in ((-100, FULL_LEFT_US, 'full LEFT'),
                                      (0, CENTRE_US, 'centre'),
                                      (+100, FULL_RIGHT_US, 'full RIGHT')):
            # Both UIs now do the identical thing: value / 100, no sign flip.
            canonical = slider / 100
            got = commanded_us(canonical, self.min_us, self.neutral_us,
                               self.max_us, self.reverse)
            self.assertEqual(got, want_us,
                             'slider %+d -> %+.1f should be %s (%d us), got %d'
                             % (slider, canonical, what, want_us, got))

    def test_the_two_uis_keep_their_own_starting_positions(self):
        """They agree on DIRECTION, not on where the slider starts. The
        dashboard opens at -100 because that is the 1805 us position the servo
        physically powers up in; the field tool opens centred."""
        dash = (ROOT / 'main' / 'dashboard.html').read_text()
        m = re.search(r'<input type="range" id="rudder"[^>]*>', dash)
        self.assertIsNotNone(m)
        self.assertIn('value="-100"', m.group(0))

        tool = (ROOT / 'tools' / 'espnow_drive.py').read_text()
        m = re.search(r'<input type="range" id="rudder"[^>]*>', tool)
        self.assertIsNotNone(m)
        self.assertIn('value="0"', m.group(0))

    # ---- the power-up position -----------------------------------------

    def test_the_servo_still_powers_up_where_it_physically_rests(self):
        """STEER_HOME went +1 -> -1 in the same change that turned reverse on,
        and the two cancel: the boot pulse is still 1805 us. Only the NAME
        changed, from "full right" to the true "full left"."""
        src = (ROOT / 'main' / 'drivers' / 'steer_driver.c').read_text()
        m = re.search(r'#define STEER_HOME\s+\(?(-?[\d.]+)f\)?', src)
        self.assertIsNotNone(m, 'STEER_HOME missing from steer_driver.c')
        home = float(m.group(1))
        self.assertEqual(home, -1.0,
                         'STEER_HOME must be -1 (LEFT) to keep booting at '
                         '1805 us now that reverse is on')
        boot_us = commanded_us(home, self.min_us, self.neutral_us, self.max_us,
                               self.reverse)
        self.assertEqual(boot_us, FULL_LEFT_US,
                         'the boot position moved off the servo\'s physical '
                         'resting stop')

    # ---- the mirror, and the things that must NOT have changed ----------

    def test_the_mirror_still_matches_the_driver(self):
        """steer_to_us() above is a re-implementation; pin the real one so it
        cannot drift. Also pins that the reverse is applied by the callers and
        never inside steer_to_us()."""
        src = (ROOT / 'main' / 'drivers' / 'steer_driver.c').read_text()
        body = src[src.index('static uint32_t steer_to_us(float s)'):]
        body = body[:body.index('\n}')]
        self.assertIn('return (uint32_t)(neutral + s * (float)(max_us - neutral) + 0.5f);', body)
        self.assertIn('return (uint32_t)(neutral + s * (float)(neutral - min_us) + 0.5f);', body)
        self.assertNotIn('STEER_REV', body,
                         'the reverse leaked into steer_to_us(); the mirror '
                         'and every caller assume it is applied outside')
        # every caller that maps the canonical value to a pulse applies it:
        # init, set, reassert, and the pulse getter telemetry reads.
        self.assertEqual(src.count('STEER_REV ? -s_steer : s_steer'), 4,
                         'init / set / reassert / get_pulse_us must all apply '
                         'the reverse')
        for fn in ('esp_err_t steer_driver_set(float steer)',
                   'void steer_driver_reassert(void)',
                   'uint32_t steer_driver_get_pulse_us(void)'):
            i = src.index(fn)
            self.assertIn('STEER_REV ? -s_steer : s_steer', src[i:i + 400], fn)

    def test_raw_pulse_control_is_untouched_by_any_of_this(self):
        """The calibration escape hatch writes the comparator directly. If the
        reverse ever reached it, probing for a mechanical stop would move the
        servo the opposite way and the operator would map the wrong end."""
        src = (ROOT / 'main' / 'drivers' / 'steer_driver.c').read_text()
        body = src[src.index('esp_err_t steer_driver_set_raw_us(uint32_t pulse_us)'):]
        body = body[:body.index('\n}')]
        self.assertNotIn('STEER_REV', body)
        self.assertNotIn('steer_to_us', body)
        self.assertIn('mcpwm_comparator_set_compare_value(s_cmp, pulse_us)', body)

    def test_no_stale_claim_that_1805_is_right_survives(self):
        """The original error, in the exact words it was written in. It spread
        from one comment into the Kconfig labels and then into a UI negation,
        so each of those three is checked for the specific wrong wording.

        Deliberately literal rather than a keyword sweep: a line may legitimately
        mention both ends at once ("1805 = LEFT ... 1195 = RIGHT"), and a
        heuristic that just looks for "right" near "1805" flags that as a
        failure. It did, on the first run of this very test."""
        kconfig = (ROOT / 'main' / 'Kconfig.projbuild').read_text()
        self.assertNotIn('int "Steering Pulse Max (us) — full right"', kconfig)
        self.assertNotIn('int "Steering Pulse Min (us) — full left"', kconfig)
        self.assertIn('int "Steering Pulse Max (us) — full LEFT"', kconfig)
        self.assertIn('int "Steering Pulse Min (us) — full RIGHT"', kconfig)

        driver = (ROOT / 'main' / 'drivers' / 'steer_driver.c').read_text()
        self.assertNotIn('home=%uus full-right', driver)
        self.assertIn('home=%uus full-left', driver)
        self.assertNotIn('Home to STEER_HOME (full-right)', driver)

        dash = (ROOT / 'main' / 'dashboard.html').read_text()
        self.assertNotIn("firmware's steer value is +1.0=full-right", dash)

        tool = (ROOT / 'tools' / 'espnow_drive.py').read_text()
        self.assertNotIn('firmware STEER_HOME = +1.0 = right', tool)


if __name__ == '__main__':
    unittest.main()
