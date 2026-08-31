"""The trim lives in Kconfig as a STRING, parsed with strtof.

A typo ("0,20", "O.20", an empty value) parses to 0.0 and silently disables the
trim -- the boat would run untrimmed and nothing would say so except one log
line. These pin the value instead.

Two knobs exist and only one is live:

    ESC_TRIM_C        split as a FRACTION of throttle -- the one in use
    ESC_TRIM_DEFAULT  the old FLAT value, kept only for comparison

The flat one is what a T40 sweep produced, and it is correct at T40 and wrong
everywhere else (measured +8.4 deg/s at T10). ESC_TRIM_C wins in main.c.
"""

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _kconfig_default(name):
    src = (ROOT / 'main' / 'Kconfig.projbuild').read_text()
    m = re.search(r'config %s\b.*?\n\s*default\s+"([^"]*)"' % name, src, re.S)
    return m.group(1) if m else None


def _sdkconfig_value(name):
    src = (ROOT / 'sdkconfig').read_text()
    m = re.search(r'^CONFIG_%s="([^"]*)"' % name, src, re.M)
    return m.group(1) if m else None


class EscTrimConfigTest(unittest.TestCase):
    def test_both_knobs_parse_as_real_numbers(self):
        for name in ('ESC_TRIM_C', 'ESC_TRIM_DEFAULT'):
            raw = _kconfig_default(name)
            self.assertIsNotNone(raw, '%s missing from Kconfig' % name)
            self.assertEqual(raw, raw.strip())
            float(raw)                   # strtof-equivalent; raises on a typo

    def test_the_compiled_values_match_the_kconfig_defaults(self):
        """Catches sdkconfig drifting away from the tracked source of truth."""
        for name in ('ESC_TRIM_C', 'ESC_TRIM_DEFAULT'):
            self.assertEqual(float(_sdkconfig_value(name)),
                             float(_kconfig_default(name)), name)

    # Where the boat actually runs straight, measured by regressing RAW yaw
    # against c over a RANGE of c -- five fits across the two adaptive sessions
    # (dataout/autotrim, dataout/autotrim2), r = +0.53..+0.83.
    #
    # These supersede the earlier 0.19-0.23 figures, which came from single
    # fixed-trim runs whose yaw was baseline-corrected. The baseline turned out
    # to be uncorrelated with the run (corr -0.16 and +0.05), so that
    # correction was adding noise, not removing bias.
    # Found by bracketing the SIGN FLIP, no line fitted: bin all 63 adaptive
    # runs by the c they ran with and find the pair of bins where median yaw
    # changes sign. Twelve independent slices -- five time windows, two
    # sessions, three throttles -- give:
    #
    #     0.145 0.155 0.160 0.162 0.165 0.167 0.168 0.175 0.182 0.195 0.203 0.212
    #
    # Median 0.171. The two cleanest windows agree closely: 0.5-2.0 s (after
    # spin-up, before the hull reaches the wall) gives 0.165, and 3.0-10 s
    # gives 0.168.
    #
    # The band is wide because that IS the measurement limit -- earlier
    # attempts to quote 0.16 or 0.20 to three figures were reading noise.
    # Fitting a line instead of bracketing the flip made it worse: the fits
    # get dragged by low-n bins and fail a split-half check (T30: 0.193 vs
    # 0.553; T40: 0.168 vs 0.099).
    C_SUPPORTED_LO, C_SUPPORTED_HI = 0.145, 0.212

    def test_c_is_inside_the_band_the_data_supports(self):
        c = float(_sdkconfig_value('ESC_TRIM_C'))
        self.assertGreater(c, 0.0, 'sign flipped -- this pushes the LEFT motor')
        self.assertGreaterEqual(c, self.C_SUPPORTED_LO)
        self.assertLessEqual(c, self.C_SUPPORTED_HI)

    def test_the_seed_is_not_load_bearing(self):
        """Closed-loop replay against the measured plant and the boat's own
        noise trace settles in the same place from 0.20 or from 0.16, so the
        seed only shapes the first ~20 s. It is pinned to a sane band, not to a
        precise value the data cannot justify -- and nothing else may come to
        depend on its exact figure."""
        src = (ROOT / 'main' / 'motor_control.c').read_text()
        self.assertIn('.c_init = parse_cfg_float(CONFIG_ESC_TRIM_C', src)
        # the learner must be free to leave the seed in both directions
        self.assertIn('.c_min = 0.10f, .c_max = 0.35f', src)
        c = float(_sdkconfig_value('ESC_TRIM_C'))
        self.assertGreater(c, 0.10, 'seeded on the clamp -- nowhere to go down')
        self.assertLess(c, 0.35, 'seeded on the clamp -- nowhere to go up')

    def test_c_is_not_the_old_baseline_corrected_figure(self):
        """0.20 came from subtracting a baseline that does not predict the run.
        Seeding there put the learner above every measured value, so it only
        ever walked downhill -- which is exactly what both sessions show."""
        c = float(_sdkconfig_value('ESC_TRIM_C'))
        self.assertLess(c, 0.19, 'back to the superseded baseline-corrected c')

    def test_c_produces_a_split_inside_what_was_measured(self):
        c = float(_sdkconfig_value('ESC_TRIM_C'))
        for throttle in (0.10, 0.20, 0.30, 0.40, 0.50):
            self.assertGreaterEqual(c * throttle, self.C_SUPPORTED_LO * throttle)
            self.assertLessEqual(c * throttle, self.C_SUPPORTED_HI * throttle)

    def test_the_flat_fallback_is_still_the_value_we_measured(self):
        """7.6% toward the RIGHT motor = +0.152 in esc_trim_mix units
        (left -= t/2, right += t/2). POSITIVE. Backwards would drive the boat
        the wrong way at twice the strength."""
        v = float(_sdkconfig_value('ESC_TRIM_DEFAULT'))
        self.assertGreater(v, 0.0, 'sign flipped -- this would push LEFT harder')
        self.assertAlmostEqual(v, 0.152, places=4)
        self.assertAlmostEqual(v * 50.0, 7.6, places=2)   # percent to the right

    def test_the_flat_value_is_kept_only_as_a_superseded_reference(self):
        """0.152 is c=0.19 at T40 -- the old baseline-corrected answer. It is
        deliberately NOT equal to ESC_TRIM_C any more, and it must stay
        unreachable while ESC_TRIM_C is set."""
        c = float(_sdkconfig_value('ESC_TRIM_C'))
        flat = float(_sdkconfig_value('ESC_TRIM_DEFAULT'))
        self.assertGreater(flat, 0.0)
        self.assertGreater(flat / (2 * 0.40), c,
                           'the flat value no longer reads as the older, '
                           'higher estimate -- one of the two is stale')
        src = (ROOT / 'main' / 'main.c').read_text()
        self.assertLess(src.index('CONFIG_ESC_TRIM_C'),
                        src.index('CONFIG_ESC_TRIM_DEFAULT'))

    def test_the_learner_is_actually_switched_on_in_the_build(self):
        """sdkconfig is STICKY: a saved "is not set" survives a Kconfig default
        change and `idf.py reconfigure` will not touch it. That is how a
        feature ships silently disabled -- the source says default y, the
        binary says off, and nothing complains. This compares the two."""
        src = (ROOT / 'main' / 'Kconfig.projbuild').read_text()
        m = re.search(r'config STABILITY_TRIMLEARN_ENABLE\b.*?\n\s*default\s+(\w+)',
                      src, re.S)
        self.assertIsNotNone(m, 'STABILITY_TRIMLEARN_ENABLE missing from Kconfig')
        self.assertEqual(m.group(1), 'y', 'the learner is disabled at the source')

        sdk = (ROOT / 'sdkconfig').read_text()
        self.assertIn('CONFIG_STABILITY_TRIMLEARN_ENABLE=y', sdk,
                      'Kconfig says the learner is on but the build has it off '
                      '-- delete the stale line from sdkconfig and reconfigure')
        # its settings only exist while it is enabled, so their presence is a
        # second, independent confirmation
        for name, want in (('DEADBAND_DPS', 0.5), ('STEP_PER_S', 0.005),
                           ('MIN_THROTTLE', 0.15)):
            m = re.search(r'^CONFIG_STABILITY_TRIMLEARN_%s="([^"]*)"' % name,
                          sdk, re.M)
            self.assertIsNotNone(m, name)
            self.assertAlmostEqual(float(m.group(1)), want, places=6, msg=name)

    def test_the_learner_can_only_move_slowly_and_within_bounds(self):
        """It drives the motors unattended, so the bounds are the safety
        argument: c held to [0.10, 0.35] at most 0.005/s, and a bound touched
        is a latched fault rather than a rail to sit on."""
        src = (ROOT / 'main' / 'motor_control.c').read_text()
        self.assertIn('.c_min = 0.10f, .c_max = 0.35f', src)
        step = float(re.search(
            r'^CONFIG_STABILITY_TRIMLEARN_STEP_PER_S="([^"]*)"',
            (ROOT / 'sdkconfig').read_text(), re.M).group(1))
        # slowest sensible correction: a full 0.10 swing must take >10 s, so a
        # transient can never yank the boat before the pilot can react
        self.assertLess(step, 0.01, 'the learner can move faster than 0.01/s')
        self.assertGreater(step, 0.0)
        learn = (ROOT / 'main' / 'trim_learn.c').read_text()
        self.assertIn('s->faulted = true;', learn)

    def test_main_prefers_c_over_the_flat_value_and_both_over_nvs(self):
        src = (ROOT / 'main' / 'main.c').read_text()
        self.assertIn('IGNORING the %u-point table in NVS', src)
        self.assertIn('strtof(CONFIG_ESC_TRIM_C', src)
        self.assertIn('strtof(CONFIG_ESC_TRIM_DEFAULT', src)
        # c is looked at first; the flat value only in its else-branch
        self.assertLess(src.index('CONFIG_ESC_TRIM_C'),
                        src.index('CONFIG_ESC_TRIM_DEFAULT'))
        self.assertIn('esc_trim_build_proportional', src)

    def test_the_learner_starts_from_the_same_number(self):
        """One number, one place. A learner seeded anywhere else would step off
        the bench answer the moment it switched on."""
        src = (ROOT / 'main' / 'motor_control.c').read_text()
        self.assertIn('.c_init = parse_cfg_float(CONFIG_ESC_TRIM_C', src)
        self.assertIsNone(_kconfig_default('STABILITY_TRIMLEARN_C_INIT'),
                          'a second source of truth for c came back')


if __name__ == '__main__':
    unittest.main()
