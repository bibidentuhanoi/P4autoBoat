"""Bench-run signal strength and the UI-observed yaw summary (Python tool only).

Two things are being tested, and one thing is deliberately NOT claimed.

WHAT THE MOTORS REALLY GET. The boat applies the learned trim on top of the
commanded split, for every run kind (bench_run.c: bench_commands() then
esc_trim_apply_pair(), trim = 2*c*base). So the nominal +/-delta is not what
the jets see:

    LEFT  differential = 2 * (c*base - delta)    zero at delta == c*base,
                                                 REVERSED below it
    RIGHT differential = 2 * (c*base + delta)    always amplified

That is intended -- these runs measure the boat in its real operating
condition, autotrim included -- but at the old 4% default and c=0.17 a T20
LEFT run netted -1.2% against RIGHT's +14.8%, and at T40 the LEFT run turned
the boat the SAME WAY as RIGHT. Raising the split fixes the DIRECTION, not the
asymmetry: RIGHT remains the stronger of the two at every setting, and
test_raising_the_split_does_not_make_the_two_directions_symmetric pins that so
no one later describes this as a symmetry fix.

THE YAW SUMMARY is derived from whatever telemetry frames this tool caught
while the boat reported itself driving (~20 Hz over the air, against the
boat's own 100 Hz SD CSV). It is a lossy convenience view, so it has to be
honest about being patchy rather than print a confident number from four
frames.

Firmware, protobuf, BASE behaviour, the learner/P algorithms and the
historical T<pct>_<B|L|R>_NN.CSV identifiers are all untouched; the last
class here guards that.
"""

import importlib.util
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'espnow_drive.py'


def _load_tool():
    spec = importlib.util.spec_from_file_location('espnow_drive_bench_test', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


T = _load_tool()


class EffectiveSplitTest(unittest.TestCase):
    """bench_effective_commands() must match the firmware exactly."""

    def test_base_is_symmetric_before_trim_and_offset_after(self):
        e = T.bench_effective_commands('both', 0.20, 0.12, 0.17)
        self.assertAlmostEqual(e['nominal_left'], 0.20)
        self.assertAlmostEqual(e['nominal_right'], 0.20)
        self.assertAlmostEqual(e['nominal_differential'], 0.0)
        # trim = 2*c*base = 0.068, applied as -/+ half
        self.assertAlmostEqual(e['trim'], 0.068, places=6)
        self.assertAlmostEqual(e['left'], 0.166, places=6)
        self.assertAlmostEqual(e['right'], 0.234, places=6)
        self.assertAlmostEqual(e['differential'], 0.068, places=6)

    def test_delta_is_ignored_for_base(self):
        """bench_commands() only branches on LEFT/RIGHT."""
        a = T.bench_effective_commands('both', 0.20, 0.04, 0.17)
        b = T.bench_effective_commands('both', 0.20, 0.30, 0.17)
        self.assertEqual(a['left'], b['left'])
        self.assertEqual(a['right'], b['right'])

    def test_the_old_default_cancelled_the_left_run(self):
        """T20, split 4%, c=0.17 -- the regression this change addresses."""
        e = T.bench_effective_commands('left', 0.20, 0.04, 0.17)
        self.assertAlmostEqual(e['nominal_differential'], -0.08, places=6)
        self.assertAlmostEqual(e['differential'], -0.012, places=6)
        # 85% of the commanded split eaten by the trim
        self.assertLess(abs(e['differential']), 0.2 * abs(e['nominal_differential']))

    def test_at_t40_the_old_default_actually_reversed_the_left_run(self):
        """Not merely weak -- the wrong way round. trim 0.136 > delta 0.08."""
        e = T.bench_effective_commands('left', 0.40, 0.04, 0.17)
        self.assertLess(e['nominal_differential'], 0.0)   # asked to go one way
        self.assertGreater(e['differential'], 0.0)        # went the other

    def test_the_new_default_keeps_the_left_run_pointing_the_right_way(self):
        e = T.bench_effective_commands('left', 0.20, 0.12, 0.17)
        self.assertLess(e['differential'], 0.0)
        self.assertAlmostEqual(e['differential'], -0.172, places=6)
        # and still correct at the throttle where 4% failed outright
        e40 = T.bench_effective_commands('left', 0.40, 0.12, 0.17)
        self.assertLess(e40['differential'], 0.0)

    def test_right_is_always_amplified_never_cancelled(self):
        for base in (0.10, 0.20, 0.40, 0.60):
            for delta in (0.01, 0.04, 0.12, 0.30):
                e = T.bench_effective_commands('right', base, delta, 0.17)
                self.assertGreater(e['differential'], 0.0,
                                   'RIGHT should never reverse (base=%s delta=%s)'
                                   % (base, delta))

    def test_the_closed_form_holds(self):
        """differential = 2*(c*base -+ delta). Checked against the mirror so a
        clamp bug cannot hide behind agreeing arithmetic."""
        for base, delta, c in ((0.20, 0.12, 0.17), (0.15, 0.05, 0.22),
                               (0.30, 0.10, 0.10)):
            l = T.bench_effective_commands('left', base, delta, c)
            r = T.bench_effective_commands('right', base, delta, c)
            self.assertAlmostEqual(l['differential'], 2 * (c * base - delta), places=6)
            self.assertAlmostEqual(r['differential'], 2 * (c * base + delta), places=6)

    def test_raising_the_split_does_not_make_the_two_directions_symmetric(self):
        """The point of the change is SIGNAL STRENGTH, not symmetry. RIGHT is
        stronger at every setting, and that gap is the asymmetry being
        measured -- do not let anyone describe this as a symmetry fix."""
        for delta in (0.05, 0.12, 0.20, 0.30):
            l = T.bench_effective_commands('left', 0.20, delta, 0.17)
            r = T.bench_effective_commands('right', 0.20, delta, 0.17)
            self.assertGreater(abs(r['differential']), abs(l['differential']),
                               'RIGHT must stay the stronger direction')

    def test_commands_are_clamped_the_way_the_jets_require(self):
        """Unidirectional jets: bench_commands() clamps at 0 BEFORE the trim,
        and esc_trim_apply_pair() clamps again after."""
        e = T.bench_effective_commands('left', 0.05, 0.30, 0.17)
        self.assertGreaterEqual(e['nominal_right'], 0.0)
        self.assertGreaterEqual(e['right'], 0.0)
        self.assertLessEqual(e['left'], 1.0)
        e = T.bench_effective_commands('right', 0.95, 0.30, 0.17)
        self.assertLessEqual(e['right'], 1.0)

    def test_zero_throttle_stays_stopped(self):
        """esc_trim_apply_pair() leaves a stopped pair alone -- otherwise a
        trim would spin one jet while the caller believes the motors are off."""
        e = T.bench_effective_commands('both', 0.0, 0.12, 0.17)
        self.assertEqual(e['left'], 0.0)
        self.assertEqual(e['right'], 0.0)

    def test_a_nonfinite_c_is_treated_as_no_trim(self):
        e = T.bench_effective_commands('left', 0.20, 0.12, float('nan'))
        self.assertAlmostEqual(e['differential'], -0.24, places=6)


class CancellationWarningTest(unittest.TestCase):

    def test_the_exact_cancellation_point_is_c_times_throttle(self):
        self.assertAlmostEqual(T.bench_split_cancel_delta(0.20, 0.17), 0.034)
        e = T.bench_effective_commands('left', 0.20, 0.034, 0.17)
        self.assertAlmostEqual(e['differential'], 0.0, places=6)

    def test_below_cancellation_it_warns_about_reversal(self):
        w = T.bench_split_warning(0.20, 0.02, 0.17)
        self.assertIsNotNone(w)
        self.assertIn('SAME way', w)

    def test_just_above_cancellation_it_warns_about_weakness(self):
        w = T.bench_split_warning(0.20, 0.04, 0.17)
        self.assertIsNotNone(w)
        self.assertIn('close to', w)
        self.assertNotIn('SAME way', w)

    def test_the_shipped_default_does_not_warn(self):
        """T20 / 12% / c=0.17 -- the values the UI now starts at."""
        self.assertIsNone(T.bench_split_warning(0.20, 0.12, 0.17))

    def test_it_warns_at_t40_with_the_old_default(self):
        self.assertIsNotNone(T.bench_split_warning(0.40, 0.04, 0.17))

    def test_no_trim_means_nothing_to_warn_about(self):
        self.assertIsNone(T.bench_split_warning(0.20, 0.01, 0.0))

    def test_the_warning_boundary_matches_the_arithmetic(self):
        """Warn exactly when the LEFT run is reversed or under 2x the trim."""
        c, base = 0.17, 0.20
        cancel = c * base
        for delta, expect_warn in ((cancel * 0.5, True), (cancel * 0.99, True),
                                   (cancel * 1.5, True), (cancel * 1.99, True),
                                   (cancel * 2.01, False), (cancel * 4.0, False)):
            got = T.bench_split_warning(base, delta, c) is not None
            self.assertEqual(got, expect_warn, 'delta=%.4f' % delta)


class HeadingWrapTest(unittest.TestCase):

    def test_the_short_way_round(self):
        self.assertAlmostEqual(T.wrap_deg(10.0), 10.0)
        self.assertAlmostEqual(T.wrap_deg(-10.0), -10.0)
        self.assertAlmostEqual(T.wrap_deg(350.0), -10.0)
        self.assertAlmostEqual(T.wrap_deg(-350.0), 10.0)
        self.assertAlmostEqual(T.wrap_deg(0.0), 0.0)

    def test_the_boundaries(self):
        """Range is [-180, +180): a half turn resolves to -180, which is the
        conventional choice for a genuinely ambiguous sign. It cannot arise
        from consecutive ~20 Hz frames anyway."""
        self.assertAlmostEqual(T.wrap_deg(180.0), -180.0)
        self.assertAlmostEqual(T.wrap_deg(-180.0), -180.0)
        self.assertAlmostEqual(T.wrap_deg(360.0), 0.0)
        self.assertAlmostEqual(T.wrap_deg(720.0), 0.0)
        # everything strictly inside the range keeps its own sign
        for d in (-179.9, -90.0, -0.1, 0.1, 90.0, 179.9):
            self.assertAlmostEqual(T.wrap_deg(d), d, places=6)

    def test_crossing_north_is_not_a_359_degree_jump(self):
        self.assertAlmostEqual(T.wrap_deg(5.0 - 355.0), 10.0)
        self.assertAlmostEqual(T.wrap_deg(355.0 - 5.0), -10.0)


class YawSummaryTest(unittest.TestCase):

    @staticmethod
    def _run(rates, hz=20.0, headings=None, t0=100.0):
        dt = 1.0 / hz
        return [(t0 + i * dt, r,
                 headings[i] if headings is not None else None)
                for i, r in enumerate(rates)]

    def test_a_steady_turn(self):
        s = T.summarize_yaw(self._run([-8.0] * 60))
        self.assertTrue(s['have'])
        self.assertEqual(s['n'], 60)
        self.assertAlmostEqual(s['mean_yaw_dps'], -8.0, places=6)
        self.assertAlmostEqual(s['peak_yaw_dps'], -8.0, places=6)
        # 59 intervals at 0.05 s = 2.95 s of integration
        self.assertAlmostEqual(s['yaw_angle_deg'], -8.0 * 2.95, places=4)
        self.assertFalse(s['incomplete'])
        self.assertFalse(s['stale'])

    def test_the_sign_is_preserved_end_to_end(self):
        neg = T.summarize_yaw(self._run([-5.0] * 60))
        pos = T.summarize_yaw(self._run([+5.0] * 60))
        self.assertLess(neg['mean_yaw_dps'], 0)
        self.assertLess(neg['yaw_angle_deg'], 0)
        self.assertLess(neg['peak_yaw_dps'], 0)
        self.assertGreater(pos['mean_yaw_dps'], 0)
        self.assertGreater(pos['yaw_angle_deg'], 0)
        self.assertGreater(pos['peak_yaw_dps'], 0)

    def test_peak_is_the_signed_excursion_not_the_magnitude(self):
        """A run that mostly turns one way but spikes hard the other must
        report the spike WITH its sign -- max(abs()) would erase exactly the
        thing this readout exists to establish."""
        s = T.summarize_yaw(self._run([-2.0] * 30 + [-30.0] + [-2.0] * 29))
        self.assertAlmostEqual(s['peak_yaw_dps'], -30.0)
        s = T.summarize_yaw(self._run([2.0] * 30 + [25.0] + [2.0] * 29))
        self.assertAlmostEqual(s['peak_yaw_dps'], 25.0)

    def test_integration_is_trapezoidal_over_real_timestamps(self):
        """A ramp integrates to the trapezoid area, and uneven sampling must
        not bias it toward whichever end was sampled denser."""
        s = T.summarize_yaw([(0.0, 0.0, None), (1.0, 10.0, None)])
        self.assertAlmostEqual(s['yaw_angle_deg'], 5.0, places=6)
        # same ramp, one extra midpoint sample -- identical area
        s2 = T.summarize_yaw([(0.0, 0.0, None), (0.5, 5.0, None), (1.0, 10.0, None)])
        self.assertAlmostEqual(s2['yaw_angle_deg'], 5.0, places=6)

    def test_a_turn_and_a_turn_back_integrates_toward_zero(self):
        s = T.summarize_yaw(self._run([10.0] * 30 + [-10.0] * 30))
        self.assertAlmostEqual(s['mean_yaw_dps'], 0.0, places=6)
        self.assertLess(abs(s['yaw_angle_deg']), 1.0)

    def test_heading_accumulates_consecutive_wrapped_deltas(self):
        """Through north, and past 180 deg total -- first-vs-last would wrap
        the wrong way and report a small number for a big turn."""
        headings = [(350.0 + 4.0 * i) % 360.0 for i in range(60)]
        s = T.summarize_yaw(self._run([4.0 * 20] * 60, headings=headings))
        self.assertTrue(s['heading_ok'])
        self.assertAlmostEqual(s['heading_change_deg'], 4.0 * 59, places=4)

    def test_heading_is_absent_when_the_transport_does_not_carry_it(self):
        s = T.summarize_yaw(self._run([-8.0] * 60))
        self.assertFalse(s['heading_ok'])
        self.assertEqual(s['heading_change_deg'], 0.0)

    def test_no_samples_at_all(self):
        s = T.summarize_yaw([])
        self.assertFalse(s['have'])
        self.assertEqual(s['n'], 0)
        self.assertTrue(s['incomplete'])

    def test_nonfinite_rates_are_dropped_not_propagated(self):
        s = T.summarize_yaw(self._run([-8.0] * 30 + [float('nan')] + [-8.0] * 29))
        self.assertEqual(s['n'], 59)
        self.assertTrue(math.isfinite(s['mean_yaw_dps']))
        self.assertTrue(math.isfinite(s['yaw_angle_deg']))

    def test_too_few_samples_is_flagged_incomplete(self):
        s = T.summarize_yaw(self._run([-8.0] * 4))
        self.assertTrue(s['have'])
        self.assertTrue(s['incomplete'])

    def test_short_coverage_is_flagged_incomplete(self):
        """Plenty of frames, but only over the first second of a 3 s run."""
        s = T.summarize_yaw(self._run([-8.0] * 20, hz=20.0))
        self.assertLess(s['span_s'], T.BENCH_DRIVE_S * T.BENCH_YAW_MIN_COVERAGE)
        self.assertTrue(s['incomplete'])

    def test_a_dropout_is_flagged_stale_and_incomplete(self):
        gappy = ([(100.0 + i * 0.05, -8.0, None) for i in range(30)]
                 + [(100.0 + 1.45 + T.TELEMETRY_STALE_S + 1.0 + i * 0.05, -8.0, None)
                    for i in range(30)])
        s = T.summarize_yaw(gappy)
        self.assertTrue(s['stale'])
        self.assertTrue(s['incomplete'])
        self.assertGreater(s['max_gap_s'], T.TELEMETRY_STALE_S)

    def test_a_complete_clean_run_is_not_flagged(self):
        s = T.summarize_yaw(self._run([-8.0] * 60))
        self.assertFalse(s['incomplete'])
        self.assertFalse(s['stale'])


class CollectionGatingTest(unittest.TestCase):
    """Samples are kept only while the BOAT says it is in the drive phase."""

    def setUp(self):
        self.link = T.BoatLink.__new__(T.BoatLink)
        self.link._lock = __import__('threading').RLock()
        self.link.bench_status = T.BoatLink._blank_bench_status()
        self.link.bench_yaw_samples = []
        self.link.bench_yaw = None

    def _state(self, state):
        self.link.bench_status = dict(self.link.bench_status,
                                      have=True, state=state)

    def test_nothing_is_collected_before_a_run(self):
        self._state(0)
        self.link._collect_bench_yaw_locked(-8.0, 10.0)
        self.assertEqual(self.link.bench_yaw_samples, [])

    def test_nothing_is_collected_during_the_motors_off_baseline(self):
        self._state(1)
        self.link._collect_bench_yaw_locked(-8.0, 10.0)
        self.assertEqual(self.link.bench_yaw_samples, [])

    def test_collected_during_the_drive_phase(self):
        self._state(T.BENCH_STATE_RUN)
        self.link._collect_bench_yaw_locked(-8.0, 10.0)
        self.assertEqual(len(self.link.bench_yaw_samples), 1)
        self.assertEqual(self.link.bench_yaw_samples[0][1], -8.0)

    def test_nothing_is_collected_during_coast(self):
        self._state(3)
        self.link._collect_bench_yaw_locked(-8.0, 10.0)
        self.assertEqual(self.link.bench_yaw_samples, [])

    def test_nothing_is_collected_before_any_bench_status_arrives(self):
        self.link.bench_status = T.BoatLink._blank_bench_status()  # have=False
        self.link._collect_bench_yaw_locked(-8.0, 10.0)
        self.assertEqual(self.link.bench_yaw_samples, [])

    def test_the_buffer_is_bounded(self):
        """A stuck 'driving' state -- a lost terminal packet -- must not grow
        this for the rest of the session."""
        self._state(T.BENCH_STATE_RUN)
        for _ in range(4200):
            self.link._collect_bench_yaw_locked(-8.0, 10.0)
        self.assertLessEqual(len(self.link.bench_yaw_samples), 4000)


class UnchangedContractTest(unittest.TestCase):
    """Firmware, protocol, BASE behaviour and file identifiers are untouched."""

    def setUp(self):
        self.src = TOOL.read_text()

    def test_the_command_kinds_are_unchanged(self):
        self.assertEqual(T.BENCH_KIND, {'both': 0, 'left': 1, 'right': 2})
        self.assertEqual(T.BENCH_KIND_NAME, {0: 'BASE', 1: 'LEFT', 2: 'RIGHT'})

    def test_the_buttons_still_send_the_same_three_kinds(self):
        for kind in ("'left'", "'right'", "'both'"):
            self.assertIn('runBench(%s, 0)' % kind, self.src,
                          'a bench button stopped sending kind %s' % kind)

    def test_the_buttons_are_named_for_the_motor_not_a_turn_direction(self):
        """Which way the boat physically swings has not been measured -- these
        runs are how it gets measured. Naming them LEFT TURN would bake in the
        guess."""
        self.assertIn('>LEFT MOTOR STRONGER<', self.src)
        self.assertIn('>RIGHT MOTOR STRONGER<', self.src)
        for premature in ('>LEFT TURN<', '>RIGHT TURN<',
                          '>TURN LEFT<', '>TURN RIGHT<'):
            self.assertNotIn(premature, self.src)

    def test_the_default_split_is_twelve_percent(self):
        m = re.search(r'<input type="number" id="bench-delta"[^>]*>', self.src)
        self.assertIsNotNone(m)
        self.assertIn('value="12"', m.group(0))
        # and it stays operator-adjustable
        self.assertIn('min="1"', m.group(0))
        self.assertIn('max="30"', m.group(0))

    def test_the_default_throttle_is_unchanged_at_twenty(self):
        m = re.search(r'<input type="number" id="bench-throttle"[^>]*>', self.src)
        self.assertIsNotNone(m)
        self.assertIn('value="20"', m.group(0))

    def test_reset_c_semantics_are_unchanged(self):
        """RESET is still one-shot and still runs a BASE test; the plain
        buttons still send 0 so a learned c carries across runs."""
        self.assertEqual(T.TRIMLEARN_C_MIN, 0.10)
        self.assertEqual(T.TRIMLEARN_C_MAX, 0.35)
        self.assertIn("runBench('both', c);", self.src)      # RESET c + BASE
        self.assertIn("runBench('both', 0)", self.src)       # plain BASE
        self.assertIn('def send_bench(self, kind, base, delta, command_seq, '
                      'reset_c=0.0):', self.src)

    def test_the_summary_is_labelled_as_approximate(self):
        self.assertIn('UI-observed / approximate', self.src)
        self.assertIn('authoritative', self.src)

    def test_no_firmware_or_proto_field_was_invented(self):
        """The summary is derived from telemetry already on the wire. If this
        ever needed a new BenchStatus field it would be a proto change, which
        this work explicitly excludes."""
        self.assertNotIn('bs.yaw', self.src)
        self.assertNotIn('bench.yaw', self.src)
        self.assertNotIn('msg.bench.yaw', self.src)

    def test_the_drive_phase_length_mirrors_the_firmware(self):
        fw = (ROOT / 'main' / 'motor_control.c').read_text()
        self.assertIn('#define BENCH_RUN_US_SPLIT 3000000', fw)
        self.assertIn('#define BENCH_RUN_US_BASE  3000000', fw)
        self.assertEqual(T.BENCH_DRIVE_S, 3.0)


class JsMirrorTest(unittest.TestCase):
    """The browser recomputes the preview locally (no round-trip per keystroke),
    so its arithmetic must match the Python reference above."""

    def setUp(self):
        self.src = TOOL.read_text()

    def test_the_js_mirror_exists_and_uses_the_same_formula(self):
        m = re.search(r'function benchEffective\(kind, base, delta, c\) \{(.*?)\n\}',
                      self.src, re.S)
        self.assertIsNotNone(m, 'the JS preview mirror is missing')
        body = m.group(1)
        self.assertIn('nl = base + delta; nr = base - delta;', body)   # left
        self.assertIn('nl = base - delta; nr = base + delta;', body)   # right
        self.assertIn('const trim = 2 * c * base;', body)
        self.assertIn('left = clamp01(nl - 0.5 * trim);', body)
        self.assertIn('right = clamp01(nr + 0.5 * trim);', body)
        self.assertIn('nl <= 0 && nr <= 0', body)   # stopped stays stopped

    def test_the_js_warning_uses_the_same_threshold(self):
        m = re.search(r'function refreshBenchPreview\(\) \{(.*?)\n\}',
                      self.src, re.S)
        self.assertIsNotNone(m)
        body = m.group(1)
        self.assertIn('const cancel = Math.abs(benchLearnC) * base;', body)
        self.assertIn('delta < cancel', body)
        self.assertIn('delta < 2 * cancel', body)

    def test_the_preview_uses_the_boats_real_c_not_a_constant(self):
        self.assertIn('benchLearnC = bn.learn_c;', self.src)
        self.assertIn('refreshBenchPreview();', self.src)

    def test_the_preview_updates_as_the_operator_types(self):
        self.assertIn("$('bench-throttle').addEventListener('input', refreshBenchPreview);",
                      self.src)
        self.assertIn("$('bench-delta').addEventListener('input', refreshBenchPreview);",
                      self.src)


if __name__ == '__main__':
    unittest.main()
