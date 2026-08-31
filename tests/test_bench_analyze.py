"""Coverage for the SD-card bench-run analyzer."""

import importlib.util
import os
import io
import re
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'bench_analyze_under_test', REPO_ROOT / 'tools' / 'bench_analyze.py')
ba = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ba
SPEC.loader.exec_module(ba)


def write_run(folder, name, steady, baseline=0.0, left=0.240, right=0.160):
    """A file shaped exactly like the boat writes: baseline, run, coast.

    Rows alternate by a hair. A real gyro column is never byte-identical row to
    row, and the analyzer treats a column that IS as a frozen sensor rather
    than a perfectly steady boat -- so a fixture of identical values would be
    thrown out before any of these tests could see it. The wobble is 2e-5
    deg/s, far below every tolerance asserted here."""
    j = lambda i: 0.00002 if i % 2 else -0.00002
    lines = ['t_s,phase,yaw_dps,left,right']
    for i in range(5):                                   # 0.0-0.4 motors off
        lines.append('%.3f,baseline,%.5f,0.000,0.000' % (i / 10.0, baseline + j(i)))
    for i in range(30):                                  # 0.5-3.4 driving
        lines.append('%.3f,run,%.5f,%.3f,%.3f'
                     % (0.5 + i / 10.0, steady + j(i), left, right))
    for i in range(10):                                  # 3.5-4.4 coasting
        lines.append('%.3f,coast,%.5f,0.000,0.000'
                     % (3.5 + i / 10.0, steady / 2 + j(i)))
    (folder / name).write_text('\n'.join(lines) + '\n')


class SummarizeTest(unittest.TestCase):
    def test_baseline_is_subtracted_and_only_the_tail_counts(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            write_run(folder, 'T20_B_01.CSV', steady=12.0, baseline=2.0)
            s = ba.summarize(ba.read_run(folder / 'T20_B_01.CSV'))
            self.assertAlmostEqual(s['baseline_dps'], 2.0, places=3)
            self.assertAlmostEqual(s['steady_dps'], 12.0, places=3)
            self.assertAlmostEqual(s['corrected_dps'], 10.0, places=3)
            self.assertEqual(s['coast_n'], 10)
            self.assertFalse(s['thin'])

    def test_a_run_with_almost_no_drive_data_is_flagged_thin(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder / 'T20_B_01.CSV').write_text(
                't_s,phase,yaw_dps,left,right\n'
                '0.100,baseline,0.000,0.000,0.000\n'
                '3.000,run,12.000,0.200,0.200\n')
            s = ba.summarize(ba.read_run(folder / 'T20_B_01.CSV'))
            self.assertTrue(s['thin'])

    def test_a_torn_last_line_does_not_abort_the_read(self):
        """Power pulled mid-write must not make the whole run unreadable."""
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'T20_B_01.CSV'
            p.write_text('t_s,phase,yaw_dps,left,right\n'
                         '0.100,baseline,0.000,0.000,0.000\n'
                         '0.200,run')          # cut off mid-row
            rows = ba.read_run(p)
            self.assertEqual(len(rows), 1)


class TrimTest(unittest.TestCase):
    def test_trim_from_the_three_runs(self):
        self.assertAlmostEqual(ba.trim_from(3.0, 13.0, -7.0, 0.04),
                               -0.012, places=5)

    def test_no_trim_when_both_pushes_read_the_same(self):
        self.assertIsNone(ba.trim_from(3.0, 5.0, 5.0, 0.04))


class CommandLineTest(unittest.TestCase):
    def test_reports_the_trim_and_which_motor_to_feed(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            write_run(folder, 'T20_B_01.CSV', steady=3.0)
            write_run(folder, 'T20_L_01.CSV', steady=13.0)
            write_run(folder, 'T20_R_01.CSV', steady=-7.0)
            out = io.StringIO()
            with redirect_stdout(out):
                rc = ba.main([str(folder), '--delta', '0.04'])
            text = out.getvalue()
            self.assertEqual(rc, 0)
            self.assertIn('T20_B_01.CSV', text)
            self.assertIn('give RIGHT more', text)
            self.assertIn('-0.012', text)

    def test_says_what_is_missing_when_a_run_has_not_been_done(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            write_run(folder, 'T20_B_01.CSV', steady=3.0)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            self.assertIn('would measure the gain', out.getvalue())

    def test_a_frozen_gyro_is_thrown_out_not_read_as_a_straight_boat(self):
        """An unchanging yaw column is a dead sensor, and it reads as the most
        convincing possible result: a boat that never turns. It must not reach
        the median."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            lines = ['t_s,phase,yaw_dps,left,right']
            for i in range(45):
                lines.append('%.3f,run,0.000,0.240,0.160' % (i / 10.0))
            (folder / 'T20_B_01.CSV').write_text('\n'.join(lines) + '\n')
            write_run(folder, 'T20_B_02.CSV', steady=6.0)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            text = out.getvalue()
            self.assertIn('GYRO STALLED', text)
            # the live run is the only one left, so the median is ITS value
            self.assertIn('median   +6.00', text)

    def test_the_applied_trim_is_read_back_out_of_the_run_itself(self):
        """c = (right-left)/(right+left): the throttle cancels, so the file
        says what trim it ran with, whatever level it ran at."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            # T30 driven with c = 0.20 -> left 0.24, right 0.36
            write_run(folder, 'T30_B_01.CSV', steady=0.0, left=0.240, right=0.360)
            s = ba.summarize(ba.read_run(folder / 'T30_B_01.CSV'))
            self.assertAlmostEqual(s['c_applied'], 0.20, places=6)
            # same c at a different throttle reads the same
            write_run(folder, 'T10_B_01.CSV', steady=0.0, left=0.080, right=0.120)
            s = ba.summarize(ba.read_run(folder / 'T10_B_01.CSV'))
            self.assertAlmostEqual(s['c_applied'], 0.20, places=6)

    def test_base_only_runs_still_get_a_verdict_and_a_c_correction(self):
        """BASE runs alone answer the only question that matters -- does it go
        straight -- and, divided by the known response, say which way to move
        c. Requiring LEFT and RIGHT to say anything would make a straight-line
        check impossible."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i, y in enumerate((1.5, 1.9, 1.4, 1.8), start=1):
                write_run(folder, 'T30_B_%02d.CSV' % i, steady=y,   # turns right
                          left=0.240, right=0.360)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            text = out.getvalue()
            self.assertIn('turns RIGHT', text)
            self.assertIn('ran c=0.200', text)
            # 1.65 deg/s / 16 per unit c = 0.103 down -> 0.097
            self.assertRegex(text, r'suggests c=0\.09\d')

    def test_a_huge_correction_is_offered_as_a_step_not_a_destination(self):
        """Far from zero yaw the response is not a straight line, so a big
        computed jump must be walked toward, not taken in one go."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i, y in enumerate((9.0, 9.6, 9.2, 9.4), start=1):
                write_run(folder, 'T10_B_%02d.CSV' % i, steady=y,
                          left=0.024, right=0.176)          # c = 0.76
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            text = out.getvalue()
            self.assertIn('big jump', text)
            self.assertIn('Move to c=0.660', text)          # 0.76 - 0.10

    def test_a_correction_that_flips_the_strong_motor_is_questioned(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i, y in enumerate((4.0, 4.4, 3.9, 4.3), start=1):
                write_run(folder, 'T30_B_%02d.CSV' % i, steady=y,
                          left=0.285, right=0.315)          # c = 0.05
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            self.assertIn('every run on file says the opposite',
                          out.getvalue())

    def test_a_straight_boat_is_told_to_leave_c_alone(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i, y in enumerate((0.15, -0.20, 0.10, -0.05), start=1):
                write_run(folder, 'T40_B_%02d.CSV' % i, steady=y,
                          left=0.320, right=0.480)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            text = out.getvalue()
            self.assertIn('STRAIGHT', text)
            self.assertIn('leave c alone', text)

    def test_empty_folder_is_reported_not_crashed(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = io.StringIO()
            with redirect_stdout(out):
                rc = ba.main([tmp])
            self.assertEqual(rc, 1)
            self.assertIn('no run files', out.getvalue())


class RepeatedRunsTest(unittest.TestCase):
    """Repeats are how you beat the noise, so every run of a kind must count --
    taking only the last one throws the evidence away."""

    def test_repeats_are_combined_by_median_not_last_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i, val in enumerate((0.2, 13.6, -8.2), start=1):
                write_run(folder, 'T10_B_%02d.CSV' % i, steady=val)
            write_run(folder, 'T10_L_01.CSV', steady=-12.2)
            write_run(folder, 'T10_R_01.CSV', steady=5.1)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder), '--delta', '0.04'])
            text = out.getvalue()
            # median of (0.2, 13.6, -8.2) is 0.2, NOT the last value -8.2
            self.assertIn('median', text)
            self.assertIn('n=3', text)

    def test_a_wildly_scattered_set_is_called_out(self):
        """If the spread swamps the reading, the trim is not measurable and the
        report must say so instead of printing a confident number."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i, val in enumerate((0.2, 13.6, -8.2), start=1):
                write_run(folder, 'T10_B_%02d.CSV' % i, steady=val)
            write_run(folder, 'T10_L_01.CSV', steady=-12.2)
            write_run(folder, 'T10_R_01.CSV', steady=5.1)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder), '--delta', '0.04'])
            self.assertIn('NOT MEASURABLE', out.getvalue())


class DiagnosticsTest(unittest.TestCase):
    def test_a_single_wild_run_is_flagged_as_an_outlier(self):
        """One jerked run among good ones must be named, so it is obvious the
        median is carrying the result rather than the data being clean."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i, v in enumerate((1.8, 1.0, 2.0, 19.0), start=1):
                write_run(folder, 'T30_R_%02d.CSV' % i, steady=v)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            self.assertIn('OUTLIER', out.getvalue())

    def test_left_and_right_must_mirror_about_base(self):
        """LEFT and RIGHT are equal and opposite splits, so their response
        about BASE must mirror. If it does not, the straight-line assumption
        the trim rests on is broken and the trim cannot be believed."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i in range(1, 4):
                write_run(folder, 'T30_B_%02d.CSV' % i, steady=0.0)
                write_run(folder, 'T30_L_%02d.CSV' % i, steady=-0.5)
                write_run(folder, 'T30_R_%02d.CSV' % i, steady=+5.0)  # 10x, not a mirror
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            self.assertIn('do not mirror', out.getvalue())


class UncertaintyTest(unittest.TestCase):
    def test_more_repeats_shrink_the_error_bar(self):
        """Averaging n runs beats down random scatter as sqrt(n), so doing the
        work of extra repeats must actually show up as a tighter answer."""
        import statistics
        scatter = [0.6, -0.6, 0.3, -0.3, 0.0]
        few = ba.trim_uncertainty(scatter[:2], gain=-38.0)
        many = ba.trim_uncertainty(scatter, gain=-38.0)
        self.assertLess(many, few)

    def test_a_single_run_reports_unknown_uncertainty(self):
        """One run gives no way to know its own scatter -- say so rather than
        reporting a suspiciously tight zero."""
        self.assertIsNone(ba.trim_uncertainty([1.0], gain=-38.0))


class BoatConventionTest(unittest.TestCase):
    """esc_trim_mix applies a trim as  left -= t/2 ; right += t/2.
    The bench split is  left += d ; right -= d.  So the boat's number is the
    NEGATIVE, DOUBLE of the measured split -- writing the raw split into the
    trim table would push the wrong way, twice as hard."""

    def test_boat_trim_is_minus_twice_the_measured_split(self):
        self.assertAlmostEqual(ba.to_boat_trim(0.0094), -0.0188, places=6)
        self.assertAlmostEqual(ba.to_boat_trim(-0.0177), 0.0354, places=6)

    def test_giving_left_more_means_a_negative_boat_trim(self):
        """A positive split means 'left stronger'. Feeding that through the
        boat's mixer must actually raise the left motor."""
        t = ba.to_boat_trim(0.01)          # want left +1%, right -1%
        left = 0.30 - 0.5 * t
        right = 0.30 + 0.5 * t
        self.assertAlmostEqual(left, 0.31, places=6)
        self.assertAlmostEqual(right, 0.29, places=6)


class WriterReaderAgreementTest(unittest.TestCase):
    """The boat writes the CSV in C; this tool reads it in Python. A silent
    disagreement means the analyzer quietly reads nothing, so the column names
    are asserted against the firmware source rather than trusted."""

    def test_analyzer_columns_match_what_the_firmware_writes(self):
        src = (REPO_ROOT / 'main' / 'motor_control.c').read_text()
        m = re.search(r'snprintf\(chunk, sizeof\(chunk\), "([^"\\]+)\\n"\)', src)
        self.assertIsNotNone(m, 'could not find the CSV header the firmware writes')
        header = m.group(1).split(',')
        # every column the analyzer reads must exist in the firmware's header
        for needed in ('t_s', 'phase', 'yaw_dps'):
            self.assertIn(needed, header)

    def test_phase_labels_match_what_the_firmware_writes(self):
        src = (REPO_ROOT / 'main' / 'motor_control.c').read_text()
        for label in ('baseline', 'run', 'coast'):
            self.assertIn('"%s"' % label, src,
                          'firmware no longer writes the %r phase label' % label)


if __name__ == '__main__':
    unittest.main()


class AdaptiveTrimTest(unittest.TestCase):
    """Once the trim adapts, two runs at one throttle are no longer two
    measurements of one thing. Averaging them blends the before and the after
    into a number describing neither -- and hides the adaptation completely,
    which is the one thing these files exist to show."""

    def test_runs_with_different_trims_are_never_averaged_together(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            # before: c = 0.20, still turning left
            for i in (1, 2, 3):
                write_run(folder, 'T30_B_%02d.CSV' % i, steady=-2.2,
                          left=0.240, right=0.360)
            # after the learner moved to c = 0.26: straight
            for i in (4, 5, 6):
                write_run(folder, 'T30_B_%02d.CSV' % i, steady=0.05,
                          left=0.222, right=0.378)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            text = out.getvalue()

            self.assertIn('driven with 2 different trims', text)
            self.assertIn('NOT averaged', text)
            # both trims reported with their OWN result
            self.assertRegex(text, r'c=0\.200.*yaw\s+-2\.20')
            self.assertRegex(text, r'c=0\.260.*yaw\s+\+0\.05')
            self.assertIn('flattest of these is c=0.260', text)
            # the blended median (-1.07) must never be presented as an answer
            self.assertNotIn('-1.07', text)

    def test_repeats_of_one_trim_are_still_averaged(self):
        """The split only kicks in when the trim actually differs -- repeats at
        one trim are how hand-held noise gets beaten down and must still be
        pooled."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            for i, y in enumerate((-2.0, -2.4, -2.2, -2.1), start=1):
                write_run(folder, 'T30_B_%02d.CSV' % i, steady=y,
                          left=0.240, right=0.360)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            text = out.getvalue()
            self.assertNotIn('different trims', text)
            self.assertIn('n=4', text)


def write_live_run(folder, name, c_start, c_end, yaw_early, yaw_late,
                   throttle=0.30, run_s=10.0):
    """A run with the learner LIVE: c walks across the drive window and the
    boat straightens as it does. Shaped exactly like the boat writes it,
    including the per-sample c column."""
    lines = ['t_s,phase,yaw_dps,left,right,c']
    j = lambda i: 0.00002 if i % 2 else -0.00002
    for i in range(5):
        lines.append('%.3f,baseline,%.5f,0.000,0.000,%.4f'
                     % (i / 10.0, j(i), c_start))
    n = int(run_s * 10)
    # The move happens in the MIDDLE of the run, flat at each end -- which is
    # both what converging then holding looks like, and what makes the two
    # end-windows report the endpoints rather than a slice of the ramp.
    lo, hi = int(n * 0.25), int(n * 0.75)
    for i in range(n):
        f = 0.0 if i <= lo else (1.0 if i >= hi else (i - lo) / float(hi - lo))
        c = c_start + (c_end - c_start) * f
        yaw = yaw_early + (yaw_late - yaw_early) * f + j(i)
        split = c * throttle
        lines.append('%.3f,run,%.5f,%.3f,%.3f,%.4f'
                     % (0.5 + i / 10.0, yaw, throttle - split,
                        throttle + split, c))
    for i in range(10):
        lines.append('%.3f,coast,%.5f,0.000,0.000,%.4f'
                     % (0.5 + run_s + i / 10.0, j(i), c_end))
    (folder / name).write_text('\n'.join(lines) + '\n')


class LiveLearnerRunTest(unittest.TestCase):
    """A BASE run keeps the learner live, so the trim moves DURING the run.
    Whether it worked can only be read inside one file -- the boat gets picked
    up and re-held between runs, so comparing separate runs cannot show it."""

    def test_a_run_where_the_trim_moved_reports_it_and_says_if_it_helped(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            write_live_run(folder, 'T30_B_01.CSV', c_start=0.200, c_end=0.244,
                           yaw_early=-2.10, yaw_late=-0.20)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            text = out.getvalue()
            self.assertIn('trim moved 0.200 -> 0.244 (+0.044)', text)
            self.assertIn('STRAIGHTENING', text)

    def test_a_trim_that_moved_the_wrong_way_is_not_called_straightening(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            write_live_run(folder, 'T30_B_01.CSV', c_start=0.200, c_end=0.150,
                           yaw_early=-0.30, yaw_late=-2.80)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            text = out.getvalue()
            self.assertIn('trim moved 0.200 -> 0.150', text)
            self.assertNotIn('STRAIGHTENING', text)
            self.assertIn('still working', text)

    def test_a_fixed_trim_run_says_nothing_about_adaptation(self):
        """LEFT/RIGHT runs, and any run from before the learner, hold one
        trim -- claiming movement there would be noise dressed as a result."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            write_live_run(folder, 'T30_B_01.CSV', c_start=0.200, c_end=0.200,
                           yaw_early=-1.0, yaw_late=-1.0)
            out = io.StringIO()
            with redirect_stdout(out):
                ba.main([str(folder)])
            self.assertNotIn('trim moved', out.getvalue())

    def test_old_files_without_the_c_column_still_read(self):
        """The column is new. Every run already on the card predates it, and c
        is still recoverable from the commands."""
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            write_run(folder, 'T30_B_01.CSV', steady=-1.0,
                      left=0.240, right=0.360)          # no c column
            s = ba.summarize(ba.read_run(folder / 'T30_B_01.CSV'))
            self.assertAlmostEqual(s['c_applied'], 0.20, places=6)
            self.assertFalse(s['adapted'])


def write_ab_run(folder, name, yaw_early, yaw_late, p_on, mtime=None,
                 c=0.170, p_corr=0.0, contact_at=None, run_s=3.0):
    """A run in the A/B format the boat now writes."""
    lines = ['t_s,phase,yaw_dps,left,right,c,p_on,p_yaw,c_learn,p_corr,split,at_cap']
    j = lambda i: 0.00002 if i % 2 else -0.00002
    T = 0.40
    for i in range(50):
        lines.append('%.3f,baseline,%.5f,0.000,0.000,%.4f,%d,0,%.4f,0,0,0'
                     % (i / 100.0, j(i), c, 1 if p_on else 0, c))
    n = int(run_s * 100)
    for i in range(n):
        t = i / 100.0
        f = i / (n - 1.0)
        y = yaw_early + (yaw_late - yaw_early) * f + j(i)
        if contact_at is not None and t >= contact_at:
            y += 30.0 if i % 2 else -30.0
        split = c * T
        lines.append('%.3f,run,%.5f,%.3f,%.3f,%.4f,%d,%.3f,%.4f,%.4f,%.4f,0'
                     % (0.5 + t, y, T - split, T + split, c,
                        1 if p_on else 0, y, c - p_corr, p_corr, split))
    for i in range(100):
        lines.append('%.3f,coast,%.5f,0.000,0.000,%.4f,%d,0,%.4f,0,0,0'
                     % (0.5 + run_s + i / 100.0, j(i), c, 1 if p_on else 0, c))
    p = folder / name
    p.write_text('\n'.join(lines) + '\n')
    if mtime:
        os.utime(p, (mtime, mtime))


class ABReportTest(unittest.TestCase):
    """The A/B report. Three windows fixed in advance, contact runs rejected,
    and every B file must actually say it was a B."""

    def run_ab(self, folder):
        out = io.StringIO()
        with redirect_stdout(out):
            rc = ba.main(['--ab', str(folder)])
        return rc, out.getvalue()

    def test_all_three_windows_are_reported_every_time(self):
        """Fixed in advance so a result cannot be rescued by picking the
        flattering window after seeing the numbers."""
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            for i in range(1, 5):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, False, 1000 + i)
            for i in range(5, 9):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -0.5, True, 1000 + i)
            _rc, text = self.run_ab(f)
            for w in ('whole 0.2-3.0s', 'early 0.2-2.0s', 'late  1.0-3.0s'):
                self.assertIn(w, text)

    def test_wall_contact_runs_are_rejected_not_averaged(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            write_ab_run(f, 'T40_B_01.CSV', -2.0, -2.0, False, 1001)
            write_ab_run(f, 'T40_B_02.CSV', -2.0, -2.0, False, 1002, contact_at=1.5)
            write_ab_run(f, 'T40_B_03.CSV', -2.0, -0.5, True, 1003)
            _rc, text = self.run_ab(f)
            self.assertIn('REJECTED', text)
            self.assertIn('T40_B_02.CSV', text)
            self.assertIn('1 rejected for wall contact', text)

    def test_one_arm_alone_refuses_to_conclude(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            for i in range(1, 5):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, False, 1000 + i)
            rc, text = self.run_ab(f)
            self.assertIn('need BOTH arms', text)
            self.assertEqual(rc, 1)

    def test_b_must_confirm_p_on(self):
        """A B arm whose files say p_on=0 is not a B arm."""
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            for i in range(1, 5):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, False, 1000 + i)
            for i in range(5, 9):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -0.4, True, 1000 + i)
            _rc, text = self.run_ab(f)
            self.assertIn('every B file confirms p_on=1: yes', text)

    def test_a_real_improvement_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            for i in range(1, 7):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, False, 1000 + i)
            for i in range(7, 13):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -0.2, True, 1000 + i)
            rc, text = self.run_ab(f)
            self.assertIn('B PASSES', text)
            self.assertEqual(rc, 0)

    def test_no_improvement_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            for i in range(1, 7):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, False, 1000 + i)
            for i in range(7, 13):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, True, 1000 + i)
            rc, text = self.run_ab(f)
            self.assertIn('B does not pass', text)
            self.assertEqual(rc, 2)


class POnClassificationTest(unittest.TestCase):
    """Arms are decided by the WHOLE file. Classifying on any single sample
    would file a run that was P-off for nearly all its length as a B, quietly
    poisoning that arm; and a run whose p_on changed mid-flight belongs to
    neither arm."""

    def run_ab(self, folder):
        out = io.StringIO()
        with redirect_stdout(out):
            rc = ba.main(['--ab', str(folder)])
        return rc, out.getvalue()

    def write_mixed(self, folder, name, flip_at, mtime):
        """p_on flips partway through -- the toggle pressed during a run."""
        lines = ['t_s,phase,yaw_dps,left,right,c,p_on,p_yaw,c_learn,p_corr,split,at_cap']
        T, c = 0.40, 0.170
        for i in range(50):
            lines.append('%.3f,baseline,0.00001,0.000,0.000,%.4f,0,0,%.4f,0,0,0'
                         % (i / 100.0, c, c))
        for i in range(300):
            t = i / 100.0
            on = 1 if t >= flip_at else 0
            lines.append('%.3f,run,-1.00001,%.3f,%.3f,%.4f,%d,0,%.4f,0,%.4f,0'
                         % (0.5 + t, T - c * T, T + c * T, c, on, c, c * T))
        for i in range(100):
            lines.append('%.3f,coast,0.00001,0.000,0.000,%.4f,1,0,%.4f,0,0,0'
                         % (3.5 + i / 100.0, c, c))
        p = folder / name
        p.write_text('\n'.join(lines) + '\n')
        os.utime(p, (mtime, mtime))

    def test_a_run_whose_p_on_changes_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            for i in range(1, 4):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, False, 1000 + i)
            for i in range(4, 7):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -0.3, True, 1000 + i)
            self.write_mixed(f, 'T40_B_07.CSV', 1.5, 1007)
            _rc, text = self.run_ab(f)
            self.assertIn('p_on CHANGED during the run', text)
            self.assertIn('T40_B_07.CSV', text)
            self.assertIn('1 rejected', text)

    def test_a_mixed_run_lands_in_neither_arm(self):
        """The counts must not include it -- rejecting it in the message but
        still averaging it in would be worse than not rejecting it."""
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            for i in range(1, 4):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, False, 1000 + i)
            for i in range(4, 7):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -0.3, True, 1000 + i)
            self.write_mixed(f, 'T40_B_07.CSV', 1.5, 1007)
            _rc, text = self.run_ab(f)
            self.assertIn('A=3 B=3', text)

    def test_the_classification_itself_is_all_not_any(self):
        """Pinned at the function, not through the report. Rejection currently
        hides the difference -- any file where all-vs-any disagree is mixed and
        thrown out -- so `any()` would slip past a report-level test and sit
        there waiting for the rejection to be relaxed."""
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            self.write_mixed(f, 'T40_B_01.CSV', 2.9, 1001)   # one late sample on
            r = ba.ab_run(f / 'T40_B_01.CSV')
            self.assertTrue(r['mixed_p'])
            self.assertFalse(r['p_on'],
                             "classified as B on a minority of samples -- the "
                             "test must be all(), not any()")
            self.assertTrue(r['rejected'])

            write_ab_run(f, 'T40_B_02.CSV', -1.0, -1.0, True, 1002)
            b = ba.ab_run(f / 'T40_B_02.CSV')
            self.assertTrue(b['p_on'])
            self.assertFalse(b['mixed_p'])

            write_ab_run(f, 'T40_B_03.CSV', -1.0, -1.0, False, 1003)
            a = ba.ab_run(f / 'T40_B_03.CSV')
            self.assertFalse(a['p_on'])
            self.assertFalse(a['mixed_p'])

    def test_all_zero_is_A_and_all_one_is_B(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp)
            for i in range(1, 5):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -2.0, False, 1000 + i)
            for i in range(5, 9):
                write_ab_run(f, 'T40_B_%02d.CSV' % i, -2.0, -0.3, True, 1000 + i)
            _rc, text = self.run_ab(f)
            self.assertIn('A=4 B=4', text)
            self.assertIn('every B file confirms p_on=1: yes', text)
