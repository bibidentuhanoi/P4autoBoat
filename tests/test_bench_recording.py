"""BASE/bench runs recorded to the computer, alongside the boat's own SD file.

WHY. The bench card's own note said it: "UI-observed / approximate -- the boat's
SD CSV is authoritative", and "boat records to its own SD card". So a bench run
left two very different traces: a proper 100 Hz file on a card you have to pull
out of the hull, and four approximate numbers on screen that vanish on the next
run. The rudder tests have had a real laptop-side CSV all along; BASE runs had
nothing you could open afterwards.

This adds that CSV. It does NOT touch the firmware or its SD write -- that
stays exactly as it is, and remains the authoritative recording. The two are
independent on purpose: the SD file is the boat's own high-rate truth, this one
is what the radio actually delivered, and when they disagree THAT is the
finding. Neither may stand in for the other.
"""

import csv
import importlib.util
import re
import threading
import unittest
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'espnow_drive.py'
SPEC = importlib.util.spec_from_file_location('espnow_drive_bench', TOOL)
D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D)


class Clock:
    def __init__(self, t=1000.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


class BenchRecordingTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.clock = Clock()
        link = D.BoatLink.__new__(D.BoatLink)
        link._lock = threading.RLock()
        link._now = self.clock
        link.pb2 = D.load_boat_pb2()
        link.connected = True
        link.bench_status = D.BoatLink._blank_bench_status()
        link.motor_status = D.BoatLink._blank_motor_status()
        link.telemetry = D.BoatLink._blank_telemetry()
        link.bench_yaw_samples = []
        link.bench_yaw = None
        link.bench_run = None
        link._bench_write = None
        link.bench_csv_name = None
        link.rudder_test = None
        link.rudder_test_result = None
        link._rudder_test_write = None
        link.bench_run = None
        link._bench_write = None
        link.bench_csv_name = None
        link.bench_dir = Path('/tmp')
        link.rudder_test_dir = self.dir
        link.bench_dir = self.dir
        self.link = link

    def tearDown(self):
        self.tmp.cleanup()

    # ---- helpers --------------------------------------------------------

    def _status(self, state, kind=0, base=0.20, samples=0, elapsed=0.0,
                learn_c=0.20, p_on=False, file_index=0, **controller):
        msg = self.link.pb2.BenchStatus(
            state=state, kind=kind, base=base, samples=samples,
            file_index=file_index, elapsed_s=elapsed, learn_c=learn_c,
            p_on=p_on, **controller)
        self.link._handle_bench_status(msg)

    def _frame(self, yaw, heading=10.0, left=0.20, right=0.20):
        self.link.motor_status = dict(
            self.link.motor_status, have=True, state=2, servo_power=True,
            left_throttle=left, right_throttle=right,
            last_rx_monotonic=self.clock.t)
        with self.link._lock:
            self.link._collect_bench_yaw_locked(yaw, heading)

    def _run(self, kind=0, yaws=(1.0, 2.0, 3.0), end=None):
        end = D.BENCH_STATE_SAVED if end is None else end
        self._status(D.BENCH_STATE_RUN, kind=kind)
        for i, y in enumerate(yaws):
            self.clock.advance(0.05)
            self._frame(y, heading=10.0 + i)
            self._status(D.BENCH_STATE_RUN, kind=kind, samples=i + 1,
                         elapsed=(i + 1) * 0.05)
        self.clock.advance(0.05)
        self._status(end, kind=kind, samples=len(yaws), file_index=7)
        self.link._flush_bench_write()

    def _written(self):
        return sorted(p.name for p in self.dir.glob('*.csv'))

    def _rows(self, name):
        with open(self.dir / name) as fh:
            body = [l for l in fh if not l.startswith('#')]
        return list(csv.DictReader(body))

    # ---- the recording --------------------------------------------------

    def test_a_base_run_writes_a_csv_to_the_computer(self):
        self._run(kind=0)
        got = self._written()
        self.assertTrue(got, 'a BASE run recorded nothing on the computer')
        self.assertTrue(got[0].startswith('BASE_'),
                        'unexpected name %r' % got[0])

    def test_the_csv_holds_the_drive_phase_samples(self):
        self._run(kind=0, yaws=(1.0, 2.0, 3.0))
        rows = self._rows(self._written()[0])
        self.assertEqual(len(rows), 3)
        self.assertAlmostEqual(float(rows[0]['yaw_dps']), 1.0, places=3)
        self.assertAlmostEqual(float(rows[2]['yaw_dps']), 3.0, places=3)

    def test_it_records_what_the_motors_were_actually_doing(self):
        """Yaw alone cannot tell a trim result from a bad run. The columns that
        diagnosed the 2026-09-02 rudder runs belong here for the same reason."""
        self._status(D.BENCH_STATE_RUN, kind=1)
        self.clock.advance(0.05)
        self._frame(2.0, left=0.26, right=0.14)
        self._status(D.BENCH_STATE_RUN, kind=1, samples=1, elapsed=0.05)
        self.clock.advance(0.05)
        self._status(D.BENCH_STATE_SAVED, kind=1, samples=1, file_index=3)
        self.link._flush_bench_write()
        rows = self._rows(self._written()[0])
        self.assertAlmostEqual(float(rows[0]['boat_left']), 0.26, places=3)
        self.assertAlmostEqual(float(rows[0]['boat_right']), 0.14, places=3)
        self.assertEqual(rows[0]['boat_state'], '2')
        self.assertEqual(rows[0]['boat_servo_power'], '1')

    def test_it_records_one_coherent_yaw_controller_snapshot(self):
        diagnostic = {
            'heading_target_deg': 12.5, 'heading_error_deg': 2.25,
            'yaw_target_dps': 1.8, 'p_term': 0.11, 'i_term': 0.07,
            'dynamic_c': 0.18, 'effective_c': 0.21, 'c_limit': 1.0,
            'ctrl_active': True, 'heading_hold': True, 'saturated': False,
        }
        self._status(D.BENCH_STATE_RUN, kind=0, p_on=True, **diagnostic)
        self.clock.advance(0.05)
        self._frame(-0.75, heading=10.0)
        self._status(D.BENCH_STATE_SAVED, kind=0, samples=1, file_index=3,
                     **diagnostic)
        self.link._flush_bench_write()

        row = self._rows(self._written()[0])[0]
        for name, value in diagnostic.items():
            expected = 1 if value is True else 0 if value is False else value
            self.assertAlmostEqual(float(row[name]), expected, places=4, msg=name)

    def test_legacy_zero_diagnostics_still_make_a_valid_row(self):
        self._run(kind=0, yaws=(0.5,))
        row = self._rows(self._written()[0])[0]
        for name in ('heading_target_deg', 'heading_error_deg', 'yaw_target_dps',
                     'p_term', 'i_term', 'dynamic_c', 'effective_c', 'c_limit',
                     'ctrl_active', 'heading_hold', 'saturated'):
            self.assertEqual(float(row[name]), 0.0, name)

    def test_header_summarizes_controller_effort_over_drive_rows(self):
        self._status(D.BENCH_STATE_RUN, kind=0, p_on=True, ctrl_active=True,
                     heading_hold=True, p_term=-0.10, i_term=0.03)
        self.clock.advance(0.05)
        self._frame(-2.0, heading=359.0)
        self._status(D.BENCH_STATE_RUN, kind=0, samples=1, elapsed=0.05,
                     p_on=True, ctrl_active=True, heading_hold=True,
                     saturated=True, p_term=0.25, i_term=0.08)
        self.clock.advance(0.05)
        self._frame(1.0, heading=1.0)
        self._status(D.BENCH_STATE_SAVED, kind=0, samples=2, file_index=4)
        self.link._flush_bench_write()

        text = (self.dir / self._written()[0]).read_text()
        self.assertIn('heading_change_deg=2.0', text)
        self.assertIn('active_fraction=1.0', text)
        self.assertIn('saturated_fraction=0.5', text)
        self.assertIn('peak_abs_p=0.25', text)
        self.assertIn('peak_abs_i=0.08', text)
        self.assertIn('final_i=0.08', text)

    def test_the_three_kinds_never_pool_under_one_glob(self):
        """BASE, LEFT and RIGHT are different experiments."""
        self._run(kind=0)
        self._run(kind=1)
        self._run(kind=2)
        got = self._written()
        self.assertEqual(len(got), 3)
        self.assertEqual(len({n.split('_')[0] for n in got}), 3, got)

    def test_repeat_runs_do_not_overwrite_each_other(self):
        self._run(kind=0)
        self._run(kind=0)
        self.assertEqual(len(self._written()), 2)

    def test_an_aborted_run_is_written_but_clearly_marked(self):
        """Partial data is still evidence -- but a tidy-looking summary of a run
        that never happened is the worst possible output."""
        self._run(kind=0, yaws=(1.0, 2.0), end=D.BENCH_STATE_FAILED)
        name = self._written()[0]
        head = (self.dir / name).read_text()
        self.assertIn('ABORTED', head)

    def test_the_header_says_where_the_data_came_from(self):
        """It must be impossible to mistake this for the boat's SD recording."""
        self._run(kind=0)
        head = (self.dir / self._written()[0]).read_text()
        self.assertIn('NOT the boat SD card', head)
        self.assertIn('file_index', head)   # so the SD file can be matched up

    def test_a_run_that_never_drove_writes_nothing(self):
        self._status(D.BENCH_STATE_STILL if hasattr(D, 'BENCH_STATE_STILL')
                     else 1, kind=0)
        self._status(D.BENCH_STATE_FAILED, kind=0)
        self.link._flush_bench_write()
        self.assertEqual(self._written(), [])


class FirmwareUntouchedTest(unittest.TestCase):
    """The boat's own SD write stays exactly as it was, and stays the
    authoritative recording."""

    def test_the_bench_firmware_still_owns_its_sd_recording(self):
        """bench_run.c accumulates the samples; motor_control.c writes them.
        Both stay exactly as they were -- this feature is Python-only, so no
        reflash is needed to get it and no firmware behaviour changes to get
        it wrong."""
        src = (ROOT / 'main' / 'motor_control.c').read_text()
        self.assertIn('static void bench_write_csv(void)', src,
                      'the boat no longer writes its own bench file')
        self.assertIn('fs_sdcard_ready()', src)

    def test_the_ui_still_says_the_sd_file_is_authoritative(self):
        page = TOOL.read_text()
        self.assertIn('the boat&#39;s SD CSV is authoritative'
                      if 'the boat&#39;s SD CSV' in page
                      else "the boat's SD CSV is authoritative", page)


if __name__ == '__main__':
    unittest.main()


class UISurfacingTest(BenchRecordingTest):

    def test_the_written_file_is_reported_to_the_ui(self):
        """A file you cannot find is nearly as bad as no file."""
        self._run(kind=0)
        self.assertTrue(self.link.bench_csv_name)
        self.assertTrue(self.link.bench_csv_name.startswith('BASE_T20_'))

    def test_the_page_shows_it(self):
        src = TOOL.read_text()
        self.assertIn("id=\"bench-csv\"", src)
        self.assertIn("$('bench-csv').textContent = s.bench_csv", src)
        self.assertIn('dataout/', src)


class BaseLongRecordingTest(BenchRecordingTest):
    """BENCH_KIND_BASE_LONG (3): the same BASE run, driven 10 s instead of 3.

    The laptop side has one job here and it is not subtle: never let a 10 s run
    be mistaken for a 3 s one. They are the same experiment at different
    durations, so their numbers look comparable and are not -- a long run
    averaged into the historical BASE_T20_* set would quietly corrupt the whole
    series. Distinct kind, distinct filename, no exceptions.
    """

    def test_the_long_kind_exists_and_is_three(self):
        self.assertEqual(D.BENCH_KIND['both_long'], 3)
        self.assertEqual(D.BENCH_KIND_NAME[3], 'BASE10')

    def test_a_long_run_writes_a_distinctly_named_csv(self):
        self._run(kind=3)
        got = self._written()
        self.assertTrue(got)
        self.assertTrue(got[0].startswith('BASE10_T20_'),
                        'unexpected name %r' % got[0])

    def test_a_long_run_can_never_be_globbed_with_historical_base_files(self):
        """BASE_T20_* is 21 runs of real 3 s data. A 10 s file landing in that
        set would be averaged with them and nobody would ever know."""
        self._run(kind=0)          # ordinary BASE
        self._run(kind=3)          # the long one
        got = self._written()
        self.assertEqual(len(got), 2)
        base = [n for n in got if n.startswith('BASE_T20_')]
        long_ = [n for n in got if n.startswith('BASE10_T20_')]
        self.assertEqual(len(base), 1)
        self.assertEqual(len(long_), 1)
        # the decisive property: the historical glob must not catch the new file
        import fnmatch
        self.assertFalse(fnmatch.fnmatch(long_[0], 'BASE_T20_*'),
                         '%s matches the historical BASE glob' % long_[0])

    def test_the_columns_are_exactly_the_same_as_base(self):
        """'Keep MotorStatus columns and raw samples exactly as recorded.'"""
        self._run(kind=0)
        self._run(kind=3)
        got = self._written()
        cols = [list(self._rows(n)[0].keys()) for n in got]
        self.assertEqual(cols[0], cols[1])
        self.assertEqual(cols[0], list(D.BENCH_CSV_COLUMNS))
        for name in ('boat_left', 'boat_right', 'boat_state',
                     'boat_servo_power'):
            self.assertIn(name, cols[0])

    def test_the_header_identifies_the_long_mode(self):
        self._run(kind=3)
        head = (self.dir / self._written()[0]).read_text()
        self.assertIn('kind=BASE10', head)
        self.assertIn('NOT the boat SD card', head)

    def test_an_aborted_long_run_is_marked_like_any_other(self):
        self._run(kind=3, yaws=(1.0, 2.0), end=D.BENCH_STATE_FAILED)
        head = (self.dir / self._written()[0]).read_text()
        self.assertIn('ABORTED', head)


class BaseLongPipelineTest(unittest.TestCase):
    """The command path, and the parts of the firmware the laptop depends on."""

    def setUp(self):
        self.tool = TOOL.read_text()
        self.mc = (ROOT / 'main' / 'motor_control.c').read_text()
        self.bh = (ROOT / 'main' / 'bench_run.h').read_text()

    def test_the_api_accepts_the_long_kind(self):
        self.assertIn("'both_long': 3", self.tool)

    def test_there_is_a_separate_clearly_labelled_button(self):
        self.assertIn('id="bench-base-short"', self.tool)
        self.assertIn('id="bench-base-long"', self.tool)
        self.assertIn('>BASE TEST 3s</button>', self.tool)
        self.assertIn('>BASE TEST 30s</button>', self.tool)
        self.assertIn("$('bench-base-short').addEventListener('click', () => runLakeId('straight'));",
                      self.tool)
        self.assertIn("$('bench-base-long').addEventListener('click', () => runLakeId('straight30'));",
                      self.tool)
        self.assertNotIn("runBench('both_long', 0)", self.tool)
        # ...and the ordinary BASE button is untouched
        self.assertIn("$('bench-base').addEventListener('click', "
                      "() => runBench('both', 0));", self.tool)

    def test_the_long_button_is_disabled_with_the_others_during_a_run(self):
        """Mutual exclusion: it must be in the same disable list, or it could
        be pressed mid-run and refused only server-side."""
        m = re.search(r"\['bench-left', 'bench-right', 'bench-base',(.*?)\]",
                      self.tool, re.S)
        self.assertIsNotNone(m, 'the bench disable list moved')
        self.assertIn('bench-base-long', m.group(1))

    def test_ordinary_base_is_still_exactly_three_seconds(self):
        self.assertIn('#define BENCH_RUN_US_BASE  3000000', self.mc)
        self.assertIn('#define BENCH_RUN_US_SPLIT 3000000', self.mc)

    def test_the_long_run_is_ten_seconds(self):
        self.assertIn('#define BENCH_RUN_US_BASE_LONG 10000000', self.mc)

    def test_the_firmware_validation_bound_includes_the_new_kind(self):
        """Without this a long request silently executes as a 3 s BASE."""
        self.assertIn('if (kind > (uint32_t)BENCH_KIND_BASE_LONG) '
                      'kind = (uint32_t)BENCH_KIND_BASE;', self.mc)

    def test_the_sd_filename_does_not_collide_with_base(self):
        """The boat's own file is authoritative; a 10 s run filed as T20_B_xx
        would be indistinguishable from the 3 s runs."""
        self.assertIn("(s_bench.kind == BENCH_KIND_BASE_LONG) ? 'G'", self.mc)

    def test_the_buffer_is_sized_for_the_long_run(self):
        m = re.search(r'#define BENCH_MAX_SAMPLES (\d+)u', self.bh)
        self.assertIsNotNone(m)
        n = int(m.group(1))
        needed = int((0.5 + 10 + 1) * 100)      # 100 Hz control task
        self.assertGreaterEqual(n, needed)
        self.assertGreaterEqual(n, needed + needed // 10, 'less than 10% margin')
        self.assertLessEqual(n, 65535, 'bench_t.count is uint16_t')

    def test_no_protobuf_change_was_needed(self):
        """kind is already a uint32 in BenchCommand and BenchStatus, so a new
        enumerated value needs no schema change and no regeneration."""
        proto = (ROOT / 'main' / 'proto' / 'boat.proto').read_text()
        self.assertIn('uint32 kind  = 1;', proto)
        self.assertRegex(proto, r'message BenchStatus \{[^}]*uint32 kind\s+= 2;')
