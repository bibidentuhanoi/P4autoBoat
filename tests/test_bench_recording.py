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
                learn_c=0.20, p_on=False, file_index=0):
        msg = self.link.pb2.BenchStatus(
            state=state, kind=kind, base=base, samples=samples,
            file_index=file_index, elapsed_s=elapsed, learn_c=learn_c,
            p_on=p_on)
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
