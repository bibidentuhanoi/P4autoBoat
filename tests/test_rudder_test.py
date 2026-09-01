"""Automatic rudder test: a laptop-driven turn, recorded over the radio.

Point of the exercise: hold a known rudder deflection at a known throttle for a
known time and see which way the boat goes, so the installed IMU's yaw sign can
finally be pinned to a physical direction.

Everything here runs on a FAKE CLOCK. The real sequence is 4.5 s of wall time
and a suite that actually slept through it would be unrunnable, so the state
machine takes `now` as a parameter and these tests hand it fabricated
monotonic values. No sleeps, no threads, no serial.

Three things this file is really guarding:

  1. The sequence is driven by the EXISTING 15 Hz command loop. It sets
     self.throttle / self.rudder and lets the loop transmit them, rather than
     opening a second command path that safety code would not know about.

  2. Every abort route ends with throttle zero AND the rudder centred. A test
     that leaves the rudder hard over on a disconnect is worse than no test.

  3. One CSV row per RECEIVED TELEMETRY FRAME, never per UI poll. Rows keyed to
     polls would silently multiply with the browser's refresh rate and make the
     integrated angle meaningless.
"""

import csv
import importlib.util
import shutil
import tempfile
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'espnow_drive.py'


def _load_tool():
    spec = importlib.util.spec_from_file_location('espnow_drive_rudder_test', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


T = _load_tool()


def read_rudder_csv(path):
    """Read a rudder-test CSV the way a consumer must: the file opens with a
    '#' provenance block (what recorded it, and that it is NOT the boat's SD
    file), so a naive DictReader would take the first comment as the header."""
    lines = [ln for ln in Path(path).read_text().splitlines()
             if not ln.startswith('#')]
    return list(csv.DictReader(lines))


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


class RudderTestBase(unittest.TestCase):
    """A BoatLink with no serial, no threads, and a clock we control."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.clock = FakeClock()
        self.writes = []
        self.write_ok = True

        link = T.BoatLink.__new__(T.BoatLink)
        link._lock = threading.RLock()
        link._now = self.clock
        link.rudder_test_dir = self.tmp
        link.connected = True
        link.throttle = 0.0
        link.motor_left = 0.0
        link.motor_right = 0.0
        link.motor_split = False
        link.rudder = 0.0
        link.winch_speed = 0.0
        link.winch_lease_until = 0.0
        link.winch_command_seq = 0
        link.armed_cmd = True
        link.force = False
        link.calibrating = False
        link.p_assist_on = False
        link.telemetry = T.BoatLink._blank_telemetry()
        link.motor_status = T.BoatLink._blank_motor_status()
        link.bench_status = T.BoatLink._blank_bench_status()
        link.bench_yaw_samples = []
        link.bench_yaw = None
        link.rudder_test = None
        link.rudder_test_result = None
        link._rudder_test_write = None
        link.seq = 0
        link.last_error = None
        link.servo_rail_cut = None
        link.calibrate_status = T.BoatLink._blank_calibrate_status()
        link.system_status = T.BoatLink._blank_system_status()
        link.bridge_status = T.BoatLink._blank_bridge_status()
        link.port = '/dev/null'
        link.ser = None
        link.pb2 = T.load_boat_pb2()
        link._close_locked = lambda: setattr(link, 'connected', False)
        # Every command the sequence emits lands here instead of a serial port.
        link._write_locked = lambda payload: (self.writes.append(payload),
                                              self.write_ok)[1]
        self.link = link
        self._boat_armed(fresh=True)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    # ---- fixtures ------------------------------------------------------

    def _boat_armed(self, fresh=True, state=2):
        self.link.motor_status = dict(
            self.link.motor_status, have=True, state=state,
            left_throttle=0.18, right_throttle=0.22,
            last_rx_monotonic=self.clock.t if fresh else self.clock.t - 60.0)

    def _start(self, sign=-1, seq=1):
        return self.link.start_rudder_test(sign, seq)

    def _tick(self):
        with self.link._lock:
            self.link._rudder_test_tick_locked(self.clock.t)

    def _frame(self, yaw=-8.0, heading=100.0):
        """One received telemetry frame."""
        with self.link._lock:
            self.link.telemetry = dict(self.link.telemetry, have=True,
                                       yaw_rate=yaw, heading=heading,
                                       last_rx_monotonic=self.clock.t)
            self.link._collect_rudder_test_row_locked(yaw, heading, self.clock.t)

    def _run_to_completion(self, dt=1.0 / 15.0, frame_every=3):
        """Drive the sequence at the real 15 Hz tick with ~5 Hz telemetry."""
        if self.link.rudder_test is None:
            ok, err = self._start(seq=self.link.winch_command_seq + 1)
            self.assertTrue(ok, err)
        i = 0
        while self.link.rudder_test is not None:
            self.clock.advance(dt)
            self._boat_armed(fresh=True)
            self._tick()
            i += 1
            if i % frame_every == 0 and self.link.rudder_test is not None:
                self._frame()
            if i > 2000:
                self.fail('sequence never terminated')
        self.link._flush_rudder_test_write()
        return self.link.rudder_test_result


class PreconditionTest(RudderTestBase):

    def test_it_starts_when_every_gate_is_satisfied(self):
        ok, err = self._start()
        self.assertTrue(ok, err)
        self.assertIsNotNone(self.link.rudder_test)

    def test_it_refuses_when_disconnected(self):
        self.link.connected = False
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('disconnect', err.lower())

    def test_it_refuses_when_the_throttle_is_not_at_zero(self):
        """The sequence owns the throttle. Starting on top of a held stick
        would yank it to zero and then to 0.20, which is not the test."""
        self.link.throttle = 0.15
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('throttle', err.lower())

    def test_it_refuses_when_the_unlinked_per_motor_sliders_are_up(self):
        self.link.motor_split = True
        self.link.motor_left = 0.2
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('throttle', err.lower())

    def test_it_refuses_when_arm_was_never_commanded(self):
        self.link.armed_cmd = False
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('arm', err.lower())

    def test_it_never_arms_the_boat_itself(self):
        """Arming spins thrusters. That decision stays with the operator."""
        self.link.armed_cmd = False
        self._start()
        self.assertFalse(self.link.armed_cmd)
        self.assertEqual(self.writes, [])

    def test_it_refuses_when_the_boat_has_not_confirmed_armed(self):
        self._boat_armed(fresh=True, state=0)      # commanded, not confirmed
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('boat', err.lower())

    def test_it_refuses_on_a_stale_motor_status(self):
        """A minute-old 'armed' is not confirmation of anything."""
        self._boat_armed(fresh=False, state=2)
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('stale', err.lower())

    def test_it_refuses_with_no_motor_status_at_all(self):
        self.link.motor_status = T.BoatLink._blank_motor_status()
        ok, err = self._start()
        self.assertFalse(ok)

    def test_it_refuses_while_p_assist_is_requested_on(self):
        """P perturbs the motors mid-turn, which is exactly what this run must
        not contain."""
        self.link.p_assist_on = True
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('p assist', err.lower())

    def test_it_refuses_while_the_boat_reports_p_assist_on(self):
        """Requested-off is not enough: the BOAT's own state is what matters,
        and the two can disagree."""
        self.link.bench_status = dict(self.link.bench_status, have=True,
                                      p_on=True,
                                      last_rx_monotonic=self.clock.t)
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('p assist', err.lower())

    def test_it_refuses_during_calibration(self):
        self.link.calibrating = True
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('calibrat', err.lower())

    def test_it_refuses_while_a_bench_run_is_going(self):
        """_bench_running_locked() deliberately still reads the real clock --
        it is pre-existing shared code and this feature had no business
        re-plumbing it -- so this one field needs a real timestamp."""
        import time as _time
        self.link.bench_status = dict(self.link.bench_status, have=True,
                                      state=2,
                                      last_rx_monotonic=_time.monotonic())
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('bench', err.lower())

    def test_it_refuses_a_second_concurrent_rudder_test(self):
        self.assertTrue(self._start(seq=1)[0])
        ok, err = self._start(seq=2)
        self.assertFalse(ok)
        self.assertIn('already', err.lower())

    def test_it_refuses_a_stale_command_sequence(self):
        self.assertTrue(self._start(seq=5)[0])
        self.link.rudder_test = None
        ok, err = self._start(seq=5)
        self.assertFalse(ok)
        self.assertIn('stale', err.lower())

    def test_only_the_two_documented_directions_are_accepted(self):
        for bad in (0, 2, -2, 'left', None, 0.5):
            ok, _ = self.link.start_rudder_test(bad, self.link.winch_command_seq + 1)
            self.assertFalse(ok, 'direction %r should be refused' % (bad,))


class SequenceTest(RudderTestBase):

    def test_phase_order_and_boundaries_are_exact(self):
        """0.5 s settle, 3.0 s drive, 1.0 s coast -- measured from t0, not
        accumulated per tick, so tick jitter cannot drift the boundaries."""
        self._start(sign=-1)
        t0 = self.clock.t
        seen = []
        for i in range(300):
            self.clock.t = t0 + i * 0.02      # absolute: no accumulated drift
            self._boat_armed(fresh=True)
            self._tick()
            if self.link.rudder_test is None:
                break
            seen.append((round(i * 0.02, 4),
                         self.link.rudder_test['phase'],
                         self.link.throttle, self.link.rudder))

        def phase_at(el):
            return [p for (t, p, _th, _r) in seen if abs(t - el) < 1e-9][0]

        self.assertEqual(phase_at(0.00), 'rudder_settle')
        self.assertEqual(phase_at(0.48), 'rudder_settle')
        self.assertEqual(phase_at(0.50), 'drive')
        self.assertEqual(phase_at(3.48), 'drive')
        self.assertEqual(phase_at(3.50), 'coast')
        self.assertEqual(phase_at(4.48), 'coast')

    def test_the_drive_phase_is_exactly_three_seconds_of_commanded_throttle(self):
        self._start(sign=-1)
        t0 = self.clock.t
        first = last = None
        for i in range(300):
            self.clock.t = t0 + i * 0.02      # absolute: no accumulated drift
            self._boat_armed(fresh=True)
            self._tick()
            if self.link.rudder_test is None:
                break
            if self.link.throttle == T.RUDDER_TEST_THROTTLE:
                el = round(i * 0.02, 4)
                first = el if first is None else first
                last = el
        self.assertAlmostEqual(first, 0.50, places=6)
        self.assertAlmostEqual(last, 3.48, places=6)   # last tick before 3.50

    def test_the_rudder_is_held_through_settle_drive_and_coast(self):
        for sign, want in ((-1, -T.RUDDER_TEST_DEFLECTION),
                           (+1, +T.RUDDER_TEST_DEFLECTION)):
            self.setUp()
            self._start(sign=sign)
            held = set()
            while self.link.rudder_test is not None:
                self._tick()
                if self.link.rudder_test is None:
                    break
                held.add((self.link.rudder_test['phase'], self.link.rudder))
                self.clock.advance(0.05)
                self._boat_armed(fresh=True)
            for phase in ('rudder_settle', 'drive', 'coast'):
                self.assertIn((phase, want), held,
                              '%s did not hold rudder %.2f' % (phase, want))

    def test_throttle_is_zero_outside_the_drive_phase(self):
        self._start(sign=+1)
        while self.link.rudder_test is not None:
            self._tick()
            if self.link.rudder_test is None:
                break
            if self.link.rudder_test['phase'] != 'drive':
                self.assertEqual(self.link.throttle, 0.0)
            self.clock.advance(0.05)
            self._boat_armed(fresh=True)

    def test_it_finishes_centred_and_stopped(self):
        self._run_to_completion()
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)
        self.assertIsNone(self.link.rudder_test)

    def test_the_sequence_uses_the_linked_throttle_not_the_split_pair(self):
        """Split commands would bypass the boat's own trim mixing."""
        self._start(sign=-1)
        self.clock.advance(1.0)
        self._tick()
        self.assertFalse(self.link.motor_split)
        self.assertEqual(self.link.throttle, T.RUDDER_TEST_THROTTLE)

    def test_a_completed_result_is_not_marked_aborted(self):
        res = self._run_to_completion()
        self.assertIsNotNone(res)
        self.assertFalse(res['aborted'])
        self.assertAlmostEqual(res['duration_s'], 4.5, delta=0.1)


class RowCaptureTest(RudderTestBase):

    def test_one_row_per_telemetry_frame_not_per_tick(self):
        self._start(sign=-1)
        for _ in range(30):          # 30 ticks
            self.clock.advance(0.02)
            self._boat_armed(fresh=True)
            self._tick()
        self.assertEqual(len(self.link.rudder_test['rows']), 0,
                         'ticks alone must not produce rows')
        for _ in range(4):           # 4 frames
            self.clock.advance(0.05)
            self._frame()
        self.assertEqual(len(self.link.rudder_test['rows']), 4)

    def test_repeated_status_polls_add_no_rows(self):
        """status() is called by every browser poll. It must be read-only."""
        self._start(sign=-1)
        self.clock.advance(0.05)
        self._frame()
        before = len(self.link.rudder_test['rows'])
        for _ in range(20):
            self.link.status()
        self.assertEqual(len(self.link.rudder_test['rows']), before)

    def test_rows_carry_the_phase_they_were_captured_in(self):
        self._start(sign=-1)
        phases = set()
        for _ in range(120):
            self.clock.advance(0.05)
            self._boat_armed(fresh=True)
            self._tick()
            if self.link.rudder_test is None:
                break
            self._frame()
            phases.add(self.link.rudder_test['rows'][-1]['phase'])
        self.assertEqual(phases, {'rudder_settle', 'drive', 'coast'})

    def test_a_row_records_command_and_boat_confirmed_state(self):
        self._start(sign=+1)
        self.clock.advance(1.0)
        self._boat_armed(fresh=True)
        self._tick()
        self._frame(yaw=-12.5, heading=42.0)
        row = self.link.rudder_test['rows'][-1]
        self.assertAlmostEqual(row['yaw_dps'], -12.5)
        self.assertAlmostEqual(row['heading_deg'], 42.0)
        self.assertAlmostEqual(row['cmd_throttle'], T.RUDDER_TEST_THROTTLE)
        self.assertAlmostEqual(row['cmd_rudder'], +T.RUDDER_TEST_DEFLECTION)
        self.assertAlmostEqual(row['boat_left'], 0.18)
        self.assertAlmostEqual(row['boat_right'], 0.22)
        self.assertEqual(row['phase'], 'drive')
        self.assertIn('t_mono', row)
        self.assertIn('elapsed_s', row)
        self.assertIn('telem_age_s', row)

    def test_the_signed_yaw_is_stored_raw(self):
        self._start(sign=-1)
        for yaw in (-12.5, 0.0, +7.25):
            self.clock.advance(0.05)
            self._frame(yaw=yaw)
            self.assertAlmostEqual(self.link.rudder_test['rows'][-1]['yaw_dps'], yaw)

    def test_no_rows_are_captured_once_the_sequence_has_ended(self):
        self._run_to_completion()
        self.clock.advance(0.05)
        self._frame()          # must be a no-op, not a crash
        self.assertIsNone(self.link.rudder_test)


class GapReportingTest(RudderTestBase):

    def test_a_clean_run_reports_no_gap(self):
        res = self._run_to_completion(frame_every=3)   # ~5 Hz
        self.assertFalse(res['gap'])
        self.assertGreater(res['frames'], 10)

    def test_a_dropout_is_reported(self):
        self._start(sign=-1)
        for _ in range(10):
            self.clock.advance(0.05)
            self._boat_armed(fresh=True)
            self._tick()
            self._frame()
        self.clock.advance(1.5)                  # radio hole
        self._boat_armed(fresh=True)
        self._tick()
        self._frame()
        while self.link.rudder_test is not None:
            self.clock.advance(0.05)
            self._boat_armed(fresh=True)
            self._tick()
            if self.link.rudder_test is not None:
                self._frame()
        self.link._flush_rudder_test_write()
        res = self.link.rudder_test_result
        self.assertTrue(res['gap'])
        self.assertGreater(res['max_gap_s'], 1.0)

    def test_a_run_with_no_telemetry_at_all_is_reported_not_crashed(self):
        self._start(sign=-1)
        while self.link.rudder_test is not None:
            self.clock.advance(0.05)
            self._boat_armed(fresh=True)
            self._tick()
        self.link._flush_rudder_test_write()
        res = self.link.rudder_test_result
        self.assertEqual(res['frames'], 0)
        self.assertTrue(res['gap'])


class AbortTest(RudderTestBase):
    """Every route out must stop the boat AND centre the rudder."""

    def _assert_safe_and_aborted(self, reason_fragment=None):
        self.assertIsNone(self.link.rudder_test)
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)
        self.link._flush_rudder_test_write()
        res = self.link.rudder_test_result
        self.assertIsNotNone(res)
        self.assertTrue(res['aborted'])
        if reason_fragment:
            self.assertIn(reason_fragment, res['abort_reason'].lower())

    def _mid_drive(self):
        self._start(sign=-1)
        self.clock.advance(1.0)
        self._boat_armed(fresh=True)
        self._tick()
        self._frame()
        self.assertEqual(self.link.rudder_test['phase'], 'drive')

    def test_stop_aborts(self):
        self._mid_drive()
        self.link.stop(self.link.winch_command_seq + 1)
        self._assert_safe_and_aborted('stop')

    def test_disarm_aborts(self):
        self._mid_drive()
        self.link.arm(False, False)
        self._assert_safe_and_aborted('disarm')

    def test_disconnect_aborts(self):
        self._mid_drive()
        self.link.disconnect()
        self._assert_safe_and_aborted('disconnect')

    def test_a_dropped_link_mid_run_aborts_on_the_next_tick(self):
        self._mid_drive()
        self.link.connected = False
        self.clock.advance(0.05)
        self._tick()
        self._assert_safe_and_aborted()

    def test_the_boat_going_disarmed_mid_run_aborts(self):
        self._mid_drive()
        self._boat_armed(fresh=True, state=0)
        self.clock.advance(0.05)
        self._tick()
        self._assert_safe_and_aborted('armed')

    def test_a_stale_boat_status_mid_run_aborts(self):
        self._mid_drive()
        self._boat_armed(fresh=False, state=2)
        self.clock.advance(0.05)
        self._tick()
        self._assert_safe_and_aborted('stale')

    def test_calibration_starting_mid_run_aborts(self):
        self._mid_drive()
        self.link.calibrating = True
        self.clock.advance(0.05)
        self._tick()
        self._assert_safe_and_aborted('calibrat')

    def test_a_serial_write_failure_aborts(self):
        """Drives the real function the stream loop calls with its send result,
        rather than re-implementing the loop's logic here."""
        self._mid_drive()
        with self.link._lock:
            self.link._rudder_test_after_send_locked(False)
        self._assert_safe_and_aborted('write')

    def test_an_exception_in_the_tick_aborts_rather_than_leaving_it_running(self):
        self._mid_drive()
        self.link.motor_status = 'not a dict'      # force an internal error
        self.clock.advance(0.05)
        self._tick()
        self._assert_safe_and_aborted()

    def test_shutdown_aborts(self):
        self._mid_drive()
        self.link._stop = threading.Event()
        self.link.shutdown()
        self._assert_safe_and_aborted()

    def test_an_aborted_run_still_writes_its_partial_csv(self):
        self._mid_drive()
        self.link.stop(self.link.winch_command_seq + 1)
        self.link._flush_rudder_test_write()
        res = self.link.rudder_test_result
        self.assertTrue(Path(res['path']).exists())
        self.assertTrue(res['aborted'])


class CsvFileTest(RudderTestBase):

    def test_the_name_identifies_throttle_and_direction(self):
        for sign, tag in ((-1, 'N80'), (+1, 'P80')):
            self.setUp()
            self._start(sign=sign)
            name = self.link.rudder_test['name']
            self.assertIn('T20', name)
            self.assertIn(tag, name)
            self.assertTrue(name.upper().endswith('.CSV'))

    def test_each_run_gets_its_own_file(self):
        names = []
        for i in range(3):
            self._run_to_completion()
            names.append(self.link.rudder_test_result['path'])
            self.link.rudder_test = None
            self.link.rudder_test_result = None
            self.link.throttle = 0.0
            self.link.rudder = 0.0
            self.link.winch_command_seq += 1
            self._boat_armed(fresh=True)
            self.assertTrue(self._start(seq=self.link.winch_command_seq + 1)[0])
            self.link.winch_command_seq += 1
        self.assertEqual(len(set(names)), len(names))
        for n in names:
            self.assertTrue(Path(n).exists())

    def test_the_csv_has_a_header_and_one_line_per_frame(self):
        res = self._run_to_completion(frame_every=3)
        rows = read_rudder_csv(res['path'])
        self.assertEqual(len(rows), res['frames'])
        for col in ('t_mono', 'elapsed_s', 'phase', 'yaw_dps', 'heading_deg',
                    'cmd_throttle', 'cmd_rudder', 'boat_left', 'boat_right',
                    'servo_us', 'telem_age_s', 'gap_s'):
            self.assertIn(col, rows[0])

    def test_the_file_says_it_is_laptop_observed(self):
        """It must never be mistaken for the boat's own 100 Hz SD recording."""
        res = self._run_to_completion()
        head = Path(res['path']).read_text()[:600].lower()
        self.assertIn('laptop', head)
        self.assertNotIn('authoritative record', head)

    def test_servo_position_is_left_empty_because_the_boat_never_sends_it(self):
        """MotorStatus carries state/throttles/winch/servo_power -- no rudder
        position or pulse. The column exists so the schema is stable if the
        firmware ever adds one; it must not be filled with the COMMAND and
        passed off as a measurement."""
        self.assertNotIn('servo_us', T.BoatLink._blank_motor_status())
        res = self._run_to_completion()
        rows = read_rudder_csv(res['path'])
        self.assertTrue(all(r['servo_us'] == '' for r in rows))


class UnchangedBenchBehaviourTest(unittest.TestCase):
    """The rudder test must not disturb what already works."""

    def setUp(self):
        self.src = TOOL.read_text()

    def test_bench_kinds_are_untouched(self):
        self.assertEqual(T.BENCH_KIND, {'both': 0, 'left': 1, 'right': 2})
        self.assertEqual(T.BENCH_KIND_NAME, {0: 'BASE', 1: 'LEFT', 2: 'RIGHT'})

    def test_bench_buttons_still_send_the_same_three_kinds(self):
        for kind in ("'left'", "'right'", "'both'"):
            self.assertIn('runBench(%s, 0)' % kind, self.src)

    def test_bench_motor_strength_labels_are_unchanged(self):
        self.assertIn('>LEFT MOTOR STRONGER<', self.src)
        self.assertIn('>RIGHT MOTOR STRONGER<', self.src)

    def test_the_bench_effective_split_maths_is_unchanged(self):
        e = T.bench_effective_commands('left', 0.20, 0.12, 0.17)
        self.assertAlmostEqual(e['differential'], -0.172, places=6)
        e = T.bench_effective_commands('right', 0.20, 0.12, 0.17)
        self.assertAlmostEqual(e['differential'], 0.308, places=6)

    def test_the_bench_defaults_are_unchanged(self):
        import re
        m = re.search(r'<input type="number" id="bench-delta"[^>]*>', self.src)
        self.assertIn('value="12"', m.group(0))
        m = re.search(r'<input type="number" id="bench-throttle"[^>]*>', self.src)
        self.assertIn('value="20"', m.group(0))

    def test_the_deflection_is_big_enough_to_read(self):
        """0.30 produced too little turn to separate from the boat's own yaw
        noise. Not 1.00 either -- that parks the servo on its stop for 4.5 s."""
        self.assertGreaterEqual(T.RUDDER_TEST_DEFLECTION, 0.5)
        self.assertLess(T.RUDDER_TEST_DEFLECTION, 1.0)
        self.assertEqual(T.RUDDER_TEST_PCT,
                         round(T.RUDDER_TEST_DEFLECTION * 100))

    def test_every_label_and_filename_follows_the_constant(self):
        """It was hardcoded in six places before this guard existed: the two
        filename tags, both button labels and both status pills. Changing the
        constant must not be able to leave a stale number anywhere."""
        pct = T.RUDDER_TEST_PCT
        self.assertIn('RUDDER &minus;%d TEST' % pct, self.src)
        self.assertIn('RUDDER +%d TEST' % pct, self.src)
        self.assertIn('hold rudder -0.%d ' % pct, self.src)
        self.assertIn('hold rudder +0.%d ' % pct, self.src)
        # the pills read the server's value rather than a literal
        self.assertIn("rt.pct", self.src)
        self.assertIn("rtr.pct", self.src)
        for stale in ('N30', 'P30', "'-30", "'+30", 'minus;30 TEST'):
            self.assertNotIn(stale, self.src,
                             'a stale 30 survived: %s' % stale)

    def test_no_firmware_proto_or_dashboard_dependency_was_added(self):
        """The sequence is built entirely from commands the boat already
        understands. A new protobuf message would mean firmware + regenerated
        bindings + a reflash, which this work explicitly excludes.

        Checked as protobuf USAGE, not as a bare substring: 'RudderTest(' also
        occurs inside the JS helper runRudderTest(), which is not a proto type.
        The first version of this assertion failed on exactly that."""
        self.assertNotIn('msg.rudder_test', self.src)
        self.assertNotIn('pb2.RudderTest', self.src)
        self.assertNotIn('self.pb2.RudderTest', self.src)
        # every command it sends is one of the pre-existing ones
        for existing in ('_send_motor_locked', '_send_steer_locked'):
            self.assertIn(existing, self.src)

    def test_the_sequence_reuses_the_existing_command_stream(self):
        """It sets throttle/rudder and lets the 15 Hz loop send them. A private
        write path would sidestep STOP, disconnect and the calibration gate."""
        self.assertIn('self._rudder_test_tick_locked(', self.src)
        i = self.src.index('def _stream_loop')
        self.assertIn('_rudder_test_tick_locked', self.src[i:i + 1800])


if __name__ == '__main__':
    unittest.main()
