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
        link.assist_rudder_on = False
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

    def _boat_armed(self, fresh=True, state=2, servo_power=True):
        self.link.motor_status = dict(
            self.link.motor_status, have=True, state=state,
            left_throttle=0.18, right_throttle=0.22, servo_power=servo_power,
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


class ServoPowerGateTest(RudderTestBase):
    """The rudder is a SERVO. With the rail unpowered it does not hold a
    deflection, so a run without it is throttle applied to a rudder that may be
    drifting anywhere -- worse than no test, because it still produces a file."""

    def test_it_refuses_to_start_with_the_rail_off(self):
        self._boat_armed(fresh=True, servo_power=False)
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('servo rail', err.lower())

    def test_the_rail_going_off_mid_run_aborts(self):
        self._start()
        self.clock.advance(1.0)
        self._boat_armed(fresh=True, servo_power=False)
        self._tick()
        self.assertIsNone(self.link.rudder_test)
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)
        self.link._flush_rudder_test_write()
        self.assertTrue(self.link.rudder_test_result['aborted'])
        self.assertIn('servo rail',
                      self.link.rudder_test_result['abort_reason'].lower())

    def test_status_going_stale_also_loses_the_rail_confirmation(self):
        """Freshness and rail state are one gate: a stale 'powered' is not
        evidence the rail is still up."""
        self._start()
        self.clock.advance(1.0)
        self._boat_armed(fresh=False, servo_power=True)
        self._tick()
        self.assertIsNone(self.link.rudder_test)
        self.assertEqual(self.link.rudder, 0.0)


class ServerSideInterlockTest(RudderTestBase):
    """Refusals live on the SERVER. The page greys these out, but a stale tab,
    a second browser or a curl reaches the HTTP API without seeing that."""

    def setUp(self):
        super().setUp()
        self.assertTrue(self._start()[0])
        self._seq = self.link.winch_command_seq

    def _next(self):
        self._seq += 1
        return self._seq

    def test_bench_start_is_refused(self):
        ok, err = self.link.send_bench('both', 0.2, 0.12, self._next())
        self.assertFalse(ok)
        self.assertIn('rudder test', err.lower())
        self.assertIsNotNone(self.link.rudder_test)   # and does NOT abort it

    def test_calibration_start_is_refused(self):
        ok, err = self.link.set_calibrate(True, self._next())
        self.assertFalse(ok)
        self.assertIn('rudder test', err.lower())
        self.assertIsNotNone(self.link.rudder_test)

    def test_a_p_assist_change_is_refused(self):
        ok, err = self.link.send_assist(True)
        self.assertFalse(ok)
        self.assertIn('rudder test', err.lower())
        self.assertFalse(self.link.p_assist_on)

    def test_winch_is_refused(self):
        ok, err = self.link.set_winch(0.5, self._next())
        self.assertFalse(ok)
        self.assertIn('rudder test', err.lower())
        self.assertEqual(self.link.winch_speed, 0.0)

    def test_a_second_rudder_test_is_refused(self):
        ok, err = self.link.start_rudder_test(1, self._next())
        self.assertFalse(ok)
        self.assertIn('already', err.lower())

    def test_the_refusals_do_not_disturb_the_running_test(self):
        """Refused means refused -- not 'quietly ends the run'."""
        phase_before = self.link.rudder_test['phase']
        self.link.send_bench('both', 0.2, 0.12, self._next())
        self.link.set_calibrate(True, self._next())
        self.link.send_assist(True)
        self.link.set_winch(0.5, self._next())
        self.assertIsNotNone(self.link.rudder_test)
        self.assertEqual(self.link.rudder_test['phase'], phase_before)
        self.assertEqual(self.link.rudder, -T.RUDDER_TEST_DEFLECTION)

    def test_stop_disarm_and_disconnect_stay_active(self):
        """These must NEVER be interlocked -- they are the way out."""
        self.link.stop(self._next())
        self.assertIsNone(self.link.rudder_test)
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)


class ManualOverrideTest(RudderTestBase):
    """Touching a stick means the operator wants control back."""

    def setUp(self):
        super().setUp()
        self.assertTrue(self._start()[0])
        self.clock.advance(1.0)
        self._tick()

    def _assert_aborted_and_safe(self):
        self.assertIsNone(self.link.rudder_test)
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)
        self.assertEqual(self.link.motor_left, 0.0)
        self.assertEqual(self.link.motor_right, 0.0)
        self.link._flush_rudder_test_write()
        self.assertTrue(self.link.rudder_test_result['aborted'])
        self.assertIn('manual',
                      self.link.rudder_test_result['abort_reason'].lower())

    def test_a_throttle_input_aborts_and_the_value_is_discarded(self):
        """Discarded, not applied: the abort has already forced throttle 0, and
        letting the half-pushed slider through would immediately undo that."""
        self.link.set_state(throttle=0.6)
        self._assert_aborted_and_safe()

    def test_a_rudder_input_aborts_and_is_discarded(self):
        self.link.set_state(rudder=0.9)
        self._assert_aborted_and_safe()

    def test_a_per_motor_input_aborts_and_is_discarded(self):
        self.link.set_state(left=0.5)
        self._assert_aborted_and_safe()

    def test_a_zero_input_still_aborts(self):
        """Releasing a stick is just as much 'I want control back'."""
        self.link.set_state(throttle=0.0)
        self._assert_aborted_and_safe()

    def test_manual_control_works_normally_once_the_run_has_ended(self):
        self.link.set_state(throttle=0.6)          # aborts
        self.link.set_state(throttle=0.6)          # now it applies
        self.assertAlmostEqual(self.link.throttle, 0.6)

    def test_servo_power_off_aborts_the_run(self):
        self.link.set_servo_power(False, self.link.winch_command_seq + 1)
        self.assertIsNone(self.link.rudder_test)
        self.assertEqual(self.link.rudder, 0.0)
        self.link._flush_rudder_test_write()
        self.assertTrue(self.link.rudder_test_result['aborted'])


class DriveCoverageTest(unittest.TestCase):
    """Frame COUNT is not coverage. Sixty frames crammed into the last second,
    or a recording that only starts 1.2 s in, both look healthy by count and
    describe most of the turn not at all."""

    @staticmethod
    def _rows(times):
        return [{'elapsed_s': t} for t in times]

    def _clean(self, hz=20.0):
        step = 1.0 / hz
        n = int(T.RUDDER_TEST_DRIVE_S / step)
        return self._rows([0.5 + i * step for i in range(n)])

    def test_a_clean_drive_is_complete(self):
        c = T.rudder_test_drive_coverage(self._clean())
        self.assertFalse(c['drive_incomplete'])
        self.assertGreaterEqual(c['drive_frames'], T.RUDDER_TEST_MIN_DRIVE_FRAMES)
        self.assertAlmostEqual(c['drive_first_delay_s'], 0.0, places=3)
        self.assertLessEqual(c['drive_tail_gap_s'], T.RUDDER_TEST_MAX_GAP_S)
        self.assertLessEqual(c['drive_max_gap_s'], T.RUDDER_TEST_MAX_GAP_S)

    def test_only_frames_inside_the_drive_window_are_counted(self):
        """Settle- and coast-phase frames are context, not measurement."""
        rows = self._rows([0.0, 0.2, 0.4] + [4.0, 4.2]) + self._clean()
        rows.sort(key=lambda r: r['elapsed_s'])
        c = T.rudder_test_drive_coverage(rows)
        self.assertEqual(c['drive_frames'], len(self._clean()))

    def test_a_late_start_is_incomplete_even_with_many_frames(self):
        rows = self._rows([1.9 + i * 0.02 for i in range(70)])   # dense, late
        c = T.rudder_test_drive_coverage(rows)
        self.assertGreaterEqual(c['drive_frames'], T.RUDDER_TEST_MIN_DRIVE_FRAMES)
        self.assertGreater(c['drive_first_delay_s'], T.RUDDER_TEST_MAX_GAP_S)
        self.assertTrue(c['drive_incomplete'])

    def test_an_early_stop_is_incomplete(self):
        rows = self._rows([0.5 + i * 0.02 for i in range(70)])   # dense, early
        c = T.rudder_test_drive_coverage(rows)
        self.assertGreater(c['drive_tail_gap_s'], T.RUDDER_TEST_MAX_GAP_S)
        self.assertTrue(c['drive_incomplete'])

    def test_a_hole_in_the_middle_is_incomplete(self):
        rows = self._rows([0.5 + i * 0.05 for i in range(15)]
                          + [2.0 + i * 0.05 for i in range(28)])
        c = T.rudder_test_drive_coverage(rows)
        self.assertGreater(c['drive_max_gap_s'], T.RUDDER_TEST_MAX_GAP_S)
        self.assertTrue(c['drive_incomplete'])

    def test_the_gap_limit_is_three_tenths_of_a_second(self):
        self.assertAlmostEqual(T.RUDDER_TEST_MAX_GAP_S, 0.30, places=6)

    def test_the_coverage_thresholds_are_pinned(self):
        """These decide whether a run counts as data, so a quiet loosening
        would silently turn thin recordings into confident ones. Nothing else
        catches it: relaxing either still leaves the maths self-consistent."""
        self.assertEqual(T.RUDDER_TEST_MIN_DRIVE_FRAMES, 20)
        self.assertAlmostEqual(T.RUDDER_TEST_MIN_DRIVE_COVERAGE, 0.8, places=6)
        self.assertAlmostEqual(T.RUDDER_TEST_DRIVE_S, 3.0, places=6)

    def test_too_few_frames_ALONE_is_enough_to_be_incomplete(self):
        """Isolates the frame count: 19 frames spanning the whole 3 s window
        with 0.16 s gaps. Span, lead-in and tail are all fine, so only the
        count can fail it."""
        rows = self._rows([0.5 + i * (2.95 / 18.0) for i in range(19)])
        c = T.rudder_test_drive_coverage(rows)
        self.assertEqual(c['drive_frames'], 19)
        self.assertLess(c['drive_frames'], T.RUDDER_TEST_MIN_DRIVE_FRAMES)
        self.assertGreaterEqual(
            c['drive_span_s'], T.RUDDER_TEST_DRIVE_S * T.RUDDER_TEST_MIN_DRIVE_COVERAGE)
        self.assertLessEqual(c['drive_max_gap_s'], T.RUDDER_TEST_MAX_GAP_S)
        self.assertTrue(c['drive_incomplete'],
                        '19 frames across a 3 s turn is too thin to trust')

    def test_a_gap_just_over_the_limit_trips_it(self):
        rows = self._rows([0.5 + i * 0.05 for i in range(20)]
                          + [1.81 + i * 0.05 for i in range(34)])
        c = T.rudder_test_drive_coverage(rows)
        self.assertTrue(c['drive_incomplete'])

    def test_a_late_start_ALONE_is_enough_to_be_incomplete(self):
        """Isolates the first-frame delay. Span and frame count are both fine
        here -- 2.6 s of the 3 s window, 53 frames, 0.05 s gaps -- so only the
        0.4 s lead-in can fail it. The earlier late-start test also failed on
        span, which let a mutation that ignored the lead-in survive."""
        rows = self._rows([0.9 + i * 0.05 for i in range(53)])   # 0.90 .. 3.50
        c = T.rudder_test_drive_coverage(rows)
        self.assertGreaterEqual(c['drive_frames'], T.RUDDER_TEST_MIN_DRIVE_FRAMES)
        self.assertGreaterEqual(
            c['drive_span_s'], T.RUDDER_TEST_DRIVE_S * T.RUDDER_TEST_MIN_DRIVE_COVERAGE)
        self.assertAlmostEqual(c['drive_first_delay_s'], 0.4, places=3)
        self.assertLessEqual(c['drive_tail_gap_s'], T.RUDDER_TEST_MAX_GAP_S)
        self.assertTrue(c['drive_incomplete'],
                        'a 0.4 s lead-in hides the start of the turn')

    def test_a_tail_gap_ALONE_is_enough_to_be_incomplete(self):
        """Mirror image: the recording stops 0.4 s before the drive window
        closes, with span and count otherwise fine."""
        rows = self._rows([0.5 + i * 0.05 for i in range(53)])   # 0.50 .. 3.10
        c = T.rudder_test_drive_coverage(rows)
        self.assertGreaterEqual(c['drive_frames'], T.RUDDER_TEST_MIN_DRIVE_FRAMES)
        self.assertGreaterEqual(
            c['drive_span_s'], T.RUDDER_TEST_DRIVE_S * T.RUDDER_TEST_MIN_DRIVE_COVERAGE)
        self.assertAlmostEqual(c['drive_first_delay_s'], 0.0, places=3)
        self.assertAlmostEqual(c['drive_tail_gap_s'], 0.4, places=3)
        self.assertTrue(c['drive_incomplete'],
                        'a 0.4 s tail hides the end of the turn')

    def test_no_frames_at_all_is_incomplete(self):
        c = T.rudder_test_drive_coverage([])
        self.assertEqual(c['drive_frames'], 0)
        self.assertTrue(c['drive_incomplete'])


class ResultIncompleteTest(RudderTestBase):

    def test_a_clean_run_is_marked_complete(self):
        res = self._run_to_completion(frame_every=1)   # ~15 Hz
        self.assertFalse(res['aborted'])
        self.assertFalse(res['incomplete'])
        self.assertGreater(res['drive_frames'], 0)

    def test_a_sparse_run_is_marked_incomplete(self):
        res = self._run_to_completion(frame_every=20)  # ~0.75 Hz
        self.assertTrue(res['incomplete'])

    def test_an_aborted_run_is_always_incomplete(self):
        self._start()
        self.clock.advance(1.0)
        self._tick()
        self._frame()
        self.link.stop(self.link.winch_command_seq + 1)
        self.link._flush_rudder_test_write()
        res = self.link.rudder_test_result
        self.assertTrue(res['aborted'])
        self.assertTrue(res['incomplete'])

    def test_an_abort_during_COAST_is_still_incomplete(self):
        """The sharp case: the drive phase ran to completion with dense frames,
        so the coverage numbers alone look perfect -- and the run was still cut
        short. An aborted run is not a completed test, and must not print as
        one. The earlier abort test aborted mid-drive, where coverage failed
        anyway, which let a mutation dropping the abort term survive."""
        self._start()
        t0 = self.clock.t
        for i in range(1, 73):                      # through settle + all of drive
            self.clock.t = t0 + i * 0.05
            self._boat_armed(fresh=True)
            self._tick()
            if self.link.rudder_test is None:
                break
            self._frame()
        self.assertIsNotNone(self.link.rudder_test)
        self.assertEqual(self.link.rudder_test['phase'], 'coast')

        # Coverage of the DRIVE window is genuinely good at this point...
        cov = T.rudder_test_drive_coverage(self.link.rudder_test['rows'])
        self.assertFalse(cov['drive_incomplete'])

        self.link.stop(self.link.winch_command_seq + 1)
        self.link._flush_rudder_test_write()
        res = self.link.rudder_test_result
        self.assertTrue(res['aborted'])
        self.assertFalse(res['drive_incomplete'])   # ...the drive data is fine
        self.assertTrue(res['incomplete'],          # ...and it is STILL not a
                        'an aborted run must never report as complete')

    def test_the_csv_records_the_coverage_verdict(self):
        res = self._run_to_completion(frame_every=20)
        head = Path(res['path']).read_text()[:900]
        self.assertIn('drive_frames=', head)
        self.assertIn('first_delay_s=', head)
        self.assertIn('tail_gap_s=', head)
        self.assertIn('INCOMPLETE', head)


class UiLockoutTest(unittest.TestCase):
    """Cosmetic only -- and the tests say so, so nobody later mistakes the
    greyed-out buttons for the safety mechanism."""

    def setUp(self):
        self.src = TOOL.read_text()

    def test_the_conflicting_controls_are_greyed_out_while_running(self):
        self.assertIn('function setRudderTestLockout(on)', self.src)
        for control in ('bench-left', 'bench-right', 'bench-base', 'bench-reset',
                        'p-assist', 'rt-minus', 'rt-plus', 'calibrate-btn'):
            self.assertIn("'%s'" % control, self.src)
        self.assertIn('setRudderTestLockout(true)', self.src)
        self.assertIn('setRudderTestLockout(false)', self.src)

    def test_every_locked_out_control_is_also_refused_server_side(self):
        """The point of the whole exercise: disabling is decoration."""
        self.assertEqual(self.src.count('_rudder_test_busy_locked()'), 7)
        for fn in ('def send_bench', 'def set_calibrate', 'def send_assist',
                   'def set_winch', 'def set_state', 'def set_servo_power'):
            i = self.src.index(fn)
            self.assertIn('_rudder_test_busy_locked()', self.src[i:i + 1500],
                          '%s has no server-side interlock' % fn)

    def test_stop_and_disarm_are_never_interlocked(self):
        for fn in ('def stop(', 'def arm('):
            i = self.src.index(fn)
            body = self.src[i:i + 900]
            self.assertNotIn('return False, \'a rudder test is running', body)


class AssistedModeTest(RudderTestBase):
    """Assisted runs hand the boat a yaw-rate TARGET and let its own rudder
    loop chase it, instead of holding a fixed deflection."""

    def _assisted(self, sign=-1, seq=1):
        return self.link.start_rudder_test(sign, seq, assisted=True)

    def test_it_turns_the_rudder_loop_on_for_the_run(self):
        self.assertFalse(self.link.assist_rudder_on)
        self.assertTrue(self._assisted()[0])
        self.assertTrue(self.link.assist_rudder_on)

    def test_it_turns_the_loop_off_again_when_the_run_ends(self):
        """Leaving it live would have the boat quietly steering during
        ordinary manual driving afterwards."""
        self._assisted()
        self._run_to_completion()
        self.assertFalse(self.link.assist_rudder_on)

    def test_it_turns_the_loop_off_on_an_abort_too(self):
        self._assisted()
        self.clock.advance(1.0)
        self._tick()
        self.link.stop(self.link.winch_command_seq + 1)
        self.assertIsNone(self.link.rudder_test)
        self.assertFalse(self.link.assist_rudder_on)

    def test_left_commands_full_left_stick(self):
        """Stick -1 is LEFT, which the firmware turns into a POSITIVE yaw
        target -- physical left produces positive IMU yaw on this boat."""
        self._assisted(sign=-1)
        self.clock.advance(1.0)
        self._boat_armed(fresh=True)
        self._tick()
        self.assertEqual(self.link.rudder_test['phase'], 'drive')
        self.assertAlmostEqual(self.link.rudder, -1.0)
        self.assertAlmostEqual(self.link.rudder_test['target_dps'],
                               +T.ASSIST_TEST_TARGET_DPS)

    def test_right_commands_full_right_stick(self):
        self._assisted(sign=+1)
        self.clock.advance(1.0)
        self._boat_armed(fresh=True)
        self._tick()
        self.assertAlmostEqual(self.link.rudder, +1.0)
        self.assertAlmostEqual(self.link.rudder_test['target_dps'],
                               -T.ASSIST_TEST_TARGET_DPS)

    def test_the_target_is_zero_outside_the_drive_phase(self):
        """The settle phase is the loop holding straight before the motors come
        on, and the coast must not keep asking for a turn the jets can no
        longer produce -- that would wind the rudder over with no authority."""
        self._assisted(sign=-1)
        seen = {}
        for i in range(120):
            self.clock.advance(0.05)
            self._boat_armed(fresh=True)
            self._tick()
            if self.link.rudder_test is None:
                break
            seen.setdefault(self.link.rudder_test['phase'], set()).add(self.link.rudder)
        self.assertEqual(seen['rudder_settle'], {0.0})
        self.assertEqual(seen['coast'], {0.0})
        self.assertEqual(seen['drive'], {-1.0})

    def test_the_files_are_named_apart_from_the_raw_runs(self):
        """Different experiments; a glob must never pool them."""
        self._assisted(sign=-1)
        self.assertTrue(self.link.rudder_test['name'].startswith('ASST_T20_L2'))
        self.link.rudder_test = None
        self.link.winch_command_seq += 1
        self._assisted(sign=+1, seq=self.link.winch_command_seq + 1)
        self.assertTrue(self.link.rudder_test['name'].startswith('ASST_T20_R2'))
        self.link.rudder_test = None
        self.link.winch_command_seq += 1
        self.link.start_rudder_test(-1, self.link.winch_command_seq + 1)
        self.assertTrue(self.link.rudder_test['name'].startswith('RUD_T20_N80'))

    def test_the_rows_carry_the_mode_and_the_boat_reported_loop_state(self):
        self._assisted(sign=-1)
        self.clock.advance(1.0)
        self.link.motor_status = dict(self.link.motor_status,
                                      rudder_cmd=-0.42, rudder_pulse_us=1700,
                                      rudder_saturated=True, assist_rudder=True,
                                      yaw_filt_dps=1.25)
        self._boat_armed(fresh=True)
        self.link.motor_status = dict(self.link.motor_status,
                                      rudder_cmd=-0.42, rudder_pulse_us=1700,
                                      rudder_saturated=True, assist_rudder=True,
                                      yaw_filt_dps=1.25)
        self._tick()
        self._frame()
        row = self.link.rudder_test['rows'][-1]
        self.assertEqual(row['mode'], 'assisted')
        self.assertAlmostEqual(row['target_dps'], +T.ASSIST_TEST_TARGET_DPS)
        self.assertAlmostEqual(row['boat_rudder_cmd'], -0.42)
        self.assertEqual(row['boat_rudder_us'], 1700)
        self.assertEqual(row['boat_saturated'], 1)
        self.assertEqual(row['boat_assist_on'], 1)
        self.assertAlmostEqual(row['yaw_filt_dps'], 1.25)

    def test_a_raw_run_leaves_the_assisted_columns_blank(self):
        self.link.start_rudder_test(-1, 1)
        self.clock.advance(1.0)
        self._boat_armed(fresh=True)
        self._tick()
        self._frame()
        row = self.link.rudder_test['rows'][-1]
        self.assertEqual(row['mode'], 'raw')
        self.assertEqual(row['target_dps'], 0.0)

    def test_the_csv_says_the_rudder_is_commanded_not_measured(self):
        self._assisted()
        res = self._run_to_completion()
        rows = read_rudder_csv(res['path'])
        for col in ('mode', 'target_dps', 'yaw_filt_dps', 'boat_rudder_cmd',
                    'boat_rudder_us', 'boat_saturated', 'boat_assist_on'):
            self.assertIn(col, rows[0])
        head = Path(res['path']).read_text()[:600].lower()
        self.assertIn('laptop', head)


class AssistedSafetyOrderingTest(RudderTestBase):
    """Four defects found in independent review. Each is a real hazard, not a
    tidiness issue, so each gets a test that fails on the old behaviour."""

    def setUp(self):
        super().setUp()
        # Decode every AssistCommand / SteerCommand / MotorCommand this test
        # writes, in order, so the SEQUENCE can be asserted -- not just the
        # final state, which looks identical either way round.
        self.sent = []
        pb2 = self.link.pb2

        def record(payload):
            msg = pb2.BoatMessage()
            try:
                msg.ParseFromString(payload)
            except Exception:                      # noqa: BLE001
                return self.write_ok
            which = msg.WhichOneof('payload')
            if which == 'assist':
                self.sent.append(('assist', msg.assist.p_on,
                                  msg.assist.rudder_assist))
            elif which == 'steer':
                self.sent.append(('steer', round(msg.steer.left, 4)))
            elif which == 'motor':
                self.sent.append(('motor', round(msg.motor.left, 4),
                                  round(msg.motor.right, 4)))
            return self.write_ok
        self.link._write_locked = record

    # ---- 2. the settle phase must hold ZERO stick -----------------------

    def test_an_assisted_run_does_not_command_full_stick_before_the_first_tick(self):
        """start_rudder_test used to assign the drive-phase stick immediately,
        so the controller saw a 2 deg/s target through the whole settle phase
        -- which is neither the profile nor what gets recorded."""
        self.assertTrue(self.link.start_rudder_test(-1, 1, assisted=True)[0])
        self.assertEqual(self.link.rudder, 0.0)
        self.assertEqual(self.link.throttle, 0.0)

    def test_the_stick_stays_zero_for_the_whole_settle_phase(self):
        self.link.start_rudder_test(-1, 1, assisted=True)
        t0 = self.clock.t
        for i in range(1, 25):                      # 0.05 .. 1.20 s
            self.clock.t = t0 + i * 0.05
            self._boat_armed(fresh=True)
            self._tick()
            if self.link.rudder_test is None:
                break
            if self.link.rudder_test['phase'] == 'rudder_settle':
                self.assertEqual(self.link.rudder, 0.0)

    def test_a_raw_run_still_commands_its_deflection_immediately(self):
        """Unchanged behaviour: for a raw run, holding the deflection IS what
        the settle phase is for."""
        self.assertTrue(self.link.start_rudder_test(-1, 1)[0])
        self.assertAlmostEqual(self.link.rudder, -T.RUDDER_TEST_DEFLECTION)

    # ---- 3. zero and SEND before leaving Assisted mode ------------------

    def _indices(self, kind):
        return [i for i, m in enumerate(self.sent) if m[0] == kind]

    def test_zero_steering_is_sent_before_assisted_mode_is_switched_off(self):
        """In Assisted mode self.rudder is a yaw-RATE demand: +/-1.0 means
        +/-2 deg/s. The instant the loop is off, that same +/-1.0 is a RAW
        steering command -- full rudder, hard over. Disabling first left a
        one-tick window of exactly that on every abort."""
        self.link.start_rudder_test(-1, 1, assisted=True)
        self.clock.advance(1.0)
        self._boat_armed(fresh=True)
        self._tick()
        self.assertAlmostEqual(self.link.rudder, -1.0)   # full stick, drive phase

        self.sent.clear()
        self.link.stop(self.link.winch_command_seq + 1)

        zero_steer = [i for i, m in enumerate(self.sent)
                      if m[0] == 'steer' and m[1] == 0.0]
        assist_off = [i for i, m in enumerate(self.sent)
                      if m[0] == 'assist' and m[1] is False and m[2] is False]
        self.assertTrue(zero_steer, 'no zero steer was sent at all')
        self.assertTrue(assist_off, 'assisted mode was never switched off')
        self.assertLess(min(zero_steer), min(assist_off),
                        'assisted mode was disabled while a full-scale rate '
                        'demand was still the standing steering command')

    def test_zero_throttle_is_also_sent_before_the_mode_change(self):
        self.link.start_rudder_test(-1, 1, assisted=True)
        self.clock.advance(1.0)
        self._boat_armed(fresh=True)
        self._tick()
        self.sent.clear()
        self.link.stop(self.link.winch_command_seq + 1)
        zero_motor = [i for i, m in enumerate(self.sent)
                      if m[0] == 'motor' and m[1] == 0.0 and m[2] == 0.0]
        assist_off = [i for i, m in enumerate(self.sent) if m[0] == 'assist']
        self.assertTrue(zero_motor)
        self.assertLess(min(zero_motor), min(assist_off))

    def test_the_ordering_holds_on_every_abort_route(self):
        for name, fire in (
                ('disarm', lambda: self.link.arm(False, False)),
                ('disconnect', lambda: self.link.disconnect()),
                ('manual input', lambda: self.link.set_state(throttle=0.5)),
        ):
            self.setUp()
            self.link.start_rudder_test(-1, 1, assisted=True)
            self.clock.advance(1.0)
            self._boat_armed(fresh=True)
            self._tick()
            self.sent.clear()
            fire()
            zero_steer = [i for i, m in enumerate(self.sent)
                          if m[0] == 'steer' and m[1] == 0.0]
            assist_off = [i for i, m in enumerate(self.sent) if m[0] == 'assist']
            self.assertTrue(zero_steer, '%s sent no zero steer' % name)
            if assist_off:
                self.assertLess(min(zero_steer), min(assist_off),
                                '%s disabled assist before zeroing' % name)
            self.assertEqual(self.link.rudder, 0.0, name)
            self.assertEqual(self.link.throttle, 0.0, name)

    def test_a_completed_run_zeroes_before_the_mode_change_too(self):
        self.link.start_rudder_test(-1, 1, assisted=True)
        while self.link.rudder_test is not None:
            self.clock.advance(1.0 / 15.0)
            self._boat_armed(fresh=True)
            self.sent.clear()
            self._tick()
        zero_steer = [i for i, m in enumerate(self.sent)
                      if m[0] == 'steer' and m[1] == 0.0]
        assist_off = [i for i, m in enumerate(self.sent) if m[0] == 'assist']
        self.assertTrue(zero_steer)
        self.assertTrue(assist_off)
        self.assertLess(min(zero_steer), min(assist_off))

    # ---- 4. both targets, side by side ----------------------------------

    def test_the_row_carries_the_requested_AND_the_boat_reported_target(self):
        """They should agree -- and when they do not, that IS the finding, so
        neither may stand in for the other."""
        self.link.start_rudder_test(-1, 1, assisted=True)
        self.clock.advance(1.0)
        self._boat_armed(fresh=True)
        self.link.motor_status = dict(self.link.motor_status,
                                      yaw_target_dps=1.75, assist_rudder=True)
        self._tick()
        self._frame()
        row = self.link.rudder_test['rows'][-1]
        self.assertAlmostEqual(row['target_dps'], +T.ASSIST_TEST_TARGET_DPS)
        self.assertAlmostEqual(row['boat_target_dps'], 1.75)
        self.assertNotEqual(row['target_dps'], row['boat_target_dps'])

    def test_both_target_columns_reach_the_csv(self):
        self.link.start_rudder_test(-1, 1, assisted=True)
        res = self._run_to_completion()
        rows = read_rudder_csv(res['path'])
        self.assertIn('target_dps', rows[0])
        self.assertIn('boat_target_dps', rows[0])


class NotDrivingTest(RudderTestBase):
    """The fault that actually killed both assisted runs on 2026-09-02: the
    boat reported zero throttle for the whole drive phase while the tool
    commanded T20, and the run only failed 2 s later on a staleness timeout
    that named the wrong cause."""

    def _drive_reporting(self, left, right, seconds=3.0):
        """Run into the drive phase with the boat reporting this throttle."""
        self.assertTrue(self._start()[0])
        t0 = self.clock.t
        i = 0
        while self.link.rudder_test is not None and i * 0.05 < seconds:
            i += 1
            self.clock.t = t0 + i * 0.05
            self.link.motor_status = dict(
                self.link.motor_status, have=True, state=2, servo_power=True,
                left_throttle=left, right_throttle=right,
                last_rx_monotonic=self.clock.t)
            self._tick()
        return self.link.rudder_test

    def test_a_boat_that_never_drives_aborts_with_an_accurate_reason(self):
        self.assertIsNone(self._drive_reporting(0.0, 0.0))
        self.link._flush_rudder_test_write()
        res = self.link.rudder_test_result
        self.assertTrue(res['aborted'])
        reason = res['abort_reason'].lower()
        self.assertIn('not driving', reason)
        self.assertIn('arm', reason)
        self.assertNotIn('stale', reason)

    def test_it_aborts_promptly_rather_than_recording_a_useless_run(self):
        """The old behaviour recorded 2 s of a drifting boat before failing."""
        self._drive_reporting(0.0, 0.0)
        self.link._flush_rudder_test_write()
        res = self.link.rudder_test_result
        # settle 0.5 + at most the confirm window, well short of the full 4.5 s
        self.assertLess(res['duration_s'],
                        0.5 + T.RUDDER_TEST_DRIVE_CONFIRM_S + 0.4)

    def test_a_driving_boat_is_not_aborted(self):
        rt = self._drive_reporting(0.18, 0.22, seconds=2.0)
        self.assertIsNotNone(rt, 'a driving boat must not trip the check')
        self.assertTrue(rt['drive_confirmed'])

    def test_spin_up_latency_is_tolerated(self):
        """Zero for the first moments is the ESCs coming up, not a fault."""
        self.assertTrue(self._start()[0])
        t0 = self.clock.t
        for i in range(1, 40):                      # into the drive phase
            self.clock.t = t0 + i * 0.05
            late = (self.clock.t - t0) > 1.2        # reports late, but reports
            self.link.motor_status = dict(
                self.link.motor_status, have=True, state=2, servo_power=True,
                left_throttle=0.18 if late else 0.0,
                right_throttle=0.22 if late else 0.0,
                last_rx_monotonic=self.clock.t)
            self._tick()
            self.assertIsNotNone(self.link.rudder_test,
                                 'aborted during normal spin-up latency')

    def test_a_boat_that_stops_driving_mid_run_is_not_re_flagged(self):
        """Once confirmed, a momentary zero report is a lost packet, not a
        disarm -- the arm and staleness gates cover a real disarm."""
        self._start()
        t0 = self.clock.t
        for i in range(1, 45):
            self.clock.t = t0 + i * 0.05
            drop = 1.5 < (self.clock.t - t0) < 2.5
            self.link.motor_status = dict(
                self.link.motor_status, have=True, state=2, servo_power=True,
                left_throttle=0.0 if drop else 0.18,
                right_throttle=0.0 if drop else 0.22,
                last_rx_monotonic=self.clock.t)
            self._tick()
        self.assertIsNotNone(self.link.rudder_test)

    def test_the_settle_and_coast_phases_are_not_checked(self):
        """They command zero throttle, so a zero report is correct there."""
        self.assertTrue(self._start()[0])
        t0 = self.clock.t
        for i in range(1, 9):                       # 0.05 .. 0.40 s, settle
            self.clock.t = t0 + i * 0.05
            self._boat_armed(fresh=True)
            self._tick()
            self.assertIsNotNone(self.link.rudder_test)


class MotorStatusFreshnessTest(RudderTestBase):
    """MotorStatus is published on CHANGE or once a second, whichever comes
    first. On a stopped boat -- the state this gate checks -- that is 1 Hz."""

    def test_the_allowance_is_not_the_telemetry_limit(self):
        """Reusing TELEMETRY_STALE_S meant two lost packets aborted a run."""
        self.assertGreater(T.RUDDER_TEST_STATUS_MAX_AGE_S, T.TELEMETRY_STALE_S)
        self.assertGreaterEqual(T.RUDDER_TEST_STATUS_MAX_AGE_S, 3.0)

    def test_two_lost_packets_at_one_hz_no_longer_abort(self):
        self.assertTrue(self._start()[0])
        self.clock.advance(2.2)          # would have aborted before
        self._tick()
        self.assertIsNotNone(self.link.rudder_test)

    def test_a_genuinely_dead_status_still_aborts(self):
        self.assertTrue(self._start()[0])
        self.clock.advance(T.RUDDER_TEST_STATUS_MAX_AGE_S + 0.5)
        self._tick()
        self.assertIsNone(self.link.rudder_test)
        self.assertEqual(self.link.rudder, 0.0)
        self.link._flush_rudder_test_write()
        self.assertIn('stale',
                      self.link.rudder_test_result['abort_reason'].lower())


class ArmStateLoggedTest(RudderTestBase):
    """The two columns whose absence made 2026-09-02 undiagnosable from the
    file alone: boat_left/right were 0.0 all through the drive phase and there
    was no way to tell "disarmed" from "armed but refusing"."""

    def test_the_row_carries_the_boat_arm_state_and_rail(self):
        self._start()
        self.clock.advance(1.0)
        self.link.motor_status = dict(self.link.motor_status, have=True,
                                      state=2, servo_power=True,
                                      left_throttle=0.18, right_throttle=0.22,
                                      last_rx_monotonic=self.clock.t)
        self._tick()
        self._frame()
        row = self.link.rudder_test['rows'][-1]
        self.assertEqual(row['boat_state'], 2)
        self.assertEqual(row['boat_servo_power'], 1)

    def test_a_disarmed_boat_is_visible_in_the_row(self):
        self._start()
        self.clock.advance(0.2)
        self.link.motor_status = dict(self.link.motor_status, have=True,
                                      state=0, servo_power=False,
                                      last_rx_monotonic=self.clock.t)
        self._frame()
        row = self.link.rudder_test['rows'][-1]
        self.assertEqual(row['boat_state'], 0)
        self.assertEqual(row['boat_servo_power'], 0)

    def test_both_columns_reach_the_csv(self):
        self._start()
        res = self._run_to_completion()
        rows = read_rudder_csv(res['path'])
        self.assertIn('boat_state', rows[0])
        self.assertIn('boat_servo_power', rows[0])


class MutualExclusionTest(RudderTestBase):
    """Motor P assist and Assisted Steering must never both be on: one
    perturbs differential thrust and the other the rudder, and a result with
    both live is attributable to neither."""

    def test_enabling_the_rudder_loop_is_refused_while_motor_p_is_on(self):
        self.link.p_assist_on = True
        ok, err = self.link.send_rudder_assist(True)
        self.assertFalse(ok)
        self.assertIn('mutually exclusive', err.lower())
        self.assertFalse(self.link.assist_rudder_on)

    def test_enabling_motor_p_is_refused_while_the_rudder_loop_is_on(self):
        self.assertTrue(self.link.send_rudder_assist(True)[0])
        ok, err = self.link.send_assist(True)
        self.assertFalse(ok)
        self.assertIn('mutually exclusive', err.lower())
        self.assertFalse(self.link.p_assist_on)

    def test_the_wire_message_never_carries_both(self):
        ok, err = self.link._send_assist_locked(True, True)
        self.assertFalse(ok)
        self.assertIn('mutually exclusive', err.lower())

    def test_an_assisted_run_is_refused_while_motor_p_is_requested(self):
        self.link.p_assist_on = True
        ok, err = self.link.start_rudder_test(-1, 1, assisted=True)
        self.assertFalse(ok)
        self.assertIn('p assist', err.lower())

    def test_a_run_is_refused_when_the_BOAT_reports_motor_p_on(self):
        """Requested-off is not enough: the boat's own answer is what counts."""
        self.link.motor_status = dict(self.link.motor_status, have=True,
                                      assist_motor_p=True, state=2,
                                      servo_power=True,
                                      last_rx_monotonic=self.clock.t)
        ok, err = self.link.start_rudder_test(-1, 1, assisted=True)
        self.assertFalse(ok)
        self.assertIn('motor p', err.lower())

    def test_turning_the_rudder_loop_on_forces_motor_p_off_on_the_wire(self):
        self.link.p_assist_on = False
        self.assertTrue(self.link.send_rudder_assist(True)[0])
        self.assertTrue(self.link.assist_rudder_on)
        self.assertFalse(self.link.p_assist_on)


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
