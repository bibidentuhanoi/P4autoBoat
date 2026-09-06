"""The one-button lake steering-identification test.

Everything runs on a fake clock against a BoatLink with no serial port and a
simulated boat that applies what it is told. The writer is real (a temp dir)
because incremental, off-lock recording is part of what is being tested.
"""

import csv
import importlib.util
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'espnow_drive.py'


def _load():
    spec = importlib.util.spec_from_file_location('espnow_drive_lake', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


T = _load()
DT = 1.0 / 15.0


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)
        return self.t


class LakeBase(unittest.TestCase):

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.clock = FakeClock()
        self.sent = []
        link = T.BoatLink.__new__(T.BoatLink)
        link._lock = threading.RLock()
        link._now = self.clock
        link.pb2 = T.load_boat_pb2()
        link.connected = True
        for k, v in dict(throttle=0.0, motor_left=0.0, motor_right=0.0, motor_split=False,
                         rudder=0.0, winch_speed=0.0, winch_lease_until=0.0,
                         winch_command_seq=0, armed_cmd=True, force=False,
                         calibrating=False, servo_rail_cut=None, p_assist_on=True,
                         assist_rudder_on=False, assist_request_id=0, _assist_req_seq=0,
                         _assist_off_req_id=None, _assist_off_next_retry=0.0,
                         _assist_off_started=0.0, _last_hb_state=None,
                         session_id='sess', session_seq=0, session_last_hb=self.clock.t,
                         lease_expired_at=None, bench_yaw_samples=[], bench_yaw=None,
                         bench_run=None, _bench_write=None, bench_csv_name=None,
                         bench_dir=self.tmp, rudder_test=None, rudder_test_result=None,
                         _rudder_test_write=None, rudder_test_dir=self.tmp,
                         lake_id=None, lake_id_result=None, _lake_writer=None,
                         _lake_id_next_cache=None, lake_id_dir=self.tmp,
                         seq=0, last_error=None, ser=None, port='/dev/null').items():
            setattr(link, k, v)
        for b in ('telemetry', 'motor_status', 'bench_status', 'calibrate_status',
                  'system_status', 'bridge_status'):
            setattr(link, b, getattr(T.BoatLink, '_blank_' + b)())
        link._close_locked = lambda: setattr(link, 'connected', False)

        def _write(payload):
            msg = link.pb2.BoatMessage()
            try:
                msg.ParseFromString(payload)
            except Exception:                        # noqa: BLE001
                return True
            k = msg.WhichOneof('payload')
            if k == 'motor':
                self.sent.append(('motor', round(msg.motor.left, 3), round(msg.motor.right, 3)))
            elif k == 'steer':
                self.sent.append(('steer', round(msg.steer.left, 3)))
            return True
        link._write_locked = _write
        self.link = link
        self._telemetry(yaw=0.0)
        self._boat(0.0, 0.0, 0.0)
        self._system(imu_ok=True)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    # ---- the simulated boat ----------------------------------------------

    def _telemetry(self, yaw=0.0, heading=90.0, gps_valid=True, speed=1.2, course=90.0,
                   sats=9, hdop=1.1, lat=10.0, lon=106.0):
        self.link.telemetry = dict(self.link.telemetry, have=True, yaw_rate=yaw,
                                   heading=heading, pitch=1.0, roll=-0.5,
                                   gps_valid=gps_valid, lat=lat, lon=lon, speed_mps=speed,
                                   course_deg=course, satellites=sats, hdop=hdop,
                                   last_rx_monotonic=self.clock.t)

    def _boat(self, left, right, rudder, pwm=None, state=2, servo=True,
              assist_rudder=False, assist_p=True, fresh=True):
        if pwm is None:
            pwm = 1516 + (rudder * 289 if rudder < 0 else rudder * -321)   # 1805 left / 1195 right
            pwm = 1516 - rudder * 289 if rudder < 0 else 1516 - rudder * 321
        self.link.motor_status = dict(
            self.link.motor_status, have=True, state=state, servo_power=servo,
            left_throttle=left, right_throttle=right, rudder_cmd=rudder,
            rudder_pulse_us=int(round(pwm)), assist_rudder=assist_rudder,
            assist_motor_p=assist_p, last_rx_monotonic=self.clock.t if fresh else self.clock.t - 60)

    def _system(self, imu_ok=True, fresh=True):
        self.link.system_status = dict(self.link.system_status, have=True, imu_ok=imu_ok,
                                       mag_ok=True, gps_ok=True, tof_a_ok=True, tof_b_ok=True,
                                       camera_ok=True,
                                       last_rx_monotonic=self.clock.t if fresh else self.clock.t - 60)

    def _hb(self):
        self.link.session_last_hb = self.clock.t

    def _tick(self):
        with self.link._lock:
            self.link._lake_id_tick_locked(self.clock.t)

    def _frame(self, yaw=0.0, heading=90.0, **kw):
        self._telemetry(yaw=yaw, heading=heading, **kw)
        with self.link._lock:
            self.link._collect_lake_id_row_locked(yaw, heading, self.clock.t)

    def _start(self, throttle=0.20, magnitude=0.30, seq=None, notes=None, label=None):
        seq = self.link.winch_command_seq + 1 if seq is None else seq
        return self.link.start_lake_id(throttle, magnitude, seq, notes=notes, firmware_label=label)

    def _all_fresh(self):
        for d in ('telemetry', 'motor_status', 'system_status'):
            setattr(self.link, d, dict(getattr(self.link, d), last_rx_monotonic=self.clock.t))
        self._hb()

    def _drive(self, seconds, boat_follows=True, yaw_model=None, frames=True,
               stop_zero_delay=0.3, lag=0.25, refresh_ms=True):
        """Advance the run: 15 Hz ticks, a telemetry frame every tick, the
        boat applying commands after `lag` seconds."""
        yaw_state = getattr(self, '_yaw_state', 0.0)
        heading = getattr(self, '_heading', 90.0)
        applied = getattr(self, '_applied', (0.0, 0.0))
        pending = getattr(self, '_pending', [])
        end = self.clock.t + seconds
        while self.clock.t < end and self.link.lake_id is not None:
            self.clock.advance(DT)
            cmd = (self.link.throttle, self.link.rudder)
            pending.append((self.clock.t + lag, cmd))
            while pending and pending[0][0] <= self.clock.t:
                applied = pending.pop(0)[1]
            if boat_follows:
                t, r = applied
                self._boat(max(0.0, t - 0.02 if t else 0.0), max(0.0, t + 0.02 if t else 0.0), r)
            elif refresh_ms:
                self.link.motor_status = dict(self.link.motor_status, last_rx_monotonic=self.clock.t)
            # refresh the SystemStatus age only; imu_ok stays whatever the test set
            self.link.system_status = dict(self.link.system_status, last_rx_monotonic=self.clock.t)
            self._hb()
            self._tick()
            if frames and self.link.lake_id is not None:
                target = (yaw_model(applied) if yaw_model else
                          (0.4 + (-applied[1] * 8.0 if applied[0] > 0 else 0.0)))
                yaw_state += (target - yaw_state) * (DT / 1.0)
                heading = (heading + yaw_state * DT) % 360.0
                self._frame(yaw=yaw_state, heading=heading)
        self._yaw_state, self._heading, self._applied, self._pending = yaw_state, heading, applied, pending

    def _writer_idle(self, timeout=3.0):
        """Wait for the queue to drain, but never forever."""
        t0 = time.time()
        w = self.link._lake_writer
        while time.time() - t0 < timeout and w.q.unfinished_tasks:
            time.sleep(0.01)
        self.assertFalse(w.q.unfinished_tasks, 'writer queue never drained')

    def _wait_result(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.link._lock:
                r = self.link.lake_id_result
            if r is not None and r.get('published'):
                return r
            time.sleep(0.01)
        self.fail('result never published')

    def _run_full(self, **kw):
        ok, err = self._start(**kw)
        self.assertTrue(ok, err)
        self._drive(2.5)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self._drive(56.0)
        return self._wait_result()

    def _files(self, name):
        d = self.tmp / name
        rows = list(csv.DictReader([l for l in open(d / 'samples.csv') if not l.startswith('#')]))
        events = list(csv.DictReader(open(d / 'events.csv')))
        summary = json.load(open(d / 'summary.json'))
        return rows, events, summary


# ---------------------------------------------------------------------------
class PhaseTableTest(unittest.TestCase):

    def test_profile_is_exactly_57_seconds_and_50_powered(self):
        for order in ('LR', 'RL'):
            ph = T.lake_id_phases(0.2, 0.3, order)
            self.assertEqual(sum(p[1] for p in ph), 57.0)
            self.assertEqual(sum(p[1] for p in ph if p[2] > 0), 50.0)
            self.assertEqual([p[0] for p in ph], ['precheck', 'straight', 'turn_a', 'recover_a',
                                                  'turn_b', 'recover_b', 'stop'])

    def test_boundaries_from_elapsed_time(self):
        ph = T.lake_id_phases(0.2, 0.3, 'LR')
        for t, name in ((0, 'precheck'), (1.99, 'precheck'), (2.0, 'straight'), (11.99, 'straight'),
                        (12.0, 'turn_a'), (22.0, 'recover_a'), (32.0, 'turn_b'),
                        (42.0, 'recover_b'), (52.0, 'stop'), (56.99, 'stop')):
            self.assertEqual(T.lake_id_phase_at(ph, t)[0], name, t)
        self.assertIsNone(T.lake_id_phase_at(ph, 57.0))

    def test_left_first_then_right_and_the_reverse(self):
        lr = T.lake_id_phases(0.2, 0.3, 'LR')
        rl = T.lake_id_phases(0.2, 0.3, 'RL')
        self.assertEqual((lr[2][3], lr[4][3]), (-0.3, 0.3))
        self.assertEqual((rl[2][3], rl[4][3]), (0.3, -0.3))
        self.assertEqual(T.lake_id_phases(0.3, 0.6, 'LR')[2][3], -0.6)
        for p in lr:
            self.assertEqual(p[2], 0.2 if p[0] not in ('precheck', 'stop') else 0.0)

    def test_centre_phases_have_zero_rudder(self):
        for p in T.lake_id_phases(0.2, 0.6, 'LR'):
            if p[0] in ('precheck', 'straight', 'recover_a', 'recover_b', 'stop'):
                self.assertEqual(p[3], 0.0)


class SelectorTest(LakeBase):

    def test_defaults_are_the_safe_ones(self):
        self.assertEqual(T.LAKE_ID_THROTTLES[0], 0.20)
        self.assertEqual(T.LAKE_ID_MAGNITUDES[0], 0.30)

    def test_only_the_whitelisted_values_are_accepted(self):
        for thr, mag in ((0.25, 0.3), (0.4, 0.3), (0.2, 0.5), (0.2, 0.8), ('x', 0.3)):
            ok, err = self._start(throttle=thr, magnitude=mag)
            self.assertFalse(ok, (thr, mag))
            self.assertIsNone(self.link.lake_id)
        for thr, mag in ((0.2, 0.3), (0.3, 0.3), (0.2, 0.6), (0.3, 0.6)):
            ok, err = self._start(throttle=thr, magnitude=mag)
            self.assertTrue(ok, err)
            with self.link._lock:
                self.link._abort_lake_id_locked('test')
            self._wait_result()

    def test_stale_sequence_is_refused(self):
        self.link.winch_command_seq = 5
        self.assertFalse(self._start(seq=5)[0])
        self.assertTrue(self._start(seq=6)[0])


class PrecheckWindowTest(LakeBase):
    """A waiting window, not a first-tick test."""

    def test_conditions_met_late_but_before_two_seconds_still_start(self):
        self._boat(0.0, 0.0, 0.0, state=0)          # not armed yet
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(1.0, boat_follows=False)
        self.assertEqual(self.link.lake_id['phase'], 'precheck')
        self.assertIn('boat_armed', self.link.lake_id_status_locked()['precheck_unmet'])
        self._boat(0.0, 0.0, 0.0, state=2)           # now it is
        self._drive(1.2, boat_follows=False)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self.assertEqual(self.link.throttle, 0.2)

    def test_motors_are_never_commanded_before_the_gate(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(1.9, boat_follows=False)
        self.assertEqual(self.link.throttle, 0.0)
        self.assertNotIn(('motor', 0.2, 0.2), self.sent)

    def test_unmet_at_two_seconds_aborts_with_the_exact_list(self):
        self._boat(0.0, 0.0, 0.0, servo=False)
        self._system(imu_ok=False)
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(2.2, boat_follows=False)
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('precheck failed', r['reason'])
        self.assertIn('servo_power', r['reason'])
        self.assertIn('imu_ok', r['reason'])
        self.assertEqual(self.link.throttle, 0.0)
        self.assertNotIn(('motor', 0.2, 0.2), self.sent)
        _rows, events, summary = self._files(r['name'])
        self.assertIn('precheck_fail', [e['event'] for e in events])
        self.assertEqual(summary['status'], 'aborted')

    def test_motorstatus_1_4s_old_passes_1_6s_fails(self):
        for age, want in ((1.4, 'straight'), (1.6, None)):
            self.setUp()
            ok, err = self._start(); self.assertTrue(ok, err)
            self._drive(1.9, boat_follows=False)
            self.clock.advance(0.2); self._hb()
            self.link.motor_status = dict(self.link.motor_status, last_rx_monotonic=self.clock.t - age)
            self.link.telemetry = dict(self.link.telemetry, last_rx_monotonic=self.clock.t)
            self.link.system_status = dict(self.link.system_status, last_rx_monotonic=self.clock.t)
            self._tick()
            if want:
                self.assertEqual(self.link.lake_id['phase'], want, age)
            else:
                r = self._wait_result()
                self.assertIn('motorstatus_fresh', r['reason'])
            self.tearDown()

    def test_each_condition_blocks_start_on_its_own(self):
        cases = {
            'gps_valid': lambda: self._telemetry(gps_valid=False),
            'boat_rudder_centred': lambda: self._boat(0.0, 0.0, 0.0, pwm=1540),
            'boat_zero_throttle': lambda: self._boat(0.1, 0.1, 0.0),
            'motor_p_on': lambda: self._boat(0.0, 0.0, 0.0, assist_p=False),
            'rudder_assist_off': lambda: self._boat(0.0, 0.0, 0.0, assist_rudder=True),
            'systemstatus_fresh': lambda: self._system(imu_ok=True, fresh=False),
            'imu_finite': lambda: self._telemetry(yaw=float('nan')),
            'browser_supervision': lambda: setattr(self.link, 'session_last_hb', self.clock.t - 5),
        }
        for cond, apply in cases.items():
            self.setUp()
            ok, err = self._start(); self.assertTrue(ok, err)
            self._drive(1.5, boat_follows=False)
            apply()
            self.clock.advance(0.6)
            for d in ('telemetry', 'motor_status', 'system_status'):
                if cond not in ('systemstatus_fresh',) or d != 'system_status':
                    setattr(self.link, d, dict(getattr(self.link, d), last_rx_monotonic=self.clock.t))
            if cond != 'browser_supervision':
                self._hb()
            self._tick()
            r = self._wait_result()
            self.assertEqual(r['status'], 'aborted', cond)
            self.assertIn(cond, r['reason'], cond)
            self.assertEqual(self.link.throttle, 0.0)
            self.tearDown()

    def test_the_folder_and_files_exist_at_button_press(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        d = self.tmp / self.link.lake_id['name']
        self.assertTrue((d / 'samples.csv').exists())
        self.assertTrue((d / 'events.csv').exists())

    def test_unwritable_folder_refuses_before_any_motor(self):
        self.link.lake_id_dir = self.tmp / 'nope.csv'
        (self.tmp / 'nope.csv').write_text('a file, not a directory')
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('cannot create', err)
        self.assertIsNone(self.link.lake_id)

    def test_refused_while_other_tests_run(self):
        self.link.calibrating = True
        self.assertFalse(self._start()[0])
        self.link.calibrating = False
        self.link.rudder_test = {'fake': True}
        self.assertFalse(self._start()[0])
        self.link.rudder_test = None
        self.link.armed_cmd = False
        self.assertIn('ARM', self._start()[1])
        self.link.armed_cmd = True
        self.link.session_id = None
        self.assertIn('session', self._start()[1])


class SequenceTest(LakeBase):

    def test_commands_per_phase_for_a_complete_LR_run(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        seen = {}
        t0 = self.clock.t
        while self.link.lake_id is not None:
            self._drive(0.5)
            if self.link.lake_id is not None:
                seen.setdefault(self.link.lake_id['phase'], (self.link.throttle, self.link.rudder))
        self.assertEqual(seen['straight'], (0.2, 0.0))
        self.assertEqual(seen['turn_a'], (0.2, -0.3))
        self.assertEqual(seen['recover_a'], (0.2, 0.0))
        self.assertEqual(seen['turn_b'], (0.2, 0.3))
        self.assertEqual(seen['recover_b'], (0.2, 0.0))
        self.assertEqual(seen['stop'], (0.0, 0.0))

    def test_powered_for_exactly_the_middle_fifty_seconds(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        t0 = self.clock.t
        powered = []
        while self.link.lake_id is not None:
            self._drive(DT)
            if self.link.lake_id is not None and self.link.throttle > 0:
                powered.append(self.clock.t - t0)
        self.assertGreaterEqual(min(powered), 2.0)
        self.assertLess(max(powered), 52.0 + DT)
        self.assertAlmostEqual(max(powered) - min(powered), 50.0, delta=2 * DT)

    def test_a_completed_run_is_complete_and_confirmed(self):
        r = self._run_full()
        self.assertEqual(r['status'], 'complete')
        self.assertTrue(r['stop_confirmed'])
        self.assertIsNone(r['write_error'])
        self.assertIsNone(self.link.lake_id)
        rows, events, summary = self._files(r['name'])
        self.assertEqual(summary['status'], 'complete')
        self.assertGreater(len(rows), 700)
        names = [e['event'] for e in events]
        for e in ('button_pressed', 'precheck_pass', 'phase_enter', 'cmd_sent',
                  'command_path_confirm', 'stop_confirmed', 'complete', 'finalize_started'):
            self.assertIn(e, names, e)
        self.assertNotIn('files_written', names)
        self.assertLess(names.index('complete'), names.index('finalize_started'))

    def test_cmd_sent_only_on_value_changes(self):
        r = self._run_full()
        _rows, events, _s = self._files(r['name'])
        self.assertEqual(sum(1 for e in events if e['event'] == 'cmd_sent'), 6)


class StopConfirmationTest(LakeBase):

    def _to_stop(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(52.5)
        self.assertEqual(self.link.lake_id['phase'], 'stop')

    def test_stale_zero_status_does_not_confirm(self):
        self._to_stop()
        self._drive(4.4)
        # boat reports zeros but the status is old
        self._boat(0.0, 0.0, 0.0, fresh=False)
        self._drive(0.3, boat_follows=False, frames=False, refresh_ms=False)
        self.assertIsNotNone(self.link.lake_id)
        self.assertEqual(self.link.lake_id['phase'], 'teardown')

    def test_confirmed_during_teardown_is_complete(self):
        self._to_stop()
        self.link.motor_status = dict(self.link.motor_status, left_throttle=0.2, right_throttle=0.2)
        self._drive(4.6, boat_follows=False)          # boat never zeroes...
        self.assertEqual(self.link.lake_id['phase'], 'teardown')
        self._drive(1.0)                              # ...then it does
        r = self._wait_result()
        self.assertEqual(r['status'], 'complete')

    def test_unconfirmed_by_59s_is_incomplete_stop_unconfirmed(self):
        self._to_stop()
        self.link.motor_status = dict(self.link.motor_status, left_throttle=0.2, right_throttle=0.2)
        self._drive(7.0, boat_follows=False)
        r = self._wait_result()
        self.assertEqual(r['status'], 'incomplete_stop_unconfirmed')
        self.assertFalse(r['stop_confirmed'])
        self.assertEqual(self.link.throttle, 0.0)
        _rows, events, summary = self._files(r['name'])
        self.assertIn('stop_unconfirmed', [e['event'] for e in events])
        self.assertIn('STOP not confirmed', ' '.join(summary['warnings']))
        self.assertEqual(summary['status'], 'incomplete_stop_unconfirmed')

    def test_zeros_keep_going_out_through_teardown(self):
        self._to_stop()
        self.sent.clear()
        self._drive(5.5, boat_follows=False)
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)


class AbortTest(LakeBase):

    def _powered(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self.assertEqual(self.link.throttle, 0.2)
        self.sent.clear()

    def _assert_aborted(self, needle, wire=True):
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn(needle, r['reason'])
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)
        if wire:                                  # a disconnected link has no wire
            self.assertIn(('motor', 0.0, 0.0), self.sent)
            self.assertIn(('steer', 0.0), self.sent)
        rows, events, summary = self._files(r['name'])
        self.assertTrue(rows, 'partial rows were not preserved')
        self.assertIn('abort', [e['event'] for e in events])
        self.assertEqual(summary['status'], 'aborted')
        self.assertIn(needle, summary['reason'])
        return r

    def test_stop(self):
        self._powered()
        self.link.stop(self.link.winch_command_seq + 1)
        self._assert_aborted('STOP')

    def test_manual_input_aborts_and_is_discarded(self):
        self._powered()
        with self.link._lock:
            self.link._apply_state_locked(throttle=0.5, operator_moved=True)
        self._assert_aborted('manual')
        self.assertEqual(self.link.throttle, 0.0)

    def test_disarm(self):
        self._powered()
        self.link.arm(False, False)
        self._assert_aborted('disarm')

    def test_disconnect(self):
        self._powered()
        self.link.connected = False
        self._tick()
        self._assert_aborted('disconnected', wire=False)

    def test_servo_power_lost(self):
        self._powered()
        self._boat(0.2, 0.2, 0.0, servo=False); self._tick()
        self._assert_aborted('servo power')

    def test_telemetry_stale(self):
        self._powered()
        self.clock.advance(1.2); self._hb()
        self._boat(0.2, 0.2, 0.0); self._system()
        self._tick()
        self._assert_aborted('telemetry stale')

    def test_motorstatus_stale(self):
        self._powered()
        self.clock.advance(1.7); self._hb(); self._telemetry(); self._system()
        self._tick()
        self._assert_aborted('MotorStatus stale')

    def test_systemstatus_stale_after_three_seconds(self):
        self._powered()
        self._drive(2.5, boat_follows=True)
        self.link.system_status = dict(self.link.system_status, last_rx_monotonic=self.clock.t - 3.1)
        self._tick()
        self._assert_aborted('SystemStatus stale')

    def test_imu_unhealthy(self):
        self._powered()
        self._system(imu_ok=False); self._tick()
        self._assert_aborted('IMU unhealthy')

    def test_mode_loss(self):
        self._powered()
        self._boat(0.2, 0.2, 0.0, assist_rudder=True); self._tick()
        self._assert_aborted('mode changed')

    def test_browser_supervision_lost_after_two_seconds(self):
        self._powered()
        self.link.session_last_hb = self.clock.t - 2.1
        self._tick()
        self._assert_aborted('supervision')

    def test_a_short_heartbeat_gap_does_not_abort(self):
        self._powered()
        self.link.session_last_hb = self.clock.t - 1.5
        self._tick()
        self.assertIsNotNone(self.link.lake_id)

    def test_non_finite_yaw(self):
        self._powered()
        self._frame(yaw=float('nan'))
        self._assert_aborted('non-finite')

    def test_boat_refusal(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(2.5, boat_follows=False)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self._drive(2.0, boat_follows=False)          # boat-applied L/R stay 0
        self._assert_aborted('not driving')

    def test_serial_write_failure(self):
        self._powered()
        with self.link._lock:
            self.link._rudder_test_after_send_locked(False)
        self._assert_aborted('serial write failed')

    def test_calibration_start(self):
        self._powered()
        self.link.calibrating = True; self._tick()
        self._assert_aborted('calibration')

    def test_recording_failure_aborts_and_keeps_zeros(self):
        self._powered()
        self.link._lake_writer.error = 'disk full'
        self._tick()
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('recording failed', r['reason'])
        self.assertEqual(self.link.throttle, 0.0)

    def test_no_yaw_magnitude_abort_only_a_warning(self):
        self._powered()
        self._drive(11.0)                             # into turn_a
        self.assertEqual(self.link.lake_id['phase'], 'turn_a')
        self._frame(yaw=400.0)
        self.assertIsNotNone(self.link.lake_id)
        self._drive(4.0, yaw_model=lambda a: -25.0)   # LEFT rudder, negative yaw: wrong sign
        self.assertIsNotNone(self.link.lake_id)
        self.assertTrue(any('opposite' in w for w in self.link.lake_id['warnings']))


class OrderTest(LakeBase):

    def _fake_run(self, cond, order, idx, status):
        d = self.tmp / ('LAKE_ID_%s_%s_%03d' % (cond, order, idx))
        d.mkdir()
        (d / 'summary.json').write_text(json.dumps({'status': status}))

    def test_alternates_per_condition_only_after_complete(self):
        self._fake_run('T20_M30', 'LR', 1, 'complete')
        self._fake_run('T20_M30', 'RL', 2, 'aborted')
        self._fake_run('T20_M30', 'RL', 3, 'incomplete_stop_unconfirmed')
        self._fake_run('T30_M30', 'LR', 1, 'complete')
        self._fake_run('T30_M30', 'RL', 2, 'complete')
        n = T.lake_id_scan(self.tmp)
        self.assertEqual(n['T20_M30']['next_order'], 'RL')       # one complete
        self.assertEqual(n['T20_M30']['next_index'], 4)
        self.assertEqual(n['T30_M30']['next_order'], 'LR')       # two complete
        self.assertEqual(n['T20_M60']['next_order'], 'LR')       # untouched condition
        self.assertEqual(n['T20_M60']['next_index'], 1)

    def test_a_real_complete_run_flips_the_order_for_its_condition_only(self):
        r = self._run_full()
        self.assertTrue(r['name'].endswith('_LR_001'))
        self.assertEqual(self.link.lake_id_next()['T20_M30']['next_order'], 'RL')
        self.assertEqual(self.link.lake_id_next()['T30_M30']['next_order'], 'LR')
        r2 = self._run_full()
        self.assertTrue(r2['name'].endswith('_RL_002'))

    def test_an_aborted_run_does_not_flip(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self.link.stop(self.link.winch_command_seq + 1)
        self._wait_result()
        self.assertEqual(self.link.lake_id_next()['T20_M30']['next_order'], 'LR')
        self.assertEqual(self.link.lake_id_next()['T20_M30']['next_index'], 2)


class RowsAndEventsTest(LakeBase):

    def test_rows_have_exactly_the_schema_columns(self):
        r = self._run_full()
        rows, _e, _s = self._files(r['name'])
        self.assertEqual(list(rows[0].keys()), list(T.LAKE_ID_CSV_COLUMNS))

    def test_rows_start_in_precheck(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._frame(yaw=0.0)
        self.assertEqual(self.link.lake_id['rows'][0]['phase'], 'precheck')

    def test_gps_values_changed_is_advisory_only(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._frame(yaw=0.0, lat=10.0)
        self._frame(yaw=0.0, lat=10.0)
        self._frame(yaw=0.0, lat=10.001)
        vals = [r['gps_values_changed'] for r in self.link.lake_id['rows']]
        self.assertEqual(vals, [0, 0, 1])
        src = TOOL.read_text()
        self.assertIn('ADVISORY only', src)
        self.assertNotIn('gps_values_changed) / ', src)

    def test_rows_are_on_disk_before_the_run_ends(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self._writer_idle()
        d = self.tmp / self.link.lake_id['name']
        n = sum(1 for l in open(d / 'samples.csv') if not l.startswith('#')) - 1
        self.assertGreater(n, 20)

    def test_applied_columns_are_named_as_commands(self):
        for c in ('boat_applied_left_cmd', 'boat_applied_right_cmd',
                  'boat_applied_rudder_cmd', 'boat_applied_rudder_pwm_us'):
            self.assertIn(c, T.LAKE_ID_CSV_COLUMNS)
        head = '\n'.join(T.LAKE_ID_SAMPLES_HEADER)
        self.assertIn('not measured RPM, thrust or servo angle', head)


class SummaryTest(unittest.TestCase):
    """Pure: synthetic rows with known answers."""

    def _rows(self, order='LR', bias=0.5, gain=8.0, tau=1.0, speed=1.5, gps=True):
        ph = T.lake_id_phases(0.2, 0.3, order)
        rows, yaw, heading, t = [], bias, 90.0, 0.0
        while t < 57.0:
            p = T.lake_id_phase_at(ph, t)
            target = bias + (-p[3] * gain if p[2] > 0 else 0.0)
            yaw += (target - yaw) * (0.05 / tau)
            heading = (heading + yaw * 0.05) % 360
            rows.append({'elapsed_s': round(t, 3), 'phase': p[0], 'yaw_dps': yaw,
                         'heading_deg': heading, 'pitch_deg': 1.0, 'roll_deg': 0.0,
                         'gps_valid': 1 if gps else 0, 'speed_mps': speed, 'course_deg': heading,
                         'satellites': 9, 'hdop': 1.0,
                         'boat_applied_left_cmd': 0.18, 'boat_applied_right_cmd': 0.22,
                         'boat_assist_motor_p': 1, 'boat_assist_rudder': 0})
            t += 0.05
        return rows

    def _sum(self, rows, **kw):
        settings = {'throttle': 0.2, 'magnitude': 0.3, 'order': kw.pop('order', 'LR'),
                    'condition': 'T20_M30', 'index': 1, 'name': 'x'}
        return T.lake_id_summarize(rows, [], settings, {'git_head': 'abc'}, 'complete', None,
                                   stop_confirmed=True, notes={k: 'ok' for k in T.LAKE_ID_NOTE_FIELDS},
                                   firmware_label='lbl', **kw)

    def test_straight_bias_and_turn_metrics(self):
        s = self._sum(self._rows())
        self.assertAlmostEqual(s['straight']['mean_yaw_dps'], 0.5, places=1)
        ta = s['turn_a']
        self.assertEqual(ta['side'], 'LEFT')
        self.assertAlmostEqual(ta['steady_yaw_minus_bias_dps'], 2.4, delta=0.1)   # 0.3 * 8
        self.assertGreater(ta['response_delay_s'], 0.0)
        self.assertLess(ta['response_delay_s'], 1.0)
        self.assertAlmostEqual(ta['rise_time_s'], 2.2, delta=0.4)                 # ln(9)*tau
        self.assertGreater(ta['heading_change_deg'], 15.0)
        tb = s['turn_b']
        self.assertEqual(tb['side'], 'RIGHT')
        self.assertAlmostEqual(tb['steady_yaw_minus_bias_dps'], -2.4, delta=0.1)

    def test_recovery_metrics_and_wording(self):
        s = self._sum(self._rows())
        ra = s['recover_a']
        self.assertGreater(ra['recovery_time_s'], 0.5)
        self.assertLess(ra['recovery_time_s'], 4.0)
        self.assertIn('autotrim', ra['note'])
        self.assertIn('not a direct measurement', s['straight']['note'].replace('NOT a measurement', 'not a direct measurement'))

    def test_turn_radius_is_provisional_with_inputs_and_rules(self):
        s = self._sum(self._rows())
        tr = s['turn_a']['turn_radius']
        self.assertTrue(tr['provisional'])
        for k in ('radius_m', 'gps_valid_fraction', 'mean_speed_mps', 'mean_yaw_dps',
                  'yaw_above_straight_bias_dps', 'yaw_sigma_vs_straight', 'rules', 'warnings', 'formula'):
            self.assertIn(k, tr)
        self.assertNotIn('valid', {k for k in tr if k in ('valid', 'invalid')})
        # v / (yaw rad/s): 1.5 / (2.9 deg/s in rad) ~ 29.6 m
        self.assertAlmostEqual(tr['radius_m'], 1.5 / (abs(tr['mean_yaw_dps']) * math.pi / 180), places=1)
        self.assertEqual(tr['rules']['min_speed_mps'], T.LAKE_ID_RULES['radius_min_speed_mps'])
        self.assertEqual(s['rules'], T.LAKE_ID_RULES)

    def test_radius_warns_but_still_reports_when_gps_is_poor(self):
        s = self._sum(self._rows(gps=False))
        tr = s['turn_a']['turn_radius']
        self.assertTrue(any('gps_valid fraction' in w for w in tr['warnings']))
        self.assertIsNone(tr['mean_speed_mps'])
        self.assertIsNotNone(tr['unavailable_reason'])

    def test_wrong_sign_is_a_warning(self):
        s = self._sum(self._rows(gain=-8.0))
        self.assertTrue(any('disagrees with the hypothesis' in w for w in s['warnings']))

    def test_missing_notes_warn(self):
        rows = self._rows()
        settings = {'throttle': 0.2, 'magnitude': 0.3, 'order': 'LR', 'condition': 'T20_M30', 'index': 1, 'name': 'x'}
        s = T.lake_id_summarize(rows, [], settings, {}, 'complete', None, stop_confirmed=True, notes={})
        self.assertEqual(sum(1 for w in s['warnings'] if 'operator note missing' in w), len(T.LAKE_ID_NOTE_FIELDS))

    def test_frozen_imu_is_a_warning_not_an_abort(self):
        rows = self._rows()
        for r in rows[300:340]:
            r['yaw_dps'] = 1.234; r['heading_deg'] = 5.0; r['pitch_deg'] = 1.0; r['roll_deg'] = 0.0
        s = self._sum(rows)
        self.assertTrue(any('identical' in w for w in s['warnings']))

    def test_provenance_and_wording_present(self):
        s = self._sum(self._rows())
        self.assertEqual(s['provenance']['git_head'], 'abc')
        self.assertEqual(s['provenance']['firmware_label'], 'lbl')
        self.assertIn('turn radius is provisional', ' '.join(s['wording']))
        self.assertEqual(s['settings']['profile_s'], 57.0)


class MetadataTest(LakeBase):

    def test_provenance_from_the_real_repo(self):
        p = T.lake_id_provenance(TOOL)
        self.assertNotEqual(p['git_head'], 'unknown')
        self.assertIsInstance(p['git_dirty'], bool)
        self.assertEqual(len(p['espnow_drive_sha256']), 64)
        self.assertIn('steer_pulse_max_us', p['rudder_pulse_config_believed'])
        self.assertIn('BELIEVED', p['rudder_pulse_config_believed']['note'])

    def test_firmware_label_is_prefilled_but_editable(self):
        r = self._run_full()
        _r, _e, s = self._files(r['name'])
        self.assertEqual(s['provenance']['firmware_label'], T.LAKE_ID_FIRMWARE_LABEL_DEFAULT)
        self.assertIn('1805', T.LAKE_ID_FIRMWARE_LABEL_DEFAULT)
        r2 = self._run_full(label='my build')
        _r, _e, s2 = self._files(r2['name'])
        self.assertEqual(s2['provenance']['firmware_label'], 'my build')

    def test_notes_are_recorded_and_missing_ones_warned(self):
        r = self._run_full(notes={'battery': '4S 15.8V'})
        _r, _e, s = self._files(r['name'])
        self.assertEqual(s['operator_notes']['battery'], '4S 15.8V')
        self.assertTrue(any('operator note missing: wind' in w for w in s['warnings']))


class WriterTest(LakeBase):

    def test_result_is_published_only_after_the_files_are_final(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(3.0)
        with self.link._lock:
            self.link._abort_lake_id_locked('test')
            self.assertTrue(self.link.lake_id['finalizing'])
        r = self._wait_result()
        d = self.tmp / r['name']
        self.assertTrue((d / 'summary.json').exists())
        self.assertFalse((d / 'summary.json.tmp').exists())
        self.assertTrue(r['published'])

    def test_publish_waits_for_a_slow_finalize(self):
        """The result must appear strictly AFTER the writer has flushed,
        closed and written summary.json -- not merely 'usually after'. A slow
        finalize makes any early publish visible."""
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(3.0)
        w = self.link._lake_writer
        real = w._finalize
        gate = threading.Event()

        def slow(summary):
            gate.wait(2.0)                    # hold the files open until released
            return real(summary)
        w._finalize = slow
        with self.link._lock:
            self.link._abort_lake_id_locked('slow test')
        time.sleep(0.2)
        with self.link._lock:
            self.assertIsNone(self.link.lake_id_result, 'published before the files were final')
            self.assertIsNotNone(self.link.lake_id)
        d = self.tmp / self.link.lake_id['name']
        self.assertFalse((d / 'summary.json').exists())
        gate.set()
        r = self._wait_result()
        self.assertTrue(r['published'])
        self.assertTrue((d / 'summary.json').exists())

    def test_a_write_error_is_recorded_and_the_run_aborts(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(3.0)
        self.link._lake_writer._fh['samples'].close()       # next row write raises
        self._frame(yaw=1.0)
        self._writer_idle()
        self.assertIsNotNone(self.link._lake_writer.error)
        self._tick()
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('recording failed', r['reason'])

    def test_no_disk_io_in_the_control_lock(self):
        src = TOOL.read_text()
        i = src.index('    def _collect_lake_id_row_locked')
        j = src.index('    def _abort_lake_id_locked', i)
        body = src[i:j]
        self.assertNotIn('open(', body)
        self.assertNotIn('.write(', body)
        k = src.index('    def _lake_id_tick_locked')
        body2 = src[k:src.index('    def _collect_lake_id_row_locked')]
        self.assertNotIn('open(', body2)


class GuardTest(unittest.TestCase):
    """This experiment is Python-only. The guard compares the committed
    history against the A+B baseline it grew from, not the working tree
    against HEAD (which only ever proved there was nothing uncommitted)."""

    BASELINE = 'e7abc06'      # the reconstructed A+B firmware the boat matches
    PATHS = ['main/', 'proto/', 'partitions.csv',
             ':(glob)**/Kconfig*', ':(glob)**/sdkconfig*']

    def _git(self, *args):
        return subprocess.run(['git', '-C', str(ROOT)] + list(args),
                              capture_output=True, text=True)

    def test_no_firmware_proto_or_config_change_since_the_baseline(self):
        anc = self._git('merge-base', '--is-ancestor', self.BASELINE, 'HEAD')
        self.assertEqual(anc.returncode, 0,
                         '%s is not an ancestor of HEAD: update BASELINE deliberately, '
                         'never let this guard pass by accident' % self.BASELINE)
        committed = self._git('diff', '--name-only', self.BASELINE, 'HEAD', '--',
                              *self.PATHS).stdout.split()
        self.assertEqual(committed, [], 'firmware/proto/config changed since %s: %s'
                         % (self.BASELINE, committed))
        working = self._git('diff', '--name-only', 'HEAD', '--', *self.PATHS).stdout.split()
        self.assertEqual(working, [], 'uncommitted firmware/proto/config change: %s' % working)

    def test_the_guard_can_actually_fail(self):
        """The baseline's own parent differs from it in main/ (that commit IS a
        firmware change), so the very same command must report it."""
        out = self._git('diff', '--name-only', self.BASELINE + '~1', self.BASELINE, '--',
                        *self.PATHS).stdout.split()
        self.assertTrue(any(f.startswith('main/') for f in out), out)

    def test_the_active_rudder_maximum_is_1805(self):
        kconfig = (ROOT / 'main' / 'Kconfig.projbuild').read_text()
        m = re.search(r'config STEER_PULSE_MAX_US\s*\n\s*int[^\n]*\n\s*default (\d+)', kconfig)
        self.assertIsNotNone(m, 'STEER_PULSE_MAX_US default not found')
        self.assertEqual(m.group(1), '1805')
        sdk = ROOT / 'sdkconfig'
        if sdk.exists():
            self.assertRegex(sdk.read_text(), r'(?m)^CONFIG_STEER_PULSE_MAX_US=1805$')
        self.assertIn('FULL_LEFT_US = 1805',
                      (ROOT / 'tests' / 'test_steer_direction.py').read_text())
        for rel in ('main/Kconfig.projbuild', 'main/drivers/steer_driver.c',
                    'main/stability_control.h'):
            self.assertNotIn('1835', (ROOT / rel).read_text(), rel)
        self.assertIn('value="1805"', (ROOT / 'main' / 'dashboard.html').read_text())


class UiTest(unittest.TestCase):

    def setUp(self):
        self.src = TOOL.read_text()

    def test_card_and_wiring(self):
        for s in ('id="lake-start"', 'id="lake-throttle"', 'id="lake-mag"', 'id="lake-fw"',
                  'id="lake-next"', 'id="lake-warn"', "api('/api/lake_id'", 'renderLakeId(s);',
                  "'lake-start',", 'on = on || _lakeActive;'):
            self.assertIn(s, self.src, s)

    def test_lake_render_runs_before_the_rudder_lockout(self):
        self.assertLess(self.src.index('    renderLakeId(s);'),
                        self.src.index('    const rt = s.rudder_test, rtr = s.rudder_test_result;'))

    def test_no_predicted_yaw_numbers(self):
        for bad in ('0.9 deg/s', '0.7 deg/s', '+0.9', '−0.7'):
            self.assertNotIn(bad, self.src)


class HttpTest(unittest.TestCase):

    def setUp(self):
        self.base_t = LakeBase(); self.base_t.setUp()
        self.link = self.base_t.link

        class H(T.Handler):
            pass
        H.link = self.link
        self.server = T.ThreadingHTTPServer(('127.0.0.1', 0), H)
        self.th = threading.Thread(target=self.server.serve_forever); self.th.start()
        self.base = 'http://%s:%d' % self.server.server_address

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.th.join(timeout=2)
        self.base_t.tearDown()

    def _post(self, path, body):
        from urllib import request as R, error as E
        rq = R.Request(self.base + path, data=json.dumps(body).encode(),
                       headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with R.urlopen(rq, timeout=2) as r:
                return r.status, json.load(r)
        except E.HTTPError as e:
            return e.code, json.load(e)

    def test_validation_and_start(self):
        self.assertEqual(self._post('/api/lake_id', {'throttle': 0.2, 'magnitude': 0.3})[0], 400)
        self.assertEqual(self._post('/api/lake_id', {'throttle': 0.2, 'magnitude': 0.3, 'seq': 1, 'notes': 'x'})[0], 400)
        code, body = self._post('/api/lake_id', {'throttle': 0.25, 'magnitude': 0.3, 'seq': 1})
        self.assertEqual(code, 409)
        code, body = self._post('/api/lake_id', {'throttle': 0.2, 'magnitude': 0.3, 'seq': 2,
                                                 'notes': {'battery': 'ok'}})
        self.assertEqual(code, 200, body)
        self.assertIsNotNone(self.link.lake_id)

    def test_status_carries_the_lake_keys(self):
        from urllib import request as R
        with R.urlopen(self.base + '/api/status', timeout=2) as r:
            s = json.load(r)
        for k in ('lake_id', 'lake_id_result', 'lake_id_next', 'lake_id_defaults'):
            self.assertIn(k, s)
        self.assertEqual(s['lake_id_next']['T20_M30']['next_order'], 'LR')
        self.assertEqual(s['lake_id_defaults']['firmware_label'], T.LAKE_ID_FIRMWARE_LABEL_DEFAULT)


if __name__ == '__main__':
    unittest.main()


class KeepaliveTest(LakeBase):
    """A repeated heartbeat is a keepalive, not a command, while a test owns
    the controls."""

    def test_an_unchanged_heartbeat_does_not_overwrite_the_running_command(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self.assertEqual(self.link.throttle, 0.2)
        self.link._last_hb_state = (0.0, 0.0, 0.0, 0.0, False)
        ok, err = self.link.control_heartbeat('sess', 99, throttle=0.0, rudder=0.0,
                                              left=0.0, right=0.0, split=False)
        self.assertTrue(ok, err)
        self.assertEqual(self.link.throttle, 0.2, 'the keepalive zeroed a running test')
        self.assertIsNotNone(self.link.lake_id)

    def test_a_changed_heartbeat_is_a_manual_input_and_aborts(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self.link._last_hb_state = (0.0, 0.0, 0.0, 0.0, False)
        self.link.control_heartbeat('sess', 99, throttle=0.3, rudder=0.0, left=0.0, right=0.0, split=False)
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('manual', r['reason'])


class NextOrderReachesThePageTest(unittest.TestCase):
    """The real page takes status over the WebSocket, not the HTTP poll. A
    cache filled only by the GET handler left NEXT ORDER at '--' in a real
    Chromium render of the served page."""

    def test_a_real_link_has_the_cache_from_construction(self):
        link = T.BoatLink(T.load_boat_pb2())
        nxt = link.lake_id_next()
        self.assertEqual(sorted(nxt), ['T20_M30', 'T20_M60', 'T30_M30', 'T30_M60'])
        self.assertIn(nxt['T20_M30']['next_order'], ('LR', 'RL'))

    def test_the_websocket_producer_fills_the_cache_too(self):
        src = TOOL.read_text()
        i = src.index('self.wfile.write(_ws_text_frame(json.dumps(self.link.status())))')
        self.assertIn('ensure_lake_id_next()', src[i - 200:i])


# ---------------------------------------------------------------------------
# Review fixes, 2026-09-05. Each class below reproduces a failure the earlier
# suite let through; every test here failed (or errored on a missing API)
# against 68122fc before the fix it guards.

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_espnow_drive import run_page_js          # noqa: E402  the node page harness


def _lake_folders(tmp):
    return sorted(p.name for p in Path(tmp).iterdir() if p.name.startswith('LAKE_ID_'))


class _BlockingWriterow:
    """Stands in for the samples DictWriter: the first writerow blocks until
    released, so the queue behind it fills exactly as it would on a stalled
    disk. Released rows go to the real writer."""

    def __init__(self, real, gate):
        self.real, self.gate, self.blocked = real, gate, threading.Event()

    def writerow(self, row):
        self.blocked.set()
        self.gate.wait(15.0)
        return self.real.writerow(row)


class FinalizeGuaranteeTest(LakeBase):
    """Item 1: finalize must never be lost to a full data queue."""

    def test_full_queue_abort_still_finalizes_closes_and_releases(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        w = self.link._lake_writer
        handles = list(w._fh.values())
        gate = threading.Event()
        blocker = _BlockingWriterow(w._w['samples'], gate)
        w._w['samples'] = blocker
        self._frame(yaw=0.1)                                # the row the writer sticks on
        self.assertTrue(blocker.blocked.wait(2.0), 'writer never picked up the row')
        template = dict(self.link.lake_id['rows'][-1])
        while w.error is None:                              # fill the DATA queue behind it
            w.put('row', template)
        self.assertEqual(w.error, 'writer queue full')
        self.sent.clear()
        self._tick()                                        # abort table sees the failure
        with self.link._lock:
            self.assertTrue(self.link.lake_id['finalizing'])
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)
        self.assertIn(('motor', 0.0, 0.0), self.sent)
        self.assertIn(('steer', 0.0), self.sent)
        gate.set()                                          # the disk comes back
        r = self._wait_result(timeout=10.0)
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('queue full', r['reason'])
        self.assertIsNone(self.link.lake_id, 'run stuck in finalizing')
        self.assertIsNone(self.link._lake_writer)
        self.assertTrue(all(fh.closed for fh in handles), 'CSV handles left open')
        d = self.tmp / r['name']
        self.assertTrue((d / 'summary.json').exists())
        s = json.load(open(d / 'summary.json'))
        self.assertEqual(s['status'], 'aborted')
        rec = s['recording']
        self.assertGreater(rec['sample_rows_dropped'], 0)
        self.assertEqual(rec['sample_rows_enqueued'], rec['sample_rows_written'])
        self.assertFalse(rec['complete'])
        self.assertEqual(self.link.lake_id_next()['T20_M30']['next_order'], 'LR')
        self.assertEqual(self.link.lake_id_next()['T20_M30']['complete_runs'], 0)

    def test_finalize_runs_even_when_the_writer_thread_never_started(self):
        d = self.tmp / 'LAKE_ID_T20_M30_LR_001'
        w = T.LakeIdWriter(d)
        w.prepare()                                         # files exist, thread NOT started
        done = threading.Event(); got = {}

        def on_done(res):
            got['res'] = res; done.set()
        w.finalize({'status': 'aborted', 'reason': 'x'}, on_done)
        self.assertTrue(done.wait(3.0), 'finalize callback never fired')
        self.assertTrue((d / 'summary.json').exists())
        self.assertIsInstance(got['res'], dict)
        self.assertTrue(got['res']['ok'], got['res'])


class _BadClose:
    """A file handle whose final close fails (a USB stick pulled, a full
    filesystem discovered at flush)."""

    def __init__(self, fh):
        self._fh = fh

    def __getattr__(self, name):
        return getattr(self._fh, name)

    def close(self):
        raise OSError('simulated final close failure')


class RecordingFailureStatusTest(LakeBase):
    """Item 2: a run whose recording failed at the end is never COMPLETE and
    never advances the LR/RL order."""

    def test_final_close_failure_is_not_complete_and_keeps_the_order(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        w = self.link._lake_writer
        w._fh['samples'] = _BadClose(w._fh['samples'])
        self._drive(2.5); self.assertEqual(self.link.lake_id['phase'], 'straight')
        self._drive(56.0)
        r = self._wait_result()
        self.assertNotEqual(r['status'], 'complete')
        self.assertEqual(r['status'], 'incomplete_recording_failed')
        self.assertIn('simulated final close failure', r['write_error'])
        self.assertIn('recording failed', r['reason'])
        self.assertEqual(r['profile_status'], 'complete')     # the motion itself did finish
        self.assertTrue(r['stop_confirmed'])
        s = json.load(open(self.tmp / r['name'] / 'summary.json'))
        self.assertEqual(s['status'], 'incomplete_recording_failed')
        self.assertIn('simulated', s['write_error'])
        self.assertEqual(s['profile_status'], 'complete')
        n = self.link.lake_id_next()['T20_M30']
        self.assertEqual(n['next_order'], 'LR')
        self.assertEqual(n['complete_runs'], 0)

    def test_summary_replace_failure_is_not_complete_and_keeps_the_order(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        d = self.tmp / self.link.lake_id['name']
        (d / 'summary.json').mkdir()                        # os.replace onto a directory fails
        self._drive(2.5); self._drive(56.0)
        r = self._wait_result()
        self.assertEqual(r['status'], 'incomplete_recording_failed')
        self.assertIsNotNone(r['write_error'])
        self.assertFalse(r['summary_written'])
        self.assertEqual(r['profile_status'], 'complete')
        n = self.link.lake_id_next()['T20_M30']
        self.assertEqual(n['next_order'], 'LR')
        self.assertEqual(n['complete_runs'], 0)

    def test_scan_never_counts_missing_unreadable_or_write_failed_summaries(self):
        for name, content in (
                ('LAKE_ID_T20_M30_LR_001', None),                      # no summary at all
                ('LAKE_ID_T20_M30_RL_002', '{not json'),               # unreadable
                ('LAKE_ID_T20_M30_LR_003', json.dumps({'status': 'complete',
                                                       'write_error': 'OSError: x'})),
                ('LAKE_ID_T20_M30_RL_004', json.dumps({'status': 'incomplete_recording_failed'})),
                ('LAKE_ID_T20_M30_LR_005', json.dumps(['not', 'an', 'object']))):
            d = self.tmp / name; d.mkdir()
            if content is not None:
                (d / 'summary.json').write_text(content)
        n = T.lake_id_scan(self.tmp)['T20_M30']
        self.assertEqual(n['complete_runs'], 0)
        self.assertEqual(n['next_order'], 'LR')
        self.assertEqual(n['next_index'], 6)

    def test_the_page_never_shows_complete_with_a_write_error(self):
        result = run_page_js(r"""
vm.createContext(context); vm.runInContext(script, context);
const st = Object.assign({}, CONNECTED_STATUS, {
  lake_id: null,
  lake_id_result: { status: 'complete', write_error: 'OSError: simulated', reason: null,
                    name: 'LAKE_ID_T20_M30_LR_001', order: 'LR', frames: 10,
                    warnings: [], summary_warnings: [], published: true },
  lake_id_next: {},
  lake_id_defaults: { throttles: [0.2, 0.3], magnitudes: [0.3, 0.6],
                      firmware_label: 'x', note_fields: ['battery'] } });
context.renderLakeId(st);
console.log(JSON.stringify({ pill: elements['lake-pill'].textContent,
                             stale: elements['lake-pill'].classList.contains('stale'),
                             phase: elements['lake-phase'].textContent }));
process.exit(0);
""")
        self.assertEqual(result.returncode, 0, result.stderr)
        out = json.loads(result.stdout.strip().splitlines()[-1])
        self.assertNotEqual(out['pill'], 'COMPLETE')
        self.assertTrue(out['stale'])
        self.assertIn('simulated', out['phase'])


class RefusalMonitorTest(LakeBase):
    """Item 3: boat-applied L/R are watched for the WHOLE powered run, on
    freshly received MotorStatus only."""

    def _events(self):
        return [e['event'] for e in self._files(self.link.lake_id_result['name'])[1]]

    def _powered_following(self, seconds=5.0):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(seconds)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self.assertTrue(self.link.lake_id['drive_confirmed'])

    def _zeros(self, seconds):
        """Fresh MotorStatus packets reporting L=R=0 while telemetry,
        SystemStatus and the browser heartbeat all stay alive."""
        self._boat(0.0, 0.0, 0.0)
        self._drive(seconds, boat_follows=False, refresh_ms=True)

    def test_initial_refusal_over_1_5_s_aborts(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(2.5, boat_follows=False)            # straight commanded, boat never applies
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self._drive(0.8, boat_follows=False)            # zero reports span ~1.3 s
        self.assertIsNotNone(self.link.lake_id, 'aborted before zero reports spanned 1.5 s')
        self._drive(0.6, boat_follows=False)            # ~1.9 s
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('not driving', r['reason'])
        self.assertIn('boat_refusal', self._events())

    def test_initial_response_inside_the_window_passes(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(8.0, lag=1.2)                       # boat applies 1.2 s after the command
        self.assertIsNotNone(self.link.lake_id)
        self.assertTrue(self.link.lake_id['drive_confirmed'])

    def test_brief_zero_after_confirmation_does_not_abort(self):
        self._powered_following()
        self._zeros(1.0)
        self.assertIsNotNone(self.link.lake_id, 'a 1 s zero interval aborted the run')
        self._drive(2.0)                                # boat drives again
        self.assertIsNotNone(self.link.lake_id)
        self.link.stop(self.link.winch_command_seq + 1)
        self._wait_result()
        self.assertNotIn('boat_refusal', self._events())

    def test_zero_reports_spanning_over_1_5_s_after_confirmation_abort(self):
        self._powered_following()
        self._zeros(3.0)
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('not driving', r['reason'])
        self.assertNotIn('stale', r['reason'])          # MotorStatus was fresh throughout
        self.assertIn('boat_refusal', self._events())
        self.assertEqual(self.link.throttle, 0.0)

    def test_recovery_to_nonzero_resets_the_timer(self):
        self._powered_following()
        self._zeros(1.0)
        self._drive(0.3)                                # a nonzero report resets the interval
        self._zeros(1.0)
        self.assertIsNotNone(self.link.lake_id, 'two short zero intervals were summed')
        self._zeros(0.8)                                # this one interval alone passes 1.5 s
        r = self._wait_result()
        self.assertIn('not driving', r['reason'])

    def test_one_old_zero_packet_and_wall_clock_are_not_proof(self):
        self._powered_following()
        self._boat(0.0, 0.0, 0.0)                       # ONE zero packet, then silence
        self._drive(1.3, boat_follows=False, refresh_ms=False)
        self.assertIsNotNone(self.link.lake_id)         # neither refusal nor stale (< 1.5 s)
        self._drive(1.0)                                # a nonzero packet clears it
        self.assertIsNotNone(self.link.lake_id)

    def test_the_stop_phase_zeros_never_count_as_refusal(self):
        r = self._run_full()
        self.assertEqual(r['status'], 'complete')
        self.assertNotIn('boat_refusal', self._events())

    def test_operator_stop_never_counts_as_refusal(self):
        self._powered_following()
        self.link.stop(self.link.winch_command_seq + 1)
        r = self._wait_result()
        self.assertIn('STOP', r['reason'])
        self.assertNotIn('boat_refusal', self._events())

    def test_packets_from_before_the_powered_command_are_ignored(self):
        """A zero packet from BEFORE the command can be up to 1.5 s old and
        still pass the precheck's freshness rule. Counted into the interval,
        it would make the first fresh zero after the command look like 1.5 s
        of refusal and abort before the boat had any time to respond."""
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(0.9, boat_follows=False)            # zero reports until t=0.9 s
        self._drive(1.2, boat_follows=False, refresh_ms=False)   # silence: 1.1 s old at the gate
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self._zeros(0.5)                                # fresh zeros 2.1-2.6 s: a NEW, 0.5 s interval
        self.assertIsNotNone(self.link.lake_id, 'the pre-command packet was counted into the interval')
        self._drive(3.0)                                # then it drives
        self.assertIsNotNone(self.link.lake_id)
        self.assertTrue(self.link.lake_id['drive_confirmed'])


class StartTwoPhaseTest(LakeBase):
    """Item 4: the recorder is prepared with the lock RELEASED, and a refused
    start leaves nothing behind but its own cleanup."""

    def _patch_open(self, fail_on):
        real = open
        opened = []

        def fake(path, *a, **k):
            if str(path).endswith(fail_on):
                raise OSError('simulated open failure: %s' % fail_on)
            fh = real(path, *a, **k)
            opened.append(fh)
            return fh
        T.open = fake
        self.addCleanup(lambda: T.__dict__.pop('open', None))
        return opened

    def test_first_file_failure_refuses_and_leaves_no_folder(self):
        self._patch_open('samples.csv')
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('cannot create', err)
        self.assertIsNone(self.link.lake_id)
        self.assertEqual(_lake_folders(self.tmp), [])

    def test_second_file_failure_closes_the_first_and_cleans_up(self):
        opened = self._patch_open('events.csv')
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertEqual(len(opened), 1)
        self.assertTrue(opened[0].closed, 'samples.csv handle leaked')
        self.assertEqual(_lake_folders(self.tmp), [])
        self.assertIsNone(self.link.lake_id)
        self.assertIsNone(self.link._lake_writer)
        T.__dict__.pop('open', None)                    # disk is fine again
        ok, err = self._start()                         # and the next press works
        self.assertTrue(ok, err)

    def test_no_disk_operation_while_the_control_lock_is_held(self):
        held = []
        real_open, real_mkdir = open, Path.mkdir
        lock = self.link._lock

        def fake_open(path, *a, **k):
            held.append(('open', str(path), lock._is_owned()))
            return real_open(path, *a, **k)

        def fake_mkdir(self_, *a, **k):
            held.append(('mkdir', str(self_), lock._is_owned()))
            return real_mkdir(self_, *a, **k)
        T.open = fake_open
        Path.mkdir = fake_mkdir
        self.addCleanup(lambda: (T.__dict__.pop('open', None), setattr(Path, 'mkdir', real_mkdir)))
        ok, err = self._start(); self.assertTrue(ok, err)
        self.assertTrue(any(op == 'mkdir' for op, _p, _h in held), held)
        self.assertTrue(any(op == 'open' for op, _p, _h in held), held)
        self.assertEqual([h for h in held if h[2]], [],
                         'disk I/O while BoatLink._lock was held: %r' % held)

    def test_state_change_during_preparation_refuses_and_cleans_only_its_own(self):
        real_prepare = T.LakeIdWriter.prepare

        def prepare_then_disarm(w):
            real_prepare(w)
            self.link.armed_cmd = False                 # the operator DISARMed meanwhile
        T.LakeIdWriter.prepare = prepare_then_disarm
        self.addCleanup(lambda: setattr(T.LakeIdWriter, 'prepare', real_prepare))
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('ARM', err)
        self.assertIsNone(self.link.lake_id)
        self.assertIsNone(self.link._lake_writer)
        self.assertEqual(_lake_folders(self.tmp), [])

    def test_a_start_that_loses_the_race_cannot_touch_the_winner(self):
        real_prepare = T.LakeIdWriter.prepare
        winner = {}

        def prepare_with_a_competitor(w):
            real_prepare(w)                             # this attempt's folder exists now
            if not winner:
                T.LakeIdWriter.prepare = real_prepare
                winner['result'] = self.link.start_lake_id(
                    0.2, 0.3, self.link.winch_command_seq + 7)   # a second press lands and wins
        T.LakeIdWriter.prepare = prepare_with_a_competitor
        self.addCleanup(lambda: setattr(T.LakeIdWriter, 'prepare', real_prepare))
        ok, err = self._start(seq=self.link.winch_command_seq + 1)
        self.assertFalse(ok, err)
        self.assertEqual(winner['result'], (True, None))
        self.assertIsNotNone(self.link.lake_id)
        self.assertTrue(self.link.lake_id['name'].endswith('_002'), self.link.lake_id['name'])
        self.assertEqual(_lake_folders(self.tmp), [self.link.lake_id['name']])
        for fh in self.link._lake_writer._fh.values():
            self.assertFalse(fh.closed, 'the winner\'s files were closed by the loser')
        with self.link._lock:
            self.link._abort_lake_id_locked('test')
        self._wait_result()

    def test_two_simultaneous_starts_collide_on_the_folder_only(self):
        real_scan = T.lake_id_scan
        barrier = threading.Barrier(2, timeout=5)

        def scan_together(d):
            out = real_scan(d)
            try:
                barrier.wait()                          # both compute the same name
            except threading.BrokenBarrierError:
                pass
            return out
        T.lake_id_scan = scan_together
        self.addCleanup(lambda: setattr(T, 'lake_id_scan', real_scan))
        results = {}

        def go(tag, seq):
            results[tag] = self.link.start_lake_id(0.2, 0.3, seq)
        ta = threading.Thread(target=go, args=('a', 11))
        tb = threading.Thread(target=go, args=('b', 12))
        ta.start(); tb.start(); ta.join(5); tb.join(5)
        oks = [tag for tag, (ok, _e) in results.items() if ok]
        self.assertEqual(len(oks), 1, results)
        loser = [tag for tag in results if tag not in oks][0]
        self.assertIn('cannot create', results[loser][1])
        self.assertIsNotNone(self.link.lake_id)
        self.assertEqual(_lake_folders(self.tmp), [self.link.lake_id['name']])
        d = Path(self.link.lake_id['dir'])
        self.assertTrue((d / 'samples.csv').exists() and (d / 'events.csv').exists())
        T.lake_id_scan = real_scan                      # the finish rescans; no barrier there
        with self.link._lock:
            self.link._abort_lake_id_locked('test')
        self._wait_result()


class RecordingCountsTest(LakeBase):
    """Item 6: summary.json separates what was received, enqueued and written."""

    def test_summary_separates_received_enqueued_and_written(self):
        r = self._run_full()
        rows, events, s = self._files(r['name'])
        rec = s['recording']
        self.assertEqual(rec['frames_received'], rec['sample_rows_enqueued'])
        self.assertEqual(rec['sample_rows_enqueued'], rec['sample_rows_written'])
        self.assertEqual(rec['sample_rows_dropped'], 0)
        self.assertEqual(rec['sample_rows_written'], len(rows))
        self.assertEqual(rec['events_attempted'], rec['events_enqueued'])
        self.assertEqual(rec['events_enqueued'], rec['events_written'])
        self.assertEqual(rec['events_written'], len(events))
        self.assertEqual(rec['events_dropped'], 0)
        self.assertTrue(rec['complete'])
        self.assertEqual(r['recording'], rec)
        self.assertEqual(s['sample_count'], len(rows))     # existing fields survive
        self.assertEqual(s['event_count'], len(events))


class LiveWarningTest(LakeBase):
    """Item 7: the tool says exactly which warnings are live."""

    def test_frozen_imu_warning_is_live_and_never_aborts(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        end = self.clock.t + 1.5
        while self.clock.t < end:
            self.clock.advance(DT)
            self._all_fresh()
            self._boat(0.18, 0.22, 0.0)
            self._tick()
            self._frame(yaw=1.234, heading=77.0)           # identical IMU tuple every frame
        self.assertIsNotNone(self.link.lake_id, 'a frozen IMU aborted the run')
        self.assertTrue(any('identical' in w for w in self.link.lake_id['warnings']),
                        self.link.lake_id['warnings'])

    def test_the_card_and_code_say_which_warnings_are_live(self):
        src = TOOL.read_text()
        i = src.index('id="lake-card"')
        card = src[i:src.index('</details>', i)]
        self.assertIn('Live warnings', card)
        self.assertIn('post-run', card)
        self.assertNotIn('# live warnings (never aborts)', src)


class FirmwareLabelTest(LakeBase):
    """Provenance: the default label must not claim a build nobody verified."""

    def test_default_label_is_honest_about_the_unknown_build(self):
        d = T.LAKE_ID_FIRMWARE_LABEL_DEFAULT
        self.assertNotIn('4e81341b', d)
        self.assertIn('unknown', d)
        self.assertIn('believed', d)
        self.assertIn('1805', d)
        r = self._run_full()
        _r, _e, s = self._files(r['name'])
        self.assertEqual(s['provenance']['firmware_label'], d)
        self.assertIn('not verified', s['provenance']['firmware_label_note'])


# ---------------------------------------------------------------------------
# Mode requirement, 2026-09-05: Motor P ON and Rudder Assist OFF for the whole
# run, confirmed by the boat, never toggled by the tool. Each test failed
# against the OFF/OFF version before the change.


def _lake_rows(order='LR', bias=0.5, gain=8.0, tau=1.0, speed=1.5, gps=True):
    """A first-order boat with a straight-running bias, sampled at 20 Hz."""
    ph = T.lake_id_phases(0.2, 0.3, order)
    rows, yaw, heading, t = [], bias, 90.0, 0.0
    while t < 57.0:
        p = T.lake_id_phase_at(ph, t)
        target = bias + (-p[3] * gain if p[2] > 0 else 0.0)
        yaw += (target - yaw) * (0.05 / tau)
        heading = (heading + yaw * 0.05) % 360
        rows.append({'elapsed_s': round(t, 3), 'phase': p[0], 'yaw_dps': yaw,
                     'heading_deg': heading, 'pitch_deg': 1.0, 'roll_deg': 0.0,
                     'gps_valid': 1 if gps else 0, 'speed_mps': speed, 'course_deg': heading,
                     'satellites': 9, 'hdop': 1.0,
                     'boat_applied_left_cmd': 0.18, 'boat_applied_right_cmd': 0.22,
                     'boat_assist_motor_p': 1, 'boat_assist_rudder': 0})
        t += 0.05
    return rows


def _lake_sum(rows, order='LR'):
    settings = {'throttle': 0.2, 'magnitude': 0.3, 'order': order,
                'condition': 'T20_M30', 'index': 1, 'name': 'x'}
    return T.lake_id_summarize(rows, [], settings, {'git_head': 'abc'}, 'complete', None,
                               stop_confirmed=True, notes={k: 'ok' for k in T.LAKE_ID_NOTE_FIELDS},
                               firmware_label='lbl')


class ModeRequirementTest(LakeBase):

    def test_start_refused_when_the_tool_has_p_off(self):
        self.link.p_assist_on = False
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('Motor P', err)
        self.assertIsNone(self.link.lake_id)

    def test_start_refused_when_the_tool_has_rudder_assist_on(self):
        self.link.assist_rudder_on = True
        ok, err = self._start()
        self.assertFalse(ok)
        self.assertIn('Rudder Assist', err)

    def test_the_gate_needs_the_boats_fresh_confirmation_of_p(self):
        self._boat(0.0, 0.0, 0.0, assist_p=False)     # tool says ON; the boat has not confirmed
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(2.2, boat_follows=False)
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('motor_p_on', r['reason'])
        self.assertEqual(self.link.throttle, 0.0)
        self.assertNotIn(('motor', 0.2, 0.2), self.sent)

    def test_a_late_confirmation_inside_the_window_starts(self):
        self._boat(0.0, 0.0, 0.0, assist_p=False)
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(1.2, boat_follows=False)
        self.assertIn('motor_p_on', self.link.lake_id_status_locked()['precheck_unmet'])
        self._boat(0.0, 0.0, 0.0, assist_p=True)      # the boat confirms at ~1.3 s
        self._drive(1.0, boat_follows=False)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self.assertEqual(self.link.throttle, 0.2)

    def test_mode_confirmation_is_on_the_record(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(3.0)
        with self.link._lock:
            self.link._abort_lake_id_locked('test')
        r = self._wait_result()
        _rows, events, _s = self._files(r['name'])
        names = [e['event'] for e in events]
        self.assertIn('mode_confirmed', names)
        self.assertLess(names.index('precheck_pass'), names.index('mode_confirmed'))

    def test_p_reported_off_mid_run_aborts_and_is_warned(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self.assertEqual(self.link.lake_id['phase'], 'straight')
        self.sent.clear()
        self._boat(0.18, 0.22, 0.0, assist_p=False)
        self._frame(yaw=0.4)
        self._tick()
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('Motor P', r['reason'])
        self.assertIn(('motor', 0.0, 0.0), self.sent)
        self.assertIn(('steer', 0.0), self.sent)
        rows, _events, summary = self._files(r['name'])
        self.assertTrue(rows, 'partial rows were not preserved')
        self.assertTrue(any('Motor P reported OFF' in w for w in summary['warnings']),
                        summary['warnings'])

    def test_rudder_assist_reported_on_mid_run_aborts(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self._boat(0.18, 0.22, 0.0, assist_rudder=True)
        self._tick()
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('Rudder Assist', r['reason'])

    def test_the_tool_side_p_switch_going_off_mid_run_aborts(self):
        ok, err = self._start(); self.assertTrue(ok, err)
        self._drive(5.0)
        self.link.p_assist_on = False
        self._tick()
        r = self._wait_result()
        self.assertEqual(r['status'], 'aborted')
        self.assertIn('Motor P', r['reason'])

    def test_the_tool_sends_no_mode_command_during_the_run(self):
        raw = []
        inner = self.link._write_locked

        def capture(payload):
            raw.append(bytes(payload)); return inner(payload)
        self.link._write_locked = capture
        r = self._run_full()
        self.assertEqual(r['status'], 'complete')
        kinds = []
        for p in raw:
            m = self.link.pb2.BoatMessage(); m.ParseFromString(p)
            kinds.append(m.WhichOneof('payload'))
        self.assertNotIn('assist', kinds)
        self.assertTrue(set(kinds) <= {'motor', 'steer', 'winch'}, kinds)
        self.assertTrue(self.link.p_assist_on)

    def test_the_card_states_the_mode(self):
        src = TOOL.read_text()
        i = src.index('id="lake-card"')
        card = src[i:src.index('</details>', i)]
        self.assertIn('Motor P must be ON', card)
        self.assertIn('Rudder Assist OFF', card)
        self.assertNotIn('Motor P and Rudder Assist must be OFF', card)


class PerformanceSummaryTest(unittest.TestCase):
    """The summary answers the practical questions, or says why it cannot."""

    PHASES = ('straight', 'turn_a', 'recover_a', 'turn_b', 'recover_b')

    def test_every_phase_reports_signed_mean_abs_peak_integrated_and_coverage(self):
        s = _lake_sum(_lake_rows())
        for ph in self.PHASES:
            y = s[ph]['yaw']
            for k in ('signed_mean_yaw_dps', 'mean_abs_yaw_dps', 'peak_yaw_dps',
                      'integrated_yaw_change_deg', 'heading_change_deg', 'coverage'):
                self.assertIn(k, y, (ph, k))
                self.assertIsNotNone(y[k], (ph, k))
            self.assertTrue(y['coverage']['coverage_ok'], ph)
        self.assertGreater(s['turn_a']['yaw']['integrated_yaw_change_deg'], 15.0)   # LEFT: positive
        self.assertLess(s['turn_b']['yaw']['integrated_yaw_change_deg'], -15.0)     # RIGHT: negative
        self.assertGreater(s['turn_a']['yaw']['peak_yaw_dps'], 2.0)
        self.assertAlmostEqual(s['straight']['yaw']['signed_mean_yaw_dps'], 0.5, places=1)
        self.assertAlmostEqual(s['straight']['yaw']['mean_abs_yaw_dps'], 0.5, places=1)
        self.assertTrue(s['turn_a']['settled'])
        self.assertIsNotNone(s['turn_a']['rise_time_s'])

    def test_an_unsettled_turn_reports_no_rise_or_delay(self):
        rows = _lake_rows()
        for i, r in enumerate(rows):
            if 17.0 <= r['elapsed_s'] < 22.0:                 # oscillating through the steady window
                r['yaw_dps'] += 3.0 * math.sin(i * 0.9)
        s = _lake_sum(rows)
        ta = s['turn_a']
        self.assertFalse(ta['settled'])
        self.assertIsNone(ta['rise_time_s'])
        self.assertIsNone(ta['response_delay_s'])
        self.assertIn('unsettled', ta['unavailable_reason'])
        self.assertTrue(any('turn_a' in w and 'unsettled' in w for w in s['warnings']), s['warnings'])
        self.assertIsNotNone(ta['yaw']['mean_abs_yaw_dps'])    # the plain facts are still there

    def test_insufficient_coverage_flags_instead_of_numbers(self):
        rows = [r for r in _lake_rows() if not (14.0 <= r['elapsed_s'] < 21.0)]   # a 7 s hole
        s = _lake_sum(rows)
        ta = s['turn_a']
        self.assertFalse(ta['yaw']['coverage']['coverage_ok'])
        self.assertIsNone(ta['rise_time_s'])
        self.assertIsNone(ta['response_delay_s'])
        self.assertIsNone(ta['steady_yaw_minus_bias_dps'])
        self.assertIn('coverage', ta['unavailable_reason'])
        self.assertTrue(any('turn_a' in w and 'coverage' in w for w in s['warnings']))

    def test_a_recovery_that_never_settles_reports_no_time(self):
        rows = _lake_rows()
        for r in rows:
            if 22.0 <= r['elapsed_s'] < 32.0:
                r['yaw_dps'] = 0.5 + 2.4                     # keeps rotating after centring
        s = _lake_sum(rows)
        ra = s['recover_a']
        self.assertIsNone(ra['recovery_time_s'])
        self.assertIsNone(ra['half_decay_time_s'])
        self.assertIn('settle', ra['unavailable_reason'])
        self.assertIn('Motor P and autotrim active', ra['note'])
        self.assertAlmostEqual(ra['yaw']['mean_abs_yaw_dps'], 2.9, places=1)
        self.assertTrue(any('recover_a' in w for w in s['warnings']))

    def test_a_settled_recovery_reports_time_half_decay_and_residual(self):
        ra = _lake_sum(_lake_rows())['recover_a']
        self.assertIsNotNone(ra['recovery_time_s'])
        self.assertIsNotNone(ra['half_decay_time_s'])
        self.assertLess(ra['half_decay_time_s'], ra['recovery_time_s'])
        self.assertAlmostEqual(ra['half_decay_time_s'], 0.69, delta=0.15)     # ln 2 * tau
        self.assertIsNotNone(ra['residual_yaw_dps'])
        self.assertLess(abs(ra['residual_yaw_dps']), 0.2)
        self.assertIsNone(ra['unavailable_reason'])

    def test_p_reported_off_in_rows_is_a_summary_warning(self):
        rows = _lake_rows()
        for r in rows[400:420]:
            r['boat_assist_motor_p'] = 0
        s = _lake_sum(rows)
        self.assertTrue(any('Motor P reported OFF' in w for w in s['warnings']))
        self.assertFalse(any('Motor P reported ON' in w for w in s['warnings']))

    def test_wording_states_combined_system_and_unrecorded_values(self):
        s = _lake_sum(_lake_rows())
        w = ' '.join(s['wording']).lower()
        self.assertIn('does not isolate', w)
        self.assertIn('waypoint', w)
        self.assertIn('turn radius is provisional', w)
        self.assertEqual(s['mode']['motor_p'], 'ON')
        self.assertEqual(s['mode']['rudder_assist'], 'OFF')
        self.assertFalse(s['mode']['p_correction_recorded'])
        self.assertFalse(s['mode']['learned_c_recorded'])
        self.assertIn('not the motor P filter', s['mode']['limitation'])
        head = '\n'.join(T.LAKE_ID_SAMPLES_HEADER)
        self.assertIn('not the motor P filter', head)
        self.assertIn('not proof of a nonzero', head)
        self.assertIn('not recorded', head)
        for k in ('turn_settled_max_std_frac', 'recovery_hold_s', 'half_decay_frac'):
            self.assertIn(k, s['rules'])

    def test_a_single_dip_into_the_band_is_not_a_recovery(self):
        """The hold rule: the boat must STAY inside the band, not touch it once
        while still swinging."""
        rows = _lake_rows()
        for r in rows:
            t = r['elapsed_s'] - 22.0                     # recover_a, phase-relative
            if 0.0 <= t < 10.0:
                if 0.95 <= t < 1.05:
                    r['yaw_dps'] = 0.5 + 0.1              # one sample deep inside the band
                elif t < 5.0:
                    r['yaw_dps'] = 0.5 + 2.0              # still swinging
                else:
                    r['yaw_dps'] = 0.5 + 0.1              # settled from 5 s on
        ra = _lake_sum(rows)['recover_a']
        self.assertIsNotNone(ra['recovery_time_s'])
        self.assertGreaterEqual(ra['recovery_time_s'], 4.9)
        self.assertLess(ra['recovery_time_s'], 5.2)
