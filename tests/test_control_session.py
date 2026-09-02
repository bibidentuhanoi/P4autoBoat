"""The control session, the drive lease, and the ways control must end.

WHY THIS EXISTS. tools/espnow_drive.py streams commands to the boat ON THE
BROWSER'S BEHALF. That is the whole hazard: close the tab at 40% throttle and
Python carries on transmitting 40% for as long as it is running, because the
boat's own link failsafe sees a perfectly healthy stream of commands. The
browser dying is invisible from the boat.

So the browser has to keep saying it is alive, and this process has to stop
driving when it stops saying so. One session at a time, a monotonic sequence,
a full-state heartbeat, and a short lease.

(dashboard.html needs none of this and does not have it. There the browser IS
the sender, so if it dies the commands simply stop and the firmware's own
CONTROL_LINK_TIMEOUT_US failsafe zeroes the boat. A ground-station lease on a
path with no intermediary would guard nothing --
test_dashboard_needs_no_lease_because_it_has_no_intermediary pins that
reasoning so nobody "fixes" the asymmetry later without understanding it.)

Everything here runs on a fake clock. No sleeps, no threads, no serial.
"""

import importlib.util
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'espnow_drive.py'


def _load():
    spec = importlib.util.spec_from_file_location('espnow_drive_session', TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


T = _load()


class Clock:
    def __init__(self, t=5000.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


class SessionBase(unittest.TestCase):

    def setUp(self):
        self.clock = Clock()
        self.sent = []
        link = T.BoatLink.__new__(T.BoatLink)
        link._lock = threading.RLock()
        link._now = self.clock
        link.pb2 = T.load_boat_pb2()
        link.connected = True
        link.throttle = 0.0
        link.motor_left = 0.0
        link.motor_right = 0.0
        link.motor_split = False
        link.rudder = 0.0
        link.winch_speed = 0.0
        link.winch_lease_until = 0.0
        link.winch_command_seq = 0
        link.armed_cmd = False
        link.force = False
        link.calibrating = False
        link.servo_rail_cut = None
        link.p_assist_on = False
        link.assist_rudder_on = False
        link.assist_request_id = 0
        link._assist_req_seq = 0
        link._assist_off_req_id = None
        link._assist_off_next_retry = 0.0
        link.session_id = None
        link.session_seq = 0
        link.session_last_hb = 0.0
        link.lease_expired_at = None
        link.telemetry = T.BoatLink._blank_telemetry()
        link.motor_status = T.BoatLink._blank_motor_status()
        link.bench_status = T.BoatLink._blank_bench_status()
        link.calibrate_status = T.BoatLink._blank_calibrate_status()
        link.system_status = T.BoatLink._blank_system_status()
        link.bridge_status = T.BoatLink._blank_bridge_status()
        link.bench_yaw_samples = []
        link.bench_yaw = None
        link.rudder_test = None
        link.rudder_test_result = None
        link._rudder_test_write = None
        link.bench_run = None
        link._bench_write = None
        link.bench_csv_name = None
        link.bench_dir = Path('/tmp')
        link.rudder_test_dir = Path('/tmp')
        link.seq = 0
        link.last_error = None
        link.ser = None
        link.port = '/dev/null'
        link._close_locked = lambda: setattr(link, 'connected', False)

        def _write(payload):
            msg = link.pb2.BoatMessage()
            try:
                msg.ParseFromString(payload)
            except Exception:                            # noqa: BLE001
                return True
            which = msg.WhichOneof('payload')
            if which == 'motor':
                self.sent.append(('motor', round(msg.motor.left, 4),
                                  round(msg.motor.right, 4)))
            elif which == 'steer':
                self.sent.append(('steer', round(msg.steer.left, 4)))
            elif which == 'winch':
                self.sent.append(('winch', round(msg.winch.speed, 4)))
            elif which == 'arm_cmd':
                self.sent.append(('arm', msg.arm_cmd.arm))
            elif which == 'assist':
                self.sent.append(('assist', msg.assist.rudder_assist,
                                  msg.assist.request_id))
            return True
        link._write_locked = _write
        self.link = link

    # ---- helpers --------------------------------------------------------

    def _open(self):
        return self.link.open_control_session()

    def _hb(self, **kw):
        self.link.session_seq += 0            # readability only
        return self.link.control_heartbeat(
            self.link.session_id, self.link.session_seq + 1, **kw)

    def _tick(self):
        """One stream-loop iteration's worth of lease enforcement."""
        with self.link._lock:
            now = self.clock.t
            self.link._assist_off_tick_locked(now)
            self.link._rudder_test_tick_locked(now)
            if (self.link.rudder_test is None
                    and not self.link._drive_lease_ok_locked(now)
                    and self.link._anything_commanded_locked()):
                self.link.lease_expired_at = now
                self.link._zero_controls_locked()
                self.link._transmit_zeros_locked()

    def _assert_stopped(self):
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.motor_left, 0.0)
        self.assertEqual(self.link.motor_right, 0.0)
        self.assertEqual(self.link.rudder, 0.0)
        self.assertEqual(self.link.winch_speed, 0.0)

    def _zeros_were_transmitted(self):
        return (('motor', 0.0, 0.0) in self.sent
                and ('steer', 0.0) in self.sent)


class SessionBasicsTest(SessionBase):

    def test_a_session_id_is_random_not_sequential(self):
        a = self._open()
        b = self._open()
        self.assertNotEqual(a, b)
        self.assertGreaterEqual(len(a), 16)

    def test_claiming_control_starts_from_a_stopped_boat(self):
        """Taking over must never inherit somebody else's throttle."""
        self.link.throttle = 0.4
        self._open()
        self._assert_stopped()
        self.assertTrue(self._zeros_were_transmitted())

    def test_a_heartbeat_from_the_wrong_session_is_refused(self):
        self._open()
        ok, err = self.link.control_heartbeat('some-other-session', 1,
                                              throttle=0.4)
        self.assertFalse(ok)
        self.assertEqual(self.link.throttle, 0.0)

    def test_a_reload_supersedes_the_old_session(self):
        old = self._open()
        self.link.control_heartbeat(old, 1, throttle=0.4)
        self.assertAlmostEqual(self.link.throttle, 0.4)
        new = self._open()                     # the page reloaded
        self._assert_stopped()
        ok, _ = self.link.control_heartbeat(old, 2, throttle=0.4)
        self.assertFalse(ok, 'a stale tab kept driving alongside the new one')
        self.assertNotEqual(old, new)

    def test_a_stale_sequence_is_refused(self):
        s = self._open()
        self.assertTrue(self.link.control_heartbeat(s, 5, throttle=0.4)[0])
        ok, err = self.link.control_heartbeat(s, 4, throttle=0.0)
        self.assertFalse(ok)
        message, code = err
        self.assertEqual(code, 'stale_seq')
        self.assertAlmostEqual(self.link.throttle, 0.4)
        # ...and the SESSION survives: an out-of-order arrival is not a dead
        # session, and tearing one down over it stops the boat for no reason.
        self.assertEqual(self.link.session_id, s)
        self.assertTrue(self.link.control_heartbeat(s, 6, throttle=0.3)[0])

    def test_the_heartbeat_carries_full_state_not_deltas(self):
        """A dropped delta would leave the boat holding a value nobody is
        asking for any more."""
        s = self._open()
        self.link.control_heartbeat(s, 1, throttle=0.4, rudder=0.5)
        self.link.control_heartbeat(s, 2, throttle=0.0, rudder=0.0)
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.rudder, 0.0)


class DriveLeaseTest(SessionBase):

    def test_browser_loss_at_forty_percent_throttle_zeroes_the_boat(self):
        """THE named case. The tab closes mid-drive and this process keeps
        streaming 40% because the boat cannot tell the difference."""
        s = self._open()
        self.link.control_heartbeat(s, 1, throttle=0.4)
        self.assertAlmostEqual(self.link.throttle, 0.4)

        self.sent.clear()
        self.clock.advance(T.CONTROL_LEASE_S + 0.05)   # browser gone
        self._tick()

        self._assert_stopped()
        self.assertTrue(self._zeros_were_transmitted(),
                        'the zeros were not put on the wire immediately')

    def test_the_lease_holds_while_heartbeats_keep_coming(self):
        s = self._open()
        for i in range(1, 40):
            self.clock.advance(1.0 / T.CONTROL_HEARTBEAT_HZ)
            self.assertTrue(
                self.link.control_heartbeat(s, i, throttle=0.4)[0])
            self._tick()
            self.assertAlmostEqual(self.link.throttle, 0.4,
                                   msg='the lease dropped a live browser')

    def test_the_lease_is_shorter_than_a_person_would_notice(self):
        self.assertLessEqual(T.CONTROL_LEASE_S, 0.5)
        # and long enough that ordinary jitter at the heartbeat rate is safe
        self.assertGreater(T.CONTROL_LEASE_S, 3.0 / T.CONTROL_HEARTBEAT_HZ)

    def test_the_heartbeat_rate_is_in_the_agreed_band(self):
        self.assertGreaterEqual(T.CONTROL_HEARTBEAT_HZ, 10)
        self.assertLessEqual(T.CONTROL_HEARTBEAT_HZ, 15)

    def test_an_expired_lease_does_not_thrash_the_wire(self):
        """Once zeroed there is nothing left to zero, so it must go quiet."""
        s = self._open()
        self.link.control_heartbeat(s, 1, throttle=0.4)
        self.clock.advance(T.CONTROL_LEASE_S + 0.05)
        self._tick()
        self.sent.clear()
        for _ in range(10):
            self.clock.advance(0.05)
            self._tick()
        self.assertEqual(self.sent, [])

    def test_a_rudder_test_carries_its_own_authority(self):
        """It drives itself and is not browser input, so the lease must not
        cut it off -- it has its own interlocks and abort routes."""
        self.link.session_id = None                 # no browser at all
        self.link.motor_status = dict(
            self.link.motor_status, have=True, state=2, servo_power=True,
            last_rx_monotonic=self.clock.t)
        self.link.armed_cmd = True
        ok, err = self.link.start_rudder_test(-1, 1)
        self.assertTrue(ok, err)
        self.clock.advance(1.0)
        self._tick()
        self.assertIsNotNone(self.link.rudder_test)
        self.assertNotEqual(self.link.rudder, 0.0)


class StopTest(SessionBase):

    def test_stop_is_never_refused_as_stale(self):
        """The one refusal that can leave the boat running."""
        s = self._open()
        self.link.control_heartbeat(s, 50, throttle=0.4)
        self.link.winch_command_seq = 999
        ok, err = self.link.stop(1)               # far behind
        self.assertTrue(ok, err)
        self._assert_stopped()

    def test_a_delayed_state_request_after_stop_cannot_restore_throttle(self):
        """THE named case. /api/state was already in flight when STOP landed."""
        s = self._open()
        self.link.control_heartbeat(s, 1, throttle=0.4)
        self.link.stop(2)
        self._assert_stopped()

        ok, err = self.link.control_heartbeat(s, 3, throttle=0.4)
        self.assertFalse(ok, 'a delayed request restored the throttle')
        self._assert_stopped()

    def test_stop_immediately_after_a_reload_still_stops(self):
        """THE named case. The page reloaded, so the browser's own sequence
        counter restarted -- STOP must not be judged against it."""
        s = self._open()
        self.link.control_heartbeat(s, 500, throttle=0.4)
        self._open()                              # reload
        self.link.throttle = 0.4                  # ...and it was still moving
        ok, err = self.link.stop(1)               # fresh counter, low number
        self.assertTrue(ok, err)
        self._assert_stopped()

    def test_stop_transmits_the_zeros_immediately(self):
        s = self._open()
        self.link.control_heartbeat(s, 1, throttle=0.4)
        self.sent.clear()
        self.link.stop(2)
        self.assertTrue(self._zeros_were_transmitted())

    def test_stop_drops_the_session_so_driving_needs_a_deliberate_restart(self):
        s = self._open()
        self.link.stop(1)
        self.assertIsNone(self.link.session_id)
        ok, _ = self.link.control_heartbeat(s, 2, throttle=0.4)
        self.assertFalse(ok)


class ArmDisarmTest(SessionBase):

    def test_a_lost_disarm_still_leaves_the_boat_stopped(self):
        """THE named case. The disarm packet never arrives, but the zeros were
        sent first and the stream keeps repeating them, so a lost disarm
        degrades to a stopped boat rather than a running one."""
        s = self._open()
        self.link.control_heartbeat(s, 1, throttle=0.4)
        self.link.armed_cmd = True

        self.sent.clear()
        self.link.arm(False, False)

        self._assert_stopped()
        zero_i = [i for i, m in enumerate(self.sent)
                  if m[0] == 'motor' and m[1] == 0.0 and m[2] == 0.0]
        arm_i = [i for i, m in enumerate(self.sent) if m[0] == 'arm']
        self.assertTrue(zero_i, 'no zeros were sent on disarm')
        if arm_i:
            self.assertLess(min(zero_i), min(arm_i),
                            'the disarm went out before the boat was zeroed')

        # ...and the stream keeps saying zero regardless of the disarm landing
        for _ in range(5):
            self.clock.advance(0.05)
            self._tick()
        self._assert_stopped()

    def test_disarm_also_drops_the_session(self):
        s = self._open()
        self.link.control_heartbeat(s, 1, throttle=0.4)
        self.link.arm(False, False)
        ok, _ = self.link.control_heartbeat(s, 2, throttle=0.4)
        self.assertFalse(ok)

    def test_arming_is_refused_unless_every_control_is_zero(self):
        """Arming spins thrusters; it must never start against a held stick."""
        s = self._open()
        self.link.control_heartbeat(s, 1, throttle=0.4)
        ok, err = self.link.arm(True, False)
        self.assertFalse(ok)
        self.assertIn('zero', err.lower())
        self.assertFalse(self.link.armed_cmd)

    def test_arming_is_allowed_from_a_stopped_boat(self):
        self._open()
        ok, err = self.link.arm(True, False)
        self.assertTrue(ok, err)
        self.assertTrue(self.link.armed_cmd)

    def test_a_held_rudder_also_blocks_arming(self):
        s = self._open()
        self.link.control_heartbeat(s, 1, rudder=0.5)
        self.assertFalse(self.link.arm(True, False)[0])


class AssistOffAcknowledgedTest(SessionBase):

    def test_a_lost_assisted_off_is_retried_until_confirmed(self):
        """THE named case. An unacknowledged OFF leaves the boat steering
        itself while the operator believes control is manual."""
        self.link.assist_rudder_on = True
        ok, err = self.link.send_rudder_assist(False)
        self.assertTrue(ok, err)
        req = self.link._assist_off_req_id
        self.assertIsNotNone(req)

        # The boat says nothing. Retries must keep going out.
        self.sent.clear()
        for _ in range(20):
            self.clock.advance(0.1)
            with self.link._lock:
                self.link._assist_off_tick_locked(self.clock.t)
        offs = [m for m in self.sent
                if m[0] == 'assist' and m[1] is False and m[2] == req]
        self.assertGreaterEqual(len(offs), 3, 'the OFF was sent once and dropped')

    def test_manual_control_is_blocked_while_off_is_unconfirmed(self):
        """Until the boat confirms, the mode its rudder is in is unknown -- and
        a stick value is a yaw RATE in one mode and an ANGLE in the other."""
        s = self._open()
        self.link.assist_rudder_on = True
        self.link.send_rudder_assist(False)
        ok, err = self.link.control_heartbeat(s, 1, throttle=0.4, rudder=0.5)
        self.assertFalse(ok)
        message, code = err
        self.assertEqual(code, 'assist_off_pending')
        self.assertIn('assisted', message.lower())
        self._assert_stopped()
        # The session survives and the lease was refreshed -- the boat is HELD
        # through the transition, not dropped.
        self.assertEqual(self.link.session_id, s)

    def test_a_matching_confirmation_clears_it_and_restores_control(self):
        s = self._open()
        self.link.assist_rudder_on = True
        self.link.send_rudder_assist(False)
        req = self.link._assist_off_req_id

        self.link.motor_status = dict(
            self.link.motor_status, have=True, assist_rudder=False,
            assist_request_id=req, last_rx_monotonic=self.clock.t)
        with self.link._lock:
            self.link._assist_off_tick_locked(self.clock.t)

        self.assertIsNone(self.link._assist_off_req_id)
        self.assertFalse(self.link.assist_rudder_on)
        self.assertTrue(self.link.control_heartbeat(s, 1, throttle=0.4)[0])

    def test_a_confirmation_for_a_DIFFERENT_request_does_not_clear_it(self):
        self.link.assist_rudder_on = True
        self.link.send_rudder_assist(False)
        req = self.link._assist_off_req_id
        self.link.motor_status = dict(
            self.link.motor_status, have=True, assist_rudder=False,
            assist_request_id=req + 12345, last_rx_monotonic=self.clock.t)
        with self.link._lock:
            self.link._assist_off_tick_locked(self.clock.t)
        self.assertIsNotNone(self.link._assist_off_req_id)

    def test_the_retry_reuses_ONE_id_so_a_late_ack_still_counts(self):
        """A fresh id per retry would make every acknowledgement answer a
        request that had already been superseded."""
        self.link.assist_rudder_on = True
        self.link.send_rudder_assist(False)
        req = self.link._assist_off_req_id
        self.sent.clear()
        for _ in range(10):
            self.clock.advance(0.1)
            with self.link._lock:
                self.link._assist_off_tick_locked(self.clock.t)
        ids = {m[2] for m in self.sent if m[0] == 'assist'}
        self.assertEqual(ids, {req})


class DashboardInterfaceTest(unittest.TestCase):
    """The dashboard is a driving interface and stays one. Its safety comes
    from somewhere else, and that reasoning is worth pinning."""

    def test_dashboard_needs_no_lease_because_it_has_no_intermediary(self):
        """In dashboard.html the BROWSER is the sender: it posts commands
        straight to the boat over its own WebSocket. If it dies the commands
        stop, and the firmware's CONTROL_LINK_TIMEOUT_US failsafe zeroes the
        boat -- the same guarantee the Python lease provides, one layer down.

        espnow_drive.py needs the lease precisely because it is an
        intermediary that keeps streaming after its browser is gone.

        If the dashboard ever grows a server-side command relay, it will need
        a lease too, and this test should be the thing that catches it."""
        dash = (ROOT / 'main' / 'dashboard.html').read_text()
        # It sends continuously while driving, straight from the browser.
        self.assertIn('setInterval(sendMotorCommand', dash)
        self.assertIn('ws.send(', dash)
        # And no relay process sits between it and the boat.
        self.assertNotIn('/api/state', dash)

        fw = (ROOT / 'main' / 'motor_control.c').read_text()
        self.assertIn('CONTROL_LINK_TIMEOUT_US', fw)
        self.assertIn('control_link_alive()', fw)


if __name__ == '__main__':
    unittest.main()
