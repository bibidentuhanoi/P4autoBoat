"""Browser lifecycle, driven through the REAL HTTP server.

These go over the wire on purpose. The session, the sequence and the refusal
codes are a contract BETWEEN the page and this process, and every bug they
guard lived in that gap rather than inside either half: a response arriving out
of order, a tab coming back and resuming a session it should not have, an ARM
with no session to drive it. Calling BoatLink directly would have proved none
of them.

The JavaScript half is exercised the only way it can be from here -- by
replaying the exact request sequence the page makes, and by pinning the
handlers that produce it. Where a test depends on page behaviour it says so.
"""

import importlib.util
import json
import re
import threading
import unittest
from pathlib import Path
from urllib import error as urllib_error
from urllib import request as urllib_request

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'espnow_drive.py'

SPEC = importlib.util.spec_from_file_location('espnow_drive_lifecycle', TOOL)
D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D)


class BrowserHttpTest(unittest.TestCase):
    """A real server, a real socket, and a link with no serial port."""

    def setUp(self):
        link = D.BoatLink.__new__(D.BoatLink)
        link._lock = threading.RLock()
        link._now = __import__('time').monotonic
        link.pb2 = D.load_boat_pb2()
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
        link.telemetry = D.BoatLink._blank_telemetry()
        link.motor_status = D.BoatLink._blank_motor_status()
        link.bench_status = D.BoatLink._blank_bench_status()
        link.calibrate_status = D.BoatLink._blank_calibrate_status()
        link.system_status = D.BoatLink._blank_system_status()
        link.bridge_status = D.BoatLink._blank_bridge_status()
        link.bench_yaw_samples = []
        link.bench_yaw = None
        link.rudder_test = None
        link.rudder_test_result = None
        link._rudder_test_write = None
        link.rudder_test_dir = Path('/tmp')
        link.seq = 0
        link.last_error = None
        link.ser = None
        link.port = '/dev/null'
        link._close_locked = lambda: None
        self.sent = []
        link._write_locked = lambda p: (self.sent.append(p), True)[1]
        self.link = link

        class TestHandler(D.Handler):
            pass
        TestHandler.link = link
        self.server = D.ThreadingHTTPServer(('127.0.0.1', 0), TestHandler)
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.start()
        host, port = self.server.server_address
        self.base = 'http://%s:%d' % (host, port)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    # ---- helpers --------------------------------------------------------

    def post(self, path, body=None):
        req = urllib_request.Request(
            self.base + path, data=json.dumps(body or {}).encode(),
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urllib_request.urlopen(req, timeout=2.0) as r:
                return r.status, json.load(r)
        except urllib_error.HTTPError as exc:
            return exc.code, json.load(exc)

    def open_session(self):
        code, body = self.post('/api/session')
        self.assertEqual(code, 200)
        self.assertTrue(body['ok'])
        return body['session_id']

    def hb(self, sid, seq, **kw):
        payload = {'session_id': sid, 'seq': seq}
        payload.update(kw)
        return self.post('/api/state', payload)

    def assert_stopped(self):
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.motor_left, 0.0)
        self.assertEqual(self.link.motor_right, 0.0)
        self.assertEqual(self.link.rudder, 0.0)


class HideAndReturnTest(BrowserHttpTest):

    def test_hiding_the_tab_clears_controls_and_drops_the_session(self):
        sid = self.open_session()
        self.hb(sid, 1, throttle=0.4)
        self.assertAlmostEqual(self.link.throttle, 0.4)

        self.post('/api/release')            # what visibilitychange sends
        self.assert_stopped()
        self.assertIsNone(self.link.session_id)

        code, body = self.hb(sid, 2, throttle=0.4)
        self.assertEqual(code, 409)
        self.assertEqual(body['code'], 'no_session')
        self.assert_stopped()

    def test_returning_gets_a_fresh_session_and_the_throttle_does_NOT_restart(self):
        """THE named case. Coming back must not resume whatever the sliders
        happen to show -- switching tabs back is not a command to drive."""
        first = self.open_session()
        self.hb(first, 1, throttle=0.4)
        self.post('/api/release')            # hidden

        second = self.open_session()         # visible again
        self.assertNotEqual(second, first)
        self.assert_stopped()

        # ...and it stays stopped until the operator actually asks
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.hb(second, 1, throttle=0.0)[0], 200)
        self.assert_stopped()

    def test_release_does_not_abort_a_calibration(self):
        """STOP kills a calibration; hiding a tab must not. That is the whole
        reason /api/release exists as a separate endpoint."""
        self.link.calibrating = True
        self.open_session()
        self.post('/api/release')
        self.assertTrue(self.link.calibrating)

    def test_the_page_releases_on_hide_and_reacquires_on_return(self):
        """The page half of the same behaviour."""
        src = TOOL.read_text()
        m = re.search(r"visibilitychange', \(\) => \{(.*?)\n\}\);", src, re.S)
        self.assertIsNotNone(m, 'the visibilitychange handler moved')
        body = m.group(1)
        self.assertIn('releaseControl(true)', body)
        self.assertIn('openSession()', body)
        rel = re.search(r'function releaseControl\(beacon\) \{(.*?)\n\}', src, re.S)
        self.assertIsNotNone(rel)
        self.assertIn('sessionId = null', rel.group(1))
        self.assertIn('/api/release', rel.group(1))
        self.assertIn("throttle: 0", rel.group(1))


class ReloadWhileConnectedTest(BrowserHttpTest):

    def test_reloading_while_connected_starts_from_zero(self):
        """THE named case."""
        first = self.open_session()
        self.hb(first, 5, throttle=0.4, rudder=0.3)
        self.assertAlmostEqual(self.link.throttle, 0.4)

        second = self.open_session()         # the page reloaded
        self.assertNotEqual(second, first)
        self.assert_stopped()

        # The reloaded page's sequence restarts at 1 and must be accepted:
        # sequences are per-session, not global.
        self.assertEqual(self.hb(second, 1, throttle=0.2)[0], 200)
        self.assertAlmostEqual(self.link.throttle, 0.2)

    def test_the_old_page_cannot_keep_driving_after_the_reload(self):
        first = self.open_session()
        second = self.open_session()
        code, body = self.hb(first, 99, throttle=0.4)
        self.assertEqual(code, 409)
        self.assertEqual(body['code'], 'wrong_session')
        self.assert_stopped()
        self.assertEqual(self.hb(second, 1, throttle=0.1)[0], 200)


class StopDisarmArmDriveTest(BrowserHttpTest):

    def test_stop_then_arm_then_drive(self):
        """THE named case. STOP drops the session by design, so ARM has to
        acquire a new one or the boat arms with nothing able to drive it."""
        sid = self.open_session()
        self.hb(sid, 1, throttle=0.4)
        self.post('/api/stop', {'seq': 1})
        self.assert_stopped()
        self.assertIsNone(self.link.session_id)

        fresh = self.open_session()          # what the ARM handler does
        code, body = self.post('/api/arm', {'arm': True, 'force': True})
        self.assertEqual(code, 200, body)
        self.assertTrue(self.link.armed_cmd)

        self.assertEqual(self.hb(fresh, 1, throttle=0.3)[0], 200)
        self.assertAlmostEqual(self.link.throttle, 0.3)

    def test_disarm_then_arm_then_drive(self):
        """THE named case, the other way in."""
        sid = self.open_session()
        self.hb(sid, 1, throttle=0.4)
        self.post('/api/arm', {'arm': True, 'force': True})
        self.post('/api/arm', {'arm': False, 'force': False})
        self.assert_stopped()
        self.assertIsNone(self.link.session_id)

        fresh = self.open_session()
        self.assertEqual(self.post('/api/arm', {'arm': True, 'force': True})[0], 200)
        self.assertEqual(self.hb(fresh, 1, throttle=0.25)[0], 200)
        self.assertAlmostEqual(self.link.throttle, 0.25)

    def test_arming_against_a_held_throttle_is_refused(self):
        sid = self.open_session()
        self.hb(sid, 1, throttle=0.4)
        code, body = self.post('/api/arm', {'arm': True, 'force': True})
        self.assertEqual(code, 409)
        self.assertIn('zero', body['error'].lower())
        self.assertFalse(self.link.armed_cmd)

    def test_the_page_acquires_a_session_before_arming(self):
        src = TOOL.read_text()
        m = re.search(r"\$\('arm-btn'\)\.addEventListener.*?\n\}\);", src, re.S)
        self.assertIsNotNone(m)
        body = m.group(0)
        self.assertIn('openSession()', body)
        self.assertIn('sessionId = null', body)   # disarm gives it up again


class ReorderedResponsesTest(BrowserHttpTest):

    def test_an_out_of_order_heartbeat_does_not_destroy_the_session(self):
        """THE named case. Two heartbeats in flight can be answered out of
        order. The loser is judged against a sequence the winner has already
        advanced -- and tearing the session down over that would stop the boat
        for nothing."""
        sid = self.open_session()
        self.assertEqual(self.hb(sid, 5, throttle=0.4)[0], 200)

        code, body = self.hb(sid, 4, throttle=0.4)     # arrived late
        self.assertEqual(code, 409)
        self.assertEqual(body['code'], 'stale_seq')

        # The session is alive and the boat is still doing what was asked.
        self.assertEqual(self.link.session_id, sid)
        self.assertAlmostEqual(self.link.throttle, 0.4)
        self.assertEqual(self.hb(sid, 6, throttle=0.35)[0], 200)
        self.assertAlmostEqual(self.link.throttle, 0.35)

    def test_assist_off_pending_does_not_destroy_the_session_either(self):
        sid = self.open_session()
        self.link.assist_rudder_on = True
        self.link.send_rudder_assist(False)

        code, body = self.hb(sid, 1, throttle=0.4)
        self.assertEqual(code, 409)
        self.assertEqual(body['code'], 'assist_off_pending')
        self.assertEqual(self.link.session_id, sid)
        self.assert_stopped()

        # and the lease was refreshed, so the boat is HELD through the
        # transition rather than dropped
        self.assertIsNone(self.link.lease_expired_at)

    def test_only_a_dead_session_makes_the_page_give_up(self):
        """The page half: stale_seq and assist_off_pending must NOT clear
        sessionId, or a single reordered response stops the boat."""
        src = TOOL.read_text()
        m = re.search(r'async function sendHeartbeat\(\) \{(.*?)\n\}', src, re.S)
        self.assertIsNotNone(m)
        body = m.group(1)
        self.assertIn("r.code === 'no_session' || r.code === 'wrong_session'",
                      body)
        # the teardown is inside that branch, not at the top level
        after = body.split("r.code === 'no_session'", 1)[1]
        self.assertIn('sessionId = null', after)

    def test_heartbeats_are_serialized_and_coalesced(self):
        """One request outstanding at a time; anything asked for meanwhile
        collapses into a single follow-up carrying the LATEST state. Full-state
        heartbeats are what make coalescing safe."""
        src = TOOL.read_text()
        m = re.search(r'async function sendHeartbeat\(\) \{(.*?)\n\}', src, re.S)
        body = m.group(1)
        self.assertIn('if (hbInFlight) { hbPending = true; return; }', body)
        self.assertIn('hbInFlight = true;', body)
        self.assertIn('finally', body)
        self.assertIn('hbInFlight = false;', body)
        self.assertIn('if (hbPending) { hbPending = false; sendHeartbeat(); }',
                      body)


class DashboardSliderReleaseTest(unittest.TestCase):
    """THE named case, on the other UI.

    dashboard.html has no session and needs none -- the browser is the sender,
    so its safety is the firmware's own CONTROL_LINK_TIMEOUT_US. But that makes
    the motor stream a HOLD: while a non-zero command stands it must keep going
    out, or the firmware times out and the motors cut while the slider still
    reads 40%.
    """

    def setUp(self):
        self.src = (ROOT / 'main' / 'dashboard.html').read_text()

    def test_releasing_the_slider_does_not_stop_a_nonzero_stream(self):
        m = re.search(r"\$\(id\)\.addEventListener\('change', \(\) => \{(.*?)\}\);",
                      self.src, re.S)
        self.assertIsNotNone(m, 'the slider release handler moved')
        body = m.group(1)
        self.assertIn('syncMotorHeartbeat()', body)
        self.assertNotIn('clearInterval', body,
                         'release still kills the stream outright; letting go '
                         'at 40% would cut the motors 400 ms later')

    def test_the_stream_is_kept_while_the_command_is_nonzero(self):
        m = re.search(r'function syncMotorHeartbeat\(\) \{(.*?)\n\}',
                      self.src, re.S)
        self.assertIsNotNone(m)
        body = m.group(1)
        self.assertIn('motorCommandIsZero()', body)
        self.assertIn('stopMotorHeartbeat()', body)
        self.assertIn('startMotorHeartbeat()', body)

    def test_zero_stops_the_stream(self):
        """At zero there is nothing to hold, and the firmware timeout is the
        correct outcome rather than a failure."""
        m = re.search(r'function motorCommandIsZero\(\) \{(.*?)\n\}',
                      self.src, re.S)
        self.assertIsNotNone(m)
        body = m.group(1)
        self.assertIn("motor-left", body)
        self.assertIn("motor-right", body)
        # returning to zero re-syncs, so the hold ends
        dbl = re.search(r"addEventListener\('dblclick', \(\) => \{(.*?)\}\);",
                        self.src, re.S)
        self.assertIn('syncMotorHeartbeat()', dbl.group(1))

    def test_the_dashboard_still_has_no_ground_station_session(self):
        """It must not grow one by accident: there is no intermediary to guard
        against, and a redundant lease would be a second failure mode."""
        self.assertNotIn('/api/state', self.src)
        self.assertNotIn('/api/session', self.src)


if __name__ == '__main__':
    unittest.main()
