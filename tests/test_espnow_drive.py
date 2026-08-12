"""Regression coverage for the laptop ESP-NOW bridge control tool."""

import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest
from urllib import error as urllib_error
from urllib import request as urllib_request


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / 'tools' / 'espnow_drive.py'
SPEC = importlib.util.spec_from_file_location('espnow_drive_under_test', MODULE_PATH)
espnow_drive = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = espnow_drive
SPEC.loader.exec_module(espnow_drive)


class BridgeStatusDecodeTest(unittest.TestCase):
    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.bridge_status = espnow_drive.BoatLink._blank_bridge_status()
        self.link.telemetry = espnow_drive.BoatLink._blank_telemetry()
        self.link.connected = False
        self.link.port = None
        self.link.throttle = 0.0
        self.link.rudder = 0.0
        self.link.winch_speed = 0.0
        self.link.winch_lease_until = 0.0
        self.link.winch_command_seq = 0
        self.link.servo_rail_cut = None
        self.link.armed_cmd = False
        self.link.force = False
        self.link.calibrating = False
        self.link.seq = 0
        self.link.last_error = None

    def decode_bridge_status(self, payload: bytes):
        frame = espnow_drive.build_frame(espnow_drive.MSG_BRIDGE_STATUS, payload, seq=7)
        self.link._handle_incoming_frame(frame[:-1])

    def test_bridge_status_decodes_task_12_lr_extension(self):
        payload = espnow_drive.struct.pack(
            '<6Ibb', 12, 34, 56, 78, 90, 123, -71, -1)

        self.decode_bridge_status(payload)

        self.assertEqual(self.link.bridge_status['uptime_s'], 12)
        self.assertEqual(self.link.bridge_status['uplink_rssi_dbm'], -71)
        self.assertEqual(self.link.bridge_status['lr_rate_config_ok'], -1)
        api_status = self.link.status()['bridge_status']
        self.assertEqual(api_status['uplink_rssi_dbm'], -71)
        self.assertEqual(api_status['lr_rate_config_ok'], -1)

    def test_bridge_status_keeps_lr_fields_unknown_for_legacy_payload(self):
        payload = espnow_drive.struct.pack('<6I', 1, 2, 3, 4, 5, 6)

        self.decode_bridge_status(payload)

        self.assertEqual(self.link.bridge_status['uptime_s'], 1)
        self.assertIsNone(self.link.bridge_status['uplink_rssi_dbm'])
        self.assertIsNone(self.link.bridge_status['lr_rate_config_ok'])

    def test_bridge_status_rejects_truncated_lr_extension(self):
        """Catches a malformed 25-byte status being accepted as legacy."""
        payload = espnow_drive.struct.pack('<6I', 1, 2, 3, 4, 5, 6) + b'\x80'

        self.decode_bridge_status(payload)

        self.assertFalse(self.link.bridge_status['have'])

    def test_page_includes_task_12_bridge_diagnostics(self):
        self.assertIn('<label>Uplink RSSI</label>', espnow_drive.PAGE)
        self.assertIn('id="b-rssi"', espnow_drive.PAGE)
        self.assertIn('id="b-lr-rate"', espnow_drive.PAGE)
        self.assertIn('Number.isFinite(b.uplink_rssi_dbm)', espnow_drive.PAGE)
        self.assertIn('`${b.uplink_rssi_dbm} dBm`', espnow_drive.PAGE)
        self.assertIn('LR/250K SET', espnow_drive.PAGE)
        self.assertIn('LR/250K FAILED', espnow_drive.PAGE)
        self.assertIn('LR DISABLED', espnow_drive.PAGE)

    def test_status_exposes_actuator_command_sequence(self):
        """Catches the browser being unable to synchronize its request counter."""
        self.link.winch_command_seq = 7

        self.assertEqual(self.link.status().get('winch_command_seq'), 7)


class ServoRailAndWinchCommandTest(unittest.TestCase):
    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.connected = True
        self.link.throttle = 0.0
        self.link.rudder = 0.0
        self.link.winch_speed = 0.0
        self.link.winch_lease_until = 0.0
        self.link.winch_command_seq = 0
        self.link.servo_rail_cut = False
        self.link.calibrating = False
        self.link.seq = 0
        self.link.last_error = None
        self.sent = []

        def capture(payload):
            self.sent.append(payload)
            return True

        self.link._write_locked = capture

    def messages(self):
        decoded = []
        for payload in self.sent:
            msg = self.link.pb2.BoatMessage()
            msg.ParseFromString(payload)
            decoded.append(msg)
        return decoded

    def test_winch_speed_is_clamped_and_sent_immediately(self):
        self.link.set_winch(2.0, 1)

        self.assertEqual(self.link.winch_speed, 1.0)
        msg = self.messages()[-1]
        self.assertEqual(msg.WhichOneof('payload'), 'winch')
        self.assertEqual(msg.winch.speed, 1.0)

    def test_expired_winch_command_stops_retransmission(self):
        """Catches a missing server-side lease check in the 15 Hz loop."""
        self.link.winch_speed = 0.6
        self.link.winch_lease_until = time.monotonic() - 1.0
        self.link.send_hz = 100.0
        self.link._stop = threading.Event()

        stream = threading.Thread(target=self.link._stream_loop)
        stream.start()
        time.sleep(0.03)
        self.link._stop.set()
        stream.join(timeout=1.0)

        self.assertFalse(stream.is_alive())
        self.assertEqual(self.link.winch_speed, 0.0)
        self.assertTrue(all(msg.winch.speed == 0.0
                            for msg in self.messages()
                            if msg.WhichOneof('payload') == 'winch'))

    def test_fresh_winch_command_remains_active_during_lease(self):
        """Catches set_winch forgetting to start or renew its command lease."""
        self.link.send_hz = 100.0
        self.link._stop = threading.Event()

        self.link.set_winch(0.5, 1)
        self.sent.clear()
        stream = threading.Thread(target=self.link._stream_loop)
        stream.start()
        time.sleep(0.03)
        self.link._stop.set()
        stream.join(timeout=1.0)

        self.assertEqual(self.link.winch_speed, 0.5)
        winch_speeds = [msg.winch.speed for msg in self.messages()
                        if msg.WhichOneof('payload') == 'winch']
        self.assertTrue(winch_speeds)
        self.assertTrue(all(speed == 0.5 for speed in winch_speeds))

    def test_calibrating_streams_only_keepalive_never_manual(self):
        """While calibrating, the stream loop must send CalibrateCommand
        keepalives and NEVER motor/steer/winch -- any manual command would trip
        the firmware's calibration abort (the whole point of pausing the
        stream)."""
        self.link.armed_cmd = True
        self.link.throttle = 0.5          # would be streamed if not calibrating
        self.link.send_hz = 100.0
        self.link._stop = threading.Event()

        ok, err = self.link.set_calibrate(True, 1)
        self.assertTrue(ok, err)
        self.sent.clear()
        stream = threading.Thread(target=self.link._stream_loop)
        stream.start()
        time.sleep(0.03)
        self.link._stop.set()
        stream.join(timeout=1.0)

        kinds = {msg.WhichOneof('payload') for msg in self.messages()}
        self.assertIn('calibrate', kinds)
        self.assertNotIn('motor', kinds)
        self.assertNotIn('steer', kinds)
        self.assertNotIn('winch', kinds)
        self.assertTrue(all(msg.calibrate.start is True for msg in self.messages()
                            if msg.WhichOneof('payload') == 'calibrate'))

    def test_calibrate_start_requires_arm(self):
        """Calibration drives the thrusters -- refuse to start it unarmed."""
        self.link.armed_cmd = False
        ok, err = self.link.set_calibrate(True, 1)
        self.assertFalse(ok)
        self.assertIn('ARM', err)
        self.assertFalse(self.link.calibrating)

    def test_stop_ends_calibration_and_sends_stop(self):
        """Panic STOP must end an in-progress sweep and re-arm the firmware
        latch by sending CalibrateCommand{start:false}."""
        self.link.armed_cmd = True
        self.assertTrue(self.link.set_calibrate(True, 1)[0])
        self.sent.clear()
        self.link.stop(2)
        self.assertFalse(self.link.calibrating)
        stops = [msg for msg in self.messages()
                 if msg.WhichOneof('payload') == 'calibrate' and msg.calibrate.start is False]
        self.assertTrue(stops, 'STOP did not send a calibrate stop frame')

    def test_power_off_stops_winch_before_cutting_servo_rail(self):
        self.link.winch_speed = 0.6

        self.link.set_servo_power(False, 1)

        self.assertTrue(self.link.servo_rail_cut)
        self.assertEqual(self.link.winch_speed, 0.0)
        steer, stop, power = self.messages()
        self.assertEqual(steer.WhichOneof('payload'), 'steer')
        self.assertEqual((steer.steer.left, steer.steer.right), (0.0, 0.0))
        self.assertEqual(stop.WhichOneof('payload'), 'winch')
        self.assertEqual(stop.winch.speed, 0.0)
        self.assertEqual(power.WhichOneof('payload'), 'servo_power')
        self.assertFalse(power.servo_power.on)

    def test_stop_also_stops_the_winch_immediately(self):
        self.link.winch_speed = -0.5

        self.link.stop(1)

        self.assertEqual(self.link.winch_speed, 0.0)
        msg = self.messages()[-1]
        self.assertEqual(msg.WhichOneof('payload'), 'winch')
        self.assertEqual(msg.winch.speed, 0.0)

    def test_page_has_momentary_winch_and_explicit_servo_rail_controls(self):
        self.assertIn('id="servo-on-btn"', espnow_drive.PAGE)
        self.assertIn('id="servo-off-btn"', espnow_drive.PAGE)
        self.assertIn('id="winch-up-btn"', espnow_drive.PAGE)
        self.assertIn('id="winch-down-btn"', espnow_drive.PAGE)
        self.assertIn(
            "api('/api/winch', 'POST', { speed, seq: ++winchCommandSeq })",
            espnow_drive.PAGE)
        self.assertIn(
            "api('/api/servo-power', 'POST', { on, seq: ++winchCommandSeq })",
            espnow_drive.PAGE)
        self.assertIn("el.addEventListener('pointerup', stop)", espnow_drive.PAGE)
        self.assertIn("el.addEventListener('pointercancel', stop)", espnow_drive.PAGE)
        self.assertIn("el.addEventListener('pointerleave', stop)", espnow_drive.PAGE)
        self.assertIn("window.addEventListener('blur', stopWinch)", espnow_drive.PAGE)
        self.assertIn('const WINCH_RENEW_MS = 100;', espnow_drive.PAGE)
        self.assertIn("navigator.sendBeacon('/api/winch', body)", espnow_drive.PAGE)
        self.assertIn('winchDir = 0;\n    setConnectedUI(true, port);', espnow_drive.PAGE)

    def test_held_winch_is_renewed_with_increasing_command_sequences(self):
        """Catches the browser sending only one command for a long button hold."""
        page_script = espnow_drive.PAGE.rsplit('<script>', 1)[1].split('</script>', 1)[0]
        harness = r"""
const fs = require('fs');
const vm = require('vm');
const script = fs.readFileSync(0, 'utf8');
const calls = [];
const elements = {};

class FakeClassList {
  add() {}
  remove() {}
  toggle() {}
}
class FakeElement {
  constructor(id = '') {
    this.id = id;
    this.value = id === 'winch-speed' ? '50' : '';
    this.textContent = '';
    this.disabled = false;
    this.options = [];
    this.listeners = {};
    this.classList = new FakeClassList();
  }
  addEventListener(name, callback) { this.listeners[name] = callback; }
  appendChild(child) { this.options.push(child); }
}
const document = {
  getElementById(id) {
    if (!elements[id]) elements[id] = new FakeElement(id);
    return elements[id];
  },
  createElement() { return new FakeElement(); },
  addEventListener() {},
};
const windowObject = { addEventListener() {} };
async function fetch(path, opts = {}) {
  if (opts.body) calls.push({ path, body: JSON.parse(opts.body) });
  return { json: async () => path === '/api/ports' ? { ports: [] } : { ok: true } };
}
class FakeWebSocket { close() {} }
const context = {
  document,
  window: windowObject,
  navigator: { sendBeacon() { return true; } },
  location: { host: '127.0.0.1:8765' },
  fetch,
  WebSocket: FakeWebSocket,
  Blob,
  console,
  setTimeout,
  setInterval,
  clearInterval,
};
vm.createContext(context);
vm.runInContext(script, context);
context.applyStatus({
  connected: true,
  port: '/dev/test',
  armed_cmd: false,
  servo_rail_cut: false,
  winch_command_seq: 0,
  seq: 0,
  last_error: null,
  telemetry: { have: false, stale: true, age_s: null },
  bridge_status: { have: false, stale: true, age_s: null },
});
elements['winch-up-btn'].listeners.pointerdown({ preventDefault() {} });
setTimeout(() => {
  const active = calls.filter(call => call.path === '/api/winch' && call.body.speed !== 0);
  elements['winch-up-btn'].listeners.pointerup();
  console.log(JSON.stringify(active));
}, 250);
"""
        result = subprocess.run(
            ['node', '-e', harness], input=page_script, text=True,
            capture_output=True, timeout=2.0, check=False)

        self.assertEqual(result.returncode, 0, result.stderr)
        active = json.loads(result.stdout.strip().splitlines()[-1])
        self.assertGreaterEqual(len(active), 2)
        sequences = [call['body']['seq'] for call in active]
        self.assertEqual(sequences, sorted(set(sequences)))


class ActuatorApiSafetyTest(unittest.TestCase):
    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.connected = True
        self.link.throttle = 0.0
        self.link.rudder = 0.0
        self.link.winch_speed = 0.0
        self.link.winch_lease_until = 0.0
        self.link.winch_command_seq = 0
        self.link.servo_rail_cut = False
        self.link.armed_cmd = False
        self.link.force = False
        self.link.calibrating = False
        self.link.seq = 0
        self.link.last_error = None
        self.sent = []

        def capture(payload):
            self.sent.append(payload)
            return True

        self.link._write_locked = capture

        class TestHandler(espnow_drive.Handler):
            pass

        TestHandler.link = self.link
        self.server = espnow_drive.ThreadingHTTPServer(('127.0.0.1', 0), TestHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.start()
        host, port = self.server.server_address
        self.base_url = f'http://{host}:{port}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=1.0)

    def post(self, path, body):
        req = urllib_request.Request(
            self.base_url + path,
            data=json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'},
            method='POST')
        try:
            with urllib_request.urlopen(req, timeout=1.0) as response:
                return response.status, json.load(response)
        except urllib_error.HTTPError as exc:
            return exc.code, json.load(exc)

    def messages(self):
        decoded = []
        for payload in self.sent:
            msg = self.link.pb2.BoatMessage()
            msg.ParseFromString(payload)
            decoded.append(msg)
        return decoded

    def test_stale_winch_request_cannot_restart_after_newer_stop(self):
        """Catches command endpoints ignoring browser request ordering."""
        self.assertEqual(self.post('/api/winch', {'speed': 0.5, 'seq': 1})[0], 200)
        self.assertEqual(self.post('/api/stop', {'seq': 2})[0], 200)

        status, body = self.post('/api/winch', {'speed': 0.8, 'seq': 1})

        self.assertEqual(status, 409)
        self.assertFalse(body['ok'])
        self.assertEqual(self.link.winch_speed, 0.0)

    def test_powered_off_servo_rail_rejects_nonzero_winch(self):
        """Catches a command being latched while physical servo power is off."""
        self.assertEqual(self.post('/api/servo-power', {'on': False, 'seq': 1})[0], 200)

        status, body = self.post('/api/winch', {'speed': 0.5, 'seq': 2})

        self.assertEqual(status, 409)
        self.assertFalse(body['ok'])
        self.assertEqual(self.link.winch_speed, 0.0)

    def test_power_on_stops_winch_before_energizing_servo_rail(self):
        """Catches a stored nonzero speed taking effect as power returns."""
        self.link.winch_speed = 0.7

        status, body = self.post('/api/servo-power', {'on': True, 'seq': 1})

        self.assertEqual(status, 200)
        self.assertTrue(body['ok'])
        self.assertEqual(self.link.winch_speed, 0.0)
        steer, stop, power = self.messages()
        self.assertEqual(steer.WhichOneof('payload'), 'steer')
        self.assertEqual((steer.steer.left, steer.steer.right), (0.0, 0.0))
        self.assertEqual(stop.WhichOneof('payload'), 'winch')
        self.assertEqual(stop.winch.speed, 0.0)
        self.assertEqual(power.WhichOneof('payload'), 'servo_power')
        self.assertTrue(power.servo_power.on)

    def test_power_on_centers_rudder_before_energizing_servo_rail(self):
        """Catches a stored rudder angle taking effect as servo power returns."""
        self.link.rudder = 0.6

        status, body = self.post('/api/servo-power', {'on': True, 'seq': 1})

        self.assertEqual(status, 200)
        self.assertTrue(body['ok'])
        self.assertEqual(self.link.rudder, 0.0)
        steer, winch, power = self.messages()
        self.assertEqual(steer.WhichOneof('payload'), 'steer')
        self.assertEqual((steer.steer.left, steer.steer.right), (0.0, 0.0))
        self.assertEqual(winch.WhichOneof('payload'), 'winch')
        self.assertEqual(winch.winch.speed, 0.0)
        self.assertEqual(power.WhichOneof('payload'), 'servo_power')
        self.assertTrue(power.servo_power.on)

    def test_disconnected_link_rejects_winch_command(self):
        """Catches the API claiming success when no serial link can transmit."""
        self.link.connected = False

        status, body = self.post('/api/winch', {'speed': 0.5, 'seq': 1})

        self.assertEqual(status, 503)
        self.assertFalse(body['ok'])
        self.assertEqual(self.link.winch_speed, 0.0)

    def test_serial_write_failure_is_reported_and_does_not_latch_winch(self):
        """Catches actuator endpoints returning success after a failed write."""
        self.link._write_locked = lambda _payload: False

        status, body = self.post('/api/winch', {'speed': 0.5, 'seq': 1})

        self.assertEqual(status, 503)
        self.assertFalse(body['ok'])
        self.assertEqual(self.link.winch_speed, 0.0)

    def test_disconnected_link_rejects_servo_power_command(self):
        """Catches reporting a rail state that was never transmitted."""
        self.link.connected = False
        self.link.servo_rail_cut = None

        status, body = self.post('/api/servo-power', {'on': True, 'seq': 1})

        self.assertEqual(status, 503)
        self.assertFalse(body['ok'])
        self.assertIsNone(self.link.servo_rail_cut)

    def test_servo_power_requires_json_boolean(self):
        """Catches bool('false') accidentally becoming a power-on command."""
        self.link.servo_rail_cut = True

        status, body = self.post('/api/servo-power', {'on': 'false', 'seq': 1})

        self.assertEqual(status, 400)
        self.assertFalse(body['ok'])
        self.assertTrue(self.link.servo_rail_cut)

    def test_winch_rejects_nonfinite_speed(self):
        """Catches NaN bypassing the numeric actuator boundary."""
        status, body = self.post('/api/winch', {'speed': float('nan'), 'seq': 1})

        self.assertEqual(status, 400)
        self.assertFalse(body['ok'])
        self.assertEqual(self.link.winch_speed, 0.0)

    def test_command_sequence_must_be_nonnegative(self):
        """Catches malformed sequence values reaching actuator ordering logic."""
        status, body = self.post('/api/winch', {'speed': 0.0, 'seq': -1})

        self.assertEqual(status, 400)
        self.assertFalse(body['ok'])

    def test_connect_does_not_replace_an_active_serial_session(self):
        """Catches connect closing a live link without first sending stops."""
        class CurrentSerial:
            closed = False

            def close(self):
                self.closed = True

        current = CurrentSerial()
        self.link.ser = current
        self.link.port = '/dev/current'

        ok, _error = self.link.connect('/dev/definitely-does-not-exist')

        self.assertFalse(ok)
        self.assertTrue(self.link.connected)
        self.assertEqual(self.link.port, '/dev/current')
        self.assertFalse(current.closed)

    def test_non_object_json_body_is_treated_as_invalid_input(self):
        """Catches endpoint crashes when JSON is valid but not an object."""
        handler = espnow_drive.Handler.__new__(espnow_drive.Handler)
        handler.headers = {'Content-Length': '2'}
        handler.rfile = io.BytesIO(b'[]')

        self.assertEqual(handler._read_body(), {})

    def test_power_on_aborts_when_zero_winch_write_fails(self):
        """Catches energizing the rail without first transmitting winch zero."""
        self.link.servo_rail_cut = True
        self.link._write_locked = lambda _payload: False

        status, body = self.post('/api/servo-power', {'on': True, 'seq': 1})

        self.assertEqual(status, 503)
        self.assertFalse(body['ok'])
        self.assertTrue(self.link.servo_rail_cut)

    def test_disconnect_invalidates_inflight_actuator_request(self):
        """Catches a pre-disconnect request taking effect after reconnect."""
        self.link.winch_command_seq = 4

        self.link.disconnect()
        self.link.connected = True
        self.link.servo_rail_cut = False
        ok, _error = self.link.set_winch(0.5, 5)

        self.assertFalse(ok)
        self.assertEqual(self.link.winch_speed, 0.0)

    def test_disconnected_stop_zeros_local_state_but_reports_not_transmitted(self):
        """Catches panic-stop claiming it reached a disconnected boat."""
        self.link.connected = False
        self.link.throttle = 0.5
        self.link.rudder = -0.5
        self.link.winch_speed = 0.5

        status, body = self.post('/api/stop', {'seq': 1})

        self.assertEqual(status, 503)
        self.assertFalse(body['ok'])
        self.assertEqual(
            (self.link.throttle, self.link.rudder, self.link.winch_speed),
            (0.0, 0.0, 0.0))

    def test_stop_reports_serial_write_failure(self):
        """Catches panic-stop returning success when every write failed."""
        self.link._write_locked = lambda _payload: False

        status, body = self.post('/api/stop', {'seq': 1})

        self.assertEqual(status, 503)
        self.assertFalse(body['ok'])


if __name__ == '__main__':
    unittest.main()
