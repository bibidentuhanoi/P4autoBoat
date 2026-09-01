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
        self.link.motor_left = 0.0
        self.link.motor_right = 0.0
        self.link.motor_split = False
        self.link.rudder = 0.0
        self.link.winch_speed = 0.0
        self.link.winch_lease_until = 0.0
        self.link.winch_command_seq = 0
        self.link.servo_rail_cut = None
        self.link.armed_cmd = False
        self.link.force = False
        self.link.calibrating = False
        self.link.calibrate_status = espnow_drive.BoatLink._blank_calibrate_status()
        self.link.system_status = espnow_drive.BoatLink._blank_system_status()
        self.link.motor_status = espnow_drive.BoatLink._blank_motor_status()
        self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
        # UI-observed yaw summary state. Set here for the same reason every
        # other field is: this fixture builds the link with __new__, so it
        # stands in for __init__ and has to carry whatever status() reads.
        self.link.bench_yaw_samples = []
        self.link.bench_yaw = None
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


class CalibrateStatusDecodeTest(unittest.TestCase):
    """CalibrateStatus rides the telemetry link tagged MSG_SENSOR (the P4's
    default framing). These pin that it's decoded, not rejected -- and that the
    sensor path is untouched."""

    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.calibrating = True
        self.link.calibrate_status = espnow_drive.BoatLink._blank_calibrate_status()
        self.link.system_status = espnow_drive.BoatLink._blank_system_status()
        self.link.motor_status = espnow_drive.BoatLink._blank_motor_status()
        self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
        self.link.telemetry = espnow_drive.BoatLink._blank_telemetry()
        self.link._diag_counts = {}

    def _feed_calibrate(self, **fields):
        msg = self.link.pb2.BoatMessage()
        for k, v in fields.items():
            setattr(msg.calibrate_status, k, v)
        frame = espnow_drive.build_frame(espnow_drive.MSG_SENSOR,
                                         msg.SerializeToString(), seq=1)
        self.link._handle_incoming_frame(frame[:-1])   # strip trailing delimiter

    def test_calibrate_status_is_decoded_not_rejected(self):
        self._feed_calibrate(state=3, level_index=2, level_throttle=0.2,
                             trim_diff=0.05, points_done=1, making_way=True)
        cs = self.link.calibrate_status
        self.assertTrue(cs['have'])
        self.assertEqual(cs['state'], 3)
        self.assertEqual(cs['points_done'], 1)
        self.assertAlmostEqual(cs['trim_diff'], 0.05, places=5)
        self.assertTrue(self.link.calibrating)   # mid-run, still calibrating

    def test_done_and_aborted_clear_local_calibrating_flag(self):
        self._feed_calibrate(state=4, points_done=3)   # DONE
        self.assertEqual(self.link.calibrate_status['state'], 4)
        self.assertFalse(self.link.calibrating)
        self.link.calibrating = True
        self._feed_calibrate(state=5)                  # ABORTED
        self.assertFalse(self.link.calibrating)

    def test_system_status_decodes_gps_chip_and_sensor_health(self):
        """The boat sends SystemStatus over the field link too; the tool must
        decode it so 'GPS chip talking' is visible (not just 'NO FIX'), and the
        boot sensor-health flags land in self.system_status."""
        msg = self.link.pb2.BoatMessage()
        msg.status.gps_ok = True            # chip talking to the UART
        msg.status.camera_ok = True
        msg.status.tof_a_ok = True
        msg.status.tof_b_ok = False
        msg.status.imu_ok = True
        msg.status.gps_detected_baud = 115200
        msg.status.gps_baud_confirmed = True
        frame = espnow_drive.build_frame(espnow_drive.MSG_STATUS,
                                         msg.SerializeToString(), seq=3)
        self.link._handle_incoming_frame(frame[:-1])
        ss = self.link.system_status
        self.assertTrue(ss['have'])
        self.assertTrue(ss['gps_ok'])
        self.assertTrue(ss['camera_ok'])
        self.assertTrue(ss['tof_a_ok'])
        self.assertFalse(ss['tof_b_ok'])
        self.assertEqual(ss['gps_detected_baud'], 115200)
        # A status frame must NOT be mistaken for telemetry.
        self.assertFalse(self.link.telemetry['have'])

    def test_motor_status_decodes_actual_arm_and_throttle(self):
        """MotorStatus (0x06) is the boat's ACTUAL arm/throttle/servo -- the tool
        used to drop it, so the operator could only see the COMMANDED arm, never
        confirm the boat armed or what throttle each ESC really ran."""
        msg = self.link.pb2.BoatMessage()
        msg.motor_status.state = 2            # ESC_STATE_ARMED
        msg.motor_status.left_throttle = 0.30
        msg.motor_status.right_throttle = 0.25
        msg.motor_status.servo_power = True
        frame = espnow_drive.build_frame(espnow_drive.MSG_MOTOR_STATUS,
                                         msg.SerializeToString(), seq=4)
        self.link._handle_incoming_frame(frame[:-1])
        m = self.link.motor_status
        self.assertTrue(m['have'])
        self.assertEqual(m['state'], 2)
        self.assertAlmostEqual(m['left_throttle'], 0.30, places=5)
        self.assertTrue(m['servo_power'])
        self.assertFalse(self.link.telemetry['have'])   # not mistaken for telemetry

    def test_sensor_frame_still_decodes(self):
        """Regression guard: adding the calibrate branch must not break the
        sensor telemetry path -- a sensor frame still lands in self.telemetry."""
        msg = self.link.pb2.BoatMessage()
        msg.sensors.imu.heading = 123.0
        msg.sensors.gps.valid = True
        frame = espnow_drive.build_frame(espnow_drive.MSG_SENSOR,
                                         msg.SerializeToString(), seq=2)
        self.link._handle_incoming_frame(frame[:-1])
        self.assertTrue(self.link.telemetry['have'])
        self.assertAlmostEqual(self.link.telemetry['heading'], 123.0, places=3)
        self.assertTrue(self.link.telemetry['gps_valid'])


class ServoRailAndWinchCommandTest(unittest.TestCase):
    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.connected = True
        self.link.throttle = 0.0
        self.link.motor_left = 0.0
        self.link.motor_right = 0.0
        self.link.motor_split = False
        self.link.rudder = 0.0
        self.link.winch_speed = 0.0
        self.link.winch_lease_until = 0.0
        self.link.winch_command_seq = 0
        self.link.servo_rail_cut = False
        self.link.calibrating = False
        self.link.calibrate_status = espnow_drive.BoatLink._blank_calibrate_status()
        self.link.system_status = espnow_drive.BoatLink._blank_system_status()
        self.link.motor_status = espnow_drive.BoatLink._blank_motor_status()
        self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
        # UI-observed yaw summary state. Set here for the same reason every
        # other field is: this fixture builds the link with __new__, so it
        # stands in for __init__ and has to carry whatever status() reads.
        self.link.bench_yaw_samples = []
        self.link.bench_yaw = None
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

    def test_unlinked_motors_stream_independent_left_right(self):
        """Unlink -> each slider drives one ESC. set_state(left=..) flips to split
        mode and the stream loop sends per-motor values, not both-equal (this is
        the single-ESC spin test)."""
        self.link.send_hz = 100.0
        self.link._stop = threading.Event()
        self.link.set_state(left=0.4, right=0.0)     # spin left only
        self.assertTrue(self.link.motor_split)
        stream = threading.Thread(target=self.link._stream_loop)
        stream.start()
        time.sleep(0.05)
        self.link._stop.set()
        stream.join(timeout=1.0)
        motors = [m for m in self.messages() if m.WhichOneof('payload') == 'motor']
        self.assertTrue(motors)
        self.assertAlmostEqual(motors[-1].motor.left, 0.4, places=5)
        self.assertAlmostEqual(motors[-1].motor.right, 0.0, places=5)

    def test_linked_throttle_still_drives_both_equal(self):
        """Linked (default) mode is unchanged: throttle drives both motors."""
        self.link.send_hz = 100.0
        self.link._stop = threading.Event()
        self.link.set_state(throttle=0.6)
        self.assertFalse(self.link.motor_split)
        stream = threading.Thread(target=self.link._stream_loop)
        stream.start()
        time.sleep(0.05)
        self.link._stop.set()
        stream.join(timeout=1.0)
        motors = [m for m in self.messages() if m.WhichOneof('payload') == 'motor']
        self.assertTrue(motors)
        self.assertAlmostEqual(motors[-1].motor.left, 0.6, places=5)
        self.assertAlmostEqual(motors[-1].motor.right, 0.6, places=5)

    def test_stop_drops_out_of_split_mode(self):
        """A held single-ESC test must not keep spinning after STOP: panic stop
        clears split mode and zeros throttle so the stream sends 0/0 to both."""
        self.link.set_state(left=0.5, right=0.5)
        self.assertTrue(self.link.motor_split)
        self.link.stop(999)
        self.assertFalse(self.link.motor_split)
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.motor_left, 0.0)
        self.assertEqual(self.link.motor_right, 0.0)

    def test_page_exposes_motor_link_and_split_sliders(self):
        self.assertIn('id="motor-link"', espnow_drive.PAGE)
        self.assertIn('id="motor-left"', espnow_drive.PAGE)
        self.assertIn('id="motor-right"', espnow_drive.PAGE)

    def test_calibrate_start_requires_arm(self):
        """Calibration drives the thrusters -- refuse to start it unarmed."""
        self.link.armed_cmd = False
        ok, err = self.link.set_calibrate(True, 1)
        self.assertFalse(ok)
        self.assertIn('ARM', err)
        self.assertFalse(self.link.calibrating)

    def test_record_sends_training_log_trigger(self):
        """The Record button fires one fire-and-forget TrainingLogCommand
        (Feature 1 dataset capture) -- an empty message whose presence is the
        trigger, mirroring dashboard.html's Record button. Not arm-gated."""
        ok, err = self.link.trigger_record()
        self.assertTrue(ok, err)
        msgs = self.messages()
        self.assertEqual(len(msgs), 1)
        self.assertEqual(msgs[0].WhichOneof('payload'), 'training_log')

    def test_record_rejected_when_disconnected(self):
        """No serial link -> the capture command is refused, not silently lost."""
        self.link.connected = False
        ok, err = self.link.trigger_record()
        self.assertFalse(ok)
        self.assertIn('disconnected', err)

    def test_page_exposes_record_control(self):
        self.assertIn('id="record-btn"', espnow_drive.PAGE)
        self.assertIn('/api/record', espnow_drive.PAGE)

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
        self.link.motor_left = 0.0
        self.link.motor_right = 0.0
        self.link.motor_split = False
        self.link.rudder = 0.0
        self.link.winch_speed = 0.0
        self.link.winch_lease_until = 0.0
        self.link.winch_command_seq = 0
        self.link.servo_rail_cut = False
        self.link.armed_cmd = False
        self.link.force = False
        self.link.calibrating = False
        self.link.calibrate_status = espnow_drive.BoatLink._blank_calibrate_status()
        self.link.system_status = espnow_drive.BoatLink._blank_system_status()
        self.link.motor_status = espnow_drive.BoatLink._blank_motor_status()
        self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
        # UI-observed yaw summary state. Set here for the same reason every
        # other field is: this fixture builds the link with __new__, so it
        # stands in for __init__ and has to carry whatever status() reads.
        self.link.bench_yaw_samples = []
        self.link.bench_yaw = None
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



class BenchRunPageTest(unittest.TestCase):
    def test_page_exposes_the_three_bench_buttons(self):
        for el in ('bench-left', 'bench-right', 'bench-base',
                   'bench-throttle', 'bench-delta'):
            self.assertIn('id="%s"' % el, espnow_drive.PAGE)
        self.assertIn('/api/bench', espnow_drive.PAGE)
        # Named for the MOTOR, not a turn direction -- which way the boat
        # physically swings has not been measured, and these runs are how it
        # gets measured. See tests/test_bench_split_and_yaw.py.
        self.assertIn('LEFT MOTOR STRONGER', espnow_drive.PAGE)
        self.assertIn('RIGHT MOTOR STRONGER', espnow_drive.PAGE)
        self.assertIn('BASE TEST', espnow_drive.PAGE)


class BenchRunLinkTest(unittest.TestCase):
    """The BOAT runs the bench test and records it to its own SD card; this tool
    only presses the button. These pin the command that goes out, and the gates
    that stop a run being started at a bad moment."""

    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.connected = True
        self.link.throttle = 0.0
        self.link.motor_left = 0.0
        self.link.motor_right = 0.0
        self.link.motor_split = False
        self.link.rudder = 0.0
        self.link.winch_speed = 0.0
        self.link.winch_lease_until = 0.0
        self.link.winch_command_seq = 0
        self.link.servo_rail_cut = False
        self.link.armed_cmd = True
        self.link.force = True
        self.link.calibrating = False
        self.link.calibrate_status = espnow_drive.BoatLink._blank_calibrate_status()
        self.link.system_status = espnow_drive.BoatLink._blank_system_status()
        self.link.motor_status = espnow_drive.BoatLink._blank_motor_status()
        self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
        self.link.telemetry = espnow_drive.BoatLink._blank_telemetry()
        self.link._diag_counts = {}
        self.link.seq = 0
        self.link.last_error = None
        self.sent = []

        def capture(payload):
            self.sent.append(payload)
            return True

        self.link._write_locked = capture

    def messages(self):
        out = []
        for payload in self.sent:
            m = self.link.pb2.BoatMessage()
            m.ParseFromString(payload)
            out.append(m)
        return out

    def test_each_button_sends_its_own_kind_to_the_boat(self):
        for seq, (kind, wire) in enumerate(
                (('both', 0), ('left', 1), ('right', 2)), start=1):
            self.sent.clear()
            self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
            ok, err = self.link.send_bench(kind, 0.20, 0.04, seq)
            self.assertTrue(ok, err)
            msgs = self.messages()
            self.assertEqual(len(msgs), 1)
            self.assertEqual(msgs[0].WhichOneof('payload'), 'bench')
            self.assertEqual(msgs[0].bench.kind, wire)
            self.assertAlmostEqual(msgs[0].bench.base, 0.20, places=5)
            self.assertAlmostEqual(msgs[0].bench.delta, 0.04, places=5)

    def test_bench_run_requires_arm(self):
        """The run spins the thrusters -- refuse it unarmed."""
        self.link.armed_cmd = False
        ok, err = self.link.send_bench('both', 0.20, 0.04, 1)
        self.assertFalse(ok)
        self.assertIn('ARM', err)
        self.assertEqual(self.sent, [])

    def test_bench_run_rejects_unknown_kind(self):
        ok, _err = self.link.send_bench('sideways', 0.20, 0.04, 1)
        self.assertFalse(ok)
        self.assertEqual(self.sent, [])

    def test_bench_run_refused_while_calibrating(self):
        self.link.calibrating = True
        ok, _err = self.link.send_bench('both', 0.20, 0.04, 1)
        self.assertFalse(ok)
        self.assertEqual(self.sent, [])

    def test_second_run_refused_while_the_boat_is_still_running_one(self):
        """The state comes from the BOAT, so this reflects what is actually
        happening rather than what the laptop assumes."""
        self.link.bench_status['have'] = True
        self.link.bench_status['state'] = 2          # driving
        self.link.bench_status['last_rx_monotonic'] = time.monotonic()
        ok, err = self.link.send_bench('left', 0.20, 0.04, 1)
        self.assertFalse(ok)
        self.assertIn('already going', err)

    def test_a_stale_running_status_does_not_block_new_runs_forever(self):
        """The boat publishes its terminal SAVED status ONCE. If that packet is
        lost over the air the tool would keep seeing 'driving' and refuse every
        later run for good. A run publishes at ~5 Hz, so a running state that
        has not been refreshed for seconds cannot possibly be live."""
        self.link.bench_status['have'] = True
        self.link.bench_status['state'] = 2                  # driving
        self.link.bench_status['last_rx_monotonic'] = time.monotonic() - 30.0
        ok, err = self.link.send_bench('both', 0.20, 0.04, 1)
        self.assertTrue(ok, err)

    def test_a_genuinely_live_run_still_blocks_a_second_one(self):
        self.link.bench_status['have'] = True
        self.link.bench_status['state'] = 2
        self.link.bench_status['last_rx_monotonic'] = time.monotonic()
        ok, err = self.link.send_bench('both', 0.20, 0.04, 1)
        self.assertFalse(ok)
        self.assertIn('already going', err)

    def test_calibration_is_refused_while_a_bench_run_is_going(self):
        self.link.bench_status['have'] = True
        self.link.bench_status['state'] = 1          # motors-off baseline
        self.link.bench_status['last_rx_monotonic'] = time.monotonic()
        ok, _err = self.link.set_calibrate(True, 5)
        self.assertFalse(ok)
        self.assertFalse(self.link.calibrating)

    def test_starting_a_run_zeroes_our_own_outputs(self):
        """The boat drives the run, so a throttle we were holding must not
        fight it during the run or resume after it."""
        self.link.throttle = 0.30
        self.link.motor_split = True
        self.link.motor_left = 0.4
        self.assertTrue(self.link.send_bench('both', 0.20, 0.04, 1)[0])
        self.assertEqual(self.link.throttle, 0.0)
        self.assertEqual(self.link.motor_left, 0.0)
        self.assertEqual(self.link.motor_right, 0.0)
        self.assertFalse(self.link.motor_split)

    def test_bench_status_from_the_boat_is_decoded(self):
        msg = self.link.pb2.BoatMessage()
        msg.bench_status.state = 4                   # SAVED
        msg.bench_status.kind = 1                    # LEFT
        msg.bench_status.base = 0.20
        msg.bench_status.samples = 431
        msg.bench_status.file_index = 3
        msg.bench_status.elapsed_s = 4.5
        frame = espnow_drive.build_frame(espnow_drive.MSG_SENSOR,
                                         msg.SerializeToString(), seq=6)
        self.link._handle_incoming_frame(frame[:-1])
        bs = self.link.bench_status
        self.assertTrue(bs['have'])
        self.assertEqual(bs['state'], 4)
        self.assertEqual(bs['kind'], 1)
        self.assertEqual(bs['samples'], 431)
        self.assertEqual(bs['file_index'], 3)
        # a bench status must not be mistaken for telemetry
        self.assertFalse(self.link.telemetry['have'])


class FieldTelemetryYawRateTest(unittest.TestCase):
    """The bench run needs gyro turn rate, which the field telemetry struct
    did not carry. Both ends must agree on the layout or the frame is
    rejected outright (the tool length-checks it)."""

    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.telemetry = espnow_drive.BoatLink._blank_telemetry()
        self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
        self.link._diag_counts = {}

    def test_field_telemetry_decodes_yaw_rate(self):
        payload = espnow_drive.struct.pack(espnow_drive.FIELD_TELEMETRY_FMT,
                              1.0, 2.0, 123.0,      # pitch, roll, heading
                              1,                     # gps_valid
                              10.5, 20.5,            # lat, lon
                              1.5, 90.0,             # speed, course
                              7,                     # satellites
                              1.2,                   # hdop
                              -8.25)                 # yaw_rate
        frame = espnow_drive.build_frame(espnow_drive.MSG_FIELD_TELEMETRY,
                                         payload, seq=1)
        self.link._handle_incoming_frame(frame[:-1])
        self.assertTrue(self.link.telemetry['have'])
        self.assertAlmostEqual(self.link.telemetry['yaw_rate'], -8.25, places=4)
        self.assertAlmostEqual(self.link.telemetry['heading'], 123.0, places=3)



class WireLayoutAgreementTest(unittest.TestCase):
    """Every decoder must agree with the firmware's espnow_telemetry_t.

    A mismatch does not crash anything -- the frame is silently length-rejected
    and telemetry simply stops arriving, which is miserable to diagnose in the
    field. So the layout is asserted here rather than trusted."""

    @staticmethod
    def _firmware_struct_bytes():
        header = (REPO_ROOT / 'main' / 'transports' / 'espnow_protocol.h').read_text()
        end = header.index('} espnow_telemetry_t;')
        start = header.rindex('typedef struct __attribute__((packed)) {', 0, end)
        sizes = {'float': 4, 'double': 8, 'uint8_t': 1,
                 'int8_t': 1, 'uint16_t': 2, 'uint32_t': 4}
        fields = espnow_drive.re.findall(
            r'^\s*(float|double|uint8_t|int8_t|uint16_t|uint32_t)\s+\w+;',
            header[start:end], espnow_drive.re.M)
        assert fields, 'could not parse espnow_telemetry_t'
        return sum(sizes[t] for t in fields)

    def test_control_tool_layout_matches_the_firmware(self):
        self.assertEqual(
            espnow_drive.struct.calcsize(espnow_drive.FIELD_TELEMETRY_FMT),
            self._firmware_struct_bytes())

    def test_visualizer_layout_matches_the_firmware(self):
        """visualize.py is the other field-mode viewer and decodes the same
        frames. It lives at the repo root, so it is easy to forget."""
        src = (REPO_ROOT / 'visualize.py').read_text()
        fmt = espnow_drive.re.search(
            r"FIELD_TELEMETRY_FMT = '([^']+)'", src).group(1)
        self.assertEqual(espnow_drive.struct.calcsize(fmt),
                         self._firmware_struct_bytes())


if __name__ == '__main__':
    unittest.main()


class BenchLearnerResetTest(unittest.TestCase):
    """The one-shot 'restart the learner at c' control.

    It exists for a single experiment -- start low, start high, and see whether
    both walk to the same c. Everything here protects the two properties that
    make that experiment mean anything: an ORDINARY run must never reset, and a
    REFUSED run must never reset."""

    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.connected = True
        self.link.throttle = 0.0
        self.link.motor_left = 0.0
        self.link.motor_right = 0.0
        self.link.motor_split = False
        self.link.rudder = 0.0
        self.link.winch_command_seq = 0
        self.link.armed_cmd = True
        self.link.calibrating = False
        self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
        self.link.telemetry = espnow_drive.BoatLink._blank_telemetry()
        self.link._diag_counts = {}
        self.sent = []
        self.link._write_locked = lambda p: (self.sent.append(p), True)[1]

    def parsed(self):
        out = []
        for payload in self.sent:
            m = self.link.pb2.BoatMessage()
            m.ParseFromString(payload)
            out.append(m)
        return out

    def test_reset_c_is_serialized_onto_the_wire(self):
        ok, err = self.link.send_bench('both', 0.40, 0.04, 1, 0.12)
        self.assertTrue(ok, err)
        m = self.parsed()[0]
        self.assertEqual(m.WhichOneof('payload'), 'bench')
        self.assertAlmostEqual(m.bench.reset_c, 0.12, places=5)
        self.assertEqual(m.bench.kind, 0)            # still an ordinary BASE run
        self.assertAlmostEqual(m.bench.base, 0.40, places=5)

    def test_an_ordinary_run_sends_zero_so_the_learned_c_survives(self):
        """c carrying over between runs is the whole mechanism under test."""
        ok, err = self.link.send_bench('both', 0.40, 0.04, 1)
        self.assertTrue(ok, err)
        self.assertEqual(self.parsed()[0].bench.reset_c, 0.0)

    def test_out_of_range_is_refused_and_nothing_is_sent(self):
        """Clamping would start the experiment from a c the operator never
        chose, and say nothing about it."""
        for seq, bad in enumerate((0.09, 0.36, -0.20, 1.0,
                                   float('nan'), float('inf')), start=1):
            self.sent.clear()
            ok, err = self.link.send_bench('both', 0.40, 0.04, seq, bad)
            self.assertFalse(ok, 'reset_c=%r was accepted' % bad)
            self.assertEqual(self.sent, [], 'a command went out for %r' % bad)
            self.assertIn('reset-c', err)

    def test_the_clamp_edges_themselves_are_accepted(self):
        for seq, edge in enumerate((espnow_drive.TRIMLEARN_C_MIN,
                                    espnow_drive.TRIMLEARN_C_MAX), start=1):
            self.sent.clear()
            self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
            ok, err = self.link.send_bench('both', 0.40, 0.04, seq, edge)
            self.assertTrue(ok, err)
            self.assertAlmostEqual(self.parsed()[0].bench.reset_c, edge, places=5)

    def test_a_refused_run_never_sends_a_reset(self):
        """A refused start must leave the learner exactly as it was -- a button
        press that visibly did nothing must not silently wipe what it knew."""
        for setup, why in ((lambda: setattr(self.link, 'armed_cmd', False), 'disarmed'),
                           (lambda: setattr(self.link, 'calibrating', True), 'calibrating'),
                           (lambda: setattr(self.link, 'connected', False), 'disconnected')):
            self.setUp()
            setup()
            ok, _err = self.link.send_bench('both', 0.40, 0.04, 1, 0.12)
            self.assertFalse(ok, why)
            self.assertEqual(self.sent, [], 'a reset went out while %s' % why)

    def test_reported_learner_c_is_decoded(self):
        msg = self.link.pb2.BoatMessage()
        msg.bench_status.state = 2
        msg.bench_status.base = 0.40
        msg.bench_status.learn_c = 0.173
        frame = espnow_drive.build_frame(espnow_drive.MSG_SENSOR,
                                         msg.SerializeToString(), seq=3)
        self.link._handle_incoming_frame(frame[:-1])
        self.assertAlmostEqual(self.link.bench_status['learn_c'], 0.173, places=5)

    def test_a_blank_status_still_has_the_field(self):
        """The page reads learn_c before the first status arrives."""
        self.assertIn('learn_c', espnow_drive.BoatLink._blank_bench_status())


class BenchResetPageTest(unittest.TestCase):
    """The control has to exist on the page, be bounded, and the plain buttons
    must stay non-resetting."""

    def test_the_reset_control_is_present_and_bounded(self):
        page = espnow_drive.PAGE
        self.assertIn('id="bench-reset-c"', page)
        self.assertIn('min="%.2f"' % espnow_drive.TRIMLEARN_C_MIN, page)
        self.assertIn('max="%.2f"' % espnow_drive.TRIMLEARN_C_MAX, page)
        self.assertIn('id="bench-reset"', page)
        self.assertIn('id="bench-learn-c"', page)

    def test_the_plain_buttons_do_not_reset(self):
        page = espnow_drive.PAGE
        for kind in ("'left'", "'right'", "'both'"):
            self.assertIn("runBench(%s, 0)" % kind, page)


class PAssistControlTest(unittest.TestCase):
    """The runtime P switch. Runtime and not a rebuild, so both arms of an A/B
    run the same binary -- rebuilding between arms would let a build difference
    pass for a result."""

    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.connected = True
        self.link.p_assist_on = False
        self.sent = []
        self.link._write_locked = lambda p: (self.sent.append(p), True)[1]

    def parsed(self):
        out = []
        for payload in self.sent:
            m = self.link.pb2.BoatMessage()
            m.ParseFromString(payload)
            out.append(m)
        return out

    def test_on_and_off_are_serialized(self):
        for want in (True, False, True):
            self.sent.clear()
            ok, err = self.link.send_assist(want)
            self.assertTrue(ok, err)
            m = self.parsed()[0]
            self.assertEqual(m.WhichOneof('payload'), 'assist')
            self.assertEqual(m.assist.p_on, want)
            self.assertEqual(self.link.p_assist_on, want)

    def test_it_defaults_off(self):
        """OFF is the control arm. It must be the default so a forgotten
        toggle cannot silently turn every run into a B."""
        self.assertFalse(self.link.p_assist_on)
        page = espnow_drive.PAGE
        self.assertIn('P ASSIST: OFF', page)
        self.assertIn('var pAssistOn = false;', page)

    def test_a_disconnected_link_sends_nothing_and_does_not_flip_state(self):
        self.link.connected = False
        ok, err = self.link.send_assist(True)
        self.assertFalse(ok)
        self.assertEqual(self.sent, [])
        self.assertFalse(self.link.p_assist_on)

    def test_a_failed_write_does_not_flip_state(self):
        """The tool's idea of the arm must never drift from the boat's."""
        self.link._write_locked = lambda p: False
        ok, _err = self.link.send_assist(True)
        self.assertFalse(ok)
        self.assertFalse(self.link.p_assist_on)

    def test_the_page_has_the_control(self):
        self.assertIn('id="p-assist"', espnow_drive.PAGE)
        self.assertIn("'/api/assist'", espnow_drive.PAGE)


class PAssistConfirmTest(unittest.TestCase):
    """The boat's own p_on, shown next to what the tool asked for. If the two
    ever disagree the A/B is void, so it must be visible rather than assumed."""

    def setUp(self):
        self.link = espnow_drive.BoatLink.__new__(espnow_drive.BoatLink)
        self.link._lock = threading.Lock()
        self.link.pb2 = espnow_drive.load_boat_pb2()
        self.link.bench_status = espnow_drive.BoatLink._blank_bench_status()
        self.link.telemetry = espnow_drive.BoatLink._blank_telemetry()
        self.link._diag_counts = {}

    def feed(self, p_on):
        msg = self.link.pb2.BoatMessage()
        msg.bench_status.state = 2
        msg.bench_status.base = 0.40
        msg.bench_status.learn_c = 0.170
        msg.bench_status.p_on = p_on
        frame = espnow_drive.build_frame(espnow_drive.MSG_SENSOR,
                                         msg.SerializeToString(), seq=4)
        self.link._handle_incoming_frame(frame[:-1])

    def test_the_boats_own_p_on_is_decoded(self):
        self.feed(True)
        self.assertTrue(self.link.bench_status['p_on'])
        self.feed(False)
        self.assertFalse(self.link.bench_status['p_on'])

    def test_a_blank_status_has_the_field(self):
        self.assertIn('p_on', espnow_drive.BoatLink._blank_bench_status())

    def test_the_page_shows_the_boats_answer_and_flags_a_mismatch(self):
        page = espnow_drive.PAGE
        self.assertIn('id="p-confirm"', page)
        self.assertIn("bn.p_on", page)
        self.assertIn("bp !== pAssistOn", page)
