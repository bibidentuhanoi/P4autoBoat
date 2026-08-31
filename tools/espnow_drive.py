#!/usr/bin/env python3
"""
Drive the boat over ESP-NOW: laptop -> S3 (USB CDC) -> C6 -> P4 -> pipeline.

The boat-receiving-commands side already exists and is proven from the
telemetry work:
    S3 cdc_rx_callback (COBS decode -> esp_now_send, broadcast)
      -> C6 espnow_recv_cb (forwards raw bytes verbatim)
        -> esp_hosted PEER_MSG_UPSTREAM -> P4 upstream_cb
          -> pipeline_handle_incoming() -> motor/steer/arm handlers
This script is the laptop-side sender for that path.

The link is bidirectional, though: the boat has ALSO been continuously
broadcasting a full SensorSnapshot (IMU + GPS + ToF + detections) over the
same ESP-NOW link since the telemetry work (sensor_task.c populates GPS on
every snapshot already -- no firmware change needed for this). The S3
forwards whatever it hears off-air to USB as COBS frames, same as it does for
visualize.py's --serial mode. This script reads that same serial connection
and surfaces GPS + IMU heading/pitch/roll (not the ToF grids or camera --
this is a control tool, not the full dashboard).

Wire format per command (must exactly match espnow_transport.c's upstream_cb
and espnow_protocol.h's espnow_pkt_hdr_t):

    [msg_type: u8][payload_len: u16 LE][seq: u8][protobuf BoatMessage bytes]

COBS-encoded, 0x00-delimited, written to the S3's USB CDC port. msg_type only
has to not be MSG_GROUND_HELLO (0x12, reserved for the ground-station beacon)
-- upstream_cb dispatches by the protobuf oneof tag, not this byte. The S3
decodes the COBS frame and esp_now_send()s it as a single broadcast packet;
control messages are a few bytes, nowhere near the 250 B ESP-NOW v1 limit, so
there is no fragmentation to do here (unlike the telemetry/JPEG downlink).

This is a local web app, not a terminal one: it runs a small HTTP server on
127.0.0.1 (never exposed on the network -- this endpoint can arm and drive a
real boat, it must not be reachable by anyone else on the LAN) and serves a
control page styled to match dashboard.html (same --bg/--card/--accent/--green
tokens, same "bench (no GPS)" force-arm checkbox, same ARM button states).
Connect happens IN the page (pick a serial port, click Connect) instead of a
CLI argument, and force-arm is a checkbox you can flip at any time instead of
a flag decided before the process starts.

Unlike dashboard.html, the browser here never touches protobuf -- the page
talks plain JSON to this script's own HTTP API, and THIS PROCESS (already
holding the tested COBS/protobuf encoder) is what writes framed bytes to the
serial port. Nothing in the browser needs to stay in sync with boat.proto.

Throttle and rudder stream continuously at ~15 Hz for as long as a serial port
is connected, regardless of whether anything changed. Nonzero winch commands
have a separate 300ms laptop-side lease which the browser renews while a button
is physically held; losing/backgrounding the page therefore stops the winch
even though the serial process remains connected. The continuous base stream
feeds the firmware's control-link-loss failsafe (motor_control.c), which stops
the boat if the whole stream goes silent for 400ms.

NOTE: the armed/force state shown in the page is still just the last command
WE SENT, never confirmed by the boat -- MotorStatus (which would confirm it)
isn't decoded here, only SensorSnapshot (GPS/IMU) is. If ARM doesn't visibly
do anything on the boat, that's consistent with "sent but refused" (no GPS
lock, force unchecked) as much as "never arrived" -- telemetry moving proves
the LINK is alive, not that any specific command was accepted.

SAFETY: bench-test on blocks before this ever touches water. An unexpected
throttle with the propeller in the water is the one genuinely dangerous
moment. "bench (no GPS)" is an explicit bench/indoor override of the arm
gate in motor_control.c -- know why you're checking it.

Usage:
    .venv/bin/python tools/espnow_drive.py
    .venv/bin/python tools/espnow_drive.py --http-port 9000
Then open the printed http://127.0.0.1:PORT/ URL (done automatically if a
browser is available).
"""
import base64
import hashlib
import json
import math
import csv
import os
import re
import struct
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# This script lives in tools/, but proto/boat_pb2.py is a repo-root package --
# running it as `python tools/espnow_drive.py` (the documented usage) puts
# tools/ on sys.path, not the repo root, so `from proto import boat_pb2` would
# fail. Insert the repo root explicitly rather than requiring callers to `cd`
# or set PYTHONPATH first.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

MSG_SENSOR = 0x02              # espnow_pkt_hdr_t.msg_type -- legacy full SensorSnapshot
MSG_BRIDGE_STATUS = 0x13       # S3's own self-report -- USB-only, never over the air
MSG_MOTOR_CMD = 0x03           # espnow_pkt_hdr_t.msg_type -- see module docstring
MSG_MOTOR_STATUS = 0x06        # MotorStatus -- the boat's ACTUAL arm/throttle/servo state (vs commanded)
MSG_STATUS = 0x07              # SystemStatus (sensor-health / boot report); sent over the field link too
# Field-mode telemetry, IMU+GPS only -- a hand-packed struct (main/transports/
# espnow_protocol.h: espnow_telemetry_t), NOT a boat.proto message. Replaces
# MSG_SENSOR on the ESP-NOW link: fits one ESP-NOW packet (no fragmentation),
# where a full SensorSnapshot with ToF needed ~55 fragments and could stall
# the boat's own publish loop. Layout must match espnow_telemetry_t exactly:
# pitch,roll,heading (f) + gps_valid (B) + lat,lon (d) + speed,course (f) +
# satellites (B) + hdop (f) + yaw_rate (f), packed, little-endian.
# yaw_rate is gyro-Z deg/s -- the signal the bench mismatch test reads.
MSG_FIELD_TELEMETRY = 0x08
FIELD_TELEMETRY_FMT = '<fffBddffBff'
ESPNOW_HDR_SIZE = 4
ESP_NOW_MAX_DATA_LEN = 250     # ESP-NOW v1 single-packet limit
SEND_HZ = 15
HTTP_HOST = '127.0.0.1'        # LAN-reachable would let anyone drive the boat
TELEMETRY_STALE_S = 2.0        # snapshot task runs ~20Hz normally -- 2s is a generous margin
WINCH_LEASE_S = 0.30           # nonzero command must be renewed before firmware's 400ms failsafe

WS_MAGIC = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'   # RFC 6455 handshake constant
WS_PUSH_HZ = 20                 # status push rate over /ws -- at least matches the
                                 # boat's own telemetry ceiling, so this is never the
                                 # bottleneck; used to be a 400ms client-side poll (2.5Hz)


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode data (no trailing delimiter). Mirrors espnow_protocol.h's
    cobs_encode() -- output never contains 0x00, caller appends the 0x00
    frame delimiter separately."""
    out = bytearray()
    code_pos = 0
    out.append(0)              # placeholder, patched when this block closes
    code = 1

    for byte in data:
        if byte == 0:
            out[code_pos] = code
            code_pos = len(out)
            out.append(0)      # placeholder for the next block
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:   # 254 literal bytes since the last code byte
                out[code_pos] = code
                code_pos = len(out)
                out.append(0)
                code = 1
    out[code_pos] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decode one COBS frame (delimiter already stripped). b'' on malformed
    input. Same algorithm as visualize.py / espnow_protocol.h -- this is the
    telemetry downlink's decoder, the mirror image of cobs_encode() above."""
    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        idx += 1
        if code == 0:
            return bytes()
        num = code - 1
        if idx + num > len(data):
            return bytes()
        out.extend(data[idx:idx + num])
        idx += num
        if code < 0xFF and idx < len(data):
            out.append(0)
    return bytes(out)


def build_frame(msg_type: int, payload: bytes, seq: int) -> bytes:
    """espnow_pkt_hdr_t + payload, COBS-encoded with a 0x00 delimiter --
    exactly what upstream_cb expects to read off the air."""
    if ESPNOW_HDR_SIZE + len(payload) > ESP_NOW_MAX_DATA_LEN:
        raise ValueError(
            f"payload {len(payload)} B would make a "
            f"{ESPNOW_HDR_SIZE + len(payload)} B packet, over the "
            f"{ESP_NOW_MAX_DATA_LEN} B ESP-NOW limit")
    hdr = struct.pack('<BHB', msg_type, len(payload), seq & 0xFF)
    return cobs_encode(hdr + payload) + b'\x00'


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ---- bench throttle-mismatch test -------------------------------------------
# Three runs -- left-stronger, right-stronger, both-same. The both-same run IS
# the mismatch (equal commands, so any turn is the two motors disagreeing); the
# two lopsided runs give the scale, in deg/s per unit of commanded difference.
# Every run starts with the motors OFF so the gyro's own drift can be measured
# at run temperature and subtracted -- that drift lands directly on top of the
# both-same reading, which is the one number the trim depends on.
# Bench throttle-mismatch test. The BOAT owns the run: it drives the profile
# and records every sample to its OWN SD card, so a radio dropout cannot spoil
# the measurement -- this tool only presses the button and shows progress.
# Reading the runs afterwards is tools/bench_analyze.py.
BENCH_KIND = {'both': 0, 'left': 1, 'right': 2}
BENCH_KIND_NAME = {0: 'BASE', 1: 'LEFT', 2: 'RIGHT'}
BENCH_STATE_NAME = {0: 'idle', 1: 'still', 2: 'driving', 3: 'coasting',
                    4: 'SAVED', 5: 'FAILED'}
# A live run publishes its state at ~5 Hz. Anything older than this cannot
# still be running -- most likely the terminal SAVED packet was lost over the
# air. Without this the tool latches on 'driving' and refuses every later run.
BENCH_RUNNING_STALE_S = 3.0

# The learner's clamp, mirrored from motor_control.c (.c_min / .c_max). A value
# outside this is refused by the boat, so refuse it here too rather than send a
# command that will be silently dropped.
TRIMLEARN_C_MIN = 0.10
TRIMLEARN_C_MAX = 0.35


def load_boat_pb2():
    try:
        from proto import boat_pb2
        return boat_pb2
    except Exception as exc:                            # noqa: BLE001
        print('FATAL: cannot load proto/boat_pb2.py: '
              f'{type(exc).__name__}: {exc}', file=sys.stderr)
        print(f'interpreter: {sys.executable}', file=sys.stderr)
        print('Run with the repo venv (where the binding was generated):',
              file=sys.stderr)
        print('  .venv/bin/python tools/espnow_drive.py', file=sys.stderr)
        print('If the file is missing:  tools/gen_proto.sh', file=sys.stderr)
        sys.exit(1)


class BoatLink:
    """Owns the serial connection and the continuous 15 Hz send loop.

    Connect/disconnect happen at runtime, driven by the web UI -- there is no
    serial port at construction time. The background thread runs for the
    life of the process; it only actually writes once `connected` is True, so
    starting it unconditionally in __init__ keeps connect() itself simple (no
    "first connect ever" special case to get the thread going).
    """

    def __init__(self, boat_pb2, send_hz: float = SEND_HZ):
        self.pb2 = boat_pb2
        self.send_hz = send_hz
        self._lock = threading.Lock()
        self.ser = None
        self.port = None
        self.connected = False
        self.throttle = 0.0          # linked "both motors" value
        self.motor_left = 0.0        # per-motor values, used when unlinked (split)
        self.motor_right = 0.0
        self.motor_split = False     # False = linked (throttle -> both); True = independent L/R
        self.rudder = 0.0
        self.winch_speed = 0.0
        self.winch_lease_until = 0.0
        self.winch_command_seq = 0
        # None means this laptop session has not commanded the rail yet. This
        # is deliberately not presented as boat-confirmed telemetry.
        self.servo_rail_cut = None
        self.armed_cmd = False
        self.force = False
        # True while an ESC-trim calibration sweep is running. The firmware owns
        # the actuators then; the stream loop sends ONLY a CalibrateCommand
        # keepalive (never motor/steer/winch, which would trip the abort).
        self.calibrating = False
        self.seq = 0
        self.last_error = None
        self.telemetry = self._blank_telemetry()
        self.calibrate_status = self._blank_calibrate_status()
        self.system_status = self._blank_system_status()
        self.motor_status = self._blank_motor_status()
        self.bench_status = self._blank_bench_status()
        # Mirrors the boat's runtime P switch. Default OFF -- the A arm must be
        # the default so a forgotten toggle cannot silently make every run a B.
        self.p_assist_on = False
        self.bridge_status = self._blank_bridge_status()
        self._diag_counts = {}
        self._stop = threading.Event()
        threading.Thread(target=self._stream_loop, daemon=True).start()
        threading.Thread(target=self._read_loop, daemon=True).start()

    @staticmethod
    def _blank_telemetry() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'heading': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'gps_valid': False, 'lat': 0.0, 'lon': 0.0,
            'speed_mps': 0.0, 'course_deg': 0.0,
            'satellites': 0, 'hdop': 0.0, 'yaw_rate': 0.0,
        }

    @staticmethod
    def _blank_calibrate_status() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'state': 0, 'level_index': 0, 'level_throttle': 0.0,
            'trim_diff': 0.0, 'yaw_avg_dps': 0.0, 'making_way': False,
            'points_done': 0,
        }

    @staticmethod
    def _blank_system_status() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'camera_ok': False, 'tof_a_ok': False, 'tof_b_ok': False,
            'imu_ok': False, 'mag_ok': False, 'gps_ok': False,
            'gps_detected_baud': 0, 'gps_baud_confirmed': False,
        }

    @staticmethod
    def _blank_motor_status() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'state': 0, 'left_throttle': 0.0, 'right_throttle': 0.0,
            'winch_speed': 0.0, 'servo_power': False,
        }

    @staticmethod
    def _blank_bench_status() -> dict:
        """Boat-reported bench-run state. The run itself lives on the boat and
        the data lands on its SD card; this is only progress and which file
        number it saved as."""
        return {
            'have': False, 'last_rx_monotonic': None,
            'state': 0, 'kind': 0, 'base': 0.0,
            'samples': 0, 'file_index': 0, 'elapsed_s': 0.0,
            'learn_c': 0.0, 'p_on': False,
        }

    @staticmethod
    def _blank_bridge_status() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'uptime_s': 0, 'espnow_pkts': 0, 'espnow_bytes': 0,
            'frames_out': 0, 'hello_sent': 0, 'reasm_drops': 0,
            # Task 13 decodes these signed bytes. None keeps older S3
            # firmware's 24-byte status payload distinguishable from a real 0.
            'uplink_rssi_dbm': None, 'lr_rate_config_ok': None,
        }

    # ---- internal, must hold self._lock ----

    def _write_locked(self, payload: bytes):
        frame = build_frame(MSG_MOTOR_CMD, payload, self.seq)
        self.seq = (self.seq + 1) & 0xFF
        try:
            self.ser.write(frame)
            self.last_error = None
            return True
        except Exception as exc:                        # noqa: BLE001
            self.last_error = str(exc)
            return False

    def _send_motor_locked(self, left: float, right: float):
        msg = self.pb2.BoatMessage()
        msg.motor.left = left
        msg.motor.right = right
        return self._write_locked(msg.SerializeToString())

    def _send_steer_locked(self, rudder: float):
        msg = self.pb2.BoatMessage()
        msg.steer.left = rudder
        msg.steer.right = rudder
        return self._write_locked(msg.SerializeToString())

    def _send_winch_locked(self, speed: float):
        msg = self.pb2.BoatMessage()
        msg.winch.speed = speed
        return self._write_locked(msg.SerializeToString())

    def _send_servo_power_locked(self, on: bool):
        msg = self.pb2.BoatMessage()
        msg.servo_power.on = on
        return self._write_locked(msg.SerializeToString())

    def _send_training_log_locked(self):
        msg = self.pb2.BoatMessage()
        msg.training_log.SetInParent()   # empty message -- its presence IS the trigger
        return self._write_locked(msg.SerializeToString())

    def _send_arm_locked(self, arm: bool, force: bool):
        msg = self.pb2.BoatMessage()
        msg.arm_cmd.arm = arm
        msg.arm_cmd.force = force
        self._write_locked(msg.SerializeToString())

    def _send_calibrate_locked(self, start: bool):
        msg = self.pb2.BoatMessage()
        msg.calibrate.start = start
        # average_into_existing left false: the firmware records FRESH in v1.
        return self._write_locked(msg.SerializeToString())

    def _close_locked(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:                                # noqa: BLE001
            pass
        self.ser = None
        self.connected = False
        self.port = None

    # ---- public API, called from HTTP handler threads ----

    def connect(self, port: str):
        import serial as pyserial
        with self._lock:
            if self.connected:
                return False, 'already connected; disconnect first'
            try:
                self.ser = pyserial.Serial(port, 921600, timeout=0.1)
            except Exception as exc:                     # noqa: BLE001
                self.last_error = str(exc)
                return False, str(exc)
            self.port = port
            self.connected = True
            # Every connect starts from a known-safe, all-zero, disarmed
            # state -- never carry over values from a previous session.
            self.throttle = 0.0
            self.motor_left = 0.0
            self.motor_right = 0.0
            self.motor_split = False
            self.rudder = 0.0
            self.winch_speed = 0.0
            self.winch_lease_until = 0.0
            self.servo_rail_cut = None
            self.armed_cmd = False
            self.force = False
            self.last_error = None
            self.telemetry = self._blank_telemetry()
            self.bridge_status = self._blank_bridge_status()
            return True, None

    def disconnect(self):
        with self._lock:
            if not self.connected:
                return
            self.winch_command_seq += 1
            self.throttle = 0.0
            self.motor_left = 0.0
            self.motor_right = 0.0
            self.motor_split = False
            self.rudder = 0.0
            self.winch_speed = 0.0
            self.winch_lease_until = 0.0
            self._send_motor_locked(0.0, 0.0)
            self._send_steer_locked(0.0)
            self._send_winch_locked(0.0)
            if self.armed_cmd:
                self._send_arm_locked(False, self.force)
                self.armed_cmd = False
            self._close_locked()

    def set_state(self, throttle=None, rudder=None, left=None, right=None):
        with self._lock:
            if throttle is not None:
                self.throttle = clamp(float(throttle), -1.0, 1.0)
                self.motor_split = False          # linked: throttle drives both
            if left is not None:
                self.motor_left = clamp(float(left), -1.0, 1.0)
                self.motor_split = True           # unlinked: independent per-motor
            if right is not None:
                self.motor_right = clamp(float(right), -1.0, 1.0)
                self.motor_split = True
            if rudder is not None:
                self.rudder = clamp(float(rudder), -1.0, 1.0)

    def set_winch(self, speed: float, command_seq: int):
        """Set winch speed and transmit immediately; the stream loop repeats
        the same value while it is held, and a release gets a zero out now."""
        with self._lock:
            if command_seq <= self.winch_command_seq:
                return False, 'stale command sequence'
            self.winch_command_seq = command_seq
            if not self.connected:
                self.winch_speed = 0.0
                self.winch_lease_until = 0.0
                return False, 'serial link is disconnected'
            self.winch_speed = clamp(float(speed), -1.0, 1.0)
            if self.winch_speed != 0.0 and self.servo_rail_cut is not False:
                self.winch_speed = 0.0
                self.winch_lease_until = 0.0
                return False, 'servo rail must be powered on before moving winch'
            self.winch_lease_until = (
                time.monotonic() + WINCH_LEASE_S
                if self.winch_speed != 0.0 else 0.0)
            if not self._send_winch_locked(self.winch_speed):
                self.winch_speed = 0.0
                self.winch_lease_until = 0.0
                return False, 'serial write failed'
            return True, None

    def set_servo_power(self, on: bool, command_seq: int):
        """Center rail actuators, then command the shared servo power rail."""
        with self._lock:
            if command_seq <= self.winch_command_seq:
                return False, 'stale command sequence'
            self.winch_command_seq = command_seq
            if not self.connected:
                self.winch_speed = 0.0
                self.winch_lease_until = 0.0
                return False, 'serial link is disconnected'
            self.rudder = 0.0
            self.winch_speed = 0.0
            self.winch_lease_until = 0.0
            steer_zero_sent = self._send_steer_locked(0.0)
            zero_sent = self._send_winch_locked(0.0)
            if on and not (steer_zero_sent and zero_sent):
                return False, 'serial write failed'
            if not self._send_servo_power_locked(on):
                return False, 'serial write failed'
            self.servo_rail_cut = not on
            return True, None

    def stop(self, command_seq: int):
        """Panic stop: zero throttle/rudder and send immediately rather than
        waiting for the next background tick. Does not touch armed_cmd --
        matches the firmware's own link-loss failsafe, which centres/zeroes
        but leaves the ARM decision to an explicit command."""
        with self._lock:
            if command_seq <= self.winch_command_seq:
                return False, 'stale command sequence'
            self.winch_command_seq = command_seq
            self.throttle = 0.0
            self.motor_left = 0.0
            self.motor_right = 0.0
            self.motor_split = False
            self.rudder = 0.0
            self.winch_speed = 0.0
            self.winch_lease_until = 0.0
            was_calibrating = self.calibrating
            self.calibrating = False          # STOP is also a calibration kill
            if not self.connected:
                return False, 'serial link is disconnected'
            writes_ok = (
                self._send_motor_locked(0.0, 0.0),
                self._send_steer_locked(0.0),
                self._send_winch_locked(0.0),
            )
            if was_calibrating:
                self._send_calibrate_locked(False)   # re-arm the firmware start latch
            if not all(writes_ok):
                return False, 'serial write failed'
            return True, None

    def set_calibrate(self, start: bool, command_seq: int):
        """Start or stop an ESC-trim calibration sweep. While started, the stream
        loop sends a CalibrateCommand keepalive (~send_hz) instead of the usual
        motor/steer/winch triple -- the firmware gates calibration on its own
        heartbeat and aborts on any manual command, so we must send neither. Stop
        (or any manual re-engagement) hands authority straight back."""
        with self._lock:
            if command_seq <= self.winch_command_seq:
                return False, 'stale command sequence'
            self.winch_command_seq = command_seq
            if not self.connected:
                return False, 'serial link is disconnected'
            if start and not self.armed_cmd:
                return False, 'ARM first — calibration drives the thrusters'
            if start and self._bench_running_locked():
                return False, 'a bench run is going \u2014 wait for it to finish'
            self.calibrating = bool(start)
            if not start:                    # leaving calibration: centre/zero
                self.throttle = 0.0
                self.motor_left = 0.0
                self.motor_right = 0.0
                self.motor_split = False
                self.rudder = 0.0
                self.winch_speed = 0.0
                self.winch_lease_until = 0.0
            if not self._send_calibrate_locked(bool(start)):
                self.calibrating = False
                return False, 'serial write failed'
            return True, None

    def arm(self, do_arm: bool, force: bool):
        with self._lock:
            self.armed_cmd = bool(do_arm)
            self.force = bool(force)
            # NOTE: disarming is also how a boat-side bench run is stopped --
            # the boat's own bench_step aborts the moment it sees !armed.
            if not do_arm and self.calibrating:   # disarm must stop calibration
                self.calibrating = False
                if self.connected:
                    self._send_calibrate_locked(False)
            if self.connected:
                self._send_arm_locked(self.armed_cmd, self.force)

    def _bench_running_locked(self):
        """True only for a run we have HEARD FROM recently. The boat publishes
        the terminal state exactly once, so a lost packet must never leave this
        stuck at 'running' -- that would refuse every run until a restart."""
        if self.bench_status['state'] not in (1, 2, 3):
            return False
        last = self.bench_status.get('last_rx_monotonic')
        if last is None:
            return False
        return (time.monotonic() - last) < BENCH_RUNNING_STALE_S

    def send_assist(self, p_on):
        """Turn the temporary P yaw assist on or off, at RUNTIME.

        Runtime and not a rebuild, so both arms of an A/B experiment run the
        same binary -- rebuilding between arms would let a build difference
        pass for a result. OFF returns the boat to exactly the pre-P path: the
        firmware drops the correction rather than letting it decay."""
        with self._lock:
            if not self.connected:
                return False, 'serial link is disconnected'
            msg = self.pb2.BoatMessage()
            msg.assist.p_on = bool(p_on)
            if not self._write_locked(msg.SerializeToString()):
                return False, 'serial write failed'
            self.p_assist_on = bool(p_on)
            return True, None

    def send_bench(self, kind, base, delta, command_seq, reset_c=0.0):
        """Ask the BOAT to run one bench test and record it to its own SD card.

        Once this is sent the boat owns the run end to end, so a radio dropout
        afterwards cannot spoil the measurement. Stopping a run mid-flight is
        done by DISARM -- the boat aborts the moment it sees itself disarmed.

        reset_c > 0 additionally restarts the trim learner from that c, ONCE,
        and only if the boat accepts the run. It exists for one experiment:
        start low, start high, and see whether both walk to the same place.
        Every ordinary run must send 0 -- c carrying over between runs is what
        makes convergence observable, so a reset on every run would destroy
        the measurement."""
        with self._lock:
            if command_seq <= self.winch_command_seq:
                return False, 'stale command sequence'
            self.winch_command_seq = command_seq
            if kind not in BENCH_KIND:
                return False, 'unknown run type'
            if not self.connected:
                return False, 'serial link is disconnected'
            if not self.armed_cmd:
                return False, 'ARM first \u2014 the bench run spins the thrusters'
            if self.calibrating:
                return False, 'calibration is running \u2014 stop it first'
            if self._bench_running_locked():
                return False, 'a bench run is already going'
            try:
                base = clamp(float(base), 0.0, 1.0)
                delta = clamp(float(delta), 0.0, 1.0)
                reset_c = float(reset_c)
            except (TypeError, ValueError):
                return False, 'throttle, delta and reset-c must be numbers'
            if not math.isfinite(reset_c):
                return False, 'reset-c must be a finite number'
            # NOT clamped: silently pulling an out-of-range value to the edge
            # would start the experiment from a c the operator never chose.
            if reset_c != 0.0 and not (TRIMLEARN_C_MIN <= reset_c <= TRIMLEARN_C_MAX):
                return False, ('reset-c must be between %.2f and %.2f (or 0 for '
                               'no reset)' % (TRIMLEARN_C_MIN, TRIMLEARN_C_MAX))
            # Our own outputs go to zero: the boat drives the run from here.
            self.throttle = 0.0
            self.motor_left = 0.0
            self.motor_right = 0.0
            self.motor_split = False
            self.rudder = 0.0
            msg = self.pb2.BoatMessage()
            msg.bench.kind = BENCH_KIND[kind]
            msg.bench.base = base
            msg.bench.delta = delta
            msg.bench.reset_c = reset_c
            if not self._write_locked(msg.SerializeToString()):
                return False, 'serial write failed'
            return True, None

    def _handle_bench_status(self, bs):
        with self._lock:
            self.bench_status = {
                'have': True, 'last_rx_monotonic': time.monotonic(),
                'state': int(bs.state), 'kind': int(bs.kind),
                'base': float(bs.base), 'samples': int(bs.samples),
                'file_index': int(bs.file_index),
                'elapsed_s': float(bs.elapsed_s),
                'learn_c': float(bs.learn_c),
                # The BOAT's own answer, not what this tool asked for. If the
                # two disagree the A/B is void, so it has to be visible.
                'p_on': bool(bs.p_on),
            }

    def trigger_record(self):
        """Fire-and-forget dataset capture (Feature 1): the boat saves a
        full-quality JPEG + sensor sidecar to SD on its own. There is no ack
        channel -- success here means the command was sent, not that the capture
        landed (mirrors dashboard.html's Record button). Not gated on arm: a
        TrainingLogCommand never touches the drive link, so it can't disturb a
        calibration sweep either."""
        with self._lock:
            if not self.connected:
                return False, 'serial link is disconnected'
            if not self._send_training_log_locked():
                return False, 'serial write failed'
            return True, None

    @staticmethod
    def _with_age(d: dict, stale_s: float) -> dict:
        out = dict(d)
        if out['last_rx_monotonic'] is not None:
            age_s = time.monotonic() - out['last_rx_monotonic']
        else:
            age_s = None
        del out['last_rx_monotonic']
        out['age_s'] = age_s
        out['stale'] = (age_s is None) or (age_s > stale_s)
        return out

    def status(self) -> dict:
        with self._lock:
            return {
                'connected': self.connected,
                'port': self.port,
                'throttle': self.throttle,
                'motor_left': self.motor_left,
                'motor_right': self.motor_right,
                'motor_split': self.motor_split,
                'rudder': self.rudder,
                'winch_speed': self.winch_speed,
                'winch_command_seq': self.winch_command_seq,
                # This is only the last command this local UI sent, never a
                # claim that the P4 has reported its actual rail state.
                'servo_rail_cut': self.servo_rail_cut,
                'armed_cmd': self.armed_cmd,
                'force': self.force,
                'calibrating': self.calibrating,
                'calibrate': self._with_age(self.calibrate_status, TELEMETRY_STALE_S),
                'system_status': self._with_age(self.system_status, 5.0),
                # Sample list deliberately excluded -- the browser polls this
                # often and the rows belong in the file, not the status blob.
                'bench': self._with_age(self.bench_status, 5.0),
                'motor_status': self._with_age(self.motor_status, TELEMETRY_STALE_S),
                'seq': self.seq,
                'last_error': self.last_error,
                'telemetry': self._with_age(self.telemetry, TELEMETRY_STALE_S),
                # Bridge status arrives ~1/s from the S3 -- a longer stale
                # window than telemetry is correct, not a copy/paste of it.
                'bridge_status': self._with_age(self.bridge_status, 3.0),
            }

    def _handle_system_status(self, st):
        """Store a SystemStatus (boot / sensor-health report). The boat already
        sends this ~1Hz over the field link; the tool used to drop MSG_STATUS,
        which is why 'NO FIX' couldn't be told apart from 'GPS chip not even
        wired'. gps_ok = the chip is talking to the UART, independent of a
        satellite fix. Boot flags (camera/tof/imu/mag) don't change after boot."""
        with self._lock:
            self.system_status = {
                'have': True, 'last_rx_monotonic': time.monotonic(),
                'camera_ok': bool(st.camera_ok), 'tof_a_ok': bool(st.tof_a_ok),
                'tof_b_ok': bool(st.tof_b_ok), 'imu_ok': bool(st.imu_ok),
                'mag_ok': bool(st.mag_ok), 'gps_ok': bool(st.gps_ok),
                'gps_detected_baud': int(st.gps_detected_baud),
                'gps_baud_confirmed': bool(st.gps_baud_confirmed),
            }

    def _handle_motor_status(self, ms):
        """Store a MotorStatus -- the boat's ACTUAL motor state / throttles /
        winch / servo-rail power, versus what we commanded. state is esc_state_t:
        0 disarmed, 1 arming, 2 armed. This is the confirmation the tool never
        had (it only showed the commanded arm) and the real per-motor throttle,
        handy for a single-ESC spin test."""
        with self._lock:
            self.motor_status = {
                'have': True, 'last_rx_monotonic': time.monotonic(),
                'state': int(ms.state),
                'left_throttle': float(ms.left_throttle),
                'right_throttle': float(ms.right_throttle),
                'winch_speed': float(ms.winch_speed),
                'servo_power': bool(ms.servo_power),
            }

    def _handle_calibrate_status(self, cs):
        """Store a CalibrateStatus received from the boat. State follows
        etc_state_t: 0 idle, 1 measure-noise, 2 ramp, 3 settle, 4 DONE,
        5 ABORTED. DONE/ABORTED are terminal -> the firmware has stopped, so
        drop our own calibrating flag even though we never sent a stop (the
        boat reached the end on its own)."""
        with self._lock:
            self.calibrate_status = {
                'have': True, 'last_rx_monotonic': time.monotonic(),
                'state': cs.state, 'level_index': cs.level_index,
                'level_throttle': cs.level_throttle, 'trim_diff': cs.trim_diff,
                'yaw_avg_dps': cs.yaw_avg_dps, 'making_way': cs.making_way,
                'points_done': cs.points_done,
            }
            if cs.state in (4, 5):
                self.calibrating = False

    def _stream_loop(self):
        period = 1.0 / self.send_hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                if self.connected:
                    if self.calibrating:
                        # Firmware owns the actuators; send ONLY the keepalive.
                        # Any motor/steer/winch here would trip the abort.
                        self._send_calibrate_locked(True)
                    else:
                        if (self.winch_speed != 0.0 and
                                time.monotonic() >= self.winch_lease_until):
                            self.winch_speed = 0.0
                        if self.motor_split:
                            self._send_motor_locked(self.motor_left, self.motor_right)
                        else:
                            self._send_motor_locked(self.throttle, self.throttle)
                        self._send_steer_locked(self.rudder)
                        self._send_winch_locked(self.winch_speed)
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()   # fell behind -- resync, don't spin

    def _diag(self, reason: str, detail: str = ''):
        """Print the first few occurrences of each distinct MSG_SENSOR
        rejection reason, then go quiet for that reason (still counted).
        Diagnostic-only -- BRIDGE_STATUS proved the S3 IS reassembling
        complete frames, so a MSG_SENSOR frame that reaches here and still
        fails is failing in THIS process, and swallowing that silently
        (the previous behaviour) is exactly what made it undiagnosable."""
        n = self._diag_counts.get(reason, 0) + 1
        self._diag_counts[reason] = n
        if n <= 5:
            msg = f'[rx] MSG_SENSOR rejected: {reason}'
            if detail:
                msg += f' -- {detail}'
            if n == 5:
                msg += ' (further occurrences counted silently)'
            print(msg, flush=True)

    def _handle_incoming_frame(self, cobs_frame: bytes):
        """One COBS-delimited frame off the serial line. Message types that
        matter here: MSG_FIELD_TELEMETRY (current firmware's IMU+GPS struct),
        MSG_SENSOR (legacy full SensorSnapshot protobuf, kept decodable in
        case older firmware is ever run against this tool), and
        MSG_BRIDGE_STATUS (the S3's own self-report of what it's heard
        off-air -- USB only, never over the radio). Anything else (a stray
        MSG_GROUND_HELLO echo) is silently ignored -- this tool doesn't
        surface the full diagnostic set visualize.py does, just enough to
        tell "boat is transmitting and S3 hears it" apart from "S3 hears
        nothing"."""
        raw = cobs_decode(cobs_frame)
        if len(raw) < ESPNOW_HDR_SIZE:
            return
        msg_type, plen, _seq = struct.unpack('<BHB', raw[:4])
        payload = raw[4:4 + plen]

        if msg_type == MSG_BRIDGE_STATUS:
            if len(payload) != plen or len(payload) < 24 or len(payload) == 25:
                return
            uptime_s, pkts, byts, frames, hello, drops = struct.unpack('<6I', payload[:24])
            # Optional Task 13 decode: S3's last received boat RSSI (dBm),
            # then its broadcast-peer LR/250K rate-config result (-1, 0, or 1).
            # Keeping the suffix optional accepts 24-byte reports from older
            # S3 firmware and leaves room for later trailing fields.
            uplink_rssi_dbm = lr_rate_config_ok = None
            if len(payload) >= 26:
                uplink_rssi_dbm, lr_rate_config_ok = struct.unpack('<bb', payload[24:26])
            with self._lock:
                self.bridge_status = {
                    'have': True, 'last_rx_monotonic': time.monotonic(),
                    'uptime_s': uptime_s, 'espnow_pkts': pkts, 'espnow_bytes': byts,
                    'frames_out': frames, 'hello_sent': hello, 'reasm_drops': drops,
                    'uplink_rssi_dbm': uplink_rssi_dbm,
                    'lr_rate_config_ok': lr_rate_config_ok,
                }
            return

        if msg_type == MSG_FIELD_TELEMETRY:
            expect_len = struct.calcsize(FIELD_TELEMETRY_FMT)
            if len(payload) != plen or plen != expect_len:
                self._diag('field telemetry length mismatch',
                            f'expected {expect_len}B, got {len(payload)}B (header said {plen}B)')
                return
            (pitch, roll, heading, gps_valid, lat, lon,
             speed_mps, course_deg, satellites, hdop,
             yaw_rate) = struct.unpack(FIELD_TELEMETRY_FMT, payload)
            self._diag_counts['ok'] = self._diag_counts.get('ok', 0) + 1
            with self._lock:
                self.telemetry = {
                    'have': True, 'last_rx_monotonic': time.monotonic(),
                    'heading': heading, 'pitch': pitch, 'roll': roll,
                    'gps_valid': bool(gps_valid), 'lat': lat, 'lon': lon,
                    'speed_mps': speed_mps, 'course_deg': course_deg,
                    'satellites': satellites, 'hdop': hdop,
                    'yaw_rate': yaw_rate,
                }
            return

        if msg_type == MSG_MOTOR_STATUS:
            # MotorStatus -- actual arm/throttle/servo state. Same decode shape as
            # SystemStatus; own branch so the sensor/telemetry paths stay untouched.
            if len(payload) != plen:
                return
            try:
                msg = self.pb2.BoatMessage()
                msg.ParseFromString(payload)
            except Exception:                            # noqa: BLE001
                return
            if msg.HasField('motor_status'):
                self._handle_motor_status(msg.motor_status)
            return

        if msg_type == MSG_STATUS:
            # SystemStatus (sensor-health / boot report). It's a BoatMessage too,
            # just tagged differently -- decode it the same way, in its own branch
            # so the sensor/telemetry paths below stay untouched.
            if len(payload) != plen:
                return
            try:
                msg = self.pb2.BoatMessage()
                msg.ParseFromString(payload)
            except Exception:                            # noqa: BLE001
                return
            if msg.HasField('status'):
                self._handle_system_status(msg.status)
            return

        if msg_type != MSG_SENSOR:
            return

        # Every return from here on is a MSG_SENSOR frame that failed to
        # become a telemetry update -- diagnose each one, don't guess.
        if len(payload) != plen:
            self._diag('length mismatch',
                        f'header says {plen}B, frame actually has {len(payload)}B '
                        f'(raw frame {len(raw)}B total)')
            return
        try:
            msg = self.pb2.BoatMessage()
            msg.ParseFromString(payload)
        except Exception as exc:                         # noqa: BLE001
            self._diag('protobuf decode failed',
                        f'{type(exc).__name__}: {exc} ({len(payload)}B payload)')
            return
        if not msg.HasField('sensors'):
            # CalibrateStatus rides the same MSG_SENSOR frames (the P4's
            # espnow_send_fn tags calibrate_status via its default case). It's
            # a valid BoatMessage with a different oneof member -- route it, then
            # fall through to the reject for anything genuinely unexpected. The
            # sensor path below is deliberately left untouched.
            if msg.HasField('bench_status'):
                self._handle_bench_status(msg.bench_status)
                return
            if msg.HasField('calibrate_status'):
                self._handle_calibrate_status(msg.calibrate_status)
                return
            self._diag('decoded but no sensors field',
                        f'which_oneof={msg.WhichOneof("payload")!r}')
            return

        self._diag_counts['ok'] = self._diag_counts.get('ok', 0) + 1
        s = msg.sensors
        with self._lock:
            self.telemetry = {
                'have': True, 'last_rx_monotonic': time.monotonic(),
                'heading': s.imu.heading, 'pitch': s.imu.pitch, 'roll': s.imu.roll,
                # Signed, raw, straight through -- same field and same meaning
                # as the compact ESP-NOW path above. This assignment REPLACES
                # the whole dict, so omitting a key does not merely hide the
                # value, it removes it: the renderer would then KeyError on
                # every full-snapshot frame. Both paths must carry the same set.
                'yaw_rate': s.imu.yaw_rate,
                'gps_valid': s.gps.valid, 'lat': s.gps.latitude, 'lon': s.gps.longitude,
                'speed_mps': s.gps.speed_mps, 'course_deg': s.gps.course_deg,
                'satellites': s.gps.satellites, 'hdop': s.gps.hdop,
            }

    def _read_loop(self):
        """Telemetry downlink: same serial connection as the command uplink,
        read from a separate thread (full-duplex USB CDC, standard pyserial
        pattern). Reads happen OUTSIDE self._lock -- the port's own 0.1s
        read timeout means holding the lock across it would periodically
        stall the 15Hz send loop and the HTTP handlers for up to 100ms."""
        buf = bytearray()
        while not self._stop.is_set():
            with self._lock:
                ser = self.ser if self.connected else None
            if ser is None:
                time.sleep(0.1)
                continue
            try:
                chunk = ser.read(4096)
            except Exception:                            # noqa: BLE001
                time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            while b'\x00' in buf:
                i = buf.index(b'\x00')
                frame, buf[:] = bytes(buf[:i]), buf[i + 1:]
                if frame:
                    self._handle_incoming_frame(frame)

    def shutdown(self):
        self.disconnect()
        self._stop.set()


# Colors/classes copied verbatim from main/dashboard.html's :root tokens and
# .btn-arm/.state-dot/.motor-slider-row rules -- see feedback_projection_
# constants in project memory: dashboard.html is the visual source of truth,
# match it exactly rather than reinvent a palette.
PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP-NOW Drive</title>
<link rel="icon" href="data:,">
<style>
  :root {
    --bg:      #05080F;
    --card:    #0B1525;
    --border:  #1A2A44;
    --accent:  #00BFFF;
    --green:   #00FF99;
    --warn:    #FFB300;
    --danger:  #FF3B3B;
    --text:    #C8D8F0;
    --dim:     #4A6080;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: 'Segoe UI', monospace;
         font-size: 13px; min-height: 100vh; display: flex; flex-direction: column;
         align-items: center; padding: 24px 16px; }
  header { display: flex; align-items: center; gap: 12px; margin-bottom: 20px; }
  header h1 { font-size: 15px; letter-spacing: 2px; text-transform: uppercase; color: var(--accent); }
  .pill { padding: 3px 10px; border-radius: 10px; font-size: 10px; letter-spacing: 1px;
          text-transform: uppercase; border: 1px solid var(--border); color: var(--dim); }
  .pill.up { color: var(--green); border-color: var(--green); }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 10px;
          padding: 16px; width: 360px; max-width: 100%; margin-bottom: 16px; overflow: hidden; }
  .card-title { display: flex; align-items: center; flex-wrap: wrap; row-gap: 4px;
                font-size: 12px; letter-spacing: 2px; text-transform: uppercase;
                color: var(--accent); margin-bottom: 12px; }
  /* Flex children default to min-width:auto, which refuses to shrink below a text
     node's unwrapped width -- the actual cause of "some rows overflow, some don't"
     (short values never hit that floor, long ones like lat/lon do). min-width:0
     lets them shrink/wrap instead of pushing the fixed-width card wider. */
  .row { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; margin-bottom: 10px; }
  .row > * { min-width: 0; }
  select, button { background: var(--bg); color: var(--text); border: 1px solid var(--border);
                    border-radius: 4px; padding: 6px 10px; font-family: inherit; font-size: 12px; }
  select { flex: 1; }
  button { cursor: pointer; }
  button:disabled { opacity: 0.35; cursor: not-allowed; }
  .btn-connect { color: var(--accent); border-color: var(--accent); }
  .btn-connect.disconnect { color: var(--danger); border-color: var(--danger); }
  .state-dot { width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0; background: var(--dim); }
  .state-dot.armed { background: var(--green); box-shadow: 0 0 6px var(--green); }
  #arm-label { font-size: 11px; color: var(--dim); }
  .btn-arm { padding: 4px 12px; border-radius: 4px; border: 1px solid var(--border);
             font-size: 10px; cursor: pointer; text-transform: uppercase; letter-spacing: 1px;
             margin-left: auto; }
  .btn-arm.arm { background: var(--bg); color: var(--green); border-color: var(--green); }
  .btn-arm.disarm { background: var(--bg); color: var(--danger); border-color: var(--danger); }
  label.bench { font-size: 11px; color: var(--warn); cursor: pointer; user-select: none; }
  label.bench input { vertical-align: middle; margin-right: 4px; }
  .slider-row label { font-size: 10px; color: var(--dim); min-width: 55px;
                       text-transform: uppercase; letter-spacing: 1px; }
  .slider-row input[type=range] { flex: 1; accent-color: var(--accent); }
  .slider-row .val { font-size: 12px; color: var(--accent); min-width: 42px;
                      text-align: right; font-weight: bold; }
  #stop-btn { width: 100%; padding: 12px; font-size: 13px; font-weight: bold;
              letter-spacing: 2px; color: var(--danger); border-color: var(--danger); }
  #hint { font-size: 10px; color: var(--danger); min-height: 14px; margin-top: 4px; }
  footer { font-size: 10px; color: var(--dim); text-align: center; max-width: 360px; }
  footer .err { color: var(--danger); }
  .disabled-overlay { opacity: 0.4; pointer-events: none; }
  .telem-row { display: flex; flex-wrap: wrap; justify-content: space-between;
               align-items: center; row-gap: 2px; margin-bottom: 8px; }
  .telem-row label { font-size: 10px; color: var(--dim); text-transform: uppercase;
                      letter-spacing: 1px; flex-shrink: 0; }
  .telem-row .val { font-size: 13px; color: var(--accent); font-weight: bold;
                     min-width: 0; flex: 1 1 auto; text-align: right;
                     overflow-wrap: break-word; }
  .telem-row .val.warn { color: var(--warn); }
  .pill { min-width: 0; overflow-wrap: break-word; white-space: normal; }
  .pill.stale { color: var(--warn); border-color: var(--warn); }
</style>
</head>
<body>

<header>
  <h1>ESP-NOW Drive</h1>
  <span class="pill" id="conn-pill">DISCONNECTED</span>
</header>

<div class="card">
  <div class="card-title">Connect</div>
  <div class="row">
    <select id="port-select"></select>
    <button id="refresh-btn" title="rescan serial ports">&#8635;</button>
  </div>
  <button id="connect-btn" class="btn-connect" style="width:100%;">CONNECT</button>
</div>

<div class="card" id="drive-card">
  <div class="card-title">Drive</div>
  <div class="row">
    <div class="state-dot" id="state-dot"></div>
    <span id="arm-label">DISARMED (commanded)</span>
    <button class="btn-arm arm" id="arm-btn">ARM</button>
  </div>
  <div class="row">
    <label class="bench"><input type="checkbox" id="bench">bench (no GPS)</label>
  </div>
  <div id="hint"></div>
  <div class="row" style="justify-content:flex-end;margin-bottom:2px;">
    <label class="bench" style="color:var(--dim);"><input type="checkbox" id="motor-link" checked>Link L+R</label>
  </div>
  <div class="row slider-row" id="throttle-row">
    <label>Throttle</label>
    <input type="range" id="throttle" min="-100" max="100" value="0" step="5">
    <span class="val" id="throttle-val">0%</span>
  </div>
  <div class="row slider-row" id="motor-left-row" style="display:none;">
    <label>Left motor</label>
    <input type="range" id="motor-left" min="-100" max="100" value="0" step="5">
    <span class="val" id="motor-left-val">0%</span>
  </div>
  <div class="row slider-row" id="motor-right-row" style="display:none;">
    <label>Right motor</label>
    <input type="range" id="motor-right" min="-100" max="100" value="0" step="5">
    <span class="val" id="motor-right-val">0%</span>
  </div>
  <div class="row slider-row">
    <label>Rudder</label>
    <input type="range" id="rudder" min="-100" max="100" value="0" step="5">
    <span class="val" id="rudder-val">0%</span>
  </div>
  <div class="row">
    <label>Servo rail</label>
    <button id="servo-on-btn">PWR ON</button>
    <button id="servo-off-btn">PWR OFF</button>
    <span class="val" id="servo-state">UNKNOWN</span>
  </div>
  <div class="row slider-row">
    <label>Winch speed</label>
    <input type="range" id="winch-speed" min="0" max="100" value="50" step="5">
    <span class="val" id="winch-speed-val">50%</span>
  </div>
  <div class="row">
    <label>Winch jog</label>
    <button id="winch-up-btn">UP / REEL IN</button>
    <button id="winch-down-btn">DOWN / PAY OUT</button>
  </div>
  <div class="row" style="margin-top:6px;">
    <button id="calibrate-btn" style="flex:1;" title="Learn per-throttle L/R ESC trim from gyro yaw. ARM first; calm water, making way. The boat thrusts itself through a level sweep and saves the trim to NVS on a clean finish. Click again, or STOP / DISARM, to abort.">Calibrate ESC</button>
    <span id="calibrate-status" style="font-size:10px;color:var(--dim);flex:1;text-align:right;overflow-wrap:break-word;">idle</span>
  </div>
  <div class="row" style="margin-top:6px;">
    <button id="record-btn" style="flex:1;" title="Trigger one dataset capture (Feature 1): the boat saves a full-quality JPEG + sensor sidecar to SD. Fire-and-forget -- confirms the command was sent, not that the capture landed.">&#9679; Record to SD</button>
    <span id="record-status" style="font-size:10px;color:var(--dim);flex:1;text-align:right;">&mdash;</span>
  </div>
  <button id="stop-btn">STOP</button>
</div>

<div class="card" id="bench-card">
  <div class="card-title">Throttle mismatch test <span class="pill" id="bench-pill" style="margin-left:6px;">IDLE</span></div>
  <div class="row slider-row">
    <label>Throttle %</label>
    <input type="number" id="bench-throttle" min="1" max="60" step="1" value="20" style="width:56px;">
    <label style="min-width:auto;margin-left:10px;">Split &#177;%</label>
    <input type="number" id="bench-delta" min="1" max="30" step="1" value="4" style="width:56px;">
  </div>
  <div class="row">
    <button id="bench-left" title="left stronger / right weaker">LEFT TEST</button>
    <button id="bench-right" title="right stronger / left weaker">RIGHT TEST</button>
    <button id="bench-base" title="both equal -- any turn IS the mismatch">BASE TEST</button>
  </div>
  <div class="row" style="margin-top:6px;">
    <label style="min-width:auto;">Restart learner at c</label>
    <input type="number" id="bench-reset-c" min="0.10" max="0.35" step="0.01"
           value="0.12" style="width:64px;">
    <button id="bench-reset" title="set the learner's c ONCE, then run a normal adaptive BASE test">RESET c + BASE</button>
  </div>
  <div class="telem-row"><label>Learner c</label><span class="val" id="bench-learn-c">--</span></div>
  <div class="row" style="margin-top:6px;">
    <label style="min-width:auto;">Fast P assist</label>
    <button id="p-assist" title="temporary proportional yaw correction on the motors -- OFF is the control arm">P ASSIST: OFF</button>
    <span class="pill" id="p-confirm" style="margin-left:8px;">boat: --</span>
  </div>
  <div class="telem-row"><label>Run</label><span class="val" id="bench-progress">--</span></div>
  <div class="telem-row"><label>Saved as</label><span class="val" id="bench-file">--</span></div>
  <div id="bench-msg" style="font-size:10px;color:var(--warn);">boat records to its own SD card; DISARM stops a run. RESET is one-shot &mdash; ordinary BASE runs keep the learned c.</div>
</div>
</div>

<div class="card" id="telemetry-card">
  <div class="card-title">Telemetry <span class="pill" id="telem-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>Heading</label><span class="val" id="t-heading">--</span></div>
  <div class="telem-row"><label>Pitch / Roll</label><span class="val" id="t-attitude">--</span></div>
  <!-- Signed on purpose and NOT labelled left/right: which sign corresponds to
       which way the boat turns depends on how the IMU is mounted, and that has
       not been measured yet. This display is how we measure it. -->
  <div class="telem-row"><label>Yaw rate (gyro Z)</label><span class="val" id="t-yawrate">--</span></div>
  <div class="telem-row"><label>GPS Fix</label><span class="val" id="t-fix">--</span></div>
  <div class="telem-row"><label>GPS chip</label><span class="val" id="t-gps-chip">--</span></div>
  <div class="telem-row"><label>Lat / Lon</label><span class="val" id="t-latlon">--</span></div>
  <div class="telem-row"><label>Sats / HDOP</label><span class="val" id="t-sats">--</span></div>
  <div class="telem-row"><label>Speed / Course</label><span class="val" id="t-speed">--</span></div>
</div>

<div class="card" id="sensors-card">
  <div class="card-title">Sensors (boot check) <span class="pill" id="sensors-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>Camera</label><span class="val" id="s-camera">--</span></div>
  <div class="telem-row"><label>ToF A</label><span class="val" id="s-tof-a">--</span></div>
  <div class="telem-row"><label>ToF B</label><span class="val" id="s-tof-b">--</span></div>
  <div class="telem-row"><label>IMU</label><span class="val" id="s-imu">--</span></div>
  <div class="telem-row"><label>Compass</label><span class="val" id="s-mag">--</span></div>
</div>

<div class="card" id="motorstatus-card">
  <div class="card-title">Motor (confirmed by boat) <span class="pill" id="mstat-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>Arm state</label><span class="val" id="m-armstate">--</span></div>
  <div class="telem-row"><label>Throttle L / R</label><span class="val" id="m-throttle">--</span></div>
  <div class="telem-row"><label>Winch</label><span class="val" id="m-winch">--</span></div>
  <div class="telem-row"><label>Servo rail</label><span class="val" id="m-servo">--</span></div>
</div>

<div class="card" id="bridge-card">
  <div class="card-title">Bridge (S3) <span class="pill" id="bridge-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>ESP-NOW pkts</label><span class="val" id="b-pkts">--</span></div>
  <div class="telem-row"><label>Uplink RSSI</label><span class="val" id="b-rssi">--</span></div>
  <div class="telem-row"><label>LR peer rate</label><span class="val" id="b-lr-rate">--</span></div>
  <div class="telem-row"><label>Frames fwd'd</label><span class="val" id="b-frames">--</span></div>
  <div class="telem-row"><label>Hello sent</label><span class="val" id="b-hello">--</span></div>
  <div class="telem-row"><label>Reasm drops</label><span class="val" id="b-drops">--</span></div>
  <div style="font-size:10px;color:var(--dim);margin-top:4px;">
    Diagnostic: this is the S3 reporting what IT heard off-air, over USB only
    -- never over the radio. Pkts stuck at 0 means the S3 hears nothing from
    the boat (radio/channel/range); pkts climbing but no telemetry above means
    a decode problem on this end.
  </div>
</div>

<footer>
  <div>tx seq <span id="seq">0</span> <span id="err" class="err"></span></div>
  <div style="margin-top:6px;">ARM/DISARM above is the last command sent, not confirmed by
  the boat (no MotorStatus decode here) -- but GPS/IMU telemetry updating means the ESP-NOW
  link itself is genuinely alive.</div>
</footer>

<script>
const $ = id => document.getElementById(id);
let connected = false;
let armedCmd = false;
let servoRailOn = false;
// Mirrors BenchStatus in boat.proto (see BENCH_KIND_NAME / BENCH_STATE_NAME).
const BENCH_KIND_NAME = { 0: 'BASE', 1: 'LEFT', 2: 'RIGHT' };
const BENCH_STATE = { 0: 'IDLE', 1: 'STILL', 2: 'DRIVING', 3: 'COASTING',
                      4: 'SAVED', 5: 'FAILED' };
let calibrating = false;
let winchCommandSeq = 0;
let winchRenewTimer = null;
const WINCH_RENEW_MS = 100;

async function api(path, method, body) {
  const opts = { method };
  if (body !== undefined) {
    opts.headers = { 'Content-Type': 'application/json' };
    opts.body = JSON.stringify(body);
  }
  const r = await fetch(path, opts);
  return r.json();
}

async function refreshPorts() {
  const { ports } = await api('/api/ports', 'GET');
  const sel = $('port-select');
  const prev = sel.value;
  sel.innerHTML = '';
  for (const p of ports) {
    const opt = document.createElement('option');
    opt.value = p.device;
    opt.textContent = p.description ? `${p.device} (${p.description})` : p.device;
    sel.appendChild(opt);
  }
  if ([...sel.options].some(o => o.value === prev)) sel.value = prev;
  if (!ports.length) {
    const opt = document.createElement('option');
    opt.textContent = 'no serial ports found';
    sel.appendChild(opt);
  }
}

function setConnectedUI(isConn, port) {
  connected = isConn;
  if (!isConn) {
    clearWinchRenewal();
    winchDir = 0;
  }
  $('conn-pill').textContent = isConn ? `CONNECTED ${port}` : 'DISCONNECTED';
  $('conn-pill').classList.toggle('up', isConn);
  $('connect-btn').textContent = isConn ? 'DISCONNECT' : 'CONNECT';
  $('connect-btn').classList.toggle('disconnect', isConn);
  $('port-select').disabled = isConn;
  $('drive-card').classList.toggle('disabled-overlay', !isConn);
  updateWinchControls();
}

$('refresh-btn').addEventListener('click', refreshPorts);

$('connect-btn').addEventListener('click', async () => {
  if (connected) {
    await api('/api/disconnect', 'POST');
    setConnectedUI(false, null);
    return;
  }
  const port = $('port-select').value;
  if (!port) return;
  const res = await api('/api/connect', 'POST', { port });
  if (res.ok) {
    winchDir = 0;
    setConnectedUI(true, port);
    $('throttle').value = 0; $('throttle-val').textContent = '0%';
    $('rudder').value = 0; $('rudder-val').textContent = '0%';
  } else {
    $('hint').textContent = `connect failed: ${res.error}`;
  }
});

$('throttle').addEventListener('input', (e) => {
  const v = parseInt(e.target.value);
  $('throttle-val').textContent = v + '%';
  api('/api/state', 'POST', { throttle: v / 100 });
});

// Link toggle: checked = one Throttle slider drives both motors; unchecked =
// splits into independent Left/Right sliders (single-ESC test). On each switch we
// seed the newly shown slider(s) from the current value and push it, so the
// motors never jump when you flip the mode.
function setMotorLink(linked) {
  $('throttle-row').style.display    = linked ? '' : 'none';
  $('motor-left-row').style.display  = linked ? 'none' : '';
  $('motor-right-row').style.display = linked ? 'none' : '';
  if (linked) {
    const v = parseInt($('motor-left').value);   // re-link at the left motor's value
    $('throttle').value = v; $('throttle-val').textContent = v + '%';
    api('/api/state', 'POST', { throttle: v / 100 });
  } else {
    const v = parseInt($('throttle').value);      // split: both start at the throttle value
    $('motor-left').value = v;  $('motor-left-val').textContent = v + '%';
    $('motor-right').value = v; $('motor-right-val').textContent = v + '%';
    api('/api/state', 'POST', { left: v / 100, right: v / 100 });
  }
}
$('motor-link').addEventListener('change', (e) => setMotorLink(e.target.checked));

$('motor-left').addEventListener('input', (e) => {
  const v = parseInt(e.target.value);
  $('motor-left-val').textContent = v + '%';
  api('/api/state', 'POST', { left: v / 100 });
});
$('motor-right').addEventListener('input', (e) => {
  const v = parseInt(e.target.value);
  $('motor-right-val').textContent = v + '%';
  api('/api/state', 'POST', { right: v / 100 });
});

$('rudder').addEventListener('input', (e) => {
  const v = parseInt(e.target.value);
  $('rudder-val').textContent = v + '%';
  // Slider is intuitive and canonical: drag RIGHT (+100) -> +1.0 on the wire
  // -> 1195us -> the boat turns RIGHT. -1 = left, 0 = centre, +1 = right,
  // sighting from behind the hull toward the bow.
  //
  // This mapping never changed, but it USED to be wrong on the water: the
  // driver called 1805us "right" when the boat says it is left, so +1.0 sent
  // the rudder hard over to the left. CONFIG_STEER_REVERSE fixed that at the
  // root. dashboard.html dropped its compensating negation in the same change,
  // so both UIs now send the same value AND move the boat the same way -- they
  // no longer disagree, and neither should be "synced" by negating one.
  //
  // This tool still starts centred at 0; dashboard.html still starts at -100,
  // which is the 1805us full-left power-up position.
  api('/api/state', 'POST', { rudder: v / 100 });
});

function winchMagnitude() {
  return Math.abs(parseInt($('winch-speed').value) || 0) / 100;
}

function sendWinch(speed) {
  return api('/api/winch', 'POST', { speed, seq: ++winchCommandSeq });
}

let winchDir = 0;
function clearWinchRenewal() {
  if (winchRenewTimer !== null) {
    clearInterval(winchRenewTimer);
    winchRenewTimer = null;
  }
}

function updateWinchControls() {
  const enabled = connected && servoRailOn;
  ['winch-speed', 'winch-up-btn', 'winch-down-btn'].forEach(id => {
    $(id).disabled = !enabled;
  });
  if (!enabled) {
    clearWinchRenewal();
    winchDir = 0;
  }
}

function stopWinch() {
  clearWinchRenewal();
  if (winchDir === 0) return;
  winchDir = 0;
  if (connected) sendWinch(0);
}

function startWinchRenewal() {
  clearWinchRenewal();
  winchRenewTimer = setInterval(() => {
    if (!connected || !servoRailOn || winchDir === 0) {
      stopWinch();
      return;
    }
    sendWinch(winchDir * winchMagnitude());
  }, WINCH_RENEW_MS);
}

$('winch-speed').addEventListener('input', (e) => {
  const v = Math.abs(parseInt(e.target.value) || 0);
  $('winch-speed-val').textContent = v + '%';
  if (winchDir !== 0) sendWinch(winchDir * (v / 100));
});

[['winch-up-btn', -1], ['winch-down-btn', +1]].forEach(([id, dir]) => {
  const el = $(id);
  const start = (e) => {
    e.preventDefault();
    if (!connected || !servoRailOn) return;
    winchDir = dir;
    sendWinch(dir * winchMagnitude());
    startWinchRenewal();
  };
  const stop = () => { if (winchDir === dir) stopWinch(); };
  el.addEventListener('pointerdown', start);
  el.addEventListener('pointerup', stop);
  el.addEventListener('pointercancel', stop);
  el.addEventListener('pointerleave', stop);
  el.addEventListener('contextmenu', (e) => e.preventDefault());
});
window.addEventListener('blur', stopWinch);
window.addEventListener('pagehide', () => {
  if (!connected || winchDir === 0) return;
  clearWinchRenewal();
  winchDir = 0;
  const body = new Blob(
    [JSON.stringify({ speed: 0, seq: ++winchCommandSeq })],
    { type: 'application/json' });
  navigator.sendBeacon('/api/winch', body);
});

function setServoPower(on) {
  stopWinch();
  return api('/api/servo-power', 'POST', { on, seq: ++winchCommandSeq });
}

$('servo-on-btn').addEventListener('click', () => setServoPower(true));
$('servo-off-btn').addEventListener('click', () => setServoPower(false));

$('stop-btn').addEventListener('click', async () => {
  clearWinchRenewal();
  winchDir = 0;
  $('throttle').value = 0; $('throttle-val').textContent = '0%';
  $('rudder').value = 0; $('rudder-val').textContent = '0%';
  await api('/api/stop', 'POST', { seq: ++winchCommandSeq });
});

$('arm-btn').addEventListener('click', async () => {
  if (!connected) return;
  const force = $('bench').checked;
  const nextArm = !armedCmd;
  $('hint').textContent = (nextArm && !force)
    ? 'sent without "bench (no GPS)" -- boat will refuse this if it has no GPS lock'
    : '';
  await api('/api/arm', 'POST', { arm: nextArm, force });
});

$('calibrate-btn').addEventListener('click', async () => {
  if (!connected) return;
  const start = !calibrating;
  if (start && !armedCmd) {
    $('hint').textContent = 'ARM first -- calibration drives the thrusters';
    return;
  }
  // While started, the tool streams a CalibrateCommand keepalive instead of
  // motor/steer/winch, so the boat never sees a manual command (which aborts).
  await api('/api/calibrate', 'POST', { start, seq: ++winchCommandSeq });
});

// Dataset capture (Feature 1) -- fire-and-forget, no ack. The boat saves the
// JPEG + sensor sidecar to SD on its own; this only confirms the command left.
// Bench throttle-mismatch test. One press records a WHOLE run -- motors off,
// then the test command, then coasting -- and the file keeps all of it. Once
// started nothing cuts it short; only STOP or DISARM ends it early.
async function runBench(kind, resetC) {
  if (!connected) { $('bench-msg').textContent = 'not connected'; return; }
  const base = parseInt($('bench-throttle').value) / 100;
  const delta = parseInt($('bench-delta').value) / 100;
  $('bench-msg').textContent = '';
  const r = await api('/api/bench', 'POST',
                      { kind, base, delta, reset_c: resetC || 0,
                        seq: ++winchCommandSeq });
  if (r && !r.ok) $('bench-msg').textContent = r.error || 'refused';
}
// The plain buttons send reset_c 0 -- they MUST keep whatever c the learner
// has reached, or convergence across runs could never be seen.
$('bench-left').addEventListener('click', () => runBench('left', 0));
$('bench-right').addEventListener('click', () => runBench('right', 0));
$('bench-base').addEventListener('click', () => runBench('both', 0));
// Runtime switch, deliberately not a rebuild: both arms of the A/B must run
// the same binary. OFF is the control arm and the default.
var pAssistOn = false;
$('p-assist').addEventListener('click', async () => {
  const want = !pAssistOn;
  const r = await api('/api/assist', 'POST', { p_on: want });
  if (r && r.ok) {
    pAssistOn = want;
    $('p-assist').textContent = 'P ASSIST: ' + (want ? 'ON' : 'OFF');
    $('p-assist').classList.toggle('up', want);
  } else if (r) {
    $('bench-msg').textContent = r.error || 'refused';
  }
});
$('bench-reset').addEventListener('click', () => {
  var c = parseFloat($('bench-reset-c').value);
  if (!isFinite(c) || c <= 0) { $('bench-msg').textContent = 'enter a starting c'; return; }
  runBench('both', c);
});

$('record-btn').addEventListener('click', async () => {
  if (!connected) { $('record-status').textContent = 'no link'; return; }
  $('record-status').textContent = 'sending...';
  const r = await api('/api/record', 'POST', {});
  $('record-status').textContent = (r && r.ok) ? 'sent ✓'
                                               : ('failed: ' + ((r && r.error) || '?'));
});

function applyStatus(s) {
    if (Number.isInteger(s.winch_command_seq)) {
      winchCommandSeq = Math.max(winchCommandSeq, s.winch_command_seq);
    }
    if (s.connected !== connected) setConnectedUI(s.connected, s.port);
    armedCmd = s.armed_cmd;
    $('state-dot').classList.toggle('armed', s.armed_cmd);
    $('arm-label').textContent = (s.armed_cmd ? 'ARMED' : 'DISARMED') + ' (commanded)';
    $('arm-btn').textContent = s.armed_cmd ? 'DISARM' : 'ARM';
    $('arm-btn').classList.toggle('arm', !s.armed_cmd);
    $('arm-btn').classList.toggle('disarm', s.armed_cmd);
    calibrating = !!s.calibrating;
    $('calibrate-btn').textContent = calibrating ? 'Stop Calibration' : 'Calibrate ESC';
    $('calibrate-btn').classList.toggle('disarm', calibrating);
    // The boat's CalibrateStatus rides the telemetry link, so show its real
    // state. etc_state_t: 1 measuring, 2 ramp, 3 settling, 4 DONE, 5 aborted.
    // (If a terminal frame is dropped by ESP-NOW we keep showing the last
    // progress line; the boat has stopped, so STOP resets it either way.)
    var cal = s.calibrate;
    var cel = $('calibrate-status');
    if (cal && cal.have && !cal.stale) {
      var st = cal.state;
      if (st === 4) {
        cel.textContent = '✓ DONE — ' + cal.points_done + ' points saved';
      } else if (st === 5) {
        cel.textContent = '✗ ABORTED — nothing saved';
      } else {
        var nm = {1: 'measuring noise', 2: 'ramping', 3: 'settling'};
        var thr = Math.round((cal.level_throttle || 0) * 100);
        cel.textContent = (nm[st] || ('state ' + st)) + ' · L' + cal.level_index +
          ' @' + thr + '% · trim ' + (cal.trim_diff || 0).toFixed(3) +
          ' · pts ' + cal.points_done + (cal.making_way ? '' : ' · (no way)');
      }
    } else if (calibrating) {
      cel.textContent = 'starting… watch the boat';
    } else {
      cel.textContent = 'idle';
    }
    servoRailOn = s.servo_rail_cut === false;
    updateWinchControls();
    $('seq').textContent = s.seq;
    $('err').textContent = s.last_error ? `tx error: ${s.last_error}` : '';

    const t = s.telemetry;
    const pill = $('telem-pill');
    if (!t.have) {
      pill.textContent = 'NO DATA YET';
      pill.classList.remove('up', 'stale');
    } else if (t.stale) {
      pill.textContent = `STALE ${t.age_s.toFixed(0)}s`;
      pill.classList.remove('up'); pill.classList.add('stale');
    } else {
      pill.textContent = 'LIVE';
      pill.classList.remove('stale'); pill.classList.add('up');
    }
    $('t-heading').textContent = t.have ? `${t.heading.toFixed(1)}°` : '--';
    $('t-attitude').textContent = t.have ? `${t.pitch.toFixed(1)}° / ${t.roll.toFixed(1)}°` : '--';
    // Always signed, so +0.20 and -0.20 cannot be misread at a glance.
    // toFixed(2) supplies the minus; only the plus has to be added. Raw value
    // -- no negation, no magnitude, no smoothing: this is the measurement.
    //
    // Stricter than the rows above it, which show the last value they saw for
    // as long as the page is open. That is tolerable for an ANGLE (a stale
    // heading is merely old) and not for a RATE: a frozen "+12.34 °/s" reads
    // as "the boat is turning right now", which is the opposite of true once
    // the link has dropped -- and actively misleading while the whole point of
    // this readout is to establish which way the boat turns. Blank it instead.
    // Same reasoning as dashboard.html blanking on ws.onclose.
    //
    // Number.isFinite also rejects a NaN/absent value outright; it is false for
    // non-numbers, so it covers the missing-key case on its own.
    $('t-yawrate').textContent =
      (t.have && !t.stale && Number.isFinite(t.yaw_rate))
        ? `${t.yaw_rate >= 0 ? '+' : ''}${t.yaw_rate.toFixed(2)} °/s` : '--';
    const fixEl = $('t-fix');
    fixEl.textContent = t.have ? (t.gps_valid ? 'VALID' : 'NO FIX') : '--';
    fixEl.classList.toggle('warn', t.have && !t.gps_valid);
    // 5 decimal places (~1.1m) is plenty for a boat and noticeably shorter
    // than the 6 (~11cm) protobuf carries -- less to wrap on a narrow card.
    $('t-latlon').textContent = (t.have && t.gps_valid)
      ? `${t.lat.toFixed(5)}, ${t.lon.toFixed(5)}` : '--';
    $('t-sats').textContent = t.have ? `${t.satellites} / ${t.hdop.toFixed(1)}` : '--';
    $('t-speed').textContent = (t.have && t.gps_valid)
      ? `${t.speed_mps.toFixed(1)} m/s / ${t.course_deg.toFixed(0)}°` : '--';

    // Sensor-health / boot check (SystemStatus). The boat sends this ~1Hz over
    // the field link; without it, 'NO FIX' hid whether the GPS chip was even
    // alive. gps_ok = chip talking to the UART, independent of a satellite lock.
    const ss = s.system_status;
    const setOk = (id, v) => {
      const el = $(id);
      el.textContent = v ? 'OK' : 'DEAD';
      el.classList.toggle('warn', !v);
    };
    const spill = $('sensors-pill');
    const chip = $('t-gps-chip');
    if (!ss || !ss.have) {
      spill.textContent = 'NO DATA YET';
      spill.classList.remove('up', 'stale');
      chip.textContent = '--';
      chip.classList.remove('warn');
    } else {
      spill.textContent = ss.stale ? `STALE ${ss.age_s.toFixed(0)}s` : 'LIVE';
      spill.classList.toggle('stale', ss.stale);
      spill.classList.toggle('up', !ss.stale);
      setOk('s-camera', ss.camera_ok);
      setOk('s-tof-a', ss.tof_a_ok);
      setOk('s-tof-b', ss.tof_b_ok);
      setOk('s-imu', ss.imu_ok);
      setOk('s-mag', ss.mag_ok);
      const baud = ss.gps_detected_baud
        ? ` @${ss.gps_detected_baud}${ss.gps_baud_confirmed ? '' : '?'}` : '';
      chip.textContent = (ss.gps_ok ? 'talking' : 'DEAD — check wiring') + baud;
      chip.classList.toggle('warn', !ss.gps_ok);
    }

    // MotorStatus -- the boat's ACTUAL arm/throttle/servo, vs what we commanded.
    // state (esc_state_t): 0 disarmed, 1 arming, 2 armed. Compare "ARMED
    // (confirmed)" here against the arm button's "(commanded)" to see if an arm
    // actually took (the thing you couldn't tell before).
    const mst = s.motor_status;
    const mpill = $('mstat-pill');
    if (!mst || !mst.have) {
      mpill.textContent = 'NO DATA YET';
      mpill.classList.remove('up', 'stale');
      $('m-armstate').textContent = '--';
      $('m-throttle').textContent = '--';
      $('m-winch').textContent = '--';
      $('m-servo').textContent = '--';
      $('m-servo').classList.remove('warn');
    } else {
      mpill.textContent = mst.stale ? `STALE ${mst.age_s.toFixed(0)}s` : 'LIVE';
      mpill.classList.toggle('stale', mst.stale);
      mpill.classList.toggle('up', !mst.stale);
      const armTxt = mst.state === 2 ? 'ARMED' : (mst.state === 1 ? 'ARMING' : 'DISARMED');
      $('m-armstate').textContent = armTxt + ' (confirmed)';
      $('m-throttle').textContent =
        `${Math.round(mst.left_throttle * 100)}% / ${Math.round(mst.right_throttle * 100)}%`;
      $('m-winch').textContent = `${Math.round(mst.winch_speed * 100)}%`;
      const servoEl = $('m-servo');
      servoEl.textContent = mst.servo_power ? 'ON' : 'OFF';
      servoEl.classList.toggle('warn', !mst.servo_power);
    }

    // The BOAT runs the test and writes the file; this just shows its progress
    // and which file number it saved as. Read the runs with tools/bench_analyze.py.
    var bn = s.bench;
    if (bn) {
      var benchPill = $('bench-pill');
      var running = bn.have && [1, 2, 3].indexOf(bn.state) !== -1;
      benchPill.textContent = !bn.have ? 'IDLE'
        : (BENCH_STATE[bn.state] || ('state ' + bn.state));
      benchPill.classList.toggle('up', !!running);
      $('bench-progress').textContent = bn.have
        ? (BENCH_KIND_NAME[bn.kind] || '?') + '  ' + bn.elapsed_s.toFixed(1)
          + 's  ' + bn.samples + ' samples'
        : '--';
      if (bn.have && bn.file_index) {
        var pct = Math.round(bn.base * 100);
        $('bench-file').textContent = 'T' + (pct < 10 ? '0' : '') + pct + '_'
          + (BENCH_KIND_NAME[bn.kind] || '?').charAt(0) + '_'
          + (bn.file_index < 10 ? '0' : '') + bn.file_index + '.CSV';
      } else if (!bn.have) {
        $('bench-file').textContent = '--';
      }
      // What the BOAT reports, next to what we asked for. A mismatch voids
      // the A/B, so it is shown rather than assumed.
      if (bn.have) {
        var bp = !!bn.p_on;
        $('p-confirm').textContent = 'boat: ' + (bp ? 'ON' : 'OFF');
        $('p-confirm').classList.toggle('up', bp);
        $('p-confirm').classList.toggle('warn', bp !== pAssistOn);
      }
      $('bench-learn-c').textContent =
        (bn.have && typeof bn.learn_c === 'number' && bn.learn_c > 0)
          ? bn.learn_c.toFixed(3) : '--';
    }

    const b = s.bridge_status;
    const bpill = $('bridge-pill');
    if (!b.have) {
      bpill.textContent = 'NO DATA YET';
      bpill.classList.remove('up', 'stale');
    } else if (b.stale) {
      bpill.textContent = `STALE ${b.age_s.toFixed(0)}s`;
      bpill.classList.remove('up'); bpill.classList.add('stale');
    } else {
      bpill.textContent = 'LIVE';
      bpill.classList.remove('stale'); bpill.classList.add('up');
    }
    $('b-pkts').textContent = b.have ? `${b.espnow_pkts} (${b.espnow_bytes} B)` : '--';
    $('b-rssi').textContent = !b.have ? '--'
      : (b.espnow_pkts > 0 && Number.isFinite(b.uplink_rssi_dbm) && b.uplink_rssi_dbm < 0)
      ? `${b.uplink_rssi_dbm} dBm` : 'UNKNOWN';
    $('b-lr-rate').textContent = !b.have ? '--'
      : b.lr_rate_config_ok === null ? 'UNKNOWN'
      : b.lr_rate_config_ok === 1 ? 'LR/250K SET'
      : b.lr_rate_config_ok === 0 ? 'LR/250K FAILED'
      : b.lr_rate_config_ok === -1 ? 'LR DISABLED' : 'UNKNOWN';
    $('b-frames').textContent = b.have ? `${b.frames_out}` : '--';
    $('b-hello').textContent = b.have ? `${b.hello_sent}` : '--';
    $('b-drops').textContent = b.have ? `${b.reasm_drops}` : '--';
}

// Status arrives by push, not poll -- /ws streams the same dict /api/status
// serves, at WS_PUSH_HZ, so the telemetry/bridge cards update as fast as the
// boat itself produces new data instead of on a fixed client-side timer.
let _statusWS = null;
let _wsRetryMs = 500;
function connectStatusWS() {
  _statusWS = new WebSocket(`ws://${location.host}/ws`);
  _statusWS.onopen = () => { _wsRetryMs = 500; };
  _statusWS.onmessage = (evt) => {
    try { applyStatus(JSON.parse(evt.data)); } catch (e) { /* malformed frame, skip */ }
  };
  _statusWS.onclose = () => {
    setTimeout(connectStatusWS, _wsRetryMs);
    _wsRetryMs = Math.min(_wsRetryMs * 2, 5000);
  };
  _statusWS.onerror = () => { _statusWS.close(); };
}

refreshPorts();
setConnectedUI(false, null);
connectStatusWS();
</script>
</body>
</html>
"""


def _ws_text_frame(text: str) -> bytes:
    """Encode one unmasked WebSocket text frame (server -> client frames are
    never masked per RFC 6455 -- only client -> server ones are)."""
    payload = text.encode('utf-8')
    length = len(payload)
    if length < 126:
        header = bytes([0x81, length])
    elif length <= 0xFFFF:
        header = bytes([0x81, 126]) + length.to_bytes(2, 'big')
    else:
        header = bytes([0x81, 127]) + length.to_bytes(8, 'big')
    return header + payload


class Handler(BaseHTTPRequestHandler):
    link: BoatLink = None   # set in main() before serve_forever()

    def log_message(self, fmt, *args):
        pass   # quiet -- /ws pushes at WS_PUSH_HZ, would spam one line per frame

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> dict:
        length = int(self.headers.get('Content-Length', 0) or 0)
        if length == 0:
            return {}
        try:
            body = json.loads(self.rfile.read(length))
            return body if isinstance(body, dict) else {}
        except Exception:                                # noqa: BLE001
            return {}

    def do_GET(self):
        if self.path == '/':
            # no-store: this page is regenerated fresh every time this script
            # starts (PAGE is a Python string, not a file), so a browser-cached
            # copy silently survives a restart and keeps running stale JS
            # against a newer backend -- same class of bug main/http_server.c's
            # dashboard_handler() already guards against, for the same reason.
            body = PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/api/status':
            self._json(self.link.status())
        elif self.path == '/api/ports':
            import serial.tools.list_ports as list_ports
            ports = [{'device': p.device, 'description': p.description}
                     for p in list_ports.comports()]
            self._json({'ports': ports})
        elif self.path == '/ws':
            self._handle_ws()
        else:
            self.send_response(404)
            self.end_headers()

    def _handle_ws(self):
        """Minimal one-way (server -> browser) WebSocket: push self.link.status()
        at WS_PUSH_HZ instead of waiting for the browser to poll /api/status.
        Commands stay on the existing REST endpoints -- they were already
        immediate (sent on the slider's 'input' event), never the bottleneck --
        so this connection never needs to decode client -> server frames.
        Detecting disconnect via a failed write is enough for a local,
        single-user tool; a full RFC 6455 read side (masking, ping/pong, close
        frames) would be real protocol work for no behavioural gain here.
        ThreadingHTTPServer gives this call its own thread for the connection's
        whole lifetime -- same pattern BoatLink._stream_loop already uses. """
        key = self.headers.get('Sec-WebSocket-Key')
        if not key or self.headers.get('Upgrade', '').lower() != 'websocket':
            self.send_response(400)
            self.end_headers()
            return
        accept = base64.b64encode(
            hashlib.sha1((key + WS_MAGIC).encode()).digest()).decode()
        self.send_response(101, 'Switching Protocols')
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', accept)
        self.end_headers()

        period = 1.0 / WS_PUSH_HZ
        try:
            while True:
                self.wfile.write(_ws_text_frame(json.dumps(self.link.status())))
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass   # browser closed/refreshed/navigated away -- not an error

    def do_POST(self):
        body = self._read_body()
        if self.path == '/api/connect':
            ok, err = self.link.connect(body.get('port', ''))
            self._json({'ok': ok, 'error': err})
        elif self.path == '/api/disconnect':
            self.link.disconnect()
            self._json({'ok': True})
        elif self.path == '/api/state':
            self.link.set_state(throttle=body.get('throttle'), rudder=body.get('rudder'),
                                left=body.get('left'), right=body.get('right'))
            self._json({'ok': True})
        elif self.path == '/api/winch':
            command_seq = body.get('seq')
            if (not isinstance(command_seq, int) or isinstance(command_seq, bool) or
                    command_seq < 0):
                self._json({'ok': False, 'error': 'seq must be a nonnegative integer'}, 400)
                return
            speed = body.get('speed')
            if (not isinstance(speed, (int, float)) or isinstance(speed, bool) or
                    not math.isfinite(speed)):
                self._json({'ok': False, 'error': 'speed must be a finite number'}, 400)
                return
            ok, err = self.link.set_winch(speed, command_seq)
            code = 200 if ok else (503 if err in (
                'serial link is disconnected', 'serial write failed') else 409)
            self._json({'ok': ok, 'error': err}, code)
        elif self.path == '/api/servo-power':
            command_seq = body.get('seq')
            if (not isinstance(command_seq, int) or isinstance(command_seq, bool) or
                    command_seq < 0):
                self._json({'ok': False, 'error': 'seq must be a nonnegative integer'}, 400)
                return
            on = body.get('on')
            if not isinstance(on, bool):
                self._json({'ok': False, 'error': 'on must be a boolean'}, 400)
                return
            ok, err = self.link.set_servo_power(on, command_seq)
            code = 200 if ok else (503 if err in (
                'serial link is disconnected', 'serial write failed') else 409)
            self._json({'ok': ok, 'error': err}, code)
        elif self.path == '/api/stop':
            command_seq = body.get('seq')
            if (not isinstance(command_seq, int) or isinstance(command_seq, bool) or
                    command_seq < 0):
                self._json({'ok': False, 'error': 'seq must be a nonnegative integer'}, 400)
                return
            ok, err = self.link.stop(command_seq)
            code = 200 if ok else (503 if err in (
                'serial link is disconnected', 'serial write failed') else 409)
            self._json({'ok': ok, 'error': err}, code)
        elif self.path == '/api/arm':
            self.link.arm(bool(body.get('arm')), bool(body.get('force')))
            self._json({'ok': True})
        elif self.path == '/api/calibrate':
            command_seq = body.get('seq')
            if (not isinstance(command_seq, int) or isinstance(command_seq, bool) or
                    command_seq < 0):
                self._json({'ok': False, 'error': 'seq must be a nonnegative integer'}, 400)
                return
            start = body.get('start')
            if not isinstance(start, bool):
                self._json({'ok': False, 'error': 'start must be a boolean'}, 400)
                return
            ok, err = self.link.set_calibrate(start, command_seq)
            code = 200 if ok else (503 if err in (
                'serial link is disconnected', 'serial write failed') else 409)
            self._json({'ok': ok, 'error': err}, code)
        elif self.path == '/api/bench':
            command_seq = body.get('seq')
            if (not isinstance(command_seq, int) or isinstance(command_seq, bool) or
                    command_seq < 0):
                self._json({'ok': False, 'error': 'seq must be a nonnegative integer'}, 400)
                return
            kind = body.get('kind')
            if not isinstance(kind, str):
                self._json({'ok': False, 'error': 'kind must be a string'}, 400)
                return
            values = {}
            for label in ('base', 'delta'):
                val = body.get(label)
                if (not isinstance(val, (int, float)) or isinstance(val, bool) or
                        not math.isfinite(val)):
                    self._json({'ok': False,
                                'error': '%s must be a finite number' % label}, 400)
                    return
                values[label] = val
            reset_c = body.get('reset_c', 0.0)
            if (not isinstance(reset_c, (int, float)) or isinstance(reset_c, bool)
                    or not math.isfinite(reset_c)):
                self._json({'ok': False,
                            'error': 'reset_c must be a finite number'}, 400)
                return
            ok, err = self.link.send_bench(kind, values['base'], values['delta'],
                                           command_seq, reset_c)
            code = 200 if ok else (503 if err in (
                'serial link is disconnected', 'serial write failed') else 409)
            self._json({'ok': ok, 'error': err}, code)
        elif self.path == '/api/assist':
            p_on = body.get('p_on')
            if not isinstance(p_on, bool):
                self._json({'ok': False, 'error': 'p_on must be true or false'}, 400)
                return
            ok, err = self.link.send_assist(p_on)
            self._json({'ok': ok, 'error': err}, 200 if ok else 503)
        elif self.path == '/api/record':
            ok, err = self.link.trigger_record()
            self._json({'ok': ok, 'error': err}, 200 if ok else 503)
        else:
            self.send_response(404)
            self.end_headers()


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--http-port', type=int, default=8765,
                    help='local port for the control page (default: 8765)')
    ap.add_argument('--hz', type=float, default=SEND_HZ,
                    help=f'command send rate to the boat (default: {SEND_HZ}). '
                         'Diagnostic knob: if the link stalls under sustained '
                         'concurrent load (commands out + telemetry in), try a '
                         'lower rate (e.g. --hz 5) to see if that changes when '
                         'or whether it happens.')
    args = ap.parse_args()

    try:
        import serial  # noqa: F401 -- fail fast if pyserial is missing
    except ImportError:
        print('ERROR: pyserial not installed. Try: .venv/bin/pip install pyserial',
              file=sys.stderr)
        return 1

    boat_pb2 = load_boat_pb2()
    link = BoatLink(boat_pb2, send_hz=args.hz)
    Handler.link = link
    print(f'Command send rate: {args.hz} Hz')

    server = ThreadingHTTPServer((HTTP_HOST, args.http_port), Handler)
    url = f'http://{HTTP_HOST}:{args.http_port}/'
    print(f'Serving on {url} (127.0.0.1 only -- not reachable from the network)')
    try:
        webbrowser.open(url)
    except Exception:                                    # noqa: BLE001
        pass
    print('Ctrl+C to stop.')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.shutdown()
        server.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
