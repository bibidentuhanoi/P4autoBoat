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

Commands stream continuously at ~15 Hz for as long as a serial port is
connected, regardless of whether anything changed -- driven by a background
thread here, not by the browser (so a backgrounded/throttled browser tab
can't stall it). This is the point, not an inefficiency: a lost packet just
holds the previous value for one tick (RC-style loss tolerance), and it is
what feeds the firmware's control-link-loss failsafe (motor_control.c) --
that failsafe stops the boat if this stream goes silent for 400ms, so closing
this tool (or losing the link) is the SAFE direction.

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
import os
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
# Field-mode telemetry, IMU+GPS only -- a hand-packed struct (main/transports/
# espnow_protocol.h: espnow_telemetry_t), NOT a boat.proto message. Replaces
# MSG_SENSOR on the ESP-NOW link: fits one ESP-NOW packet (no fragmentation),
# where a full SensorSnapshot with ToF needed ~55 fragments and could stall
# the boat's own publish loop. Layout must match espnow_telemetry_t exactly:
# pitch,roll,heading (f) + gps_valid (B) + lat,lon (d) + speed,course (f) +
# satellites (B) + hdop (f), packed, little-endian.
MSG_FIELD_TELEMETRY = 0x08
FIELD_TELEMETRY_FMT = '<fffBddffBf'
ESPNOW_HDR_SIZE = 4
ESP_NOW_MAX_DATA_LEN = 250     # ESP-NOW v1 single-packet limit
SEND_HZ = 15
HTTP_HOST = '127.0.0.1'        # LAN-reachable would let anyone drive the boat
TELEMETRY_STALE_S = 2.0        # snapshot task runs ~20Hz normally -- 2s is a generous margin

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
        self.throttle = 0.0
        self.rudder = 0.0
        self.armed_cmd = False
        self.force = False
        self.seq = 0
        self.last_error = None
        self.telemetry = self._blank_telemetry()
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
            'satellites': 0, 'hdop': 0.0,
        }

    @staticmethod
    def _blank_bridge_status() -> dict:
        return {
            'have': False, 'last_rx_monotonic': None,
            'uptime_s': 0, 'espnow_pkts': 0, 'espnow_bytes': 0,
            'frames_out': 0, 'hello_sent': 0, 'reasm_drops': 0,
        }

    # ---- internal, must hold self._lock ----

    def _write_locked(self, payload: bytes):
        frame = build_frame(MSG_MOTOR_CMD, payload, self.seq)
        self.seq = (self.seq + 1) & 0xFF
        try:
            self.ser.write(frame)
            self.last_error = None
        except Exception as exc:                        # noqa: BLE001
            self.last_error = str(exc)

    def _send_motor_locked(self, throttle: float):
        msg = self.pb2.BoatMessage()
        msg.motor.left = throttle
        msg.motor.right = throttle
        self._write_locked(msg.SerializeToString())

    def _send_steer_locked(self, rudder: float):
        msg = self.pb2.BoatMessage()
        msg.steer.left = rudder
        msg.steer.right = rudder
        self._write_locked(msg.SerializeToString())

    def _send_arm_locked(self, arm: bool, force: bool):
        msg = self.pb2.BoatMessage()
        msg.arm_cmd.arm = arm
        msg.arm_cmd.force = force
        self._write_locked(msg.SerializeToString())

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
                self._close_locked()
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
            self.rudder = 0.0
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
            self.throttle = 0.0
            self.rudder = 0.0
            self._send_motor_locked(0.0)
            self._send_steer_locked(0.0)
            if self.armed_cmd:
                self._send_arm_locked(False, self.force)
                self.armed_cmd = False
            self._close_locked()

    def set_state(self, throttle=None, rudder=None):
        with self._lock:
            if throttle is not None:
                self.throttle = clamp(float(throttle), -1.0, 1.0)
            if rudder is not None:
                self.rudder = clamp(float(rudder), -1.0, 1.0)

    def stop(self):
        """Panic stop: zero throttle/rudder and send immediately rather than
        waiting for the next background tick. Does not touch armed_cmd --
        matches the firmware's own link-loss failsafe, which centres/zeroes
        but leaves the ARM decision to an explicit command."""
        with self._lock:
            self.throttle = 0.0
            self.rudder = 0.0
            if self.connected:
                self._send_motor_locked(0.0)
                self._send_steer_locked(0.0)

    def arm(self, do_arm: bool, force: bool):
        with self._lock:
            self.armed_cmd = bool(do_arm)
            self.force = bool(force)
            if self.connected:
                self._send_arm_locked(self.armed_cmd, self.force)

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
                'rudder': self.rudder,
                'armed_cmd': self.armed_cmd,
                'force': self.force,
                'seq': self.seq,
                'last_error': self.last_error,
                'telemetry': self._with_age(self.telemetry, TELEMETRY_STALE_S),
                # Bridge status arrives ~1/s from the S3 -- a longer stale
                # window than telemetry is correct, not a copy/paste of it.
                'bridge_status': self._with_age(self.bridge_status, 3.0),
            }

    def _stream_loop(self):
        period = 1.0 / self.send_hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                if self.connected:
                    self._send_motor_locked(self.throttle)
                    self._send_steer_locked(self.rudder)
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
            if len(payload) != plen or len(payload) < 24:
                return
            uptime_s, pkts, byts, frames, hello, drops = struct.unpack('<6I', payload[:24])
            with self._lock:
                self.bridge_status = {
                    'have': True, 'last_rx_monotonic': time.monotonic(),
                    'uptime_s': uptime_s, 'espnow_pkts': pkts, 'espnow_bytes': byts,
                    'frames_out': frames, 'hello_sent': hello, 'reasm_drops': drops,
                }
            return

        if msg_type == MSG_FIELD_TELEMETRY:
            expect_len = struct.calcsize(FIELD_TELEMETRY_FMT)
            if len(payload) != plen or plen != expect_len:
                self._diag('field telemetry length mismatch',
                            f'expected {expect_len}B, got {len(payload)}B (header said {plen}B)')
                return
            (pitch, roll, heading, gps_valid, lat, lon,
             speed_mps, course_deg, satellites, hdop) = struct.unpack(FIELD_TELEMETRY_FMT, payload)
            self._diag_counts['ok'] = self._diag_counts.get('ok', 0) + 1
            with self._lock:
                self.telemetry = {
                    'have': True, 'last_rx_monotonic': time.monotonic(),
                    'heading': heading, 'pitch': pitch, 'roll': roll,
                    'gps_valid': bool(gps_valid), 'lat': lat, 'lon': lon,
                    'speed_mps': speed_mps, 'course_deg': course_deg,
                    'satellites': satellites, 'hdop': hdop,
                }
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
            self._diag('decoded but no sensors field',
                        f'which_oneof={msg.WhichOneof("payload")!r}')
            return

        self._diag_counts['ok'] = self._diag_counts.get('ok', 0) + 1
        s = msg.sensors
        with self._lock:
            self.telemetry = {
                'have': True, 'last_rx_monotonic': time.monotonic(),
                'heading': s.imu.heading, 'pitch': s.imu.pitch, 'roll': s.imu.roll,
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
  <div class="row slider-row">
    <label>Throttle</label>
    <input type="range" id="throttle" min="-100" max="100" value="0" step="5">
    <span class="val" id="throttle-val">0%</span>
  </div>
  <div class="row slider-row">
    <label>Rudder</label>
    <input type="range" id="rudder" min="-100" max="100" value="0" step="5">
    <span class="val" id="rudder-val">0%</span>
  </div>
  <button id="stop-btn">STOP</button>
</div>

<div class="card" id="telemetry-card">
  <div class="card-title">Telemetry <span class="pill" id="telem-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>Heading</label><span class="val" id="t-heading">--</span></div>
  <div class="telem-row"><label>Pitch / Roll</label><span class="val" id="t-attitude">--</span></div>
  <div class="telem-row"><label>GPS Fix</label><span class="val" id="t-fix">--</span></div>
  <div class="telem-row"><label>Lat / Lon</label><span class="val" id="t-latlon">--</span></div>
  <div class="telem-row"><label>Sats / HDOP</label><span class="val" id="t-sats">--</span></div>
  <div class="telem-row"><label>Speed / Course</label><span class="val" id="t-speed">--</span></div>
</div>

<div class="card" id="bridge-card">
  <div class="card-title">Bridge (S3) <span class="pill" id="bridge-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>ESP-NOW pkts</label><span class="val" id="b-pkts">--</span></div>
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
  $('conn-pill').textContent = isConn ? `CONNECTED ${port}` : 'DISCONNECTED';
  $('conn-pill').classList.toggle('up', isConn);
  $('connect-btn').textContent = isConn ? 'DISCONNECT' : 'CONNECT';
  $('connect-btn').classList.toggle('disconnect', isConn);
  $('port-select').disabled = isConn;
  $('drive-card').classList.toggle('disabled-overlay', !isConn);
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

$('rudder').addEventListener('input', (e) => {
  const v = parseInt(e.target.value);
  $('rudder-val').textContent = v + '%';
  api('/api/state', 'POST', { rudder: v / 100 });
});

$('stop-btn').addEventListener('click', async () => {
  $('throttle').value = 0; $('throttle-val').textContent = '0%';
  $('rudder').value = 0; $('rudder-val').textContent = '0%';
  await api('/api/stop', 'POST');
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

function applyStatus(s) {
    if (s.connected !== connected) setConnectedUI(s.connected, s.port);
    armedCmd = s.armed_cmd;
    $('state-dot').classList.toggle('armed', s.armed_cmd);
    $('arm-label').textContent = (s.armed_cmd ? 'ARMED' : 'DISARMED') + ' (commanded)';
    $('arm-btn').textContent = s.armed_cmd ? 'DISARM' : 'ARM';
    $('arm-btn').classList.toggle('arm', !s.armed_cmd);
    $('arm-btn').classList.toggle('disarm', s.armed_cmd);
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
            return json.loads(self.rfile.read(length))
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
            self.link.set_state(throttle=body.get('throttle'), rudder=body.get('rudder'))
            self._json({'ok': True})
        elif self.path == '/api/stop':
            self.link.stop()
            self._json({'ok': True})
        elif self.path == '/api/arm':
            self.link.arm(bool(body.get('arm')), bool(body.get('force')))
            self._json({'ok': True})
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
