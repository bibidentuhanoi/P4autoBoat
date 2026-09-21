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

The page is three columns on a laptop: what you DO on the left (connect,
drive, the tests), the map in the middle, what the boat TELLS you on the
right (telemetry, confirmed motor state, sensor health, bridge). The map is
Leaflet with OpenStreetMap tiles (a satellite and a dark layer are one click
away), fetched from the internet when the page loads; with no internet the
map box says so and everything else keeps working. The boat's trail is kept
by THIS process (GpsTrack) so a reload gets it back, and served at /api/track.

NOTE: the armed/force state in the Drive card is the last command WE SENT.
The boat's own answer -- real arm state, real per-motor throttle, servo rail
-- arrives in MotorStatus and is shown in "Motor (confirmed by boat)"; the two
disagreeing is exactly the thing to look at when an ARM does nothing. If ARM
doesn't visibly do anything, that's consistent with "sent but refused" (no
GPS lock, force unchecked) as much as "never arrived" -- telemetry moving
proves the LINK is alive, not that any specific command was accepted.

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
import collections
import hashlib
import json
import math
import csv
import os
import random
import re
import struct
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

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
GPS_TRACK_MAX_POINTS = 5000    # the map's trail; oldest fixes drop first
GPS_TRACK_MIN_MOVE_M = 1.5     # under typical fix jitter, so a moored boat
                               # does not scribble the trail full of noise
# Leaflet, pinned and hash-checked. The page loads it from the CDN at runtime
# (never from <head>): an offline laptop still gets the page instantly.
LEAFLET_JS_URL = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'
LEAFLET_JS_SRI = 'sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo='
LEAFLET_CSS_URL = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'
LEAFLET_CSS_SRI = 'sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY='

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
# 'both_long' (3) is BENCH_KIND_BASE_LONG: the same BASE run driven for 10 s
# instead of 3. A distinct kind and a distinct FILENAME, deliberately -- the
# two are the same experiment at different durations, so their numbers look
# comparable and are not. A 10 s file landing in the historical BASE_T20_* set
# would be averaged with 21 runs of real 3 s data and nobody would ever know.
BENCH_KIND = {'both': 0, 'left': 1, 'right': 2, 'both_long': 3}
BENCH_KIND_NAME = {0: 'BASE', 1: 'LEFT', 2: 'RIGHT', 3: 'BASE10'}
BENCH_STATE_NAME = {0: 'idle', 1: 'still', 2: 'driving', 3: 'coasting',
                    4: 'SAVED', 5: 'FAILED'}
# A live run publishes its state at ~5 Hz. Anything older than this cannot
# still be running -- most likely the terminal SAVED packet was lost over the
# air. Without this the tool latches on 'driving' and refuses every later run.
BENCH_RUNNING_STALE_S = 3.0
# A run we ASKED for is treated as live from the request until the boat's
# first BenchStatus about it, or until this window passes unanswered.
BENCH_REQUEST_PENDING_S = 3.0
# BenchStatus.state == 2 is the drive phase (motor_control.c / bench_run.c:
# 0 idle, 1 baseline, 2 run, 3 coast, 4 saved, 5 failed). Named because the
# yaw summary is delimited by it and a bare 2 in that test reads as nothing.
BENCH_STATE_RUN = 2
# The only two states a completed drive phase can hand off to: the clock runs
# out into the motors-off coast, which then saves. Any other exit from RUN is
# the boat aborting (bench_step's disarm / excessive-yaw fail paths), and it
# saves nothing.
BENCH_STATE_COAST = 3
BENCH_STATE_SAVED = 4
BENCH_STATE_FAILED = 5

# The learner's clamp, mirrored from motor_control.c (.c_min / .c_max). A value
# outside this is refused by the boat, so refuse it here too rather than send a
# command that will be silently dropped.
TRIMLEARN_C_MIN = 0.10
TRIMLEARN_C_MAX = 0.35

# Length of the boat's drive phase PER KIND, mirrored from motor_control.c:
# BENCH_RUN_US_BASE / _SPLIT are 3 s; BENCH_RUN_US_BASE_LONG (kind 3,
# BASE_LONG) is 10 s. Only used to judge whether the yaw telemetry we happened
# to catch covers the whole run -- and that judgement has to know which run:
# 3 s of frames is a complete BASE and less than a third of a BASE10.
BENCH_DRIVE_S = 3.0                       # the default; every kind but one
BENCH_DRIVE_S_BY_KIND = {0: 3.0, 1: 3.0, 2: 3.0, 3: 10.0}
# The boat publishes telemetry at roughly 20 Hz, so a 3 s drive phase should
# yield ~60 frames (a 10 s BASE10 ~200). Below this the summary is too thin to
# mean anything.
BENCH_YAW_MIN_SAMPLES = 12
# Fraction of the drive phase the caught samples must span to count as complete.
BENCH_YAW_MIN_COVERAGE = 0.6
# Longest hole in the caught frames that still leaves a run describable.
#
# Deliberately NOT TELEMETRY_STALE_S. That one is a LIVENESS test -- "is the
# boat still talking to us" -- and 2 s of it is a generous margin for a
# readout that only has to stop showing a frozen number. Here the question is
# a different one: can these samples describe the turn? A 2 s hole in a 3 s
# run leaves one third of it and would still pass, while the integrated angle
# quietly interpolated straight across the missing two thirds. At ~20 Hz this
# is about six consecutive dropped frames.
BENCH_YAW_MAX_GAP_S = 0.30


# ---- automatic rudder test --------------------------------------------------
# A laptop-driven turn: hold a known rudder deflection at a known throttle for a
# known time and record what the gyro says, so the installed IMU's yaw sign can
# be pinned to a physical direction. Purely a tool feature -- the boat has no
# idea this exists, which is why the recording is over-the-air and lossy rather
# than the boat's own 100 Hz SD file.
#
# The sequence drives NOTHING itself. It sets self.throttle / self.rudder and
# lets the existing 15 Hz stream loop transmit them, so STOP, DISARM,
# disconnect and the calibration gate all keep working exactly as they did.
# Normalized steer, +1 = right (post-2026-08-31). 0.80 rather than a token
# nudge: 0.30 produced too little turn to read against the boat's own yaw
# noise, and the whole point is a deflection big enough to see unambiguously.
# Not 1.00 -- that parks the servo on its mechanical stop for 4.5 s.
RUDDER_TEST_DEFLECTION = 0.80
# Percent form, DERIVED. Everything that shows or names the deflection reads
# this, so changing the constant above cannot leave a stale "30" in a filename,
# a button or a status pill -- which it did, in six places, before this existed.
RUDDER_TEST_PCT = int(round(RUDDER_TEST_DEFLECTION * 100))
RUDDER_TEST_THROTTLE = 0.20        # linked, both motors -- matches the T20 bench runs
# (name, duration, commanded throttle). Boundaries are measured from t0, never
# accumulated per tick, so 15 Hz jitter cannot drift them.
RUDDER_TEST_PHASES = (
    ('rudder_settle', 0.5, 0.0),   # rudder over, motors off: let the servo arrive
    ('drive',         3.0, RUDDER_TEST_THROTTLE),
    ('coast',         1.0, 0.0),   # motors off, rudder still over: residual yaw
)
RUDDER_TEST_TOTAL_S = sum(d for _n, d, _t in RUDDER_TEST_PHASES)
# The boat's MotorStatus must be this fresh to count as confirmation.
#
# NOT TELEMETRY_STALE_S. That is 2.0 s and is tuned for the ~20 Hz telemetry
# stream, where 2 s is forty missed frames. MotorStatus is published by
# task_runtime_diagnostics on CHANGE or once a second, whichever comes first
# (runtime_metrics.c), so on a boat whose motors are stopped -- exactly the
# state this gate is checking -- it arrives at 1 Hz and nothing more.
#
# Borrowing the 2 s limit therefore meant TWO lost packets aborted a run, which
# over a lossy ESP-NOW link is close to inevitable: it killed both assisted
# runs on 2026-09-02, reporting "boat status is stale" while the real fault was
# elsewhere. 3.5 s tolerates three consecutive misses at the 1 Hz cadence.
#
# Relaxing it costs nothing in safety because the direct check below --
# "is the boat actually driving" -- catches the real fault faster and says
# what it is, instead of inferring it from a missing packet.
RUDDER_TEST_STATUS_MAX_AGE_S = 3.5
# A hole longer than this in the received frames means the recording cannot
# describe the turn. Same reasoning (and value) as the bench summary.
RUDDER_TEST_MAX_GAP_S = BENCH_YAW_MAX_GAP_S
# The DRIVE phase is the measurement; the settle and coast phases are context.
# So coverage is judged on the drive window alone -- a run with plenty of
# frames that happen to land in the coast phase has recorded nothing useful.
RUDDER_TEST_DRIVE_S = dict((n, d) for n, d, _t in RUDDER_TEST_PHASES)['drive']
RUDDER_TEST_MIN_DRIVE_COVERAGE = 0.8    # of the 3 s drive window
RUDDER_TEST_MIN_DRIVE_FRAMES = 20       # ~20 Hz over 3 s should give ~60
# How long the boat may report zero throttle while the drive phase is
# commanding T20 before the run is abandoned. Covers the command reaching the
# boat, the ESCs spinning up and a MotorStatus coming back at 1 Hz -- so it has
# to clear one full publish interval with margin. Anything longer than this is
# not latency, it is a boat that is not driving (almost always: not armed).
RUDDER_TEST_DRIVE_CONFIRM_S = 1.5
# How long to wait for the BOAT to confirm Assisted Steering before giving up.
# assist_rudder is in MotorStatus's immediate-publish set, so a healthy boat
# acknowledges within one radio round trip; 2 s is many times that.
RUDDER_TEST_ASSIST_ACK_S = 2.0

# ---- the single active browser control session -----------------------------
#
# This process STREAMS ON THE BROWSER'S BEHALF. That is the whole hazard: close
# the tab at 40% throttle and Python carries on transmitting 40% forever,
# because the boat's own link failsafe sees a perfectly healthy stream. The
# browser dying has to be detectable HERE, and the only way is for the browser
# to keep saying it is alive.
#
# (dashboard.html has no such problem and needs no such lease: the browser IS
# the sender there, so if it dies the commands stop and the firmware's own
# CONTROL_LINK_TIMEOUT_US failsafe zeroes the boat. Adding a ground-station
# lease to a path with no intermediary would guard nothing.)
#
# One session at a time. A reload supersedes the old one, so a stale tab cannot
# keep driving alongside a fresh one.
CONTROL_HEARTBEAT_HZ = 12          # browser -> here; inside the lease with margin
CONTROL_LEASE_S = 0.30             # ~3.6 missed heartbeats
# How often an unconfirmed Assisted-OFF is re-sent.
ASSIST_OFF_RETRY_S = 0.25
# How long manual control may be held while an assisted-OFF goes unconfirmed
# AND no status is available to answer the question another way. A bound, not a
# preference: an unbounded hold is a lockout the operator cannot escape.
ASSIST_OFF_BLOCK_MAX_S = 3.0

RUDDER_TEST_DIR = Path('/workspaces/BoatEspP4/dataout')
# Assisted Steering: the same 0.5/3.0/1.0 profile, but instead of holding a
# fixed rudder it hands the boat a yaw-rate TARGET and lets the firmware's
# rudder loop chase it. Full stick is CONFIG_STABILITY_SAS_RMAX_DPS, which the
# first pool configuration sets to 2 deg/s -- so stick -1 asks for +2 (left)
# and stick +1 for -2 (right).
ASSIST_TEST_TARGET_DPS = 2.0
# A BASE/bench run, as this laptop saw it over the radio. The boat writes its
# own 100 Hz file to SD and that stays authoritative; this is what the link
# actually delivered, and when the two disagree THAT is the finding -- so
# neither may stand in for the other.
BENCH_CSV_COLUMNS = (
    't_mono', 'elapsed_s', 'yaw_dps', 'heading_deg',
    # What the motors were really doing. Yaw alone cannot tell a trim result
    # from a run the boat refused -- the same gap that made the 2026-09-02
    # rudder runs undiagnosable until boat_state/boat_servo_power were added.
    'boat_left', 'boat_right', 'boat_state', 'boat_servo_power',
    'bench_state', 'bench_elapsed_s', 'bench_samples',
    'learn_c',           # the trim learner's c, as the BOAT reports it
    'boat_p_on',         # the boat's own P-assist state; a mismatch voids an A/B
    'heading_target_deg', 'heading_error_deg', 'yaw_target_dps',
    'p_term', 'i_term', 'dynamic_c', 'effective_c', 'c_limit',
    'ctrl_active', 'heading_hold', 'saturated',
    'telem_age_s', 'gap_s',
)

RUDDER_TEST_CSV_COLUMNS = (
    't_mono', 'elapsed_s', 'phase', 'yaw_dps', 'heading_deg',
    'cmd_throttle', 'cmd_rudder', 'boat_left', 'boat_right',
    'servo_us', 'telem_age_s', 'gap_s',
    # --- assisted mode only; blank for a raw run -------------------------
    'mode',              # 'raw' or 'assisted'
    'target_dps',        # yaw rate REQUESTED of the firmware loop
    'yaw_filt_dps',      # the loop's own filtered yaw, as the boat reports it
    'boat_rudder_cmd',   # firmware-COMMANDED rudder (no position feedback)
    'boat_rudder_us',    # the pulse that command maps to
    'boat_saturated',    # the loop's output cap bound it
    'boat_assist_on',    # the BOAT's own answer, not what we asked for
    'boat_target_dps',   # the target the BOAT says its loop is holding
    # The two fields that would have diagnosed the 2026-09-02 runs outright.
    # boat_left/boat_right showed 0.0 all through the drive phase and there was
    # no way to tell "disarmed" from "armed but refusing" without them.
    'boat_state',        # esc_state_t: 0 disarmed, 1 arming, 2 armed
    'boat_servo_power',  # servo rail actually powered, as the boat reports it
)


# ---- one-button lake steering-identification test ---------------------------
#
# The foundational dataset for the yaw-rate and waypoint controllers: one press
# drives straight, deflects the raw physical rudder one way, recovers, deflects
# it the other way, recovers, and stops -- recording every field-telemetry
# frame from the moment the button is pressed until the boat has confirmed it
# is stopped. Laptop-driven exactly like the rudder test: this process sets
# throttle/rudder every tick and the existing 15 Hz stream transmits them. The
# boat does not know this test exists. No Motor P, no Rudder Assist, no mode
# switching during the run; those are separate experiments.
#
# SCIENTIFIC WORDING, deliberately repeated wherever the data is written:
#   - MotorStatus carries boat-APPLIED software commands (throttle, rudder,
#     PWM). There is no RPM, thrust or servo-angle feedback on this boat.
#   - The yaw PI is active while the rudder is centred (STRAIGHT and both
#     RECOVER phases), and suspended during the two TURN phases. The slow trim
#     learner stays frozen while PI is enabled; its stored c remains the
#     feed-forward baseline. BenchStatus records both c and PI effort.
#   - "Straight bias" is observed yaw/heading/course bias plus the commanded
#     L/R differential; it is not a measurement of physical motor mismatch.
LAKE_ID_DIR = RUDDER_TEST_DIR
LAKE_ID_THROTTLES = (0.20, 0.30)        # never auto-increased; server whitelist
LAKE_ID_MAGNITUDES = (0.30, 0.60)
LAKE_ID_PRECHECK_S = 2.0
LAKE_ID_PHASE_S = 10.0
LAKE_ID_STOP_S = 5.0
LAKE_ID_PROFILE_S = 57.0                # 2 + 5*10 + 5
LAKE_ID_POWERED_S = 50.0
LAKE_ID_PROFILES = ('full', 'straight', 'straight30', 'yawpulse')
YAW_PULSE_STRAIGHT_S = 20.0
YAW_PULSE_TURN_S = 2.0
YAW_PULSE_RECOVER_S = 10.0
STRAIGHT_RUN_S = 3.0                     # BASE 3s: precheck 2 + straight 3 + stop 5 = 10 s
STRAIGHT_LONG_RUN_S = 30.0               # BASE 30s: precheck 2 + straight 30 + stop 5 = 37 s
STRAIGHT_THROTTLE_MIN = 0.05             # the bench card's throttle box, as a fraction
STRAIGHT_THROTTLE_MAX = 0.60
LAKE_ID_TEARDOWN_MAX_S = 2.0            # 57 -> 59 at most, zeros throughout
LAKE_ID_TELEM_MAX_AGE_S = 1.0
LAKE_ID_TELEM_POWERED_MAX_AGE_S = 5.0  # finish through short downlink fades; mark the gap
LAKE_ID_MOTORSTATUS_MAX_AGE_S = 1.5     # gate + STOP confirm: a change publish is due there
# Powered phases: during a turn nothing in MotorStatus changes, so the boat only
# re-sends it once a second, and each lost packet is another 1 s of age. Two runs
# died on 2026-09-06 at one lost re-send ("MotorStatus stale (1.56 s)") and one
# on 2026-09-11 at two ("2.56 s", with SystemStatus lost the same way). TWO lost
# re-sends are tolerated -- the value the rudder test already uses for the same
# reason -- and a dead link still ends the run within 3.5 s. The gate and the
# STOP confirmation keep their own, tighter limits.
LAKE_ID_MOTORSTATUS_POWERED_MAX_AGE_S = 3.5
LAKE_ID_SYSTEMSTATUS_POWERED_MAX_AGE_S = 4.0   # ~1 Hz publish; two lost + jitter
# The boat is ALIVE if field telemetry (20 Hz, same radio, same firmware) is this
# fresh. Then a missing once-a-second status packet is a lost packet, not a dead
# boat: bench 2026-09-11 lost three and four in a row with telemetry still
# flowing. While alive, the status streams may go stale up to the hard cap below
# (their receive ages are in every recorded row); once telemetry goes quiet
# too, the limits above apply. The boat's own link-loss failsafe and the
# operator's STOP do not depend on any of this.
LAKE_ID_TELEM_ALIVE_S = 0.5
LAKE_ID_STATUS_ALIVE_CAP_S = 10.0
LAKE_ID_SYSTEMSTATUS_MAX_AGE_S = 3.0    # ~1 Hz publish; three missed = gone
LAKE_ID_SUPERVISION_S = 2.0             # browser heartbeat; NOT the 300 ms lease
LAKE_ID_STOP_CONFIRM_MAX_AGE_S = 1.5
LAKE_ID_RUDDER_NEUTRAL_US = 1516
LAKE_ID_RUDDER_CENTRE_TOL_US = 10
LAKE_ID_DRIVE_CONFIRM_S = RUDDER_TEST_DRIVE_CONFIRM_S
LAKE_ID_MAX_GAP_S = RUDDER_TEST_MAX_GAP_S
LAKE_ID_MIN_PHASE_COVERAGE = RUDDER_TEST_MIN_DRIVE_COVERAGE
LAKE_ID_QUEUE_MAX = 8192
LAKE_ID_FLUSH_EVERY_ROWS = 20
LAKE_ID_MAX_ROWS = 6000                 # ~59 s at 20 Hz is ~1200; a stuck run cannot grow this
# Provisional analysis thresholds. Every one is RECORDED in summary.json next
# to the numbers it was applied to, and none produces a valid/invalid label --
# only warnings -- until our own lake data justifies hard limits.
LAKE_ID_RULES = {
    'steady_window_s': 4.0,             # last N s of a turn = steady window
    'response_threshold_frac': 0.2,     # delay = |yaw-bias| first >= this * |steady-bias|
    'rise_low_frac': 0.1, 'rise_high_frac': 0.9,
    'recovery_band_frac': 0.2,          # recovered when |yaw-bias| <= this * |steady_prev-bias|
    'diff_drift_warn': 0.02,            # commanded L/R differential drift within a phase
    'frozen_imu_s': 1.0,                # identical IMU tuple for this long -> warning only
    'gps_advisory_min_sats': 6, 'gps_advisory_max_hdop': 2.5,
    'radius_min_gps_valid_fraction': 0.8,
    'radius_min_speed_mps': 0.3,
    'radius_min_yaw_sigma': 3.0,        # |mean yaw - straight bias| / straight yaw std
    'straight_steady_speed_tail_s': 5.0,
    'turn_settled_max_std_frac': 0.35,  # steady-window std <= this * |steady - bias| = settled
    'recovery_hold_s': 2.0,             # must STAY inside the band this long to count as recovered
    'half_decay_frac': 0.5,             # time for |yaw - bias| to first fall to this * the turn's steady
}
# The tool cannot read the P4 flash. The operator should copy the actual App
# version shown in the boat boot log after flashing; never prefill an old date.
LAKE_ID_FIRMWARE_LABEL_DEFAULT = 'unknown / enter boat App version from boot log'
LAKE_ID_FIRMWARE_LABEL_NOTE = ('operator-supplied / believed; not verified against the '
                               'flashed binary (the tool cannot read it)')
LAKE_ID_NOTE_FIELDS = ('battery', 'payload_load', 'mechanical_config', 'wind',
                       'current', 'waves', 'unusual_events')
LAKE_ID_CSV_COLUMNS = (
    't_utc', 't_mono', 'elapsed_s', 'phase', 'phase_elapsed_s', 'order',
    'throttle_set', 'magnitude_set',
    'yaw_dps', 'heading_deg', 'pitch_deg', 'roll_deg',
    'gps_valid', 'lat', 'lon', 'speed_mps', 'course_deg', 'satellites', 'hdop',
    'gps_values_changed',
    'cmd_throttle', 'cmd_rudder', 'cmd_left', 'cmd_right',
    'boat_applied_left_cmd', 'boat_applied_right_cmd',
    'boat_applied_rudder_cmd', 'boat_applied_rudder_pwm_us',
    'boat_state', 'boat_servo_power', 'boat_assist_motor_p', 'boat_assist_rudder',
    'boat_yaw_target_dps', 'boat_yaw_filt_dps', 'boat_saturated',
    'imu_ok', 'mag_ok', 'gps_ok', 'tof_a_ok', 'tof_b_ok', 'camera_ok',
    'telem_age_s', 'motor_status_age_s', 'fusion_age_ms', 'system_status_age_s', 'gap_s',
    'tool_send_age_s', 'bridge_age_s', 'bridge_uplink_rssi_dbm', 'bridge_espnow_pkts',
    'bridge_frames_out', 'bridge_reasm_drops', 'bench_learn_c_last', 'bench_status_age_s',
    'heading_target_deg', 'heading_error_deg', 'yaw_target_dps', 'yaw_filt_dps',
    'rate_error_dps',
    'p_term', 'i_term', 'dynamic_c', 'effective_c', 'c_limit',
    'ctrl_active', 'heading_hold', 'saturated',
)
LAKE_ID_EVENT_COLUMNS = ('t_utc', 't_mono', 'elapsed_s', 'phase', 'event', 'detail')
LAKE_ID_SAMPLES_HEADER = (
    '# lake steering-identification run -- recorded by tools/espnow_drive.py from '
    'ESP-NOW field-telemetry frames (~20 Hz), NOT the boat SD card',
    '# boat_applied_* are boat-APPLIED SOFTWARE COMMANDS from MotorStatus: not '
    'measured RPM, thrust or servo angle -- no actuator feedback exists',
    '# cmd_left/cmd_right are the explicit motor commands sent by this tool; '
    'cmd_rudder is the PHYSICAL rudder command and remains zero in motor-only profiles',
    '# gps_values_changed is ADVISORY only: lat/lon/speed/course differ from the '
    'previous row. It is not a fix flag, timestamp or sequence and must not be '
    'used for GPS rate, latency or freshness; GPS is 10 Hz under 20 Hz frames',
    '# mode: see summary.json mode and settings.raw_throttle_test. Normal runs require '
    'Motor P ON; raw diagnostic runs require Motor P OFF. Rudder Assist stays OFF',
    '# in normal firmware the legacy Motor P switch enables heading hold plus yaw-rate PI. '
    'A commanded motor differential suspends its output and freezes I; after 0.5 s equal motors it '
    'recaptures the current heading. Positive yaw/heading error is LEFT, negative is RIGHT',
    '# boat_assist_motor_p is the enable switch as reported by the boat, not proof of a nonzero '
    'correction; ctrl_active reports whether the yaw controller is applying output',
    '# heading_target_deg through saturated are the latest coherent controller snapshot from '
    'MotorStatus; motor_status_age_s states its radio age and fusion_age_ms is the age of the '
    'fusion sample at the firmware control tick. p_term/i_term/dynamic_c/effective_c/c_limit '
    'are dimensionless motor-command fractions',
    '# boat_yaw_filt_dps / boat_yaw_target_dps belong to the RUDDER controller (Assisted '
    'Steering), not the motor yaw controller; with Rudder Assist OFF they read 0',
    '# autotrim: the slow learned c is frozen while the yaw PI controller is enabled; dynamic_c '
    'is the fast PI correction and effective_c is the total correction actually requested',
    '# GPS fix is ADVISORY, not a gate: without it lat/lon/speed/course carry no fix '
    'and the turn radius is unavailable; yaw and heading metrics are unaffected',
    '# link columns: tool_send_age_s = seconds since THIS tool last wrote a motor frame to the '
    'bridge; bridge_* = the S3 bridge\'s own counters (packets heard on air, frames forwarded '
    'to USB, reassembly drops) and the uplink RSSI it reports; the bridge does not report '
    'uplink send failures. bench_learn_c_last comes from the latest BenchStatus; live controller '
    'fields come from MotorStatus',
)
LAKE_ID_STATUS_COMPLETE = 'complete'
LAKE_ID_STATUS_STOP_UNCONFIRMED = 'incomplete_stop_unconfirmed'
LAKE_ID_STATUS_ABORTED = 'aborted'
# The motion finished but the RECORD of it did not (a final flush, close or
# summary write failed). Never COMPLETE, never advances the LR/RL order.
LAKE_ID_STATUS_RECORDING_FAILED = 'incomplete_recording_failed'


def lake_id_is_straight_profile(profile):
    return profile in ('straight', 'straight30')


def lake_id_phases(throttle, magnitude, order, profile='full'):
    """(name, duration_s, throttle, rudder) x7 for the full steering ID, x3 for
    the straight-only profile. Canonical rudder: -1 = LEFT."""
    t = float(throttle)
    if lake_id_is_straight_profile(profile):
        run_s = STRAIGHT_LONG_RUN_S if profile == 'straight30' else STRAIGHT_RUN_S
        return (
            ('precheck', LAKE_ID_PRECHECK_S, 0.0, 0.0),
            ('straight', run_s,              t,   0.0),
            ('stop',     LAKE_ID_STOP_S,     0.0, 0.0),
        )
    if profile == 'yawpulse':
        s = -1.0 if order == 'LR' else 1.0
        m = abs(float(magnitude))
        return (
            ('precheck',  LAKE_ID_PRECHECK_S,       0.0, 0.0),
            ('straight',  YAW_PULSE_STRAIGHT_S,       t, 0.0),
            ('turn_a',    YAW_PULSE_TURN_S,           t, s * m),
            ('recover_a', YAW_PULSE_RECOVER_S,        t, 0.0),
            ('turn_b',    YAW_PULSE_TURN_S,           t, -s * m),
            ('recover_b', YAW_PULSE_RECOVER_S,        t, 0.0),
            ('stop',      LAKE_ID_STOP_S,            0.0, 0.0),
        )
    s = -1.0 if order == 'LR' else 1.0
    m = abs(float(magnitude))
    return (
        ('precheck',  LAKE_ID_PRECHECK_S, 0.0, 0.0),
        ('straight',  LAKE_ID_PHASE_S,    t,   0.0),
        ('turn_a',    LAKE_ID_PHASE_S,    t,   s * m),
        ('recover_a', LAKE_ID_PHASE_S,    t,   0.0),
        ('turn_b',    LAKE_ID_PHASE_S,    t,  -s * m),
        ('recover_b', LAKE_ID_PHASE_S,    t,   0.0),
        ('stop',      LAKE_ID_STOP_S,     0.0, 0.0),
    )


def lake_id_phase_at(phases, elapsed):
    """(name, dur, thr, rud, start_s) for this elapsed time, or None once the
    57 s profile is over. From elapsed time, never a counter, so a late tick
    cannot shift a boundary."""
    edge = 0.0
    for name, dur, thr, rud in phases:
        if elapsed < edge + dur:
            return name, dur, thr, rud, edge
        edge += dur
    return None


def lake_id_phase_windows(phases):
    out, edge = {}, 0.0
    for name, dur, _t, _r in phases:
        out[name] = (edge, edge + dur)
        edge += dur
    return out


def lake_id_profile_s(phases):
    """Whole profile, seconds. STOP confirmation and teardown clock against
    this, never a constant: a 10 s run must not be judged by a 57 s one."""
    return float(sum(p[1] for p in phases))


def lake_id_powered_s(phases):
    return float(sum(p[1] for p in phases if p[2] > 0))


def lake_id_condition(throttle, magnitude):
    return 'T%02d_M%02d' % (int(round(throttle * 100)), int(round(magnitude * 100)))


LAKE_ID_FOLDER_RE = re.compile(r'^LAKE_ID_(T\d{2}_M\d{2})_(LR|RL)_(\d{3})$')


def lake_id_scan(directory):
    """Per condition: how many COMPLETE runs exist and the next free index.
    Read from disk so it survives restarts. Aborted and stop-unconfirmed runs
    occupy an index but never advance the LR/RL order."""
    out = {}
    try:
        entries = list(Path(directory).iterdir())
    except OSError:
        entries = []
    for p in entries:
        m = LAKE_ID_FOLDER_RE.match(p.name)
        if not m or not p.is_dir():
            continue
        cond, _order, idx = m.group(1), m.group(2), int(m.group(3))
        c = out.setdefault(cond, {'complete': 0, 'max_index': 0})
        c['max_index'] = max(c['max_index'], idx)
        # Only a readable summary that says COMPLETE *and* carries no write
        # error counts. Missing, unreadable, malformed or write-failed runs
        # occupy their index and nothing more.
        try:
            with open(p / 'summary.json') as fh:
                summary = json.load(fh)
        except (OSError, ValueError):
            continue
        if (isinstance(summary, dict) and summary.get('status') == LAKE_ID_STATUS_COMPLETE
                and not summary.get('write_error')):
            c['complete'] += 1
    result = {}
    for t in LAKE_ID_THROTTLES:
        for m_ in LAKE_ID_MAGNITUDES:
            cond = lake_id_condition(t, m_)
            c = out.get(cond, {'complete': 0, 'max_index': 0})
            result[cond] = {
                'complete_runs': c['complete'],
                'next_order': 'LR' if c['complete'] % 2 == 0 else 'RL',
                'next_index': c['max_index'] + 1,
            }
    return result


STRAIGHT_FOLDER_RE = re.compile(r'^STRAIGHT_(T\d{2})_(\d{3})$')


def straight_run_condition(throttle):
    return 'STRAIGHT_T%02d' % int(round(throttle * 100))


def straight_run_scan(directory):
    """Per throttle: COMPLETE straight runs on disk and the next free index.
    Same rules as the lake scan (an aborted run keeps its index); no LR/RL
    order because a straight run has none."""
    out = {}
    try:
        entries = list(Path(directory).iterdir())
    except OSError:
        entries = []
    for p in entries:
        m = STRAIGHT_FOLDER_RE.match(p.name)
        if not m or not p.is_dir():
            continue
        cond, idx = 'STRAIGHT_' + m.group(1), int(m.group(2))
        c = out.setdefault(cond, {'complete': 0, 'max_index': 0})
        c['max_index'] = max(c['max_index'], idx)
        try:
            with open(p / 'summary.json') as fh:
                summary = json.load(fh)
        except (OSError, ValueError):
            continue
        if (isinstance(summary, dict) and summary.get('status') == LAKE_ID_STATUS_COMPLETE
                and not summary.get('write_error')):
            c['complete'] += 1
    result = {}
    for cond in set(out) | {straight_run_condition(t) for t in LAKE_ID_THROTTLES}:
        c = out.get(cond, {'complete': 0, 'max_index': 0})
        result[cond] = {'complete_runs': c['complete'], 'next_index': c['max_index'] + 1}
    return result


def lake_id_utc(t=None):
    t = time.time() if t is None else t
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(t)) + '.%03dZ' % int((t % 1) * 1000)


def lake_id_provenance(tool_path):
    """What ran. git HEAD alone is not enough for uncommitted code, so: HEAD,
    dirty flag, modified files, and the sha256 of the file actually executing.
    Everything that cannot be established says 'unknown' rather than guessing."""
    import subprocess
    tool_path = Path(tool_path).resolve()
    root = tool_path.parent.parent
    out = {'git_head': 'unknown', 'git_dirty': None, 'git_modified_files': [],
           'espnow_drive_sha256': 'unknown', 'espnow_drive_path': str(tool_path),
           'rudder_pulse_config_believed': {'source': 'unknown'}}
    try:
        out['espnow_drive_sha256'] = hashlib.sha256(tool_path.read_bytes()).hexdigest()
    except OSError:
        pass
    try:
        head = subprocess.run(['git', '-C', str(root), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, timeout=3)
        if head.returncode == 0:
            out['git_head'] = head.stdout.strip()
        st = subprocess.run(['git', '-C', str(root), 'status', '--porcelain',
                             '--untracked-files=no'],
                            capture_output=True, text=True, timeout=3)
        if st.returncode == 0:
            files = [l[3:] for l in st.stdout.splitlines() if l.strip()]
            out['git_dirty'] = bool(files)
            out['git_modified_files'] = files
    except (OSError, subprocess.SubprocessError):
        pass
    cfg = root / 'sdkconfig'
    try:
        text = cfg.read_text()
        vals = {}
        for key in ('STEER_PULSE_MIN_US', 'STEER_PULSE_NEUTRAL_US', 'STEER_PULSE_MAX_US'):
            m = re.search(r'^CONFIG_%s=(\d+)$' % key, text, re.M)
            vals[key.lower()] = int(m.group(1)) if m else None
        vals['source'] = str(cfg)
        vals['sdkconfig_sha256'] = hashlib.sha256(text.encode()).hexdigest()
        vals['note'] = ('BELIEVED from the repo sdkconfig at run start; the tool '
                        'cannot read the flashed firmware')
        out['rudder_pulse_config_believed'] = vals
    except OSError:
        pass
    return out


def _lake_mean(xs):
    return (sum(xs) / len(xs)) if xs else None


def _lake_std(xs):
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def _lake_integrate_wrapped(vals):
    """Sum of consecutive wrap-aware deltas: a turn through >180 deg still
    adds up correctly."""
    vals = [v for v in vals if isinstance(v, (int, float)) and math.isfinite(v)]
    return sum(wrap_deg(vals[i] - vals[i - 1]) for i in range(1, len(vals))) if len(vals) >= 2 else None


def _lake_fnum(v, nd=4):
    return round(v, nd) if isinstance(v, (int, float)) and math.isfinite(v) else None


def yaw_controller_summary(rows):
    """Reduce controller diagnostics from exactly the supplied drive rows.

    Callers choose the phase before calling this function. Missing telemetry
    stays missing: an empty or old-protobuf run must never look like a perfect
    zero-effort controller run.
    """
    def numbers(name):
        return [float(r[name]) for r in rows
                if isinstance(r.get(name), (int, float))
                and not isinstance(r.get(name), bool)
                and math.isfinite(r[name])]

    def flags(name):
        return [1.0 if bool(r[name]) else 0.0 for r in rows
                if isinstance(r.get(name), (bool, int, float))
                and math.isfinite(float(r[name]))]

    yaws = numbers('yaw_dps')
    headings = numbers('heading_deg')
    active = flags('ctrl_active')
    saturated = flags('saturated')
    p_terms = numbers('p_term')
    i_terms = numbers('i_term')
    return {
        'frames': len(rows),
        'heading_change_deg': _lake_fnum(_lake_integrate_wrapped(headings)),
        'mean_yaw_dps': _lake_fnum(_lake_mean(yaws)),
        'peak_yaw_dps': _lake_fnum(max(yaws, key=abs)) if yaws else None,
        'active_fraction': _lake_fnum(_lake_mean(active)) if active else None,
        'saturated_fraction': _lake_fnum(_lake_mean(saturated)) if saturated else None,
        'peak_abs_p': _lake_fnum(max((abs(v) for v in p_terms), default=None)),
        'peak_abs_i': _lake_fnum(max((abs(v) for v in i_terms), default=None)),
        'final_i': _lake_fnum(i_terms[-1]) if i_terms else None,
    }


def lake_id_phase_coverage(rows, window):
    """The rudder-test coverage rule, per phase: where the frames sit inside
    the window, not just how many there are."""
    start, end = window
    inside = [r for r in rows if start <= r['elapsed_s'] < end]
    length = end - start
    out = {'frames': len(inside), 'first_delay_s': round(length, 3),
           'tail_gap_s': round(length, 3), 'span_s': 0.0,
           'max_gap_s': round(length, 3), 'coverage_ok': False}
    if not inside:
        return out
    first, last = inside[0]['elapsed_s'], inside[-1]['elapsed_s']
    gaps = [inside[i]['elapsed_s'] - inside[i - 1]['elapsed_s'] for i in range(1, len(inside))]
    worst = max([first - start, end - last] + gaps)
    out.update({'first_delay_s': round(first - start, 3), 'tail_gap_s': round(end - last, 3),
                'span_s': round(last - first, 3), 'max_gap_s': round(worst, 3)})
    out['coverage_ok'] = (out['span_s'] >= length * LAKE_ID_MIN_PHASE_COVERAGE
                          and worst <= LAKE_ID_MAX_GAP_S)
    return out


def _lake_phase_yaw(rows, window):
    """The plain facts of one phase, reported whatever else can or cannot be
    computed: signed mean yaw, mean |yaw|, the peak, the integral of gyro yaw
    over the phase, the compass heading change, and frame coverage."""
    a, b = window
    pts = [(r['elapsed_s'], r['yaw_dps']) for r in rows
           if a <= r['elapsed_s'] < b and isinstance(r['yaw_dps'], (int, float))
           and math.isfinite(r['yaw_dps'])]
    out = {'frames': len(pts), 'coverage': lake_id_phase_coverage(rows, window),
           'signed_mean_yaw_dps': None, 'mean_abs_yaw_dps': None,
           'peak_yaw_dps': None, 'peak_at_s': None,
           'integrated_yaw_change_deg': None, 'heading_change_deg': None,
           'note': ('signed yaw: the LEFT-rudder hypothesis is positive; integrated = '
                    'trapezoid of gyro yaw over the phase; heading_change = compass')}
    if pts:
        ys = [y for _t, y in pts]
        out['signed_mean_yaw_dps'] = _lake_fnum(_lake_mean(ys))
        out['mean_abs_yaw_dps'] = _lake_fnum(_lake_mean([abs(y) for y in ys]))
        pk = max(pts, key=lambda q: abs(q[1]))
        out['peak_yaw_dps'] = _lake_fnum(pk[1])
        out['peak_at_s'] = _lake_fnum(pk[0] - a)
        if len(pts) >= 2:
            out['integrated_yaw_change_deg'] = _lake_fnum(sum(
                0.5 * (pts[k - 1][1] + pts[k][1]) * (pts[k][0] - pts[k - 1][0])
                for k in range(1, len(pts))))
    inside = [r for r in rows if a <= r['elapsed_s'] < b]
    out['heading_change_deg'] = _lake_fnum(_lake_integrate_wrapped([r['heading_deg'] for r in inside]))
    return out


def lake_id_summarize(rows, events, settings, provenance, status, reason,
                      abort_phase=None, stop_confirmed=None, notes=None,
                      firmware_label=None, max_gap_s=0.0, rules=None):
    """Pure. Everything the summary says is derived from the rows here, so a
    test can hand in synthetic rows with known answers.

    It answers three practical questions -- how much the boat yaws while
    straight, how strongly and quickly it turns each way, and how quickly the
    rotation reduces after centring -- and where the data cannot support a
    number (poor coverage, an unsettled response, a recovery that never
    settles) it says so instead of reporting a misleading gain or time."""
    rules = dict(LAKE_ID_RULES if rules is None else rules)
    profile = settings.get('profile', 'full')
    if profile == 'yawpulse':
        # A 4 s steady window would swallow both seconds of a pulse, including
        # its startup. Use the final second and still require the settled test.
        rules['steady_window_s'] = min(rules['steady_window_s'], 1.0)
    phases = lake_id_phases(settings['throttle'], settings['magnitude'], settings['order'],
                            profile=profile)
    windows = lake_id_phase_windows(phases)
    warnings = []

    def yaw_in(name):
        a, b = windows[name]
        return [(r['elapsed_s'], r['yaw_dps']) for r in rows
                if a <= r['elapsed_s'] < b and isinstance(r['yaw_dps'], (int, float))
                and math.isfinite(r['yaw_dps'])]

    def rows_in(name):
        a, b = windows[name]
        return [r for r in rows if a <= r['elapsed_s'] < b]

    coverage = {n: lake_id_phase_coverage(rows, windows[n]) for n in windows}
    for n, c in coverage.items():
        if n != 'precheck' and status == LAKE_ID_STATUS_COMPLETE and not c['coverage_ok']:
            warnings.append('%s: telemetry coverage below policy (span %.2f s, max gap %.2f s)'
                            % (n, c['span_s'], c['max_gap_s']))
    if max_gap_s > LAKE_ID_MAX_GAP_S:
        warnings.append('telemetry gap %.2f s exceeds %.2f s' % (max_gap_s, LAKE_ID_MAX_GAP_S))

    # ---- straight bias -------------------------------------------------
    sy = [y for _t, y in yaw_in('straight')]
    sr = rows_in('straight')
    bias = _lake_mean(sy)
    bias_std = _lake_std(sy)
    tail = [r for r in sr if r['elapsed_s'] >= windows['straight'][1] - rules['straight_steady_speed_tail_s']]
    sp = [r['speed_mps'] for r in tail if r.get('gps_valid') and isinstance(r.get('speed_mps'), (int, float))]
    diffs = [(r['boat_applied_right_cmd'] - r['boat_applied_left_cmd']) for r in sr
             if isinstance(r.get('boat_applied_right_cmd'), (int, float))
             and isinstance(r.get('boat_applied_left_cmd'), (int, float))]
    straight = {
        'note': ('raw throttle diagnostic: equal requested motor commands, C/P trim and '
                 'ESC shaping bypassed by the selected firmware; yaw can also reflect '
                 'hull, rudder, wind or current, not only motor mismatch'
                 if settings.get('raw_throttle_test') else
                 'straight running with heading/yaw-rate PI ON: observed yaw/heading/'
                 'course bias plus the commanded L/R differential -- NOT a measurement of '
                 'physical motor mismatch, and not P\'s isolated contribution'),
        'yaw': _lake_phase_yaw(rows, windows['straight']),
        'mean_yaw_dps': _lake_fnum(bias), 'yaw_std_dps': _lake_fnum(bias_std),
        'heading_drift_deg': _lake_fnum(_lake_integrate_wrapped([r['heading_deg'] for r in sr])),
        'course_drift_deg': _lake_fnum(_lake_integrate_wrapped(
            [r['course_deg'] for r in sr if r.get('gps_valid')])),
        'steady_speed_mps': _lake_fnum(_lake_mean(sp)),
        'commanded_lr_diff_start': _lake_fnum(diffs[0]) if diffs else None,
        'commanded_lr_diff_end': _lake_fnum(diffs[-1]) if diffs else None,
        'commanded_lr_diff_range': _lake_fnum(max(diffs) - min(diffs)) if diffs else None,
        'controller': yaw_controller_summary(sr),
    }

    # ---- per-phase commanded differential drift (autotrim + P moving) ---
    for n in ('straight', 'recover_a', 'recover_b', 'turn_a', 'turn_b'):
        d = [(r['boat_applied_right_cmd'] - r['boat_applied_left_cmd']) for r in rows_in(n)
             if isinstance(r.get('boat_applied_right_cmd'), (int, float))
             and isinstance(r.get('boat_applied_left_cmd'), (int, float))]
        if d and (max(d) - min(d)) > rules['diff_drift_warn']:
            warnings.append('%s: commanded L/R differential moved %.3f (> %.3f, provisional)'
                            % (n, max(d) - min(d), rules['diff_drift_warn']))

    # ---- turns -----------------------------------------------------------
    def radius_block(name, pr, b, coverage_ok):
        """PROVISIONAL turn radius: inputs and rules reported together."""
        win = [r for r in pr if r['elapsed_s'] >= b - rules['steady_window_s']]
        valid = [r for r in win if r.get('gps_valid') and isinstance(r.get('speed_mps'), (int, float))]
        frac = (len(valid) / len(win)) if win else 0.0
        v = _lake_mean([r['speed_mps'] for r in valid])
        yaw_abs = _lake_mean([r['yaw_dps'] for r in win
                              if isinstance(r['yaw_dps'], (int, float)) and math.isfinite(r['yaw_dps'])])
        above = abs(yaw_abs - bias) if (yaw_abs is not None and bias is not None) else None
        sigma = (above / bias_std) if (above is not None and bias_std > 0) else None
        rw = []
        if not coverage_ok:
            rw.append('telemetry coverage below policy')
        if frac < rules['radius_min_gps_valid_fraction']:
            rw.append('gps_valid fraction %.2f < %.2f' % (frac, rules['radius_min_gps_valid_fraction']))
        if v is None or v < rules['radius_min_speed_mps']:
            rw.append('mean speed %s < %.2f m/s' % (_lake_fnum(v), rules['radius_min_speed_mps']))
        if sigma is None or sigma < rules['radius_min_yaw_sigma']:
            rw.append('yaw above straight bias %s sigma < %.1f' % (_lake_fnum(sigma, 2), rules['radius_min_yaw_sigma']))
        radius = None
        if v is not None and yaw_abs is not None and yaw_abs != 0.0:
            radius = v / (abs(yaw_abs) * math.pi / 180.0)
        warnings.extend('%s turn radius: %s' % (name, w) for w in rw)
        return {
            'provisional': True, 'radius_m': _lake_fnum(radius, 2),
            'gps_valid_fraction': round(frac, 3), 'mean_speed_mps': _lake_fnum(v),
            'mean_yaw_dps': _lake_fnum(yaw_abs), 'yaw_above_straight_bias_dps': _lake_fnum(above),
            'yaw_sigma_vs_straight': _lake_fnum(sigma, 2),
            'window_s': rules['steady_window_s'], 'rules': {
                'min_gps_valid_fraction': rules['radius_min_gps_valid_fraction'],
                'min_speed_mps': rules['radius_min_speed_mps'],
                'min_yaw_sigma': rules['radius_min_yaw_sigma']},
            'warnings': rw,
            'unavailable_reason': None if radius is not None else 'speed or yaw unavailable',
            'formula': 'speed_mps / (|mean_yaw_dps| * pi / 180)',
        }

    def turn(name, rudder_cmd):
        pts = yaw_in(name)
        a, b = windows[name]
        pr = rows_in(name)
        yawblk = _lake_phase_yaw(rows, windows[name])
        out = {'rudder_cmd': rudder_cmd, 'motor_half_difference': rudder_cmd,
               'side': 'LEFT' if rudder_cmd < 0 else 'RIGHT',
               'expected_yaw_sign_hypothesis': '+' if rudder_cmd < 0 else '-',
               'note': ('motor differential response of the operational boat: negative half-difference '
                        'makes the right motor stronger and turns LEFT. Motor P remains enabled but '
                        'its correction is suspended during deliberate motor mismatch'),
               'yaw': yawblk, 'peak_yaw_dps': yawblk['peak_yaw_dps'],
               'settled': None, 'unavailable_reason': None,
               'steady_yaw_dps': None, 'steady_yaw_minus_bias_dps': None, 'steady_yaw_std_dps': None,
               'response_delay_s': None, 'rise_time_s': None,
               'heading_change_deg': _lake_fnum(_lake_integrate_wrapped([r['heading_deg'] for r in pr])),
               'course_change_deg': _lake_fnum(_lake_integrate_wrapped(
                   [r['course_deg'] for r in pr if r.get('gps_valid')]))}
        if not pts or bias is None:
            out['unavailable_reason'] = out['unavailable'] = 'no yaw samples in phase'
            return out
        cov = yawblk['coverage']
        if not cov['coverage_ok']:
            out['unavailable_reason'] = ('insufficient telemetry coverage (span %.2f s, max gap '
                                         '%.2f s): response metrics not reported'
                                         % (cov['span_s'], cov['max_gap_s']))
            warnings.append('%s: insufficient coverage -- rise time, delay and steady yaw not reported' % name)
            out['turn_radius'] = radius_block(name, pr, b, False)
            return out
        steady_pts = [y for t, y in pts if t >= b - rules['steady_window_s']]
        steady = _lake_mean(steady_pts)
        std = _lake_std(steady_pts)
        rel = (steady - bias) if steady is not None else None
        out['steady_yaw_dps'] = _lake_fnum(steady)
        out['steady_yaw_minus_bias_dps'] = _lake_fnum(rel)
        out['steady_yaw_std_dps'] = _lake_fnum(std)
        if rel is None:
            out['unavailable_reason'] = 'no samples in the steady window'
            out['turn_radius'] = radius_block(name, pr, b, True)
            return out
        detected = (abs(rel) >= rules['radius_min_yaw_sigma'] * bias_std) if bias_std > 0 else (abs(rel) > 0)
        settled = bool(detected and std <= rules['turn_settled_max_std_frac'] * abs(rel))
        out['settled'] = settled
        if not detected:
            out['unavailable_reason'] = ('no clear response: |steady - bias| %.2f deg/s is under %.1f '
                                         'sigma of the straight scatter (%.2f deg/s)'
                                         % (abs(rel), rules['radius_min_yaw_sigma'], bias_std))
            warnings.append('%s: no clear response above the straight scatter -- rise time and delay '
                            'not reported' % name)
        elif not settled:
            out['unavailable_reason'] = ('unsettled: steady-window std %.2f deg/s exceeds %.2f of '
                                         '|steady - bias| %.2f deg/s'
                                         % (std, rules['turn_settled_max_std_frac'], abs(rel)))
            warnings.append('%s: unsettled response -- rise time and delay not reported (std %.2f '
                            'vs |steady - bias| %.2f deg/s)' % (name, std, abs(rel)))
        else:
            sign = 1.0 if rel >= 0 else -1.0
            excursions = [(t, (y - bias) * sign) for t, y in pts]
            thr = rules['response_threshold_frac'] * abs(rel)
            lo, hi = rules['rise_low_frac'] * abs(rel), rules['rise_high_frac'] * abs(rel)
            t_thr = next((t for t, e in excursions if e >= thr), None)
            t_lo = next((t for t, e in excursions if e >= lo), None)
            t_hi = next((t for t, e in excursions if e >= hi), None)
            out['response_delay_s'] = _lake_fnum(t_thr - a) if t_thr is not None else None
            out['rise_time_s'] = _lake_fnum(t_hi - t_lo) if (t_lo is not None and t_hi is not None) else None
        expected = -1.0 if rudder_cmd < 0 else 1.0        # LEFT -> positive yaw
        if rel != 0 and (rel > 0) != (expected < 0):
            warnings.append('%s: steady yaw sign %s disagrees with the hypothesis (%s motor split -> %s yaw)'
                            % (name, '+' if rel > 0 else '-', out['side'], out['expected_yaw_sign_hypothesis']))
        out['turn_radius'] = radius_block(name, pr, b, True)
        return out

    def recovery(name, prev_turn):
        pts = yaw_in(name)
        a, b = windows[name]
        yawblk = _lake_phase_yaw(rows, windows[name])
        out = {'note': ('recovery after equal motor commands: heading/yaw-rate PI resumes after its '
                        'centred delay and recaptures the current heading; not a passive hull time constant'),
               'yaw': yawblk, 'recovery_time_s': None, 'half_decay_time_s': None,
               'recovery_band_dps': None, 'overshoot_dps': None, 'residual_yaw_dps': None,
               'unavailable_reason': None,
               'residual_heading_change_deg': _lake_fnum(_lake_integrate_wrapped(
                   [r['heading_deg'] for r in rows_in(name)]))}
        if not pts or bias is None:
            out['unavailable_reason'] = out['unavailable'] = 'no yaw samples in phase'
            return out
        cov = yawblk['coverage']
        if not cov['coverage_ok']:
            out['unavailable_reason'] = ('insufficient telemetry coverage (span %.2f s, max gap '
                                         '%.2f s): recovery time not reported' % (cov['span_s'], cov['max_gap_s']))
            warnings.append('%s: insufficient coverage -- recovery time not reported' % name)
            return out
        prev = prev_turn.get('steady_yaw_minus_bias_dps')
        if prev is None or prev == 0:
            out['unavailable_reason'] = ('previous turn has no usable steady yaw (%s)'
                                         % (prev_turn.get('unavailable_reason') or 'zero'))
            return out
        band = rules['recovery_band_frac'] * abs(prev)
        sign = 1.0 if prev > 0 else -1.0
        out['recovery_band_dps'] = _lake_fnum(band)
        dev = [(t, abs(y - bias)) for t, y in pts]
        half = rules['half_decay_frac'] * abs(prev)
        t_half = next((t for t, d in dev if d <= half), None)
        out['half_decay_time_s'] = _lake_fnum(t_half - a) if t_half is not None else None
        # Recovered = entered the band AND stayed inside it for recovery_hold_s.
        # One sample dipping into the band while the boat is still swinging is
        # not a recovery, and the hold must fit inside the phase to be proven.
        hold = rules['recovery_hold_s']
        t_rec = None
        for k, (t, d) in enumerate(dev):
            if d > band:
                continue
            if t + hold > b:
                break
            if all(d2 <= band for t2, d2 in dev[k:] if t2 <= t + hold):
                t_rec = t
                break
        out['recovery_time_s'] = _lake_fnum(t_rec - a) if t_rec is not None else None
        last = [y - bias for t, y in pts if t >= b - rules['steady_window_s']]
        out['residual_yaw_dps'] = _lake_fnum(_lake_mean(last))
        opposite = [(y - bias) * -sign for t, y in pts]
        out['overshoot_dps'] = _lake_fnum(max(opposite)) if opposite and max(opposite) > 0 else 0.0
        if t_rec is None:
            out['unavailable_reason'] = ('did not settle: never stayed within %.2f deg/s of the straight '
                                         'bias for %.1f s inside the phase' % (band, hold))
            warnings.append('%s: rotation did not settle within the phase -- recovery time not reported'
                            % name)
        return out

    ta = turn('turn_a', phases[2][3])
    tb = turn('turn_b', phases[4][3])
    ra = recovery('recover_a', ta)
    rb = recovery('recover_b', tb)
    left = ta if ta['side'] == 'LEFT' else tb
    right = tb if left is ta else ta
    asym = {}
    ls, rs = left.get('steady_yaw_minus_bias_dps'), right.get('steady_yaw_minus_bias_dps')
    if ls and rs:
        asym['left_over_right_steady_ratio'] = _lake_fnum(abs(ls) / abs(rs), 3) if rs else None
    if left.get('response_delay_s') is not None and right.get('response_delay_s') is not None:
        asym['delay_difference_s'] = _lake_fnum(left['response_delay_s'] - right['response_delay_s'], 3)

    # ---- data quality --------------------------------------------------
    powered = [r for r in rows if windows['straight'][0] <= r['elapsed_s'] < windows['stop'][0]]
    inval = sum(1 for r in powered if not r.get('gps_valid'))
    if powered and inval == len(powered):
        warnings.append('no GPS fix during the run -- speed, course and turn radius unavailable '
                        '(advisory: a fix is not a gate; yaw and heading metrics are unaffected)')
    elif powered and inval:
        warnings.append('gps_valid false on %d of %d powered rows' % (inval, len(powered)))
    sats = [r['satellites'] for r in rows if isinstance(r.get('satellites'), (int, float))]
    hd = [r['hdop'] for r in rows if isinstance(r.get('hdop'), (int, float))]
    if sats and min(sats) < rules['gps_advisory_min_sats']:
        warnings.append('advisory: satellites fell to %d (< %d)' % (min(sats), rules['gps_advisory_min_sats']))
    if hd and max(hd) > rules['gps_advisory_max_hdop']:
        warnings.append('advisory: hdop reached %.2f (> %.2f)' % (max(hd), rules['gps_advisory_max_hdop']))
    if any(r.get('boat_assist_motor_p') == 0 for r in powered):
        warnings.append('Motor P reported OFF during the run (required ON): the mode requirement was not met')
    if any(r.get('boat_assist_rudder') == 1 for r in rows):
        warnings.append('Rudder Assist reported ON during the run (required OFF)')
    # frozen IMU: identical tuple for >= frozen_imu_s -- warning only
    run_start, run_len = None, 0
    for r in rows:
        key = (r['yaw_dps'], r['heading_deg'], r['pitch_deg'], r['roll_deg'])
        if run_start is not None and key == run_start[0]:
            if r['elapsed_s'] - run_start[1] >= rules['frozen_imu_s'] and run_len == 0:
                warnings.append('IMU values identical for >= %.1f s from t=%.2f s (warning only)'
                                % (rules['frozen_imu_s'], run_start[1]))
                run_len = 1
        else:
            run_start, run_len = (key, r['elapsed_s']), 0
    for k in LAKE_ID_NOTE_FIELDS:
        if not (notes or {}).get(k):
            warnings.append('operator note missing: %s' % k)
    if stop_confirmed is False:
        warnings.append('STOP not confirmed by a fresh MotorStatus within the 2 s teardown')

    return {
        'schema': 'lake_id_summary_v2',
        'status': status, 'reason': reason, 'abort_phase': abort_phase,
        'stop_confirmed': stop_confirmed,
        'settings': dict(settings, phases=[list(p) for p in phases],
                          profile_s=lake_id_profile_s(phases), powered_s=lake_id_powered_s(phases),
                         # The freshness limits that were in force: a run's record
                         # must say how tolerant it was of a lossy link.
                         freshness_limits_s={
                             'telemetry': LAKE_ID_TELEM_MAX_AGE_S,
                             'motorstatus_gate': LAKE_ID_MOTORSTATUS_MAX_AGE_S,
                             'motorstatus_powered': LAKE_ID_MOTORSTATUS_POWERED_MAX_AGE_S,
                             'systemstatus_gate': LAKE_ID_SYSTEMSTATUS_MAX_AGE_S,
                             'systemstatus_powered': LAKE_ID_SYSTEMSTATUS_POWERED_MAX_AGE_S,
                             'stop_confirm': LAKE_ID_STOP_CONFIRM_MAX_AGE_S,
                             'browser_supervision': LAKE_ID_SUPERVISION_S,
                             'telemetry_alive': LAKE_ID_TELEM_ALIVE_S,
                             'status_alive_cap': LAKE_ID_STATUS_ALIVE_CAP_S}),
        'mode': {
            'motor_p': 'ON', 'rudder_assist': 'OFF',
            'required': ('Motor P ON and Rudder Assist OFF for the whole run, confirmed by the '
                         'boat before any throttle; the tool never toggles a mode during the run'),
            'p_gating_note': ('the heading/yaw-rate PI output is suspended while the requested '
                              'motor half-difference exceeds 0.02; after 0.5 s equal motors it recaptures the '
                              'current heading and resumes. The I term and slow trim learner freeze '
                              'while manual steering has authority'),
            'p_correction_recorded': True,
            'learned_c_recorded': True,
            'limitation': ('boat_assist_motor_p is the P switch as reported by the boat, not proof '
                           'of a nonzero correction; controller fields are the latest BenchStatus '
                           'snapshot and bench_status_age_s reports its age; boat_yaw_filt_dps and '
                           'boat_yaw_target_dps belong to the rudder controller (Assisted Steering), '
                           'not the motor yaw controller'),
        } if not settings.get('raw_throttle_test') else {
            'motor_p': 'OFF', 'rudder_assist': 'OFF',
            'raw_throttle_test': True,
            'required': 'Motor P OFF and Rudder Assist OFF, confirmed by MotorStatus',
            'trim_and_shaping': 'bypassed in CONFIG_ESC_RAW_THROTTLE_TEST firmware',
            'limitation': 'The launch option identifies the intended build; telemetry '
                          'does not verify the firmware switch or measure RPM/thrust.',
        },
        'provenance': dict(provenance or {}, firmware_label=firmware_label or 'unknown',
                           firmware_label_note=LAKE_ID_FIRMWARE_LABEL_NOTE),
        'operator_notes': {k: (notes or {}).get(k, '') for k in LAKE_ID_NOTE_FIELDS},
        'sample_count': len(rows), 'event_count': len(events),
        'phase_coverage': coverage,
        'telemetry_max_gap_s': round(max_gap_s, 3),
        'straight': straight,
        'turn_a': ta, 'recover_a': ra, 'turn_b': tb, 'recover_b': rb,
        'asymmetry': asym,
        'link': lake_id_link_stats(rows),
        'rules': rules,
        'wording': [
            'MotorStatus values are boat-applied software commands, not measured RPM, thrust or servo angle',
            'this recording measures the COMBINED system -- motor differential, yaw PI and learned trim '
             'together; it does not isolate P\'s contribution',
            'it does not demonstrate or prove assisted waypoint steering; it is preparation for it',
            'turn phases measure the motor differential response of the operational boat with yaw PI '
            'suspended by the firmware during deliberate motor mismatch',
            ('recoveries use raw throttle with motor corrections disabled'
             if settings.get('raw_throttle_test') else
             'recoveries use heading/yaw-rate PI with learned trim feed-forward, not passive hull tests'),
            'straight bias is not a direct measurement of physical motor mismatch',
            'turn radius is provisional; its inputs and rules are reported with it',
            'the P flag is the switch state, not proof of a nonzero correction; BenchStatus '
            'records the latest learned c and controller terms with their age',
            'a phase with insufficient coverage, no clear or unsettled response, or a recovery '
            'that did not settle reports no gain or time for it, by rule',
            *(['for 2 s yaw pulses, steady yaw is estimated from the final 1 s only; inspect '
               'settled and the raw samples before using it as a turn-rate value']
              if profile == 'yawpulse' else []),
        ],
        'warnings': warnings,
    }


# ---- STRAIGHT profiles: same machinery, straight-only powered windows --------
STRAIGHT_BIN_S = 0.5
STRAIGHT_START_S = 1.0        # 'start' = the first second of the powered window
STRAIGHT_END_S = 2.0          # 'end'   = its last two seconds
STRAIGHT_STRAIGHT_DPS = 3.0   # |mean| below this = not turning
STRAIGHT_TURN_DPS = 5.0       # |mean| above this = clearly turning
STRAIGHT_WOBBLE_DPS = 4.0     # spread above this = wobbling
STRAIGHT_SIDE_DPS = 1.0       # |mean yaw| below this over the run = no dominant side


def lake_id_link_stats(rows):
    """What the link did during a run, reduced from the per-row link columns.
    Answers 'who went quiet' after an abort: the tool (tool_send_age), the
    radio (bridge counters, RSSI) or the boat (status ages, telemetry gaps)."""
    def nums(k):
        return [r[k] for r in rows if isinstance(r.get(k), (int, float))]
    rssi, pkts = nums('bridge_uplink_rssi_dbm'), nums('bridge_espnow_pkts')
    frames, drops = nums('bridge_frames_out'), nums('bridge_reasm_drops')
    send, msage, gaps = nums('tool_send_age_s'), nums('motor_status_age_s'), nums('gap_s')
    return {
        'tool_send_age_max_s': _lake_fnum(max(send)) if send else None,
        'telemetry_gap_max_s': _lake_fnum(max(gaps)) if gaps else None,
        'motor_status_age_max_s': _lake_fnum(max(msage)) if msage else None,
        'bridge_rssi_min_dbm': min(rssi) if rssi else None,
        'bridge_rssi_max_dbm': max(rssi) if rssi else None,
        'bridge_espnow_pkts_delta': (pkts[-1] - pkts[0]) if len(pkts) >= 2 else None,
        'bridge_frames_out_delta': (frames[-1] - frames[0]) if len(frames) >= 2 else None,
        'bridge_reasm_drops_delta': (drops[-1] - drops[0]) if len(drops) >= 2 else None,
        'note': ('tool_send_age = seconds since this tool last wrote a motor frame to the bridge, '
                 'sampled at each telemetry frame; bridge counters are the S3 bridge\'s own '
                 '(packets heard on air, frames forwarded to USB, reassembly drops); the bridge '
                 'does not report uplink send failures, so a lost command frame leaves no count'),
    }


def straight_run_shape(start_mean, start_spread, end_mean, end_spread):
    """One label for how the yaw went through the run. Negative = RIGHT."""
    def side(m):
        return 'R' if m < 0 else 'L'
    s_str = abs(start_mean) < STRAIGHT_STRAIGHT_DPS and start_spread < STRAIGHT_WOBBLE_DPS
    e_str = abs(end_mean) < STRAIGHT_STRAIGHT_DPS and end_spread < STRAIGHT_WOBBLE_DPS
    if s_str and e_str:
        return 'straight all run'
    if (start_spread >= STRAIGHT_WOBBLE_DPS or abs(start_mean) >= STRAIGHT_STRAIGHT_DPS) and e_str:
        return 'wobble/turn %s at start -> straight by the end' % side(start_mean)
    if s_str and abs(end_mean) >= STRAIGHT_TURN_DPS:
        return 'quiet at start -> turning %s by the end' % side(end_mean)
    if (start_mean * end_mean < 0 and abs(start_mean) >= STRAIGHT_STRAIGHT_DPS
            and abs(end_mean) >= STRAIGHT_STRAIGHT_DPS):
        return 'REVERSES %s -> %s' % (side(start_mean), side(end_mean))
    if start_mean * end_mean > 0 and abs(end_mean) > abs(start_mean) + 3.0:
        return 'turning %s from the start, growing' % side(end_mean)
    if start_mean * end_mean > 0 and abs(start_mean) > abs(end_mean) + 3.0:
        return 'turn %s easing off' % side(start_mean)
    if start_spread >= STRAIGHT_WOBBLE_DPS and end_spread >= STRAIGHT_WOBBLE_DPS:
        return 'wobble all run, no clear side'
    if abs(end_mean) >= STRAIGHT_STRAIGHT_DPS:
        return 'steady turn %s' % side(end_mean)
    return 'mostly straight, some wobble'


def straight_run_analyze(rows, window):
    """Pure. The yaw through the powered window: half-second bins, start and
    end means, total turn, peaks each way, time spent each way, dominant side
    and the shape label. Negative yaw = RIGHT turn (gyro sign on this boat)."""
    a, b = window
    pts = [(r['elapsed_s'], r['yaw_dps']) for r in rows
           if a <= r['elapsed_s'] < b and isinstance(r.get('yaw_dps'), (int, float))
           and math.isfinite(r['yaw_dps'])]
    bins = []
    for i in range(int(round((b - a) / STRAIGHT_BIN_S))):
        t0 = a + i * STRAIGHT_BIN_S
        ys = [y for t, y in pts if t0 <= t < t0 + STRAIGHT_BIN_S]
        bins.append({'t0_s': round(t0 - a, 3), 'yaw_dps': _lake_fnum(_lake_mean(ys)) if ys else None,
                     'frames': len(ys)})
    out = {'frames': len(pts), 'yaw_bins': bins, 'coverage': lake_id_phase_coverage(rows, window),
           'start_mean_dps': None, 'start_spread_dps': None,
           'end_mean_dps': None, 'end_spread_dps': None,
           'mean_yaw_dps': None, 'median_yaw_dps': None, 'total_turn_deg': None,
           'peak_right_dps': None, 'peak_left_dps': None,
           'time_right_pct': None, 'time_left_pct': None,
           'dominant_side': 'none', 'shape': 'no data', 'heading_change_deg': None,
           'note': ('negative yaw = right turn; start = first %.0f s and end = last %.0f s of the '
                    'powered window; dominant side from the mean yaw over the window (|mean| < '
                    '%.1f deg/s = none); a reversal inside half a second is a collision or a hand'
                    % (STRAIGHT_START_S, STRAIGHT_END_S, STRAIGHT_SIDE_DPS))}
    if not pts:
        return out
    ys = [y for _t, y in pts]
    start = [y for t, y in pts if t < a + STRAIGHT_START_S]
    end = [y for t, y in pts if t >= b - STRAIGHT_END_S]
    mean_yaw = _lake_mean(ys)
    total = (sum(0.5 * (pts[k - 1][1] + pts[k][1]) * (pts[k][0] - pts[k - 1][0])
                 for k in range(1, len(pts))) if len(pts) >= 2 else 0.0)
    inside = [r for r in rows if a <= r['elapsed_s'] < b]
    out.update({
        'mean_yaw_dps': _lake_fnum(mean_yaw),
        'median_yaw_dps': _lake_fnum(sorted(ys)[len(ys) // 2]),
        'total_turn_deg': _lake_fnum(total),
        'peak_right_dps': _lake_fnum(min(ys)), 'peak_left_dps': _lake_fnum(max(ys)),
        'time_right_pct': _lake_fnum(100.0 * sum(1 for y in ys if y < -STRAIGHT_STRAIGHT_DPS) / len(ys), 1),
        'time_left_pct': _lake_fnum(100.0 * sum(1 for y in ys if y > STRAIGHT_STRAIGHT_DPS) / len(ys), 1),
        'dominant_side': ('right' if mean_yaw < -STRAIGHT_SIDE_DPS else
                          'left' if mean_yaw > STRAIGHT_SIDE_DPS else 'none'),
        'heading_change_deg': _lake_fnum(_lake_integrate_wrapped(
            [r['heading_deg'] for r in inside if isinstance(r.get('heading_deg'), (int, float))])),
    })
    if start and end:
        sm, ss, em, es = _lake_mean(start), _lake_std(start), _lake_mean(end), _lake_std(end)
        out.update({'start_mean_dps': _lake_fnum(sm), 'start_spread_dps': _lake_fnum(ss),
                    'end_mean_dps': _lake_fnum(em), 'end_spread_dps': _lake_fnum(es),
                    'shape': straight_run_shape(sm, ss, em, es)})
    else:
        out['shape'] = 'partial data'
    return out


def straight_run_summarize(rows, events, settings, provenance, status, reason,
                           abort_phase=None, stop_confirmed=None, notes=None,
                           firmware_label=None, max_gap_s=0.0, rules=None):
    """Pure. The record of a straight-only run: what was commanded, what the
    boat applied, what the link did, and how the yaw went -- in the shape the
    bench question asks: which way, and from when."""
    phases = lake_id_phases(settings['throttle'], settings['magnitude'], settings['order'],
                            settings.get('profile', 'straight'))
    windows = lake_id_phase_windows(phases)
    warnings = []
    coverage = {n: lake_id_phase_coverage(rows, windows[n]) for n in windows}
    for n, c in coverage.items():
        if n != 'precheck' and status == LAKE_ID_STATUS_COMPLETE and not c['coverage_ok']:
            warnings.append('%s: telemetry coverage below policy (span %.2f s, max gap %.2f s)'
                            % (n, c['span_s'], c['max_gap_s']))
    if max_gap_s > LAKE_ID_MAX_GAP_S:
        warnings.append('telemetry gap %.2f s exceeds %.2f s' % (max_gap_s, LAKE_ID_MAX_GAP_S))
    straight = straight_run_analyze(rows, windows['straight'])
    sr = [r for r in rows if windows['straight'][0] <= r['elapsed_s'] < windows['straight'][1]]
    lefts = [r['boat_applied_left_cmd'] for r in sr
             if isinstance(r.get('boat_applied_left_cmd'), (int, float))]
    rights = [r['boat_applied_right_cmd'] for r in sr
              if isinstance(r.get('boat_applied_right_cmd'), (int, float))]
    diffs = [rr - ll for ll, rr in zip(lefts, rights)]
    straight.update({
        'commanded_throttle': settings['throttle'],
        'applied_left_mean': _lake_fnum(_lake_mean(lefts)) if lefts else None,
        'applied_right_mean': _lake_fnum(_lake_mean(rights)) if rights else None,
        'applied_lr_diff_mean': _lake_fnum(_lake_mean(diffs)) if diffs else None,
        'applied_lr_diff_range': _lake_fnum(max(diffs) - min(diffs)) if diffs else None,
        'controller': yaw_controller_summary(sr),
    })
    p_state = settings.get('p_at_start')
    for k in LAKE_ID_NOTE_FIELDS:
        if not (notes or {}).get(k):
            warnings.append('operator note missing: %s' % k)
    if stop_confirmed is False:
        warnings.append('STOP not confirmed by a fresh MotorStatus within the 2 s teardown')
    return {
        'schema': 'straight_run_summary_v1',
        'status': status, 'reason': reason, 'abort_phase': abort_phase,
        'stop_confirmed': stop_confirmed,
        'settings': dict(settings, phases=[list(p) for p in phases],
                         profile_s=lake_id_profile_s(phases), powered_s=lake_id_powered_s(phases),
                         freshness_limits_s={
                             'telemetry': LAKE_ID_TELEM_MAX_AGE_S,
                             'motorstatus_gate': LAKE_ID_MOTORSTATUS_MAX_AGE_S,
                             'motorstatus_powered': LAKE_ID_MOTORSTATUS_POWERED_MAX_AGE_S,
                             'systemstatus_gate': LAKE_ID_SYSTEMSTATUS_MAX_AGE_S,
                             'systemstatus_powered': LAKE_ID_SYSTEMSTATUS_POWERED_MAX_AGE_S,
                             'stop_confirm': LAKE_ID_STOP_CONFIRM_MAX_AGE_S,
                             'browser_supervision': LAKE_ID_SUPERVISION_S,
                             'telemetry_alive': LAKE_ID_TELEM_ALIVE_S,
                             'status_alive_cap': LAKE_ID_STATUS_ALIVE_CAP_S}),
        'mode': {
            'motor_p': 'ON' if p_state else ('OFF' if p_state is False else 'unknown'),
            'rudder_assist': 'OFF',
            'required': ('Rudder Assist OFF for the whole run; Motor P either way, recorded at the '
                         'gate, and the run aborts if it flips'),
            'p_correction_recorded': True,
            'learned_c_recorded': True,
            'limitation': ('boat_assist_motor_p is the P switch as reported by the boat, not proof '
                           'of a nonzero correction; controller values and learned c come from the '
                           'latest BenchStatus snapshot, whose age is recorded per row'),
        },
        'provenance': dict(provenance or {}, firmware_label=firmware_label or 'unknown',
                           firmware_label_note=LAKE_ID_FIRMWARE_LABEL_NOTE),
        'operator_notes': {k: (notes or {}).get(k, '') for k in LAKE_ID_NOTE_FIELDS},
        'sample_count': len(rows), 'event_count': len(events),
        'phase_coverage': coverage,
        'telemetry_max_gap_s': round(max_gap_s, 3),
        'straight': straight,
        'link': lake_id_link_stats(rows),
        'wording': [
            'MotorStatus values are boat-applied software commands, not measured RPM, thrust or servo angle',
            'laptop/radio-observed at ~20 Hz, NOT the boat\'s 100 Hz SD recording of a bench run',
            'yaw is the boat\'s gyro; negative = right turn; a sharp reversal inside half a second is '
            'a collision or a hand, not the jets',
            'straight bias is the combined system (jets, trim, P if on, rudder, hull, water), not a '
            'direct measurement of motor mismatch',
        ],
        'warnings': warnings,
    }


class LakeIdWriter:
    """Bounded background writer. The control loop only ever enqueues (never
    blocks, never touches a file); this thread does every open/write/flush/
    close and the atomic summary. Any failure is recorded in `error` for the
    control loop to act on -- the thread itself stays alive so a later
    finalize can still preserve whatever it can.

    Lifecycle: prepare() (disk work, BoatLink lock RELEASED) -> start() (the
    thread) -> put() -> finalize(). A start refused after prepare() calls
    discard(), which removes only what prepare() created.

    The queue holds ONE slot more than the data limit and put() refuses rows
    and events at that limit, so finalize() always has a free slot. A full
    data queue is a recording failure that aborts the run -- and the abort's
    own finalize must never be the item that gets dropped."""

    def __init__(self, directory, maxsize=LAKE_ID_QUEUE_MAX):
        import queue
        self.dir = Path(directory)
        self.data_max = int(maxsize)
        self.q = queue.Queue(maxsize=self.data_max + 1)
        self.error = None
        self.rows_enqueued = 0
        self.rows_dropped = 0
        self.rows_written = 0
        self.events_enqueued = 0
        self.events_dropped = 0
        self.events_written = 0
        self.finalized = threading.Event()
        self.result = None
        self._fh = {}
        self._w = {}
        self._pending = 0
        self._created = []              # files THIS writer created, in order
        self._dir_created = False
        self._finalize_queued = False
        self._thread = threading.Thread(target=self._loop, name='lake-id-writer', daemon=True)

    # ---- lifecycle ---------------------------------------------------------

    def prepare(self):
        """Create the folder and both CSVs with their headers. Disk work: call
        with the BoatLink lock RELEASED. Raises on failure -- after closing
        and removing everything this call itself created, and nothing else.
        The thread is not started here."""
        try:
            self.dir.mkdir(parents=True, exist_ok=False)
            self._dir_created = True
            self._open_csv('samples', LAKE_ID_CSV_COLUMNS, LAKE_ID_SAMPLES_HEADER)
            self._open_csv('events', LAKE_ID_EVENT_COLUMNS, ())
            for fh in self._fh.values():
                fh.flush()
        except Exception:
            self.discard()
            raise

    def _open_csv(self, kind, columns, header_lines):
        path = self.dir / (kind + '.csv')
        fh = open(path, 'w', newline='')
        self._fh[kind] = fh
        self._created.append(path)
        for line in header_lines:
            fh.write(line + '\n')
        w = csv.DictWriter(fh, fieldnames=list(columns))
        w.writeheader()
        self._w[kind] = w

    def start(self):
        """Start the writer thread. Only after prepare() succeeded."""
        if 'samples' not in self._fh or 'events' not in self._fh:
            raise RuntimeError('recorder not prepared')
        self._thread.start()

    def discard(self):
        """Undo prepare(): close every handle, remove the files this writer
        created, then its folder if that leaves it empty. Never removes
        anything it did not create."""
        for fh in list(self._fh.values()):
            try:
                fh.close()
            except Exception:                            # noqa: BLE001
                pass
        self._fh.clear()
        self._w.clear()
        for path in reversed(self._created):
            try:
                os.unlink(path)
            except OSError:
                pass
        self._created = []
        if self._dir_created:
            try:
                self.dir.rmdir()                          # refuses a non-empty folder
            except OSError:
                pass
            self._dir_created = False

    # ---- the data path (control thread, under the BoatLink lock) -----------

    def put(self, kind, item):
        """Non-blocking. True if enqueued. Rows and events are refused at the
        DATA limit, which keeps the slot above it free for finalize(); a
        refusal is a recording failure (recorded in `error`), never a stall."""
        import queue
        full = self.q.qsize() >= self.data_max
        if not full:
            try:
                self.q.put_nowait((kind, item))
            except queue.Full:
                full = True
        if full:
            self.error = self.error or 'writer queue full'
            if kind == 'row':
                self.rows_dropped += 1
            elif kind == 'event':
                self.events_dropped += 1
            return False
        if kind == 'row':
            self.rows_enqueued += 1
        elif kind == 'event':
            self.events_enqueued += 1
        return True

    def finalize(self, summary, on_done):
        """Hand the run to the writer: flush, close, write summary.json, then
        on_done(result). Non-blocking and cannot be dropped: it takes the
        reserved slot, and if the thread is not alive (never started, or gone)
        it runs on a one-shot thread so on_done still fires. False only if a
        finalize is already pending."""
        import queue
        if self._finalize_queued:
            return False
        self._finalize_queued = True
        if self._thread.is_alive():
            try:
                self.q.put_nowait(('finalize', (summary, on_done)))
                return True
            except queue.Full:                            # cannot happen: see put()
                pass
        threading.Thread(target=self._finalize_and_report, args=(summary, on_done),
                         name='lake-id-finalize', daemon=True).start()
        return True

    # ---- the thread --------------------------------------------------------

    def _loop(self):
        while True:
            kind, item = self.q.get()
            try:
                if kind == 'row':
                    self._w['samples'].writerow(item)
                    self.rows_written += 1
                    self._pending += 1
                    if self._pending >= LAKE_ID_FLUSH_EVERY_ROWS:
                        self._fh['samples'].flush(); self._pending = 0
                elif kind == 'event':
                    self._w['events'].writerow(item)
                    self._fh['events'].flush()
                    self.events_written += 1
                elif kind == 'finalize':
                    summary, on_done = item
                    self._finalize_and_report(summary, on_done)
                    return
            except Exception as exc:                       # noqa: BLE001
                # ANY failure, not just OSError: a closed handle raises
                # ValueError, a bad row ValueError, and a dying writer would
                # take the finalize -- and the partial data -- with it. Record
                # it for the control loop (which aborts motion) and keep
                # draining so finalize can still preserve what it can.
                self.error = self.error or ('%s: %s' % (type(exc).__name__, exc))
                print('[lake-id] recording failed: %s' % self.error, file=sys.stderr, flush=True)
            finally:
                self.q.task_done()

    def _finalize_and_report(self, summary, on_done):
        try:
            result = self._finalize(summary)
        except Exception as exc:                           # noqa: BLE001
            err = '%s: %s' % (type(exc).__name__, exc)
            self.error = self.error or err
            result = self._verdict(summary, err, 'finalize', False)
        self.result = result
        self.finalized.set()
        try:
            on_done(result)
        except Exception as exc:                           # noqa: BLE001
            print('[lake-id] result callback failed: %s: %s' % (type(exc).__name__, exc),
                  file=sys.stderr, flush=True)

    def _recording(self, summary, err):
        """The honest count block: received vs enqueued vs written, so a lost
        row or event is visible in summary.json rather than implied away."""
        rec = dict(summary.get('recording') or {})
        rec.update({
            'sample_rows_enqueued': self.rows_enqueued,
            'sample_rows_dropped': self.rows_dropped,
            'sample_rows_written': self.rows_written,
            'events_enqueued': self.events_enqueued,
            'events_dropped': self.events_dropped,
            'events_written': self.events_written,
            'write_error': err,
            'complete': err is None,
            'note': ('written = handed to the file; the final flush/close verdict is '
                     'write_error, and dropped rows/events are lost'),
        })
        return rec

    def _verdict(self, summary, err, stage, summary_written):
        """The structured result on_done() receives. A COMPLETE run whose
        record failed is downgraded here, once, for both the file and the
        in-memory result."""
        status, profile_status = summary.get('status'), summary.get('profile_status')
        reason = summary.get('reason')
        if err is not None and status == LAKE_ID_STATUS_COMPLETE:
            profile_status, status = LAKE_ID_STATUS_COMPLETE, LAKE_ID_STATUS_RECORDING_FAILED
            reason = 'recording failed: %s' % err
        return {'ok': err is None, 'error': err, 'stage': stage,
                'summary_written': bool(summary_written), 'status': status,
                'profile_status': profile_status, 'reason': reason,
                'recording': self._recording(summary, err)}

    def _finalize(self, summary):
        """Flush and close both CSVs, then summary.json via tmp + atomic
        replace. Returns the structured verdict; a failure anywhere here is
        part of it -- the motion may have finished, the record of it did not."""
        err = self.error
        stage = 'recording' if err else None
        for name, fh in list(self._fh.items()):
            try:
                fh.flush(); fh.close()
            except Exception as exc:                       # noqa: BLE001
                if err is None:
                    err, stage = '%s: %s' % (type(exc).__name__, exc), 'close_' + name
        self._fh.clear()
        summary = dict(summary)
        summary['recording'] = self._recording(summary, err)
        summary['write_error'] = err
        summary['samples_written'] = self.rows_written
        if err is not None and summary.get('status') == LAKE_ID_STATUS_COMPLETE:
            summary['profile_status'] = LAKE_ID_STATUS_COMPLETE
            summary['status'] = LAKE_ID_STATUS_RECORDING_FAILED
            summary['reason'] = 'recording failed: %s' % err
        summary_written = False
        tmp = self.dir / 'summary.json.tmp'
        try:
            with open(tmp, 'w') as fh:
                json.dump(summary, fh, indent=1, sort_keys=True)
                fh.flush(); os.fsync(fh.fileno())
            os.replace(tmp, self.dir / 'summary.json')     # atomic
            summary_written = True
        except Exception as exc:                           # noqa: BLE001
            if err is None:
                err, stage = '%s: %s' % (type(exc).__name__, exc), 'summary'
            try:
                os.unlink(tmp)
            except OSError:
                pass
        return self._verdict(summary, err, stage, summary_written)


def rudder_test_drive_coverage(rows):
    """How well the caught frames actually cover the DRIVE window.

    Frame count alone is not coverage. Sixty frames crammed into the last
    second, or a recording that only started 1.2 s in, both look healthy by
    count and describe most of the turn not at all. So this measures where the
    frames sit relative to the 0.5 .. 3.5 s drive window:

      drive_frames      how many landed inside it
      drive_first_delay how long after the window opened the first one arrived
      drive_tail_gap    how long before it closed the last one arrived
      drive_span        first to last, inside the window
      drive_max_gap     worst hole between consecutive frames -- INCLUDING the
                        lead-in and tail, because a hole at either edge hides
                        exactly as much of the turn as one in the middle
    """
    start = RUDDER_TEST_PHASES[0][1]            # drive opens after the settle
    end = start + RUDDER_TEST_DRIVE_S
    inside = [r for r in rows if start <= r['elapsed_s'] < end]
    out = {
        'drive_frames': len(inside),
        'drive_first_delay_s': round(RUDDER_TEST_DRIVE_S, 3),
        'drive_tail_gap_s': round(RUDDER_TEST_DRIVE_S, 3),
        'drive_span_s': 0.0,
        'drive_max_gap_s': round(RUDDER_TEST_DRIVE_S, 3),
        'drive_incomplete': True,
    }
    if not inside:
        return out
    first, last = inside[0]['elapsed_s'], inside[-1]['elapsed_s']
    out['drive_first_delay_s'] = round(first - start, 3)
    out['drive_tail_gap_s'] = round(end - last, 3)
    out['drive_span_s'] = round(last - first, 3)
    gaps = [inside[i]['elapsed_s'] - inside[i - 1]['elapsed_s']
            for i in range(1, len(inside))]
    worst = max(gaps) if gaps else 0.0
    worst = max(worst, out['drive_first_delay_s'], out['drive_tail_gap_s'])
    out['drive_max_gap_s'] = round(worst, 3)
    out['drive_incomplete'] = (
        out['drive_frames'] < RUDDER_TEST_MIN_DRIVE_FRAMES
        or out['drive_span_s'] < RUDDER_TEST_DRIVE_S * RUDDER_TEST_MIN_DRIVE_COVERAGE
        or worst > RUDDER_TEST_MAX_GAP_S)
    return out


def rudder_test_phase_at(elapsed):
    """(phase name, commanded throttle) at `elapsed` seconds, or None when the
    sequence is over. Computed from the elapsed time rather than a running
    counter so a missed or late tick cannot shift a boundary."""
    edge = 0.0
    for name, dur, thr in RUDDER_TEST_PHASES:
        edge += dur
        if elapsed < edge:
            return name, thr
    return None


# ---- what the motors ACTUALLY get -------------------------------------------
# The boat applies the learned trim on top of the commanded split, for every
# run kind (bench_run.c: bench_commands() then esc_trim_apply_pair(), with
# trim = 2 * c * base from motor_control.c's bench_tick). That is deliberate --
# these runs are meant to measure the boat in its real operating condition,
# autotrim included -- but it means the nominal +/-delta is NOT what the jets
# see, and the two directions are NOT symmetric:
#
#     LEFT  differential = 2 * (c*base - delta)     <- zero at delta == c*base,
#                                                      REVERSED below it
#     RIGHT differential = 2 * (c*base + delta)     <- always amplified
#
# This function is the reference implementation; the browser mirrors it for the
# live preview. Nothing here changes the boat -- it only shows the operator what
# the boat is already going to do.
def bench_effective_commands(kind, base, delta, c):
    """Predict the (left, right) a bench run will really command.

    kind: 'both' | 'left' | 'right'. Returns a dict with the nominal pair, the
    trimmed pair actually sent, and both differentials (right - left, signed).
    Mirrors bench_commands() + esc_trim_apply_pair() exactly, clamps included."""
    base = clamp(float(base), 0.0, 1.0)
    delta = clamp(float(delta), 0.0, 1.0)
    c = float(c)
    if not math.isfinite(c):
        c = 0.0

    if kind == 'left':
        nl, nr = base + delta, base - delta
    elif kind == 'right':
        nl, nr = base - delta, base + delta
    else:
        nl = nr = base
    # The jets are unidirectional: bench_commands() clamps before the trim.
    nl, nr = clamp(nl, 0.0, 1.0), clamp(nr, 0.0, 1.0)

    trim = 2.0 * c * base
    if nl <= 0.0 and nr <= 0.0:
        left, right = nl, nr          # esc_trim_apply_pair: stopped stays stopped
    else:
        left = clamp(nl - 0.5 * trim, 0.0, 1.0)
        right = clamp(nr + 0.5 * trim, 0.0, 1.0)

    return {
        'nominal_left': nl, 'nominal_right': nr,
        'nominal_differential': nr - nl,
        'left': left, 'right': right,
        'differential': right - left,
        'trim': trim,
    }


def bench_split_cancel_delta(base, c):
    """The delta at which the trim exactly cancels a LEFT run: delta == c*base.
    Below it the LEFT run turns the boat the SAME way a RIGHT run does."""
    return abs(float(c)) * clamp(float(base), 0.0, 1.0)


def bench_split_warning(base, delta, c):
    """None when the chosen split is clearly stronger than the trim, otherwise
    a plain-English warning.

    This does NOT make the two directions symmetric and must not be described
    as doing so -- RIGHT is always the stronger of the two. It only keeps the
    LEFT run pointing the way its name says."""
    cancel = bench_split_cancel_delta(base, c)
    delta = clamp(float(delta), 0.0, 1.0)
    if cancel <= 0.0:
        return None
    if delta < cancel:
        return ('LEFT MOTOR STRONGER will drive the boat the SAME way as RIGHT: '
                'trim %.1f%% exceeds the %.1f%% split, so the nominal left run '
                'is reversed. Use more than %.1f%%.'
                % (cancel * 100.0, delta * 100.0, cancel * 100.0))
    if delta < 2.0 * cancel:
        return ('split %.1f%% is close to the %.1f%% the trim cancels -- the '
                'LEFT run will be weak and the two directions very lopsided. '
                'Above %.1f%% is clearer.'
                % (delta * 100.0, cancel * 100.0, 2.0 * cancel * 100.0))
    return None


# ---- UI-observed yaw summary ------------------------------------------------
# Computed from the telemetry frames this tool happened to receive while the
# boat reported itself in the drive phase. The boat's own SD CSV is sampled at
# 100 Hz and is the authoritative record; this is a lossy over-the-air view,
# good for "which way did it turn, and roughly how hard".
def wrap_deg(delta):
    """Signed shortest angular difference, in [-180, +180).

    Exactly 180 comes back as -180: a half turn is genuinely ambiguous in sign,
    and this is the conventional resolution. It never arises in practice here,
    because the deltas being wrapped are between CONSECUTIVE ~20 Hz frames."""
    return ((float(delta) + 180.0) % 360.0) - 180.0


def _r3(v):
    """Round for a header line. None/'' pass through so a missing value stays
    visibly missing rather than becoming a plausible 0.0."""
    return v if not isinstance(v, float) else round(v, 3)


def summarize_yaw(samples, aborted=False, drive_s=BENCH_DRIVE_S):
    """samples: list of (t_monotonic, yaw_rate_dps, heading_deg | None).

    Gyro yaw is the primary measurement. Heading change is accumulated from
    CONSECUTIVE wrapped deltas rather than first-vs-last, so a run that turns
    through more than 180 deg still adds up correctly.

    `aborted` marks a run the boat did not carry to completion (disarm, an
    excessive-yaw trip). Such a run is ALWAYS incomplete no matter how clean
    the frames look: the drive phase was cut short, so the samples describe
    part of a turn while the boat saved nothing to compare against."""
    out = {
        'have': False, 'n': 0, 'span_s': 0.0,
        'mean_yaw_dps': 0.0, 'peak_yaw_dps': 0.0, 'yaw_angle_deg': 0.0,
        'heading_change_deg': 0.0, 'heading_ok': False,
        'stale': False, 'incomplete': True, 'max_gap_s': 0.0,
        'aborted': bool(aborted),
    }
    usable = [s for s in samples
              if isinstance(s[1], (int, float)) and not isinstance(s[1], bool)
              and math.isfinite(s[1])]
    out['n'] = len(usable)
    if not usable:
        return out
    out['have'] = True

    rates = [s[1] for s in usable]
    # Signed peak: the largest EXCURSION, keeping its direction. max(abs) would
    # throw away the one thing this whole readout exists to establish.
    out['peak_yaw_dps'] = max(rates, key=abs)
    # Fallback only: with a single sample there is no span to divide by, and
    # that one reading is the only answer available.
    out['mean_yaw_dps'] = sum(rates) / len(rates)

    if len(usable) >= 2:
        out['span_s'] = usable[-1][0] - usable[0][0]
        gaps = [usable[i][0] - usable[i - 1][0] for i in range(1, len(usable))]
        out['max_gap_s'] = max(gaps) if gaps else 0.0
        out['stale'] = out['max_gap_s'] > BENCH_YAW_MAX_GAP_S
        # Trapezoidal: the rate is a continuous signal sampled unevenly, so
        # rectangles would bias whichever end happened to be sampled denser.
        angle = 0.0
        for i in range(1, len(usable)):
            dt = usable[i][0] - usable[i - 1][0]
            if dt <= 0.0:
                continue
            angle += 0.5 * (usable[i][1] + usable[i - 1][1]) * dt
        out['yaw_angle_deg'] = angle

        # TIME-weighted, not a sample average. Frames arrive unevenly -- a
        # burst then a hole -- and an arithmetic mean counts each frame equally
        # regardless of how long it stood for, so a dense burst during one part
        # of the turn drags the answer toward that part. Dividing the
        # integrated angle by the span it was integrated over is the mean rate
        # that actually produced the observed turn, and keeps
        # mean * span == angle exactly.
        if out['span_s'] > 0.0:
            out['mean_yaw_dps'] = out['yaw_angle_deg'] / out['span_s']

        headings = [s[2] for s in usable
                    if isinstance(s[2], (int, float)) and not isinstance(s[2], bool)
                    and math.isfinite(s[2])]
        if len(headings) >= 2:
            out['heading_ok'] = True
            out['heading_change_deg'] = sum(
                wrap_deg(headings[i] - headings[i - 1])
                for i in range(1, len(headings)))

    out['incomplete'] = (out['aborted']
                         or out['n'] < BENCH_YAW_MIN_SAMPLES
                         or out['span_s'] < drive_s * BENCH_YAW_MIN_COVERAGE
                         or out['stale'])
    return out


class GpsTrack:
    """The boat's trail for the map: every valid fix that moved at least
    min_move_m from the last one kept. It lives in this process rather than
    in the page so a browser reload (or a second tab) gets the whole trail
    back with one request (/api/track). Bounded: the oldest points drop
    first, so a long day on the water cannot grow it without limit."""

    def __init__(self, max_points: int = GPS_TRACK_MAX_POINTS,
                 min_move_m: float = GPS_TRACK_MIN_MOVE_M):
        self.max_points = max_points
        self.min_move_m = min_move_m
        self._points = collections.deque(maxlen=max_points)

    def __len__(self):
        return len(self._points)

    @staticmethod
    def _usable(lat, lon) -> bool:
        # NaN/inf and out-of-range values are garbage; (0, 0) is the "null
        # island" a receiver reports before it has any idea where it is.
        return (math.isfinite(lat) and math.isfinite(lon)
                and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0
                and not (lat == 0.0 and lon == 0.0))

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2) -> float:
        """Metres between two fixes. Equirectangular: exact enough for the few
        metres between consecutive fixes, and cheap enough for 20 Hz."""
        k = 111320.0                              # metres per degree of latitude
        dy = (lat2 - lat1) * k
        dx = (lon2 - lon1) * k * math.cos(math.radians((lat1 + lat2) / 2.0))
        return math.hypot(dx, dy)

    def add(self, lat, lon, valid: bool = True) -> bool:
        """Record a fix. Returns True if it became a new trail point."""
        if not valid or not self._usable(lat, lon):
            return False
        if self._points:
            last_lat, last_lon = self._points[-1]
            if self.distance_m(last_lat, last_lon, lat, lon) < self.min_move_m:
                return False
        self._points.append((float(lat), float(lon)))
        return True

    def clear(self):
        self._points.clear()

    def points(self) -> list:
        return [[lat, lon] for lat, lon in self._points]


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

    def __init__(self, boat_pb2, send_hz: float = SEND_HZ, raw_throttle_test=False):
        self.pb2 = boat_pb2
        self.raw_throttle_test = bool(raw_throttle_test)
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
        self.gps_track = GpsTrack()      # the map's trail, fed by every fix
        self.calibrate_status = self._blank_calibrate_status()
        self.system_status = self._blank_system_status()
        self.motor_status = self._blank_motor_status()
        self.bench_status = self._blank_bench_status()
        self.bench_requested_at = None    # monotonic time of the last send_bench()
        # Yaw telemetry caught while the boat reported itself driving, and the
        # summary derived from it once the phase ends. UI-observed and lossy --
        # the boat's 100 Hz SD CSV is the authoritative record.
        self.bench_yaw_samples = []
        self.bench_yaw = None
        # ...and the same run, recorded in full to a CSV on THIS computer.
        # Independent of the boat's SD file, which stays authoritative.
        self.bench_run = None            # rows of the run currently driving
        self.bench_csv_name = None       # last file written, for the UI
        self._bench_write = None         # queued CSV, flushed outside the lock
        self.bench_dir = RUDDER_TEST_DIR
        # Automatic rudder test: one shared state machine, ticked by the 15 Hz
        # stream loop. `_now` is injectable so the tests can drive the 4.5 s
        # sequence on a fake clock instead of actually sleeping through it.
        self._now = time.monotonic
        self.rudder_test = None
        self.rudder_test_result = None
        self._rudder_test_write = None
        self.rudder_test_dir = RUDDER_TEST_DIR
        # One-button lake steering-identification test (see LAKE_ID_*).
        self.lake_id = None
        self.lake_id_result = None
        self._lake_writer = None
        self._lake_id_next_cache = None
        self.lake_id_dir = LAKE_ID_DIR
        self._last_send_mono = None       # stamped by _stream_send_locked
        # Fill the next-order cache ONCE here, at startup on the main thread
        # with no lock held, so every status path (HTTP poll and the WebSocket
        # push alike) has it from the first message. The real page uses the
        # WebSocket, and a GET-only pre-step left NEXT ORDER at '--'.
        self._lake_id_refresh_next()
        # Mirrors the boat's runtime P switch. Default OFF -- the A arm must be
        # the default so a forgotten toggle cannot silently make every run a B.
        self.p_assist_on = False
        # Assisted Steering (rudder loop). Requested state; the BOAT's own
        # answer arrives in MotorStatus.assist_rudder and is what gets shown.
        self.assist_rudder_on = False
        # Correlates an acknowledgement with the request that caused it. Time
        # of arrival cannot: it records when THIS process handled a packet, so
        # a stale "assisted is on" processed after a new request would satisfy
        # a time-based check and authorise a rate demand the boat is not in a
        # mode to receive.
        # The one session allowed to command motion. Everything else may still
        # read status and press STOP.
        self.session_id = None
        self.session_seq = 0
        self.session_last_hb = 0.0
        # Assisted-OFF is a transition that must be ACKNOWLEDGED, not merely
        # sent: an unacknowledged OFF leaves the boat steering itself while the
        # operator believes they have manual control.
        self._assist_off_req_id = None
        self._assist_off_next_retry = 0.0
        self._assist_off_started = 0.0
        self._assist_pending_p_on = False
        self._assist_pending_rudder = False
        # The previous heartbeat's control values, so a repeated one can be
        # told from the operator actually moving something.
        self._last_hb_state = None
        self.lease_expired_at = None     # for the UI: why the boat went quiet
        self.assist_request_id = 0
        # 0 means "not yet seeded"; the first request picks a random start so
        # a restart cannot reuse the previous session's ids.
        self._assist_req_seq = 0
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
            'rudder_cmd': 0.0, 'rudder_pulse_us': 0, 'rudder_saturated': False,
            'assist_rudder': False, 'assist_motor_p': False,
            'yaw_target_dps': 0.0, 'yaw_filt_dps': 0.0,
            'assist_request_id': 0,
            'heading_target_deg': 0.0, 'heading_error_deg': 0.0,
            'p_term': 0.0, 'i_term': 0.0, 'dynamic_c': 0.0,
            'effective_c': 0.0, 'c_limit': 0.0,
            'ctrl_active': False, 'heading_hold': False, 'saturated': False,
            'fusion_age_ms': 0, 'rate_error_dps': 0.0,
            'motor_yaw_target_dps': 0.0, 'motor_yaw_filt_dps': 0.0,
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
            'heading_target_deg': 0.0, 'heading_error_deg': 0.0,
            'yaw_target_dps': 0.0, 'p_term': 0.0, 'i_term': 0.0,
            'dynamic_c': 0.0, 'effective_c': 0.0, 'c_limit': 0.0,
            'ctrl_active': False, 'heading_hold': False,
            'saturated': False,
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
        return self._write_locked(msg.SerializeToString())

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
            self._abort_rudder_test_locked('serial link disconnected')
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

    def _apply_state_locked(self, throttle=None, rudder=None,
                            left=None, right=None, split=None,
                            operator_moved=True):
        # Touching a stick mid-run means the operator wants control back.
        # Abort, and DISCARD the value they sent rather than applying it:
        # the abort has already forced throttle 0 / rudder centred, and
        # letting a half-pushed slider through would immediately undo that.
        # The next command after the abort is theirs to make deliberately.
        #
        # `operator_moved` is what makes that judgement possible. It used to be
        # inferred from a field simply BEING PRESENT, which held while the page
        # sent one-shot deltas -- but a full-state heartbeat carries every field
        # every time, at 12 Hz, so the presence test fired on the first keepalive
        # after a run started and aborted it within 83 ms. A repeated identical
        # value is not the operator touching anything.
        if self._rudder_test_busy_locked() and operator_moved and (
                throttle is not None or rudder is not None
                or left is not None or right is not None):
            self._abort_rudder_test_locked('manual control input')
            return
        if self._rudder_test_busy_locked():
            # A repeated heartbeat is a KEEPALIVE, not a command. While a
            # laptop-driven test owns the controls, applying its (unchanged,
            # zero) values here would overwrite the test's command between
            # ticks and leave stray zeros in the recorded cmd_throttle.
            return
        if throttle is not None:
            self.throttle = clamp(float(throttle), -1.0, 1.0)
            if split is None:
                self.motor_split = False      # linked: throttle drives both
        if left is not None:
            self.motor_left = clamp(float(left), -1.0, 1.0)
            if split is None:
                self.motor_split = True       # unlinked: independent per-motor
        if right is not None:
            self.motor_right = clamp(float(right), -1.0, 1.0)
            if split is None:
                self.motor_split = True
        if rudder is not None:
            self.rudder = clamp(float(rudder), -1.0, 1.0)
        if split is not None:
            # EXPLICIT wins. Inferring the mode from which fields arrived cannot
            # work once every field always arrives: `left`/`right` were present
            # in every heartbeat, so the boat sat permanently in per-motor mode
            # and the operator's throttle -- accepted, stored and acknowledged
            # -- was read by nothing.
            self.motor_split = bool(split)

    def set_state(self, throttle=None, rudder=None, left=None, right=None):
        """Direct control entry, no session. Retained for the automated tests
        and the rudder-test abort path, which run inside this process and are
        not a browser. Browser input goes through control_heartbeat()."""
        with self._lock:
            self._apply_state_locked(throttle, rudder, left, right)

    def set_winch(self, speed: float, command_seq: int):
        """Set winch speed and transmit immediately; the stream loop repeats
        the same value while it is held, and a release gets a zero out now."""
        with self._lock:
            if command_seq <= self.winch_command_seq:
                return False, 'stale command sequence'
            self.winch_command_seq = command_seq
            if self._rudder_test_busy_locked():
                return False, 'a rudder test is running — wait for it to finish'
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
            # Servo power OFF drops the rail the rudder needs to hold its
            # deflection, so it ends the run rather than racing it.
            if self._rudder_test_busy_locked():
                self._abort_rudder_test_locked('servo power changed')
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
        but leaves the ARM decision to an explicit command -- with ONE
        exception: a bench run the tool believes is live or just requested
        owns the ESCs and overrides these zeros every cycle, and the only
        stop every firmware version honours for it is DISARM, so that case
        disarms.

        STOP never puts a BenchCommand on the wire. The boat runs main
        9543ed1, whose BenchCommand has no `abort` field and whose pipeline
        reads ANY bench payload as a start (kind 0, base 0 -- clamped, not
        refused): an abort sent to it would start a 3 s zero-throttle BASE
        run. tests/test_stop_main_compat.py decodes every STOP frame with
        that firmware's own schema."""
        with self._lock:
            # NEVER rejected as stale. A stop that arrives out of order is
            # still a stop, and refusing it because a sequence number looks old
            # is the one refusal that can leave the boat running.
            if command_seq > self.winch_command_seq:
                self.winch_command_seq = command_seq
            # Drop the session. Any /api/state already in flight when this
            # landed now belongs to a session that no longer exists, so it
            # cannot restore the throttle a moment after the stop.
            self.session_id = None
            self.session_seq = 0
            self._zero_controls_locked()
            was_calibrating = self.calibrating
            self.calibrating = False          # STOP is also a calibration kill
            self._abort_rudder_test_locked('STOP pressed')
            if not self.connected:
                return False, 'serial link is disconnected'
            writes_ok = [
                self._send_motor_locked(0.0, 0.0),
                self._send_steer_locked(0.0),
                self._send_winch_locked(0.0),
            ]
            if was_calibrating:
                self._send_calibrate_locked(False)   # re-arm the firmware start latch
            if self._bench_running_locked() or self._bench_locally_pending_locked():
                # The zeros above are already out, so a lost disarm degrades
                # to a boat being told nothing rather than a running one.
                self.armed_cmd = False
                self.bench_requested_at = None
                writes_ok.append(self._send_arm_locked(False, self.force))
                print('[stop] a bench run is live or pending: DISARMED to stop it '
                      '(the version-compatible way -- re-ARM when ready)', flush=True)
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
            if start and self._rudder_test_busy_locked():
                return False, 'a rudder test is running — wait for it to finish'
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
            if not do_arm:
                # Zero and TRANSMIT before the disarm packet, so the boat is
                # already being told nothing even if the disarm itself is lost.
                # The stream then keeps repeating those zeros: a lost disarm
                # degrades to a stopped boat rather than a running one.
                self._abort_rudder_test_locked('disarmed')
                self._zero_controls_locked()
                self._transmit_zeros_locked()
                self.session_id = None
            else:
                # Arming spins thrusters. Refuse unless every control is at
                # zero, so it can never start against a held stick.
                if self._anything_commanded_locked():
                    return False, 'set every control to zero before arming'
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
            return True, None

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

    def _bench_locally_pending_locked(self):
        """A run this tool asked for within BENCH_REQUEST_PENDING_S that the
        boat has not yet said anything about. The boat starts it on its next
        control tick, so a STOP inside that window must assume it is live."""
        t = getattr(self, 'bench_requested_at', None)
        if t is None:
            return False
        return (time.monotonic() - t) <= BENCH_REQUEST_PENDING_S

    # ---- control session ------------------------------------------------

    def open_control_session(self):
        """Claim control. Supersedes any existing session, and starts from a
        stopped MANUAL drive: taking over must never inherit somebody else's
        throttle. A server-owned test keeps its own command authority.

        Random id, not a counter, for the same reason the assist request id is
        random -- a reload must not be able to reuse the previous session's
        identity and have delayed requests from it accepted."""
        with self._lock:
            automated = (getattr(self, 'rudder_test', None) is not None
                         or getattr(self, 'lake_id', None) is not None)
            if not automated:
                self._zero_controls_locked()
            self.session_id = '%08x%08x' % (random.getrandbits(32),
                                            random.getrandbits(32))
            self.session_seq = 0
            self.session_last_hb = self._now()
            self.lease_expired_at = None
            # Seed it with the zeroed state this session starts from, so the
            # page's first heartbeat is recognised as "unchanged" rather than
            # as the operator grabbing a control.
            self._last_hb_state = (0.0, 0.0, 0.0, 0.0, False)
            if self.connected and not automated:
                self._transmit_zeros_locked()
            return self.session_id

    def release_control_session(self):
        """Give up manual control deliberately: zero, transmit, drop the session.

        Distinct from stop() because that also aborts a calibration, and a tab
        being hidden must not do that. Distinct from letting the lease expire
        because this is immediate and unambiguous."""
        with self._lock:
            automated = (getattr(self, 'rudder_test', None) is not None
                         or getattr(self, 'lake_id', None) is not None)
            if not automated:
                self._zero_controls_locked()
                if self.connected:
                    self._transmit_zeros_locked()
            self.session_id = None
            self.session_seq = 0

    def _zero_controls_locked(self):
        """Every actuator this process commands, to zero, in one step. Not a
        loop over endpoints: a partial stop is the failure mode this exists to
        prevent."""
        self.throttle = 0.0
        self.motor_left = 0.0
        self.motor_right = 0.0
        self.motor_split = False
        self.rudder = 0.0
        self.winch_speed = 0.0
        self.winch_lease_until = 0.0

    def _transmit_zeros_locked(self):
        """Put those zeros on the wire NOW rather than waiting up to a stream
        tick. Best-effort: a failed write still leaves the local state zeroed,
        so the next tick keeps trying."""
        if not self.connected:
            return
        self._send_motor_locked(0.0, 0.0)
        self._send_steer_locked(0.0)
        self._send_winch_locked(0.0)

    def _anything_commanded_locked(self):
        return (self.throttle != 0.0 or self.motor_left != 0.0
                or self.motor_right != 0.0 or self.rudder != 0.0
                or self.winch_speed != 0.0)

    def _drive_lease_ok_locked(self, now):
        """May this process stream NON-ZERO commands right now?"""
        return (self.session_id is not None
                and (now - self.session_last_hb) <= CONTROL_LEASE_S)

    def control_heartbeat(self, session_id, seq, throttle=None, rudder=None,
                          left=None, right=None, split=None):
        """The browser's full-state heartbeat: 'I am alive, and this is every
        control value I intend'.

        Full state rather than deltas on purpose -- a dropped delta would
        otherwise leave the boat holding a value nobody is asking for any
        more."""
        with self._lock:
            # The CODE matters as much as the message. Two of these mean the
            # session is gone and the browser must acquire a new one; two mean
            # this single request was not applied and the session is perfectly
            # healthy. Conflating them made a reordered response or a
            # mid-transition assist-OFF tear down a working session and stop
            # the boat for no reason.
            if self.session_id is None:
                return False, ('no control session — reload the page',
                               'no_session')
            if session_id != self.session_id:
                # Includes the post-STOP case: STOP drops the session, so a
                # request already in flight when it landed cannot restore
                # anything.
                return False, ('not the active control session', 'wrong_session')
            if not isinstance(seq, int) or seq <= self.session_seq:
                # Out-of-order arrival, not a dead session. Refuse the REQUEST
                # and keep the session: heartbeats are sent continuously and
                # the next in-order one lands in under 100 ms.
                return False, ('stale command sequence', 'stale_seq')
            self.session_seq = seq
            self.session_last_hb = self._now()
            self.lease_expired_at = None
            if self._assist_off_pending_locked():
                # Assisted mode may still be ON aboard, where these values mean
                # something entirely different. Refuse rather than guess -- but
                # the session is fine and the lease has just been refreshed, so
                # the boat is held rather than dropped.
                return False, ('waiting for the boat to confirm Assisted '
                               'Steering OFF', 'assist_off_pending')
            # Did the OPERATOR move something, or is this the same values
            # again? At 12 Hz most heartbeats are the latter, and treating
            # every one as fresh input aborted running rudder tests instantly.
            state = (throttle, rudder, left, right, split)
            moved = (self._last_hb_state is not None
                     and state != self._last_hb_state)
            self._last_hb_state = state
            self._apply_state_locked(throttle, rudder, left, right, split,
                                     operator_moved=moved)
            return True, None

    def _assist_off_pending_locked(self):
        """True while we have asked for Assisted Steering OFF and the boat has
        not yet confirmed it.

        This blocks ordinary manual control, because until the boat confirms,
        the mode its rudder is in is unknown -- and a stick value means a
        yaw RATE in one mode and a rudder ANGLE in the other."""
        if self._assist_off_req_id is None:
            return False
        ms = self.motor_status
        # The question manual control actually depends on is "is the boat in
        # assisted mode RIGHT NOW" -- not "did it acknowledge my particular
        # request". A status saying assist_rudder is false answers it whatever
        # request id it carries, and demanding the id match meant a boat that
        # never echoes one (firmware that predates the field, or a single lost
        # reply) locked manual control out PERMANENTLY, with no way back short
        # of restarting this tool.
        if ms.get('have') and not ms.get('assist_rudder'):
            return False
        # No usable status at all: hold briefly, then let the operator drive
        # rather than strand them. The retry keeps running either way, and the
        # firmware clears assist itself on link loss or disarm.
        return (self._now() - self._assist_off_started) < ASSIST_OFF_BLOCK_MAX_S

    def _assist_off_tick_locked(self, now):
        """Retry the OFF until MotorStatus confirms it, using the SAME request
        id throughout so a late confirmation of an earlier attempt still
        counts. A single unacknowledged OFF would leave the boat steering
        itself while the operator believed they had manual control."""
        req = self._assist_off_req_id
        if req is None:
            return
        ms = self.motor_status
        want_p = bool(getattr(self, '_assist_pending_p_on', self.p_assist_on))
        want_rudder = bool(getattr(self, '_assist_pending_rudder', False))
        if (ms.get('have')
                and bool(ms.get('assist_motor_p')) == want_p
                and bool(ms.get('assist_rudder')) == want_rudder
                and ms.get('assist_request_id') == req):
            self._assist_off_req_id = None
            self._assist_off_next_retry = 0.0
            self.p_assist_on = want_p
            self.assist_rudder_on = want_rudder
            return
        if now >= self._assist_off_next_retry and self.connected:
            self._assist_off_next_retry = now + ASSIST_OFF_RETRY_S
            msg = self.pb2.BoatMessage()
            msg.assist.p_on = want_p
            msg.assist.rudder_assist = want_rudder
            msg.assist.request_id = req
            self._write_locked(msg.SerializeToString())

    def _next_assist_request_id(self):
        """Unique within a session AND across restarts, never 0.

        A counter starting at 1 every launch is not enough: restart this tool
        while the boat is still in assisted mode from the last session, and its
        standing MotorStatus echoes id 1 -- which the new session's first
        request also claims. The stale packet then answers for a request it
        never saw, which is the whole failure the id exists to prevent.

        So the sequence STARTS somewhere random in the 32-bit space. Two
        launches colliding needs the same start out of ~4.29e9, rather than
        being certain.

        0 is skipped on wraparound: it is the boat's power-on value, and
        accepting it would let a boat that has applied nothing look as though
        it had just acknowledged us."""
        seq = getattr(self, '_assist_req_seq', 0)
        if seq == 0:
            seq = random.getrandbits(32)
        seq = (seq + 1) & 0xFFFFFFFF
        if seq == 0:
            seq = 1                       # wraparound never lands on 0
        self._assist_req_seq = seq
        return seq

    def _send_steer_rate_locked(self, target_dps):
        """Yaw-rate demand, deg/s. Its own message on purpose -- see the
        SteerRateCommand comment in boat.proto. Caller holds the lock."""
        msg = self.pb2.BoatMessage()
        msg.steer_rate.target_dps = float(target_dps)
        return self._write_locked(msg.SerializeToString())

    def _send_assist_locked(self, p_on, rudder_assist):
        """One AssistCommand carrying BOTH switches. Caller holds the lock.

        They are separate fields on purpose -- p_on is the MOTOR
        differential-thrust assist, rudder_assist is the RUDDER yaw-rate loop --
        but they ride one message so the boat never sees a moment with both on.
        The firmware refuses that combination too; this just never asks."""
        if getattr(self, 'raw_throttle_test', False) and (p_on or rudder_assist):
            return False, 'raw throttle test requires Motor P and Rudder Assist OFF'
        if p_on and rudder_assist:
            return False, 'motor P assist and Assisted Steering are mutually exclusive'
        if not self.connected:
            return False, 'serial link is disconnected'
        req_id = self._next_assist_request_id()
        if not rudder_assist:
            # Track it until confirmed; _assist_off_tick_locked retries with
            # this same id and clears it on the matching acknowledgement.
            self._assist_off_req_id = req_id
            self._assist_off_started = self._now()
            self._assist_off_next_retry = self._now() + ASSIST_OFF_RETRY_S
            self._assist_pending_p_on = bool(p_on)
            self._assist_pending_rudder = False
        else:
            self._assist_off_req_id = None
        msg = self.pb2.BoatMessage()
        msg.assist.p_on = bool(p_on)
        msg.assist.rudder_assist = bool(rudder_assist)
        msg.assist.request_id = req_id
        if not self._write_locked(msg.SerializeToString()):
            return False, 'serial write failed'
        self.p_assist_on = bool(p_on)
        self.assist_rudder_on = bool(rudder_assist)
        self.assist_request_id = req_id
        return True, None

    def send_assist(self, p_on):
        """Turn the temporary MOTOR P yaw assist on or off, at RUNTIME.

        Runtime and not a rebuild, so both arms of an A/B experiment run the
        same binary -- rebuilding between arms would let a build difference
        pass for a result. OFF returns the boat to exactly the pre-P path: the
        firmware drops the correction rather than letting it decay."""
        with self._lock:
            # P perturbs the motors. Changing it mid-run would put a different
            # controller under half the recording.
            if self._rudder_test_busy_locked():
                return False, 'a rudder test is running — wait for it to finish'
            if p_on and self.assist_rudder_on:
                return False, ('Assisted Steering is ON — the two assists are '
                               'mutually exclusive, switch it OFF first')
            return self._send_assist_locked(bool(p_on), self.assist_rudder_on)

    def send_rudder_assist(self, on):
        """Assisted Steering (the RUDDER yaw-rate loop) on or off, at RUNTIME.

        Default OFF after every boot: the firmware's boot mode is Raw Manual
        and nothing here persists a mode."""
        with self._lock:
            if self._rudder_test_busy_locked():
                return False, 'a rudder test is running — wait for it to finish'
            if on and self.p_assist_on:
                return False, ('motor P assist is ON — the two assists are '
                               'mutually exclusive, switch it OFF first')
            return self._send_assist_locked(self.p_assist_on if not on else False,
                                            bool(on))

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
            if self._rudder_test_busy_locked():
                return False, 'a rudder test is running — wait for it to finish'
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
            self.bench_requested_at = time.monotonic()
            return True, None

    # ---- automatic rudder test ------------------------------------------
    #
    # One shared non-blocking state machine, ticked from _stream_loop at the
    # existing 15 Hz under the existing lock. It only sets self.throttle and
    # self.rudder; the loop already sends those, so nothing here bypasses
    # STOP, DISARM, disconnect or the calibration gate.

    def start_rudder_test(self, sign, command_seq, assisted=False):
        """Begin the sequence. `sign` is -1 or +1 (the two buttons).

        Refuses rather than fixing anything up: it never arms the boat, never
        zeroes a throttle the operator is holding, and never turns P off. Each
        of those is a decision the operator has to make knowingly, and quietly
        doing it for them is how a "test" ends up measuring something else."""
        with self._lock:
            if command_seq <= self.winch_command_seq:
                return False, 'stale command sequence'
            self.winch_command_seq = command_seq
            if sign not in (-1, 1) or isinstance(sign, bool):
                return False, 'direction must be -1 or +1'
            if self.rudder_test is not None:
                return False, 'a rudder test is already running'
            if getattr(self, 'lake_id', None) is not None:
                return False, 'a lake steering test is running'
            if not self.connected:
                return False, 'serial link is disconnected'
            if self.calibrating:
                return False, 'calibration is running — stop it first'
            if self._bench_running_locked():
                return False, 'a bench run is going — wait for it to finish'
            # The sequence owns the throttle for the next 4.5 s. Taking it over
            # from a held stick would yank the boat to zero and then to 0.20,
            # which is not the test that gets recorded.
            if (self.throttle != 0.0 or self.motor_left != 0.0
                    or self.motor_right != 0.0):
                return False, 'set the throttle to zero first'
            if not self.armed_cmd:
                return False, 'ARM first — the rudder test spins the thrusters'
            ok, why = self._boat_armed_and_fresh_locked(self._now())
            if not ok:
                return False, why
            # P perturbs the motors mid-turn, which is exactly what this run
            # must not contain. Both the request AND the boat's own answer,
            # because the two can disagree and only the boat's is real.
            if self.p_assist_on:
                return False, 'switch P assist OFF first'
            if self.bench_status.get('have') and self.bench_status.get('p_on'):
                return False, 'the boat reports P assist ON — switch it OFF first'
            ms = self.motor_status
            if ms.get('have') and ms.get('assist_motor_p'):
                return False, ('the boat reports MOTOR P assist ON — the two '
                               'assists are mutually exclusive, switch it OFF')

            if assisted:
                # Turn the rudder loop ON before the sequence starts, so the
                # 0.5 s settle phase is already running the controller at
                # target zero rather than switching mode mid-run.
                ok, why = self._send_assist_locked(False, True)
                if not ok:
                    return False, why

            now = self._now()
            name, path = self._next_rudder_test_path(sign, assisted)
            # An assisted run's profile clock does not start until the boat has
            # confirmed the mode. Until then t0 is None, every phase lookup is
            # skipped, and the stick stays at zero.
            ack_deadline = (now + RUDDER_TEST_ASSIST_ACK_S) if assisted else None
            self.rudder_test = {
                'sign': int(sign),
                'assisted': bool(assisted),
                # Raw: a fixed deflection. Assisted: FULL stick, which the
                # firmware turns into +/- SAS_RMAX_DPS of yaw-rate target.
                'rudder': (float(sign) if assisted
                           else float(sign) * RUDDER_TEST_DEFLECTION),
                'target_dps': (-float(sign) * ASSIST_TEST_TARGET_DPS
                               if assisted else 0.0),
                't0': (None if assisted else now),
                'phase': ('awaiting_assist' if assisted
                          else RUDDER_TEST_PHASES[0][0]),
                'assist_sent_at': now if assisted else None,
                'assist_request_id': self.assist_request_id if assisted else 0,
                'rate_dps': 0.0,
                'ack_deadline': ack_deadline,
                'rows': [], 'name': name, 'path': str(path),
                'last_frame_mono': None, 'max_gap_s': 0.0,
                'drive_started': None, 'drive_confirmed': False,
            }
            self.rudder_test_result = None
            self.throttle = 0.0
            self.motor_split = False
            # RAW: command the deflection immediately rather than waiting up to
            # 67 ms for the next tick -- the settle phase is only 0.5 s and
            # holding the deflection IS what it is for.
            #
            # ASSISTED: the settle phase is the loop holding STRAIGHT before
            # the motors come on, so the stick starts at zero and the tick
            # raises it when the drive phase opens. Assigning full stick here
            # handed the controller a 2 deg/s target for the whole settle
            # phase, which is not the profile and not what gets recorded.
            self.rudder = 0.0 if assisted else self.rudder_test['rudder']
            return True, None

    def _rudder_test_busy_locked(self):
        """Server-side interlock, not a UI convenience.

        The page greys these controls out while a run is going, but disabled
        buttons are decoration: a stale tab, a second browser, a curl, or the
        keyboard shortcuts all reach the HTTP API directly. The refusal has to
        live here, where every one of those paths converges."""
        return self.rudder_test is not None or getattr(self, 'lake_id', None) is not None

    def _boat_armed_and_fresh_locked(self, now):
        """(ok, why). The BOAT's own confirmation, not what we commanded."""
        ms = self.motor_status
        if not ms.get('have'):
            return False, 'no MotorStatus from the boat yet — wait a moment'
        last = ms.get('last_rx_monotonic')
        if last is None or (now - last) > RUDDER_TEST_STATUS_MAX_AGE_S:
            return False, 'boat status is stale — not safe to drive blind'
        if int(ms.get('state', 0)) != 2:
            return False, 'the boat does not report itself armed'
        # The rudder is a SERVO. Without the rail powered it does not hold a
        # deflection, so a run without this is throttle applied to a rudder
        # that may be drifting anywhere -- worse than no test.
        if not ms.get('servo_power'):
            return False, 'the boat reports the servo rail is OFF'
        return True, None

    def _next_rudder_test_path(self, sign, assisted=False):
        """T20 plus the direction, first free index -- never overwrite.

        Raw and assisted runs get DIFFERENT prefixes on purpose: they are not
        the same experiment and must never be pooled by a glob."""
        pct = int(RUDDER_TEST_THROTTLE * 100 + 0.5)
        if assisted:
            # sign -1 = left stick = POSITIVE yaw target -> L2
            tag = ('L' if sign < 0 else 'R') + '%g' % ASSIST_TEST_TARGET_DPS
            base = 'ASST_T%02u_%s' % (pct, tag)
        else:
            tag = ('N' if sign < 0 else 'P') + '%02u' % RUDDER_TEST_PCT
            base = 'RUD_T%02u_%s' % (pct, tag)
        directory = Path(getattr(self, 'rudder_test_dir', RUDDER_TEST_DIR))
        for i in range(1, 1000):
            name = '%s_%02u.csv' % (base, i)
            path = directory / name
            if not path.exists():
                return name, path
        return base + '_overflow.csv', directory / (base + '_overflow.csv')

    def _collect_rudder_test_row_locked(self, yaw_rate, heading, now=None):
        """One row per RECEIVED TELEMETRY FRAME. Called from the decode paths,
        never from status(): rows keyed to UI polls would multiply with the
        browser's refresh rate and make the integrated angle meaningless."""
        rt = self.rudder_test
        if rt is None:
            return
        now = self._now() if now is None else now
        if rt['t0'] is None:
            # Still waiting for the boat to confirm Assisted mode. Nothing is
            # being commanded yet, so there is no run to record against.
            return
        elapsed = now - rt['t0']
        phase = rudder_test_phase_at(elapsed)
        gap = 0.0 if rt['last_frame_mono'] is None else (now - rt['last_frame_mono'])
        if rt['last_frame_mono'] is not None and gap > rt['max_gap_s']:
            rt['max_gap_s'] = gap
        rt['last_frame_mono'] = now
        ms = self.motor_status
        tel_last = self.telemetry.get('last_rx_monotonic')
        rt['rows'].append({
            't_mono': round(now, 4),
            'elapsed_s': round(elapsed, 4),
            'phase': phase[0] if phase else 'done',
            'yaw_dps': yaw_rate,
            'heading_deg': heading,
            'cmd_throttle': self.throttle,
            'cmd_rudder': self.rudder,
            'boat_left': ms.get('left_throttle') if ms.get('have') else '',
            'boat_right': ms.get('right_throttle') if ms.get('have') else '',
            # MotorStatus carries state/throttles/winch/servo_power and NO
            # rudder position or pulse. The column exists so the schema is
            # stable if the firmware ever adds one -- filling it with the
            # COMMAND would pass an intention off as a measurement.
            'servo_us': '',
            'telem_age_s': round(now - tel_last, 4) if tel_last else '',
            'gap_s': round(gap, 4),
            'mode': 'assisted' if rt.get('assisted') else 'raw',
            # What we ASKED the loop for this instant -- zero outside the drive
            # phase, matching what the tick actually commands.
            # What the tick last put on the wire, not what the phase implies.
            'target_dps': rt.get('rate_dps', 0.0),
            'yaw_filt_dps': ms.get('yaw_filt_dps', '') if ms.get('have') else '',
            'boat_rudder_cmd': ms.get('rudder_cmd', '') if ms.get('have') else '',
            'boat_rudder_us': ms.get('rudder_pulse_us', '') if ms.get('have') else '',
            'boat_saturated': (1 if ms.get('rudder_saturated') else 0)
                              if ms.get('have') else '',
            'boat_assist_on': (1 if ms.get('assist_rudder') else 0)
                              if ms.get('have') else '',
            # Both targets, side by side. `target_dps` is what this laptop
            # asked for; this is what the boat says its loop is actually
            # holding. They should agree -- and when they do not, that IS the
            # finding, so neither may stand in for the other.
            'boat_target_dps': ms.get('yaw_target_dps', '') if ms.get('have') else '',
            'boat_state': ms.get('state', '') if ms.get('have') else '',
            'boat_servo_power': (1 if ms.get('servo_power') else 0)
                                if ms.get('have') else '',
        })

    def _rudder_test_tick_locked(self, now):
        """Advance the sequence. Caller holds the lock. Never blocks."""
        rt = self.rudder_test
        if rt is None:
            return
        try:
            if not self.connected:
                return self._abort_rudder_test_locked('serial link disconnected')
            if not self.armed_cmd:
                return self._abort_rudder_test_locked('disarmed')
            if self.calibrating:
                return self._abort_rudder_test_locked('calibration started')
            ok, why = self._boat_armed_and_fresh_locked(now)
            if not ok:
                return self._abort_rudder_test_locked(why)

            # ---- Assisted mode must be CONFIRMED before the profile runs ----
            #
            # In Assisted mode the stick is a yaw-RATE demand: -1.0 means
            # +2 deg/s. If the boat is NOT in that mode, the identical -1.0 is
            # a RAW steering command -- full rudder, hard over. So the stick
            # may never leave zero on the word of what we asked for; only the
            # boat's own assist_rudder makes it safe.
            if rt.get('assisted'):
                ms = self.motor_status
                # Correlated by ID, never by arrival time. last_rx_monotonic
                # records when THIS process handled a packet, so a stale
                # assist_rudder=true processed after the request satisfies a
                # time comparison while describing a mode the boat has since
                # left -- reproduced with acknowledgements disabled, where it
                # started the profile and put -1.0 on the wire.
                #
                # The boat echoes AssistCommand.request_id only once its
                # control task has applied the change, so a matching id is
                # proof about this request and nothing else.
                confirmed = (ms.get('have') and ms.get('assist_rudder')
                             and rt['assist_request_id'] != 0
                             and ms.get('assist_request_id')
                                 == rt['assist_request_id'])
                if rt['t0'] is None:
                    if confirmed:
                        rt['t0'] = now          # profile starts here, not before
                        rt['phase'] = RUDDER_TEST_PHASES[0][0]
                    else:
                        # Held at zero throughout the wait -- see above.
                        self.throttle = 0.0
                        self.motor_split = False
                        self.rudder = 0.0
                        if now >= rt['ack_deadline']:
                            return self._abort_rudder_test_locked(
                                'boat never confirmed Assisted Steering '
                                '(waited %.1f s) — not sending a rate demand '
                                'the boat would read as full rudder'
                                % RUDDER_TEST_ASSIST_ACK_S)
                        return
                elif not confirmed:
                    # It dropped mid-run. Everything we are commanding would be
                    # reinterpreted as raw steering, so stop now.
                    return self._abort_rudder_test_locked(
                        'boat dropped out of Assisted Steering mid-run')

            phase = rudder_test_phase_at(now - rt['t0'])
            if phase is None:
                return self._finish_rudder_test_locked(aborted=False, reason=None)

            # Is the boat DOING what the drive phase is asking? Commanding T20
            # and getting zero back means the boat is refusing -- disarmed, or
            # the firmware's safe_stop branch, which also pins the steering
            # command to zero and so silently flattens an assisted target too.
            # Both assisted runs on 2026-09-02 recorded 2 s of exactly that and
            # only failed later, on a staleness timeout that named the wrong
            # cause. Catch it directly and say so.
            if phase[0] == 'drive' and phase[1] > 0.0:
                ms = self.motor_status
                # ONLY judge this on a FRESH status. A frozen one says nothing
                # about what the boat is doing -- and saying "not driving,
                # check ARM" from a stale packet is worse than saying nothing,
                # because it sends the operator after the wrong fault. On
                # 2026-09-02 the boat drove correctly while MotorStatus had
                # stopped arriving; this check would have blamed the arm state.
                if rt.get('drive_started') is None:
                    rt['drive_started'] = now
                last = ms.get('last_rx_monotonic')
                driving = ms.get('have') and (
                    abs(ms.get('left_throttle', 0.0)) > 0.01
                    or abs(ms.get('right_throttle', 0.0)) > 0.01)
                if driving:
                    rt['drive_confirmed'] = True
                elif not rt.get('drive_confirmed'):
                    # Judge ONLY on a status that arrived after the boat had
                    # time to spin up. A status from before that -- or a frozen
                    # one -- says nothing about whether it is driving NOW, and
                    # blaming the arm state from it sends the operator after
                    # the wrong fault. On 2026-09-02 the boat drove correctly
                    # while MotorStatus had stopped arriving 0.16 s into the
                    # drive phase; this rule leaves that case to the staleness
                    # gate, which names itself honestly.
                    informed = (last is not None and
                                last >= rt['drive_started'] + RUDDER_TEST_DRIVE_CONFIRM_S)
                    if informed:
                        return self._abort_rudder_test_locked(
                            'boat is not driving — commanded %.2f, boat reports '
                            '%.2f/%.2f (check ARM)'
                            % (phase[1], ms.get('left_throttle', 0.0),
                               ms.get('right_throttle', 0.0)))
            rt['phase'] = phase[0]
            self.throttle = phase[1]
            self.motor_split = False
            if rt.get('assisted'):
                # Target yaw ZERO outside the drive phase: the settle phase is
                # the loop holding straight before the motors come on, and the
                # coast must not keep asking for a turn the jets can no longer
                # produce -- that would wind the rudder over with no authority.
                # THE RAW STICK STAYS AT ZERO FOR THE WHOLE ASSISTED RUN.
                #
                # The demand goes out as a SteerRateCommand -- deg/s, its own
                # message, routed by the firmware to the yaw-rate loop and
                # nowhere else. So there is no longer a value in flight that
                # could be reinterpreted as a rudder angle: lose the mode
                # mid-run, drop a packet, get the ordering wrong, and the worst
                # case is a centred rudder rather than one hard over.
                #
                # This replaces the old scheme, where the same SteerCommand
                # value meant "+2 deg/s" with the loop on and "full left
                # rudder" with it off.
                self.rudder = 0.0
                rt['rate_dps'] = (rt['target_dps']
                                  if (phase[0] == 'drive' and confirmed) else 0.0)
            else:
                self.rudder = rt['rudder']
            # The stream loop sends these straight after; a write failure there
            # is caught by _stream_loop and routed back here as an abort.
        except Exception as exc:                             # noqa: BLE001
            self._abort_rudder_test_locked(
                '%s: %s' % (type(exc).__name__, exc))

    def _rudder_test_after_send_locked(self, sent_ok):
        """Called by the stream loop with the result of this tick's writes.

        A rudder test that cannot reach the boat is not a test -- and worse,
        the boat is holding a deflection nobody can now change. End it rather
        than keep recording rows against commands that never landed."""
        if not sent_ok:
            self._abort_rudder_test_locked('serial write failed')

    def _abort_rudder_test_locked(self, reason):
        # The lake steering test shares every interlock and abort route of the
        # rudder test (STOP, manual input, disarm, disconnect, servo power,
        # calibration, a failed write) by going through this one function.
        if getattr(self, 'lake_id', None) is not None:
            self._abort_lake_id_locked(reason)
        if self.rudder_test is None:
            return
        self._finish_rudder_test_locked(aborted=True, reason=reason)

    def _finish_rudder_test_locked(self, aborted, reason):
        """Stop the boat, centre the rudder, hand the CSV off to be written.

        The write happens OUTSIDE the lock (see _flush_rudder_test_write) so a
        few milliseconds of file I/O never sits inside the 15 Hz command loop."""
        rt = self.rudder_test
        if rt is None:
            return
        self.rudder_test = None
        # ORDER MATTERS. Zero the commands, PUSH them, and only then leave
        # Assisted mode.
        #
        # In Assisted mode self.rudder is a yaw-RATE demand: +/-1.0 means
        # "+/-2 deg/s", and the firmware's loop turns it into a few tenths of
        # rudder. The instant the loop is switched off that same +/-1.0 is read
        # as a RAW steering command -- full rudder, hard over. Disabling first
        # and zeroing after leaves a window of exactly that, one stream-loop
        # tick wide, on every abort.
        self.throttle = 0.0
        self.motor_left = 0.0
        self.motor_right = 0.0
        self.motor_split = False
        self.rudder = 0.0
        rt['rate_dps'] = 0.0
        if self.connected:
            self._send_motor_locked(0.0, 0.0)
            self._send_steer_locked(0.0)
            if rt.get('assisted'):
                self._send_steer_rate_locked(0.0)
        # Now it is safe: whatever mode the boat is in, it has been told zero.
        # Switched off on EVERY exit including every abort -- leaving the loop
        # live would have it quietly steering during ordinary manual driving.
        if rt.get('assisted') and self.connected:
            self._send_assist_locked(False, False)
        rows = rt['rows']
        span = (rows[-1]['t_mono'] - rows[0]['t_mono']) if len(rows) >= 2 else 0.0
        drive = rudder_test_drive_coverage(rows)
        result = {
            'name': rt['name'], 'path': rt['path'], 'sign': rt['sign'],
            'pct': RUDDER_TEST_PCT,
            # t0 is None when the run never got past the Assisted-mode
            # confirmation wait -- there is no profile to have a duration.
            # Computing it anyway raised inside the finish path, which the
            # tick's own except-handler then swallowed by re-aborting an
            # already-cleared run: the abort left NO result at all.
            'frames': len(rows),
            'duration_s': (round(self._now() - rt['t0'], 3)
                           if rt['t0'] is not None else 0.0),
            'span_s': round(span, 3), 'max_gap_s': round(rt['max_gap_s'], 3),
            # No frames at all is the worst gap there is, so say so rather than
            # reporting a tidy 0.0 for a recording that captured nothing.
            'gap': (not rows) or rt['max_gap_s'] > RUDDER_TEST_MAX_GAP_S,
            'aborted': bool(aborted),
            'abort_reason': reason if aborted else None,
        }
        result.update(drive)
        # An aborted run cut the drive phase short, so it can never be
        # complete however good the frames it did catch look.
        result['incomplete'] = bool(aborted) or drive['drive_incomplete']
        self.rudder_test_result = result
        self._rudder_test_write = (rt, result)

    # ---- BASE/bench run recording ---------------------------------------

    def _bench_csv_path(self, kind, base):
        """BASE / LEFT / RIGHT get DIFFERENT prefixes, for the same reason raw
        and assisted rudder runs do: they are not the same experiment and must
        never be pooled by a glob."""
        name = BENCH_KIND_NAME.get(int(kind), 'BENCH%d' % int(kind))
        stem = '%s_T%02u' % (name, int(base * 100 + 0.5))
        directory = Path(getattr(self, 'bench_dir', RUDDER_TEST_DIR))
        for i in range(1, 1000):
            path = directory / ('%s_%02u.csv' % (stem, i))
            if not path.exists():
                return path
        return directory / (stem + '_overflow.csv')

    def _queue_bench_write_locked(self, aborted, end_state, file_index):
        """Hand the finished run to the stream loop to write. The write itself
        happens outside the lock, like the rudder test's, so a slow or full
        filesystem cannot stall the command stream."""
        run = self.bench_run
        self.bench_run = None
        if run is None or not run['rows']:
            return              # a run that never drove has nothing to record
        self._bench_write = (run, {
            'aborted': bool(aborted),
            'end_state': int(end_state),
            'file_index': int(file_index),
            'summary': self.bench_yaw,
            'path': self._bench_csv_path(run['kind'], run['base']),
        })

    def _flush_bench_write(self):
        """Called from _stream_loop with the lock NOT held, and by tests."""
        pending = self._bench_write
        if pending is None:
            return
        self._bench_write = None
        run, result = pending
        s = result['summary'] or {}
        controller = yaw_controller_summary(run['rows'])
        head = [
            # Provenance first. It must be impossible to mistake this for the
            # boat's own 100 Hz SD recording of the same run.
            'laptop/radio-observed BASE run -- recorded by '
            'tools/espnow_drive.py from ESP-NOW telemetry frames, '
            'NOT the boat SD card',
            'kind=%s throttle=%.2f boat_sd_file_index=%d  '
            '(that index is the authoritative file on the boat)'
            % (BENCH_KIND_NAME.get(run['kind'], run['kind']), run['base'],
               result['file_index']),
            'frames=%d mean_yaw_dps=%s peak_yaw_dps=%s span_s=%s max_gap_s=%s'
            % (len(run['rows']), _r3(s.get('mean_yaw_dps')),
               _r3(s.get('peak_yaw_dps')), _r3(s.get('span_s')),
               _r3(s.get('max_gap_s'))),
            'heading_change_deg=%s active_fraction=%s saturated_fraction=%s '
            'peak_abs_p=%s peak_abs_i=%s final_i=%s'
            % (_r3(s.get('heading_change_deg')),
               _r3(controller.get('active_fraction')),
               _r3(controller.get('saturated_fraction')),
               _r3(controller.get('peak_abs_p')),
               _r3(controller.get('peak_abs_i')),
               _r3(controller.get('final_i'))),
            'controller signs: positive yaw is LEFT; positive heading error means target '
            'is clockwise/right of current; p_term, i_term, '
            'dynamic_c, effective_c and c_limit are motor-command fractions',
        ]
        if result['aborted']:
            # A tidy-looking summary of a run that never happened is the worst
            # possible output, so this is loud and first-class.
            head.append('ABORTED end_state=%s (%s) -- the boat cut this run '
                        'short and saved NOTHING to SD; these frames describe '
                        'part of a turn with no file to check them against'
                        % (result['end_state'],
                           BENCH_STATE_NAME.get(result['end_state'], '?')))
        err = self._write_csv(result['path'], head, BENCH_CSV_COLUMNS,
                              run['rows'], 'bench')
        self.bench_csv_name = (('write failed: ' + err) if err
                               else Path(result['path']).name)

    def _write_csv(self, path, header_lines, columns, rows, label):
        """Shared by the rudder-test and BASE-run writers: create the
        directory, write '# ' provenance lines, then the rows -- and never let
        a filesystem error take down the control stream."""
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w', newline='') as fh:
                for line in header_lines:
                    fh.write('# %s\n' % line)
                w = csv.DictWriter(fh, fieldnames=list(columns))
                w.writeheader()
                for row in rows:
                    w.writerow(row)
            return None
        except OSError as exc:
            print('[%s] could not write %s: %s' % (label, path, exc),
                  file=sys.stderr, flush=True)
            return '%s: %s' % (type(exc).__name__, exc)

    # ---- lake steering-identification test --------------------------------

    def _laptop_test_active_locked(self):
        return self.rudder_test is not None or getattr(self, 'lake_id', None) is not None

    def lake_id_next(self):
        """Per-condition next order/index. READS THE CACHE ONLY: status() runs
        under the lock ten times a second and must never touch the disk. The
        cache is filled outside the lock by ensure_lake_id_next() (the status
        handler's pre-step), start_lake_id() and the finalize callback."""
        cache = getattr(self, '_lake_id_next_cache', None)
        return dict(cache) if cache else {}

    def ensure_lake_id_next(self):
        """Fill the cache if empty. Disk I/O; call with the lock NOT held."""
        if getattr(self, '_lake_id_next_cache', None) is None:
            self._lake_id_refresh_next()

    def _lake_id_refresh_next(self):
        directory = getattr(self, 'lake_id_dir', LAKE_ID_DIR)
        cache = dict(lake_id_scan(directory))
        cache.update(straight_run_scan(directory))    # distinct keys: STRAIGHT_T20 vs T20_M30
        self._lake_id_next_cache = cache

    def _lake_id_refusal_locked(self, command_seq, profile='full'):
        """Every reason a START is refused, in one place, because it is asked
        TWICE: before the recorder is prepared (a hopeless press never touches
        the disk) and again after (nothing that changed while the lock was
        released can slip through). Claims nothing."""
        if command_seq <= self.winch_command_seq:
            return 'stale command sequence'
        if self.lake_id is not None:
            return 'a lake steering test is already running'
        if self.rudder_test is not None:
            return 'a rudder test is running'
        if not self.connected:
            return 'serial link is disconnected'
        if self.calibrating:
            return 'calibration is running — stop it first'
        if self._bench_running_locked():
            return 'a bench run is going — wait for it to finish'
        if (self.throttle != 0.0 or self.motor_left != 0.0
                or self.motor_right != 0.0 or self.rudder != 0.0):
            return 'set the throttle and rudder to zero first'
        if not self.armed_cmd:
            return 'ARM first — the lake test spins the thrusters'
        if self.session_id is None:
            return 'no browser control session — reload the page'
        if lake_id_is_straight_profile(profile):
            # Motor P ON or OFF is valid for a BASE comparison, but the state
            # must be the one the BOAT acknowledged.  A successful serial
            # write only proves the request left this process.
            if self._assist_off_req_id is not None:
                want = 'ON' if getattr(self, '_assist_pending_p_on', False) else 'OFF'
                return 'wait for the boat to confirm Motor P %s before BASE' % want
        elif getattr(self, 'raw_throttle_test', False):
            if self.p_assist_on:
                return 'Motor P must be OFF for the raw throttle test'
        elif not self.p_assist_on:
            return 'Motor P must be ON — use the P toggle first, then START'
        if self.assist_rudder_on or self._assist_off_pending_locked():
            return 'Rudder Assist must be OFF, confirmed by the boat — turn it off first'
        return None

    def start_lake_id(self, throttle, magnitude, command_seq, notes=None,
                      firmware_label=None, profile='full'):
        """One press. Refusals are the server's, not the page's. Returns
        (ok, error).

        validate -> prepare OUTSIDE the lock -> revalidate -> install. The
        recorder's mkdir/open/write/flush must never sit inside the lock the
        15 Hz stream and STOP are waiting for, and a start that is refused
        after preparing cleans up only what it created."""
        if profile not in LAKE_ID_PROFILES:
            return False, 'profile must be one of %s' % (LAKE_ID_PROFILES,)
        try:
            throttle = float(throttle); magnitude = float(magnitude)
        except (TypeError, ValueError):
            return False, 'throttle and magnitude must be numbers'
        if lake_id_is_straight_profile(profile):
            # The bench card's throttle box, like the BASE tests this stands in
            # for: any whole percent in STRAIGHT_THROTTLE_MIN..MAX, refused
            # outside it and never clamped -- a run starts from the value the
            # operator chose or not at all.
            if not (math.isfinite(throttle)
                    and STRAIGHT_THROTTLE_MIN - 1e-9 <= throttle <= STRAIGHT_THROTTLE_MAX + 1e-9):
                return False, ('throttle must be between %d%% and %d%% for a straight run'
                               % (round(STRAIGHT_THROTTLE_MIN * 100), round(STRAIGHT_THROTTLE_MAX * 100)))
            throttle = round(throttle, 2)
            magnitude = 0.0               # no turns: the rudder stays centred throughout
        elif profile == 'yawpulse':
            if not (math.isfinite(throttle) and 0.15 <= throttle <= 0.50):
                return False, 'yaw-pulse throttle must be between 15% and 50%'
            if not (math.isfinite(magnitude) and 0.05 <= magnitude <= 0.20
                    and magnitude <= throttle):
                return False, 'yaw-pulse motor half-difference must be 5% to 20% and not exceed throttle'
            throttle, magnitude = round(throttle, 2), round(magnitude, 2)
        else:
            if not any(abs(throttle - t) < 1e-9 for t in LAKE_ID_THROTTLES):
                return False, 'throttle must be one of %s' % (LAKE_ID_THROTTLES,)
            throttle = min(LAKE_ID_THROTTLES, key=lambda t: abs(t - throttle))
            if not any(abs(magnitude - m) < 1e-9 for m in LAKE_ID_MAGNITUDES):
                return False, 'rudder magnitude must be one of %s' % (LAKE_ID_MAGNITUDES,)
            magnitude = min(LAKE_ID_MAGNITUDES, key=lambda m: abs(m - magnitude))
        notes = {k: str((notes or {}).get(k, '') or '')[:500] for k in LAKE_ID_NOTE_FIELDS}
        label = (firmware_label if isinstance(firmware_label, str) and firmware_label.strip()
                 else ('raw-throttle-test' if getattr(self, 'raw_throttle_test', False)
                       else LAKE_ID_FIRMWARE_LABEL_DEFAULT))
        # ---- pass 1: validate under the lock; claims nothing ---------------
        with self._lock:
            why = self._lake_id_refusal_locked(command_seq, profile)
        if why:
            return False, why
        # ---- disk and git work with the lock RELEASED ----------------------
        provenance = lake_id_provenance(__file__)
        base_dir = Path(getattr(self, 'lake_id_dir', LAKE_ID_DIR))
        scan = dict(lake_id_scan(base_dir))
        scan.update(straight_run_scan(base_dir))
        self._lake_id_next_cache = scan
        if lake_id_is_straight_profile(profile):
            cond = straight_run_condition(throttle)
            order, index = 'NA', scan.get(cond, {'next_index': 1})['next_index']
            name = '%s_%03d' % (cond, index)
        elif profile == 'yawpulse':
            cond = 'YAW_T%02d_D%02d' % (round(throttle * 100), round(magnitude * 100))
            index = 1 + len(list(base_dir.glob(cond + '_*_???')))
            order = 'LR' if index % 2 else 'RL'
            name = '%s_%s_%03d' % (cond, order, index)
        else:
            cond = lake_id_condition(throttle, magnitude)
            order, index = scan[cond]['next_order'], scan[cond]['next_index']
            name = 'LAKE_ID_%s_%s_%03d' % (cond, order, index)
        directory = base_dir / name
        writer = LakeIdWriter(directory)
        try:
            writer.prepare()                  # mkdir(exist_ok=False): unique or refused
        except Exception as exc:              # noqa: BLE001  (cleaned up its own files)
            return False, 'cannot create the run folder: %s' % exc
        # ---- pass 2: revalidate, then install -------------------------------
        with self._lock:
            why = self._lake_id_refusal_locked(command_seq, profile)
            if why is None:
                try:
                    writer.start()
                except Exception as exc:      # noqa: BLE001
                    why = 'cannot start the recorder: %s' % exc
            if why is None:
                self.winch_command_seq = command_seq
                now = self._now()
                phases = lake_id_phases(throttle, magnitude, order, profile)
                self.lake_id = {
                    't0': now, 'throttle': throttle, 'magnitude': magnitude,
                    'profile': profile, 'profile_s': lake_id_profile_s(phases),
                    'powered_s': lake_id_powered_s(phases), 'p_at_start': None,
                    'raw_throttle_test': getattr(self, 'raw_throttle_test', False),
                    'order': order, 'condition': cond, 'index': index, 'name': name,
                    'dir': str(directory),
                    'phases': phases,
                    'phase': 'precheck', 'phase_start': now,
                    'precheck': {}, 'precheck_met_at': {},
                    'rows': [], 'events': 0, 'frames_received': 0,
                    'last_frame_mono': None, 'max_gap_s': 0.0,
                    'last_gps': None, 'cmd_last': None,
                    'drive_started': None, 'drive_confirmed': False,
                    'ms_seen_rx': None, 'zero_since': None,
                    'stop_entered_at': None, 'stop_confirmed': None, 'teardown_logged': False,
                    'warnings': [], 'warning_counts': {}, 'finalizing': False,
                    'status_lossy': False, 'failsafe_logged': False,
                    'notes': notes, 'firmware_label': label, 'provenance': provenance,
                    'imu_nonfinite': False, 'imu_last': None,
                }
                self._lake_writer = writer
                self.lake_id_result = None
                self._lake_id_event_locked('button_pressed',
                                           'profile=%s throttle=%.2f magnitude=%.2f order=%s'
                                           % (profile, throttle, magnitude, order))
                self._lake_id_event_locked('phase_enter', 'precheck: zeros commanded, waiting up to %.1f s'
                                           % LAKE_ID_PRECHECK_S)
                self.throttle = 0.0; self.motor_left = 0.0; self.motor_right = 0.0
                self.motor_split = False; self.rudder = 0.0
                return True, None
        writer.discard()                      # disk work: outside the lock again
        return False, why

    def _lake_id_event_locked(self, event, detail=''):
        rt = self.lake_id
        if rt is None:
            return
        now = self._now()
        rt['events'] += 1
        self._lake_writer.put('event', {
            't_utc': lake_id_utc(), 't_mono': round(now, 4),
            'elapsed_s': round(now - rt['t0'], 4), 'phase': rt['phase'],
            'event': event, 'detail': detail})

    def _lake_id_warn_locked(self, text):
        """Live warning, deduplicated by text, visible on the card and in the
        events. A repeat only bumps its count: a condition that holds for a
        whole phase must not become one event per frame."""
        rt = self.lake_id
        if rt is None:
            return
        rt['warning_counts'][text] = rt['warning_counts'].get(text, 0) + 1
        if text in rt['warnings']:
            return
        rt['warnings'].append(text)
        self._lake_id_event_locked('warning', text)

    def _lake_id_motor_p_gate_locked(self, ms):
        """The Motor P gate condition, by profile. Full profile: P must be ON
        (OFF for the raw-throttle build), both as requested and as the boat
        reports it. Straight run: either state, but the boat's report must
        match the switch -- what gets recorded must be what actually ran."""
        rt = getattr(self, 'lake_id', None)
        if rt is not None and lake_id_is_straight_profile(rt.get('profile')):
            return {'motor_p_consistent': (bool(ms.get('have')) and
                                           bool(ms.get('assist_motor_p')) == bool(self.p_assist_on))}
        want_on = not getattr(self, 'raw_throttle_test', False)
        return {('motor_p_on' if want_on else 'motor_p_off'):
                (bool(ms.get('have')) and bool(self.p_assist_on) == want_on and
                 bool(ms.get('assist_motor_p')) == want_on)}

    def _lake_id_precheck_locked(self, now):
        """Every required condition, as a dict of name -> met. Evaluated every
        tick of the 2 s window; only the state at t=2 s decides."""
        ms, tel, ss = self.motor_status, self.telemetry, self.system_status
        ms_age = (now - ms['last_rx_monotonic']) if (ms.get('have') and ms.get('last_rx_monotonic') is not None) else None
        tel_age = (now - tel['last_rx_monotonic']) if (tel.get('have') and tel.get('last_rx_monotonic') is not None) else None
        ss_age = (now - ss['last_rx_monotonic']) if (ss.get('have') and ss.get('last_rx_monotonic') is not None) else None
        finite = all(isinstance(tel.get(k), (int, float)) and math.isfinite(tel.get(k))
                     for k in ('yaw_rate', 'heading', 'pitch', 'roll'))
        pwm = ms.get('rudder_pulse_us')
        return {
            'link_connected': bool(self.connected),
            'armed_cmd': bool(self.armed_cmd),
            'browser_supervision': (self.session_id is not None
                                    and (now - self.session_last_hb) <= LAKE_ID_SUPERVISION_S),
            'telemetry_fresh': tel_age is not None and tel_age <= LAKE_ID_TELEM_MAX_AGE_S,
            'imu_finite': bool(tel.get('have')) and finite,
            'motorstatus_fresh': ms_age is not None and ms_age <= LAKE_ID_MOTORSTATUS_MAX_AGE_S,
            'boat_armed': int(ms.get('state', 0) or 0) == 2,
            'boat_zero_throttle': (ms.get('have') and ms.get('left_throttle') == 0.0
                                   and ms.get('right_throttle') == 0.0),
            'boat_rudder_centred': (ms.get('have') and ms.get('rudder_cmd') == 0.0
                                    and isinstance(pwm, (int, float))
                                    and abs(pwm - LAKE_ID_RUDDER_NEUTRAL_US) <= LAKE_ID_RUDDER_CENTRE_TOL_US),
            # Both the request AND the boat's own answer, in a fresh MotorStatus:
            # the two can disagree and only the boat's is real.
            **self._lake_id_motor_p_gate_locked(ms),
            'rudder_assist_off': (not ms.get('assist_rudder') and not self._assist_off_pending_locked()),
            'systemstatus_fresh': ss_age is not None and ss_age <= LAKE_ID_SYSTEMSTATUS_MAX_AGE_S,
            'imu_ok': bool(ss.get('imu_ok')),
            'no_other_test': (not self.calibrating and not self._bench_running_locked()
                              and self.rudder_test is None),
            'local_zero': self.throttle == 0.0 and self.rudder == 0.0,
            'recording_ok': self._lake_writer.error is None,
        }

    def _lake_id_powered_abort_locked(self, now, rt, ph):
        """The abort table for a powered or stopping phase. Returns a reason
        or None. NO yaw-magnitude limit: unexpected yaw is a warning and the
        operator owns STOP."""
        ms, tel, ss = self.motor_status, self.telemetry, self.system_status
        if not self.connected:
            return 'serial link disconnected'
        if not self.armed_cmd:
            return 'disarmed'
        if self.calibrating:
            return 'calibration started'
        if self._lake_writer.error:
            return 'recording failed: %s' % self._lake_writer.error
        if rt['imu_nonfinite']:
            return 'non-finite IMU value'
        # The boat's OWN link-loss failsafe (no command heard for 0.4 s): motors
        # zeroed, servo rail cut, not latched. It shows up here as the rail
        # reported OFF with the boat still armed. On the record once per stretch.
        rail_off = bool(ms.get('have')) and not ms.get('servo_power')
        if rail_off and not rt['failsafe_logged']:
            rt['failsafe_logged'] = True
            self._lake_id_event_locked('boat_failsafe', 'boat cut its servo rail: it stopped hearing '
                                       'the laptop for 0.4 s (its link-loss failsafe); L=%s R=%s'
                                       % (ms.get('left_throttle'), ms.get('right_throttle')))
            self._lake_id_warn_locked('boat link-loss failsafe tripped (it stopped hearing the laptop '
                                      'for 0.4 s: motors zeroed, servo rail cut)')
        elif not rail_off:
            rt['failsafe_logged'] = False
        if ph is not None and ph[0] == 'stop':
            # Zeros are already commanded and the STOP confirmation owns what
            # happens next. A lossy link, the boat's failsafe, a lost mode flag
            # or a quiet browser cannot make the boat less safe now, and each
            # would only throw away a finished profile seconds before its record.
            return None
        tel_age = (now - tel['last_rx_monotonic']) if tel.get('last_rx_monotonic') is not None else None
        if tel_age is not None and tel_age > LAKE_ID_TELEM_MAX_AGE_S:
            self._lake_id_warn_locked('telemetry gap exceeded %.1f s; affected samples are incomplete'
                                      % LAKE_ID_TELEM_MAX_AGE_S)
        if tel_age is None or tel_age > LAKE_ID_TELEM_POWERED_MAX_AGE_S:
            return 'telemetry stale (%.2f s)' % (tel_age if tel_age is not None else -1)
        alive = tel_age <= LAKE_ID_TELEM_ALIVE_S
        ms_limit = LAKE_ID_STATUS_ALIVE_CAP_S if alive else LAKE_ID_MOTORSTATUS_POWERED_MAX_AGE_S
        ss_limit = LAKE_ID_STATUS_ALIVE_CAP_S if alive else LAKE_ID_SYSTEMSTATUS_POWERED_MAX_AGE_S
        ms_age = (now - ms['last_rx_monotonic']) if ms.get('last_rx_monotonic') is not None else None
        if ms_age is None or ms_age > ms_limit:
            return 'MotorStatus stale (%.2f s)' % (ms_age if ms_age is not None else -1)
        ss_age = (now - ss['last_rx_monotonic']) if ss.get('last_rx_monotonic') is not None else None
        if ss_age is None or ss_age > ss_limit:
            return 'SystemStatus stale (%.2f s)' % (ss_age if ss_age is not None else -1)
        # On the record: a status stream older than its normal limit, kept
        # alive by telemetry. One event per stretch, not per tick.
        lossy = (ms_age > LAKE_ID_MOTORSTATUS_POWERED_MAX_AGE_S
                 or ss_age > LAKE_ID_SYSTEMSTATUS_POWERED_MAX_AGE_S)
        if lossy and not rt['status_lossy']:
            rt['status_lossy'] = True
            self._lake_id_event_locked('status_lossy', 'MotorStatus %.2f s / SystemStatus %.2f s old, '
                                       'telemetry %.2f s: boat alive, status packets being lost'
                                       % (ms_age, ss_age, tel_age))
        elif not lossy and rt['status_lossy']:
            rt['status_lossy'] = False
            self._lake_id_event_locked('status_recovered', 'MotorStatus %.2f s / SystemStatus %.2f s old'
                                       % (ms_age, ss_age))
        if not ss.get('imu_ok'):
            return 'IMU unhealthy (imu_ok=false)'
        if int(ms.get('state', 0) or 0) != 2:
            return 'boat not armed (state %s)' % ms.get('state')
        if ms.get('assist_rudder') or self.assist_rudder_on:
            return 'mode changed: Rudder Assist reported ON (must stay OFF)'
        if lake_id_is_straight_profile(rt.get('profile')):
            want = rt.get('p_at_start')
            if want is not None and (bool(ms.get('assist_motor_p')) != want
                                     or bool(self.p_assist_on) != want):
                return ('mode changed: Motor P flipped mid-run (was %s at start)'
                        % ('ON' if want else 'OFF'))
        elif getattr(self, 'raw_throttle_test', False):
            if ms.get('assist_motor_p') or self.p_assist_on:
                return 'mode changed: Motor P reported ON (must stay OFF for raw test)'
        elif not ms.get('assist_motor_p') or not self.p_assist_on:
            return 'mode changed: Motor P reported OFF (must stay ON)'
        # The server-side 15 Hz profile owns the controls once started. A
        # browser reload must not destroy a valid run; STOP and DISARM remain
        # available, and the boat still has its independent 400 ms watchdog.
        if ph is not None and ph[2] > 0.0:
            # Boat-applied L/R are watched for the WHOLE powered run, not just
            # until the first acknowledgement: the boat's own link-loss failsafe
            # zeroes the motors while its telemetry keeps flowing, and a run
            # that carried on would record a believable dataset of nothing.
            # Judged on NEWLY RECEIVED MotorStatus only, by packet receive
            # times: one old zero packet and the wall clock prove nothing.
            if rt['drive_started'] is None:
                rt['drive_started'] = now             # the powered command goes out this tick
            rx = ms.get('last_rx_monotonic') if ms.get('have') else None
            if rx is not None and rx != rt['ms_seen_rx']:
                rt['ms_seen_rx'] = rx
                if rx > rt['drive_started']:          # packets from before the command say nothing
                    left = ms.get('left_throttle', 0.0) or 0.0
                    right = ms.get('right_throttle', 0.0) or 0.0
                    if left != 0.0 or right != 0.0:
                        rt['zero_since'] = None
                        if not rt['drive_confirmed']:
                            rt['drive_confirmed'] = True
                            self._lake_id_event_locked('command_path_confirm',
                                                       'boat-applied L=%.3f R=%.3f' % (left, right))
                    else:
                        if rt['zero_since'] is None:
                            rt['zero_since'] = rx
                        span = rx - rt['zero_since']
                        if span > LAKE_ID_DRIVE_CONFIRM_S:
                            self._lake_id_event_locked(
                                'boat_refusal', 'commanded %.2f, boat-applied L/R zero across fresh '
                                'reports spanning %.2f s' % (ph[2], span))
                            return ('boat is not driving — commanded %.2f, boat-applied L/R zero '
                                    'for %.1f s' % (ph[2], span))
        return None

    def _lake_id_tick_locked(self, now):
        rt = getattr(self, 'lake_id', None)
        if rt is None or rt['finalizing']:
            return
        elapsed = now - rt['t0']
        ph = lake_id_phase_at(rt['phases'], elapsed)

        # ---- PRECHECK: a waiting window ---------------------------------
        if ph is not None and ph[0] == 'precheck':
            self.throttle = 0.0; self.motor_left = 0.0; self.motor_right = 0.0
            self.motor_split = False; self.rudder = 0.0
            conds = self._lake_id_precheck_locked(now)
            for k, v in conds.items():
                if rt['precheck'].get(k) != v:
                    self._lake_id_event_locked('precheck_met' if v else 'precheck_wait', k)
            rt['precheck'] = conds
            if not self.connected:
                return self._abort_lake_id_locked('serial link disconnected')
            if self._lake_writer.error:
                return self._abort_lake_id_locked('recording failed: %s' % self._lake_writer.error)
            return
        if ph is not None and rt['phase'] == 'precheck':
            # THE GATE: judged on the LATEST states, evaluated here and now --
            # not on the dict from the previous tick, which could be up to
            # 67 ms stale and let a condition that just broke slip through.
            conds = self._lake_id_precheck_locked(now)
            rt['precheck'] = conds
            unmet = [k for k, v in conds.items() if not v]
            if unmet:
                self._lake_id_event_locked('precheck_fail', ', '.join(unmet))
                return self._abort_lake_id_locked('precheck failed: ' + ', '.join(unmet))
            self._lake_id_event_locked('precheck_pass', 'all conditions met at t=%.2f s' % elapsed)
            # GPS is ADVISORY, not a gate: nothing in the control loop or the
            # safeguards uses it. Without a fix the run still measures yaw,
            # heading and the turn/recovery response; only position, speed,
            # course and the provisional radius go missing -- say so now.
            if not self.telemetry.get('gps_valid'):
                self._lake_id_warn_locked('GPS: no fix at start (%s satellites) -- position, speed, '
                                          'course and turn radius will be unavailable'
                                          % self.telemetry.get('satellites'))
            ms = self.motor_status
            rt['p_at_start'] = bool(ms.get('assist_motor_p'))
            self._lake_id_event_locked(
                'mode_confirmed', 'Requested Motor P mode and Rudder Assist OFF confirmed by MotorStatus '
                '(assist_motor_p=%d, assist_rudder=%d, request_id=%s)'
                % (1 if ms.get('assist_motor_p') else 0, 1 if ms.get('assist_rudder') else 0,
                   ms.get('assist_request_id')))

        # ---- 57 s and beyond: STOP confirmation, then bounded teardown ------
        if ph is None:
            self.throttle = 0.0; self.motor_left = 0.0; self.motor_right = 0.0
            self.motor_split = False; self.rudder = 0.0
            ms = self.motor_status
            fresh = (ms.get('have') and ms.get('last_rx_monotonic') is not None
                     and rt['stop_entered_at'] is not None
                     and ms['last_rx_monotonic'] > rt['stop_entered_at']
                     and (now - ms['last_rx_monotonic']) <= LAKE_ID_STOP_CONFIRM_MAX_AGE_S)
            zero = (ms.get('left_throttle') == 0.0 and ms.get('right_throttle') == 0.0
                    and ms.get('rudder_cmd') == 0.0)
            if fresh and zero:
                rt['stop_confirmed'] = True
                self._lake_id_event_locked('stop_confirmed',
                                           'fresh MotorStatus: L/R=0, rudder=0 at t=%.2f s' % elapsed)
                return self._finish_lake_id_locked(LAKE_ID_STATUS_COMPLETE, None)
            if elapsed < rt['profile_s'] + LAKE_ID_TEARDOWN_MAX_S:
                if not rt['teardown_logged']:
                    rt['teardown_logged'] = True
                    rt['phase'] = 'teardown'
                    self._lake_id_event_locked('phase_enter', 'teardown: zeros continue, waiting for confirmation')
                return
            rt['stop_confirmed'] = False
            self._lake_id_event_locked('stop_unconfirmed', 'no fresh zero confirmation by t=%.2f s' % elapsed)
            return self._finish_lake_id_locked(LAKE_ID_STATUS_STOP_UNCONFIRMED,
                                               'STOP not confirmed by a fresh MotorStatus')

        # ---- powered and STOP phases --------------------------------------
        name, _dur, thr, rud, start = ph
        reason = self._lake_id_powered_abort_locked(now, rt, ph)
        if reason:
            return self._abort_lake_id_locked(reason)
        if name != rt['phase']:
            rt['phase'] = name; rt['phase_start'] = rt['t0'] + start
            self._lake_id_event_locked('phase_enter', '%s: throttle=%.2f motor_diff=%+.2f' % (name, thr, rud))
            if name == 'stop':
                rt['stop_entered_at'] = now
        # Lake steering is differential thrust. `rud` is the canonical
        # half-difference carried by the phase table: negative means the
        # RIGHT motor is stronger and the boat turns LEFT.
        self.throttle = thr
        self.motor_left = max(0.0, min(1.0, thr + rud))
        self.motor_right = max(0.0, min(1.0, thr - rud))
        self.motor_split = True
        self.rudder = 0.0                 # physical rudder/servo is not used
        if rt['cmd_last'] != (thr, rud):
            rt['cmd_last'] = (thr, rud)
            self._lake_id_event_locked(
                'cmd_sent', 'motor L=%.2f R=%.2f (repeated by the 15 Hz stream)'
                % (self.motor_left, self.motor_right))

    def _collect_lake_id_row_locked(self, yaw_rate, heading, now=None):
        """One row per RECEIVED telemetry frame, from button press to the end.
        Enqueued to the writer; never written here."""
        rt = getattr(self, 'lake_id', None)
        if rt is None or rt['finalizing']:
            return
        now = self._now() if now is None else now
        tel, ms, ss = self.telemetry, self.motor_status, self.system_status
        bs, bench = self.bridge_status, self.bench_status
        rt['frames_received'] += 1
        elapsed = now - rt['t0']
        gap = 0.0 if rt['last_frame_mono'] is None else (now - rt['last_frame_mono'])
        if rt['last_frame_mono'] is not None and gap > rt['max_gap_s']:
            rt['max_gap_s'] = gap
        rt['last_frame_mono'] = now
        ph = lake_id_phase_at(rt['phases'], elapsed)
        phase = rt['phase'] if ph is None else ph[0]
        phase_start = (ph[4] if ph is not None else rt['profile_s'])
        gps_now = (tel.get('lat'), tel.get('lon'), tel.get('speed_mps'), tel.get('course_deg'))
        changed = 1 if (rt['last_gps'] is not None and gps_now != rt['last_gps']) else 0
        rt['last_gps'] = gps_now
        vals = [yaw_rate, heading, tel.get('pitch'), tel.get('roll')]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
            rt['imu_nonfinite'] = True
        def age(d):
            l = d.get('last_rx_monotonic')
            return round(now - l, 4) if (d.get('have') and l is not None) else ''
        row = {
            't_utc': lake_id_utc(), 't_mono': round(now, 4), 'elapsed_s': round(elapsed, 4),
            'phase': phase, 'phase_elapsed_s': round(elapsed - phase_start, 4),
            'order': rt['order'], 'throttle_set': rt['throttle'], 'magnitude_set': rt['magnitude'],
            'yaw_dps': yaw_rate, 'heading_deg': heading,
            'pitch_deg': tel.get('pitch'), 'roll_deg': tel.get('roll'),
            'gps_valid': 1 if tel.get('gps_valid') else 0,
            'lat': tel.get('lat'), 'lon': tel.get('lon'), 'speed_mps': tel.get('speed_mps'),
            'course_deg': tel.get('course_deg'), 'satellites': tel.get('satellites'),
            'hdop': tel.get('hdop'), 'gps_values_changed': changed,
            'cmd_throttle': self.throttle, 'cmd_rudder': self.rudder,
            'cmd_left': self.motor_left, 'cmd_right': self.motor_right,
            'boat_applied_left_cmd': ms.get('left_throttle') if ms.get('have') else '',
            'boat_applied_right_cmd': ms.get('right_throttle') if ms.get('have') else '',
            'boat_applied_rudder_cmd': ms.get('rudder_cmd') if ms.get('have') else '',
            'boat_applied_rudder_pwm_us': ms.get('rudder_pulse_us') if ms.get('have') else '',
            'boat_state': ms.get('state') if ms.get('have') else '',
            'boat_servo_power': (1 if ms.get('servo_power') else 0) if ms.get('have') else '',
            'boat_assist_motor_p': (1 if ms.get('assist_motor_p') else 0) if ms.get('have') else '',
            'boat_assist_rudder': (1 if ms.get('assist_rudder') else 0) if ms.get('have') else '',
            'boat_yaw_target_dps': ms.get('yaw_target_dps') if ms.get('have') else '',
            'boat_yaw_filt_dps': ms.get('yaw_filt_dps') if ms.get('have') else '',
            'boat_saturated': (1 if ms.get('rudder_saturated') else 0) if ms.get('have') else '',
            'imu_ok': (1 if ss.get('imu_ok') else 0) if ss.get('have') else '',
            'mag_ok': (1 if ss.get('mag_ok') else 0) if ss.get('have') else '',
            'gps_ok': (1 if ss.get('gps_ok') else 0) if ss.get('have') else '',
            'tof_a_ok': (1 if ss.get('tof_a_ok') else 0) if ss.get('have') else '',
            'tof_b_ok': (1 if ss.get('tof_b_ok') else 0) if ss.get('have') else '',
            'camera_ok': (1 if ss.get('camera_ok') else 0) if ss.get('have') else '',
            'telem_age_s': age(tel), 'motor_status_age_s': age(ms),
            'fusion_age_ms': ms.get('fusion_age_ms') if ms.get('have') else '',
            'system_status_age_s': age(ss),
            'gap_s': round(gap, 4),
            # The link, from this side: who went quiet is answerable after the fact.
            'tool_send_age_s': (round(now - self._last_send_mono, 4)
                                if getattr(self, '_last_send_mono', None) is not None else ''),
            'bridge_age_s': age(bs),
            'bridge_uplink_rssi_dbm': bs.get('uplink_rssi_dbm') if bs.get('have') else '',
            'bridge_espnow_pkts': bs.get('espnow_pkts') if bs.get('have') else '',
            'bridge_frames_out': bs.get('frames_out') if bs.get('have') else '',
            'bridge_reasm_drops': bs.get('reasm_drops') if bs.get('have') else '',
            'bench_learn_c_last': bench.get('learn_c') if bench.get('have') else '',
            'bench_status_age_s': age(bench),
            'heading_target_deg': ms.get('heading_target_deg') if ms.get('have') else '',
            'heading_error_deg': ms.get('heading_error_deg') if ms.get('have') else '',
            'yaw_target_dps': ms.get('motor_yaw_target_dps') if ms.get('have') else '',
            'yaw_filt_dps': ms.get('motor_yaw_filt_dps') if ms.get('have') else '',
            'rate_error_dps': ms.get('rate_error_dps') if ms.get('have') else '',
            'p_term': ms.get('p_term') if ms.get('have') else '',
            'i_term': ms.get('i_term') if ms.get('have') else '',
            'dynamic_c': ms.get('dynamic_c') if ms.get('have') else '',
            'effective_c': ms.get('effective_c') if ms.get('have') else '',
            'c_limit': ms.get('c_limit') if ms.get('have') else '',
            'ctrl_active': (1 if ms.get('ctrl_active') else 0) if ms.get('have') else '',
            'heading_hold': (1 if ms.get('heading_hold') else 0) if ms.get('have') else '',
            'saturated': (1 if ms.get('saturated') else 0) if ms.get('have') else '',
        }
        if len(rt['rows']) < LAKE_ID_MAX_ROWS:
            rt['rows'].append(row)
        self._lake_writer.put('row', row)     # a refusal is counted and aborts via the tick
        if rt['imu_nonfinite']:
            return self._abort_lake_id_locked('non-finite IMU value')
        # LIVE warnings -- exactly these two, and neither aborts: (1) the IMU
        # tuple identical for >= frozen_imu_s, (2) yaw sign opposite to the
        # hypothesis during a turn. The recording error is shown live too but
        # it ABORTS (abort table). GPS advisories, L/R differential drift,
        # coverage, missing notes and the STOP verdict are POST-RUN analysis
        # in lake_id_summarize().
        key = (yaw_rate, heading, tel.get('pitch'), tel.get('roll'))
        if rt['imu_last'] is not None and key == rt['imu_last'][0]:
            if now - rt['imu_last'][1] >= LAKE_ID_RULES['frozen_imu_s']:
                self._lake_id_warn_locked('IMU values identical for >= %.1f s (warning only; check the IMU)'
                                          % LAKE_ID_RULES['frozen_imu_s'])
        else:
            rt['imu_last'] = (key, now)
        turn_diff = ph[3] if ph is not None and phase in ('turn_a', 'turn_b') else 0.0
        if phase in ('turn_a', 'turn_b') and isinstance(yaw_rate, (int, float)) and turn_diff != 0.0:
            expected_pos = turn_diff < 0.0  # RIGHT motor stronger -> LEFT -> positive yaw
            y = yaw_rate
            if elapsed - phase_start > 3.0 and abs(y) > 0.5 and (y > 0) != expected_pos:
                # Nothing that changes per frame in the text: one warning per
                # phase, counted; the event's own timestamp says when it began.
                self._lake_id_warn_locked('%s: yaw sign opposite to the hypothesis (motor_diff %+.2f)'
                                          % (phase, turn_diff))

    def _abort_lake_id_locked(self, reason):
        if self.lake_id is None or self.lake_id['finalizing']:
            return
        self._lake_id_event_locked('abort', reason)
        self._finish_lake_id_locked(LAKE_ID_STATUS_ABORTED, reason)

    def _finish_lake_id_locked(self, status, reason):
        """Zeros first and on the wire now; then hand the run to the writer.
        The result is PUBLISHED only when the writer has flushed and closed
        both CSVs and atomically written summary.json."""
        rt = self.lake_id
        if rt is None or rt['finalizing']:
            return
        self.throttle = 0.0; self.motor_left = 0.0; self.motor_right = 0.0
        self.motor_split = False; self.rudder = 0.0
        if self.connected:
            self._send_motor_locked(0.0, 0.0)
            self._send_steer_locked(0.0)
        if status == LAKE_ID_STATUS_COMPLETE:
            self._lake_id_event_locked('complete', 'profile finished and stop confirmed')
        elif status == LAKE_ID_STATUS_STOP_UNCONFIRMED:
            self._lake_id_event_locked('incomplete', reason or '')
        self._lake_id_event_locked('finalize_started', 'flushing CSVs, writing summary.json')
        rt['finalizing'] = True
        settings = {'throttle': rt['throttle'], 'magnitude': rt['magnitude'], 'order': rt['order'],
                    'raw_throttle_test': rt.get('raw_throttle_test', False),
                    'condition': rt['condition'], 'index': rt['index'], 'name': rt['name'],
                    'profile': rt['profile'], 'p_at_start': rt['p_at_start']}
        summarize = straight_run_summarize if lake_id_is_straight_profile(rt['profile']) else lake_id_summarize
        summary = summarize(
            rt['rows'], list(range(rt['events'])), settings, rt['provenance'], status, reason,
            abort_phase=rt['phase'], stop_confirmed=rt['stop_confirmed'], notes=rt['notes'],
            firmware_label=rt['firmware_label'], max_gap_s=rt['max_gap_s'])
        summary['live_warnings'] = list(rt['warnings'])
        summary['live_warning_counts'] = dict(rt['warning_counts'])
        summary['t_utc_end'] = lake_id_utc()
        # What THIS side counted; the writer adds enqueued/written/dropped.
        summary['recording'] = {'frames_received': rt['frames_received'],
                                'sample_rows_in_memory': len(rt['rows']),
                                'events_attempted': rt['events']}
        pending = {'name': rt['name'], 'dir': rt['dir'], 'status': status, 'reason': reason,
                   'profile': rt['profile'],
                   'phase': rt['phase'], 'order': rt['order'], 'condition': rt['condition'],
                   'throttle': rt['throttle'], 'magnitude': rt['magnitude'],
                   'frames': len(rt['rows']), 'frames_received': rt['frames_received'],
                   'max_gap_s': round(rt['max_gap_s'], 3),
                   'stop_confirmed': rt['stop_confirmed'], 'warnings': list(rt['warnings']),
                   'summary_warnings': list(summary['warnings'])}
        if lake_id_is_straight_profile(rt['profile']):
            st = summary['straight']
            pending.update(shape=st['shape'], dominant_side=st['dominant_side'],
                           total_turn_deg=st['total_turn_deg'])

        def on_done(result):
            # The writer's structured verdict is THE status: a run whose final
            # flush, close or summary write failed is published as
            # incomplete_recording_failed, never as complete, whatever the
            # motion did (that is kept as profile_status).
            #
            # Rescan FIRST (disk, on the writer thread, no lock): the moment
            # the result is visible, NEXT ORDER already reflects it. The other
            # way round the page could show COMPLETE next to a stale order.
            # A failed rescan must not hold the result back, though.
            try:
                self._lake_id_refresh_next()
            except Exception as exc:                       # noqa: BLE001
                print('[lake-id] next-order rescan failed: %s: %s' % (type(exc).__name__, exc),
                      file=sys.stderr, flush=True)
            with self._lock:
                self.lake_id_result = dict(
                    pending, published=True,
                    status=result.get('status') or pending['status'],
                    reason=result.get('reason') if result.get('status') else pending['reason'],
                    profile_status=result.get('profile_status'),
                    write_error=result.get('error'), write_stage=result.get('stage'),
                    summary_written=bool(result.get('summary_written')),
                    recording=result.get('recording'))
                self.lake_id = None
                self._lake_writer = None

        self._lake_writer.finalize(summary, on_done)

    def lake_id_status_locked(self):
        rt = getattr(self, 'lake_id', None)
        if rt is None:
            return None
        now = self._now()
        return {'active': True, 'name': rt['name'], 'phase': rt['phase'],
                'elapsed_s': round(now - rt['t0'], 2), 'profile_s': rt['profile_s'],
                'profile': rt['profile'],
                'order': rt['order'], 'condition': rt['condition'],
                'throttle': rt['throttle'], 'magnitude': rt['magnitude'],
                'frames': len(rt['rows']), 'finalizing': rt['finalizing'],
                'precheck_unmet': [k for k, v in rt['precheck'].items() if not v],
                'warnings': list(rt['warnings']),
                'recording_error': self._lake_writer.error if self._lake_writer else None}

    def _flush_rudder_test_write(self):
        """Write the pending CSV. Called from _stream_loop with the lock NOT
        held, and directly by tests."""
        pending = self._rudder_test_write
        if pending is None:
            return
        self._rudder_test_write = None
        rt, result = pending
        # Provenance first, so nobody can mistake this for the boat's own
        # 100 Hz SD recording of a bench run.
        head = [
            'laptop/radio-observed rudder test -- recorded by '
            'tools/espnow_drive.py from ESP-NOW telemetry frames, '
            'NOT the boat SD card',
            'throttle=%.2f rudder=%+.2f phases=%s'
            % (RUDDER_TEST_THROTTLE, rt['rudder'],
               '/'.join('%s:%.1fs' % (n, d)
                        for n, d, _t in RUDDER_TEST_PHASES)),
            'frames=%d duration_s=%.3f max_gap_s=%.3f%s'
            % (result['frames'], result['duration_s'], result['max_gap_s'],
               (' ABORTED=' + str(result['abort_reason']))
               if result['aborted'] else ''),
            'drive_frames=%d first_delay_s=%.3f tail_gap_s=%.3f '
            'span_s=%.3f max_gap_s=%.3f%s'
            % (result['drive_frames'], result['drive_first_delay_s'],
               result['drive_tail_gap_s'], result['drive_span_s'],
               result['drive_max_gap_s'],
               ' INCOMPLETE' if result['incomplete'] else ''),
        ]
        err = self._write_csv(result['path'], head, RUDDER_TEST_CSV_COLUMNS,
                              rt['rows'], 'rudder-test')
        if err:
            result['write_error'] = err

    def _collect_bench_yaw_locked(self, yaw_rate, heading):
        """One telemetry frame, kept only if the boat says it is DRIVING.

        Caller already holds self._lock -- both decode paths assign
        self.telemetry inside it, and this has to see the same bench state they
        were decoded against.

        Bounded: a firmware bench run is 3 s of ~20 Hz telemetry (~60 frames),
        or 10 s for a BASE10 (~200), so a cap well above either costs nothing and stops a
        stuck 'driving' state (a lost
        terminal packet, say) growing this without limit for the whole
        session."""
        if not self.bench_status.get('have'):
            return
        if self.bench_status.get('state') != BENCH_STATE_RUN:
            return
        if len(self.bench_yaw_samples) >= 4000:
            return
        now = self._now()
        self.bench_yaw_samples.append((now, yaw_rate, heading))

        # ...and the same frame, in full, for the CSV. Same cap and the same
        # RUN-only gate, so the file and the on-screen summary can never
        # describe different sets of frames.
        run = self.bench_run
        if run is None:
            return
        bs, ms = self.bench_status, self.motor_status
        tel_last = self.telemetry.get('last_rx_monotonic')
        gap = (now - run['last_t']) if run['last_t'] is not None else 0.0
        run['last_t'] = now
        run['rows'].append({
            't_mono': round(now, 4),
            'elapsed_s': round(now - run['t0'], 4),
            'yaw_dps': round(yaw_rate, 4),
            'heading_deg': ('' if heading is None else round(heading, 3)),
            'boat_left': ms.get('left_throttle', '') if ms.get('have') else '',
            'boat_right': ms.get('right_throttle', '') if ms.get('have') else '',
            'boat_state': ms.get('state', '') if ms.get('have') else '',
            'boat_servo_power': (1 if ms.get('servo_power') else 0)
                                if ms.get('have') else '',
            'bench_state': bs.get('state', ''),
            'bench_elapsed_s': round(bs.get('elapsed_s', 0.0), 4),
            'bench_samples': bs.get('samples', ''),
            # float32 off the wire: round, or every row carries six digits
            # of noise that look like precision and are not.
            'learn_c': round(bs.get('learn_c', 0.0), 6),
            'boat_p_on': 1 if bs.get('p_on') else 0,
            'heading_target_deg': round(bs.get('heading_target_deg', 0.0), 4),
            'heading_error_deg': round(bs.get('heading_error_deg', 0.0), 4),
            'yaw_target_dps': round(bs.get('yaw_target_dps', 0.0), 4),
            'p_term': round(bs.get('p_term', 0.0), 6),
            'i_term': round(bs.get('i_term', 0.0), 6),
            'dynamic_c': round(bs.get('dynamic_c', 0.0), 6),
            'effective_c': round(bs.get('effective_c', 0.0), 6),
            'c_limit': round(bs.get('c_limit', 0.0), 6),
            'ctrl_active': 1 if bs.get('ctrl_active') else 0,
            'heading_hold': 1 if bs.get('heading_hold') else 0,
            'saturated': 1 if bs.get('saturated') else 0,
            'telem_age_s': round(now - tel_last, 4) if tel_last else '',
            'gap_s': round(gap, 4),
        })

    def _handle_bench_status(self, bs):
        with self._lock:
            was_driving = (self.bench_status.get('have')
                           and self.bench_status.get('state') == BENCH_STATE_RUN)
            if int(bs.state) != 0:
                self.bench_requested_at = None    # the boat has spoken about the run
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
                # getattr keeps old test fixtures and legacy adapters valid;
                # a protobuf decoded with the new schema supplies the same
                # zero defaults when these fields were absent on the wire.
                'heading_target_deg': float(getattr(bs, 'heading_target_deg', 0.0)),
                'heading_error_deg': float(getattr(bs, 'heading_error_deg', 0.0)),
                'yaw_target_dps': float(getattr(bs, 'yaw_target_dps', 0.0)),
                'p_term': float(getattr(bs, 'p_term', 0.0)),
                'i_term': float(getattr(bs, 'i_term', 0.0)),
                'dynamic_c': float(getattr(bs, 'dynamic_c', 0.0)),
                'effective_c': float(getattr(bs, 'effective_c', 0.0)),
                'c_limit': float(getattr(bs, 'c_limit', 0.0)),
                'ctrl_active': bool(getattr(bs, 'ctrl_active', False)),
                'heading_hold': bool(getattr(bs, 'heading_hold', False)),
                'saturated': bool(getattr(bs, 'saturated', False)),
            }
            now_driving = (int(bs.state) == BENCH_STATE_RUN)
            # Collect only across the DRIVE phase. The boat's own state is what
            # delimits it -- guessing from elapsed_s would drift, and the
            # motors-off baseline/coast readings are not part of the turn.
            if now_driving and not was_driving:
                self.bench_yaw_samples = []          # a new run: start clean
                self.bench_yaw = None
                # The boat's own RUN edge opens the recording, for the same
                # reason it delimits the summary: guessing the phase from
                # elapsed_s would drift, and the motors-off baseline and coast
                # are not part of the turn.
                self.bench_run = {
                    't0': self._now(), 'last_t': None, 'rows': [],
                    'kind': int(bs.kind), 'base': float(bs.base),
                }
            elif was_driving and not now_driving:
                # A run only ends cleanly by running out the clock, which takes
                # it RUN -> COAST -> SAVED. Anything else leaving the drive
                # phase means the boat cut it short (disarm, excessive-yaw
                # trip) and saved NOTHING, so the frames we caught describe
                # part of a turn with no file to check them against. Treat that
                # as aborted even when the samples themselves look clean --
                # a tidy-looking summary of a run that never happened is the
                # worst possible output here.
                aborted = int(bs.state) not in (BENCH_STATE_COAST,
                                                BENCH_STATE_SAVED)
                self.bench_yaw = summarize_yaw(
                    self.bench_yaw_samples, aborted=aborted,
                    # Judged against THIS run's drive length. With the 3 s
                    # default a 3 s fragment of a BASE10 passed as complete.
                    drive_s=BENCH_DRIVE_S_BY_KIND.get(int(bs.kind), BENCH_DRIVE_S))
                self.bench_yaw['kind'] = self.bench_status['kind']
                self.bench_yaw['base'] = self.bench_status['base']
                self.bench_yaw['end_state'] = int(bs.state)
                self._queue_bench_write_locked(aborted, int(bs.state),
                                               int(bs.file_index))

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
                # UI-observed yaw summary for the last completed drive phase.
                # None until one finishes. Deliberately NOT age-stamped: it
                # describes a run that is already over, so it does not go stale
                # the way live telemetry does.
                'bench_yaw': dict(self.bench_yaw) if self.bench_yaw else None,
                'bench_yaw_live': len(self.bench_yaw_samples),
                'bench_csv': self.bench_csv_name,
                # Read-only view of the rudder test. status() is hit by every
                # browser poll, so it must never touch rudder_test['rows'].
                'lake_id': self.lake_id_status_locked(),
                'lake_id_result': (dict(self.lake_id_result) if getattr(self, 'lake_id_result', None) else None),
                'lake_id_next': self.lake_id_next(),
                'lake_id_defaults': {'throttles': list(LAKE_ID_THROTTLES),
                                     'magnitudes': list(LAKE_ID_MAGNITUDES),
                                     'firmware_label': ('raw-throttle-test' if getattr(self, 'raw_throttle_test', False)
                                                        else LAKE_ID_FIRMWARE_LABEL_DEFAULT),
                                     'raw_throttle_test': getattr(self, 'raw_throttle_test', False),
                                     'note_fields': list(LAKE_ID_NOTE_FIELDS)},
                'rudder_test': ({
                    'active': True,
                    'phase': self.rudder_test['phase'],
                    'awaiting_assist': self.rudder_test['t0'] is None,
                    'sign': self.rudder_test['sign'],
                    'elapsed_s': (round(self._now() - self.rudder_test['t0'], 2)
                                  if self.rudder_test['t0'] is not None else 0.0),
                    'total_s': RUDDER_TEST_TOTAL_S,
                    'pct': RUDDER_TEST_PCT,
                    'frames': len(self.rudder_test['rows']),
                    'name': self.rudder_test['name'],
                } if self.rudder_test else None),
                'assist_rudder_on': self.assist_rudder_on,
                'assist_mode_pending': getattr(self, '_assist_off_req_id', None) is not None,
                'assist_mode_want_p': bool(getattr(self, '_assist_pending_p_on',
                                                   getattr(self, 'p_assist_on', False))),
                'rudder_test_result': (dict(self.rudder_test_result)
                                       if self.rudder_test_result else None),
                'motor_status': self._with_age(self.motor_status, TELEMETRY_STALE_S),
                'seq': self.seq,
                'last_error': self.last_error,
                'telemetry': self._with_age(self.telemetry, TELEMETRY_STALE_S),
                # The trail itself is served by /api/track on demand; this
                # push runs 20x a second, so it carries only the count.
                'gps_track_points': self._track_len_locked(),
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
                # COMMANDED rudder, never measured -- no position feedback.
                'rudder_cmd': float(ms.rudder_cmd),
                'rudder_pulse_us': int(ms.rudder_pulse_us),
                'rudder_saturated': bool(ms.rudder_saturated),
                'assist_rudder': bool(ms.assist_rudder),
                'assist_motor_p': bool(ms.assist_motor_p),
                'yaw_target_dps': float(ms.yaw_target_dps),
                'yaw_filt_dps': float(ms.yaw_filt_dps),
                'assist_request_id': int(ms.assist_request_id),
                'heading_target_deg': float(getattr(ms, 'heading_target_deg', 0.0)),
                'heading_error_deg': float(getattr(ms, 'heading_error_deg', 0.0)),
                'p_term': float(getattr(ms, 'p_term', 0.0)),
                'i_term': float(getattr(ms, 'i_term', 0.0)),
                'dynamic_c': float(getattr(ms, 'dynamic_c', 0.0)),
                'effective_c': float(getattr(ms, 'effective_c', 0.0)),
                'c_limit': float(getattr(ms, 'c_limit', 0.0)),
                'ctrl_active': bool(getattr(ms, 'ctrl_active', False)),
                'heading_hold': bool(getattr(ms, 'heading_hold', False)),
                'saturated': bool(getattr(ms, 'saturated', False)),
                'fusion_age_ms': int(getattr(ms, 'fusion_age_ms', 0)),
                'rate_error_dps': float(getattr(ms, 'rate_error_dps', 0.0)),
                'motor_yaw_target_dps': float(getattr(ms, 'motor_yaw_target_dps', 0.0)),
                'motor_yaw_filt_dps': float(getattr(ms, 'motor_yaw_filt_dps', 0.0)),
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

    # ---- the map's trail ------------------------------------------------

    def _record_fix_locked(self, gps_valid, lat, lon):
        """Add a fix to the trail. Caller holds self._lock. The tests build
        links with __new__ and only the attributes they need, so a link with
        no track simply does not record -- that is not an error."""
        track = getattr(self, 'gps_track', None)
        if track is not None:
            track.add(lat, lon, valid=bool(gps_valid))

    def _track_len_locked(self) -> int:
        track = getattr(self, 'gps_track', None)
        return len(track) if track is not None else 0

    def track_snapshot(self) -> dict:
        """The whole trail, for /api/track -- on demand, never in the 20 Hz
        status push."""
        with self._lock:
            return {'points': self.gps_track.points(),
                    'max_points': self.gps_track.max_points,
                    'min_move_m': self.gps_track.min_move_m}

    def track_clear(self):
        with self._lock:
            self.gps_track.clear()

    def _stream_send_locked(self):
        """One tick of the 15 Hz command stream. Caller holds the lock."""
        if self.calibrating:
            # Firmware owns the actuators; send ONLY the keepalive.
            # Any motor/steer/winch here would trip the abort.
            self._send_calibrate_locked(True)
            return
        if (self.winch_speed != 0.0 and
                time.monotonic() >= self.winch_lease_until):
            self.winch_speed = 0.0
        if self.motor_split:
            ok = self._send_motor_locked(self.motor_left, self.motor_right)
        else:
            ok = self._send_motor_locked(self.throttle, self.throttle)
        if ok:
            # Recorded into every lake/straight row as tool_send_age_s, so
            # an abort can say whether THIS side went quiet.
            self._last_send_mono = self._now()
        # Motor-only lake profiles do not use the physical rudder. Sending a
        # zero SteerCommand beside every MotorCommand doubles the uplink load
        # and can keep an old drive proposal alive when a motor frame is lost.
        if getattr(self, 'lake_id', None) is None:
            ok = self._send_steer_locked(self.rudder) and ok
        # Only while an assisted run is live: the firmware
        # ignores it otherwise, but there is no reason to put
        # it on the air at all.
        if self.rudder_test is not None and \
                self.rudder_test.get('assisted'):
            ok = self._send_steer_rate_locked(
                self.rudder_test.get('rate_dps', 0.0)) and ok
        # A lake run never moves the winch: its packet is a third
        # of the uplink the boat's 0.4 s failsafe watches, for
        # nothing. Manual driving keeps the keepalive as before.
        if self.winch_speed != 0.0 or getattr(self, 'lake_id', None) is None:
            ok = self._send_winch_locked(self.winch_speed) and ok
        self._rudder_test_after_send_locked(ok)

    def _stream_loop(self):
        period = 1.0 / self.send_hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                # Before the sends: the sequence only sets throttle/rudder and
                # the block below transmits them, so it inherits every existing
                # safety path instead of opening a second command route.
                now_mono = time.monotonic()
                self._assist_off_tick_locked(now_mono)
                self._rudder_test_tick_locked(now_mono)
                self._lake_id_tick_locked(now_mono)
                # THE LEASE. A rudder test drives itself and is not browser
                # input, so it keeps its own authority; anything else must be
                # backed by a live browser saying so.
                if (self.rudder_test is None and getattr(self, 'lake_id', None) is None
                        and not self._drive_lease_ok_locked(now_mono)
                        and self._anything_commanded_locked()):
                    if self.lease_expired_at is None:
                        self.lease_expired_at = now_mono
                        age = now_mono - self.session_last_hb
                        reason = ('no active control session' if self.session_id is None else
                                  'last browser heartbeat %.2fs ago' % age)
                        print('[lease] %s (limit %.2fs) — zeroing manual controls. '
                              'Keep the control tab visible; if this repeats while visible, '
                              'check browser/HTTP responsiveness.'
                              % (reason, CONTROL_LEASE_S), flush=True)
                    self._zero_controls_locked()
                    self._transmit_zeros_locked()
                if self.connected:
                    self._stream_send_locked()
            # File I/O deliberately outside the lock -- a few ms of CSV write
            # must never sit inside the 15 Hz command loop's critical section.
            self._flush_rudder_test_write()
            self._flush_bench_write()
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
                self._collect_bench_yaw_locked(yaw_rate, heading)
                self._collect_rudder_test_row_locked(yaw_rate, heading)
                self._collect_lake_id_row_locked(yaw_rate, heading)
                self._record_fix_locked(gps_valid, lat, lon)
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
            self._collect_bench_yaw_locked(s.imu.yaw_rate, s.imu.heading)
            self._collect_rudder_test_row_locked(s.imu.yaw_rate, s.imu.heading)
            self._collect_lake_id_row_locked(s.imu.yaw_rate, s.imu.heading)
            self._record_fix_locked(s.gps.valid, s.gps.latitude, s.gps.longitude)

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
        self._flush_rudder_test_write()   # the abort above may have queued one
        self._flush_bench_write()
        self._stop.set()


# Colors/classes copied verbatim from main/dashboard.html's :root tokens and
# .btn-arm/.state-dot/.motor-slider-row rules -- see feedback_projection_
# constants in project memory: dashboard.html is the visual source of truth,
# match it exactly rather than reinvent a palette.
_PAGE_TEMPLATE = """<!DOCTYPE html>
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
         align-items: center; padding: 16px 16px 24px; }
  header { display: flex; align-items: center; gap: 12px; margin-bottom: 16px;
           width: 100%; max-width: 1640px; }
  .menu-btn { margin-left: auto; font-size: 10px; letter-spacing: 1px; padding: 4px 10px; }
  .menu-btn.on { color: var(--accent); border-color: var(--accent); }
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

  /* ---- layout: what you DO on the left, the map in the middle, what the
     boat TELLS you on the right. Cards keep their 360px width; only the
     wrappers are new. The map panel is sticky, so it stays in view while
     either column scrolls. Hide the map (MAP in the top bar -- a bench
     session has no GPS) and the two columns sit side by side instead. */
  .layout { width: 100%; max-width: 1640px; display: grid; column-gap: 16px;
            grid-template-columns: 360px minmax(0, 1fr) 360px;
            grid-template-areas: "do map tell"; align-items: start; }
  .layout.no-map { grid-template-columns: 360px 360px; grid-template-areas: "do tell";
                   justify-content: center; }
  .col { display: flex; flex-direction: column; align-items: center; min-width: 0; }
  .col-do { grid-area: do; }
  .col-tell { grid-area: tell; }
  .map-panel { grid-area: map; position: sticky; top: 16px; min-width: 0;
               height: calc(100vh - 32px); min-height: 420px; margin-bottom: 16px;
               display: flex; flex-direction: column; background: var(--card);
               border: 1px solid var(--border); border-radius: 10px; padding: 12px; }
  .map-panel[hidden] { display: none; }
  @media (max-width: 1180px) {
    .layout { grid-template-columns: 360px minmax(0, 1fr);
              grid-template-areas: "do map" "tell map"; }
  }
  @media (max-width: 780px) {
    .layout, .layout.no-map { grid-template-columns: minmax(0, 1fr);
                              grid-template-areas: "do" "map" "tell"; }
    .layout.no-map { grid-template-areas: "do" "tell"; }
    .map-panel { position: static; height: 70vh; }
  }
  /* Folding cards keep every row, just tucked under the title; the title
     pill keeps updating while folded. */
  details.card > summary.card-title { cursor: pointer; list-style: none;
                                      margin-bottom: 0; user-select: none; }
  details.card > summary.card-title::-webkit-details-marker { display: none; }
  details.card > summary.card-title::before { content: '▸'; color: var(--dim); margin-right: 8px; }
  details.card[open] > summary.card-title { margin-bottom: 12px; }
  details.card[open] > summary.card-title::before { content: '▾'; }

  /* ---- map ---- */
  .map-menu { gap: 8px; margin-bottom: 8px; }
  .map-menu .title { margin-right: auto; }
  .map-menu select { flex: 0 1 auto; padding: 3px 6px; font-size: 11px; }
  .map-menu button { padding: 3px 8px; font-size: 10px; text-transform: uppercase;
                     letter-spacing: 1px; }
  .map-readout { display: flex; flex-wrap: wrap; gap: 4px 14px; margin-bottom: 8px;
                 font-size: 11px; }
  .map-readout label { font-size: 10px; color: var(--dim); text-transform: uppercase;
                       letter-spacing: 1px; margin-right: 5px; }
  .map-readout b { color: var(--accent); }
  .map-box { position: relative; flex: 1 1 auto; min-height: 240px; border-radius: 6px;
             overflow: hidden; background: #0A1220; border: 1px solid var(--border); }
  #map { position: absolute; top: 0; left: 0; right: 0; bottom: 0; }
  #map-note { position: absolute; left: 50%; top: 50%; transform: translate(-50%, -50%);
              z-index: 1100; pointer-events: none; max-width: 80%; text-align: center;
              background: rgba(5, 8, 15, 0.88); border: 1px solid var(--border);
              border-radius: 6px; padding: 8px 12px; font-size: 11px; color: var(--warn); }
  #map-note[hidden] { display: none; }
  .boat-icon { background: none; border: none; }
  .boat-arrow { display: block; width: 26px; height: 26px; filter: drop-shadow(0 0 3px #000); }
  .boat-arrow.stale { opacity: 0.45; }
  .leaflet-container { background: #0A1220; font-family: inherit; }
</style>
</head>
<body>

<header>
  <h1>ESP-NOW Drive</h1>
  <span class="pill" id="conn-pill">DISCONNECTED</span>
  <button class="menu-btn on" id="map-toggle" title="show or hide the map (a bench session has no GPS)">MAP</button>
</header>

<div class="layout" id="layout">
<div class="col col-do" id="col-do">

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

<details class="card" id="bench-card">
  <summary class="card-title">Throttle mismatch test <span class="pill" id="bench-pill" style="margin-left:6px;">IDLE</span></summary>
  <div class="row slider-row">
    <label>Throttle %</label>
    <input type="number" id="bench-throttle" min="1" max="60" step="1" value="20" style="width:56px;">
    <label style="min-width:auto;margin-left:10px;">Split &#177;%</label>
    <input type="number" id="bench-delta" min="1" max="30" step="1" value="12" style="width:56px;">
  </div>
  <!-- What the jets ACTUALLY get. The boat applies the learned trim on top of
       the commanded split for every kind, so the nominal +/-delta is not what
       runs. Shown rather than hidden: the asymmetry is the thing being
       measured, not a bug to paper over. -->
  <div class="telem-row"><label>Effective L / R</label><span class="val" id="bench-eff">--</span></div>
  <div class="telem-row"><label>Trim used</label><span class="val" id="bench-c-src">--</span></div>
  <div id="bench-warn" style="font-size:10px;color:var(--warn);min-height:12px;"></div>
  <div class="row">
    <!-- Named for the MOTOR, not a turn direction: which way the boat
         physically swings has not been measured yet, and these runs are how we
         measure it. Do not rename to LEFT TURN / RIGHT TURN until it has. -->
    <button id="bench-left" title="port jet commanded stronger, starboard weaker (before trim)">LEFT MOTOR STRONGER</button>
    <button id="bench-right" title="starboard jet commanded stronger, port weaker (before trim)">RIGHT MOTOR STRONGER</button>
    <button id="bench-base" title="boat-owned 3 s BASE recording on its SD card">BOAT BASE 3s</button>
    <button id="bench-base-short" title="laptop-recorded normal-drive check: precheck 2 s, centred rudder at this throttle for 3 s, stop 5 s. Motor P may be ON or OFF and its confirmed switch state is recorded.">BASE TEST 3s</button>
    <button id="bench-base-long" title="laptop-recorded normal-drive check: precheck 2 s, centred rudder at this throttle for 30 s, stop 5 s. Motor P may be ON or OFF and its confirmed switch state is recorded.">BASE TEST 30s</button>
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
  <!-- Derived from the telemetry frames THIS TOOL caught during the 3 s drive
       phase (~20 Hz over the air). The boat samples at 100 Hz to its SD card
       and that file is authoritative; this is a quick "which way, how hard". -->
  <div class="telem-row" style="margin-top:6px;"><label>Yaw mean / peak</label><span class="val" id="bench-yaw-rate">--</span></div>
  <div class="telem-row"><label>Yaw angle</label><span class="val" id="bench-yaw-angle">--</span></div>
  <div class="telem-row"><label>Heading &#916;</label><span class="val" id="bench-yaw-hdg">--</span></div>
  <div class="telem-row"><label>Samples</label><span class="val" id="bench-yaw-n">--</span></div>
  <div class="telem-row"><label>Saved CSV</label><span class="val" id="bench-csv">--</span></div>
  <div id="bench-yaw-note" style="font-size:10px;color:var(--dim);">UI-observed / approximate &mdash; the boat's SD CSV is authoritative. This laptop also saves its own radio-observed CSV to dataout/.</div>
  <div id="bench-msg" style="font-size:10px;color:var(--warn);">boat records to its own SD card; DISARM stops a run. RESET is one-shot &mdash; ordinary BASE runs keep the learned c.</div>
</details>

<details class="card" id="rudder-test-card">
  <summary class="card-title">Rudder test <span class="pill" id="rt-pill" style="margin-left:6px;">IDLE</span></summary>
  <!-- Fixed sequence: rudder over for 0.5s, then T20 for exactly 3.0s, then
       1.0s of coast still held over, then centre. Named by SIGN only -- which
       way the boat physically turns is what this test is FOR, so the buttons
       must not claim a direction nobody has established yet. -->
  <div class="row">
    <button id="rt-minus" title="hold rudder -0.80 through a 3 s T20 drive">RUDDER &minus;80 TEST</button>
    <button id="rt-plus" title="hold rudder +0.80 through a 3 s T20 drive">RUDDER +80 TEST</button>
  </div>
  <div class="telem-row"><label>Phase</label><span class="val" id="rt-phase">--</span></div>
  <div class="telem-row"><label>Saved as</label><span class="val" id="rt-file">--</span></div>
  <div class="telem-row"><label>Frames</label><span class="val" id="rt-frames">--</span></div>
  <div class="row" style="margin-top:6px;">
    <label style="min-width:auto;">Assisted Steering</label>
    <button id="rt-assist" title="rudder yaw-rate loop -- OFF at every boot; mutually exclusive with motor P assist">ASSIST: OFF</button>
    <span class="pill" id="rt-assist-confirm" style="margin-left:8px;">boat: --</span>
  </div>
  <!-- Assisted runs: same 0.5/3.0/1.0 profile, but the boat's own rudder loop
       chases a yaw-rate TARGET instead of holding a fixed deflection. -->
  <div class="row">
    <button id="rt-al" title="assisted: hold +2 deg/s (left) through a 3 s T20 drive">ASSIST L +2&deg;/s</button>
    <button id="rt-ar" title="assisted: hold -2 deg/s (right) through a 3 s T20 drive">ASSIST R &minus;2&deg;/s</button>
  </div>
  <div id="rt-msg" style="font-size:10px;color:var(--warn);min-height:12px;"></div>
  <div style="font-size:10px;color:var(--dim);">Laptop/radio-observed, ~20&nbsp;Hz &mdash; not the boat's 100&nbsp;Hz SD recording. ARM first; STOP or DISARM aborts.</div>
</details>
<details class="card" open id="lake-card">
  <summary class="card-title">Lake steering ID <span class="pill" id="lake-pill" style="margin-left:6px;">IDLE</span></summary>
  <div id="lake-raw-mode" style="color:var(--warn);"></div>
  <div style="font-size:10px;color:var(--dim);margin-bottom:4px;">Motor-only tests: the physical rudder and servo rail are unused, and Rudder Assist OFF is required. YAW PULSE drives straight 20 s, makes the right motor stronger for 2 s (turn left), recovers 10 s, then makes the left motor stronger for 2 s (turn right). Motor P must be ON and boat-confirmed before throttle. GPS fix is advisory.</div>
  <div class="motor-slider-row" style="gap:6px;flex-wrap:wrap;">
    <label style="font-size:10px;">throttle</label>
    <select id="lake-throttle"><option value="0.20" selected>T20</option><option value="0.30">T30</option></select>
    <label style="font-size:10px;">motor difference</label>
    <select id="lake-mag"><option value="0.30" selected>30%</option><option value="0.60">60%</option></select>
    <button id="lake-start" title="one press runs the whole 57 s profile; STOP aborts">START LAKE TEST</button>
  </div>
  <div class="motor-slider-row" style="gap:6px;flex-wrap:wrap;">
    <label style="font-size:10px;">motor yaw pulse</label>
    <select id="yaw-throttle"><option value="0.15">T15</option><option value="0.20">T20</option><option value="0.25">T25</option><option value="0.30">T30</option><option value="0.35">T35</option><option value="0.40" selected>T40</option><option value="0.45">T45</option><option value="0.50">T50</option></select>
    <select id="yaw-diff"><option value="0.05">&plusmn;5%</option><option value="0.10" selected>&plusmn;10%</option><option value="0.15">&plusmn;15%</option></select>
    <button id="lake-yaw-pulse" title="straight 20 s, right motor stronger 2 s, recover 10 s, left motor stronger 2 s, recover 10 s, stop">YAW PULSE TEST</button>
  </div>
  <div class="telem-row"><label>Next order</label><span class="val" id="lake-next">--</span></div>
  <div class="motor-slider-row" style="gap:6px;"><label style="font-size:10px;white-space:nowrap;">firmware label</label>
    <input type="text" id="lake-fw" style="flex:1;min-width:120px;background:var(--bg);color:var(--fg);border:1px solid var(--border);border-radius:4px;padding:2px 4px;font-size:10px;" title="prefilled with the build BELIEVED flashed: operator-supplied, not verified by the tool; edit if you flashed something else"></div>
  <div id="lake-notes" style="display:grid;grid-template-columns:1fr 1fr;gap:3px;margin:3px 0;"></div>
  <div class="telem-row"><label>Phase</label><span class="val" id="lake-phase">--</span></div>
  <div class="telem-row"><label>Elapsed</label><span class="val" id="lake-elapsed">--</span></div>
  <div class="telem-row"><label>Frames</label><span class="val" id="lake-frames">--</span></div>
  <div class="telem-row"><label>Run</label><span class="val" id="lake-file">--</span></div>
  <div id="lake-warn" style="font-size:10px;color:var(--warn);min-height:12px;"></div>
  <div id="lake-msg" style="font-size:10px;color:var(--warn);min-height:12px;"></div>
  <div style="font-size:10px;color:var(--dim);">MotorStatus values are boat-APPLIED software commands, not measured RPM, thrust or servo angle. Turns measure the operational trimmed boat; recoveries include active autotrim.</div>
  <div style="font-size:10px;color:var(--dim);">Live warnings (never abort): yaw sign opposite to the hypothesis, IMU values frozen. A recording error aborts. GPS advisories, L/R differential drift, coverage and missing notes are post-run analysis in summary.json.</div>
</details>

</div>

<section class="map-panel" id="map-panel">
  <div class="card-title map-menu">
    <span class="title">Map</span>
    <span class="pill" id="map-fix">NO DATA</span>
    <select id="map-layer" title="map layer">
      <option value="osm">OpenStreetMap</option>
      <option value="satellite">Satellite (Esri)</option>
      <option value="dark">Dark (Carto)</option>
    </select>
    <label class="bench" style="color:var(--dim);"><input type="checkbox" id="map-follow" checked>Follow boat</label>
    <button id="map-center" title="centre on the boat and follow it again (also retries loading the map)">&#8982; Centre</button>
    <button id="map-clear" title="forget the trail drawn so far -- on this page and in the tool">Clear trail</button>
  </div>
  <div class="map-readout">
    <span><label>Lat / Lon</label><b id="map-pos">--</b></span>
    <span><label>Speed</label><b id="map-speed">--</b></span>
    <span><label>Course</label><b id="map-course">--</b></span>
    <span><label>Heading</label><b id="map-heading">--</b></span>
    <span><label>Sats / HDOP</label><b id="map-sats">--</b></span>
    <span><label>Trail</label><b id="map-trail">0 pts</b></span>
  </div>
  <div class="map-box">
    <div id="map"></div>
    <div id="map-note">loading map…</div>
  </div>
</section>

<div class="col col-tell" id="col-tell">

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

<div class="card" id="motorstatus-card">
  <div class="card-title">Motor (confirmed by boat) <span class="pill" id="mstat-pill" style="margin-left:6px;">NO DATA YET</span></div>
  <div class="telem-row"><label>Arm state</label><span class="val" id="m-armstate">--</span></div>
  <div class="telem-row"><label>Throttle L / R</label><span class="val" id="m-throttle">--</span></div>
  <div class="telem-row"><label>Winch</label><span class="val" id="m-winch">--</span></div>
  <div class="telem-row"><label>Servo rail</label><span class="val" id="m-servo">--</span></div>
</div>

<details class="card" id="sensors-card" open>
  <summary class="card-title">Sensors (boot check) <span class="pill" id="sensors-pill" style="margin-left:6px;">NO DATA YET</span></summary>
  <div class="telem-row"><label>Camera</label><span class="val" id="s-camera">--</span></div>
  <div class="telem-row"><label>ToF A</label><span class="val" id="s-tof-a">--</span></div>
  <div class="telem-row"><label>ToF B</label><span class="val" id="s-tof-b">--</span></div>
  <div class="telem-row"><label>IMU</label><span class="val" id="s-imu">--</span></div>
  <div class="telem-row"><label>Compass</label><span class="val" id="s-mag">--</span></div>
</details>

<details class="card" id="bridge-card" open>
  <summary class="card-title">Bridge (S3) <span class="pill" id="bridge-pill" style="margin-left:6px;">NO DATA YET</span></summary>
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
</details>

</div>
</div>

<footer>
  <div>tx seq <span id="seq">0</span> <span id="err" class="err"></span></div>
  <div style="margin-top:6px;">ARM/DISARM in the Drive card is the last command sent; the boat's
  own answer is in "Motor (confirmed by boat)". GPS/IMU telemetry updating means the ESP-NOW
  link itself is genuinely alive.</div>
</footer>

<script>
const $ = id => document.getElementById(id);
let connected = false;
let armedCmd = false;
let servoRailOn = false;
// Mirrors BenchStatus in boat.proto (see BENCH_KIND_NAME / BENCH_STATE_NAME).
const BENCH_KIND_NAME = { 0: 'BASE', 1: 'LEFT', 2: 'RIGHT', 3: 'BASE10' };
// The letter the BOAT puts in the SD filename (motor_control.c, bench_write_csv).
// Explicit, not derived from the display name: BASE10.charAt(0) is 'B', and the
// boat writes 'G'. Showing a filename that does not exist on the card is worse
// than showing none.
const BENCH_SD_CHAR = { 0: 'B', 1: 'L', 2: 'R', 3: 'G' };
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
  const became = isConn && !connected;
  connected = isConn;
  if (became) ensureSession();      // covers a reload while already connected
  if (!isConn) { stopHeartbeat(); sessionId = null; }
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
  if (res && res.ok) await openSession();   // claim control for THIS tab
  if (res.ok) {
    winchDir = 0;
    setConnectedUI(true, port);
    zeroDriveUI();
  } else {
    $('hint').textContent = `connect failed: ${res.error}`;
  }
});

// Every path that puts the boat at zero -- STOP, DISARM, a fresh control
// session, a reconnect, the tab going away, calibration -- must put the
// SLIDERS at zero too. Only the linked throttle and the rudder used to be
// reset, so in per-motor mode the Left/Right sliders kept showing a value the
// boat no longer had, and the next nudge sent that motor straight back to it.
// The link/unlink MODE is the operator's choice and survives; the values do not.
function zeroDriveUI() {
  for (const id of ['throttle', 'motor-left', 'motor-right', 'rudder']) {
    $(id).value = 0;
    $(id + '-val').textContent = '0%';
  }
  ctrl = { throttle: 0, rudder: 0, left: 0, right: 0, split: !$('motor-link').checked };
}

$('throttle').addEventListener('input', (e) => {
  const v = parseInt(e.target.value);
  $('throttle-val').textContent = v + '%';
  ctrl.throttle = v / 100; ctrl.left = 0; ctrl.right = 0;
  ctrl.split = false; sendHeartbeat();
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
    ctrl.throttle = v / 100; ctrl.left = 0; ctrl.right = 0;
  ctrl.split = false; sendHeartbeat();
  } else {
    const v = parseInt($('throttle').value);      // split: both start at the throttle value
    $('motor-left').value = v;  $('motor-left-val').textContent = v + '%';
    $('motor-right').value = v; $('motor-right-val').textContent = v + '%';
    ctrl.left = v / 100; ctrl.right = v / 100; ctrl.throttle = 0;
    ctrl.split = true; sendHeartbeat();
  }
}
$('motor-link').addEventListener('change', (e) => setMotorLink(e.target.checked));

$('motor-left').addEventListener('input', (e) => {
  const v = parseInt(e.target.value);
  $('motor-left-val').textContent = v + '%';
  ctrl.left = v / 100; ctrl.throttle = 0; ctrl.split = true; sendHeartbeat();
});
$('motor-right').addEventListener('input', (e) => {
  const v = parseInt(e.target.value);
  $('motor-right-val').textContent = v + '%';
  ctrl.right = v / 100; ctrl.throttle = 0; ctrl.split = true; sendHeartbeat();
});

// ---- control session + heartbeat -------------------------------------------
// This process streams on our behalf, so it cannot tell "the operator is
// holding 40% throttle" from "the tab closed 20 seconds ago" unless we keep
// saying we are here. Miss the lease and the boat is zeroed.
var sessionId = null, ctrlSeq = 0, hbTimer = null;
// `split` is the MOTOR MODE, sent explicitly. false = the linked throttle
// drives both; true = independent left/right. The server used to infer it from
// which fields a request carried, which a full-state heartbeat makes
// impossible -- every field is always present.
var ctrl = { throttle: 0, rudder: 0, left: 0, right: 0, split: false };

async function openSession() {
  const r = await api('/api/session', 'POST', {});
  if (r && r.ok) {
    sessionId = r.session_id;
    ctrlSeq = 0;
    // Taking control starts from a stopped boat; mirror that locally so the
    // first heartbeat cannot re-assert a stale slider position.
    zeroDriveUI();
    stopHeartbeat();
    hbTimer = setInterval(sendHeartbeat, Math.round(1000 / (r.heartbeat_hz || 12)));
  }
  return r;
}

// A single slow HTTP response must not prevent the next full-state heartbeat
// from reaching the server before its 300 ms lease expires. Allow a FEW
// overlapping requests: sequence numbers reject a late older command, while
// the cap prevents an unresponsive server from accumulating unlimited fetches.
// Epochs keep a delayed response from a released/old session from touching a
// new session. Full-state requests make a dropped or stale one harmless.
const MAX_HB_IN_FLIGHT = 4;
var hbInFlight = 0, hbPending = false, hbEpoch = 0;

// SESSION WATCHDOG. A session can be missing for several unrelated reasons --
// the page was reloaded while already connected (the connect button never
// fires, so nothing claimed one), STOP or DISARM gave it up, or a heartbeat
// was refused because it had gone. Every one of those used to leave the UI
// permanently dead: sliders moved, nothing happened, and the only clue was
// "[lease] browser heartbeat lost" in the terminal.
//
// So instead of re-acquiring at each of those sites, one watchdog notices the
// state and fixes it. Re-acquiring is always safe: a new session starts at
// ZERO, so this can restore the ability to drive but never motion itself.
var acquiring = false;
async function ensureSession() {
  if (!connected || sessionId || acquiring || document.hidden) return;
  acquiring = true;
  try { await openSession(); } finally { acquiring = false; }
}
setInterval(ensureSession, 500);

async function sendHeartbeat() {
  if (!sessionId) return;
  if (hbInFlight >= MAX_HB_IN_FLIGHT) { hbPending = true; return; }
  const sentSession = sessionId, epoch = hbEpoch;
  hbInFlight++;
  try {
    const r = await api('/api/state', 'POST', {
      session_id: sentSession, seq: ++ctrlSeq,
      throttle: ctrl.throttle, rudder: ctrl.rudder,
      left: ctrl.left, right: ctrl.right, split: ctrl.split });
    if (r && !r.ok) {
      // ONLY a dead session tears this down. stale_seq means one request
      // arrived out of order, and assist_off_pending means the boat is
      // mid-transition -- in both cases the session is healthy and the lease
      // was refreshed, so dropping it would stop the boat for no reason.
      if ((r.code === 'no_session' || r.code === 'wrong_session')
          && epoch === hbEpoch && sessionId === sentSession) {
        // Give this one up, and let the watchdog claim a fresh one. NOT a
        // dead end: going quiet here is what made the UI unrecoverable.
        stopHeartbeat();
        sessionId = null;
        ensureSession();
      }
    }
  } catch (_e) {
    // If no state reaches the server, its unchanged lease still stops motion.
  } finally {
    if (epoch !== hbEpoch) return;  // old session's response
    hbInFlight--;
    if (hbPending && hbInFlight < MAX_HB_IN_FLIGHT) {
      hbPending = false;
      sendHeartbeat();
    }
  }
}

function stopHeartbeat() {
  if (hbTimer) { clearInterval(hbTimer); hbTimer = null; }
  hbEpoch++;
  hbInFlight = 0;
  hbPending = false;
}

// Going away must both stop the stream AND give up the session, so nothing can
// resume against a browser that is no longer watching.
//
// /api/release rather than /api/stop: a hidden tab must not abort a running
// calibration, which STOP does. The lease still covers everything neither
// event catches -- a crash, the network dying, a laptop lid closing.
function releaseControl(beacon) {
  stopHeartbeat();
  zeroDriveUI();
  sessionId = null;
  const body = new Blob(['{}'], { type: 'application/json' });
  if (beacon && navigator.sendBeacon) navigator.sendBeacon('/api/release', body);
  else api('/api/release', 'POST', {});
}

window.addEventListener('pagehide', () => releaseControl(true));

document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    releaseControl(true);
  } else if (connected) {
    // Coming back takes a FRESH session, starting at zero. Resuming the old
    // one would restore whatever the sliders happen to show, and the operator
    // did not ask for that by switching tabs back.
    openSession();
  }
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
  ctrl.rudder = v / 100;
  sendHeartbeat();          // immediate, then the timer keeps the lease alive
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
  zeroDriveUI();
  await api('/api/stop', 'POST', { seq: ++winchCommandSeq });
  // STOP drops the session server-side; mirror it so the UI does not keep
  // heartbeating a session that no longer exists. ARM re-acquires one.
  stopHeartbeat();
  sessionId = null;
});

$('arm-btn').addEventListener('click', async () => {
  if (!connected) return;
  const force = $('bench').checked;
  const nextArm = !armedCmd;
  $('hint').textContent = (nextArm && !force)
    ? 'sent without "bench (no GPS)" -- boat will refuse this if it has no GPS lock'
    : '';
  if (nextArm) {
    // STOP and DISARM both drop the session by design, so ARM has to acquire a
    // fresh one -- otherwise the boat arms with nothing able to drive it, and
    // the first slider move is refused as 'no_session'. A new session also
    // starts at ZERO, which is exactly what ARM requires: the server refuses
    // to arm against a held control.
    if (!sessionId) await openSession();
    else { zeroDriveUI(); await sendHeartbeat(); }
  }
  const r = await api('/api/arm', 'POST', { arm: nextArm, force });
  if (r && !r.ok) $('hint').textContent = r.error || 'refused';
  if (!nextArm) {
    // DISARM dropped the session server-side; stop pretending to hold one.
    stopHeartbeat();
    sessionId = null;
    zeroDriveUI();
  }
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
  const r = await api('/api/calibrate', 'POST', { start, seq: ++winchCommandSeq });
  // The boat is under the firmware's own control now (or has just been handed
  // back, zeroed). Sliders and heartbeat must say zero as well: otherwise the
  // throttle the operator held BEFORE the sweep would be quietly resumed the
  // moment it ends, with nobody touching anything.
  if (!start || (r && r.ok)) zeroDriveUI();
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
// The laptop-recorded normal-drive checks are separate deliberate durations.
// Both take the ordinary motor path so the selected P mode participates.
$('bench-base-short').addEventListener('click', () => runLakeId('straight'));
$('bench-base-long').addEventListener('click', () => runLakeId('straight30'));
$('lake-yaw-pulse').addEventListener('click', () => runLakeId('yawpulse'));

// The 4.5 s sequence runs on the server's own command loop; this call returns
// at once and progress arrives through the normal status poll. Nothing here
// sleeps or holds the page.
// Purely visual. Every one of these is ALSO refused server-side while a run
// is going -- a disabled button is decoration, and a stale tab, a second
// browser or a curl all reach the HTTP API without ever seeing it.
var _rtLockedOut = null;
function setRudderTestLockout(on) {
  on = on || _lakeActive;              // a lake run locks the same controls
  if (_rtLockedOut === on) return;      // don't fight the user every poll
  _rtLockedOut = on;
  ['bench-left', 'bench-right', 'bench-base', 'bench-base-short', 'bench-base-long', 'bench-reset', 'p-assist', 'lake-start', 'lake-yaw-pulse',
   'rt-minus', 'rt-plus', 'rt-al', 'rt-ar', 'rt-assist',
   'calibrate-btn'].forEach(function (id) {
    const el = $(id);
    if (!el) return;
    el.disabled = on;
    // Cosmetic, and guarded: this runs inside the status renderer, so a throw
    // here would take the whole UI update down with it -- for a greyed-out
    // button. The disabled flag above is the part that matters, and the real
    // refusal is server-side regardless.
    if (el.style) el.style.opacity = on ? '0.4' : '';
  });
}

async function runRudderTest(sign) {
  if (!connected) { $('rt-msg').textContent = 'not connected'; return; }
  $('rt-msg').textContent = '';
  const r = await api('/api/rudder_test', 'POST',
                      { sign, seq: ++winchCommandSeq });
  if (r && !r.ok) $('rt-msg').textContent = r.error || 'refused';
}
$('rt-minus').addEventListener('click', () => runRudderTest(-1));
$('rt-plus').addEventListener('click', () => runRudderTest(1));

// Assisted runs. sign -1 is LEFT stick, which the firmware turns into a
// POSITIVE yaw-rate target -- physical left produces positive IMU yaw on this
// boat (21 recorded runs).
async function runAssistTest(sign) {
  if (!connected) { $('rt-msg').textContent = 'not connected'; return; }
  $('rt-msg').textContent = '';
  const r = await api('/api/rudder_test', 'POST',
                      { sign, assisted: true, seq: ++winchCommandSeq });
  if (r && !r.ok) $('rt-msg').textContent = r.error || 'refused';
}
$('rt-al').addEventListener('click', () => runAssistTest(-1));
$('rt-ar').addEventListener('click', () => runAssistTest(1));

// ---- lake steering-identification test ------------------------------------
// One button. The 57 s profile runs on the server's stream loop; this page
// only starts it, shows it, and (through STOP or any slider) aborts it.
var LAKE_NOTE_FIELDS = ['battery', 'payload_load', 'mechanical_config', 'wind', 'current', 'waves', 'unusual_events'];
var _lakeActive = false, _lakeFwPrefilled = false, _lakeNextCache = null, _lakeNotesBuilt = false;
function lakeCondKey() {
  const t = Math.round(parseFloat($('lake-throttle').value) * 100);
  const m = Math.round(parseFloat($('lake-mag').value) * 100);
  return 'T' + String(t).padStart(2, '0') + '_M' + String(m).padStart(2, '0');
}
function renderLakeNext() {
  const n = _lakeNextCache && _lakeNextCache[lakeCondKey()];
  $('lake-next').textContent = n
    ? ((n.next_order === 'LR' ? 'LEFT then RIGHT' : 'RIGHT then LEFT') + '  (run #' + n.next_index
       + ', ' + n.complete_runs + ' complete for ' + lakeCondKey().replace('_', '/') + ')')
    : '--';
}
function buildLakeNotes(fields) {
  if (_lakeNotesBuilt) return;
  _lakeNotesBuilt = true;
  const box = $('lake-notes');
  (fields || LAKE_NOTE_FIELDS).forEach(function (k) {
    const inp = document.createElement('input');
    inp.type = 'text'; inp.id = 'lake-note-' + k; inp.placeholder = k.replace('_', ' ');
    inp.style.cssText = 'background:var(--bg);color:var(--fg);border:1px solid var(--border);border-radius:4px;padding:2px 4px;font-size:10px;';
    box.appendChild(inp);
  });
}
async function runLakeId(profile) {
  // Straight profiles use the bench card's throttle box and keep rudder centred.
  const straight = profile === 'straight' || profile === 'straight30';
  const yawpulse = profile === 'yawpulse';
  const msg = straight ? $('bench-msg') : $('lake-msg');
  if (!connected) { msg.textContent = 'not connected'; return; }
  msg.textContent = '';
  const notes = {};
  LAKE_NOTE_FIELDS.forEach(function (k) { const el = $('lake-note-' + k); notes[k] = el ? el.value : ''; });
  const r = await api('/api/lake_id', 'POST', {
    throttle: straight ? parseInt($('bench-throttle').value) / 100 : (yawpulse ? parseFloat($('yaw-throttle').value) : parseFloat($('lake-throttle').value)),
    magnitude: straight ? 0 : (yawpulse ? parseFloat($('yaw-diff').value) : parseFloat($('lake-mag').value)),
    profile: (straight || yawpulse) ? profile : 'full',
    firmware_label: $('lake-fw').value, notes: notes,
    seq: ++winchCommandSeq });
  if (r && !r.ok) msg.textContent = r.error || 'refused';
}
$('lake-start').addEventListener('click', function () { runLakeId('full'); });
$('lake-throttle').addEventListener('change', renderLakeNext);
$('lake-mag').addEventListener('change', renderLakeNext);
function renderLakeId(s) {
  const li = s.lake_id, lr = s.lake_id_result;
  _lakeNextCache = s.lake_id_next || null;
  if (s.lake_id_defaults) {
    if (s.lake_id_defaults.raw_throttle_test) {
      $('lake-raw-mode').textContent = 'RAW THROTTLE TEST: Motor P and Rudder Assist must stay OFF. Use the matching raw-test firmware; linked throttle sends equal linear commands.';
      $('lake-raw-mode').nextElementSibling.hidden = true;
    }
    buildLakeNotes(s.lake_id_defaults.note_fields);
    if (!_lakeFwPrefilled && !$('lake-fw').value) {
      $('lake-fw').value = s.lake_id_defaults.firmware_label;   // prefilled, editable, never silently replaced
      _lakeFwPrefilled = true;
    }
  }
  renderLakeNext();
  const pill = $('lake-pill');
  if (li && li.active) {
    _lakeActive = true;
    pill.textContent = li.finalizing ? 'FINALIZING' : li.phase.toUpperCase();
    pill.classList.add('up'); pill.classList.remove('stale');
    $('lake-phase').textContent = li.phase
      + (li.precheck_unmet && li.precheck_unmet.length ? '  waiting: ' + li.precheck_unmet.join(', ') : '');
    $('lake-elapsed').textContent = li.elapsed_s.toFixed(1) + ' / ' + li.profile_s + ' s';
    $('lake-frames').textContent = li.frames;
    $('lake-file').textContent = li.name + '  (' + (li.profile === 'straight30' ? 'BASE 30s, laptop-driven'
      : li.profile === 'straight' ? 'BASE 3s, laptop-driven'
      : (li.order === 'LR' ? 'LEFT then RIGHT' : 'RIGHT then LEFT')) + ')';
    $('lake-warn').textContent = (li.warnings || []).join('  |  ')
      + (li.recording_error ? '   RECORDING FAILED: ' + li.recording_error : '');
    $('lake-warn').style.color = li.recording_error ? 'var(--danger)' : 'var(--warn)';
  } else {
    _lakeActive = false;
    // COMPLETE needs both: the profile finished AND its record was written.
    const done = !!(lr && lr.status === 'complete' && !lr.write_error);
    pill.textContent = lr ? (done ? 'COMPLETE'
                            : (lr.status === 'aborted' ? 'ABORTED' : 'INCOMPLETE')) : 'IDLE';
    pill.classList.remove('up');
    pill.classList.toggle('stale', !!(lr && !done));
    if (lr) {
      $('lake-phase').textContent = lr.status.toUpperCase().replace(/_/g, ' ')
        + (lr.reason ? ':  ' + lr.reason : '')
        + ((lr.profile === 'straight' || lr.profile === 'straight30') && lr.shape
           ? '   ' + lr.shape + '  |  dominant: ' + lr.dominant_side
             + (lr.total_turn_deg == null ? '' : '  |  total turn ' + lr.total_turn_deg.toFixed(1) + ' deg')
           : '')
        + (lr.write_error ? '   WRITE ERROR: ' + lr.write_error : '');
      $('lake-elapsed').textContent = '--';
      $('lake-frames').textContent = lr.frames;
      $('lake-file').textContent = lr.name + '  ('
        + (lr.profile === 'straight30' ? 'BASE 30s, laptop-driven'
           : lr.profile === 'straight' ? 'BASE 3s, laptop-driven' : lr.order) + ')';
      $('lake-warn').textContent = (lr.summary_warnings || []).concat(lr.warnings || []).join('  |  ');
      $('lake-warn').style.color = lr.write_error ? 'var(--danger)' : 'var(--warn)';
    }
  }
  $('lake-start').disabled = !!(li && li.active) || _rtLockedOut;
}

// Runtime mode switch. Raw Manual is the boot mode and stays the default.
var assistRudderOn = false;
$('rt-assist').addEventListener('click', async () => {
  const want = !assistRudderOn;
  const r = await api('/api/rudder_assist', 'POST', { on: want });
  if (r && r.ok) {
    assistRudderOn = want;
    $('rt-assist').textContent = 'ASSIST: ' + (want ? 'ON' : 'OFF');
    $('rt-assist').classList.toggle('up', want);
  } else if (r) {
    $('rt-msg').textContent = r.error || 'refused';
  }
});

// ---- effective split preview -----------------------------------------------
// Mirror of bench_effective_commands() in this file's Python half, which is the
// tested reference. The boat applies trim = 2*c*base on top of the commanded
// split (bench_run.c), so:
//     LEFT  differential = 2*(c*base - delta)   zero at delta == c*base,
//                                               REVERSED below it
//     RIGHT differential = 2*(c*base + delta)   always amplified
// Raising the split does NOT make the two directions symmetric -- RIGHT stays
// the stronger one. It only keeps LEFT pointing the way its name says.
function clamp01(v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
function benchEffective(kind, base, delta, c) {
  base = clamp01(base); delta = clamp01(delta);
  if (!isFinite(c)) c = 0;
  let nl, nr;
  if (kind === 'left')       { nl = base + delta; nr = base - delta; }
  else if (kind === 'right') { nl = base - delta; nr = base + delta; }
  else                       { nl = base;         nr = base; }
  nl = clamp01(nl); nr = clamp01(nr);
  const trim = 2 * c * base;
  let left = nl, right = nr;
  if (!(nl <= 0 && nr <= 0)) {
    left = clamp01(nl - 0.5 * trim);
    right = clamp01(nr + 0.5 * trim);
  }
  return { left, right, differential: right - left,
           nominalDifferential: nr - nl, trim };
}
function sgn2(v) { return (v >= 0 ? '+' : '') + v.toFixed(2); }

// Current learner c. Until a BenchStatus arrives this is the FLASHED DEFAULT,
// not the boat's actual value -- the learner moves c while driving, so by the
// time you press a button it may be anywhere in [0.10, 0.35]. The preview is
// still worth showing (it is the right shape, and it warns), but it must not
// look like a measurement, so it is marked assumed until the boat speaks.
var benchLearnC = 0.17;
var benchLearnCFromBoat = false;
function refreshBenchPreview() {
  const base = parseInt($('bench-throttle').value) / 100;
  const delta = parseInt($('bench-delta').value) / 100;
  if (!isFinite(base) || !isFinite(delta)) { $('bench-eff').textContent = '--'; return; }
  const L = benchEffective('left', base, delta, benchLearnC);
  const R = benchEffective('right', base, delta, benchLearnC);
  $('bench-eff').textContent =
    'L-str ' + (L.left * 100).toFixed(1) + '/' + (L.right * 100).toFixed(1)
    + ' (' + sgn2(L.differential * 100) + ')   '
    + 'R-str ' + (R.left * 100).toFixed(1) + '/' + (R.right * 100).toFixed(1)
    + ' (' + sgn2(R.differential * 100) + ')';
  // Say which c this was computed from, and make an assumed one look assumed.
  // Everything on this row -- both differentials and the warning threshold --
  // scales with c, so a stale 0.17 against a learner that has walked to 0.24
  // is a different prediction entirely.
  var cEl = $('bench-c-src');
  cEl.textContent = benchLearnCFromBoat
    ? ('c = ' + benchLearnC.toFixed(3) + ' (boat)')
    : ('c = ' + benchLearnC.toFixed(3) + ' ASSUMED — no BenchStatus yet');
  cEl.classList.toggle('warn', !benchLearnCFromBoat);
  // Same threshold as bench_split_warning(): delta == c*base is exact cancellation.
  const cancel = Math.abs(benchLearnC) * base;
  let warn = '';
  if (cancel > 0 && delta < cancel) {
    warn = 'LEFT MOTOR STRONGER will drive the boat the SAME way as RIGHT: trim '
         + (cancel * 100).toFixed(1) + '% exceeds the ' + (delta * 100).toFixed(1)
         + '% split, so the nominal left run is reversed. Use more than '
         + (cancel * 100).toFixed(1) + '%.';
  } else if (cancel > 0 && delta < 2 * cancel) {
    warn = 'split ' + (delta * 100).toFixed(1) + '% is close to the '
         + (cancel * 100).toFixed(1) + '% the trim cancels -- the LEFT run will be '
         + 'weak and the two directions very lopsided. Above '
         + (2 * cancel * 100).toFixed(1) + '% is clearer.';
  }
  $('bench-warn').textContent = warn;
}
$('bench-throttle').addEventListener('input', refreshBenchPreview);
$('bench-delta').addEventListener('input', refreshBenchPreview);
refreshBenchPreview();
// Runtime switch, deliberately not a rebuild: both arms of the A/B must run
// the same binary. OFF is the control arm and the default.
var pAssistOn = false;       // confirmed boat state, never merely requested state
var pAssistPending = false;
$('p-assist').addEventListener('click', async () => {
  if (pAssistPending) return;
  const want = !pAssistOn;
  const r = await api('/api/assist', 'POST', { p_on: want });
  if (r && r.ok) {
    pAssistPending = true;
    $('p-assist').disabled = true;
    $('p-assist').textContent = 'P ASSIST: WAITING FOR BOAT ' + (want ? 'ON' : 'OFF');
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
    mapUpdate(t, s.gps_track_points);

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

    // The button follows MotorStatus, not HTTP success. HTTP success means the
    // laptop wrote a request; only the echoed request/state proves the boat
    // applied it.
    pAssistPending = !!s.assist_mode_pending;
    if (mst && mst.have && !mst.stale) pAssistOn = !!mst.assist_motor_p;
    const pButton = $('p-assist');
    const pWant = pAssistPending ? !!s.assist_mode_want_p : pAssistOn;
    pButton.disabled = pAssistPending;
    pButton.textContent = pAssistPending
      ? 'P ASSIST: WAITING FOR BOAT ' + (pWant ? 'ON' : 'OFF')
      : 'P ASSIST: ' + (pAssistOn ? 'ON' : 'OFF');
    pButton.classList.toggle('up', pAssistOn && !pAssistPending);

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
          + (BENCH_SD_CHAR[bn.kind] || '?') + '_'
          + (bn.file_index < 10 ? '0' : '') + bn.file_index + '.CSV';
      } else if (!bn.have) {
        $('bench-file').textContent = '--';
      }
      // What the BOAT reports, next to what we asked for. A mismatch voids
      // the A/B, so it is shown rather than assumed.
      if (bn.have) {
        var bp = (mst && mst.have && !mst.stale) ? !!mst.assist_motor_p : !!bn.p_on;
        $('p-confirm').textContent = 'boat: ' + (bp ? 'ON' : 'OFF');
        $('p-confirm').classList.toggle('up', bp);
        $('p-confirm').classList.toggle('warn', bp !== pAssistOn);
      }
      $('bench-learn-c').textContent =
        (bn.have && typeof bn.learn_c === 'number' && bn.learn_c > 0)
          ? bn.learn_c.toFixed(3) : '--';
      // Feed the boat's REAL c back into the preview, so what is shown before
      // and during a run is what the jets are actually going to get -- not the
      // flashed default the page started with.
      if (bn.have && typeof bn.learn_c === 'number' && bn.learn_c > 0
          && (bn.learn_c !== benchLearnC || !benchLearnCFromBoat)) {
        benchLearnC = bn.learn_c;
        benchLearnCFromBoat = true;   // no longer a guess
        refreshBenchPreview();
      }
    }

    // The laptop-side CSV of the last completed run. Outside the bench_yaw
    // guard on purpose: a run can be written and still have no usable summary,
    // and the file is exactly what you want to open when that happens.
    $('bench-csv').textContent = s.bench_csv || '--';

    // ---- UI-observed yaw summary of the last completed drive phase ----------
    const by = s.bench_yaw;
    if (by && by.have) {
      $('bench-yaw-rate').textContent =
        sgn2(by.mean_yaw_dps) + ' / ' + sgn2(by.peak_yaw_dps) + ' °/s';
      $('bench-yaw-angle').textContent = sgn2(by.yaw_angle_deg) + ' °';
      $('bench-yaw-hdg').textContent =
        by.heading_ok ? (sgn2(by.heading_change_deg) + ' °') : '--';
      $('bench-yaw-n').textContent =
        by.n + ' over ' + by.span_s.toFixed(1) + 's'
        + (by.aborted ? '  ABORTED' : '')
        + (by.stale ? '  GAPPY (' + by.max_gap_s.toFixed(2) + 's)' : '')
        + (by.incomplete ? '  INCOMPLETE' : '');
      // Loud when the sample set does not actually cover the run: a confident
      // -8.3 deg/s from four frames is worse than no number at all.
      $('bench-yaw-n').classList.toggle('warn', !!(by.incomplete || by.stale));
      $('bench-yaw-note').textContent = by.aborted
        ? 'Run ABORTED before the drive phase finished — the boat saved no '
          + 'file. These numbers describe part of a turn; discard them.'
        : ((by.incomplete || by.stale)
            ? 'UI-observed / approximate, and this one is patchy — read the '
              + "boat's SD CSV, which is authoritative."
            : "UI-observed / approximate — the boat's SD CSV is authoritative.");
    } else if (typeof s.bench_yaw_live === 'number' && s.bench_yaw_live > 0) {
      $('bench-yaw-n').textContent = s.bench_yaw_live + ' collecting…';
      $('bench-yaw-rate').textContent = '--';
      $('bench-yaw-angle').textContent = '--';
      $('bench-yaw-hdg').textContent = '--';
    }

    // The BOAT's own mode, next to what we asked for. A mismatch voids the
    // run, so it is shown rather than assumed -- same rule as motor P.
    const mstA = s.motor_status;
    if (mstA && mstA.have) {
      const ba = !!mstA.assist_rudder;
      $('rt-assist-confirm').textContent = 'boat: ' + (ba ? 'ON' : 'OFF');
      $('rt-assist-confirm').classList.toggle('up', ba);
      $('rt-assist-confirm').classList.toggle('warn', ba !== assistRudderOn);
    }

    // ---- rudder test ------------------------------------------------------
    renderLakeId(s);                 // before the rudder-test block: it sets _lakeActive for the lockout
    const rt = s.rudder_test, rtr = s.rudder_test_result;
    const rtPill = $('rt-pill');
    if (rt && rt.active) {
      rtPill.textContent = rt.awaiting_assist
        ? 'WAITING FOR ASSIST'
        : (rt.sign < 0 ? '-' : '+') + rt.pct + ' ' + rt.phase.toUpperCase();
      rtPill.classList.add('up'); rtPill.classList.remove('stale');
      $('rt-phase').textContent = rt.awaiting_assist
        ? 'waiting for the boat to confirm Assisted Steering...'
        : rt.phase + '  ' + rt.elapsed_s.toFixed(1) + ' / ' + rt.total_s.toFixed(1) + 's';
      $('rt-file').textContent = rt.name;
      $('rt-frames').textContent = rt.frames + ' collecting...';
      setRudderTestLockout(true);
    } else {
      setRudderTestLockout(false);
      rtPill.textContent = rtr ? (rtr.aborted ? 'ABORTED' : 'DONE') : 'IDLE';
      rtPill.classList.remove('up');
      rtPill.classList.toggle('stale', !!(rtr && rtr.aborted));
      if (rtr) {
        $('rt-phase').textContent =
          (rtr.sign < 0 ? '-' : '+') + rtr.pct + '  ' + rtr.duration_s.toFixed(2) + 's'
          + (rtr.aborted ? '  ABORTED: ' + rtr.abort_reason : '');
        $('rt-file').textContent = rtr.name;
        $('rt-frames').textContent =
          rtr.frames + ' (' + rtr.drive_frames + ' in drive)'
          + '  first +' + rtr.drive_first_delay_s.toFixed(2) + 's'
          + '  tail ' + rtr.drive_tail_gap_s.toFixed(2) + 's'
          + '  maxgap ' + rtr.drive_max_gap_s.toFixed(2) + 's'
          + (rtr.incomplete ? '  INCOMPLETE' : '');
        $('rt-frames').classList.toggle('warn', !!(rtr.incomplete || rtr.gap));
        if (rtr.write_error) $('rt-msg').textContent = rtr.write_error;
      }
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

// ---- map -------------------------------------------------------------------
// The boat on a real map, live. Leaflet and the tiles come from the internet;
// the library is loaded HERE, when the page runs, never from <head>, so a
// laptop with no connection still gets the page at once and the map box says
// what is missing. The trail is kept by the Python side (/api/track); this
// page mirrors it and applies the same distance rule between fetches.
const PAGE_CONFIG = __PAGE_CONFIG__;
const MAP_LAYERS = {
  osm: { url: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png', maxZoom: 19,
         attribution: '&copy; OpenStreetMap contributors' },
  satellite: { url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
               maxZoom: 19, attribution: 'Tiles &copy; Esri' },
  dark: { url: 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png', subdomains: 'abcd',
          maxZoom: 20, attribution: '&copy; OpenStreetMap contributors &copy; CARTO' },
};
// Fallback order when a layer's SERVER is unreachable. tile.openstreetmap.org
// is blackholed on some networks (it resolved to 127.0.0.1 on the dev box)
// while Esri answers -- and a phone hotspot in the field may or may not reach
// either. A dead layer must not leave the map grey with "no internet?" when a
// working one is a click away; a manual pick resets the chain.
const MAP_LAYER_FALLBACK = ['osm', 'satellite', 'dark'];
const MAP_LAYER_LABEL = { osm: 'OpenStreetMap', satellite: 'Satellite (Esri)', dark: 'Dark (Carto)' };
const MAP_FIRST_FIX_ZOOM = 17;      // a lake or a pool: a few hundred metres across
const MAP_HOME = [16.5, 106.5];     // Vietnam, until the boat says where it is
const MAP_TRAIL_MAX = PAGE_CONFIG.trail_max_points;
const MAP_TRAIL_MIN_MOVE_M = PAGE_CONFIG.trail_min_move_m;
const mapState = { map: null, marker: null, trail: null, layer: null, ready: false,
                   loading: false, shown: true, follow: true, hasFix: false, last: null,
                   lastHeading: null, markerStale: false, tilesFailing: false,
                   trailPts: [], serverPts: 0, fetchingTrail: false };

// localStorage is a convenience, never a dependency: missing, full or
// throwing (some private windows do) all fall back to the defaults.
function storeGet(key, fallback) {
  try {
    const v = localStorage.getItem(key);
    return v === null ? fallback : v;
  } catch (e) { return fallback; }
}
function storeSet(key, value) {
  try { localStorage.setItem(key, value); } catch (e) { /* no storage: fine */ }
}

function mapNote(text) {
  const n = $('map-note');
  // A fallback in force is always worth saying, alongside whatever else.
  const extra = mapState.fallbackNote || '';
  const shown = text ? (text + (extra ? '  ·  ' + extra : '')) : extra;
  n.textContent = shown;
  n.hidden = !shown;
}

// What the box in the middle of the map should say right now, derived from
// state rather than from whichever event happened to fire last.
function mapRefreshNote() {
  if (mapState.tilesFailing) {
    mapNote('map tiles are not loading — no internet?');
  } else if (!mapState.hasFix) {
    mapNote((mapState.last ? 'last known position — ' : '')
            + 'waiting for a GPS fix (indoors? hide the map with MAP, top right)');
  } else {
    mapNote('');
  }
}

// MAP in the top bar. A bench session has no GPS, so the map can get out of
// the way: with it hidden the two columns sit side by side.
function mapSetShown(on) {
  mapState.shown = !!on;
  $('map-panel').hidden = !mapState.shown;
  $('layout').classList.toggle('no-map', !mapState.shown);
  $('map-toggle').classList.toggle('on', mapState.shown);
  storeSet('espnow.map.shown', mapState.shown ? '1' : '0');
  // Leaflet sized itself while the box was display:none (0 x 0); tell it.
  if (mapState.shown && mapState.ready) mapState.map.invalidateSize();
}

function mapBoot() {
  mapSetShown(storeGet('espnow.map.shown', '1') !== '0');
  if (mapState.ready || mapState.loading) return;
  if (typeof L !== 'undefined') { mapInit(); return; }
  mapState.loading = true;
  mapNote('loading map…');
  const css = document.createElement('link');
  css.rel = 'stylesheet';
  css.href = PAGE_CONFIG.leaflet_css;
  css.integrity = PAGE_CONFIG.leaflet_css_sri;
  css.crossOrigin = '';
  document.head.appendChild(css);
  const js = document.createElement('script');
  js.src = PAGE_CONFIG.leaflet_js;
  js.integrity = PAGE_CONFIG.leaflet_js_sri;
  js.crossOrigin = '';
  let settled = false;
  js.onload = () => { if (settled) return; settled = true; mapState.loading = false; mapInit(); };
  js.onerror = () => { if (settled) return; settled = true; mapFail(); };
  setTimeout(() => { if (!settled) { settled = true; mapFail(); } }, 20000);
  document.head.appendChild(js);
}

function mapFail() {
  mapState.loading = false;
  mapNote('map not available: Leaflet did not load from ' + PAGE_CONFIG.leaflet_js
          + ' (no internet?). Everything else works. Centre = try again.');
}

function mapInit() {
  if (mapState.ready) return;
  const map = L.map('map', { zoomControl: true, attributionControl: true });
  mapState.map = map;
  mapSetLayer(storeGet('espnow.map.layer', 'osm'));
  const remembered = mapRememberedPosition();
  mapState.last = remembered;
  mapState.markerStale = true;
  map.setView(remembered || MAP_HOME, remembered ? 16 : 5);
  mapState.trail = L.polyline([], { color: '#00BFFF', weight: 3, opacity: 0.85 }).addTo(map);
  mapState.marker = L.marker(remembered || MAP_HOME, {
    icon: boatIcon(0, true), opacity: remembered ? 1 : 0,
    interactive: false, zIndexOffset: 1000 }).addTo(map);
  // Dragging the map is the operator saying "let me look around": follow
  // switches itself off. Zooming does not -- you still want the boat centred.
  map.on('dragstart', () => mapSetFollow(false));
  mapState.ready = true;
  mapRefreshNote();
  mapFetchTrail();
}

function mapRememberedPosition() {
  const parts = storeGet('espnow.map.last', '').split(',');
  if (parts.length !== 2) return null;
  const lat = Number(parts[0]), lon = Number(parts[1]);
  if (!Number.isFinite(lat) || !Number.isFinite(lon) || (lat === 0 && lon === 0)) return null;
  return [lat, lon];
}

function mapSetLayer(key, opts) {
  opts = opts || {};
  if (!MAP_LAYERS[key]) key = 'osm';
  const def = MAP_LAYERS[key];
  if (mapState.layer) mapState.map.removeLayer(mapState.layer);
  mapState.tilesFailing = false;
  mapState.layerKey = key;
  mapState.fallbackNote = opts.fallbackNote || null;
  const layer = L.tileLayer(def.url, { subdomains: def.subdomains || 'abc',
                                       maxZoom: def.maxZoom, attribution: def.attribution });
  // "load" fires once every visible tile is done, failed ones included, so
  // count the failures per round rather than clearing the note on "load".
  let failed = 0, loaded = 0;
  layer.on('loading', () => { failed = 0; loaded = 0; });
  layer.on('tileerror', () => { failed += 1; });
  layer.on('tileload', () => { loaded += 1; });
  layer.on('load', () => {
    if (failed > 0 && loaded === 0) {
      // Every tile of the round failed and none arrived: the server is
      // unreachable, not a tile. Try the next layer once instead of sitting
      // on a grey map.
      const tried = mapState.triedLayers || (mapState.triedLayers = []);
      if (!tried.includes(key)) tried.push(key);
      const next = MAP_LAYER_FALLBACK.find((k) => !tried.includes(k));
      if (next) {
        mapSetLayer(next, { fallbackNote: MAP_LAYER_LABEL[key] + ' tiles unreachable — using '
                                          + MAP_LAYER_LABEL[next], noPersist: true });
        mapRefreshNote();                 // say so now, not after the next tile round
        return;
      }
    }
    mapState.tilesFailing = failed > 0;
    mapRefreshNote();
  });
  layer.addTo(mapState.map);
  mapState.layer = layer;
  $('map-layer').value = key;
  if (!opts.noPersist) storeSet('espnow.map.layer', key);   // a fallback is not a preference
}

function boatIcon(heading, stale) {
  const h = Number.isFinite(heading) ? Math.round(heading) : 0;
  return L.divIcon({
    className: 'boat-icon', iconSize: [26, 26], iconAnchor: [13, 13],
    html: '<svg class="boat-arrow' + (stale ? ' stale' : '') + '" viewBox="0 0 24 24"'
        + ' style="transform: rotate(' + h + 'deg)">'
        + '<path d="M12 2 L19 21 L12 17 L5 21 Z" fill="#00BFFF" stroke="#05080F"'
        + ' stroke-width="1.5"></path></svg>' });
}

function mapSetFollow(on) {
  mapState.follow = !!on;
  $('map-follow').checked = mapState.follow;
}

// Called on every status push (20 Hz). The readout strip always updates; the
// marker and trail only once Leaflet is up.
function mapUpdate(t, serverPts) {
  const have = !!(t && t.have);
  const fix = have && !!t.gps_valid && Number.isFinite(t.lat) && Number.isFinite(t.lon)
              && !(t.lat === 0 && t.lon === 0);
  const pill = $('map-fix');
  if (!have) {
    pill.textContent = 'NO DATA';
    pill.classList.remove('up', 'stale');
  } else if (t.stale) {
    pill.textContent = 'STALE ' + t.age_s.toFixed(0) + 's';
    pill.classList.remove('up'); pill.classList.add('stale');
  } else if (!fix) {
    pill.textContent = 'NO FIX';
    pill.classList.remove('up'); pill.classList.add('stale');
  } else {
    pill.textContent = 'FIX';
    pill.classList.remove('stale'); pill.classList.add('up');
  }
  $('map-pos').textContent = fix ? t.lat.toFixed(6) + ', ' + t.lon.toFixed(6) : '--';
  $('map-speed').textContent = fix
    ? t.speed_mps.toFixed(1) + ' m/s (' + (t.speed_mps * 3.6).toFixed(1) + ' km/h)' : '--';
  $('map-course').textContent = fix ? t.course_deg.toFixed(0) + '°' : '--';
  $('map-heading').textContent = have ? t.heading.toFixed(0) + '°' : '--';
  $('map-sats').textContent = have ? t.satellites + ' / ' + t.hdop.toFixed(1) : '--';
  if (Number.isInteger(serverPts)) mapSyncTrail(serverPts);
  if (!mapState.ready) return;
  if (!fix) {
    // Keep the boat where it was: the last known position is still the best
    // guess, just dimmer. Never jump to 0,0.
    if (mapState.last && !mapState.markerStale) {
      mapState.marker.setIcon(boatIcon(mapState.lastHeading, true));
      mapState.markerStale = true;
    }
    return;
  }
  const stale = !!t.stale;
  const ll = [t.lat, t.lon];
  mapState.marker.setLatLng(ll);
  if (mapState.lastHeading === null || stale !== mapState.markerStale
      || Math.abs(t.heading - mapState.lastHeading) >= 2) {
    mapState.marker.setIcon(boatIcon(t.heading, stale));
    mapState.lastHeading = t.heading;
    mapState.markerStale = stale;
  }
  if (!mapState.last) mapState.marker.setOpacity(1);
  mapState.last = ll;
  if (!mapState.hasFix) {
    mapState.hasFix = true;
    mapState.map.setView(ll, MAP_FIRST_FIX_ZOOM);
    mapRefreshNote();
  } else if (mapState.follow) {
    mapState.map.panTo(ll, { animate: false });
  }
  if (!stale) mapTrailAppend(t.lat, t.lon);
}

function geoDistanceM(lat1, lon1, lat2, lon2) {
  const k = 111320;                 // metres per degree of latitude
  const dy = (lat2 - lat1) * k;
  const dx = (lon2 - lon1) * k * Math.cos((lat1 + lat2) / 2 * Math.PI / 180);
  return Math.sqrt(dx * dx + dy * dy);
}

function mapTrailAppend(lat, lon) {
  const pts = mapState.trailPts;
  if (pts.length) {
    const p = pts[pts.length - 1];
    if (geoDistanceM(p[0], p[1], lat, lon) < MAP_TRAIL_MIN_MOVE_M) return;
  }
  pts.push([lat, lon]);
  if (pts.length > MAP_TRAIL_MAX) {
    pts.splice(0, pts.length - MAP_TRAIL_MAX);
    mapState.trail.setLatLngs(pts);
  } else {
    mapState.trail.addLatLng([lat, lon]);
  }
  $('map-trail').textContent = pts.length + ' pts';
  storeSet('espnow.map.last', lat.toFixed(6) + ',' + lon.toFixed(6));
}

async function mapFetchTrail() {
  if (!mapState.ready || mapState.fetchingTrail) return;
  mapState.fetchingTrail = true;
  try {
    const r = await api('/api/track', 'GET');
    if (!r || !Array.isArray(r.points)) return;
    mapState.trailPts = r.points.slice();
    mapState.serverPts = r.points.length;
    mapState.trail.setLatLngs(mapState.trailPts);
    $('map-trail').textContent = mapState.trailPts.length + ' pts';
  } catch (e) {
    // the tool was unreachable for a moment; the next status asks again
  } finally {
    mapState.fetchingTrail = false;
  }
}

function mapSyncTrail(serverPts) {
  // The trail lives in the Python process; this page only mirrors it. A count
  // that went DOWN means it was cleared (or the tool restarted); one well
  // ahead of ours means this page missed points (the map loaded late).
  if (!mapState.ready) return;
  if (serverPts < mapState.serverPts || serverPts > mapState.trailPts.length + 20) {
    if (mapState.fetchingTrail) return;        // ask again on the next status
    mapFetchTrail();
  }
  mapState.serverPts = serverPts;
}

$('map-toggle').addEventListener('click', () => mapSetShown(!mapState.shown));
$('map-layer').addEventListener('change', (e) => {
  mapState.triedLayers = [];                      // a manual pick restarts the chain
  if (mapState.ready) mapSetLayer(e.target.value);
  else storeSet('espnow.map.layer', e.target.value);
});
$('map-follow').addEventListener('change', (e) => mapSetFollow(e.target.checked));
$('map-center').addEventListener('click', () => {
  if (!mapState.ready) { mapBoot(); return; }     // doubles as "try loading again"
  mapSetFollow(true);
  if (mapState.last) mapState.map.setView(mapState.last, Math.max(mapState.map.getZoom(), 16));
});
$('map-clear').addEventListener('click', async () => {
  mapState.trailPts = [];
  mapState.serverPts = 0;
  if (mapState.ready) mapState.trail.setLatLngs([]);
  $('map-trail').textContent = '0 pts';
  await api('/api/track/clear', 'POST', {});
});

// ---- folding cards ----------------------------------------------------------
// The two tests and the two diagnostics fold under their titles so the Drive
// card and the map get the screen. Nothing is removed -- the title pill keeps
// updating while folded -- and each card remembers how you left it.
const FOLDING_CARDS = ['bench-card', 'rudder-test-card', 'sensors-card', 'bridge-card'];
function foldingCardsInit() {
  for (const id of FOLDING_CARDS) {
    const card = $(id);
    const remembered = storeGet('espnow.card.' + id, null);
    if (remembered !== null) card.open = remembered === '1';
    card.addEventListener('toggle', () => storeSet('espnow.card.' + id, card.open ? '1' : '0'));
  }
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
foldingCardsInit();
mapBoot();
</script>
</body>
</html>
"""

# What the page needs to know from this side, in one place, so the two never
# drift apart: the trail rule it applies between fetches, and the pinned
# Leaflet files with their hashes.
PAGE_CONFIG = {
    'leaflet_js': LEAFLET_JS_URL, 'leaflet_js_sri': LEAFLET_JS_SRI,
    'leaflet_css': LEAFLET_CSS_URL, 'leaflet_css_sri': LEAFLET_CSS_SRI,
    'trail_max_points': GPS_TRACK_MAX_POINTS,
    'trail_min_move_m': GPS_TRACK_MIN_MOVE_M,
}
PAGE = _PAGE_TEMPLATE.replace('__PAGE_CONFIG__', json.dumps(PAGE_CONFIG))


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
            self.link.ensure_lake_id_next()      # disk scan, before status() takes the lock
            self._json(self.link.status())
        elif self.path == '/api/ports':
            import serial.tools.list_ports as list_ports
            ports = [{'device': p.device, 'description': p.description}
                     for p in list_ports.comports()]
            self._json({'ports': ports})
        elif self.path == '/api/track':
            self._json(self.link.track_snapshot())
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
                self.link.ensure_lake_id_next()          # no-op once cached; never under the lock
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
        elif self.path == '/api/track/clear':
            self.link.track_clear()
            self._json({'ok': True})
        elif self.path == '/api/session':
            # Claim control. Supersedes any existing session and starts from a
            # stopped boat.
            self._json({'ok': True,
                        'session_id': self.link.open_control_session(),
                        'heartbeat_hz': CONTROL_HEARTBEAT_HZ,
                        'lease_s': CONTROL_LEASE_S})
        elif self.path == '/api/release':
            # The browser is going away (hidden, navigating, closing). Zero the
            # controls and drop the session, but WITHOUT STOP's other effects:
            # STOP also kills a running calibration, and merely switching tabs
            # must not abort one.
            self.link.release_control_session()
            self._json({'ok': True})
        elif self.path == '/api/state':
            # The browser's full-state heartbeat. Carries the session id and a
            # monotonic seq, so a delayed request cannot apply out of order and
            # one from a superseded session cannot apply at all.
            ok, err = self.link.control_heartbeat(
                body.get('session_id'), body.get('seq'),
                throttle=body.get('throttle'), rudder=body.get('rudder'),
                left=body.get('left'), right=body.get('right'),
                split=body.get('split'))
            if ok:
                self._json({'ok': True})
            else:
                message, code = err
                self._json({'ok': False, 'error': message, 'code': code}, 409)
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
            ok, err = self.link.arm(bool(body.get('arm')),
                                    bool(body.get('force')))
            self._json({'ok': ok} if ok else {'ok': False, 'error': err},
                       200 if ok else 409)
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
        elif self.path == '/api/rudder_test':
            command_seq = body.get('seq')
            if (not isinstance(command_seq, int) or isinstance(command_seq, bool) or
                    command_seq < 0):
                self._json({'ok': False, 'error': 'seq must be a nonnegative integer'}, 400)
                return
            sign = body.get('sign')
            if sign not in (-1, 1) or isinstance(sign, bool):
                self._json({'ok': False, 'error': 'sign must be -1 or +1'}, 400)
                return
            assisted = body.get('assisted', False)
            if not isinstance(assisted, bool):
                self._json({'ok': False, 'error': 'assisted must be a boolean'}, 400)
                return
            # Returns immediately: the 4.5 s sequence runs on the stream loop,
            # never in this handler.
            ok, err = self.link.start_rudder_test(sign, command_seq, assisted)
            self._json({'ok': ok, 'error': err} if not ok else {'ok': True})
        elif self.path == '/api/lake_id':
            command_seq = body.get('seq')
            if (not isinstance(command_seq, int) or isinstance(command_seq, bool) or
                    command_seq < 0):
                self._json({'ok': False, 'error': 'seq must be a nonnegative integer'}, 400)
                return
            notes = body.get('notes') or {}
            if not isinstance(notes, dict):
                self._json({'ok': False, 'error': 'notes must be an object'}, 400)
                return
            # Returns immediately: the 57 s profile runs on the stream loop.
            ok, err = self.link.start_lake_id(
                body.get('throttle'), body.get('magnitude'), command_seq,
                notes=notes, firmware_label=body.get('firmware_label'),
                profile=body.get('profile') or 'full')
            self._json({'ok': ok, 'error': err} if not ok else {'ok': True}, 200 if ok else 409)
        elif self.path == '/api/rudder_assist':
            on = body.get('on')
            if not isinstance(on, bool):
                self._json({'ok': False, 'error': 'on must be true or false'}, 400)
                return
            ok, err = self.link.send_rudder_assist(on)
            self._json({'ok': ok, 'error': err} if not ok else {'ok': True})
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
    ap.add_argument('--raw-throttle-test', action='store_true',
                    help='use CONFIG_ESC_RAW_THROTTLE_TEST firmware; lake runs require '
                         'Motor P OFF and record the raw diagnostic mode')
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
    link = BoatLink(boat_pb2, send_hz=args.hz, raw_throttle_test=args.raw_throttle_test)
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
