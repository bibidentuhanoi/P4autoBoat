# ESP-NOW Telemetry Foundation — Design

**Date:** 2026-08-06
**Branch:** `feat/winch-servo-web-control`
**Status:** Approved design, pending implementation plan

---

## 1. Context & Reframe

The goal as stated: *"build the foundation of ESP-NOW telemetry, keep the WiFi
infrastructure, build ESP-NOW alongside so in the field with no WiFi it falls
back to ESP-NOW."*

Investigation this session established that **the failover already exists and
works**. This project is therefore NOT "build failover" — it is "close the one
gap that makes the ground station deaf, and deliberately trim the field-mode
payload."

### What already works (verified — do NOT modify)

- **Boot-time failover** (`main/main.c:222-236`): after `wifi_init()`, under
  `#if CONFIG_ESPNOW_ENABLED`, `if (wifi_ret != ESP_OK)` calls
  `espnow_transport_init()`; otherwise the WS/HTTP servers start. Mode is chosen
  once at boot and fixed for the session.
- **Automatic telemetry fan-out**: `espnow_transport_init()` calls
  `pipeline_register_transport(espnow_send_fn, NULL)`
  (`main/transports/espnow_transport.c:222`). Thereafter
  `pipeline_publish_sensors()` (20 Hz) and `pipeline_publish_status()` (~1 Hz)
  fan out to ESP-NOW with no additional code.
- **The full radio relay chain**: P4 → C6 (`esp_hosted_send_custom_data`,
  `PEER_MSG_VIDEO`) → C6 fragments into 244-byte ESP-NOW packets → air → S3
  reassembles → COBS-frames over USB CDC → laptop.
- **`visualize.py` receive path**: `serial_reader()` accumulates to the `0x00`
  COBS delimiter, `cobs_decode()`s, `handle_serial_packet()` parses the 4-byte
  `espnow_pkt_hdr_t` and dispatches by `msg_type`. JPEG display path works.

### The single blocking gap

`visualize.py:171` does `from proto import boat_pb2`. The `proto/` directory
**does not exist** in the repo (only the nanopb-generated *C* files exist). So
every `MSG_SENSOR` packet arrives correctly, hits `except ImportError: return`
in `parse_sensor_snapshot()`, and is silently discarded. The boat has been
transmitting valid telemetry the whole time; nothing on the ground could decode
it.

---

## 2. Goals & Non-Goals

### Goals

1. Make ESP-NOW telemetry decodable end-to-end on the laptop (generate the
   missing Python protobuf binding).
2. Prevent recurrence of the schema-drift class of bug (a single regeneration
   entry point).
3. Trim the field-mode snapshot to fit the ESP-NOW link budget without
   diverging the schema or the code path.

### Non-Goals (explicit — do not let these creep in)

- **Inbound control** (laptop → boat commands). Separate sub-project; carries
  the broadcast asymmetry and missing reliability primitives.
- **The WS-keyed failsafe fix** (`motor_control.c` watchdog keys on
  `ws_transport_client_count()`). Belongs to the control sub-project; telemetry
  arms nothing.
- **JPEG / video over ESP-NOW** (the orphaned `espnow_transport_send_jpeg`).
  Later.
- **Reliability primitives** — sequence-number checking, `esp_now_register_send_cb`,
  application ACK/retry, unicast. Best-effort is correct for a 20 Hz telemetry
  stream; a dropped snapshot costs 50 ms, not safety.
- **Simultaneous WiFi + ESP-NOW / runtime switching.** Excluded by the
  boot-time-only decision.

---

## 3. The Boot-Time Invariant

Because the transport is chosen once at boot and never switches, **the mode is
fixed for the entire session.** This eliminates dual-encoding, per-transport
payload negotiation, and mode-flapping. Every simplification below depends on
it.

---

## 4. Change 1 — Python binding + anti-drift regeneration script

### Problem

`proto/boat_pb2.py` does not exist; the laptop cannot decode any protobuf.
Generating it also introduces a **fourth** copy of the schema, raising
drift risk (the hand-mirrored `dashboard.html` `protoSchema` drifting once caused
the 2026-07 UI lockout, because protobuf.js drops unknown fields silently).

Schema representations after this change:

| # | Representation | Generator | Committed |
|---|----------------|-----------|-----------|
| 1 | `main/proto/boat.proto` | source of truth | yes |
| 2 | `main/proto/boat.pb.c` / `.h` | nanopb 0.4.9.1 + `boat.options` | yes |
| 3 | `main/dashboard.html` `protoSchema` | hand-mirrored protobuf.js | yes |
| 4 | `proto/boat_pb2.py` (new) | `grpc_tools.protoc` | yes |

### Build: `tools/gen_proto.sh`

A single script that regenerates the machine-generated copies (#2 and #4) from
the one source (#1):

- **nanopb C** (`boat.pb.c` / `.h`): via `nanopb_generator`
  (`/opt/esp/python_env/idf5.4_py3.12_env/bin/nanopb_generator`, in the ESP-IDF
  Python env), which auto-consumes `main/proto/boat.options` for field-size caps.
- **Python** (`proto/boat_pb2.py` + `proto/__init__.py`): via
  `python3 -m grpc_tools.protoc` from the repo venv (`grpc_tools` present,
  `protobuf 6.33.6`).
- The two generators live in **different Python environments** — the script
  invokes each with its correct interpreter/binary and fails loudly if either is
  missing.
- On completion, prints a reminder that `dashboard.html`'s `protoSchema` (#3, the
  one copy that cannot be auto-generated) must be hand-mirrored if `boat.proto`
  changed.

### Behavior & safety

- Idempotent: re-running with an unchanged `.proto` must reproduce byte-identical
  output.
- The script verifies its output files exist after generation; no half-generated
  state.
- **Implementation commits only `proto/boat_pb2.py` + `proto/__init__.py`.**
  The regenerated `boat.pb.c` / `.h` are expected to be byte-identical to the
  committed versions (same generator version, same options); the implementation
  verifies `git diff` is clean on those two files. If they differ, that is a
  drift signal to investigate, not to auto-commit.

---

## 5. Change 2 — Trim the field-mode payload at source

### Mechanism

- A boot-set global flag `g_field_mode` (declared `extern` in `include/common.h`,
  defined in `main.c`), default `false`.
- Set in `main.c` from `(wifi_ret != ESP_OK)` in the ESP-NOW branch, **before**
  `xTaskCreate(task_sensor_snapshot, ...)`. Boot order guarantees the flag is set
  before the snapshot task starts — no read-before-write race.
- `task_sensor_snapshot` (`sensor_task.c`) reads the flag. When `true`, it skips
  populating the three diagnostic repeated fields on **both** ToF grids:
  `sigma`, `target_status`, `nb_target_detected`. It still populates `valid` and
  `distances` (the actual obstacle data), plus `imu`, `gps`, and `detections`.

### Why this shrinks the wire with no schema change

In proto3, a `repeated` field with zero elements occupies **zero bytes** on the
wire — it is absent, not "encoded empty." Leaving those arrays unpopulated
shrinks the encoded `SensorSnapshot` by ~40–50% (≈1 KB → ≈0.5 KB, ≈5 air
fragments → ≈3) with **no** `.proto` edit and **no** regeneration.

### One decoder for both

The same `boat_pb2.py` decodes both payloads: the full snapshot (WiFi) returns
the arrays populated; the trimmed snapshot (ESP-NOW) returns those three fields
as empty lists. No separate "small" schema, no second binding.

### Scope note

In field mode there is no HTTP server running, so the JSON `/api/snapshot`
endpoint being trimmed is irrelevant (it does not run in that mode).

---

## 6. Data Flow (unchanged except payload size)

```
task_sensor_snapshot builds SensorSnapshot
   └─[if g_field_mode: omit sigma/target_status/nb_target on tof_a, tof_b]
      └─> pipeline_publish_sensors()
           └─> nanopb encode (once)
                └─> espnow_send_fn: prepend espnow_pkt_hdr_t (MSG_SENSOR)
                     └─> esp_hosted_send_custom_data(PEER_MSG_VIDEO, ...)
                          └─> C6: fragment into 244B ESP-NOW packets ── air ──>
                               └─> S3: reassemble + COBS + 0x00 ── USB CDC ──>
                                    └─> visualize.py: accumulate→COBS-decode→
                                         parse hdr → boat_pb2 decode  ◄── NEW working link
```

The only newly-functional link is the final `boat_pb2` decode. Everything left
of it already runs today.

---

## 7. Error Handling

- **`gen_proto.sh`**: exits non-zero with a clear message if `nanopb_generator`
  or `grpc_tools` is unavailable; verifies each expected output file exists
  before exiting 0; never leaves a partially-written binding.
- **`g_field_mode`**: defaults to `false` (full, safe payload). Only set `true`
  on confirmed WiFi failure. Set before the consuming task starts (no race).
- **`visualize.py`**: retains its existing `try/except` around
  `parse_sensor_snapshot`, so a corrupt frame (e.g. from a dropped air fragment)
  is skipped, never fatal to the reader loop.

---

## 8. Testing

### Host-only (automatable, but PROXY only — not proof of field function)

1. Run `tools/gen_proto.sh`; assert `proto/boat_pb2.py` and `proto/__init__.py`
   exist and `import`s cleanly under the repo venv.
2. Round-trip: build a `BoatMessage` with a `SensorSnapshot` in Python, serialize,
   deserialize, assert field equality.
3. Trim assertion: decode a trimmed-style message; assert `sigma`,
   `target_status`, `nb_target_detected` are empty while `distances`, `imu`,
   `gps` are populated.
4. Assert `git diff` is clean on `main/proto/boat.pb.c` and `boat.pb.h` after
   running the script (drift check).

### Hardware acceptance (the ONLY proof that counts)

Per standing project rule, nothing is "done" from build-green / schema
round-trip alone — those are "compiles + logic present, UNVERIFIED ON HARDWARE"
until a bench test on real ESP32-P4:

- Boot the boat with **no** WiFi AP reachable.
- Run `python visualize.py --serial /dev/ttyACM0`.
- **Pass = the IMU horizon/compass animates and both ToF grids populate** from
  live ESP-NOW telemetry.

This gate is the user's to run and sign off.

---

## 9. Files Touched

| File | Change |
|------|--------|
| `tools/gen_proto.sh` | **new** — single regeneration entry point |
| `proto/boat_pb2.py` | **new (generated, committed)** |
| `proto/__init__.py` | **new (empty, committed)** — makes `from proto import boat_pb2` work |
| `include/common.h` | add `extern bool g_field_mode;` |
| `main/main.c` | define `g_field_mode`; set it in the ESP-NOW branch before tasks start |
| `main/sensor_task.c` | gate the three ToF diagnostic arrays on `g_field_mode` |

No changes to: the failover logic, `pipeline.c`, `espnow_transport.c`, the C6/S3
bridges, or `boat.proto` (hence no schema regeneration required by Change 2).

---

## 10. Open Questions

None. Payload-trim scope, failover trigger (boot-time only), and coexistence
mode (idle standby) are all decided.
