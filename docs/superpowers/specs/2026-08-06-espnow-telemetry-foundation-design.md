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
- **Regenerating the committed nanopb C** (`boat.pb.c` / `.h`). Those files are
  already correct and are compiled into the *working WiFi/WS path* as well as
  ESP-NOW. This project does not touch them (see §4); regenerating them is
  deliberately out of scope to protect the working transport.

---

## 3. The Boot-Time Invariant

Because the transport is chosen once at boot and never switches, **the mode is
fixed for the entire session.** This eliminates dual-encoding, per-transport
payload negotiation, and mode-flapping. Every simplification below depends on
it.

Two consequences follow directly and are worth stating explicitly:

- **WiFi telemetry is never trimmed.** WiFi and ESP-NOW are never registered
  simultaneously, so the single snapshot built each 50 ms tick is consumed by
  exactly one transport and `g_field_mode` always matches it. No code path trims
  the WiFi payload.
- **Only payload shrinks, never rate.** Both modes publish at 20 Hz; field mode
  drops three ToF diagnostic arrays per snapshot and nothing else. Trimmed
  ≈500 B × 20 Hz ≈ 80 kbps sits well inside ESP-NOW's ~214–555 kbps budget
  (~3 air fragments per snapshot), so 20 Hz is sustainable, not merely nominal.

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
| 4 | `proto/boat_pb2.py` (new) | `grpc_tools.protoc` **in `.venv`** | yes |

### Toolchain environment (verified 2026-08-06)

Ground truth, checked on this machine (an earlier draft asserted these wrong —
corrected after design review):

| Interpreter | protobuf | `grpc_tools` | Role |
|-------------|----------|--------------|------|
| repo `.venv` | **7.34.1** | **absent** (must be installed) | runs `visualize.py` (has pyserial / PIL / numpy) |
| ESP-IDF env | 6.33.6 | present | builds firmware; **not** the telemetry runtime |
| system `python3` | — | absent | irrelevant |

`nanopb_generator` is 0.4.9.1 in the IDF env, matching the committed C banner,
with no timestamp line in its output.

**The invariant that governs Change 1:** the interpreter that *generates*
`boat_pb2.py` and the interpreter that *runs* `visualize.py` must share the same
protobuf **major** version. protobuf 4+ gencode embeds a runtime-version guard;
generating under 6.x and importing under 7.x can fail to load. The repo `.venv`
(protobuf 7.34.1) is the runtime, so generation must happen there too — not in
the IDF env, despite that being where `grpc_tools` currently lives.

### Build: `tools/gen_proto.sh` (Python-only)

The script regenerates **only** the Python binding (#4). It does **not** touch
the nanopb C (#2): those files are already correct, committed, and compiled into
the working WiFi path, so regenerating them is an unnecessary risk to a working
transport (see §2 non-goals).

- **Prereq:** `grpcio-tools` installed into `.venv` and pinned in
  `requirements.txt`. The script invokes `.venv/bin/python -m grpc_tools.protoc`,
  so generation and the `visualize.py` runtime are the *same* interpreter —
  protobuf gencode/runtime compatibility is satisfied by construction, not hope.
- **Generates:** `proto/boat_pb2.py` + `proto/__init__.py` from
  `main/proto/boat.proto` (output rooted at repo top so `from proto import
  boat_pb2` resolves).
- **Fails loudly** if `grpcio-tools` is missing from `.venv`, printing the exact
  `pip install` line to fix it; verifies the output file exists before exiting 0;
  never leaves a half-written binding.
- **Guards the committed C:** asserts `git diff` is clean on
  `main/proto/boat.pb.c` / `.h` after running — proving the script did not
  perturb the shared firmware protobuf layer.
- **Drift detection, not a reminder:** if `main/proto/boat.proto` is newer
  (mtime) than `main/dashboard.html`, the script warns loudly that the
  hand-mirrored `protoSchema` (#3) may be stale — the exact 2026-07 UI-lockout
  failure mode. A printed reminder alone is the control that already failed once;
  making staleness *detectable* is the point.
- **Fallback:** if no `grpcio-tools` compatible with protobuf 7.x is installable,
  align the other way — pin `.venv` to the protobuf major that an available
  `grpcio-tools` bundles, keeping generate-env == run-env. Either way the §8
  import round-trip under `.venv` is the gate that proves compatibility.

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

### Two implementation guards (from design review)

- **Gate the proto-copy, not the sensor read.** The flag must gate where these
  values are *copied into the snapshot* in `sensor_task.c`, never where they are
  *read from the ToF driver*. Gating the read would starve any other consumer of
  the live arrays and would manifest only in field mode — the hardest mode to
  debug (no dashboard, no WiFi). Confirm during implementation that nothing else
  consumes the live `sigma` / `target_status` / `nb_target_detected`.
- **Fail safe toward the full payload.** `g_field_mode` is defined `false` at
  file scope and set `true` *only* inside the ESP-NOW branch next to
  `espnow_transport_init()`; it can never become `true` when
  `CONFIG_ESPNOW_ENABLED` is off. Setting it true on a successful WiFi boot would
  trim WiFi telemetry and break the dashboard's ToF overlay (which needs full
  grids), so the flag defaults to the WiFi-safe value and the boot log records
  the chosen mode once.

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

- **`gen_proto.sh`**: exits non-zero with the exact `pip install` fix line if
  `grpcio-tools` is missing from `.venv`; verifies `proto/boat_pb2.py` exists
  before exiting 0; asserts it left `boat.pb.c` / `.h` untouched; never leaves a
  partially-written binding.
- **`g_field_mode`**: defaults to `false` (full, safe payload). Only set `true`
  on confirmed WiFi failure. Set before the consuming task starts (no race).
- **`visualize.py`**: retains its existing `try/except` around
  `parse_sensor_snapshot`, so a corrupt frame (e.g. from a dropped air fragment)
  is skipped, never fatal to the reader loop.

---

## 8. Testing

### Host-only (automatable, but PROXY only — not proof of field function)

1. Run `tools/gen_proto.sh`; assert `proto/boat_pb2.py` and `proto/__init__.py`
   exist and **`from proto import boat_pb2` imports cleanly under `.venv`**
   (`.venv/bin/python` — the same interpreter `visualize.py` uses). This import,
   not mere file generation, is the gate for Change 1: it is what proves
   gencode/runtime protobuf compatibility.
2. Round-trip: build a `BoatMessage` with a `SensorSnapshot` in Python, serialize,
   deserialize, assert field equality — under `.venv`.
3. Trim assertion: decode a trimmed-style message; assert `sigma`,
   `target_status`, `nb_target_detected` are empty while `distances`, `imu`,
   `gps` are populated.
4. Assert the script left `main/proto/boat.pb.c` and `boat.pb.h` **untouched**
   (`git diff` clean) — proving Python-only scoping held and the shared firmware
   protobuf layer was not perturbed.

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
| `tools/gen_proto.sh` | **new** — Python-binding regeneration only (does not touch the nanopb C) |
| `requirements.txt` | pin `grpcio-tools` (dev dep for regeneration, installed into `.venv`) |
| `proto/boat_pb2.py` | **new (generated, committed)** |
| `proto/__init__.py` | **new (empty, committed)** — makes `from proto import boat_pb2` work |
| `include/common.h` | add `extern bool g_field_mode;` |
| `main/main.c` | define `g_field_mode`; set it in the ESP-NOW branch before tasks start |
| `main/sensor_task.c` | gate the three ToF diagnostic arrays on `g_field_mode` (at snapshot-copy, not sensor-read) |

No changes to: the failover logic, `pipeline.c`, `espnow_transport.c`, the C6/S3
bridges, `main/proto/boat.pb.c` / `.h`, or `boat.proto` (hence no schema
regeneration and no perturbation of the protobuf layer the working WiFi path
depends on).

---

## 10. Open Questions

None blocking. Payload-trim scope, failover trigger (boot-time only), and
coexistence mode (idle standby) are all decided. One implementation detail is
pinned by test rather than by guesswork: the exact `grpcio-tools` version for
`.venv` is whatever `pip` resolves against protobuf 7.34.1 — the `import
boat_pb2` round-trip under `.venv` (§8) is the gate that confirms the resolved
versions are compatible, regardless of the specific numbers.

---

## 11. Known Behavior & Limitations (accepted, not fixed here)

The boot-time-only failover model is a deliberate "keep it simple" choice. Its
consequences are accepted for this project and recorded here so they are not
later mistaken for defects:

- **~30 s field-boot delay before ESP-NOW starts.** `wifi_init()` blocks up to
  `WIFI_CONNECT_TIMEOUT_MS = 30000` (`main/wifi_manager.c:15`) searching for an
  AP before the boot branch falls through to `espnow_transport_init()`. In the
  field (no AP), telemetry therefore begins ~30 s after power-on. The 30 s is
  conservative on purpose — shortening it risks a slow dock AP falsely tripping
  field mode and costing the dashboard/video when WiFi was actually available.
- **`CONFIG_ESPNOW_WIFI_TIMEOUT_S` is a dead knob.** Defined at
  `main/Kconfig.projbuild:418` (default 10 s) but consumed nowhere; the real
  timeout is the hardcoded 30 s above. Left as-is, noted so nobody tunes it
  expecting an effect.
- **Mid-mission WiFi loss = go dark, no ESP-NOW rescue.**
  `espnow_transport_init()` is called only from the boot branch
  (`main/main.c:226`); nothing re-inits it at runtime. If WiFi is up at boot and
  drops later, `wifi_manager` retries WiFi forever (fast, then every 10 s) and no
  telemetry flows until it returns. This is **pre-existing** behavior, unchanged
  by this project. Runtime WiFi→ESP-NOW failover is explicitly a separate, larger
  sub-project (see §2 non-goals).
