# Protobuf Data Pipeline — Design Spec

**Date:** 2026-03-26
**Status:** Draft
**Scope:** Replace JSON REST polling with protobuf-over-WebSocket; transport-agnostic pipeline for future protocol migration

---

## Problem

The current `sensor_api.c` serves JSON over REST endpoints (`/api/snapshot`, `/api/imu`). The dashboard polls at 5Hz via `fetch()`. This architecture has three problems:

1. **ctrl_port collision** — two `httpd` servers both use `HTTPD_DEFAULT_CONFIG()` which binds ctrl socket to UDP port 32768. Second server fails with `EADDRINUSE` (errno 112).
2. **Polling waste** — dashboard polls at 5Hz regardless of data freshness. REST is request-response, wrong model for real-time telemetry.
3. **Transport lock-in** — JSON + HTTP is tightly coupled to WiFi/browser. Future protocols (ESP-NOW, BLE, UART) would require rewriting serialization and routing logic.

## Solution

A transport-agnostic data pipeline using Protocol Buffers (nanopb) with pluggable transport adapters. Phase 1 implements WebSocket transport for the browser dashboard. The pipeline design allows adding ESP-NOW or other transports by implementing a single function pointer.

Camera (MJPEG) stays on its own dedicated transport path — it is explicitly excluded from the protobuf pipeline due to frame size constraints on constrained transports (ESP-NOW 250-byte limit).

---

## Architecture

```
┌──────────────── ESP32-P4 ─────────────────────────────────────────┐
│                                                                    │
│  task_imu_fusion (100Hz)  ─┐                                      │
│                             ├─→ task_sensor_snapshot (5Hz)         │
│  tof_read_grid (on demand) ─┘         │                           │
│                                       ▼                           │
│                              pipeline_publish()                   │
│                              nanopb encode BoatMessage             │
│                                       │                           │
│                              ┌────────┴────────┐                  │
│                              ▼                  ▼                  │
│                     ws_transport_send()   [future transports]     │
│                     binary WS frame       espnow / ble / uart    │
│                              │                                    │
│                              ▼                                    │
│                     httpd port 80                                 │
│                     ├── GET /        → dashboard.html             │
│                     └── WS  /ws      → protobuf binary           │
│                                                                    │
│  Camera (separate path, not protobuf):                            │
│                     httpd port 81                                 │
│                     └── GET /stream  → MJPEG                     │
└────────────────────────────────────────────────────────────────────┘

Browser Dashboard:
  - protobuf.js decodes binary WS messages → updates UI
  - Commands encoded via protobuf.js → sent as binary WS messages
  - Camera: <img src="http://${host}:81/stream"> (unchanged)
```

---

## Proto Messages

File: `main/proto/boat.proto`

```protobuf
syntax = "proto3";
package boat;

message IMUData {
  float pitch   = 1;
  float roll    = 2;
  float heading = 3;
}

message ToFGrid {
  bool          valid     = 1;
  repeated int32 distances = 2;  // 64 values, row-major 8x8
}

message SensorSnapshot {
  uint64  timestamp_us = 1;
  IMUData imu          = 2;
  ToFGrid tof_a        = 3;
  ToFGrid tof_b        = 4;
}

message MotorCommand {
  float throttle = 1;  // -1.0 to 1.0
  float rudder   = 2;  // -1.0 to 1.0
}

message SystemStatus {
  uint32 heap_free    = 1;
  int32  wifi_rssi    = 2;
  uint64 uptime_us    = 3;
}

// Envelope — every message on the wire is a BoatMessage
message BoatMessage {
  oneof payload {
    SensorSnapshot sensors  = 1;
    MotorCommand   motor    = 2;
    SystemStatus   status   = 3;
  }
}
```

**Extensibility:** Adding a new message type = add a field to the `oneof`. No existing code breaks. Field numbers are stable — never reuse a retired number.

**Flat structure:** Proto is intentionally 2 levels max (BoatMessage → SensorSnapshot → IMUData/ToFGrid). nanopb re-encodes sub-messages twice (size calculation pass + write pass) — deeper nesting causes exponential encoding time on the RISC-V cores. Keep it flat.

**nanopb options:** Static allocation via `.options` file. Max `distances` count = 64. Estimated struct sizes:
- `boat_SensorSnapshot`: ~540 bytes (timestamp 8B + IMU 12B + 2× ToFGrid 257B each)
- `boat_BoatMessage`: ~550 bytes (envelope + largest oneof variant)
- `boat_MotorCommand`: ~8 bytes

---

## Transport Interface

File: `main/transport.h`

```c
#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#define PIPELINE_MAX_TRANSPORTS 4

typedef esp_err_t (*transport_send_fn)(const uint8_t *buf, size_t len, void *ctx);

typedef struct {
    transport_send_fn send;
    void *ctx;
} transport_t;
```

File: `main/pipeline.h`

```c
#pragma once
#include "esp_err.h"
#include "transport.h"
#include "proto/boat.pb.h"

// Initialize the pipeline (creates mutex, zeroes state)
esp_err_t pipeline_init(void);

// Register a transport adapter for outgoing data
esp_err_t pipeline_register_transport(transport_send_fn send, void *ctx);

// Called by sensor task: encodes snapshot to protobuf, fans out to all transports
void pipeline_publish_sensors(const boat_SensorSnapshot *snap);

// Called by transport when it receives raw bytes: decodes and dispatches
void pipeline_handle_incoming(const uint8_t *buf, size_t len);

// Register a handler for motor commands (called from pipeline_handle_incoming)
typedef void (*motor_command_handler_fn)(const boat_MotorCommand *cmd);
void pipeline_register_motor_handler(motor_command_handler_fn handler);
```

---

## Pipeline Implementation

File: `main/pipeline.c`

Responsibilities:
1. **Publish path:** `pipeline_publish_sensors()` takes a C struct, encodes via `pb_encode()` into a stack buffer (~512 bytes), iterates registered transports, calls each `send()`.
2. **Receive path:** `pipeline_handle_incoming()` takes raw bytes, decodes via `pb_decode()`, inspects `which_payload`, dispatches to registered handler (e.g., motor command handler).
3. **Thread safety:** Encoding uses a local stack buffer — no mutex needed. Transport `send()` functions are responsible for their own thread safety (see WS transport below).

---

## WebSocket Transport

File: `main/transports/ws_transport.c`

Responsibilities:
1. Track connected WebSocket client file descriptors (max 4) in a mutex-protected list. The mutex guards access because the fd list is written from the httpd task (connect/disconnect events) and read from the sensor task (publish fan-out).
2. `ws_transport_send()`: for each connected client, call `httpd_ws_send_frame_async()` with binary opcode. **On send failure, remove the stale fd immediately** — ESP-IDF does not reliably deliver `HTTPD_WS_TYPE_CLOSE` for all disconnection scenarios (client crash, network drop). Failed sends are the primary cleanup mechanism.
3. `ws_on_receive()`: when a binary WS message arrives, call `pipeline_handle_incoming()`.
4. Handle connect/disconnect lifecycle.

The WebSocket handler registered with httpd:
- On `HTTPD_WS_TYPE_CONNECTED`: lock mutex, add fd to client list, unlock
- On `HTTPD_WS_TYPE_CLOSE`: lock mutex, remove fd from client list, unlock
- On `HTTPD_WS_TYPE_BINARY`: pass payload to `pipeline_handle_incoming()`

`ws_transport_send()` (called from sensor task context):
- Lock mutex, snapshot the fd list, unlock
- Iterate snapshot: call `httpd_ws_send_frame_async()` for each fd
- On failure: lock mutex, remove failed fd, unlock

---

## HTTP Server

File: `main/http_server.c`

Single httpd on port 80 with `ctrl_port = 32768`:
- `GET /` → serves embedded `dashboard.html`
- `WS /ws` → WebSocket endpoint (protobuf binary)

Config:
```c
httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
cfg.server_port      = 80;
cfg.ctrl_port        = 32768;
cfg.stack_size       = 8192;
cfg.max_open_sockets = 7;
cfg.max_uri_handlers = 4;
```

---

## Camera Stream Server

File: `main/camera_stream.c` (one-line change: add `ctrl_port = 32769`)

Dedicated httpd on port 81 with `ctrl_port = 32769`:
- `GET /stream` → blocking MJPEG handler

```c
httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
cfg.server_port      = CONFIG_HTTP_STREAM_PORT;  // 81
cfg.ctrl_port        = 32769;  // different from port-80 server
cfg.stack_size       = 8192;
cfg.max_open_sockets = 4;
cfg.max_uri_handlers = 2;
```

Camera is explicitly a separate transport path. Not part of the protobuf pipeline.

---

## Sensor Snapshot Task

File: `main/sensor_task.c` (extracted from `sensor_api.c`, which is deleted)

```c
void task_sensor_snapshot(void *pvParameters) {
    tof_devices_t *devs = (tof_devices_t *)pvParameters;

    // IMPORTANT: static allocation — boat_SensorSnapshot with two 64-element
    // ToF grids is ~540 bytes. Declaring on stack risks overflow in smaller tasks.
    // This task runs at 5Hz so a single static instance is fine (no reentrancy).
    static boat_SensorSnapshot snap;

    while (true) {
        snap = (boat_SensorSnapshot)boat_SensorSnapshot_init_zero;
        snap.timestamp_us = esp_timer_get_time();

        FusionResult imu;
        fusion_get_result(&imu);
        snap.imu.pitch   = imu.pitch;
        snap.imu.roll    = imu.roll;
        snap.imu.heading = imu.heading;

        VL53L5CX_ResultsData tof_a, tof_b;
        snap.has_tof_a = (tof_read_grid(&devs->dev_a, &tof_a) == ESP_OK);
        if (snap.has_tof_a) {
            snap.tof_a.valid = true;
            snap.tof_a.distances_count = 64;
            for (int i = 0; i < 64; i++)
                snap.tof_a.distances[i] = tof_a.distance_mm[i * VL53L5CX_NB_TARGET_PER_ZONE];
        }
        // same for tof_b...

        pipeline_publish_sensors(&snap);

        vTaskDelay(pdMS_TO_TICKS(200));  // 5Hz
    }
}
```

---

## Dashboard Changes

File: `main/dashboard.html`

### JavaScript data layer replacement:

```javascript
// Load protobuf schema (embedded as string or fetched)
const root = protobuf.parse(PROTO_SCHEMA).root;
const BoatMessage = root.lookupType('boat.BoatMessage');

const ws = new WebSocket(`ws://${window.location.host}/ws`);
ws.binaryType = 'arraybuffer';

ws.onmessage = (evt) => {
  const msg = BoatMessage.decode(new Uint8Array(evt.data));
  if (msg.sensors) {
    const s = msg.sensors;
    updateIMU(s.imu.pitch, s.imu.roll, s.imu.heading);
    updateToF('tof-a', s.tofA);
    updateToF('tof-b', s.tofB);
    updateStatus('connected');
  }
};

ws.onclose = () => updateStatus('disconnected');

// Send motor command
function sendMotor(throttle, rudder) {
  const msg = BoatMessage.create({ motor: { throttle, rudder } });
  ws.send(BoatMessage.encode(msg).finish());
}
```

### What changes:
- Remove `fetch()` polling loop and `setInterval`
- Remove `apiUrl()` / manual IP input (WS connects to same origin)
- Add protobuf.js (CDN or embedded)
- Camera `<img>` tag unchanged — still points to `:81/stream`
- IMU/ToF rendering functions unchanged — just wired to WS instead of fetch

### What stays:
- All CSS/visual design
- Horizon canvas, ToF heatmap canvases
- FPS counter (tracks WS message rate instead of img.onload)

---

## File Structure (Phase 1)

```
main/
├── proto/
│   ├── boat.proto              # Message definitions (source of truth)
│   ├── boat.options            # nanopb options (static allocation limits)
│   └── boat.pb.c / boat.pb.h  # Auto-generated by nanopb_generate_cpp() at build time
├── pipeline.c                  # Encode, decode, fan-out
├── pipeline.h
├── transport.h                 # Abstract transport interface
├── transports/
│   └── ws_transport.c/h       # WebSocket adapter
├── http_server.c/h            # Port 80: dashboard + WS endpoint
├── camera_stream.c/h          # Port 81: MJPEG (only change: ctrl_port = 32769)
├── sensor_task.c/h            # Snapshot task (extracted from sensor_api.c)
├── drivers/
│   ├── imu_driver.c/h         # Unchanged
│   ├── tof_driver.c/h         # Unchanged
│   └── camera_driver.c/h      # Unchanged
├── sensor_fusion.c/h          # Unchanged
├── calibration.c/h            # Unchanged
├── file_system.c/h            # Unchanged
├── wifi_manager.c/h           # Unchanged
├── dashboard.html             # Rewritten data layer (protobuf.js + WS)
├── CMakeLists.txt             # Add nanopb, proto generation, ws_transport
├── Kconfig.projbuild          # Add WS config options
└── main.c                     # Init pipeline, register WS transport
```

### Deleted:
- `main/sensor_api.c` — replaced by `pipeline.c` + `ws_transport.c` + `http_server.c` + `sensor_task.c`
- `main/sensor_api.h` — replaced by `pipeline.h` + `http_server.h` + `sensor_task.h`

---

## Dependencies

### nanopb (new)
- **Install via managed component** (not manual copy):
  ```bash
  idf.py add-dependency "nanopb/nanopb^0.4.9"
  ```
  This downloads into `managed_components/` and ESP-IDF prioritizes it over any built-in versions.

- **CMake auto-generation** — no manual `protoc` runs. In `main/CMakeLists.txt`:
  ```cmake
  set(PROTO_FILES "proto/boat.proto")
  nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

  idf_component_register(
      SRCS "main.c" "pipeline.c" "http_server.c" "sensor_task.c"
           "transports/ws_transport.c" ${PROTO_SRCS}
      INCLUDE_DIRS "." "proto"
      REQUIRES nanopb esp_http_server esp_timer driver
  )
  ```

- Zero dynamic allocation — all buffers statically sized via `.options`
- **Version trap:** ESP-IDF bundles `protobuf-c` internally (for `protocomm`). We use nanopb (completely separate library), so no version conflict. Do NOT mix nanopb and protobuf-c in the same project.
- **protoc version matching:** Run `protoc --version` in devcontainer and on host/browser-side. Must match major versions (both 3.x.x or both 25.x). Mismatched versions cause silent decode failures.

`main/proto/boat.options`:
```
boat.ToFGrid.distances   max_count:64
boat.BoatMessage          no_unions:false
```

**Unknown fields protection:** nanopb ignores unknown fields by default (safe). But add strict size limits to all `bytes`/`string` fields in `.options` to prevent OOM from malformed packets. Our current proto has no string/bytes fields, so this is future-proofing.

### protobuf.js (dashboard)
- Lightweight browser protobuf decoder
- Embed minified `protobuf.min.js` via `EMBED_TXTFILES` or load from CDN
- ~40KB minified — fits comfortably in flash

### esp_http_server WebSocket support
- Already available in ESP-IDF v5.4+
- Enable `CONFIG_HTTPD_WS_SUPPORT=y` in sdkconfig

---

## Boot Sequence (updated main.c)

```
app_main()
├── fs_init()
├── SCCB bus + camera_init()
├── Sensor I2C bus
├── imu_init()
├── tof_init()
├── Calibration check
├── fusion_init()
├── pipeline_init()                    ← NEW
├── wifi_init()
│   ├── camera_stream_server_start()   (port 81, ctrl_port=32769)
│   ├── http_server_start()            ← NEW (port 80, ctrl_port=32768)
│   └── ws_transport_init(server)      ← NEW (registers with pipeline)
├── xTaskCreate(task_imu_fusion)
└── xTaskCreate(task_sensor_snapshot)   (calls pipeline_publish_sensors)
```

---

## Phase 2: ESP-NOW Transport (Future — Plan Only)

### Scope
Add `espnow_transport.c` implementing `transport_send_fn`. Register alongside WebSocket transport. Both run simultaneously — sensor data fans out to all connected transports.

### Design considerations
- ESP-NOW max payload: 250 bytes. `SensorSnapshot` with full 64-element ToF grids exceeds this.
- Options: (a) send IMU-only snapshots at high rate, ToF at lower rate in separate messages; (b) downsample ToF to 4x4 (16 values); (c) fragment and reassemble.
- ESP-NOW and WiFi coexist on the same radio — must be on the same channel.
- Peer management: broadcast or registered peer MAC addresses.
- Will require new proto messages or snapshot variants for constrained payloads.

### Implementation steps (future)
1. Define constrained message variants in `boat.proto` if needed
2. Write `main/transports/espnow_transport.c` implementing `transport_send_fn`
3. Add ESP-NOW init to `main.c` (peer config, channel)
4. Call `pipeline_register_transport(&espnow_send, ctx)`
5. Test alongside WebSocket — both receive same data

### Not in Phase 1
- No ESP-NOW code written
- No peer management
- No fragmentation logic
- Pipeline is ready — transport interface is the only contract

---

## ESP32 Pitfalls & Mitigations

Lessons from ESP-IDF community and benchmarks — baked into the design:

| Pitfall | Risk | Mitigation in this design |
|---------|------|---------------------------|
| **Stack overflow** | nanopb structs are large (~540B for SensorSnapshot). Local vars on a 4KB task stack → crash. | Snapshot struct declared `static` in sensor task. Encode buffer is stack-local but capped at 512B (sufficient for wire format, smaller than struct). |
| **Heap fragmentation** | `protobuf-c` uses `malloc` for every decode. Long-running boat telemetry → fragmentation → OOM. | Using nanopb with static allocation. Zero `malloc` in encode/decode path. |
| **Nested message perf** | nanopb double-encodes sub-messages (size pass + write pass). Deep nesting = exponential cost. | Proto kept flat: max 2 levels (BoatMessage → Snapshot → IMU/ToF). No deeper nesting allowed. |
| **Version trap** | ESP-IDF bundles `protobuf-c` for `protocomm`. Mixing versions causes `#error`. | Using nanopb (separate library). No conflict with IDF's internal protobuf-c. |
| **PSRAM for large buffers** | Internal RAM is limited. Large encode buffers may fail to allocate. | Encode buffer is ~512B stack (fine for internal RAM). Camera frames (large) are excluded from protobuf entirely — they stay on MJPEG path using existing PSRAM-allocated DMA buffers. |
| **Camera in protobuf** | Wrapping JPEG frames in protobuf adds CPU overhead for no gain. | Camera explicitly excluded from pipeline. MJPEG streams raw JPEG via dedicated httpd. |

---

## Testing Strategy

### Unit (host-side)
- Encode/decode round-trip: C struct → nanopb encode → nanopb decode → verify fields match
- Pipeline fan-out: register mock transports, publish, verify all received identical bytes

### Integration (on device)
- Connect browser dashboard, verify sensor data renders
- Open MJPEG stream simultaneously, verify no interference
- Monitor heap usage over 10 minutes — no leaks
- Disconnect/reconnect WebSocket — verify clean recovery
- Multiple browser tabs — verify all receive data

### Acceptance
- Dashboard loads at `http://<ip>/`
- IMU numbers update in real-time (no polling artifacts)
- ToF heatmaps update in real-time
- Camera stream runs smoothly alongside sensor data
- No `ctrl_port` errors in boot log
- `idf.py size` shows acceptable flash/RAM usage with nanopb + protobuf.js

---

## Phase 3: LiveKit Video Stream (Future — Plan Only)

### Motivation

LiveKit is a WebRTC media server that provides sub-second latency video with built-in adaptive bitrate, multi-participant support, and a hosted SDK. The ESP32-P4 camera currently streams MJPEG over a dedicated HTTP server — functional but with no signaling, no NAT traversal, and no adaptive quality. LiveKit would replace or augment the MJPEG path with a proper real-time video pipeline.

### How it fits

The protobuf pipeline (Phase 1) handles sensor telemetry over WebSocket. LiveKit handles video — they are independent transports. The ESP32-P4 would publish video via the LiveKit C SDK (or a custom WebRTC implementation using `esp_websocket_client` for signaling). The browser dashboard would subscribe via LiveKit's JavaScript SDK, rendering the video track in a `<video>` element alongside the existing sensor panels.

### Design considerations

- **ESP-IDF WebRTC**: Espressif maintains `esp-webrtc-solution` (`github.com/espressif/esp-webrtc-solution`) which provides a native WebRTC stack for ESP32. This is the most direct path — no LiveKit C SDK required. It uses ICE/DTLS/SRTP internally.
- **LiveKit server**: Can be self-hosted (Docker) or use LiveKit Cloud. Self-hosted on a local network eliminates NAT traversal concerns for the boat use case.
- **PSRAM requirement**: WebRTC frame buffering needs PSRAM. ESP32-P4 has 32MB PSRAM — sufficient.
- **Encoding**: H.264 or MJPEG-over-WebRTC. The P4's HW JPEG encoder (already in use) produces frames at ~15fps; the `esp-webrtc-solution` can wrap JPEG frames into RTP.
- **Coexistence with MJPEG**: Keep MJPEG as fallback. LiveKit can be enabled/disabled via Kconfig.
- **Telemetry data channel**: LiveKit supports WebRTC data channels — telemetry could migrate from WebSocket to LiveKit data channel, unifying all I/O through one SDK. This is Phase 3+ territory.

### Implementation steps (future)

1. Evaluate `esp-webrtc-solution` integration with existing `camera_driver.c` pipeline
2. Stand up LiveKit server (Docker, local network)
3. Add LiveKit/WebRTC component to `idf_component.yml`
4. Write `main/transports/webrtc_transport.c` for video publish
5. Update `dashboard.html` to include LiveKit JS SDK and subscribe to video track
6. Optional: migrate WS telemetry to LiveKit data channel

### Not in Phase 1 or 2

- No WebRTC code in current implementation
- MJPEG stream on port 81 stays as-is
- Transport interface is ready when the time comes
