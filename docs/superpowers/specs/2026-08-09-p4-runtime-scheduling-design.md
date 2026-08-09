# ESP32-P4 Runtime Scheduling Design

**Date:** 2026-08-09
**Status:** Approved design, pending implementation plan

## Context

The boat firmware currently creates application tasks independently, generally without core affinity. IMU fusion, ToF acquisition, snapshots, camera work, GPS parsing, transport work, and detection therefore compete according to incidental FreeRTOS placement. IMU and ToF also contend for the same I2C mutex, and `g_inference_active` pauses sensor and fusion work during PicoDet inference. In practice, ToF can win the I2C mutex often enough to stall IMU/magnetometer updates.

The ESP32-P4 provides two high-performance FreeRTOS cores. The firmware must use them as two explicit lanes: one deterministic safety/control lane and one heavy/background lane. The design must preserve the working manual-control behavior while leaving clean interfaces and measured CPU capacity for later waypoint and ML autonomy.

The current tested PicoDet model takes approximately 57 ms per inference. The future ML control path has a separately agreed 260 ms camera-to-accepted-command deadline, but ML driving is not part of this phase.

## Goals

- Keep manual RC, radio freshness, failsafe, GPS, navigation sensing, and physical actuator ownership deterministic on HP Core 0.
- Move camera, detection, telemetry, snapshots, logging, and other heavy work to HP Core 1.
- Guarantee that camera or inference work cannot pause control, IMU, magnetometer, or failsafe processing.
- Replace IMU/ToF mutex competition with one deadline-aware I2C bus owner.
- Centralize task core affinity, priority, stack size, period, and deadline definitions.
- Add a manual-only command-arbitration boundary that can accept disabled waypoint and ML sources later.
- Keep winch, servo power, arm/disarm, and failsafe outside future autonomous driving authority.
- Measure task timing, jitter, deadline misses, stack margin, command age, sensor age, and per-core load.
- Reserve a shared heartbeat interface for a future LP-core safety supervisor.

## Non-goals

- No waypoint navigation, waypoint task, route planner, or waypoint UI.
- No ML driving, continuous ML control loop, or ML/waypoint mode selector.
- No new GPS-loss, compass-fault, or obstacle-avoidance policy beyond freshness and observability.
- No change to the current steering, throttle mixing, arming, winch, servo-power, or 400 ms radio-failsafe semantics except moving ownership into the scheduled control lane.
- No C6 or S3 firmware changes.
- No diagnostics dashboard/API expansion in this phase; compact periodic logs are sufficient.
- No LP-core authority over GPIO, PWM, servo power, or propulsion in this phase.

## Design principles

1. Use native FreeRTOS priority, affinity, notifications, and fixed-period scheduling; do not build a custom scheduler.
2. Core 0 owns deadlines and hardware safety. Core 1 owns throughput-oriented work.
3. Only one application task writes actuator hardware during normal runtime.
4. Only one application task performs runtime I2C transactions.
5. Transport callbacks validate and publish; they never block on heavy work or directly drive hardware.
6. Latest-value channels are used for continuous controls so old steering commands cannot accumulate.
7. Discrete safety commands use a separate bounded event path so an OFF or disarm request cannot be overwritten by axis traffic.
8. Cross-core readers consume timestamped immutable snapshots and never hold a control task lock.
9. Runtime instrumentation is bounded, allocation-free after startup, and aggregated outside real-time tasks.

## Hardware execution lanes

### HP Core 0: deterministic control and navigation

Core 0 contains the application tasks whose late execution can make the boat unsafe or uncontrollable. Higher numeric values indicate higher FreeRTOS application priority.

| Priority | Logical task | Activation | Responsibility |
|---:|---|---|---|
| 10 | `ControlTask` | Fixed 10 ms / 100 Hz, plus urgent notification | Select the accepted manual command, enforce freshness and limits, apply throttle/rudders/winch/servo power, and enter failsafe |
| 9 | `ArmSequencer` | Event-driven | Perform the existing ESC arm/disarm sequence without blocking `ControlTask`; submit sequence states to the actuator owner rather than writing unrelated outputs |
| 8 | `SensorBusTask` | Fixed 20 ms IMU deadline | Sole runtime I2C owner; sample IMU/magnetometer first and perform eligible ToF reads in remaining slots |
| 7 | `FusionTask` | Notified by each valid IMU sample | Update orientation/navigation state from cached sensor data without taking the I2C bus |
| 6 | `GpsTask` | UART/event driven | Parse UBX/NMEA and publish a timestamped GPS snapshot |

ESP-IDF and ESP-Hosted may create system tasks whose affinity is outside the application's direct control. Their receive callbacks must do bounded validation/copy work, publish into the command ingress structure, notify `ControlTask`, and return.

### HP Core 1: heavy and best-effort work

| Priority | Logical task | Activation | Responsibility |
|---:|---|---|---|
| 7 | `DetectTask` | On demand only | Run PicoDet; no direct access to actuator drivers |
| 6 | camera acquisition/drain | Frame/event driven | Maintain the camera pipeline and coordinate camera buffer ownership with detection |
| 5 | `SnapshotTask` | Fixed 50 ms / 20 Hz | Build telemetry snapshots from immutable sensor/navigation caches |
| 4 | ToF processing | Notified by a completed grid | Convert/cache grids and perform non-I2C postprocessing |
| 3 | transport TX | Queue/event driven | WebSocket, Wi-Fi, and ESP-NOW telemetry serialization/transmission |
| 2 | diagnostics, logging, status indication | Periodic/best effort | Aggregate and report metrics without printing from real-time tasks |

Exact stack sizes are centralized but are not guessed permanently. Existing sizes are the starting point, and measured stack high-water marks determine final values with at least 25% retained margin. Every task creation result is checked.

## Command ownership and arbitration

The mode arbiter controls only the future driving axes: throttle and rudder.

```text
manual drive slot ---------+
waypoint drive slot (off) --+--> mode arbiter --> ControlTask --> ESC + rudders
ML drive slot (off) --------+

manual auxiliary inputs -------------------------> ControlTask --> winch
safety/manual discrete events -------------------> ControlTask --> servo power
arm/disarm requests --> ArmSequencer/ControlTask ----------------> ESC state
link freshness ----------------------------------> ControlTask --> failsafe
```

Phase one compiles all three source identifiers into the interface but enables only `MANUAL`. No waypoint or ML producer task exists, and no transport message can select those disabled modes.

Continuous inputs are separated by command class rather than packed into one overwritable queue:

- A latest-value drive slot stores throttle, rudder, sequence number, receive timestamp, and source.
- A latest-value winch slot stores winch speed and its timestamp.
- Existing direct-steering/raw-steering commands use their own latest-value slot so drive packets cannot erase them before application.
- Servo-power and arm/disarm are discrete events in a small bounded queue. Servo-power OFF and disarm set urgent flags and notify `ControlTask`; they cannot be dropped behind ordinary motion updates.

Winch and servo-power control remain manual/safety functions even after autonomous drive sources are added. Future waypoint and ML modes may propose throttle and rudder only unless a later, separately reviewed safety design expands their authority.

`ControlTask` is the normal runtime owner of calls that change ESC throttle, rudder PWM, winch PWM, or the servo-power GPIO. The existing behaviors remain:

- A nonzero steering or winch command may automatically energize the shared servo rail.
- An explicit servo-power OFF stops the winch and de-energizes the rail.
- Link loss stops throttle and winch, centres steering, and cuts servo power.
- Normal ESC arming remains gated on a valid GPS lock unless the existing explicit override is used.

The current 100 ms timer-based watchdog logic moves into the 10 ms control cadence. A command age of at least 400 ms enters failsafe at the next control tick, yielding a defined 400-410 ms response window. Auxiliary or status traffic must not accidentally refresh the propulsion control link; existing accepted control-heartbeat semantics are preserved.

## Sensor and I2C scheduling

`SensorBusTask` is the only runtime owner of the shared I2C bus. Other tasks request data from caches or submit a bounded operation request; they do not take a global I2C mutex.

Every 20 ms cycle follows this order:

1. Wake using an absolute `vTaskDelayUntil` schedule.
2. Read the IMU and magnetometer.
3. Timestamp and publish the raw sample, then notify `FusionTask`.
4. Measure the time remaining before the next IMU deadline.
5. If a ToF read has enough bounded slack, read one grid; otherwise skip it and increment a deadline-protection counter.

ToF A and B alternate in eligible 100 ms slots. One sensor is read per slot, so each individual sensor targets one grid every 200 ms, or 5 Hz. Initial measurements indicate the combined two-grid operation takes about 25 ms; the implementation must measure each sensor separately and set its transaction timeout below the available slack. A slow or failed ToF transaction is abandoned/recovered rather than allowed to push the next IMU gap beyond 40 ms.

ToF postprocessing runs on Core 1 from the completed grid snapshot. Fusion consumes the cached IMU/magnetometer sample and performs no bus I/O. The `g_inference_active` gate is removed from IMU, magnetometer, fusion, ToF scheduling, and snapshot production. Camera and inference may still coordinate their shared camera buffers entirely within Core 1.

## GPS placement and state

GPS remains an independent UART sensor and does not use the I2C schedule. Its task is pinned to Core 0 below control, sensor I/O, and fusion. The UART task may block waiting for bytes because higher-priority Core 0 work preempts it immediately when runnable.

GPS publishes an immutable snapshot containing at least position, speed/course fields already available from the driver, satellite count, fix validity, source/protocol state, update timestamp, and calculated age. Snapshot/telemetry readers use this cache instead of entering the parser's state.

A missing or stale GPS fix is visible in metrics and continues to block normal arming as it does today. It does not stop already-authorized manual rudder, winch, or servo-power operation. New GPS-loss behavior during autonomous travel is explicitly deferred.

## Cross-core data publication

Sensor, navigation, GPS, actuator-status, and runtime-metric data are published as fixed-size versioned snapshots. A writer fills the inactive buffer, issues the required memory ordering, and atomically publishes its index/version. Readers copy a stable version and retry if the version changed during the copy.

This avoids long cross-core mutex holds. Large ToF grids may use a fixed pool plus generation index rather than copying through queues. All snapshots carry an `esp_timer_get_time()` monotonic timestamp, validity flags, and a sequence/generation number.

No runtime path allocates memory to publish a control or sensor update. Queues, buffers, metrics, and task stacks are created during initialization.

## Failure handling

| Failure | Required behavior |
|---|---|
| Manual control age reaches 400 ms | At the next 10 ms control tick: zero throttle/winch, centre rudders, cut servo power, record link-loss reason |
| Servo-power OFF or disarm arrives during traffic | Urgent notification preempts ordinary axis backlog; event is never overwritten by a newer axis value |
| Continuous command overload | Replace the older value with the newest command and increment the overwrite counter |
| Invalid/out-of-range command | Reject, count, and retain the last valid command; do not refresh control freshness |
| IMU/magnetometer deadline miss | Record execution/gap metrics and mark navigation age; manual control continues |
| ToF lacks time or times out | Skip/recover that ToF cycle and preserve the next IMU deadline |
| GPS invalid/stale | Publish invalid age, retain arming gate, and keep authorized manual control available |
| Camera/detection/Core 1 overload | Drop or delay best-effort work; Core 0 timing remains unaffected |
| Critical Core 0 task cannot be created | Keep actuators de-energized and fail startup visibly |
| Optional Core 1 task cannot be created | Disable the affected feature, report the fault, and retain safe manual operation |

## Runtime metrics

A fixed-size metrics record is maintained for every application task. Real-time tasks update counters and bounded timing accumulators only. A Core 1 diagnostic task emits a compact aggregate at a low rate.

Metrics include:

- configured and observed core, priority, and stack high-water mark;
- last, maximum, and histogram-based execution time;
- wake-up jitter, maximum inter-run gap, and deadline misses;
- manual command age, command-to-output latency, invalid commands, and slot overwrites;
- failsafe count and reason;
- IMU/magnetometer sample gap and I2C timeout count;
- per-ToF read duration, grid age, skip count, and failure count;
- GPS fix age, UART overflow/error count, parse errors, and valid-fix rate;
- camera, snapshot, transport, and detection duration/drop counts;
- per-core idle/utilization measurements where supported by the configured ESP-IDF runtime statistics.

Metrics use fixed histogram buckets or bounded accumulators rather than dynamically storing samples. Error logs are rate-limited; repetitive failures increment counters.

## LP-core boundary

Phase one reserves a versioned shared-memory heartbeat structure containing the Core 0 heartbeat sequence, last update time, control state, failsafe state, and supervisor observation counters. Core 0 updates it from the deterministic control cadence.

The LP core may observe and report missed heartbeats, but it receives no authority to change GPIO, PWM, servo power, or propulsion in this phase. Hardware intervention requires a later bench-tested design that defines pin ownership, boot/reset behavior, false-positive handling, and recovery.

The LP core is not treated as a third FreeRTOS core and receives no normal application workload.

## Verification and acceptance

### Automated verification

Host-side tests cover:

- latest-value replacement for continuous drive, winch, and steering slots;
- discrete OFF/disarm events surviving continuous command overload;
- disabled waypoint and ML sources never becoming active;
- command validation and freshness rules;
- failsafe boundaries immediately before, at, and after 400 ms;
- servo-power OFF forcing commanded winch state to zero;
- immutable snapshot generation/reader consistency;
- fixed-period and ToF alternating-slot scheduling calculations;
- metrics counter and histogram behavior.

The P4 firmware must build successfully with all task-creation results handled. Existing host tests must remain green.

### Hardware stress verification

Record a baseline before affinity or ownership changes. Then run a ten-minute bench test with:

- continuous ESP-NOW manual throttle/rudder updates;
- repeated winch movement and stop commands;
- repeated servo-power ON/OFF operations;
- both ToF sensors active;
- GPS UART traffic;
- camera streaming;
- 20 Hz telemetry snapshots;
- repeated approximately 57 ms PicoDet inference.

Acceptance criteria:

- P4 receive-to-physical-command latency never exceeds 20 ms under the test load.
- Servo-power OFF and winch stop are applied within 20 ms.
- Link-loss failsafe applies within 400-410 ms of the last accepted control update.
- IMU/magnetometer target 50 Hz and no observed sample gap exceeds 40 ms.
- Each ToF sensor averages approximately 5 Hz without causing an IMU deadline violation; all skips are counted.
- GPS UART does not overflow and fix age remains bounded by the configured receiver update period while valid data is present.
- Camera or detection activity causes no Core 0 control or fusion pause.
- Every task retains at least 25% measured stack margin after the stress run.
- Task creation, deadline misses, queue/slot overwrites, and feature failures are never silent.
- Core 1 utilization and detection cadence are recorded so headroom for the future 260 ms ML path is an evidence-based decision.
- Existing manual dashboard, winch, servo-power, steering, arming, and telemetry behavior remains functional.

## Migration sequence

1. Add centralized scheduling definitions, checked task creation, and bounded runtime metrics; record the current baseline.
2. Add the manual-only control ingress/arbiter boundary and consolidate actuator ownership while preserving existing outputs and failsafe behavior.
3. Pin Core 0 control, arm sequencing, GPS, sensor bus, and fusion tasks with the approved priorities.
4. Replace IMU/ToF mutex competition with `SensorBusTask` and move ToF postprocessing to Core 1.
5. Pin and classify Core 1 camera, detection, snapshot, transport, diagnostics, and status tasks.
6. Remove inference gating from Core 0 sensing/fusion and confine camera-buffer coordination to Core 1.
7. Add the passive LP heartbeat ABI/observer without hardware actuation.
8. Run automated verification, baseline comparison, and the full hardware stress test before considering future autonomy work.

Each migration stage must remain buildable and independently testable. If a timing regression appears, the metrics from the immediately preceding stage identify whether affinity, bus ownership, command ingress, or Core 1 load caused it.

## Future extension contract

Waypoint and ML work may later publish timestamped throttle/rudder proposals into their disabled source slots. The Core 0 mode manager will validate source enablement, proposal age, confidence/policy gates, and manual/failsafe preemption before accepting a proposal. Neither future source gains direct driver access.

The target future ML budget remains:

- capture/preprocess: up to 50 ms;
- inference hard budget: up to 100 ms, with the current model observed near 57 ms;
- decision/policy: up to 20 ms;
- cross-core proposal handoff: up to 10 ms;
- contingency/headroom: 80 ms;
- total camera-to-accepted-command deadline: 260 ms.

This phase creates and measures the boundary but does not implement any of those autonomous producers or policies.
