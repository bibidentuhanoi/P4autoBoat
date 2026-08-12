# Feature 2: Continuous Synchronized Dataset Recording

Date: 2026-08-12

## Purpose

Feature 2 records a bounded, continuous perception session while a human drives the boat. It preserves camera, dual-ToF, IMU/fusion, GPS, and applied manual-control history with a common P4 timebase so a notebook can later reconstruct causal camera–sensor samples for obstacle detection, ToF–camera fusion, and future steering/throttle learning.

This feature extends the hardware-validated single-trigger dataset capture without changing the deterministic control architecture. It records raw observations and human control labels only. PicoDet inference, obstacle policy, waypoint following, and autonomous actuation are later consumers and are not part of this feature.

## Fixed Recording Contract

- A command starts one fixed 30-second recording session.
- Camera acquisition targets 20 FPS, or 600 attempted frames per session.
- Dataset fusion export targets 10 Hz, or approximately 300 synchronized samples per session.
- Live MJPEG is disabled during recording and restored after finalization.
- WiFi and ESP-NOW use the same protobuf command.
- A trigger received during startup, recording, or finalization is acknowledged but ignored.
- There is no manual stop command in this version.
- Recording is optional best-effort work. It drops or degrades before control, radio traffic, sensor acquisition, or safety behavior.

## Execution Priorities and Isolation

The existing execution lanes remain authoritative:

- Core 0 owns the 100 Hz Control task, actuator application, IMU acquisition, fusion, GPS, and ToF reads.
- Core 1 owns camera work, telemetry, ToF processing, and the low-priority TrainingLog recorder/writer.
- The recorder never takes an actuator lock, writes actuator hardware, or performs work in a transport receive callback.
- Incoming throttle, rudder, winch, servo-power, disarm, and failsafe traffic is never rate-limited.
- Only heavy outbound dashboard telemetry may be reduced during recording.

The SD card and C6 transport share the SDMMC controller, so task affinity alone is insufficient. The writer is radio-first:

1. Camera records enter a preallocated PSRAM queue without waiting for storage.
2. The writer issues measured, bounded SD transactions and yields between them.
3. If command age or transport health worsens, storage output pauses immediately after the in-flight transaction.
4. Queue pressure first reduces heavy full-ToF dashboard telemetry, then camera acquisition from 20 to 15 to 10 FPS.
5. If the queue fills, camera records are dropped and counted. Control commands are never dropped for recording.
6. If no bounded SD transaction size satisfies control latency on hardware, active capture uses a larger PSRAM spool and defers SD flushing until safe.

Physical ToF acquisition and full-fidelity BoatLog records are not reduced with dashboard telemetry.

## Architecture

### Recorder state machine

The recorder has four externally visible states:

1. `IDLE`: live MJPEG may run; a recording command is accepted.
2. `STARTING`: validate SD and buffers, allocate the next session, stop live MJPEG, and open the prepared BoatLog file.
3. `RECORDING`: acquire timestamped JPEGs for 30 seconds, accept timestamped sensor/control events, adapt camera FPS under pressure, and ignore duplicate triggers.
4. `FINALIZING`: stop accepting camera records, drain accepted records, append the session summary, close the file, restore MJPEG, and return to `IDLE`.

An error transitions through finalization when possible. Failure to record never blocks boot or manual operation.

No additional timer task is created. At session start the recorder computes one absolute `stop_us = start_us + 30 seconds`; each completed camera frame compares its existing acquisition timestamp with that deadline. A 600-attempt ceiling is a second bound, but dropped frames never extend the wall-clock duration.

### Producer/writer boundary

Camera and sensor producers do not perform filesystem work. They publish fixed metadata plus bounded payload references into preallocated channels. One low-priority Core 1 writer is the sole BoatLog appender.

Camera ownership is held only long enough to dequeue a completed JPEG, assign its frame ID and timestamp, copy it into an available PSRAM slot, and requeue the V4L2 buffer. A full camera pool drops that frame rather than blocking acquisition.

Sensor/control records are much smaller than JPEG records and receive reserved capacity so a camera burst cannot consume every queue slot. Continuous sources may use latest-value or bounded event channels according to their semantics, but the writer must preserve their original sequence numbers and timestamps.

## Timestamp Contract

All records use `esp_timer_get_time()` on the P4 as a common monotonic microsecond clock. Storage time and laptop arrival time are never used for matching.

- Camera records store frame ID and the timestamp at which V4L2 reports the completed JPEG.
- ToF records store sensor identity, generation, stream count, data-ready observation time, read-start time, read-complete time, and validity. Torn/rejected grids never become valid samples.
- IMU records store generation, transaction bounds, sample midpoint, and raw values.
- Fusion records store generation, source sample generation/time, output time, and fused values.
- GPS records store generation, complete UART-frame receive time, fix validity, protocol, and navigation values.
- Manual-control records distinguish transport acceptance time from the 100 Hz Control-task application time and preserve source, axes, winch, servo-power, arm/disarm, and failsafe state.

Raw timing bounds remain in the log so later calibration can account for fixed camera or ToF latency without recollecting data.

## Causal Synchronization

Matching happens in the notebook, not in the storage writer. For each 10 Hz exported perception sample, the notebook reconstructs only data the real firmware could have known at that decision instant:

1. Choose the latest complete, non-torn ToF-A and ToF-B available by the decision time.
2. Choose the closest completed camera frame that was also available by that time.
3. Choose causal IMU/fusion, GPS, accepted-control, and applied-control records.
4. Record signed time deltas from the camera and decision timestamps.
5. Mark any source outside its configured freshness/tolerance window invalid instead of silently substituting stale or future data.

The initial accepted camera-to-ToF matching tolerance is 75 ms. ToF-A and ToF-B retain independent timestamps and deltas because they are not perfectly simultaneous. Two camera frames may legitimately reference the same 10 Hz ToF generation, although the default fused export produces one selected camera frame per approximately 10 Hz perception cycle.

Dropped camera frames create sequence gaps but do not shift later matches. A smaller coherent dataset is preferred to a larger incorrectly paired dataset.

## Full ToF Fidelity

Both ToFs retain the complete four-target 8x8 output. The recorder never reduces the grid to the closest target.

For each sensor, every valid grid stores:

- `distance_mm[256]`
- `sigma_mm[256]`
- `target_status[256]`
- `targets_detected[64]`
- generation, stream count, and acquisition/read timing

The fixed layout is:

```text
index = zone * 4 + target
```

Indexes 0–3 are zone 0, indexes 4–7 are zone 1, and indexes 252–255 are zone 63. Status and sigma remain paired with every distance so downstream processing can reject unreliable measurements deliberately.

## BoatLog Storage Format

Each session is one append-only binary file. Names remain FAT 8.3-compatible because this board's current FAT configuration has already rejected long session names on hardware:

```text
/sdcard/training/00021.tmp
```

The file header contains:

- magic and BoatLog format version;
- firmware/build identity;
- session ID and trigger source;
- camera resolution and requested FPS;
- ToF configuration and array layout;
- calibration identity/version;
- session start timestamp.

Every record is independently recoverable:

```text
magic | record_type | flags | sequence |
timestamp_us | payload_length | payload | CRC32
```

Record types include:

- `CAMERA_JPEG`
- `TOF_A_GRID`
- `TOF_B_GRID`
- `IMU_SAMPLE`
- `FUSION_RESULT`
- `GPS_FIX`
- `CONTROL_ACCEPTED`
- `CONTROL_APPLIED`
- `FRAME_DROPPED`
- `RECORDER_EVENT`
- `SESSION_END`

Fixed binary payload layouts and direct copies are used instead of high-rate protobuf serialization. On normal completion, final counters and the termination reason are appended, the file is flushed/closed, and `00021.tmp` is renamed to `00021.blg`. The notebook treats `.blg` as a BoatLog file; the human-readable `session_0021` name is used only for the exported laptop directory.

After reset or power loss, the notebook scans through the last record with a valid header, bounded length, and CRC. It discards only the incomplete/corrupt tail. A missing `SESSION_END` marks the session interrupted but does not invalidate its recovered prefix.

The next session file should be prepared/preallocated while idle rather than during active control to reduce FAT allocation stalls at trigger time.

## Protobuf Command and Runtime Status

The current empty Feature 1 command remains wire-compatible by adding a defaulted mode:

```proto
enum TrainingLogMode {
  SINGLE_CAPTURE = 0;
  RECORD_30_SECONDS = 1;
}

message TrainingLogCommand {
  TrainingLogMode mode = 1;
}
```

Existing empty messages continue to request a single capture. Both the WiFi dashboard and `espnow_drive.py` add a separate `Record 30 Seconds` action that sends `RECORD_30_SECONDS`.

Compact recorder status is published once per second and includes:

- state and session ID;
- remaining milliseconds;
- attempted, saved, and dropped camera frames;
- effective FPS multiplied by 100;
- queue utilization;
- termination/error reason.

The live dashboard hides or marks the camera stream paused while recording. Motor, rudder, winch, servo-power, recording status, link health, and basic sensor telemetry remain available. Heavy full-ToF dashboard telemetry may reduce temporarily under queue or transport pressure.

## Notebook and Dataset Export

The original BoatLog remains the source of truth. A notebook validates and converts it into:

```text
session_NNNN/
  images/
    frame_000001.jpg
    ...
  frames.csv
  tof_a.csv
  tof_b.csv
  imu.csv
  fusion.csv
  gps.csv
  controls.csv
  matches.csv
```

`frames.csv` maps camera IDs and timestamps to extracted JPEGs and drop reasons. `matches.csv` contains one row per exported fused sample with the selected camera, ToF, IMU/fusion, GPS, and control generations, signed timing deltas, validity, and rejection reason.

`tof_a.csv` and `tof_b.csv` contain one row per valid generation. Wide, explicit columns preserve every target without duplication:

```text
distance_z00_t0 ... distance_z63_t3
sigma_z00_t0 ... sigma_z63_t3
status_z00_t0 ... status_z63_t3
targets_z00 ... targets_z63
```

The normalized files let pandas follow exact frame/generation references without repeating full grids across camera records. The notebook may also create a preview MP4/MJPEG and optional per-image `.txt` sidecars containing the complete matched data. JSON is not part of the required export.

## Pressure and Failure Policy

- Missing SD, unavailable buffers, or insufficient reserved space rejects the trigger before MJPEG is stopped.
- Temporary SD slowdown queues records, reduces heavy telemetry, and adapts camera FPS before dropping frames.
- A full camera queue drops and accounts for the camera frame; reserved sensor/control capacity remains available.
- Temporary missing/stale sensor data does not stop manual operation. Its match is marked invalid.
- SD write/CRC failure stops new recording work, preserves the recoverable prefix when possible, restores normal camera ownership, and reports the error.
- C6/SDIO trouble or rising command age pauses storage output and may terminate recording before control is affected.
- Camera failure finalizes the recoverable session and restores the normal camera path.
- Every termination path records a reason when storage remains usable.

The final session summary reports camera attempts/saves/drops, effective FPS, complete/rejected ToF generations, valid/invalid fused samples, maximum queue utilization, SD pauses, maximum write duration, maximum command age, and termination reason.

## Verification

### Host verification

Tests cover:

- BoatLog encode/decode, CRC, length bounds, unknown record types, and truncated-tail recovery;
- fixed-capacity pool/queue exhaustion and exact dropped-frame accounting;
- adaptive 20-to-15-to-10 FPS behavior;
- duplicate-trigger rejection and the fixed 30-second deadline;
- causal matching with no future-data leakage;
- independent ToF-A/ToF-B timestamps and stale-data rejection;
- complete 256-target ToF layout and column mapping;
- CSV export and optional text-sidecar generation;
- corrupt, missing, rejected, and stale source records.

### Hardware acceptance

Run separate WiFi-control and ESP-NOW-control tests with active recording and reduced dashboard telemetry as required:

| Requirement | Acceptance |
|---|---:|
| Session duration | 30 seconds |
| Camera target | 600 attempted frames |
| Minimum recorded | 300 frames (10 FPS floor) |
| Healthy target | Near 600 frames |
| Fused export target | Approximately 300 samples |
| Accepted camera–ToF delta | At most 75 ms |
| Motor/rudder command latency | At most 20 ms |
| Control cadence | 100 Hz |
| Recording-induced manual failsafes | 0 |
| C6/SDIO restarts | 0 |
| Torn ToF frames published | 0 |

Stress tests include repeated throttle/rudder/winch/servo-power changes, forced queue pressure, SD slowdown, camera failure, missing ToF generations, power loss mid-session, and several back-to-back 30-second sessions. Runtime metrics must show stable PSRAM/heap use, no leak across sessions, bounded write intervals, and MJPEG restoration after success and every recoverable failure.

Notebook acceptance requires successful extraction of images, normalized CSV tables, exact cross-generation matches, full ToF distances/status/sigma/counts, and optional `.txt` sidecars from both clean and interrupted sessions.

## Explicit Non-Goals

- Running PicoDet during recording
- Autonomous obstacle-avoidance decisions
- Waypoint navigation
- ML steering/throttle selection
- Browser-side real-time camera–ToF fusion
- On-device MP4 encoding
- JSON dataset export
- Reducing ToF to one target per zone

These features may consume the timestamped BoatLog and CSV contract later without changing the raw recording format.
