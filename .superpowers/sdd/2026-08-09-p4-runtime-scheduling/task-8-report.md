# Task 8 Report — Sole-owner SensorBus and Core 1 ToF processing

## Delivered

- Replaced the runtime I2C mutex with one Core 0 \`SensorBus\` owner. It reads
  the IMU first on every absolute 20 ms iteration, publishes the existing raw
  IMU snapshot, and notifies FusionTask without inference gating.
- Integrated the Task 6 policy for independent 5 Hz ToF A/B due times. Each
  sensor starts with a 15,000 us budget, retains its own maximum observed
  ranging/readiness duration, and is eligible only when that budget plus the
  policy's 2 ms guard fits before the next absolute IMU deadline.
- Added two fixed raw result buffers per ToF sensor. Lock-free per-slot atomic
  claims prevent Core 0 from reusing a buffer while Core 1 copies it; a busy
  slot is counted as a skip and retried without risking the IMU deadline.
- Added notification-driven Core 1 \`ToFProc\`. It copies a stable published
  generation, preserves the existing target-status gating and three-frame
  median conversion, stores the existing cache, counts overwritten
  generations, and catches up to the newest result before waiting again.
- Kept SnapshotTask at 20 Hz as a cache-only consumer. It still publishes the
  existing fusion, ToF, GPS, detections, and status data, but no longer reads
  the IMU or waits for inference.
- Created Fusion, SensorBus, ToFProc, and Snapshot through their approved
  runtime task IDs. Fusion's handle is registered before SensorBus starts.
- Removed \`g_i2c_mutex\` from runtime application sources and updated IMU driver
  ownership comments to require SensorBusTask after serialized startup.

## TDD evidence

- RED: \`.venv-runtime-tests/bin/python -m pytest
  tests/test_runtime_architecture.py -q\` failed in the new sole-owner test on
  the live \`g_i2c_mutex\` definition (\`1 failed, 4 passed\`).
- GREEN: the required focused command passed after implementation
  (\`7 passed in 0.59s\`).
- Self-review found that a generation-only double-buffer validation could
  permit concurrent reuse during a slow cross-core copy. It was replaced with
  lock-free per-slot read/write claims, then the focused tests and P4 build
  were rerun.

## Verification

- \`.venv-runtime-tests/bin/python -m pytest tests/test_sensor_schedule.py
  tests/test_sample_snapshot.py tests/test_runtime_architecture.py -q\`:
  \`7 passed in 0.59s\`.
- \`.venv-runtime-tests/bin/python -m pytest -q
  --ignore=tests/test_espnow_drive.py\`: \`13 passed in 3.68s\`.
- ESP32-P4 \`idf.py build\` with \`IDF_TOOLS_PATH=/opt/esp\` and the supplied
  \`/opt/esp/python_env/idf5.4_py3.12_env\`: passed. Image size is \`0x407900\`;
  the smallest app partition has 50% free.
- Ownership scan: no \`g_i2c_mutex\` in application sources, no inference gate
  in \`sensor_task.c\` or \`sensor_fusion.c\`, and the sole runtime
  \`tof_read_grid()\` call is in \`task_sensor_bus\`.
- \`git diff --check\`: clean.

## Scope

No task affinity changes beyond the already-approved runtime task IDs, GPS
ownership changes, C6/S3 work, or Task 9 behavior were added. The pre-existing
untracked \`.venv-runtime-tests/\` directory remains untouched and is excluded
from the commit.
