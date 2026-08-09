# ESP32-P4 Runtime Scheduling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a deterministic two-core runtime in which Core 0 exclusively protects manual control, actuator safety, GPS, IMU/magnetometer, and I2C deadlines while Core 1 runs camera, detection, snapshots, telemetry, diagnostics, optional microSD training recording, and future autonomy workloads.

**Architecture:** Add one central task registry, a bounded metrics layer, a manual-only command ingress/arbiter, one Core 0 actuator owner, and one Core 0 I2C owner. Pass timestamped fixed-size snapshots across cores; record optional training samples through a bounded Core 1 JPEG-copy pool and event-driven microSD writer; reserve disabled waypoint/ML drive slots and a passive LP-core heartbeat without implementing autonomous driving or LP hardware actuation.

**Tech Stack:** ESP-IDF 5.4, ESP32-P4 SMP FreeRTOS, C11/C++17, ESP timer, I2C master, UART, MCPWM, nanopb, ESP-DL, ULP LP-core build system, Python `unittest`/`pytest`, host C compiler.

## Global Constraints

- Manual control is the only enabled drive source; waypoint and ML source identifiers exist but have no producer task and cannot be selected.
- Future waypoint and ML sources may propose throttle and rudder only; winch, servo power, arm/disarm, and failsafe remain Core 0 manual/safety functions.
- Core 0 control cadence is 10 ms (100 Hz); accepted command age of 400 ms enters failsafe on the next tick, producing a 400-410 ms response window.
- IMU/magnetometer target is 20 ms (50 Hz), with no accepted hardware stress-test gap above 40 ms.
- ToF A and B each target 5 Hz and must yield to the next IMU deadline.
- P4 receive-to-physical-command latency, servo-power OFF, and winch stop must each remain at or below 20 ms under stress.
- Camera or inference work must never gate Core 0 control, IMU, magnetometer, fusion, ToF scheduling, or snapshot production.
- Training recording is optional and runs only on Core 1. Its only producer-side work is a bounded JPEG copy into a preallocated PSRAM slot; a full pool, queue, slow card, write failure, or missing card drops the training sample and increments metrics rather than blocking any camera, radio, sensor, or control path.
- No task on Core 0 may mount, open, write, flush, or close microSD files. An unavailable card must leave boot, Wi-Fi/ESP-NOW, manual driving, and all safety behavior unchanged.
- Future ML remains unimplemented; preserve and measure room for the agreed 260 ms path (50 ms capture/preprocess, 100 ms inference hard budget, 20 ms policy, 10 ms handoff, 80 ms contingency) while the currently tested PicoDet inference is approximately 57 ms.
- Runtime publishing and metrics use fixed-size startup allocations; no control/sensor update allocates memory after startup.
- Critical Core 0 task creation failure keeps outputs de-energized and aborts startup visibly; optional Core 1 failure disables only that feature.
- LP core is an observer only and must not include or call GPIO, MCPWM, ESC, winch, or steering APIs.
- No C6 or S3 firmware changes are part of this implementation.
- Preserve unrelated untracked `.superpowers/` and `boatMainReimagine.FCStd` files.

## File structure

| File | Responsibility |
|---|---|
| `main/runtime_schedule.h/.c` | Authoritative task IDs, names, cores, priorities, stacks, periods, deadlines, and criticality |
| `main/runtime_task.h/.c` | Checked `xTaskCreatePinnedToCore` wrapper and task-handle registration |
| `main/runtime_metrics.h/.c` | Fixed-size timing/counter records and compact one-second diagnostics |
| `main/control_arbiter.h/.c` | Pure-C manual drive slots, disabled future source slots, auxiliary slots, discrete safety-event ring, and 400 ms decision logic |
| `main/arm_sequence.h/.c` | Pure-C, nonblocking, cancelable three-second ESC arming state machine |
| `main/motor_control.c/.h` | Pipeline ingress adapters, Core 0 `ControlTask`, Core 0 `ArmSequencer`, and sole actuator application path |
| `main/drivers/esc_driver.c/.h` | Nonblocking begin/complete/disarm primitives; no three-second delay inside the driver |
| `main/sensor_schedule.h/.c` | Pure-C 20 ms IMU / alternating 5 Hz ToF deadline policy |
| `main/imu_sample.h` | Timestamped raw IMU/magnetometer sample contract |
| `main/sample_snapshot.h/.c` | Fixed-size versioned double-buffer for cross-task samples |
| `main/training_logger.h/.c` | Fixed-size Core 1 JPEG pool, metadata contract, and non-blocking microSD training writer |
| `main/Kconfig.projbuild` | Opt-in training-recorder configuration and conservative capture/pool limits |
| `main/sensor_task.c/.h` | Sole Core 0 I2C owner plus Core 1 ToF processing and existing snapshot publication |
| `main/sensor_fusion.c/.h` | Fusion math from cached raw samples; no I2C access |
| `main/lp_supervisor.h/.c` | HP-side LP binary loader, heartbeat writer, and observation reader |
| `main/ulp/CMakeLists.txt` | LP-core subproject build |
| `main/ulp/main.c` | Passive heartbeat observer, no hardware actuation |
| `tests/test_runtime_*.c/.py` | Host-compiled policy/state tests |
| `tests/test_runtime_architecture.py` | Source-level ownership/affinity invariants |
| `tools/runtime_stress_check.py` | Parse compact `RTM` logs and enforce bench thresholds |

---

### Task 1: Centralize the application task schedule

**Files:**
- Create: `main/runtime_schedule.h`
- Create: `main/runtime_schedule.c`
- Create: `main/runtime_task.h`
- Create: `main/runtime_task.c`
- Create: `tests/test_runtime_schedule.c`
- Create: `tests/test_runtime_schedule.py`
- Modify: `main/CMakeLists.txt:1-25`

**Interfaces:**
- Produces: `runtime_task_id_t`, `runtime_task_spec_t`, `runtime_schedule_get(runtime_task_id_t)`, `runtime_schedule_validate(void)`, and `runtime_task_create(runtime_task_id_t, TaskFunction_t, void *, TaskHandle_t *)`.
- Consumes: ESP-IDF `xTaskCreatePinnedToCore` only in `runtime_task.c`.

- [ ] **Step 1: Write the failing schedule-table test**

```c
#include <assert.h>
#include "runtime_schedule.h"

int main(void) {
    assert(runtime_schedule_validate());
    const runtime_task_spec_t *control = runtime_schedule_get(RUNTIME_TASK_CONTROL);
    const runtime_task_spec_t *bus = runtime_schedule_get(RUNTIME_TASK_SENSOR_BUS);
    const runtime_task_spec_t *gps = runtime_schedule_get(RUNTIME_TASK_GPS);
    const runtime_task_spec_t *detect = runtime_schedule_get(RUNTIME_TASK_DETECT);
    const runtime_task_spec_t *training = runtime_schedule_get(RUNTIME_TASK_TRAINING_LOG);
    assert(control->core == 0 && control->priority == 10 && control->period_us == 10000);
    assert(bus->core == 0 && bus->priority == 8 && bus->period_us == 20000);
    assert(gps->core == 0 && gps->priority == 6);
    assert(detect->core == 1 && detect->priority == 7 && detect->stack_size == 32768);
    assert(training->core == 1 && training->priority == 2 && !training->critical);
    assert(runtime_schedule_get(RUNTIME_TASK_WAYPOINT) == 0);
    assert(runtime_schedule_get(RUNTIME_TASK_ML_CONTROL) == 0);
    return 0;
}
```

The Python wrapper follows `tests/test_wifi_fallback.py`: compile `main/runtime_schedule.c` plus this C file with `cc -std=c11 -Wall -Wextra -Werror -Imain`, run the temporary binary, and require exit code zero.

- [ ] **Step 2: Run the focused test and confirm the missing-module failure**

Run: `python -m pytest tests/test_runtime_schedule.py -q`

Expected: FAIL because `runtime_schedule.h` and `runtime_schedule.c` do not exist.

- [ ] **Step 3: Define the authoritative schedule**

Define this exact public shape:

```c
typedef enum {
    RUNTIME_TASK_CONTROL,
    RUNTIME_TASK_ARM_SEQUENCE,
    RUNTIME_TASK_SENSOR_BUS,
    RUNTIME_TASK_FUSION,
    RUNTIME_TASK_GPS,
    RUNTIME_TASK_DETECT,
    RUNTIME_TASK_CAMERA_DRAIN,
    RUNTIME_TASK_SNAPSHOT,
    RUNTIME_TASK_TOF_PROCESS,
    RUNTIME_TASK_WS_TX,
    RUNTIME_TASK_DIAGNOSTICS,
    RUNTIME_TASK_TRAINING_LOG,
    RUNTIME_TASK_STATUS_LED,
    RUNTIME_TASK_COUNT,
    RUNTIME_TASK_WAYPOINT,
    RUNTIME_TASK_ML_CONTROL,
} runtime_task_id_t;

typedef struct {
    const char *name;
    uint32_t stack_size;
    uint8_t priority;
    int8_t core;
    uint32_t period_us;
    uint32_t deadline_us;
    bool critical;
} runtime_task_spec_t;
```

Populate the table with: Control `4096/10/0/10000/10000/critical`; ArmSeq `4096/9/0/event/critical`; SensorBus `8192/8/0/20000/20000/critical`; Fusion `4096/7/0/event/40000/critical`; GPS `4096/6/0/event/soft`; Detect `32768/7/1/event/soft`; CamDrain `2048/6/1/event/soft`; Snapshot `16384/5/1/50000/50000/soft`; ToFProc `6144/4/1/event/200000/soft`; WS_TX `8192/3/1/event/soft`; Diagnostics `4096/2/1/1000000/soft`; TrainingLog `8192/2/1/event/soft`; StatusLED `2048/2/1/event/soft`. Return `NULL` for the waypoint and ML IDs.

- [ ] **Step 4: Implement checked pinned task creation**

```c
esp_err_t runtime_task_create(runtime_task_id_t id, TaskFunction_t fn,
                              void *arg, TaskHandle_t *out) {
    const runtime_task_spec_t *spec = runtime_schedule_get(id);
    if (!spec || !fn) return ESP_ERR_INVALID_ARG;
    BaseType_t ok = xTaskCreatePinnedToCore(fn, spec->name, spec->stack_size,
                                            arg, spec->priority, out, spec->core);
    if (ok != pdPASS) {
        ESP_LOGE("RUNTIME", "task create failed: %s core=%d prio=%u stack=%u critical=%d",
                 spec->name, spec->core, spec->priority,
                 (unsigned)spec->stack_size, spec->critical);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
```

Register both new `.c` files in `main/CMakeLists.txt`, but do not migrate existing task creation yet.

- [ ] **Step 5: Run the focused test and P4 build**

Run: `python -m pytest tests/test_runtime_schedule.py -q`

Expected: PASS.

Run: `bash -lc 'source /opt/esp/idf/export.sh >/dev/null && idf.py build'`

Expected: build completes and links `runtime_schedule.c` and `runtime_task.c` without changing runtime behavior.

- [ ] **Step 6: Commit the schedule foundation**

```bash
git add main/runtime_schedule.h main/runtime_schedule.c main/runtime_task.h main/runtime_task.c main/CMakeLists.txt tests/test_runtime_schedule.c tests/test_runtime_schedule.py
git commit -m "feat(runtime): centralize P4 task schedule"
```

### Task 2: Add bounded runtime metrics and capture the baseline

**Files:**
- Create: `main/runtime_metrics.h`
- Create: `main/runtime_metrics.c`
- Create: `tests/test_runtime_metrics.c`
- Create: `tests/test_runtime_metrics.py`
- Modify: `main/runtime_task.c`
- Modify: `main/main.c:294-315`
- Modify: `main/CMakeLists.txt`
- Modify: `sdkconfig.defaults`

**Interfaces:**
- Produces: `runtime_metrics_init()`, `runtime_metrics_record_cycle(id, scheduled_us, started_us, finished_us)`, `runtime_metrics_cycle_begin(id, scheduled_us, started_us)`, `runtime_metrics_cycle_end(id, finished_us)`, `runtime_metrics_count(id, event)`, `runtime_metrics_set_stack(id, words)`, `runtime_metrics_snapshot(id, out)`, and `task_runtime_diagnostics(void *)`.
- Consumes: task specs and handles from Task 1.

- [ ] **Step 1: Write the failing accumulator test**

Test three cycles: `(scheduled,start,finish)=(0,100,600)`, `(10000,10500,11200)`, and `(20000,45000,46000)`. Assert `runs=3`, `max_exec_us=1000`, `max_jitter_us=25000`, `deadline_misses=1` for a 10 ms deadline, and `max_gap_us=34500` between starts.

```c
runtime_metrics_reset_for_test();
runtime_metrics_record_cycle(RUNTIME_TASK_CONTROL, 0, 100, 600);
runtime_metrics_record_cycle(RUNTIME_TASK_CONTROL, 10000, 10500, 11200);
runtime_metrics_record_cycle(RUNTIME_TASK_CONTROL, 20000, 45000, 46000);
runtime_metric_snapshot_t m;
runtime_metrics_snapshot(RUNTIME_TASK_CONTROL, &m);
assert(m.runs == 3 && m.max_exec_us == 1000);
assert(m.max_jitter_us == 25000 && m.deadline_misses == 1);
assert(m.max_gap_us == 34500);
```

- [ ] **Step 2: Run the test and confirm missing symbols**

Run: `python -m pytest tests/test_runtime_metrics.py -q`

Expected: FAIL because the metrics API does not exist.

- [ ] **Step 3: Implement fixed records and compact logs**

Use one fixed record per `RUNTIME_TASK_COUNT`, C11 32-bit atomic counters plus a per-record sequence lock, eight execution-time histogram buckets (`<=100`, `<=250`, `<=500`, `<=1000`, `<=2500`, `<=5000`, `<=10000`, `>10000` microseconds), and no heap allocation after `runtime_metrics_init()`. Define `runtime_metric_event_t` with `RUNTIME_EVENT_DEADLINE_MISS`, `RUNTIME_EVENT_COMMAND_OVERWRITE`, `RUNTIME_EVENT_INVALID_COMMAND`, `RUNTIME_EVENT_SENSOR_SKIP`, `RUNTIME_EVENT_SENSOR_ERROR`, and `RUNTIME_EVENT_FEATURE_DISABLED`. Clamp calculated durations/gaps to `UINT32_MAX`; keep absolute timestamps local to the task so the host-testable accumulator does not require ESP-IDF headers. Guard the diagnostics task declarations/implementation with `#ifdef ESP_PLATFORM` so the same `.c` file compiles in the host harness.

Emit one parseable line per registered task each second:

```text
RTM task=Control core=0 prio=10 runs=100 max_exec_us=420 max_gap_us=10110 max_jitter_us=110 misses=0 stack_free_words=1880
```

Enable `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y` in `sdkconfig.defaults`; let it select trace/stat formatting dependencies. Use `uxTaskGetSystemState()` in the diagnostics task to calculate Core 0/Core 1 utilization when the returned task records expose core/runtime data; otherwise emit `core_load_supported=0` and retain the application timing metrics.

- [ ] **Step 4: Register task handles and start diagnostics without changing affinity**

Call `runtime_metrics_init()` near the start of `app_main`, before any subsystem can call `runtime_task_create()`. Have `runtime_task_create()` call `runtime_metrics_register_handle(id, *out)` after successful creation. Temporarily start `task_runtime_diagnostics` from `app_main` through `runtime_task_create(RUNTIME_TASK_DIAGNOSTICS, ...)`. Add begin/end calls to the current IMU, ToF, snapshot, detection, camera-drain, and WS-TX loops using `esp_timer_get_time()`.

- [ ] **Step 5: Run host tests and build**

Run: `python -m pytest tests/test_runtime_schedule.py tests/test_runtime_metrics.py -q`

Expected: PASS.

Run: `bash -lc 'source /opt/esp/idf/export.sh >/dev/null && idf.py reconfigure && idf.py build'`

Expected: PASS; `sdkconfig` reflects generated run-time statistics support.

- [ ] **Step 6: Capture a pre-affinity hardware baseline**

Require an explicitly verified P4 port and save the log:

```bash
: "${BOAT_P4_PORT:?Set BOAT_P4_PORT to the verified ESP32-P4 serial device}"
test -c "$BOAT_P4_PORT"
source /opt/esp/idf/export.sh >/dev/null
idf.py -p "$BOAT_P4_PORT" flash monitor | tee build/runtime-baseline.log
```

During the ten-minute capture, exercise manual steering, winch, servo power, camera, ToF, GPS, telemetry, and repeated Detect commands. Stop the monitor with `Ctrl-]`. Preserve the log under `build/` only; do not commit it.

- [ ] **Step 7: Commit metrics**

```bash
git add main/runtime_metrics.h main/runtime_metrics.c main/runtime_task.c main/main.c main/CMakeLists.txt sdkconfig.defaults sdkconfig tests/test_runtime_metrics.c tests/test_runtime_metrics.py
git commit -m "feat(runtime): add bounded scheduling metrics"
```

### Task 3: Build the manual-only command ingress and arbiter

**Files:**
- Create: `main/control_arbiter.h`
- Create: `main/control_arbiter.c`
- Create: `tests/test_control_arbiter.c`
- Create: `tests/test_control_arbiter.py`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Produces: `control_arbiter_init`, `control_arbiter_submit_drive`, `control_arbiter_submit_winch`, `control_arbiter_submit_steer`, `control_arbiter_submit_steer_raw`, `control_arbiter_push_event`, and `control_arbiter_decide`.
- Produces disabled `CONTROL_SOURCE_WAYPOINT` and `CONTROL_SOURCE_ML` enum values but rejects them with `CONTROL_REJECT_SOURCE_DISABLED`.
- Consumes no ESP-IDF headers; callers supply monotonic microseconds.

- [ ] **Step 1: Write failing policy tests**

Cover these exact cases in one C binary:

```c
control_arbiter_t a;
control_arbiter_init(&a);
assert(control_arbiter_submit_drive(&a, CONTROL_SOURCE_MANUAL, 0.4f, 0.2f, 1000) == CONTROL_ACCEPTED);
assert(control_arbiter_submit_drive(&a, CONTROL_SOURCE_ML, 0.8f, 0.0f, 1001) == CONTROL_REJECT_SOURCE_DISABLED);

control_decision_t d;
control_arbiter_decide(&a, 2000, &d);
assert(!d.failsafe && d.left == 0.6f && d.right == 0.2f);

control_arbiter_submit_drive(&a, CONTROL_SOURCE_MANUAL, 0.1f, -0.1f, 3000);
control_arbiter_decide(&a, 4000, &d);
assert(d.left == 0.0f && d.right == 0.2f); /* newest command, no backlog */

assert(control_arbiter_push_event(&a, CONTROL_EVENT_SERVO_POWER_OFF, 5000));
for (int i = 0; i < CONTROL_EVENT_CAPACITY * 2; ++i)
    control_arbiter_submit_drive(&a, CONTROL_SOURCE_MANUAL, 0.2f, 0.0f, 5001 + i);
control_arbiter_decide(&a, 6000, &d);
assert(d.servo_power_off);

control_arbiter_decide(&a, 405100, &d);
assert(d.failsafe);
```

Add range tests for NaN, throttle/rudder outside `[-1,1]`, winch outside `[-1,1]`, raw steering pulse outside the driver's existing calibration safety envelope of `400..2600` microseconds, and queue saturation. OFF/disarm events must either remain queued or coalesce into the urgent bitset; they may never return false.

- [ ] **Step 2: Run the focused test and confirm failure**

Run: `python -m pytest tests/test_control_arbiter.py -q`

Expected: FAIL because the arbiter files do not exist.

- [ ] **Step 3: Implement fixed latest-value slots and a bounded event ring**

Use these public types:

```c
typedef enum { CONTROL_SOURCE_MANUAL, CONTROL_SOURCE_WAYPOINT, CONTROL_SOURCE_ML } control_source_t;
typedef enum {
    CONTROL_EVENT_SERVO_POWER_ON,
    CONTROL_EVENT_SERVO_POWER_OFF,
    CONTROL_EVENT_ARM,
    CONTROL_EVENT_FORCE_ARM,
    CONTROL_EVENT_DISARM,
} control_event_kind_t;

typedef struct {
    float left;
    float right;
    float winch;
    float steer;
    uint32_t steer_raw_us;
    bool steer_raw;
    bool drive_changed;
    bool winch_changed;
    bool steer_changed;
    bool servo_power_on;
    bool servo_power_off;
    bool arm;
    bool force_arm;
    bool disarm;
    bool failsafe;
    int64_t newest_rx_us;
} control_decision_t;
```

Store drive proposals in a three-entry source array, but keep `active_source=MANUAL` and an enabled bitmask containing only manual. Continuous slots overwrite by timestamp. Discrete events use an eight-entry ring; OFF/disarm also set urgent coalescing bits before ring insertion so ring saturation cannot lose the safe action.

- [ ] **Step 4: Run arbiter tests and register the module**

Run: `python -m pytest tests/test_control_arbiter.py -q`

Expected: PASS.

Add `control_arbiter.c` to `main/CMakeLists.txt`, then run the P4 build.

- [ ] **Step 5: Commit the pure policy layer**

```bash
git add main/control_arbiter.h main/control_arbiter.c main/CMakeLists.txt tests/test_control_arbiter.c tests/test_control_arbiter.py
git commit -m "feat(control): add manual-only command arbiter"
```

### Task 4: Make one Core 0 task own throttle, rudders, winch, servo power, and failsafe

**Files:**
- Modify: `main/motor_control.c:275-470`
- Modify: `main/motor_control.h`
- Modify: `main/pipeline.c:213-305`
- Create: `tests/test_runtime_architecture.py`

**Interfaces:**
- Consumes: Task 3 arbiter and Task 1 `RUNTIME_TASK_CONTROL`.
- Produces: `motor_control_notify_link_rx(int64_t received_us)`, `motor_control_get_status(boat_MotorStatus *)`, and a persistent Core 0 `ControlTask`.

- [ ] **Step 1: Write the failing actuator-ownership source test**

The Python test reads `main/motor_control.c`, extracts each pipeline handler body, and asserts that `motor_command_handler`, `winch_command_handler`, `steer_command_handler`, `steer_raw_command_handler`, and `servo_power_command_handler` contain `control_arbiter_` submission calls but none of:

```python
FORBIDDEN = (
    "esc_driver_set_throttle(", "winch_driver_set_speed(",
    "winch_driver_set_power(", "steer_driver_set(",
    "steer_driver_set_raw_us(",
)
```

It also asserts that all five driver-write tokens occur inside one `control_apply_decision()` function or boot/shutdown functions explicitly named in the test. The test rejects every `pipeline_publish_` call from `task_control` and `control_apply_decision` so Core 0 can never block on protobuf encoding or transport fanout.

- [ ] **Step 2: Run the architecture test and confirm current direct-write failure**

Run: `python -m pytest tests/test_runtime_architecture.py -q`

Expected: FAIL because the current handlers call drivers directly and the watchdog timer also writes hardware.

- [ ] **Step 3: Convert handlers to bounded ingress adapters**

Keep current protobuf decoding and handler registration. Under a short `portMUX_TYPE` critical section, handlers submit already-decoded values to the arbiter and update the accepted-control timestamp only for motor, winch, steer, steer-raw, servo-power, and arm/disarm commands. Detect commands must not refresh propulsion freshness.

Delete `pipeline.c`'s unconditional `s_last_rx_us` update and remove `pipeline_recent_command()`. The arbiter's newest accepted manual-control timestamp becomes the only 400 ms liveness input; a decodable Detect message can no longer keep propulsion armed by itself.

Preserve motor mixing exactly:

```c
left = cmd->left != 0.0f || cmd->right != 0.0f
     ? cmd->left : cmd->throttle + cmd->rudder;
right = cmd->left != 0.0f || cmd->right != 0.0f
      ? cmd->right : cmd->throttle - cmd->rudder;
if (fmaxf(fabsf(left), fabsf(right)) > 1.0f) {
    float scale = fmaxf(fabsf(left), fabsf(right));
    left /= scale;
    right /= scale;
}
```

Notify `ControlTask` immediately for servo-power OFF and disarm. Continuous values wait no longer than the next 10 ms control tick.

- [ ] **Step 4: Implement the absolute-deadline control loop**

`ControlTask` uses an absolute next-deadline tick. An urgent notification wakes it early without moving the next scheduled deadline. It snapshots/decides under the arbiter lock, releases the lock, then calls one function that applies changed outputs.

```c
static void task_control(void *arg) {
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        const int64_t start_us = esp_timer_get_time();
        control_decision_t d;
        control_arbiter_decide(&s_arbiter, start_us, &d);
        control_apply_decision(&d);
        next += pdMS_TO_TICKS(10);
        TickType_t now = xTaskGetTickCount();
        if (next > now) ulTaskNotifyTake(pdTRUE, next - now);
        else { runtime_metrics_count(RUNTIME_TASK_CONTROL, RUNTIME_EVENT_DEADLINE_MISS); next = now; }
    }
}
```

Move the watchdog actions into this decision/application path and remove the `esp_timer` watchdog. Failsafe keeps the current behavior: throttle zero, winch zero, steering centre (`0.0f`), servo rail off, and no persistent explicit rail-cut latch.

Preserve the existing heading-assist dry-run cadence with a divide-by-10 counter inside `ControlTask`: call `heading_assist_dry_run_tick()` every 100 ms, never from an ESP timer callback, and keep dry-run output unable to change actuators.

Explicit servo-power OFF keeps its distinct behavior: latch `s_rail_cut=true`, winch zero, steering home (`1.0f`), heading-assist reset. Explicit ON clears the latch. Nonzero steering/winch auto-powers only when the explicit latch is clear.

After applying a changed output, publish a small versioned `boat_MotorStatus` double buffer in `motor_control.c`; `motor_control_get_status()` copies a stable generation without calling a transport. Extend the existing Core 1 Diagnostics task to call `motor_control_get_status()` and `pipeline_publish_motor_status()` at 1 Hz or when the dirty generation changes. Delete the old Core 0 `publish_status()`/`fanout_locked()` call path.

- [ ] **Step 5: Run tests and firmware build**

Run: `python -m pytest tests/test_control_arbiter.py tests/test_runtime_architecture.py -q`

Expected: PASS.

Run the P4 build and verify `motor_wd` no longer appears in the map/log while `Control` is created on Core 0 priority 10.

- [ ] **Step 6: Commit actuator ownership**

```bash
git add main/motor_control.c main/motor_control.h main/pipeline.c tests/test_runtime_architecture.py
git commit -m "refactor(control): centralize actuator ownership"
```

### Task 5: Replace blocking ESC arming with a cancelable sequence

**Files:**
- Create: `main/arm_sequence.h`
- Create: `main/arm_sequence.c`
- Create: `tests/test_arm_sequence.c`
- Create: `tests/test_arm_sequence.py`
- Modify: `main/drivers/esc_driver.h`
- Modify: `main/drivers/esc_driver.c:153-210`
- Modify: `main/motor_control.c:361-387,474-505`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Produces: `arm_sequence_request`, `arm_sequence_step`, `esc_driver_arm_begin`, and `esc_driver_arm_complete`.
- Consumes: GPS lock/force decision, monotonic microseconds, and urgent disarm events.

- [ ] **Step 1: Write failing arm-state tests**

```c
arm_sequence_t s;
arm_sequence_init(&s, 3000000);
assert(arm_sequence_request(&s, ARM_REQUEST_ARM, false, 1000));
assert(arm_sequence_step(&s, 1000, false) == ARM_ACTION_REJECT_NO_GPS);
assert(arm_sequence_request(&s, ARM_REQUEST_ARM, true, 2000));
assert(arm_sequence_step(&s, 2000, false) == ARM_ACTION_BEGIN);
assert(arm_sequence_step(&s, 3001999, false) == ARM_ACTION_NONE);
assert(arm_sequence_step(&s, 3002000, false) == ARM_ACTION_COMPLETE);

arm_sequence_init(&s, 3000000);
arm_sequence_request(&s, ARM_REQUEST_ARM, true, 0);
assert(arm_sequence_step(&s, 0, false) == ARM_ACTION_BEGIN);
arm_sequence_request(&s, ARM_REQUEST_DISARM, false, 1000000);
assert(arm_sequence_step(&s, 1000000, false) == ARM_ACTION_DISARM);
assert(arm_sequence_step(&s, 4000000, false) == ARM_ACTION_NONE);
```

- [ ] **Step 2: Confirm the test fails, then implement the pure state machine**

Run: `python -m pytest tests/test_arm_sequence.py -q`

Expected before implementation: FAIL for missing files. After implementation: PASS.

- [ ] **Step 3: Split the ESC driver's blocking operation**

Replace the three-second `vTaskDelay` inside `esc_driver_arm()` with:

```c
esp_err_t esc_driver_arm_begin(void);    /* DISARMED -> ARMING, neutral PWM */
esp_err_t esc_driver_arm_complete(void); /* ARMING -> ARMED, no delay */
```

Keep the existing driver mutex/state checks. Remove the blocking `esc_driver_arm()` API after all callers migrate; retain `esc_driver_disarm()`.

- [ ] **Step 4: Create one persistent ArmSeq task**

Replace per-command `xTaskCreate(arm_task_fn, ...)` with one `RUNTIME_TASK_ARM_SEQUENCE` task and a fixed queue of arm requests. The task runs `arm_sequence_step`, checks `gps_driver_has_lock()` for non-force requests, and submits `BEGIN`, `COMPLETE`, or `DISARM` as discrete events to `ControlTask`. It never calls ESC, winch, steer, or GPIO drivers. `ControlTask` alone invokes the new ESC begin/complete/disarm primitives.

Use queue receive with a timeout equal to the next state-machine deadline so a disarm request cancels the three-second wait immediately.

- [ ] **Step 5: Extend architecture tests and build**

Assert that `motor_control.c` contains no `xTaskCreate(`, `arm_task_fn`, or `vTaskDelay(pdMS_TO_TICKS(ESC_ARMING_DELAY_MS))`; assert that driver calls do not occur in the ArmSeq task body.

Run all control tests, then the P4 build.

- [ ] **Step 6: Commit nonblocking arming**

```bash
git add main/arm_sequence.h main/arm_sequence.c main/drivers/esc_driver.h main/drivers/esc_driver.c main/motor_control.c main/CMakeLists.txt tests/test_arm_sequence.c tests/test_arm_sequence.py tests/test_runtime_architecture.py
git commit -m "refactor(control): make ESC arming nonblocking"
```

### Task 6: Implement the IMU-first / alternating-ToF scheduling policy

**Files:**
- Create: `main/sensor_schedule.h`
- Create: `main/sensor_schedule.c`
- Create: `tests/test_sensor_schedule.c`
- Create: `tests/test_sensor_schedule.py`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Produces: `sensor_schedule_init`, `sensor_schedule_next_imu_deadline`, `sensor_schedule_choose_tof`, `sensor_schedule_note_tof_result`, and `sensor_schedule_note_skip`.
- Consumes: current monotonic time, measured ToF budget, 2 ms guard, and sensor availability.

- [ ] **Step 1: Write failing deterministic schedule tests**

Test the exact policy:

```c
sensor_schedule_t s;
sensor_schedule_init(&s, 0);
assert(sensor_schedule_next_imu_deadline(&s) == 20000);
assert(sensor_schedule_choose_tof(&s, 3000, 20000, 12000, true, true) == SENSOR_TOF_A);
sensor_schedule_note_tof_result(&s, SENSOR_TOF_A, 12000, true);
assert(sensor_schedule_choose_tof(&s, 103000, 120000, 12000, true, true) == SENSOR_TOF_B);
sensor_schedule_note_tof_result(&s, SENSOR_TOF_B, 12000, true);
assert(sensor_schedule_choose_tof(&s, 203000, 220000, 12000, true, true) == SENSOR_TOF_A);

sensor_schedule_init(&s, 0);
assert(sensor_schedule_choose_tof(&s, 7000, 20000, 12000, true, true) == SENSOR_TOF_NONE); /* 12 ms + 2 ms guard > 13 ms */
assert(s.deadline_protection_skips == 1);
```

Also test A absent, B absent, both absent, failed reads, and that a skipped oldest-due sensor is retried rather than silently advanced.

- [ ] **Step 2: Run failing test, implement policy, run passing test**

Run: `python -m pytest tests/test_sensor_schedule.py -q`

Expected before: FAIL for missing module. Expected after: PASS.

Use independent `next_due_us` values initialized as A=`start`, B=`start+100000`; successful or attempted reads advance that sensor by 200000 microseconds. Select the oldest available due sensor only when `next_imu_deadline_us - now_us >= measured_budget_us + 2000`.

- [ ] **Step 3: Commit the pure sensor policy**

```bash
git add main/sensor_schedule.h main/sensor_schedule.c main/CMakeLists.txt tests/test_sensor_schedule.c tests/test_sensor_schedule.py
git commit -m "feat(sensors): add deadline-aware sensor schedule"
```

### Task 7: Separate raw IMU acquisition from fusion math

**Files:**
- Create: `main/imu_sample.h`
- Create: `main/sample_snapshot.h`
- Create: `main/sample_snapshot.c`
- Create: `tests/test_sample_snapshot.c`
- Create: `tests/test_sample_snapshot.py`
- Modify: `main/sensor_fusion.h`
- Modify: `main/sensor_fusion.c:83-305`
- Modify: `main/sensor_task.h`
- Modify: `main/sensor_task.c`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Produces: `imu_sample_t`, `sample_snapshot_publish`, `sample_snapshot_read`, and `fusion_update_sample(const imu_sample_t *)`.
- Consumes: existing calibration/fusion equations unchanged after raw values and `dt` are supplied.

- [ ] **Step 1: Write the failing versioned-snapshot test**

Define `imu_sample_t` with `ax/ay/az`, `gx/gy/gz`, `mx/my/mz`, `accel_gyro_valid`, `mag_valid`, `captured_us`, and `sequence`. Publish samples 1 and 2, then assert the reader obtains a complete matching generation and timestamp rather than mixed fields.

```c
sample_snapshot_t box;
sample_snapshot_init(&box);
imu_sample_t one = {.ax=1, .mx=11, .captured_us=100, .sequence=1};
imu_sample_t two = {.ax=2, .mx=22, .captured_us=200, .sequence=2};
sample_snapshot_publish(&box, &one);
sample_snapshot_publish(&box, &two);
imu_sample_t out;
assert(sample_snapshot_read(&box, &out));
assert(out.sequence == 2 && out.ax == 2 && out.mx == 22 && out.captured_us == 200);
```

- [ ] **Step 2: Implement the fixed double buffer and pass its host test**

Run: `python -m pytest tests/test_sample_snapshot.py -q`

Expected before: FAIL. Expected after: PASS under both normal and a 100,000-iteration writer/reader stress test compiled with `-pthread`.

- [ ] **Step 3: Refactor fusion to consume a sample**

Move `imu_read_accel_gyro`, `imu_read_mag`, bus scan, reinitialize, and recover calls out of `sensor_fusion.c`. Keep the existing complementary-filter, magnetometer validity, yaw initialization, calibration, and diagnostic equations in `fusion_update_sample()` using `sample->captured_us` for `dt`.

`task_imu_fusion` becomes an event consumer: wait for notification, load the newest sample, call `fusion_update_sample`, record skipped generations, and publish the existing `FusionResult` through the versioned snapshot below. It must contain no I2C call or I2C mutex.

Replace the fusion-result mutex with a two-entry sequence-locked `FusionResult` snapshot. `fusion_get_result()` retains its existing signature but copies a stable published generation, so Core 1 snapshot production never locks Core 0 fusion.

- [ ] **Step 4: Move IMU health/recovery ownership to the sensor side**

Add `sensor_read_imu_sample(imu_sample_t *)` in `sensor_task.c`. Call, in order, `imu_read_accel_gyro`, `imu_read_mag`, the current health-counter logic, and the current `imu_reinit_*`/`imu_recover_*` functions at the same sustained-failure thresholds. Publish every valid/partially valid sample and notify FusionTask.

- [ ] **Step 5: Add architecture assertions and build**

Extend `tests/test_runtime_architecture.py` to assert `sensor_fusion.c` contains none of `imu_read_`, `imu_recover_`, `imu_reinit_`, `g_i2c_mutex`, or `g_inference_active`.

Run sample, sensor-schedule, and architecture tests; then run the P4 build.

- [ ] **Step 6: Commit acquisition/fusion separation**

```bash
git add main/imu_sample.h main/sample_snapshot.h main/sample_snapshot.c main/sensor_fusion.h main/sensor_fusion.c main/sensor_task.h main/sensor_task.c main/CMakeLists.txt tests/test_sample_snapshot.c tests/test_sample_snapshot.py tests/test_runtime_architecture.py
git commit -m "refactor(sensors): separate IMU acquisition from fusion"
```

### Task 8: Make SensorBus the sole I2C owner and move ToF processing to Core 1

**Files:**
- Modify: `main/sensor_task.h`
- Modify: `main/sensor_task.c:45-210,230-395`
- Modify: `main/main.c:56,120-121,294-315`
- Modify: `main/include/common.h:1-15`
- Modify: `main/drivers/imu_driver.h`
- Modify: `main/drivers/imu_driver.c`
- Modify: `tests/test_runtime_architecture.py`

**Interfaces:**
- Produces: `task_sensor_bus(void *tof_devices)`, `task_tof_processor(void *)`, and the existing ToF cache reader used by `task_sensor_snapshot`.
- Consumes: Task 6 policy, Task 7 IMU snapshot, two fixed ToF result buffers per sensor, and Task 1 task IDs.

- [ ] **Step 1: Add failing sole-owner assertions**

Assert:

```python
assert "g_i2c_mutex" not in all_application_sources
assert only_function_containing("tof_read_grid(") == "task_sensor_bus"
assert "g_inference_active" not in read("main/sensor_task.c")
assert "g_inference_active" not in read("main/sensor_fusion.c")
```

Run the architecture test and confirm it fails on current mutex references.

- [ ] **Step 2: Implement the absolute 20 ms SensorBus loop**

Create FusionTask first so its handle is available, then create SensorBus. Every SensorBus iteration:

1. Record scheduled/start time.
2. Call `sensor_read_imu_sample` and publish/notify fusion.
3. Ask `sensor_schedule_choose_tof` using the next absolute IMU deadline, per-sensor measured maximum read time, and 2 ms guard.
4. If A or B is selected, read only that device into the inactive fixed result buffer, publish its generation, and notify ToFProc.
5. Record result/skip/failure metrics.
6. Advance the absolute IMU deadline with `vTaskDelayUntil`; do not use a work-relative delay.

Seed each ToF budget at 15000 microseconds, update it with the maximum observed successful/readiness-check duration, and cap eligibility at the remaining slack. A failed sensor does not prevent the other sensor from continuing.

Replace the old task creation block in `app_main` in this task: create Fusion, SensorBus, ToFProc, and Snapshot through `runtime_task_create()` using their approved IDs. This immediately pins the new bus/fusion handoff to Core 0 and ToF processing/snapshots to Core 1; do not leave these newly split tasks unpinned until Task 9.

- [ ] **Step 3: Implement Core 1 ToF processing**

`task_tof_processor` waits for a notification, copies the stable published `VL53L5CX_ResultsData` generation, performs the existing median/target-status conversion, and calls the existing `tof_cache_store`. If a newer generation arrives while processing, count the overwritten grid and process the newest one next.

`task_sensor_snapshot` remains at 20 Hz and only reads the ToF cache, fusion result, GPS fix, detections, and motor status. Remove all inference waits from it.

- [ ] **Step 4: Remove the global mutex and update ownership comments**

Delete `g_i2c_mutex` from `main.c` and `common.h`; remove every semaphore take/give around IMU/ToF runtime calls. Change driver comments from “Call under g_i2c_mutex” to “Call only from SensorBusTask after startup.” Initialization remains serialized in `app_main` before tasks start.

- [ ] **Step 5: Run tests, ownership scan, and P4 build**

Run: `python -m pytest tests/test_sensor_schedule.py tests/test_sample_snapshot.py tests/test_runtime_architecture.py -q`

Expected: PASS and zero `g_i2c_mutex` references outside historical design documents.

Run the P4 build.

- [ ] **Step 6: Commit sole bus ownership**

```bash
git add main/sensor_task.h main/sensor_task.c main/main.c main/include/common.h main/drivers/imu_driver.h main/drivers/imu_driver.c tests/test_runtime_architecture.py
git commit -m "refactor(sensors): make SensorBus sole I2C owner"
```

### Task 9: Pin every application task and confine inference gating to Core 1 camera ownership

**Files:**
- Modify: `main/main.c:294-315`
- Modify: `main/drivers/gps_driver.h`
- Modify: `main/drivers/gps_driver.c:20-21,637-640`
- Modify: `main/detect_task.cpp:182-205`
- Modify: `main/camera_stream.c:30-90,139`
- Modify: `main/transports/ws_transport.c:18-19,338-344`
- Modify: `main/drivers/status_led.c:94-98`
- Modify: `main/sensor_task.c`
- Modify: `main/sensor_fusion.c`
- Modify: `tests/test_runtime_architecture.py`

**Interfaces:**
- Consumes: `runtime_task_create` and all schedule IDs.
- Produces: no unpinned application task creation.

- [ ] **Step 1: Add failing affinity assertions**

The source test scans `main/**/*.[ch]` and `main/**/*.cpp` and rejects raw `xTaskCreate(` calls. Permit `xTaskCreatePinnedToCore(` only inside `runtime_task.c`. It also asserts the configured table values for each task and checks that `g_inference_active` appears only in `detect_task.cpp`, `detect_task.h`, and `camera_stream.c`.

- [ ] **Step 2: Migrate each remaining creation site**

Use these exact IDs:

```c
runtime_task_create(RUNTIME_TASK_GPS, gps_task, NULL, &s_task);
runtime_task_create(RUNTIME_TASK_DETECT, detect_task_fn, NULL, &s_detect_task);
runtime_task_create(RUNTIME_TASK_CAMERA_DRAIN, camera_drain_task, NULL, &s_drain_task);
runtime_task_create(RUNTIME_TASK_WS_TX, ws_tx_task, NULL, &s_tx_task);
runtime_task_create(RUNTIME_TASK_STATUS_LED, status_led_task, NULL, &s_led_task);
```

Control and ArmSeq were migrated in Tasks 4-5; Diagnostics was migrated in Task 2; Fusion, SensorBus, ToFProc, and Snapshot were migrated in Task 8. Verify all of them against the central table in the affinity test.

- [ ] **Step 3: Enforce critical/optional startup behavior**

For Control, ArmSeq, SensorBus, and Fusion creation failure: call `motor_control_disarm()`, force `winch_driver_set_power(false)`, log the task name, and return through `ESP_ERROR_CHECK` so startup cannot continue with partial Core 0 safety services.

For GPS, Detect, CamDrain, ToFProc, Snapshot, WS_TX, Diagnostics, TrainingLog, and StatusLED: return/log the error, set that feature unavailable, and continue manual Core 0 control. Snapshot/WS failure is highly visible but does not energize hardware. TrainingLog failure never changes camera/detection operation and merely increments the optional-feature-disabled metric.

- [ ] **Step 4: Make GPS UART reception event-driven and measurable**

Install the UART with a fixed 16-entry event queue:

```c
static QueueHandle_t s_uart_events;
ESP_RETURN_ON_ERROR(
    uart_driver_install(uart_num, UART_RX_BUF, 0, 16, &s_uart_events, 0),
    TAG, "uart_driver_install");
```

Change `gps_task` from a 200 ms `uart_read_bytes` poll to `xQueueReceive(s_uart_events, &event, portMAX_DELAY)`. On `UART_DATA`, read exactly the available bytes and feed the existing byte-state parser. On `UART_FIFO_OVF` or `UART_BUFFER_FULL`, increment the corresponding fixed runtime counter, call `uart_flush_input`, reset the event queue, and reset the partial NMEA/UBX parser state. Increment parse-error and line-overflow counters at their existing reset points.

Add `gps_driver_get_runtime_status(gps_runtime_status_t *)` returning UART FIFO overflows, UART buffer-full events, parser line overflows, parse errors, current protocol authority (UBX or NMEA), last frame timestamp, and fix age. Diagnostics reads this snapshot once per second; it never enters parser internals.

Replace the current GPS fix mutex with a two-entry sequence-locked snapshot containing `gps_fix_t` plus `gps_runtime_status_t`. Each completed UBX/NMEA update copies the current stable state to the inactive entry, mutates the local entry, and atomically publishes its generation. `gps_driver_get_fix()` and `gps_driver_get_runtime_status()` retain bounded stable-copy behavior without a cross-core mutex.

- [ ] **Step 5: Remove cross-lane inference gates**

Keep `g_inference_active` only for Detect/camera buffer coordination on Core 1. Delete waits from sensor acquisition, fusion, and snapshots. Add runtime metrics around detection and camera acquisition so their load is visible without pausing Core 0.

- [ ] **Step 6: Run all host tests and build**

Run: `python -m pytest tests -q`

Expected: all existing and new host tests pass.

Run the P4 build and inspect the map/log for every application task's core and priority.

- [ ] **Step 7: Commit affinity migration**

```bash
git add main/main.c main/drivers/gps_driver.h main/drivers/gps_driver.c main/detect_task.cpp main/camera_stream.c main/transports/ws_transport.c main/drivers/status_led.c main/sensor_task.c main/sensor_fusion.c tests/test_runtime_architecture.py
git commit -m "feat(runtime): pin application tasks to control and heavy lanes"
```

### Task 10: Add an optional, bounded microSD training recorder

**Files:**
- Create: `main/training_logger.h`
- Create: `main/training_logger.c`
- Create: `tests/test_training_logger.c`
- Create: `tests/test_training_logger.py`
- Modify: `main/Kconfig.projbuild`
- Modify: `main/CMakeLists.txt`
- Modify: `main/main.c`
- Modify: `main/detect_task.cpp`
- Modify: `main/runtime_metrics.h/.c`

**Interfaces:**
- Produces: `training_logger_init()`, `training_logger_reserve()`, `training_logger_copy_jpeg()`, `training_logger_commit()`, `training_logger_cancel()`, and `training_logger_get_status()`.
- Consumes: the optional `sd_card_ready()` mount, the existing camera JPEG only while its capture buffer is valid, Task 7/8 timestamped sensor snapshots, Task 4 motor/control status, and PicoDet result metadata.
- Does not consume or call any actuator, I2C, Wi-Fi, ESP-NOW, GPS-parser, or transport send API.

- [ ] **Step 1: Write the failing bounded-pool and serialization tests**

Create a host-C test for the platform-independent pool/state-machine code. Verify that two reservations succeed, a third reservation returns immediately with `TRAINING_LOGGER_DROPPED_FULL`, `cancel` returns the slot, and only `commit` makes a completed job visible to the writer. Verify metadata is copied by value: mutate every producer-side input after commit and ensure the job retains its original capture timestamp, sensor generations/ages, manual control state, GPS validity, and detection result count. Test file-name construction rejects `/`, `..`, and overlong session IDs, and test that an image is published only after a closed `.part` file is renamed to its final `.jpg` name.

The Python wrapper compiles the pure-C pool/serialization source with `cc -std=c11 -Wall -Wextra -Werror -Imain` and runs it in a temporary directory. It must not require a card, ESP-IDF headers, camera hardware, or a serial device.

- [ ] **Step 2: Run the focused test and confirm the missing-module failure**

Run: `python -m pytest tests/test_training_logger.py -q`

Expected: FAIL because `training_logger.h` and `training_logger.c` do not exist.

- [ ] **Step 3: Define the recording contract and implement the fixed pool**

Add opt-in Kconfig values: `CONFIG_TRAINING_LOG_ENABLED` defaults to `n`, `CONFIG_TRAINING_LOG_PERIOD_MS` defaults to `1000`, `CONFIG_TRAINING_LOG_POOL_COUNT` defaults to `2`, and `CONFIG_TRAINING_LOG_MAX_JPEG_BYTES` defaults to a value no greater than the active camera JPEG buffer. Allocate exactly that many JPEG slots in PSRAM during `training_logger_init`; allocate no heap memory after init.

Use slot states `FREE`, `FILLING`, `QUEUED`, and `WRITING`. `reserve()` and `commit()` are non-blocking. `copy_jpeg()` makes the bounded PSRAM copy while the camera capture is still valid; it never writes a file. If no slot exists, the request is skipped before copying and the caller continues normal detection. `commit()` transfers a fixed-size descriptor to a fixed-length queue; if enqueue fails it releases the slot, records a queue-drop event, and returns immediately. Record accepted, pool-full, queue-full, card-unavailable, write-error, and successful-write counters through `runtime_metrics`.

Define a versioned metadata record containing at minimum: sequence and `capture_us`; width, height, pixel format and JPEG length; GPS fix plus age/generation; fusion orientation plus age/generation; ToF A/B values plus age/generation; manual throttle/rudder/winch/servo-power/failsafe state; current drive-mode identifier; and PicoDet detections/results plus their inference duration. These are observational labels only: recording never changes drive behavior.

- [ ] **Step 4: Write samples only from the Core 1 TrainingLog task**

Create `RUNTIME_TASK_TRAINING_LOG` as `TrainingLog`, `8192` stack words, priority `2`, Core `1`, event-driven, non-critical. Start it only after the existing optional SD initialization reports `sd_card_ready()`; otherwise mark the feature disabled and continue normal startup.

For each queued job, create a collision-safe session directory below `/sdcard/training/`, write `<sequence>.jpg.part`, close it, and rename it to `<sequence>.jpg`. Then write its `<sequence>.json.part`, close it, and rename it to `<sequence>.json`. A reset/power loss can therefore leave only an ignored `.part` file, never a falsely complete sample. On an open/write/close/rename error, release the slot, increment the write-error counter, and continue with the next job. The writer is the only code that opens, writes, flushes, closes, or renames training files.

- [ ] **Step 5: Connect the recorder to the Core 1 detection owner**

At the configured one-Hz initial rate, the detect task first attempts `reserve()`. If it succeeds, copy the captured JPEG into the slot before `camera_release_frame()`, run normal inference, snapshot the already-published sensor/control/GPS state, attach the inference results, and `commit()` the job. On any capture/inference error, call `cancel()`. If recording is disabled or unavailable, take no training path at all. Camera/drain/inference scheduling and all Core 0 deadlines remain unchanged; training loss is explicitly acceptable.

- [ ] **Step 6: Verify optional-failure behavior and build**

Run `python -m pytest tests/test_training_logger.py -q`, then all host tests and `idf.py reconfigure && idf.py build`. Boot once with no card and once with a deliberately unwritable/full card: confirm manual RC, ESP-NOW/Wi-Fi, telemetry, IMU/ToF/GPS, and detection continue; diagnostics shows the recorder unavailable or write errors; no control deadline-miss counter changes. With a writable card and the config enabled, confirm one recoverable JPG/JSON pair per second and that a forced reset leaves only ignorable `.part` files.

- [ ] **Step 7: Commit the training-recorder task**

```bash
git add main/training_logger.h main/training_logger.c main/Kconfig.projbuild main/CMakeLists.txt main/main.c main/detect_task.cpp main/runtime_schedule.h main/runtime_schedule.c main/runtime_metrics.h main/runtime_metrics.c tests/test_training_logger.c tests/test_training_logger.py tests/test_runtime_schedule.py
git commit -m "feat(training): add bounded microSD sample recorder"
```

### Task 11: Add the passive LP-core heartbeat observer

**Files:**
- Create: `main/lp_heartbeat_shared.h`
- Create: `main/lp_supervisor.h`
- Create: `main/lp_supervisor.c`
- Create: `main/ulp/CMakeLists.txt`
- Create: `main/ulp/main.c`
- Create: `tests/test_lp_heartbeat.c`
- Create: `tests/test_lp_heartbeat.py`
- Modify: `main/CMakeLists.txt`
- Modify: `main/motor_control.c`
- Modify: `sdkconfig.defaults`
- Modify: `tests/test_runtime_architecture.py`

**Interfaces:**
- Produces: `lp_supervisor_init`, `lp_supervisor_heartbeat(control_state, failsafe_state)`, and `lp_supervisor_get_observation`.
- Consumes: generated LP binary/shared-symbol header and the Core 0 control cadence.

- [ ] **Step 1: Write a failing pure observer-policy test**

Place the no-hardware policy helper in `lp_heartbeat_shared.h` so host and LP builds share it:

```c
lp_observer_state_t s = {0};
assert(!lp_observer_sample(&s, 1));
for (int i = 0; i < 4; ++i) assert(!lp_observer_sample(&s, 1));
assert(lp_observer_sample(&s, 1)); /* five unchanged 100 ms observations */
assert(s.missed_heartbeat_count == 1);
assert(!lp_observer_sample(&s, 2));
assert(s.unchanged_samples == 0);
```

- [ ] **Step 2: Implement and pass the host policy test**

Run: `python -m pytest tests/test_lp_heartbeat.py -q`

Expected before: FAIL. Expected after: PASS.

- [ ] **Step 3: Add the ESP-IDF LP subproject**

Append to `sdkconfig.defaults`:

```text
CONFIG_ULP_COPROC_ENABLED=y
CONFIG_ULP_COPROC_TYPE_LP_CORE=y
CONFIG_ULP_COPROC_RESERVE_MEM=4096
```

In `main/CMakeLists.txt`, add `ulp` to component requirements and:

```cmake
ulp_add_project("p4_supervisor" "${CMAKE_SOURCE_DIR}/main/ulp/")
```

In `main/ulp/CMakeLists.txt`, use the ESP-IDF LP build-system pattern:

```cmake
cmake_minimum_required(VERSION 3.16)
project(${ULP_APP_NAME})
add_executable(${ULP_APP_NAME} main.c)
ulp_apply_default_options(${ULP_APP_NAME})
ulp_apply_default_sources(${ULP_APP_NAME})
ulp_add_build_binary_targets(${ULP_APP_NAME})
target_include_directories(${ULP_APP_NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/..")
```

- [ ] **Step 4: Implement the passive LP loop**

Expose LP globals for heartbeat sequence, control state, failsafe state, observations, and missed-heartbeat count. The LP main loop samples at 100 ms:

```c
#include "ulp_lp_core_utils.h"
#include "lp_heartbeat_shared.h"

volatile uint32_t hp_heartbeat_seq;
volatile uint32_t hp_control_state;
volatile uint32_t hp_failsafe_state;
volatile uint32_t observation_count;
volatile uint32_t missed_heartbeat_count;

int main(void) {
    lp_observer_state_t observer = {0};
    for (;;) {
        ulp_lp_core_delay_us(100000);
        observation_count++;
        if (lp_observer_sample(&observer, hp_heartbeat_seq))
            missed_heartbeat_count++;
    }
}
```

Do not include any GPIO, MCPWM, driver, wake/reset, or actuator header.

- [ ] **Step 5: Load the LP binary and update heartbeat from ControlTask**

`lp_supervisor_init()` loads `_binary_p4_supervisor_bin_start/end` using `ulp_lp_core_load_binary`, then starts it with `ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU`. `ControlTask` increments the generated `ulp_hp_heartbeat_seq` once per completed control cycle and publishes numeric control/failsafe state. Diagnostics reads the LP counters once per second.

LP init failure is reported and disables only the observer; it does not block manual control.

- [ ] **Step 6: Assert passive scope and build**

Extend the architecture test to reject `gpio`, `mcpwm`, `esc_driver`, `winch_driver`, and `steer_driver` tokens in `main/ulp/main.c` and `main/lp_supervisor.c`.

Run all host tests, then `idf.py reconfigure && idf.py build`. Confirm the P4 application and `p4_supervisor` LP binary are both produced.

- [ ] **Step 7: Commit the passive observer**

```bash
git add main/lp_heartbeat_shared.h main/lp_supervisor.h main/lp_supervisor.c main/ulp/CMakeLists.txt main/ulp/main.c main/CMakeLists.txt main/motor_control.c sdkconfig.defaults sdkconfig tests/test_lp_heartbeat.c tests/test_lp_heartbeat.py tests/test_runtime_architecture.py
git commit -m "feat(safety): add passive LP heartbeat observer"
```

### Task 12: Automate final log checks and run full verification

**Files:**
- Create: `tools/runtime_stress_check.py`
- Create: `tests/test_runtime_stress_check.py`
- Modify: `main/runtime_metrics.c`

**Interfaces:**
- Consumes: `RTM` lines from Task 2 and sensor/control/LP counters added by later tasks.
- Produces: exit code zero only when automated timing thresholds pass.

- [ ] **Step 1: Write failing parser tests**

Use an inline synthetic log containing Control, SensorBus, Fusion, GPS, ToF A/B, Detect, stack, and failsafe records. Assert a good log returns no failures and separate mutations detect each limit:

```python
assert evaluate(GOOD_LOG) == []
assert "control latency 20001 us > 20000 us" in evaluate(GOOD_LOG.replace("max_latency_us=19000", "max_latency_us=20001"))
assert "IMU gap 40001 us > 40000 us" in evaluate(GOOD_LOG.replace("imu_max_gap_us=39000", "imu_max_gap_us=40001"))
assert "failsafe latency outside 400000..410000 us" in evaluate(GOOD_LOG.replace("failsafe_latency_us=405000", "failsafe_latency_us=420000"))
```

- [ ] **Step 2: Implement strict key/value parsing**

Parse only lines beginning `RTM `. Require final records for:

- `control max_latency_us`, `power_off_max_us`, `winch_stop_max_us`, and `failsafe_latency_us`;
- `sensor imu_max_gap_us`, `imu_hz_milli`, `tof_a_hz_milli`, `tof_b_hz_milli`, and I2C/skip counters;
- `gps uart_overflows` and `fix_age_us` when GPS data is valid;
- `detect max_exec_us` and observed cadence, with an explicit headroom report against the future 260 ms budget but no autonomous pass/fail decision;
- every registered task's `stack_free_percent`, core, priority, and deadline misses;
- `lp observations` and missed heartbeat count, reported but not used to actuate.

Enforce 20,000 us control/power/winch limits, 400,000-410,000 us failsafe, 40,000 us IMU gap, IMU average at least 49,000 milli-Hz, each available ToF average at least 4,500 milli-Hz, GPS UART overflows zero, and stack free at least 25%.

- [ ] **Step 3: Run the complete automated test suite**

Run: `python -m pytest tests -q`

Expected: existing dashboard/ESP-NOW/Wi-Fi fallback tests and all runtime tests pass.

- [ ] **Step 4: Run clean P4 verification builds**

```bash
source /opt/esp/idf/export.sh >/dev/null
idf.py fullclean
idf.py reconfigure
idf.py build
```

Expected: successful P4 and LP-core compilation with no undefined task/ULP symbols. Record application size from the build summary and compare it with the pre-change build.

- [ ] **Step 5: Run the ten-minute hardware stress acceptance test**

```bash
: "${BOAT_P4_PORT:?Set BOAT_P4_PORT to the verified ESP32-P4 serial device}"
test -c "$BOAT_P4_PORT"
source /opt/esp/idf/export.sh >/dev/null
idf.py -p "$BOAT_P4_PORT" flash monitor | tee build/runtime-final.log
```

During the capture, continuously drive manual rudder/throttle, repeatedly move/stop the winch, toggle servo power, remove the control link once to measure failsafe, keep both ToFs and GPS active, stream the camera, retain 20 Hz snapshots, and trigger repeated approximately 57 ms PicoDet runs. Stop after at least ten minutes, then run:

```bash
python tools/runtime_stress_check.py build/runtime-final.log
```

Expected: `PASS: P4 runtime scheduling acceptance thresholds satisfied` and exit code zero. Compare `build/runtime-final.log` with `build/runtime-baseline.log`; Core 0 sensor/control gaps must improve or remain within the hard limits while Core 1 load remains measured.

- [ ] **Step 6: Commit the acceptance tooling**

```bash
git add tools/runtime_stress_check.py tests/test_runtime_stress_check.py main/runtime_metrics.c
git commit -m "test(runtime): enforce P4 scheduling thresholds"
```

- [ ] **Step 7: Run final repository checks**

Run:

```bash
git diff --check HEAD~12..HEAD
python -m pytest tests -q
source /opt/esp/idf/export.sh >/dev/null
idf.py build
git status --short
```

Expected: no whitespace errors; all tests pass; build succeeds; only the user's pre-existing untracked `.superpowers/` and `boatMainReimagine.FCStd` remain outside the implementation commits.
