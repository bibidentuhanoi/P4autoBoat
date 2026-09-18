# Yaw-Rate PI and Heading-Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the limited motor P-assist with a cascaded heading and yaw-rate PI controller that reacts during a 3-second BASE test, rejects persistent mismatch during long runs, respects motor headroom, and records its internal state.

**Architecture:** A pure `yaw_heading_control` module owns filters, heading capture, PI state, anti-windup, and dynamic authority. `motor_control.c` supplies fresh fusion and operator state, freezes the old slow learner while the new controller is active, and mixes the returned effective `c`. Existing bench recording and additive protobuf fields expose the control state to the SD card and Python Lake-ID recorder.

**Tech Stack:** ESP-IDF C11, nanopb/protobuf, Python 3, pytest, host-compiled C assertion tests.

**Spec:** `docs/superpowers/specs/2026-09-18-yaw-rate-pi-heading-hold-design.md`

## Global Constraints

- Positive gyro yaw and increasing heading mean left; negative means right.
- Increasing `c` gives the right motor more command and the left motor less.
- Initial gains are gyro tau `0.15 s`, rate Kp `0.050`, rate Ki `0.020`, heading-error tau `0.35 s`, heading Kp `0.80`, and heading yaw target cap `8.0 deg/s`.
- The controller is inactive below throttle `0.15` and during steering magnitude above `0.02`.
- Manual heading capture delay is `500 ms`; BASE captures on its first valid driving sample.
- Both ESC minimum drive thresholds remain five percent/1050 us.
- The old 10 deg/s P gate and fixed `0.075` correction cap must not exist in the new control path.
- The slow trim learner is frozen while the new controller is active; learned `c` remains feed-forward.
- No dynamic controller state is persisted.
- Motor commands remain in `[0, 1]` and preserve common throttle whenever the headroom equation permits.
- Existing protobuf tag meanings remain unchanged; diagnostics use new tags.
- `BASE TEST 3s` stays 3 seconds and laptop `BASE TEST 30s` stays 30 seconds.

---

### Task 1: Pure yaw-rate PI and heading controller

**Files:**
- Create: `main/yaw_heading_control.h`
- Create: `main/yaw_heading_control.c`
- Create: `tests/test_yaw_heading_control.c`
- Create: `tests/test_yaw_heading_control.py`
- Modify: `main/CMakeLists.txt`

**Interfaces:**
- Consumes: finite heading/yaw measurements, freshness/activation flags, steering state, common throttle, learned feed-forward `c`, timestamp delta.
- Produces: `yaw_heading_output_t yaw_heading_control_update(yaw_heading_control_t *, const yaw_heading_cfg_t *, const yaw_heading_input_t *)` and reset/init helpers.
- `yaw_heading_output_t` exposes active flags, target/error/filter values, P, I, dynamic correction, effective `c`, authority limit, and saturation.

- [ ] **Step 1: Write the failing public-behaviour test**

Create a host C test with the shipped configuration and helpers that run one
50 Hz sample at a time. It must assert these concrete cases:

```c
/* target zero, right yaw: increase c and keep correcting above 10 deg/s */
out = tick(&ctl, -20.0f, 100.0f, true, 0.40f, 0.21f, false);
assert(out.active);
assert(out.p_term > 0.0f);
assert(out.dynamic_c > 0.0f);

/* opposite yaw produces the opposite sign */
reset_active(&ctl, 100.0f);
out = tick(&ctl, +20.0f, 100.0f, true, 0.40f, 0.21f, false);
assert(out.dynamic_c < 0.0f);

/* T40 permits full symmetric c authority and never exceeds motor bounds */
assert(closef(out.c_limit, 1.0f, 1e-6f));
assert(out.effective_c >= -1.0f && out.effective_c <= 1.0f);
```

Also test 359/1-degree wrapping, integral growth under a persistent rate error,
conditional anti-windup at both bounds, integral unwind, steering freeze,
500 ms recapture, invalid-heading fallback, stale-gyro reset, low-throttle reset,
and a T80 authority limit of `0.25`.

- [ ] **Step 2: Add the pytest compiler harness and confirm RED**

Compile with strict warnings:

```python
subprocess.run([
    os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
    "-I", str(ROOT / "main"),
    str(ROOT / "main" / "yaw_heading_control.c"),
    str(ROOT / "tests" / "test_yaw_heading_control.c"),
    "-lm", "-o", str(binary),
], check=True)
subprocess.run([str(binary)], check=True)
```

Run: `python -m pytest tests/test_yaw_heading_control.py -q`

Expected: FAIL because the controller files/API do not exist.

- [ ] **Step 3: Implement the pure controller API and state machine**

Define explicit structures without ESP headers:

```c
typedef struct {
    float yaw_tau_s, rate_kp, rate_ki;
    float heading_tau_s, heading_kp, max_yaw_target_dps;
    float min_throttle, steering_deadband;
    float recapture_delay_s;
} yaw_heading_cfg_t;

typedef struct {
    float dt_s, yaw_rate_dps, heading_deg;
    float throttle, feedforward_c, steering;
    bool enabled, driving, gyro_fresh, heading_valid;
    bool base_capture_now;
} yaw_heading_input_t;
```

Implement `wrap_180`, bounded first-order filters using
`alpha = dt / (tau + dt)`, immediate gyro initialization, heading capture,
rate PI, dynamic `c` bounds, conditional integration, and output flags. Reject
non-finite inputs and `dt <= 0` by resetting and returning inactive output.
Preserve the integral only during deliberate steering; reset it for every
other inactive safety condition.

- [ ] **Step 4: Run the pure controller tests and confirm GREEN**

Run: `python -m pytest tests/test_yaw_heading_control.py -q`

Expected: PASS with the C binary reporting all controller cases passed.

- [ ] **Step 5: Register the source and commit**

Add `"yaw_heading_control.c"` beside the existing trim modules in
`main/CMakeLists.txt`.

```bash
git add main/yaw_heading_control.[ch] main/CMakeLists.txt \
  tests/test_yaw_heading_control.[cy]
git commit -m "feat: add yaw rate PI heading controller"
```

### Task 2: Measured-plant simulations and replay regression

**Files:**
- Modify: `tests/test_yaw_heading_control.c`
- Create: `tests/test_yaw_heading_replay.py`

**Interfaces:**
- Consumes: the Task 1 pure controller; measured plant gain `16.1`, time constant `1.10 s`, delay `0.30 s`; optional recorded `dataout/THEPTESTV3/T40_B_13.CSV`.
- Produces: deterministic 3-second and 30-second acceptance evidence.

- [ ] **Step 1: Add failing closed-loop simulation assertions**

Use a 50 Hz first-order plant with a 15-sample delay queue:

```c
yaw_dot = (16.1f * applied_dynamic_c + disturbance_dps - yaw) / 1.10f;
yaw += yaw_dot * 0.02f;
heading = wrap_360(heading + yaw * 0.02f);
```

For the 3-second case, apply a constant `-10 deg/s` equivalent disturbance and
compare controller-on with feed-forward-only. Require correction by sample 2,
continuous activity above 10 deg/s, and smaller absolute final yaw.

For the 30-second case, require final-10-second mean absolute yaw below
`0.5 deg/s`, heading error moving toward zero, and no sustained saturation in
the final 10 seconds.

- [ ] **Step 2: Run and inspect RED before relaxing nothing**

Run: `python -m pytest tests/test_yaw_heading_control.py -q`

Expected: FAIL until Task 1 dynamics meet the spec. If the specified gains fail,
fix the algorithmic defect first; any gain change requires updating the spec,
Kconfig defaults, tests, and this plan together.

- [ ] **Step 3: Add the recorded high-yaw replay regression**

Parse the comment/header variants of `T40_B_13.CSV`, feed each yaw sample into
the controller at its recorded cadence, and assert every active sample with
`yaw < -10` has positive dynamic correction. Skip only when the dataset is not
present, using an explicit pytest skip reason.

- [ ] **Step 4: Run simulations and replay GREEN, then commit**

Run:

```bash
python -m pytest tests/test_yaw_heading_control.py \
  tests/test_yaw_heading_replay.py -q
```

```bash
git add tests/test_yaw_heading_control.c tests/test_yaw_heading_replay.py
git commit -m "test: verify yaw controller against boat dynamics"
```

### Task 3: Firmware integration, Kconfig, and heading validity

**Files:**
- Modify: `main/Kconfig.projbuild`
- Modify: `sdkconfig`
- Modify: `main/sensor_fusion.h`
- Modify: `main/sensor_fusion.c`
- Modify: `main/motor_control.c`
- Modify: `tests/test_runtime_architecture.py`
- Modify: `tests/test_normal_driving_learns.c`
- Modify: `tests/test_normal_driving_learns.py`

**Interfaces:**
- Consumes: Task 1 controller; `FusionResult`; existing learned `c`, runtime motor-assist switch, command decision, bench ownership, and ESC mixer.
- Produces: one authoritative `effective_c` for the motor mixer and controller diagnostics snapshot for telemetry/logging.

- [ ] **Step 1: Write failing source-integration tests**

Pin these properties in `test_runtime_architecture.py`:

```python
assert 'yaw_heading_control_update(&s_yaw_heading' in tick
assert 'fabsf(f.yaw_rate) <=' not in tick
assert 'trim_learn_update' not in active_controller_branch
assert 'effective_c = s_yaw_heading_out.effective_c' in apply_body
assert 's_p_moved = true' in fresh_fusion_branch
```

Add checks that BASE sets `base_capture_now`, LEFT/RIGHT and calibration reset
the controller, steering suspends its output, and stale fusion resets dynamic
state. Extend the ordinary-driving C mirror to prove learned `c` remains the
feed-forward value while PI changes applied motor split.

- [ ] **Step 2: Run focused tests and confirm RED**

Run:

```bash
python -m pytest tests/test_runtime_architecture.py \
  tests/test_normal_driving_learns.py -q
```

Expected: FAIL because firmware still calls `trim_assist` directly.

- [ ] **Step 3: Add exact Kconfig defaults and fusion validity**

Replace the operational P-assist Kconfig entries with controller entries for
the seven gains/limits in the spec. Keep compatibility aliases only where old
tests or saved sdkconfig require them; operational code reads one canonical
value for each setting. Update `sdkconfig` to the exact shipped values.

Add `bool heading_valid` to `FusionResult` and its atomic publication slot.
Set it once yaw is initialized. Preserve validity through brief rejected mag
samples while gyro propagation remains healthy, and clear it on fusion reset or
the existing unusable sensor-health condition.

- [ ] **Step 4: Wire the controller into `motor_control.c`**

Initialize config once from Kconfig with bounded parsing. On each new fusion
sequence, create `yaw_heading_input_t`, update the controller, and mark motor
output dirty when its correction changes. Use existing common throttle and
learned `c`; replace the old `clamp(learned_c + p_corr, c_min, c_max)` with the
controller's dynamically bounded `effective_c`.

Freeze `trim_learn_update` while controller output is active. Keep the runtime
switch and wire field compatible, but update comments/log labels to say motor
yaw controller rather than P-only assist. Ensure assist-disabled behaviour is
bit-for-bit the existing learner/mixer path.

- [ ] **Step 5: Run integration tests GREEN and commit**

Run:

```bash
python -m pytest tests/test_runtime_architecture.py \
  tests/test_normal_driving_learns.py \
  tests/test_trim_learn.py tests/test_trim_assist.py \
  tests/test_trim_paths_endtoend.py -q
```

```bash
git add main/Kconfig.projbuild sdkconfig main/sensor_fusion.[ch] \
  main/motor_control.c tests/test_runtime_architecture.py \
  tests/test_normal_driving_learns.[cy]
git commit -m "feat: apply yaw PI heading hold to motors"
```

### Task 4: Firmware BASE recording and bench diagnostics

**Files:**
- Modify: `main/bench_run.h`
- Modify: `main/bench_run.c`
- Modify: `main/motor_control.c`
- Modify: `tests/test_bench_run.c`
- Modify: `tests/test_bench_run.py`
- Modify: `tests/test_bench_recording.py`

**Interfaces:**
- Consumes: controller output snapshot from Task 3.
- Produces: complete per-sample controller diagnostics in firmware SD BASE CSV.

- [ ] **Step 1: Write failing bench sample and CSV tests**

Replace the P-only bench assist fixture with values for heading, heading target,
heading error, yaw target, filtered yaw, rate error, P, I, dynamic correction,
effective `c`, authority limit, active flags, and saturation. Assert
`bench_step()` copies each field without changing motor ownership.

Pin the CSV header and a formatted row in `test_bench_recording.py`. Retain the
existing first columns and append the new diagnostic columns so old analysis
can still identify `t_s`, `phase`, `yaw_dps`, `left`, and `right`.

- [ ] **Step 2: Run and confirm RED**

Run:

```bash
python -m pytest tests/test_bench_run.py tests/test_bench_recording.py -q
```

- [ ] **Step 3: Extend bench structures and writer**

Rename P-only internal structures to controller diagnostics, update buffer-size
accounting and overflow assertions, and append these exact CSV columns:

```text
heading_deg,heading_target_deg,heading_error_deg,yaw_target_dps,
yaw_filt_dps,rate_error_dps,p_term,i_term,dynamic_c,effective_c,
c_limit,ctrl_active,heading_hold,saturated
```

Do not change `BASE`, `BASE_LONG`, LEFT, RIGHT, baseline, drive, or coast timing.

- [ ] **Step 4: Run GREEN and commit**

```bash
python -m pytest tests/test_bench_run.py tests/test_bench_recording.py -q
git add main/bench_run.[ch] main/motor_control.c \
  tests/test_bench_run.[cy] tests/test_bench_recording.py
git commit -m "feat: record yaw controller bench diagnostics"
```

### Task 5: Additive telemetry and generated protobuf bindings

**Files:**
- Modify: `main/proto/boat.proto`
- Modify: `main/proto/boat.pb.h`
- Modify: `main/proto/boat.pb.c`
- Modify: `proto/boat_pb2.py`
- Modify: `main/dashboard.html`
- Modify: `main/motor_control.c`
- Modify: `tools/espnow_drive.py`
- Modify: `tests/test_runtime_architecture.py`
- Modify: `tests/test_espnow_drive.py`
- Modify: `tests/test_bench_recording.py`

**Interfaces:**
- Consumes: Task 3 controller output snapshot and Task 4 bench status flow.
- Produces: additive `BenchStatus` fields decoded by Python without altering existing tags.

- [ ] **Step 1: Write failing schema and round-trip tests**

Append fields after tag 8:

```proto
float heading_target_deg = 9;
float heading_error_deg  = 10;
float yaw_target_dps     = 11;
float p_term             = 12;
float i_term             = 13;
float dynamic_c          = 14;
float effective_c        = 15;
float c_limit            = 16;
bool  ctrl_active        = 17;
bool  heading_hold       = 18;
bool  saturated          = 19;
```

Before editing the schema, tests should assert these fields round-trip through
Python protobuf and that serialized `BoatMessage` remains within the transport
payload limit. Pin unique new tags and unchanged tags 1-8.

- [ ] **Step 2: Run schema tests and confirm RED**

Run:

```bash
python -m pytest tests/test_runtime_architecture.py \
  tests/test_espnow_drive.py tests/test_bench_recording.py -q
```

- [ ] **Step 3: Extend schema and regenerate all mirrors**

Edit `boat.proto`, run `tools/gen_proto.sh` for Python, run the repository's
installed nanopb generator against `main/proto/boat.proto` and
`main/proto/boat.options` for C, then run:

```bash
python tools/gen_dashboard_schema.py
```

Inspect the generated diff to prove existing field tags and union tags did not
change. Update any compile-time encoded-size assertion to the generated
`boat_BenchStatus_size` and `boat_BoatMessage_size` values.

- [ ] **Step 4: Publish and decode diagnostics**

Populate new `BenchStatus` fields from the stable controller snapshot in
`motor_control.c`. Extend `_blank_bench_status`, `_handle_bench_status`, and the
Python locked snapshot with the same names. Existing firmware that omits the
new fields must decode as zero/false without errors.

- [ ] **Step 5: Run GREEN and commit**

```bash
python -m pytest tests/test_runtime_architecture.py \
  tests/test_espnow_drive.py tests/test_bench_recording.py \
  tests/test_stop_main_compat.py -q
git add main/proto/boat.proto main/proto/boat.pb.[ch] proto/boat_pb2.py \
  main/dashboard.html main/motor_control.c tools/espnow_drive.py \
  tests/test_runtime_architecture.py tests/test_espnow_drive.py \
  tests/test_bench_recording.py
git commit -m "feat: stream yaw controller diagnostics"
```

### Task 6: Lake-ID CSV columns and summaries

**Files:**
- Modify: `tools/espnow_drive.py`
- Modify: `tests/test_bench_recording.py`
- Modify: `tests/test_espnow_drive.py`
- Modify: `tests/test_lake_id.py`
- Modify: `tests/test_straight_run.py`

**Interfaces:**
- Consumes: Task 5 decoded bench diagnostics plus existing telemetry heading and applied motor commands.
- Produces: self-describing 3-second/30-second laptop CSVs and controller summary metrics.

- [ ] **Step 1: Write failing recorder and summary tests**

Assert CSV rows include:

```text
heading_target_deg,heading_error_deg,yaw_target_dps,p_term,i_term,
dynamic_c,effective_c,c_limit,ctrl_active,heading_hold,saturated
```

Feed a deterministic sequence and assert summary values for net heading change,
mean/peak yaw, active fraction, saturated fraction, peak absolute P/I, and final
I. Assert legacy zero/default messages produce valid blank-compatible rows.

Pin durations: `straight` remains 3 seconds and `straight30` remains 30 seconds.

- [ ] **Step 2: Run and confirm RED**

```bash
python -m pytest tests/test_bench_recording.py tests/test_espnow_drive.py \
  tests/test_lake_id.py tests/test_straight_run.py -q
```

- [ ] **Step 3: Implement columns and summaries**

Take all controller fields under the existing `BoatLink` lock, write them from
one coherent snapshot, and add comment lines defining sign and units. Calculate
summary statistics only from drive-phase rows. Use `None`/`nan` for metrics that
have no samples rather than manufacturing zero performance.

- [ ] **Step 4: Run GREEN and commit**

```bash
python -m pytest tests/test_bench_recording.py tests/test_espnow_drive.py \
  tests/test_lake_id.py tests/test_straight_run.py -q
git add tools/espnow_drive.py tests/test_bench_recording.py \
  tests/test_espnow_drive.py tests/test_lake_id.py tests/test_straight_run.py
git commit -m "feat: summarize Lake ID yaw controller runs"
```

### Task 7: Full verification and flash-ready artifact

**Files:**
- Verify all changed files.
- Update: `docs/superpowers/plans/2026-09-18-yaw-rate-pi-heading-hold.md` checkboxes only after their evidence exists.

**Interfaces:**
- Consumes: all prior tasks.
- Produces: reproducible host-test, protocol, firmware-build, and artifact-hash evidence.

- [ ] **Step 1: Inspect diff and run format/static guards**

```bash
git diff --check
git status --short
```

Confirm no unrelated file was overwritten and both five-percent ESC thresholds
remain configured.

- [ ] **Step 2: Run all controller and Lake-ID focused tests**

```bash
python -m pytest \
  tests/test_yaw_heading_control.py tests/test_yaw_heading_replay.py \
  tests/test_normal_driving_learns.py tests/test_trim_learn.py \
  tests/test_trim_assist.py tests/test_trim_paths_endtoend.py \
  tests/test_bench_run.py tests/test_bench_recording.py \
  tests/test_runtime_architecture.py tests/test_espnow_drive.py \
  tests/test_lake_id.py tests/test_straight_run.py \
  tests/test_stop_main_compat.py -q
```

- [ ] **Step 3: Run the complete host suite**

Run: `python -m pytest tests -q`

Expected: all tests pass with no unexpected skips or warnings.

- [ ] **Step 4: Build firmware from the Lake-ID worktree**

Run under the repository ESP-IDF environment:

```bash
idf.py build
```

Expected: exit 0 and `build/BoatEspP4.bin` produced.

- [ ] **Step 5: Record artifact identity and final source state**

```bash
sha256sum build/BoatEspP4.bin
git status --short
git log -7 --oneline
```

Report the exact binary hash, focused/full test counts, build exit status, and
remaining pre-existing worktree changes. Do not claim lake performance until
physical 3-second and 30-second tests are recorded.
