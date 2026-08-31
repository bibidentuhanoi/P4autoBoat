# Stability-SAS Slice A→B→C Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put the first *real, running* stability controller on a water-jet catamaran — a capped rudder yaw-rate damper — on the safest possible footing: a clean yaw-rate signal (A), evenly-pushing motors (B), and a gated, capped feedback loop (C).

**Architecture:** Three independent slices. **A** publishes the gyro yaw-rate out of the fusion task so the control loop reads a real spin signal instead of differencing heading. **B** compensates each motor's dead-band in a pure, unit-tested mapping function so equal command ≈ equal thrust. **C** adds a pure, unit-tested inner rate-loop (`rudder = Kr·(rate_demand − yaw_rate)`, clamped) and wires it into the control task, actuating the **rudder only**, behind a Kconfig flag and inside the existing ARM + 400 ms-failsafe safety model.

**Tech Stack:** C11, ESP-IDF v5.4, FreeRTOS, MCPWM. Host unit tests compile individual `.c` files with `cc` and stubbed ESP-IDF headers (no hardware needed); firmware builds with `idf.py build`.

---

## Orientation (read first — you have zero prior context)

- **This is firmware for a ~0.5 m water-jet catamaran** (2 BLDC-driven jets, 2 rudders on ONE servo signal, GPIO32). Jets are **unidirectional** (no reverse). Full design rationale: `docs/superpowers/specs/2026-08-11-boat-stability-sas-autopilot-design.md` — read §2, §5, §6 before Task C.
- **Work in this worktree** (`.worktrees/stability-sas`, branch `feat/stability-sas`). **Activate the venv first:** `source /workspaces/BoatEspP4/.venv/bin/activate` — it exists only in the main checkout, not per-worktree. Without it, `pytest` falls back to a system Python with a mismatched protobuf runtime and ~23 tests in `test_espnow_drive.py` show spurious failures unrelated to anything in this plan. Baseline with the venv active: `python -m pytest tests/ -q` → **45 passed** (46 once Task A has landed).
- **Host-test pattern (this is how ALL logic is tested):** a `tests/test_X.py` uses `subprocess` to compile the real `main/X.c` + `tests/test_X.c` with `cc -std=c11 -Wall -Wextra -Werror`, then runs the binary (asserts exit 0). Pure modules (no ESP-IDF deps) compile alone; modules with ESP deps get stubbed headers written to a temp dir (see `tests/test_sensor_fusion.py` for the stub pattern). Study `tests/test_control_arbiter.py` (pure) and `tests/test_sensor_fusion.py` (stubbed) — they are your templates.
- **Kconfig symbols don't exist until `sdkconfig` is regenerated.** Every new `CONFIG_*` MUST get an `#ifndef CONFIG_X / #define CONFIG_X <default> / #endif` fallback block in the `.c` that reads it (pattern already in `main/motor_control.c:39-56`). This keeps the build green before `menuconfig` is run.
- **Floats in Kconfig are stored as strings** and parsed with `strtof` (e.g. `CONFIG_FUSION_YAW_ALPHA "0.98"`). Follow that convention.
- **Commit after every task.** Small commits. Run the full host suite (`python -m pytest tests/ -q`) before each commit — it must stay at 45+ passing.
- **Safety is non-negotiable in Task C:** the loop may actuate ONLY when `esc_driver_get_state() == ESC_STATE_ARMED` AND `control_link_alive()`. It touches the **rudder only**, never the ESCs. It is compile-time OFF by default (`CONFIG_STABILITY_SAS_ENABLE` default `n`).

---

## File Structure

**New files (all pure C, easy to unit-test):**
- `main/esc_map.c` + `main/esc_map.h` — pure per-motor throttle→pulse mapping with dead-band floor (Task B).
- `main/stability_control.c` + `main/stability_control.h` — pure inner rate-loop math + state (Task C).
- `tests/test_esc_map.c` + `tests/test_esc_map.py` — Task B unit tests.
- `tests/test_stability_control.c` + `tests/test_stability_control.py` — Task C unit tests.
- `tests/test_fusion_yaw_rate.c` + `tests/test_fusion_yaw_rate.py` — Task A unit test.

**Modified files:**
- `main/sensor_fusion.h` — add `yaw_rate`, `sequence` to `FusionResult` (Task A); add `captured_us` (Task C, second touch — needed for a real age-based staleness timeout, not just "did the sample counter move").
- `main/sensor_fusion.c` — publish yaw-rate through the lock-free snapshot (Task A); publish the timestamp too (Task C).
- `main/drivers/esc_driver.c` — use `esc_map` with per-motor floors (Task B).
- `main/motor_control.c` — call the SAS tick from the control cycle (Task C).
- `main/Kconfig.projbuild` — ESC floors (Task B) + SAS config (Task C).
- `main/CMakeLists.txt` — add `esc_map.c`, `stability_control.c` to `SRCS` (Tasks B, C).

---

## Task A: Publish yaw-rate from the fusion task

**Why:** the control loop must read the real gyro spin rate (mag-independent). Today `FusionResult` carries only `{pitch, roll, heading}`; the gyro rate (`gz_rate`, computed at `sensor_fusion.c:171`) is thrown away.

**Files:**
- Modify: `main/sensor_fusion.h:8-12` (struct), `main/sensor_fusion.c` (snapshot slot, init, publish, get, update)
- Create: `tests/test_fusion_yaw_rate.c`, `tests/test_fusion_yaw_rate.py`

- [ ] **Step 1: Extend the result struct**

`main/sensor_fusion.h`, replace the struct (lines 8-12):

```c
typedef struct {
    float pitch;
    float roll;
    float heading;
    float yaw_rate;   /* deg/s about vertical axis, from gyro-Z (mag-independent) */
    uint32_t sequence; /* published sample sequence; use to detect fusion stalls */
} FusionResult;
```

Add `#include <stdint.h>` at the top of `sensor_fusion.h` if not already pulled in via `common.h` (it is, but keep it explicit-safe — verify it compiles).

- [ ] **Step 2: Add the yaw-rate slot to the lock-free snapshot**

`main/sensor_fusion.c`. In `fusion_result_slot_t` (lines 24-29) add a field:

```c
typedef struct {
    atomic_uint version;
    atomic_uint pitch_bits;
    atomic_uint roll_bits;
    atomic_uint heading_bits;
    atomic_uint yaw_rate_bits;
} fusion_result_slot_t;
```

In `fusion_result_snapshot_init()` (lines 68-78), add inside the loop:

```c
        atomic_init(&slot->yaw_rate_bits, float_bits(0.0f));
```

In `fusion_publish_result()` (lines 80-90), store it alongside the others (before the release store of `version`):

```c
    atomic_store_explicit(&slot->yaw_rate_bits, float_bits(result->yaw_rate), memory_order_relaxed);
```

In `fusion_get_result()` (lines 121-125), add to the candidate and set the sequence:

```c
        FusionResult candidate = {
            .pitch = bits_float((uint32_t)atomic_load_explicit(&slot->pitch_bits, memory_order_relaxed)),
            .roll = bits_float((uint32_t)atomic_load_explicit(&slot->roll_bits, memory_order_relaxed)),
            .heading = bits_float((uint32_t)atomic_load_explicit(&slot->heading_bits, memory_order_relaxed)),
            .yaw_rate = bits_float((uint32_t)atomic_load_explicit(&slot->yaw_rate_bits, memory_order_relaxed)),
            .sequence = sequence,
        };
```

- [ ] **Step 3: Fill yaw_rate when publishing**

`main/sensor_fusion.c`, `fusion_update_sample()`. `gz_rate` is already computed at line 171. In the `fusion_publish_result(...)` call (lines 196-200), add the field:

```c
    fusion_publish_result(sample->sequence, &(FusionResult){
        .roll = roll - calib->roll_tare,
        .pitch = pitch - calib->pitch_tare,
        .heading = heading,
        .yaw_rate = gz_rate,
    });
```

(`sequence` is set on the reader side in Step 2, so it is not needed here.)

- [ ] **Step 4: Write the failing test**

Create `tests/test_fusion_yaw_rate.py` by copying the harness scaffolding **verbatim** from `tests/test_sensor_fusion.py` (the `STUB_HEADERS` dict, `HARNESS_STUBS`, and the `subprocess.run([... compile ...])` block), changing only the class name and the test `.c` filename to `test_fusion_yaw_rate.c`.

Create `tests/test_fusion_yaw_rate.c`:

```c
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common.h"
#include "imu_sample.h"
#include "sensor_fusion.h"

/* Feed one settled sample, then a second sample with a known gyro-Z; assert the
 * published yaw_rate reflects gz / 131.0 and sequence advances. */
int main(void)
{
    static CalibrationData calib = { .m_scale = {1, 1, 1} };
    fusion_init(&calib);

    imu_sample_t s = {0};
    s.accel_gyro_valid = true;
    s.az = 1000;            /* ~1g down so tilt math is finite */
    s.gz = 1310;            /* 1310 / 131.0 = 10.0 deg/s */
    s.captured_us = 1000000;
    s.sequence = 1;
    fusion_update_sample(&s);

    s.captured_us = 1020000; /* +20 ms */
    s.sequence = 2;
    fusion_update_sample(&s);

    FusionResult r;
    fusion_get_result(&r);
    assert(r.sequence == 2);
    assert(fabsf(r.yaw_rate - 10.0f) < 0.5f);
    return 0;
}
```

- [ ] **Step 5: Run the test — verify it passes**

Run: `python -m pytest tests/test_fusion_yaw_rate.py -q`
Expected: PASS. (If the compile fails on a missing `imu_sample.h` symbol, add the minimal `imu_sample.h` fields you use to the `STUB_HEADERS` copy — mirror how `tests/test_sensor_fusion.py` already stubs `imu_sample_t`.)

- [ ] **Step 6: Run the full suite + commit**

Run: `python -m pytest tests/ -q` → expect **46 passed**.
```bash
git add main/sensor_fusion.h main/sensor_fusion.c tests/test_fusion_yaw_rate.c tests/test_fusion_yaw_rate.py
git commit -m "feat(fusion): publish gyro yaw-rate + sequence in FusionResult"
```

---

## Task B: Per-motor ESC dead-band linearization

**Why:** `throttle_to_us()` maps `[0,1]→[1000,2000]µs` identically for both motors, but the left jet spins from ~5% and the right from ~20%. Equal command → unequal thrust → the boat veers. Fix: give each motor a floor so any nonzero command lands just above *its* start point. Same idea as `CONFIG_WINCH_DEADBAND_US`.

**Files:**
- Create: `main/esc_map.c`, `main/esc_map.h`, `tests/test_esc_map.c`, `tests/test_esc_map.py`
- Modify: `main/drivers/esc_driver.c`, `main/Kconfig.projbuild`, `main/CMakeLists.txt`

- [ ] **Step 1: Write the pure mapping (header)**

Create `main/esc_map.h`:

```c
#pragma once
#include <stdint.h>

/* Map a normalized throttle t (0..1) to an ESC pulse width, skipping the
 * motor's dead-band: t==0 -> min_us (off); any t>0 lands at or above
 * floor_frac of the usable span, so the motor actually starts. floor_frac is
 * that motor's measured start point as a fraction (e.g. 0.05 left, 0.20 right). */
uint32_t esc_throttle_to_us(float t, uint32_t min_us, uint32_t max_us, float floor_frac);
```

- [ ] **Step 2: Write the failing test**

Create `tests/test_esc_map.py` (copy the *pure* pattern from `tests/test_control_arbiter.py`, changing the two source paths to `main/esc_map.c` and `tests/test_esc_map.c`, and the class/binary names).

Create `tests/test_esc_map.c`:

```c
#include <assert.h>
#include "esc_map.h"

int main(void)
{
    /* Off stays off. */
    assert(esc_throttle_to_us(0.0f, 1000, 2000, 0.20f) == 1000);
    assert(esc_throttle_to_us(-0.5f, 1000, 2000, 0.20f) == 1000);

    /* Full stays full regardless of floor. */
    assert(esc_throttle_to_us(1.0f, 1000, 2000, 0.20f) == 2000);
    assert(esc_throttle_to_us(2.0f, 1000, 2000, 0.20f) == 2000);

    /* floor_frac 0 == old linear behaviour. */
    assert(esc_throttle_to_us(0.5f, 1000, 2000, 0.0f) == 1500);

    /* A tiny command jumps to just above the floor, not to ~min. */
    uint32_t tiny = esc_throttle_to_us(0.001f, 1000, 2000, 0.20f);
    assert(tiny >= 1195 && tiny <= 1205);   /* ~1000 + 0.20*1000 */

    /* Mid command with a 0.20 floor: eff = 0.20 + 0.5*0.80 = 0.60. */
    assert(esc_throttle_to_us(0.5f, 1000, 2000, 0.20f) == 1600);
    return 0;
}
```

Run: `python -m pytest tests/test_esc_map.py -q` → Expected: FAIL (no `esc_map.c` yet).

- [ ] **Step 3: Implement the pure mapping**

Create `main/esc_map.c`:

```c
#include "esc_map.h"

uint32_t esc_throttle_to_us(float t, uint32_t min_us, uint32_t max_us, float floor_frac)
{
    if (t <= 0.0f) return min_us;                 /* off / no reverse */
    if (t > 1.0f) t = 1.0f;
    if (floor_frac < 0.0f) floor_frac = 0.0f;
    if (floor_frac > 0.95f) floor_frac = 0.95f;   /* leave headroom */
    float eff = floor_frac + t * (1.0f - floor_frac);   /* (0,1] -> [floor,1] */
    return (uint32_t)((float)min_us + eff * (float)(max_us - min_us) + 0.5f);
}
```

Run: `python -m pytest tests/test_esc_map.py -q` → Expected: PASS.

- [ ] **Step 4: Add Kconfig floors**

`main/Kconfig.projbuild`, inside `menu "ESC Motor Configuration"` (after `ESC_PULSE_NEUTRAL_US`, before `endmenu` near line 162):

```
        config ESC_MIN_THR_LEFT_PCT
            int "Left ESC minimum-thrust floor (% throttle where the LEFT jet just starts)"
            default 5
            range 0 90
            help
                Any non-zero throttle command is remapped to start at this %,
                so the left jet begins pushing immediately. Measure it: jets in
                water, raise the left command until the impeller just moves.

        config ESC_MIN_THR_RIGHT_PCT
            int "Right ESC minimum-thrust floor (% throttle where the RIGHT jet just starts)"
            default 20
            range 0 90
            help
                Same as the left floor, for the right jet (typically higher).
```

- [ ] **Step 5: Wire it into the driver**

`main/drivers/esc_driver.c`:

Add near the top (after `#include "sdkconfig.h"`, ~line 11), the fallbacks + the esc_map include:

```c
#include "esc_map.h"

#ifndef CONFIG_ESC_MIN_THR_LEFT_PCT
#define CONFIG_ESC_MIN_THR_LEFT_PCT 5
#endif
#ifndef CONFIG_ESC_MIN_THR_RIGHT_PCT
#define CONFIG_ESC_MIN_THR_RIGHT_PCT 20
#endif
```

Replace the whole `throttle_to_us` function (lines 51-58) with a version that takes a floor:

```c
/* Unidirectional ESC (HW-517 arms at min): 0 = MIN (off/stop), 1 = MAX (full).
 * floor_frac skips this motor's dead-band. No reverse — t<=0 clamps to stop. */
static uint32_t throttle_to_us(float t, float floor_frac)
{
    return esc_throttle_to_us(t, CONFIG_ESC_PULSE_MIN_US, CONFIG_ESC_PULSE_MAX_US, floor_frac);
}
```

In `esc_driver_set_throttle` (line 200), pass the per-side floors:

```c
    esp_err_t err = set_pulse_us(
        throttle_to_us(left,  CONFIG_ESC_MIN_THR_LEFT_PCT  / 100.0f),
        throttle_to_us(right, CONFIG_ESC_MIN_THR_RIGHT_PCT / 100.0f));
```

(The `arm/disarm/init` paths that call `set_pulse_us(off_us, off_us)` are unchanged — off is still min.)

- [ ] **Step 6: Register the new source file**

`main/CMakeLists.txt`: add `"esc_map.c"` to the `SRCS` list. Verify by reading the file first — match the existing formatting exactly.

- [ ] **Step 7: Firmware build + full host suite + commit**

Run: `python -m pytest tests/ -q` → expect **47 passed**.
Run: `idf.py build` → expect success (this is the firmware-compile gate; if the ESP-IDF env isn't sourced, run `. $IDF_PATH/export.sh` first).
```bash
git add main/esc_map.c main/esc_map.h main/drivers/esc_driver.c main/Kconfig.projbuild main/CMakeLists.txt tests/test_esc_map.c tests/test_esc_map.py
git commit -m "feat(esc): per-motor dead-band linearization for asymmetric jets"
```

> **Hardware note (do not skip):** the `5`/`20` defaults are estimates. Confirm each floor with jets in water and adjust via `menuconfig`. This is the only bench/bucket measurement in the whole slice.

---

## Task C: Capped rudder yaw-rate damper (the algorithm)

**Why:** the first real feedback loop. Reads yaw-rate (Task A), moves the **rudder** to hold the pilot's commanded turn-rate and damp unwanted rotation. Rudder-only (safe, no thrust coupling), capped authority, gated by ARM + link-alive, compile-time off by default.

**Control law (inner rate loop) — deliberately P-only, not full PID:** `rudder_out = clamp( Kr · (steer·Rmax − yaw_rate_filtered), ±cap )`, where `yaw_rate_filtered` is a **time-aware** low-pass of the measured rate (Step 2 — not a per-call blend fraction). Centered stick → pure damping (opposes rotation); deflected stick → drives toward the demanded yaw-rate. No derivative term — would need angular-acceleration estimation off an already-noisy rate signal. No integral term in this inner loop — rudder authority collapses at low water flow (spec §2.0), so an integrator here risks winding up while starved of authority; a bounded, conditional integral belongs in the *outer* heading-hold loop, explicitly out of scope for this slice (see "Open items"). See spec §5.2.

**Files:**
- Modify: `main/sensor_fusion.h`, `main/sensor_fusion.c` (extend `FusionResult` a second time — add `captured_us`, same additive pattern Task A already established and had reviewed), `tests/test_fusion_yaw_rate.c` (one more assertion)
- Create: `main/stability_control.c`, `main/stability_control.h`, `tests/test_stability_control.c`, `tests/test_stability_control.py`
- Modify: `main/motor_control.c`, `main/Kconfig.projbuild`, `main/CMakeLists.txt`

- [ ] **Step 1: Give FusionResult a real timestamp**

The staleness check in Step 6 needs to know *how long ago* fusion last actually ran, not just "did the sample counter move since last tick" — a fusion task that silently hangs (e.g. I2C contention, which has happened on this project before) while the control link stays alive must not leave the rudder frozen at its last command forever.

`main/sensor_fusion.h`, extend the struct again:

```c
typedef struct {
    float pitch;
    float roll;
    float heading;
    float yaw_rate;
    uint32_t sequence;
    uint64_t captured_us;  /* IMU sample timestamp this result was derived from */
} FusionResult;
```

`main/sensor_fusion.c`. The existing fields are published as 32-bit float-bit patterns in individual `atomic_uint`s because the file establishes (via `_Static_assert(ATOMIC_INT_LOCK_FREE == 2, ...)`) that 32-bit atomics are genuinely lock-free on this target — do NOT assume a 64-bit atomic is *also* lock-free without the same kind of proof. Split the timestamp into two 32-bit atomic halves instead, matching the existing pattern exactly.

In `fusion_result_slot_t`, add two more fields:

```c
typedef struct {
    atomic_uint version;
    atomic_uint pitch_bits;
    atomic_uint roll_bits;
    atomic_uint heading_bits;
    atomic_uint yaw_rate_bits;
    atomic_uint captured_us_lo;
    atomic_uint captured_us_hi;
} fusion_result_slot_t;
```

In `fusion_result_snapshot_init()`, add inside the loop:

```c
        atomic_init(&slot->captured_us_lo, 0);
        atomic_init(&slot->captured_us_hi, 0);
```

In `fusion_publish_result()`, store both halves alongside the existing relaxed stores (still before the version release store):

```c
    atomic_store_explicit(&slot->captured_us_lo,
                          (uint32_t)(result->captured_us & 0xFFFFFFFFu), memory_order_relaxed);
    atomic_store_explicit(&slot->captured_us_hi,
                          (uint32_t)(result->captured_us >> 32), memory_order_relaxed);
```

In `fusion_get_result()`, reconstruct the 64-bit value and add it to the candidate:

```c
        uint64_t captured_us =
            ((uint64_t)atomic_load_explicit(&slot->captured_us_hi, memory_order_relaxed) << 32) |
            (uint64_t)atomic_load_explicit(&slot->captured_us_lo, memory_order_relaxed);
        FusionResult candidate = {
            .pitch = bits_float((uint32_t)atomic_load_explicit(&slot->pitch_bits, memory_order_relaxed)),
            .roll = bits_float((uint32_t)atomic_load_explicit(&slot->roll_bits, memory_order_relaxed)),
            .heading = bits_float((uint32_t)atomic_load_explicit(&slot->heading_bits, memory_order_relaxed)),
            .yaw_rate = bits_float((uint32_t)atomic_load_explicit(&slot->yaw_rate_bits, memory_order_relaxed)),
            .sequence = sequence,
            .captured_us = captured_us,
        };
```

In `fusion_update_sample()`, thread the real sample timestamp through the publish call:

```c
    fusion_publish_result(sample->sequence, &(FusionResult){
        .roll = roll - calib->roll_tare,
        .pitch = pitch - calib->pitch_tare,
        .heading = heading,
        .yaw_rate = gz_rate,
        .captured_us = sample->captured_us,
    });
```

Update `tests/test_fusion_yaw_rate.c` to assert the timestamp propagates — add after the existing asserts:

```c
    assert(r.captured_us == 1020000);
```

Run: `python -m pytest tests/test_fusion_yaw_rate.py -q` → still PASS. Run the full suite → still **46 passed** (no new test file yet, so the count doesn't move).

- [ ] **Step 2: Write the pure controller header (time-aware filter)**

Create `main/stability_control.h`:

```c
#pragma once
#include <stdbool.h>

typedef struct {
    float r_max_dps;    /* stick=1.0 commands this yaw rate (deg/s) */
    float k_r;          /* rudder-norm per (deg/s) of rate error */
    float yaw_tau_s;    /* low-pass TIME CONSTANT (seconds) on measured yaw rate --
                          * NOT a per-call blend fraction, so the effective filter
                          * doesn't drift if the fusion tick rate changes under load */
    float out_cap;      /* max |rudder| the loop may command (0..1) */
} stab_cfg_t;

typedef struct {
    float yaw_filt;
    bool  initialized;
} stab_state_t;

void  stab_reset(stab_state_t *st);

/* dt_s: real elapsed seconds since the previous call for THIS state (measured
 * and clamped by the caller -- this function trusts dt_s as given; pass 0.0f
 * on the first call after a reset to snap the filter straight to the
 * measurement instead of blending from zero).
 * steer_cmd_norm: pilot input [-1,1] = yaw-rate demand.
 * yaw_rate_dps: measured yaw rate (deg/s) from FusionResult.yaw_rate.
 * Returns the rudder command [-out_cap, +out_cap]. Pure; no side effects
 * beyond updating st. */
float stab_rudder_update(stab_state_t *st, const stab_cfg_t *cfg,
                         float dt_s, float steer_cmd_norm, float yaw_rate_dps);
```

- [ ] **Step 3: Write the failing test**

Create `tests/test_stability_control.py` (copy the pure pattern from `tests/test_control_arbiter.py`; sources `main/stability_control.c` + `tests/test_stability_control.c`).

Create `tests/test_stability_control.c`:

```c
#include <assert.h>
#include <math.h>
#include "stability_control.h"

int main(void)
{
    stab_cfg_t cfg = { .r_max_dps = 45.0f, .k_r = 0.010f, .yaw_tau_s = 0.10f, .out_cap = 0.50f };
    stab_state_t st;
    stab_reset(&st);

    /* First call after reset (dt=0) snaps the filter straight to the
     * measurement -- centered stick, boat already rotating +20 deg/s ->
     * rudder opposes it immediately, no filter lag on the very first sample. */
    float u = stab_rudder_update(&st, &cfg, 0.0f, 0.0f, 20.0f);
    assert(u < 0.0f);
    assert(fabsf(u - (-0.010f * 20.0f)) < 1e-4f);   /* -0.20 */

    /* Full stick, no rotation -> drive toward demand (positive). */
    stab_reset(&st);
    u = stab_rudder_update(&st, &cfg, 0.0f, 1.0f, 0.0f);
    assert(fabsf(u - (0.010f * 45.0f)) < 1e-4f);     /* +0.45 */

    /* Output clamps to the cap. */
    stab_reset(&st);
    u = stab_rudder_update(&st, &cfg, 0.0f, 0.0f, 10000.0f);
    assert(fabsf(u + 0.50f) < 1e-6f);                /* -cap */

    /* Steer input clamps to [-1,1]. */
    stab_reset(&st);
    u = stab_rudder_update(&st, &cfg, 0.0f, 5.0f, 0.0f);
    assert(fabsf(u - (0.010f * 45.0f)) < 1e-4f);

    /* dt-awareness: with tau=0.10s, a dt=0.10s step blends the filter exactly
     * halfway (alpha = dt/(tau+dt) = 0.5) toward a new measurement -- proves
     * the filter reacts to elapsed TIME, not call count. */
    stab_reset(&st);
    stab_rudder_update(&st, &cfg, 0.0f, 0.0f, 0.0f);        /* prime filter at 0 */
    u = stab_rudder_update(&st, &cfg, 0.10f, 0.0f, 20.0f);  /* one dt=tau step toward 20 */
    float expected_filt = 0.5f * 20.0f;                     /* alpha=0.5 blend from 0 */
    assert(fabsf(u - (-0.010f * expected_filt)) < 1e-3f);

    return 0;
}
```

Run: `python -m pytest tests/test_stability_control.py -q` → Expected: FAIL.

- [ ] **Step 4: Implement the pure controller**

Create `main/stability_control.c`:

```c
#include "stability_control.h"

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

void stab_reset(stab_state_t *st)
{
    st->yaw_filt = 0.0f;
    st->initialized = false;
}

float stab_rudder_update(stab_state_t *st, const stab_cfg_t *cfg,
                         float dt_s, float steer_cmd_norm, float yaw_rate_dps)
{
    if (!st->initialized || dt_s <= 0.0f) {
        st->yaw_filt = yaw_rate_dps;   /* snap on first sample / non-positive dt */
        st->initialized = true;
    } else {
        /* Discrete first-order low-pass, dt-aware: alpha = dt/(tau+dt). Same
         * steady-state behaviour as the classic exp(-dt/tau) form without
         * needing expf() on this target, and stable for any dt_s >= 0. */
        float alpha = dt_s / (cfg->yaw_tau_s + dt_s);
        st->yaw_filt += alpha * (yaw_rate_dps - st->yaw_filt);
    }
    float r_cmd = clampf(steer_cmd_norm, -1.0f, 1.0f) * cfg->r_max_dps;
    float u = cfg->k_r * (r_cmd - st->yaw_filt);
    return clampf(u, -cfg->out_cap, cfg->out_cap);
}
```

Run: `python -m pytest tests/test_stability_control.py -q` → Expected: PASS.

- [ ] **Step 5: Add Kconfig (flag + gains + staleness timeout)**

`main/Kconfig.projbuild`, inside `menu "Manual Heading Assist"` (after `HEADING_ASSIST_MAX_DIFF`, before its `endmenu`):

```
        config STABILITY_SAS_ENABLE
            bool "Enable live rudder yaw-rate stabilizer (actuates the rudder)"
            default n
            help
                OFF = manual steering unchanged. ON = the control loop drives the
                rudder from the yaw-rate loop when ARMED and the link is alive.
                Rudder only; never the ESCs. Start OFF; flip on for tethered tests.

        config STABILITY_SAS_RMAX_DPS
            int "SAS: yaw-rate at full stick (deg/s)"
            default 45
            range 5 180

        config STABILITY_SAS_KR
            string "SAS: rudder per deg/s of rate error"
            default "0.010"

        config STABILITY_SAS_YAW_TAU_S
            string "SAS: yaw-rate low-pass time constant (seconds)"
            default "0.15"
            help
                Time constant, not a per-sample blend fraction -- the effective
                filter stays the same even if the fusion tick rate changes.

        config STABILITY_SAS_OUT_CAP
            string "SAS: max rudder authority (0..1)"
            default "0.50"

        config STABILITY_SAS_MAX_AGE_MS
            int "SAS: max fusion-data age before centring the rudder (ms)"
            default 200
            range 50 2000
            help
                If the newest FusionResult is older than this, treat fusion as
                stalled: reset the controller and centre the rudder rather than
                holding the last command forever. Independent of the 400ms
                control-link failsafe -- this catches a hung SENSOR task even
                while the control LINK stays alive.
```

- [ ] **Step 6: Wire the SAS into the control task**

`main/motor_control.c`:

(a) Add include + fallbacks near the other `#ifndef CONFIG_HEADING_ASSIST_*` block (~line 56):

```c
#include "stability_control.h"

#ifndef CONFIG_STABILITY_SAS_ENABLE
#define CONFIG_STABILITY_SAS_ENABLE 0
#endif
#ifndef CONFIG_STABILITY_SAS_RMAX_DPS
#define CONFIG_STABILITY_SAS_RMAX_DPS 45
#endif
#ifndef CONFIG_STABILITY_SAS_KR
#define CONFIG_STABILITY_SAS_KR "0.010"
#endif
#ifndef CONFIG_STABILITY_SAS_YAW_TAU_S
#define CONFIG_STABILITY_SAS_YAW_TAU_S "0.15"
#endif
#ifndef CONFIG_STABILITY_SAS_OUT_CAP
#define CONFIG_STABILITY_SAS_OUT_CAP "0.50"
#endif
#ifndef CONFIG_STABILITY_SAS_MAX_AGE_MS
#define CONFIG_STABILITY_SAS_MAX_AGE_MS 200
#endif
```

(b) Add static state near the heading-assist statics (~line 153):

```c
static stab_state_t s_stab_state;
static stab_cfg_t s_stab_cfg;
static bool s_stab_cfg_loaded = false;
static uint32_t s_stab_last_seq = 0;
static int64_t s_stab_last_tick_us = 0;
```

(c) Add the tick function just above `run_control_cycle` (~line 789). It reuses the existing `parse_cfg_float`, `clampf`, `control_link_alive`, `imu_icm_ok`, `s_manual_rudder`, `steer_driver_set`, `esc_driver_get_state`:

```c
static void stability_sas_tick(int64_t now_us)
{
#if CONFIG_STABILITY_SAS_ENABLE
    if (!s_stab_cfg_loaded) {
        s_stab_cfg_loaded = true;
        s_stab_cfg.r_max_dps = (float)CONFIG_STABILITY_SAS_RMAX_DPS;
        s_stab_cfg.k_r       = parse_cfg_float(CONFIG_STABILITY_SAS_KR,       0.010f, 0.0f, 1.0f);
        s_stab_cfg.yaw_tau_s = parse_cfg_float(CONFIG_STABILITY_SAS_YAW_TAU_S, 0.15f, 0.01f, 5.0f);
        s_stab_cfg.out_cap   = parse_cfg_float(CONFIG_STABILITY_SAS_OUT_CAP,   0.50f, 0.0f, 1.0f);
        stab_reset(&s_stab_state);
        s_stab_last_tick_us = 0;
    }

    /* Gate: only when armed, link alive, IMU healthy. Otherwise stay hands-off
     * and reset so we never resume on stale state. */
    if (esc_driver_get_state() != ESC_STATE_ARMED ||
        !control_link_alive() || !imu_icm_ok()) {
        stab_reset(&s_stab_state);
        s_stab_last_tick_us = 0;
        return;
    }

    FusionResult f = {0};
    fusion_get_result(&f);

    /* Bounded-age check on the real IMU timestamp -- catches a hung fusion
     * task even while the control link stays alive. sequence alone only
     * tells us "not a fresh sample this tick", not "how long has it been". */
    int64_t age_us = (f.captured_us != 0) ? (now_us - (int64_t)f.captured_us) : INT64_MAX;
    if (f.sequence == 0 || age_us > (int64_t)CONFIG_STABILITY_SAS_MAX_AGE_MS * 1000) {
        stab_reset(&s_stab_state);
        s_stab_last_tick_us = 0;
        steer_driver_set(0.0f);   /* fusion is gone -- centre, don't hold a stale command */
        return;
    }
    if (f.sequence == s_stab_last_seq) {
        return;   /* same sample as last tick -- hold the last command */
    }
    s_stab_last_seq = f.sequence;

    float dt_s = (s_stab_last_tick_us == 0) ? 0.0f
               : clampf((float)(now_us - s_stab_last_tick_us) / 1000000.0f, 0.0f, 0.5f);
    s_stab_last_tick_us = now_us;

    float rudder = stab_rudder_update(&s_stab_state, &s_stab_cfg, dt_s, s_manual_rudder, f.yaw_rate);
    steer_driver_set(rudder);
#endif
}
```

(d) Call it once per control cycle, passing the cycle's own timestamp. In `run_control_cycle` (~line 806), immediately after `control_apply_decision(&decision);`:

```c
    control_apply_decision(&decision);
    stability_sas_tick(start_us);
```

- [ ] **Step 7: Register the source + build (both configurations)**

`main/CMakeLists.txt`: add `"stability_control.c"` to `SRCS`.
Run: `python -m pytest tests/ -q` → expect **48 passed**.
Run: `idf.py build` → success with the default `CONFIG_STABILITY_SAS_ENABLE=n` (SAS compiles out to nothing; behavior identical to `main`).
Run: `idf.py menuconfig` → *Manual Heading Assist* → turn ON `STABILITY_SAS_ENABLE` → `idf.py build` again → must ALSO succeed. This is the only check that the live `#if CONFIG_STABILITY_SAS_ENABLE` branch actually compiles before it ever reaches hardware — nothing in the host test suite exercises it, since host tests deliberately stay off ESP-IDF/FreeRTOS/driver dependencies (`motor_control.c` pulls in all of them, which is why it isn't host-tested as a whole anywhere in this codebase — consistent with the existing test architecture, not a gap unique to this task). Leave it enabled for Step 9.

- [ ] **Step 8: Commit**

```bash
git add main/sensor_fusion.h main/sensor_fusion.c tests/test_fusion_yaw_rate.c \
        main/stability_control.c main/stability_control.h main/motor_control.c \
        main/Kconfig.projbuild main/CMakeLists.txt \
        tests/test_stability_control.c tests/test_stability_control.py
git commit -m "feat(control): gated, capped rudder yaw-rate stabilizer (SAS inner loop)"
```

- [ ] **Step 9: Bench dry test (no water, no props)**

With `STABILITY_SAS_ENABLE` still on from Step 7: `idf.py build flash monitor`.
Arm the boat (bench/force-arm), power the servo rail, hold it in your hands and **twist it** left/right. Expected: the rudder swings to *oppose* the twist. Wrong direction → flip the sign by negating `f.yaw_rate` in the tick (or set `YAW_GYRO_SIGN` — but do it in the tick to keep fusion heading untouched) and re-test. Jets never spin (SAS never touches the ESCs). Disarm → rudder goes quiet. Also confirm the staleness path: with SAS armed and running, if the IMU/fusion path ever hiccups, confirm the rudder centres rather than freezing, within `STABILITY_SAS_MAX_AGE_MS`.

- [ ] **Step 10: Tethered water test (the real measurement)**

Leash the boat, hard-kill in hand. Arm, drive. Expected: holds a straight line when you center the stick; turns smoothly when you steer; no fishtailing/oscillation. Watch the `CTRL_*` log. If it oscillates, lower `STABILITY_SAS_KR`; if sluggish, raise it. If it's twitchy/noisy, raise `STABILITY_SAS_YAW_TAU_S` (more smoothing); if it feels laggy, lower it. **Test at more than one throttle level** — rudder authority in a jet's wash changes with jet flow, so gains that feel right at low throttle may be wrong at speed; note whether behavior holds across the range you test. Record the values that work.

---

## Global verification

- Host suite green after every task: `python -m pytest tests/ -q` (45 → 46 → 47 → 48 passing; Task C's Step 1 revises `sensor_fusion.c` again but adds no new test file, so it doesn't move the count on its own).
- Firmware compiles: `idf.py build` after Tasks B and C, **and again after Task C with `CONFIG_STABILITY_SAS_ENABLE=y`** (Task C Step 7) — the only check that the live branch compiles at all.
- Default build behavior is **unchanged** (SAS OFF): manual driving identical to `main`.

## Open items / do not assume

- **Rudder location (spec §2.0):** confirm by eye whether the rudders sit in the jet efflux or the hull wake. It does not affect A/B/C, but it decides the *next* slice (speed- vs throttle-scheduled allocation, differential-thrust assist) — do not build allocation until this is known.
- **Gains are seeds, not truth.** `Rmax/Kr/yaw_tau_s/out_cap` and the two ESC floors are tuned on the water from the log, not on the bench. `max_age_ms` is a safety bound, not a feel gain — it can stay at its default; only widen it if legitimate fusion jitter is triggering false centring, and only after checking why fusion is jittering in the first place.
- **Heading-hold (outer loop) is NOT in this slice.** It requires characterizing mag-heading-vs-throttle first (spec §2.2). Do not add it here.

## Self-review (done at write time)

- **Spec coverage:** A = spec §2.2 + §9-Phase-0 (publish yaw-rate); B = spec §5.5 (dead-band linearization); C = spec §5.2 inner loop + §9 slice C. Heading-hold/allocation deliberately deferred (spec §9 "after"). ✔
- **Placeholder scan:** none — all code blocks are complete; the only "measure on hardware" items are explicitly steps 8-9 and the floors, which is correct for firmware. ✔
- **Type consistency:** `FusionResult.yaw_rate`/`.sequence`/`.captured_us` are consumed in C step 6c; `stab_cfg_t`/`stab_state_t`/`stab_rudder_update`/`stab_reset` names and the `dt_s` parameter match between header, test, and integration; `esc_throttle_to_us` signature matches between header, test, and driver. ✔

**Revision (2026-08-12), after external review:** Task C originally checked staleness via `sequence`-changed only (no bounded time-based timeout) and used a fixed per-call low-pass fraction (`yaw_lpf`) instead of a time-constant. Both were real gaps against this doc's own §6 principles ("measured, clamped dt everywhere"; "stale fusion → hold, then centre") — verified against the actual plan text and the seqlock's existing atomicity guarantees before fixing, not applied blindly. Fixed by: adding `captured_us` to `FusionResult` (Task C Step 1, split into two 32-bit atomics — a 64-bit atomic is NOT assumed lock-free on this target, matching the file's own existing `_Static_assert` discipline) and a bounded `STABILITY_SAS_MAX_AGE_MS` timeout that centres the rudder on stale data; and replacing `yaw_lpf` with a `dt`-aware `yaw_tau_s` time-constant filter (`alpha = dt/(tau+dt)`) fed by the control cycle's own timestamp. Scoped down one suggestion (a full host-integration test of the wired `CONFIG_STABILITY_SAS_ENABLE` branch in `motor_control.c`) to an explicit dual-configuration `idf.py build` gate instead, since no existing test in this codebase host-compiles `motor_control.c` as a whole (it's saturated with ESP-IDF/FreeRTOS/driver deps) — building both configurations is the proportionate check, not a novel test-architecture addition.
