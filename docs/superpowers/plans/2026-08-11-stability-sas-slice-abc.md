# Stability-SAS Slice A→B→C Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put the first *real, running* stability controller on a water-jet catamaran — a capped rudder yaw-rate damper — on the safest possible footing: a clean yaw-rate signal (A), evenly-pushing motors (B), and a gated, capped feedback loop (C).

**Architecture:** Three independent slices. **A** publishes the gyro yaw-rate out of the fusion task so the control loop reads a real spin signal instead of differencing heading. **B** compensates each motor's dead-band in a pure, unit-tested mapping function so equal command ≈ equal thrust. **C** adds a pure, unit-tested inner rate-loop (`rudder = Kr·(rate_demand − yaw_rate)`, clamped) and wires it into the control task, actuating the **rudder only**, behind a Kconfig flag and inside the existing ARM + 400 ms-failsafe safety model.

**Tech Stack:** C11, ESP-IDF v5.4, FreeRTOS, MCPWM. Host unit tests compile individual `.c` files with `cc` and stubbed ESP-IDF headers (no hardware needed); firmware builds with `idf.py build`.

---

## Orientation (read first — you have zero prior context)

- **This is firmware for a ~0.5 m water-jet catamaran** (2 BLDC-driven jets, 2 rudders on ONE servo signal, GPIO32). Jets are **unidirectional** (no reverse). Full design rationale: `docs/superpowers/specs/2026-08-11-boat-stability-sas-autopilot-design.md` — read §2, §5, §6 before Task C.
- **Work in this worktree** (`.worktrees/stability-sas`, branch `feat/stability-sas`). Baseline is green: `python -m pytest tests/ -q` → **45 passed**.
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
- `main/sensor_fusion.h` — add `yaw_rate`, `sequence` to `FusionResult` (Task A).
- `main/sensor_fusion.c` — publish yaw-rate through the lock-free snapshot (Task A).
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

**Control law (inner rate loop):** `rudder_out = clamp( Kr · (steer·Rmax − yaw_rate_filtered), ±cap )`. Centered stick → pure damping (opposes rotation); deflected stick → drives toward the demanded yaw-rate. See spec §5.2.

**Files:**
- Create: `main/stability_control.c`, `main/stability_control.h`, `tests/test_stability_control.c`, `tests/test_stability_control.py`
- Modify: `main/motor_control.c`, `main/Kconfig.projbuild`, `main/CMakeLists.txt`

- [ ] **Step 1: Write the pure controller header**

Create `main/stability_control.h`:

```c
#pragma once
#include <stdbool.h>

typedef struct {
    float r_max_dps;    /* stick=1.0 commands this yaw rate (deg/s) */
    float k_r;          /* rudder-norm per (deg/s) of rate error */
    float yaw_lpf;      /* 0..1 low-pass factor on measured yaw rate */
    float out_cap;      /* max |rudder| the loop may command (0..1) */
} stab_cfg_t;

typedef struct {
    float yaw_filt;
    bool  initialized;
} stab_state_t;

void  stab_reset(stab_state_t *st);

/* steer_cmd_norm: pilot input [-1,1] = yaw-rate demand.
 * yaw_rate_dps: measured yaw rate (deg/s) from FusionResult.yaw_rate.
 * Returns the rudder command [-out_cap, +out_cap]. Pure; no side effects
 * beyond updating st. */
float stab_rudder_update(stab_state_t *st, const stab_cfg_t *cfg,
                         float steer_cmd_norm, float yaw_rate_dps);
```

- [ ] **Step 2: Write the failing test**

Create `tests/test_stability_control.py` (copy the pure pattern from `tests/test_control_arbiter.py`; sources `main/stability_control.c` + `tests/test_stability_control.c`).

Create `tests/test_stability_control.c`:

```c
#include <assert.h>
#include <math.h>
#include "stability_control.h"

int main(void)
{
    stab_cfg_t cfg = { .r_max_dps = 45.0f, .k_r = 0.010f, .yaw_lpf = 1.0f, .out_cap = 0.50f };
    stab_state_t st;
    stab_reset(&st);

    /* Centered stick, boat rotating +20 deg/s -> rudder opposes (negative). */
    float u = stab_rudder_update(&st, &cfg, 0.0f, 20.0f);
    assert(u < 0.0f);
    assert(fabsf(u - (-0.010f * 20.0f)) < 1e-4f);   /* -0.20 */

    /* Full stick, no rotation -> drive toward demand (positive). */
    stab_reset(&st);
    u = stab_rudder_update(&st, &cfg, 1.0f, 0.0f);
    assert(fabsf(u - (0.010f * 45.0f)) < 1e-4f);     /* +0.45 */

    /* Output clamps to the cap. */
    stab_reset(&st);
    u = stab_rudder_update(&st, &cfg, 0.0f, 10000.0f);
    assert(fabsf(u + 0.50f) < 1e-6f);                /* -cap */

    /* Steer input clamps to [-1,1]. */
    stab_reset(&st);
    u = stab_rudder_update(&st, &cfg, 5.0f, 0.0f);
    assert(fabsf(u - (0.010f * 45.0f)) < 1e-4f);
    return 0;
}
```

Run: `python -m pytest tests/test_stability_control.py -q` → Expected: FAIL.

- [ ] **Step 3: Implement the pure controller**

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
                         float steer_cmd_norm, float yaw_rate_dps)
{
    if (!st->initialized) {
        st->yaw_filt = yaw_rate_dps;
        st->initialized = true;
    } else {
        st->yaw_filt += cfg->yaw_lpf * (yaw_rate_dps - st->yaw_filt);
    }
    float r_cmd = clampf(steer_cmd_norm, -1.0f, 1.0f) * cfg->r_max_dps;
    float u = cfg->k_r * (r_cmd - st->yaw_filt);
    return clampf(u, -cfg->out_cap, cfg->out_cap);
}
```

Run: `python -m pytest tests/test_stability_control.py -q` → Expected: PASS.

- [ ] **Step 4: Add Kconfig (flag + gains)**

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

        config STABILITY_SAS_YAW_LPF
            string "SAS: yaw-rate low-pass factor (0..1)"
            default "0.30"

        config STABILITY_SAS_OUT_CAP
            string "SAS: max rudder authority (0..1)"
            default "0.50"
```

- [ ] **Step 5: Wire the SAS into the control task**

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
#ifndef CONFIG_STABILITY_SAS_YAW_LPF
#define CONFIG_STABILITY_SAS_YAW_LPF "0.30"
#endif
#ifndef CONFIG_STABILITY_SAS_OUT_CAP
#define CONFIG_STABILITY_SAS_OUT_CAP "0.50"
#endif
```

(b) Add static state near the heading-assist statics (~line 153):

```c
static stab_state_t s_stab_state;
static stab_cfg_t s_stab_cfg;
static bool s_stab_cfg_loaded = false;
static uint32_t s_stab_last_seq = 0;
```

(c) Add the tick function just above `run_control_cycle` (~line 789). It reuses the existing `parse_cfg_float`, `control_link_alive`, `imu_icm_ok`, `s_manual_rudder`, `steer_driver_set`, `esc_driver_get_state`:

```c
static void stability_sas_tick(void)
{
#if CONFIG_STABILITY_SAS_ENABLE
    if (!s_stab_cfg_loaded) {
        s_stab_cfg_loaded = true;
        s_stab_cfg.r_max_dps = (float)CONFIG_STABILITY_SAS_RMAX_DPS;
        s_stab_cfg.k_r    = parse_cfg_float(CONFIG_STABILITY_SAS_KR,    0.010f, 0.0f, 1.0f);
        s_stab_cfg.yaw_lpf= parse_cfg_float(CONFIG_STABILITY_SAS_YAW_LPF,0.30f, 0.01f, 1.0f);
        s_stab_cfg.out_cap= parse_cfg_float(CONFIG_STABILITY_SAS_OUT_CAP,0.50f, 0.0f, 1.0f);
        stab_reset(&s_stab_state);
    }

    /* Gate: only when armed, link alive, IMU healthy. Otherwise stay hands-off
     * and reset so we never resume on stale state. */
    if (esc_driver_get_state() != ESC_STATE_ARMED ||
        !control_link_alive() || !imu_icm_ok()) {
        stab_reset(&s_stab_state);
        return;
    }

    FusionResult f = {0};
    fusion_get_result(&f);
    if (f.sequence == 0 || f.sequence == s_stab_last_seq) {
        return;   /* no fresh fusion data this tick — hold, don't act on stale */
    }
    s_stab_last_seq = f.sequence;

    float rudder = stab_rudder_update(&s_stab_state, &s_stab_cfg, s_manual_rudder, f.yaw_rate);
    steer_driver_set(rudder);
#endif
}
```

(d) Call it once per control cycle. In `run_control_cycle` (~line 806), immediately after `control_apply_decision(&decision);`:

```c
    control_apply_decision(&decision);
    stability_sas_tick();
```

- [ ] **Step 6: Register the source + build**

`main/CMakeLists.txt`: add `"stability_control.c"` to `SRCS`.
Run: `python -m pytest tests/ -q` → expect **48 passed**.
Run: `idf.py build` → success (builds with SAS OFF by default; the tick compiles out to nothing).

- [ ] **Step 7: Commit**

```bash
git add main/stability_control.c main/stability_control.h main/motor_control.c main/Kconfig.projbuild main/CMakeLists.txt tests/test_stability_control.c tests/test_stability_control.py
git commit -m "feat(control): gated, capped rudder yaw-rate stabilizer (SAS inner loop)"
```

- [ ] **Step 8: Bench dry test (no water, no props)**

Enable it: `idf.py menuconfig` → *Manual Heading Assist* → turn ON `STABILITY_SAS_ENABLE`. `idf.py build flash monitor`.
Arm the boat (bench/force-arm), power the servo rail, hold it in your hands and **twist it** left/right. Expected: the rudder swings to *oppose* the twist. Wrong direction → flip the sign by negating `f.yaw_rate` in the tick (or set `YAW_GYRO_SIGN` — but do it in the tick to keep fusion heading untouched) and re-test. Jets never spin (SAS never touches the ESCs). Disarm → rudder goes quiet.

- [ ] **Step 9: Tethered water test (the real measurement)**

Leash the boat, hard-kill in hand. Arm, drive. Expected: holds a straight line when you center the stick; turns smoothly when you steer; no fishtailing/oscillation. Watch the `CTRL_*` log. If it oscillates, lower `STABILITY_SAS_KR`; if sluggish, raise it. If it wobbles at speed, raise `STABILITY_SAS_YAW_LPF` toward 1.0 (less smoothing) or lower it (more smoothing) per the log. Record the values that work.

---

## Global verification

- Host suite green after every task: `python -m pytest tests/ -q` (45 → 46 → 47 → 48 passing).
- Firmware compiles: `idf.py build` after Tasks B and C.
- Default build behavior is **unchanged** (SAS OFF): manual driving identical to `main`.

## Open items / do not assume

- **Rudder location (spec §2.0):** confirm by eye whether the rudders sit in the jet efflux or the hull wake. It does not affect A/B/C, but it decides the *next* slice (speed- vs throttle-scheduled allocation, differential-thrust assist) — do not build allocation until this is known.
- **Gains are seeds, not truth.** `Rmax/Kr/yaw_lpf/out_cap` and the two ESC floors are tuned on the water from the log, not on the bench.
- **Heading-hold (outer loop) is NOT in this slice.** It requires characterizing mag-heading-vs-throttle first (spec §2.2). Do not add it here.

## Self-review (done at write time)

- **Spec coverage:** A = spec §2.2 + §9-Phase-0 (publish yaw-rate); B = spec §5.5 (dead-band linearization); C = spec §5.2 inner loop + §9 slice C. Heading-hold/allocation deliberately deferred (spec §9 "after"). ✔
- **Placeholder scan:** none — all code blocks are complete; the only "measure on hardware" items are explicitly steps 8-9 and the floors, which is correct for firmware. ✔
- **Type consistency:** `FusionResult.yaw_rate`/`.sequence` (A) are consumed in C step 5c; `stab_cfg_t`/`stab_state_t`/`stab_rudder_update`/`stab_reset` names match between header, test, and integration; `esc_throttle_to_us` signature matches between header, test, and driver. ✔
