# ESC Differential-Trim Auto-Calibration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Learn per-throttle-level left/right ESC differential trim from gyro yaw-rate (no benchable RPM), store it in NVS, apply it at runtime so equal forward demand produces no unwanted yaw — the DC-bias half of "most stable boat," under the SAS rudder loop.

**Architecture:** A slow integrator finds the trim that nulls yaw with rudders centered (converged-integrator→saved-trim, the ArduPilot `SERVO_AUTO_TRIM` pattern). Pure, host-tested units do the math and the state machine; the ControlTask runs it one tick at a time; a protobuf command triggers it from either dashboard. Full rationale: `docs/superpowers/specs/2026-08-12-esc-trim-autocal-design.md` — **read it before Task 3.**

**Tech Stack:** C11, ESP-IDF v5.4, FreeRTOS, nanopb, NVS. Host unit tests compile individual `.c` with `cc` + stubbed headers; firmware builds with `idf.py build`.

---

## Orientation (read first — zero prior context assumed)

- **Worktree:** `/workspaces/BoatEspP4/.worktrees/stability-sas`, branch `feat/stability-sas`. **Activate the venv:** `source /workspaces/BoatEspP4/.venv/bin/activate` (exists only in the main checkout). Baseline: `python -m pytest tests/ -q` → **48 passed**.
- **Host-test pattern:** each `tests/test_X.py` compiles the real `main/X.c` + a `tests/test_X.c` with `cc -std=c11 -Wall -Wextra -Werror` and runs it. Pure modules compile alone (template: `tests/test_control_arbiter.py`); the state-machine template to mirror is `tests/test_arm_sequence.{c,py}` — study it, `arm_sequence.c` is the same shape as what you build in Task 3.
- **Two milestones.** Tasks 1–3 are **pure core** — they change *no runtime behavior* (the trim table starts empty → runtime trim is 0; the state machine isn't invoked yet), so Milestone 1 is safe to merge on its own. Tasks 4–6 wire it live (proto, ControlTask, UI) = Milestone 2, the on-water part.
- **ESC trim gets its OWN NVS record — it does NOT extend `CalibrationData`.** `fs_load_calibration` (`file_system.c:166`) rejects the *whole* `"imu_cal"` blob if its size doesn't exactly match `sizeof(CalibrationData)` — appending fields there would silently wipe the boat's existing, already-calibrated gyro/mag bias on the first boot after this ships, a regression unrelated to ESC trim. Task 1 instead mirrors the existing `fs_save_tof_xtalk`/`fs_load_tof_xtalk` keyed-blob pattern under a new `"esc_trim"` key. `main/include/common.h` is not touched by this plan at all.
- **Proto parity is mandatory:** any `boat.proto` change MUST be mirrored into BOTH `main/dashboard.html`'s `protoSchema` string AND `tools/espnow_drive.py`'s generated `boat_pb2.py`, in the same commit — protobuf.js silently drops unknown fields, which has caused a UI lockout before.
- **Safety (Tasks 3, 5):** the routine actuates live thrusters. It runs ONLY when ARMED + link-alive + IMU-healthy; any abort (Stop, link loss, disarm, IMU fault, manual steering, trim clamp, excessive yaw, timeout) → throttle 0, rudder centered, in-RAM table discarded. NVS write happens ONLY on a clean full sweep.
- **Kconfig fallback pattern:** every new `CONFIG_*` gets an `#ifndef CONFIG_X / #define CONFIG_X <default> / #endif` block in the `.c` that reads it (as in `motor_control.c:57`).

---

## File Structure

**New (pure, host-tested):**
- `main/esc_trim.c` + `.h` — trim-table interpolation + apply-to-L/R math (Tasks 1–2).
- `main/esc_trim_cal.c` + `.h` — the calibration state machine (Task 3), mirrors `arm_sequence`.
- `tests/test_esc_trim.{c,py}`, `tests/test_esc_trim_cal.{c,py}`.

**Modified:**
- `main/file_system.{c,h}` — new `fs_save_esc_trim`/`fs_load_esc_trim`, own NVS key, independent of `CalibrationData` (Task 1).
- `main/main.c` — load the trim blob at boot, separately from `fs_load_calibration` (Task 1).
- `main/motor_control.c` — apply trim in the drive path (Task 2); run `calibration_tick`, telemetry, NVS save (Task 5).
- `main/proto/boat.proto` (+ regenerated `boat.pb.{c,h}`, `proto/boat_pb2.py`) — `CalibrateCommand` + `CalibrateStatus` (Task 4).
- `main/pipeline.{c,h}` — register the calibrate handler (Task 4).
- `main/dashboard.html`, `tools/espnow_drive.py` — trigger + progress + protoSchema parity (Tasks 4, 6).
- `main/CMakeLists.txt` — add `esc_trim.c`, `esc_trim_cal.c`.

---

# MILESTONE 1 — pure core (safe to merge; no behavior change)

## Task 1: Trim table storage (own NVS record) + interpolation

**Files:** create `main/esc_trim.{c,h}`, `tests/test_esc_trim.{c,py}`; modify `main/file_system.{c,h}`, `main/main.c`, `main/CMakeLists.txt`. **`main/include/common.h` is NOT touched** — see Step 1.

- [ ] **Step 1: Define the types — a self-contained header, NOT an extension of `CalibrationData`**

`file_system.c`'s `fs_load_calibration` (`file_system.c:166`) rejects the *entire* `"imu_cal"` blob if its stored size doesn't exactly match `sizeof(CalibrationData)`. Appending fields to that struct would mean the first boot after this ships silently wipes the boat's existing gyro bias / mag bias / mag scale / tares back to defaults — a real regression unrelated to ESC trim. This gets its own NVS record instead, mirroring the existing `fs_save_tof_xtalk`/`fs_load_tof_xtalk` keyed-blob pattern.

`main/esc_trim.h`:

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define ESC_TRIM_MAX_POINTS 8
#define ESC_TRIM_NVS_MAGIC 0x54524931u   /* "TRI1" -- bump if the on-disk layout ever changes */

typedef struct __attribute__((packed)) {
    float throttle_frac;   /* common throttle 0..1 this point was learned at */
    float trim_diff;       /* signed differential: left = T - trim/2, right = T + trim/2 */
} EscTrimPoint;

/* The whole persisted record -- one NVS blob, one key ("esc_trim"), entirely
 * independent of CalibrationData/"imu_cal". magic_word guards against a
 * corrupted or absent blob; count==0 is always a safe "no trim" state. */
typedef struct __attribute__((packed)) {
    uint32_t magic_word;
    uint8_t  count;
    EscTrimPoint points[ESC_TRIM_MAX_POINTS];
} EscTrimNvsBlob;

/* Interpolate the learned differential trim for a given common throttle
 * (0..1). Points need not be pre-sorted. count==0 -> 0. Outside the
 * calibrated range, clamp to the nearest endpoint's trim (do NOT extrapolate
 * -- a linear extrapolation off the ends can command large differentials the
 * calibration never validated). */
float esc_trim_lookup(const EscTrimPoint *pts, uint8_t count, float common_throttle);

/* Apply the looked-up trim symmetrically about the common throttle, preserving
 * the pilot's turn differential already present in *left/*right:
 *   common = (L+R)/2 ; L -= trim/2 ; R += trim/2.
 * This is the exact "trim on common, pilot turn intentional" ordering; the
 * per-ESC floor/curve (esc_map) still runs AFTER this, in esc_driver. */
void esc_trim_apply(float *left, float *right, const EscTrimPoint *pts, uint8_t count);
```

- [ ] **Step 2: NVS load/save, mirroring `fs_save_tof_xtalk`/`fs_load_tof_xtalk`**

`main/file_system.h`: add `#include "esc_trim.h"` and declare:

```c
void fs_save_esc_trim(const EscTrimNvsBlob *blob);
bool fs_load_esc_trim(EscTrimNvsBlob *blob);   /* true = valid blob loaded; false = defaulted to empty (blob->count = 0) */
```

`main/file_system.c`, add near `fs_save_tof_xtalk`/`fs_load_tof_xtalk` (same `nvs_open("storage", ...)` namespace, different key — `"esc_trim"`):

```c
void fs_save_esc_trim(const EscTrimNvsBlob *blob)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed for esc_trim save: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(h, "esc_trim", blob, sizeof(*blob));
    if (err == ESP_OK) {
        nvs_commit(h);
        ESP_LOGI(TAG, "ESC trim table saved (%u points)", (unsigned)blob->count);
    } else {
        ESP_LOGE(TAG, "ESC trim save failed: %s", esp_err_to_name(err));
    }
    nvs_close(h);
}

bool fs_load_esc_trim(EscTrimNvsBlob *blob)
{
    blob->magic_word = ESC_TRIM_NVS_MAGIC;
    blob->count = 0;   /* the safe default if anything below fails */

    nvs_handle_t h;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &h);
    if (err != ESP_OK) return false;

    EscTrimNvsBlob temp;
    size_t len = sizeof(temp);
    err = nvs_get_blob(h, "esc_trim", &temp, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(temp) || temp.magic_word != ESC_TRIM_NVS_MAGIC) {
        ESP_LOGI(TAG, "No valid ESC trim table in NVS -- starting with none");
        return false;
    }
    *blob = temp;
    ESP_LOGI(TAG, "ESC trim table loaded (%u points)", (unsigned)blob->count);
    return true;
}
```

- [ ] **Step 3: Write the failing test (interpolation)**

`tests/test_esc_trim.py`: copy the pure pattern from `tests/test_control_arbiter.py`, sources `main/esc_trim.c` + `tests/test_esc_trim.c`, `-I main` — `esc_trim.h` is fully self-contained (`<stdint.h>`/`<stdbool.h>` only), no stub headers needed.

`tests/test_esc_trim.c`:

```c
#include <assert.h>
#include <math.h>
#include "esc_trim.h"

int main(void)
{
    EscTrimPoint pts[3] = {
        { .throttle_frac = 0.2f, .trim_diff = -0.06f },
        { .throttle_frac = 0.4f, .trim_diff = -0.10f },
        { .throttle_frac = 0.8f, .trim_diff = -0.18f },
    };

    /* Empty table -> no trim. */
    assert(esc_trim_lookup(pts, 0, 0.5f) == 0.0f);

    /* Exact at a point. */
    assert(fabsf(esc_trim_lookup(pts, 3, 0.4f) - (-0.10f)) < 1e-6f);

    /* Linear between 0.4 and 0.8: halfway (0.6) -> -0.14. */
    assert(fabsf(esc_trim_lookup(pts, 3, 0.6f) - (-0.14f)) < 1e-6f);

    /* Below the lowest / above the highest -> clamp to endpoint, no extrapolation. */
    assert(fabsf(esc_trim_lookup(pts, 3, 0.05f) - (-0.06f)) < 1e-6f);
    assert(fabsf(esc_trim_lookup(pts, 3, 0.95f) - (-0.18f)) < 1e-6f);

    /* apply: common preserved, pilot turn preserved, trim symmetric.
     * left=0.5,right=0.5 -> common 0.5 -> trim(0.5)= -0.12 -> L=0.56,R=0.44. */
    float L = 0.5f, R = 0.5f;
    esc_trim_apply(&L, &R, pts, 3);
    assert(fabsf(L - 0.56f) < 1e-6f && fabsf(R - 0.44f) < 1e-6f);

    /* A pilot turn (L!=R) rides through untouched in differential terms:
     * L=0.6,R=0.4 -> common 0.5, turn +0.1 -> trim -0.12 -> L=0.66,R=0.34. */
    L = 0.6f; R = 0.4f;
    esc_trim_apply(&L, &R, pts, 3);
    assert(fabsf(L - 0.66f) < 1e-6f && fabsf(R - 0.34f) < 1e-6f);
    return 0;
}
```

Run: `python -m pytest tests/test_esc_trim.py -q` → FAIL (no `esc_trim.c`).

- [ ] **Step 4: Implement the interpolation math**

`main/esc_trim.c`:

```c
#include "esc_trim.h"

float esc_trim_lookup(const EscTrimPoint *pts, uint8_t count, float common_throttle)
{
    if (!pts || count == 0) return 0.0f;

    /* Find the lowest point >= throttle. Points are few (<= 8) and typically
     * already ascending, but do not assume it -- scan for the bracketing pair. */
    const EscTrimPoint *lo = &pts[0];
    const EscTrimPoint *hi = &pts[0];
    for (uint8_t i = 0; i < count; ++i) {
        if (pts[i].throttle_frac <= lo->throttle_frac) lo = &pts[i];  /* global min */
        if (pts[i].throttle_frac >= hi->throttle_frac) hi = &pts[i];  /* global max */
    }
    if (common_throttle <= lo->throttle_frac) return lo->trim_diff;   /* clamp low  */
    if (common_throttle >= hi->throttle_frac) return hi->trim_diff;   /* clamp high */

    /* Bracket: largest point below, smallest point above. */
    const EscTrimPoint *a = lo, *b = hi;
    for (uint8_t i = 0; i < count; ++i) {
        float t = pts[i].throttle_frac;
        if (t <= common_throttle && t >= a->throttle_frac) a = &pts[i];
        if (t >= common_throttle && t <= b->throttle_frac) b = &pts[i];
    }
    float span = b->throttle_frac - a->throttle_frac;
    if (span <= 0.0f) return a->trim_diff;   /* duplicate throttles -> take one */
    float frac = (common_throttle - a->throttle_frac) / span;
    return a->trim_diff + frac * (b->trim_diff - a->trim_diff);
}

void esc_trim_apply(float *left, float *right, const EscTrimPoint *pts, uint8_t count)
{
    if (!left || !right) return;
    float common = 0.5f * (*left + *right);
    float trim = esc_trim_lookup(pts, count, common);
    *left  -= 0.5f * trim;
    *right += 0.5f * trim;
}
```

Run: `python -m pytest tests/test_esc_trim.py -q` → PASS.

- [ ] **Step 5: Load at boot + register + full suite + commit**

`main/main.c`, near (but NOT part of the same call as) wherever `fs_load_calibration` loads the IMU blob — add a separate, independent load:

```c
    EscTrimNvsBlob esc_trim_blob;
    fs_load_esc_trim(&esc_trim_blob);   /* false -> blob.count already 0, safe default */
```

Note: the `motor_control_set_esc_trim(...)` call is deliberately NOT added here — that function doesn't exist until Task 2. Task 1's own gate is a clean `idf.py build`, so this step stops at the load. Task 2 adds the setter call in `main.c` itself, alongside the function definition (see Task 2 Step 1 below).

`main/CMakeLists.txt`: add `"esc_trim.c"` to `SRCS`.
Run: `python -m pytest tests/ -q` → **49 passed**. `idf.py build` → exit 0.
```bash
git add main/esc_trim.c main/esc_trim.h main/file_system.c main/file_system.h main/main.c main/CMakeLists.txt tests/test_esc_trim.c tests/test_esc_trim.py
git commit -m "feat(esc): own-NVS-record trim table + interpolation (independent of CalibrationData)"
```

## Task 2: Apply the trim at runtime (control layer)

**Why:** with a table present, equal-throttle commands must come out balanced. Applied on the **common** throttle, *before* the pilot turn is meaningful and *before* `esc_map`'s floor (which stays last, in `esc_driver`).

**Files:** modify `main/motor_control.c` (holds the loaded table, applies trim in the drive path).

- [ ] **Step 1: Hold the loaded table**

`main/motor_control.c`: add a file-scope copy of the trim table, populated at init from the independently-loaded `EscTrimNvsBlob` (Task 1's `fs_load_esc_trim` — nothing to do with `CalibrationData`). For the general shape of "load once at boot, hand to a motor_control setter," look at how `fusion_init(&calib)` already receives the unrelated IMU `CalibrationData` — same *pattern* (a setter called once from `main.c` at startup), different, independent data. Add a setter called once at startup:

```c
#include "esc_trim.h"

static EscTrimPoint s_esc_trim[ESC_TRIM_MAX_POINTS];
static uint8_t s_esc_trim_count = 0;

void motor_control_set_esc_trim(const EscTrimPoint *pts, uint8_t count)
{
    s_esc_trim_count = (count > ESC_TRIM_MAX_POINTS) ? ESC_TRIM_MAX_POINTS : count;
    for (uint8_t i = 0; i < s_esc_trim_count; ++i) s_esc_trim[i] = pts[i];
}
```
Declare it in `motor_control.h`. Task 2 adds the `motor_control_set_esc_trim(...)` call in `main.c`, right after the `fs_load_esc_trim` line Task 1 added — a separate call from `fs_load_calibration`, not sharing its struct.

- [ ] **Step 2: Apply in the drive path**

In `control_apply_decision`, inside `if (decision->drive_changed) { ... }`, the drive branch currently reads (verify against the real file — this plan was written against an earlier line count and has already drifted once):
```c
            float left;
            float right;
            esc_driver_get_throttle(&left, &right);
            if (left != decision->left || right != decision->right) {
                esc_driver_set_throttle(decision->left, decision->right);
                changed = true;
            }
```
Note `left`/`right` here already name the *current* ESC state (from `esc_driver_get_throttle`), not the *target* — do not reuse those names for the trimmed target values, or you'll shadow/collide with them in the same scope. Apply trim to a *new* pair of locals holding the target, compare those against the current `left`/`right`, and pass the trimmed pair to `esc_driver_set_throttle` — but ONLY when calibration is NOT active (Task 5 sets `s_calibrating`; until Task 5 the symbol is `false`, so guard with a plain static bool defaulting false):

```c
            float left;
            float right;
            esc_driver_get_throttle(&left, &right);
            float target_left  = decision->left;
            float target_right = decision->right;
            if (!s_calibrating) {                 /* s_calibrating: static bool, default false */
                esc_trim_apply(&target_left, &target_right, s_esc_trim, s_esc_trim_count);
            }
            if (left != target_left || right != target_right) {
                esc_driver_set_throttle(target_left, target_right);
                changed = true;
            }
```
The point that matters, however you name things: `esc_trim_apply` runs on the mixed L/R target *before* `esc_driver_set_throttle`, gated off during calibration, comparing against the pre-existing "did it actually change" check rather than replacing it. Add `static bool s_calibrating = false;` near the other statics; Task 5 drives it.

- [ ] **Step 3: Build + suite + commit**

With an empty table (default) `esc_trim_apply` is a no-op, so behavior is unchanged. Run `python -m pytest tests/ -q` → still 49. `idf.py build` → exit 0.
```bash
git add main/motor_control.c main/motor_control.h main/main.c
git commit -m "feat(esc): apply per-throttle trim on common throttle before floor map"
```

## Task 3: The calibration state machine (pure) — the crux

**Read the spec's §5 (state machine) and §4 (constants) first.** This mirrors `arm_sequence.c`: a pure struct + `init` + `start` + `step`, fully host-tested with synthetic sensor sequences. It commands nothing directly — `step()` *returns* what the ESCs/rudder should be this tick and what to record; the ControlTask (Task 5) applies it.

**Files:** create `main/esc_trim_cal.{c,h}`, `tests/test_esc_trim_cal.{c,py}`; modify `main/CMakeLists.txt`.

- [ ] **Step 1: Header** — `main/esc_trim_cal.h`:

```c
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "common.h"

typedef enum {
    ETC_IDLE, ETC_MEASURE_NOISE, ETC_RAMP, ETC_SETTLE, ETC_DONE, ETC_ABORTED
} etc_state_t;

typedef struct {
    float   ki;                /* integrator gain (slow) */
    float   trim_clamp;        /* ± differential guard rail; hitting it aborts */
    float   accept_k;          /* sigma multiple for the accept band (e.g. 3) */
    float   excessive_yaw_dps; /* abort trip; own value, NOT SAS's R_MAX (spec §4/§7) */
    float   min_speed_mps;     /* making-way gate */
    uint32_t window_ticks;     /* moving-average window length N */
    uint32_t noise_ticks;      /* stationary noise/bias measurement window */
    int64_t settle_in_us;      /* wait after a throttle step before integrating */
    int64_t level_timeout_us;  /* per-level give-up backstop */
    uint8_t level_count;
    float   levels[ESC_TRIM_MAX_POINTS]; /* common-throttle fractions to visit */
    bool    average_into_existing;       /* correction #4 two-heading mode */
} etc_cfg_t;

typedef struct {
    etc_state_t state;
    uint8_t  level_idx;
    float    trim_diff;
    float    b0, sigma;        /* measured residual bias + noise std */
    /* running accumulators (noise measure + moving window), impl-private */
    double   acc_sum, acc_sumsq;
    uint32_t acc_n;
    int64_t  phase_start_us;
    EscTrimPoint out_table[ESC_TRIM_MAX_POINTS];
    uint8_t  out_count;
} etc_t;

typedef struct {
    bool  active;         /* calibration owns the actuators this tick */
    float left_cmd;       /* ESC commands for this tick (level ∓ trim/2) */
    float right_cmd;
    /* rudder is always commanded to 0.0 while active; caller enforces it */
    bool  done;           /* full sweep complete -> caller saves out_table to NVS */
    bool  aborted;
    const char *reason;   /* NULL unless aborted/done */
} etc_out_t;

void esc_trim_cal_init(etc_t *s, const etc_cfg_t *cfg);
void esc_trim_cal_start(etc_t *s, const etc_cfg_t *cfg);
void esc_trim_cal_abort(etc_t *s);   /* external stop / gate failure */

/* One control tick. Gates (armed/link/imu/manual_override) are evaluated by
 * the caller and passed in; any false-when-required aborts. yaw_rate_dps is
 * raw gyro; the routine subtracts its own measured b0. manual_override means
 * "the pilot submitted a new throttle, winch, or steer command since
 * calibration started" -- any one of the three, not steering alone (Task 5
 * computes it from a single timestamp comparison, see that task). */
etc_out_t esc_trim_cal_step(etc_t *s, const etc_cfg_t *cfg, int64_t now_us,
                            float yaw_rate_dps, float gps_speed_mps,
                            bool armed, bool link_alive, bool imu_ok,
                            bool manual_override);
```

- [ ] **Step 2: Write the behavioral tests FIRST** (they are the real spec).

`tests/test_esc_trim_cal.py`: pure pattern (`test_control_arbiter.py` shape), sources `main/esc_trim_cal.c` + `main/esc_trim.c` (for `EscTrimPoint`/`common.h`, `-I main`) + `tests/test_esc_trim_cal.c`.

`tests/test_esc_trim_cal.c` — cover, at minimum, each of these as a named block (a simple driver that steps the machine with a synthetic yaw model `yaw = plant_gain * (bias − trim_diff) + noise`, advancing `now_us`):

1. **Converges & records straight-line:** with a constant thrust bias, the integrator drives `trim_diff` until modeled yaw enters the band, holds for `window_ticks`, and emits one recorded point at the level's throttle with `trim ≈ bias/plant_gain`. Full sweep of 2 levels → `done`, `out_count == 2`.
2. **Bias subtraction:** feed a constant gyro offset with zero true motion; assert the machine measures `b0`, subtracts it, and the recorded `trim_diff ≈ 0` (does NOT trim to cancel a sensor offset).
3. **Runaway → clamp-abort:** feed a yaw whose sign *reinforces* trim (wrong-sign / unfixable). Assert `trim_diff` reaches `trim_clamp` and `step` returns `aborted` with a clamp reason, not a saturate-and-continue.
4. **Excessive-yaw abort:** inject a yaw spike > `excessive_yaw_dps`; assert immediate `aborted`.
5. **Timeout:** feed persistent noise that never settles; assert `aborted` after `level_timeout_us`.
6. **Making-way gate:** hold `gps_speed_mps < min_speed_mps`; assert the level does not record (either skipped or timeout-abort per your chosen policy — pick one and assert it).
7. **Gate aborts:** each of `armed=false`, `link_alive=false`, `imu_ok=false`, `manual_override=true` at any active tick → immediate `aborted`, `active` drops, outputs safe.

Run → FAIL (no impl).

- [ ] **Step 3: Reference implementation** — `main/esc_trim_cal.c`.

Implement the §5 state machine against those tests. Key invariants the tests pin down (do not deviate): abort is checked first every tick (gates, excessive yaw, timeout, clamp) and forces `active=false`, `left/right=0`; `MEASURE_NOISE` fills `b0`(mean)/`sigma`(std) over `noise_ticks` at zero thrust; `SETTLE` integrates `trim_diff += ki*(yaw − b0)*dt` (clamp each step; clamp-reached → abort), maintains a rolling mean of `(yaw − b0)` over `window_ticks`, and records when `|mean| < accept_k*sigma/sqrt(window_ticks)` AND the boat is making way AND `trim_diff`'s change over the window is negligible; after the last level, `AVERAGE` mode means each recorded point is averaged with the same-throttle point already in the passed-in existing table (Task 5 supplies it), else FRESH overwrite; `done` carries `out_table/out_count`. Because this is the crux and long, implement it incrementally: make test 1 pass, then 2, then each abort test, re-running after each.

Run: `python -m pytest tests/test_esc_trim_cal.py -q` → PASS (all blocks).

- [ ] **Step 4: Register + suite + commit**

`main/CMakeLists.txt`: add `"esc_trim_cal.c"`. `python -m pytest tests/ -q` → **51 passed**. `idf.py build` → exit 0.
```bash
git add main/esc_trim_cal.c main/esc_trim_cal.h main/CMakeLists.txt tests/test_esc_trim_cal.c tests/test_esc_trim_cal.py
git commit -m "feat(esc): calibration state machine (gyro-driven integral trim search)"
```

**Milestone 1 gate:** 51 host tests pass; `idf.py build` (both `SAS_ENABLE` states) exits 0; default runtime behavior unchanged (empty table, machine uninvoked). This is a safe merge point.

---

# MILESTONE 2 — wire it live (proto, ControlTask, UI)

## Task 4: Protocol — `CalibrateCommand` + `CalibrateStatus`

**Files:** `main/proto/boat.proto` (+ regen `boat.pb.{c,h}`, `proto/boat_pb2.py`), `main/pipeline.{c,h}`, and the `protoSchema` in `main/dashboard.html` + `tools/espnow_drive.py`.

- [ ] **Step 1: proto additions** — in `main/proto/boat.proto`:

```proto
message CalibrateCommand {
  bool  start                 = 1;  // true = begin sweep, false = stop/abort
  bool  average_into_existing = 2;  // FRESH (false) vs both-heading AVERAGE (true)
}

message CalibrateStatus {
  uint32 state          = 1;  // etc_state_t
  uint32 level_index    = 2;
  float  level_throttle = 3;
  float  trim_diff      = 4;  // live
  float  yaw_avg_dps    = 5;  // windowed
  bool   making_way     = 6;
  uint32 points_done    = 7;
}
```
Add to the `BoatMessage` oneof: `CalibrateCommand calibrate = 13;` and `CalibrateStatus calibrate_status = 14;` (confirm 13/14 are free — `training_log = 12` is the current highest).

- [ ] **Step 2: regenerate** both generated forms (this repo has done it before — find the command in git history: `git log --oneline -- main/proto/boat.pb.c | head`, or the nanopb generator invocation used previously). Regenerate `main/proto/boat.pb.{c,h}` (nanopb) and `proto/boat_pb2.py` (protoc). Do NOT hand-edit generated files.

- [ ] **Step 3: mirror protoSchema (parity — same commit).** Update the `protoSchema` string in `main/dashboard.html` and the schema `tools/espnow_drive.py` loads (`load_boat_pb2`) to include both new messages and the two new oneof fields. Missing this silently breaks decoding.

- [ ] **Step 4: pipeline handler.** In `pipeline.{c,h}`, add a `calibrate` handler registration mirroring `pipeline_register_arm_handler` / the `SteerCommand` path. Route `BoatMessage.calibrate` to it. Commit.

`git commit -m "feat(proto): CalibrateCommand + CalibrateStatus, mirrored into both dashboards"`

## Task 5: Wire into ControlTask + telemetry + NVS save

**Files:** `main/motor_control.c` only — no arbiter changes (see the note below on why an earlier draft of this task that touched `control_arbiter.c` was wrong).

**Read spec §7a before this task.** It defines the exact ownership/heartbeat contract this implements, and it is written against the real code — the paragraphs below restate the parts this task needs.

**Ground truth this design leans on (verified by reading `motor_control.c`, not assumed):**
- `s_last_control_rx_us` (file-scope, `motor_control.c:108`) is refreshed *only* by `notify_link_rx_locked()`, called *only* from inside the `manual_transport_control_arbiter_submit_*` wrappers and the event-push wrapper, *only* on an accepted submission (`motor_control.c:412,425,438,451,462`). Nothing else touches it. It already means, precisely, "a manual command was just accepted."
- `control_apply_decision`'s zeroing branch (`safe_stop`) is driven directly by `control_link_alive()`, which reads that same timestamp (`motor_control.c:696-698,138-144`) — **not** by `decision->failsafe` or `decision->drive_changed`. The comment at `motor_control.c:668-677` says this explicitly.
- `decision->drive_changed` is **not** a usable "fresh manual input" signal for this task: it's defined as `drive->changed || failsafe != previous_failsafe` inside the arbiter, and `run_control_cycle` layers a *second* failsafe recomputation on top (`motor_control.c:918-922`) that can also set it. It fires on any failsafe transition, including the natural one that happens ~400ms after a pilot goes hands-off to watch a calibration sweep — using it here would abort every real run shortly after it starts, with no new input at all. An earlier draft of this task also proposed invalidating the arbiter's manual proposal on calibration start; that's worse, because invalidating it *immediately* flips failsafe and self-triggers the same false abort on the very first tick. Neither is used below.

- [ ] **Step 1: config + state.** Add `#ifndef` Kconfig fallbacks + `menuconfig` entries for the `etc_cfg_t` values: `STABILITY_TRIMCAL_KI`, `_CLAMP`, `_ACCEPT_K`, `_MIN_SPEED_MPS`, `_WINDOW_TICKS`, `_NOISE_TICKS`, `_SETTLE_MS`, `_LEVEL_TIMEOUT_MS`, the level list, **and `STABILITY_TRIMCAL_MAX_YAW_DPS`** (its own value — do NOT reuse `CONFIG_STABILITY_SAS_RMAX_DPS`; see spec §4/§7 for why reusing it would create an ordering dependency between two subsystems that must stay independent, since calibration runs with SAS suppressed). A sensible starting default is the same number as `R_MAX`'s default — same physical idea ("a hard turn"), just not the same variable.

Add file-scope statics (near `s_calibrating` from Task 2):
```c
static bool s_cal_start_pending = false;
static bool s_cal_stop_pending = false;
static int64_t s_cal_baseline_rx_us = 0;
static int64_t s_cal_last_rx_us = 0;   /* calibration UI's own heartbeat */
static etc_t s_cal;
static etc_cfg_t s_cal_cfg;
```
`s_calibrating` itself (Task 2) stays Control-task-owned: only `calibration_tick` (Step 3, runs in the Control task) ever writes it. The two `_pending` flags are the cross-task handoff surface — the pipeline-handler task (Step 2) only ever sets them, under `s_arbiter_lock`, the same lock already used for every other cross-task write in this file.

- [ ] **Step 2: command handler.** The `CalibrateCommand` pipeline handler does the following and nothing else:
```c
portENTER_CRITICAL(&s_arbiter_lock);
if (cmd->start) s_cal_start_pending = true;
else s_cal_stop_pending = true;
s_cal_last_rx_us = received_us;
portEXIT_CRITICAL(&s_arbiter_lock);
```
It must call **none** of `motor_control_notify_link_rx()` / `manual_transport_control_arbiter_submit_*` — those refresh `s_last_control_rx_us`, the general link heartbeat, which calibration traffic must never touch (spec §7a). `s_cal_last_rx_us` is calibration's own, separate liveness signal, used only to detect the calibration UI itself going quiet mid-run.

- [ ] **Step 3: `calibration_tick(now_us)`** — called in `run_control_cycle` right after `control_apply_decision(&decision)` and right before `stability_sas_tick(...)` (so calibration's own actuator writes, when active, are always the last write of the tick — no overwrite race against `control_apply_decision`'s own zeroing branch).

Consume the pending flags first, under the same lock:
```c
portENTER_CRITICAL(&s_arbiter_lock);
bool start_req = s_cal_start_pending; s_cal_start_pending = false;
bool stop_req  = s_cal_stop_pending;  s_cal_stop_pending  = false;
int64_t link_rx_us = s_last_control_rx_us;
portEXIT_CRITICAL(&s_arbiter_lock);
```
If `stop_req` and `s_calibrating`: treat exactly like an abort (falls into the same `out.aborted`-handling as below — do not special-case it, Stop is just an externally-forced abort). If `start_req` and not already `s_calibrating`: honor it only if `esc_driver_get_state() == ESC_STATE_ARMED && control_link_alive()`; otherwise log why and drop it (a rejected start is not retried automatically — the operator must press start again). On honoring: `s_cal_baseline_rx_us = link_rx_us` (the value just read above); `esc_trim_cal_init(&s_cal, &s_cal_cfg)`; `s_calibrating = true`.

While `s_calibrating` (including the tick it just became true on): compute `bool manual_override = link_rx_us > s_cal_baseline_rx_us;` — any accepted throttle, winch, or steer submission since calibration started advances `s_last_control_rx_us`, so this one comparison covers all three, not steering alone. Also compute `bool cal_link_alive = (now_us - s_cal_last_rx_us) < CONTROL_DRIVE_TIMEOUT_US;` (reuse the existing 400ms constant — calibration's own dashboard going quiet is the same kind of staleness as a driving link going quiet). Call:
```c
etc_out_t out = esc_trim_cal_step(&s_cal, &s_cal_cfg, now_us,
                                   fusion.yaw_rate, gps_speed_mps,
                                   esc_driver_get_state() == ESC_STATE_ARMED,
                                   control_link_alive() && cal_link_alive,
                                   imu_icm_ok(),
                                   manual_override || stop_req);
```
(`fusion.yaw_rate` from the same snapshot read `stability_sas_tick` already uses; `gps_speed_mps` from `gps_driver_get_fix`.) If `out.active`: `esc_driver_set_throttle(out.left_cmd, out.right_cmd)` (bypassing the saved-table apply — `s_calibrating` gates that in Task 2) and `steer_driver_set(0.0f)`; publish `CalibrateStatus` into the double-buffered status (`s_status_buffers` generation pattern, same mechanism `status_commit_current` already uses).

On `out.done`: `motor_control_set_esc_trim(out.out_table, out.out_count)` **and `fs_save_esc_trim(&blob)`** (Task 1's own NVS record — building `blob` from `out.out_table`/`out.out_count` plus `ESC_TRIM_NVS_MAGIC`; never `fs_save_calibration`, which is the unrelated IMU blob and would silently fail its own size check on the next boot if this ever touched it). On `out.aborted`: discard, do not save, log `out.reason`. Either way: `s_calibrating = false`.

Do **not** write `s_last_control_rx_us` anywhere in this function. Leaving it exactly where the pilot's last real command left it is what makes `control_apply_decision`'s existing `control_link_alive()`/`safe_stop` path correctly zero the actuators the instant calibration stops writing — the same mechanism that already holds throttle at zero through a dead link today, reused rather than reinvented. (If calibration aborts within the same tick it started — some gate already failing — the pilot's still-fresh, sub-400ms-old command may resume instead of a hard zero; that's correct, not a gap, per spec §7a.)

- [ ] **Step 4: SAS suppression.** In `stability_sas_tick`, early-return while `s_calibrating` is true (one line).

- [ ] **Step 5: build both configs + commit.** `idf.py build` with `STABILITY_SAS_ENABLE` off *and* on → exit 0. `python -m pytest tests/ -q` → 51 (wiring isn't host-tested; the pure core is).
```bash
git add main/motor_control.c
git commit -m "feat(control): run ESC trim auto-cal in ControlTask, own heartbeat, save to its own NVS record"
```

## Task 6: Dashboard triggers (both UIs)

**Files:** `main/dashboard.html`, `tools/espnow_drive.py`.

- [ ] Add a **Start Auto-Calibration** control + an `average_into_existing` toggle + a live progress readout (state, level, trim, windowed yaw, points done) fed by `CalibrateStatus`. **Stop reuses each tool's existing E-stop button** — no new stop button. Concretely: extend each dashboard's existing E-stop click handler to *also* send `CalibrateCommand{start:false}` alongside whatever it already sends. This is not redundant with the `armed` gate `esc_trim_cal_step` already checks every tick (Task 5, Step 3) — that gate catches a real disarm too, but disarm goes through the async arm-sequence task and may take more than one control tick to land, while `CalibrateCommand{start:false}` is a direct pipeline message processed the same tick it arrives (Task 5, Step 2/3's `stop_req` path). Both exist; `stop_req` is the fast path, `armed` going false is the backstop. Match each UI's existing send pattern (`BoatMessage.create({ calibrate: {...} })`). Commit.

`git commit -m "feat(ui): ESC auto-calibration trigger + progress on both dashboards"`

---

## Global verification
- Host suite green after each pure task: 48 → 49 (T1) → 49 (T2) → 51 (T3).
- `idf.py build` after Tasks 2, 3, 5 (Task 5 in both `SAS_ENABLE` states).
- Default behavior unchanged until a table is learned (empty table = 0 trim; machine uninvoked).

## First-tethered-run acceptance (from spec §7 — do before trusting any saved value)
- **Sign check:** induce/observe a right yaw at a fixed level; confirm `trim_diff` moves to *oppose* it (CTRL log). Wrong sign → runs to clamp and aborts within ~1–2 s (fails safe); fix = flip the `ki` sign in `esc_trim_cal_step`.
- Calm water, tethered, hard-kill in hand. Verify each abort (Stop/E-stop, disarm, link pull, manual throttle, manual steer) drops thrust immediately.
- After a clean sweep: confirm NVS holds the table under its own `"esc_trim"` key (reboot, check the load log), that the separate `"imu_cal"` blob is untouched, and that equal-throttle driving now tracks straighter than before.

## Open items / do not assume
- **AVERAGE (two-heading) mode** ships but is exercised by running twice (once each heading); no mid-sweep auto-reverse.
- **Level list default** (20/40/60/80 %) is provisional — confirm on the water.
- Continuous-ramp calibration is explicitly out of scope.

## Self-review
- **Spec coverage:** integrator-search (§2)→T3; no-magic-numbers constants (§4, incl. making-way + b0)→T3 cfg/tests; runtime ordering (§8)→T1/T2; state machine (§5)→T3; scheduler model (§6)→T5 (one-tick tick, SAS suppression); safety/aborts (§7)→T3 tests + T5; storage (§8)→T1; both dashboards (§8)→T4/T6. ✔
- **Placeholders:** pure units (T1, T3) carry full code/tests; wiring tasks (T2, T4–T6) are exact specs + build/parity gates, matching how this codebase leaves ESP-IDF-saturated files (motor_control, pipeline) to build-gate rather than host-test — consistent, not a gap. ✔
- **Type consistency:** `EscTrimPoint`/`esc_trim_count` (T1) consumed by `esc_trim_apply` (T2), `esc_trim_cal` output table (T3), and `fs_save_esc_trim`/`EscTrimNvsBlob` (T1, called from T5) — never `fs_save_calibration`, which stays untouched throughout; `etc_cfg_t`/`etc_t`/`etc_out_t`/`esc_trim_cal_step`/`manual_override` names match header↔tests↔wiring. ✔
