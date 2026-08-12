# ESC Differential-Trim Auto-Calibration — Concept & Design

**Date:** 2026-08-12
**Status:** Design (approved in direction; no code yet). Extends the stability-SAS work on branch `feat/stability-sas`.
**Owner:** Kiet
**Sibling spec:** `2026-08-11-boat-stability-sas-autopilot-design.md` (the rudder yaw-rate stabilizer this builds on)

---

## 1. Goal

The two water-jet motors are not matched — left starts producing thrust at ~5 % command, right at ~20 %, and the slopes above those floors differ too (observed on hardware: right ramps faster). Task B's per-motor floor fix made both motors *start* together; it did not make them produce *equal thrust* at a given command. So an equal-throttle "go straight" command still yaws the boat.

We cannot measure motor RPM or thrust on a bench. So we learn the correction **from the boat's own motion**: with rudders centered and steady throttle, gyro yaw-rate tells us which jet is stronger; adjust the left/right differential until the boat travels straight; save that differential per throttle level. Runtime interpolates the table so equal forward demand produces no unwanted yaw at any speed.

**This is the DC-bias half of "most stable boat."** The trim table removes the *permanent* thrust imbalance so the boat tracks straight open-loop; the SAS rudder loop (sibling spec) then handles only the *dynamic* disturbances (gusts, wake, current) instead of burning its limited authority fighting a constant lean. Neither alone gets there; together they do.

**Hard constraint (the user's, and the spine of this design): no magic numbers that determine the result.** We cannot bench the motors, so the answer must come from sensor data, not from constants picked out of thin air. §4 accounts for every constant that remains and shows none of them decides whether the boat goes straight.

---

## 2. Core principle: calibration is a control loop, not a search

The trim that makes the boat go straight is exactly the trim at which **integrated yaw-rate stops accumulating**. So we do not *search* (nudge-by-a-step, wait, re-check — every part of which is a guessed constant). We let a slow integrator *find* it:

```
per control tick, at a fixed throttle level, rudder locked centered, SAS suppressed:
    trim_diff += Ki * yaw_rate * dt          # yaw_rate: gyro-Z, from FusionResult
    left_cmd   = level - trim_diff / 2         # symmetric about the requested level
    right_cmd  = level + trim_diff / 2         # -> total commanded thrust preserved
```

Boat yaws right → integrator shifts thrust toward the right side → yaw falls → when the boat runs straight, `yaw_rate ≈ 0`, so `trim_diff` **stops moving on its own**. The value it settles at *is* the answer for that level. This is the same bounded-integrator ("half-I") concept already in the SAS design, reused — here driving ESC differential instead of rudder, and run slowly to steady state.

This deletes the worst magic numbers outright: **no step size** (the integrator self-scales), **no fixed convergence time** (it converges when the physics says straight), **no arbitrary yaw threshold to nudge against.**

### 2a. Prior art — this is a known, proven pattern (2026-08-12 web check)

"Let a controller's integrator wind up to counter a persistent imbalance, then *save* that converged value as a trim offset" is exactly **ArduPilot's `SERVO_AUTO_TRIM`** (Plane): it "constantly monitors how much correction is needed to maintain level," stores that correction into the persistent `SERVOx_TRIM` every ~10 s, limits it to ~20 % of range (a clamp), and only updates when there is *no pilot input*, the vehicle is *under autopilot control*, and it is *above a minimum ground speed*. That is our design, feature-for-feature: converged integrator → saved trim, a clamp guard rail, and gate-on-conditions. ArduRover has the ground-vehicle analog (**Save Steering Trim** for skid-steering). Twin-thruster USV testing in the literature sets port/starboard to different speeds and characterizes yaw "in the absence of wind and current" — validating both the differential-thrust→yaw model and correction #4 (calm water). Thruster "bias faults" are a recognized fault class handled with integral methods — and our trim-clamp-abort is exactly a bias/stuck-fault detector. Two refinements below (§3, the making-way gate and residual-bias subtraction) come straight from that prior art.

---

## 3. The four corrections (all folded in)

1. **Gyro only.** The loop uses `FusionResult.yaw_rate` (gyro-Z). The magnetometer is never consulted while jets run — motor current + nearby magnets make mag heading throttle-dependent on this hull (documented). Gyro is the signal Task A published specifically because it is motor-independent.
2. **"Settled" is a sustained low *moving* average, not the stationary noise floor.** A moving boat in wind/current/waves has real yaw that is not motor imbalance. So: at the start, thrusters off and boat ~still, **measure both the residual gyro-yaw *bias* `b0` and the noise std `σ`** — then during the loop use `(yaw_rate − b0)`, so a gyro-bias drift since the last calibration cannot leak into the trim (a classic online-gyro-bias concern; without it the boat would learn to curve just to cancel a sensor offset). Then require the **time-averaged** `(yaw − b0)` over a sustained window *while under way* to hold inside the accept band before a point is recorded, disturbance averaged out. **And a making-way gate** (refinement from ArduPilot's min-ground-speed rule): a point counts only while GPS speed shows the boat actually translating — at a throttle so low the boat barely moves, yaw is drift/noise, not thrust imbalance, so that level is flagged un-calibratable rather than trusted.
3. **Ki is a real tuning/safety value, not just "speed."** Too high oscillates; too low doesn't just take longer — a long integration window can *bake in* the current/wind bias. So Ki is slow-but-bounded, trim is clamped, and **hitting the clamp is an abort** (not saturate-and-continue).
4. **Calm water, and sweep both headings.** A single sweep in one heading learns "compensate for today's current/wind," not motor imbalance. Running each level in one heading and again in the reversed heading and **averaging** cancels a steady earth-frame current/wind (it flips relative to the hull when the boat reverses; motor bias does not). Supported via an AVERAGE-into-existing command mode (§8): run once each way, second run averages. Calibrate in the calmest water available regardless.

---

## 4. Every remaining constant — and why none of them is the answer

| Constant | Role | Why it is not a "magic answer" number |
|---|---|---|
| **Accept band** (`k·σ/√N`) | "is yaw ≈ 0 yet" | `σ` is **measured** from this boat's own stationary gyro noise at the start of every run — not picked. `N` is the moving window's sample count; the mean of N samples has std `σ/√N`, so the band is a **statistical confidence** the windowed mean is indistinguishable from zero, not a hand-tuned threshold. `k` is a confidence multiple (3 = 3σ ≈ 99.7 %), a standard statistical choice. |
| **Residual bias `b0`** | subtracted from yaw during the loop | **Measured** at rest (thrusters off) at run start; not a constant. Keeps a drifted gyro bias out of the trim. |
| **Making-way gate** | point counts only while translating | **GPS-measured** speed; a conservative "actually moving vs. not" guard (mirrors ArduPilot's min-ground-speed rule), not a tuning knob. |
| **Ki** (integrator gain) | convergence *speed* | Sets how fast, not where. The settling trim is a property of the boat, found regardless of Ki. Chosen slow; correction #3's risks (oscillation / bias bake-in) are bounded by the clamp + timeout, not by getting Ki exactly right. |
| **Trim clamp** (±, e.g. 2× the known floor gap ≈ ±30 %) | safety guard rail | If straightening needs more differential than this, a jet is mechanically wrong (jammed impeller, broken prop) — fail loud, don't command wild asymmetry. A guard rail, not a tuning knob. |
| **Excessive-yaw abort** | safety trip | **Reuses SAS's existing `R_MAX`** — if calibration ever sees more yaw than a deliberate full-stick turn, the boat is turning hard, not drifting: abort. No new number. |
| **Per-level timeout** | give-up backstop | Convergence comes from §2 settling; the timeout only bounds a run that never settles. Pure safety. |
| **Throttle levels** (e.g. 20/40/60/80 %) | where to sample | **Configuration, not magic** — the operator's chosen sample density, a list, adjustable. The one place a human picks the points. |
| **Level settle-in wait** | let the hull reach steady speed after a throttle step, before integrating | Physical settling of the boat's speed; modest and conservative; affects only *when integration starts*, not the value it finds. |

None of the top rows decides whether the boat goes straight. They are confidence, speed, and guard rails.

---

## 5. The calibration state machine (inside ControlTask, one tick at a time)

Lives in the existing **Control task (core 0, prio 10, 100 Hz)** as a `calibration_tick()` beside `stability_sas_tick()`. Each tick does one tiny update — integrate, update the rolling window, check transitions — and returns. It never waits, never loops for seconds, never holds the CPU. Pure sequencing logic is factored into a host-testable module (`esc_trim_cal.c`, mirroring how `arm_sequence.c` is a testable state machine); the ControlTask wiring is the thin glue.

```
IDLE
  └─(CalibrateCommand.start, and ARMED + link-alive)─► MEASURE_NOISE
MEASURE_NOISE   (thrusters idle, boat ~still): collect gyro yaw window,
                compute residual bias b0 (mean) AND noise σ (std). Sets the
                accept band and the bias subtracted for the whole run.
  └─────────────────────────────────────────────────► RAMP  (level[0])
RAMP(level i)   ramp both ESCs to level i (trim_diff carried from prior
                level as the starting guess); wait settle-in, and for GPS
                speed to show the boat making way (else flag level
                un-calibratable and skip).
  └─────────────────────────────────────────────────► SETTLE(level i)
SETTLE(level i) each tick: e = yaw - b0; trim_diff += Ki*e*dt; apply
                level∓trim/2; push e into rolling window. When the windowed
                mean stays in the accept band for the full window, AND
                trim_diff is no longer drifting, AND the boat is still
                making way ─► RECORD.
RECORD(level i) store (level i, trim_diff) in the in-RAM table.
  ├─ more levels ─► THROTTLE_DOWN ─► RAMP(level i+1)
  └─ last level  ─► FINISH
FINISH          throttle → 0, rudder centered, write table to NVS
                (FRESH: overwrite; AVERAGE: mean with existing), ► IDLE.
ABORT (from any active state) throttle → 0, rudder centered, DISCARD the
                in-RAM table (previous NVS table untouched), ► IDLE.
```

`THROTTLE_DOWN` briefly eases throttle toward idle between levels — a visible progress beat and a clean restart for the next level (matches "stop, then thrust again at a different rate").

---

## 6. Control-authority & scheduler model

Calibration changes *control authority*, never the *schedule*:

- **Scheduler untouched.** SensorBus / Fusion / GPS / ToF keep running on core 0; Detect-ML / camera / WS_TX / SD logging keep running on core 1. The boat keeps perceiving and streaming telemetry throughout — you can watch it calibrate.
- **While active, calibration is the exclusive ESC command source, holds the rudder at 0.0, and suppresses SAS** (`stability_sas_tick` early-returns while calibration is active — a single shared "who owns the actuators" flag). This is the "another command producer plugs into the arbiter" pattern the architecture was built for.
- **Manual override always wins.** Any manual steering command (or throttle) instantly reclaims authority — which is exactly an abort (§7). The human is never locked out.
- **Non-blocking status out.** Progress (state, current level, live trim, windowed-yaw, recorded points) is published into a double-buffered status struct (the `s_status_buffers` generation-counter pattern already used for motor status); the WS / ESP-NOW TX task reads it. The control loop never blocks on telemetry.

---

## 7. Safety & aborts

**Preconditions to start:** ARMED **and** control-link-alive (same gate as driving). Thrusters are live during calibration — this is an on-water, tethered, hard-kill-in-hand activity.

**Immediate abort on any of:** existing Stop / E-stop button, control-link loss, disarm, IMU fault (`imu_icm_ok()` false or fusion stale — reuse the SAS staleness/age check), **manual steering command**, **trim clamp saturation** (§4), **excessive yaw** (> `R_MAX`), **per-level timeout**. Every abort: throttle → 0, rudder centered, in-RAM table discarded, SAS + manual driving resume immediately. Nothing latched.

**NVS write only on clean full-sweep completion.** A partial or aborted sweep never persists; the previous known-good table survives.

**First-tethered-run sign check (mandatory acceptance test).** A wrong feedback sign is positive feedback: the integrator runs straight to the trim clamp and aborts within a second or two — it **fails safe** (clamp-abort, not runaway). Fix = flip one sign constant. Acceptance: induce/observe a right yaw and confirm `trim_diff` moves to *oppose* it before trusting any saved value.

---

## 8. Command, storage, runtime application

**Command — new protobuf `CalibrateCommand`** (shape like `ArmCommand`, so it is transport-agnostic): `bool start`, `bool average_into_existing` (FRESH vs the both-heading AVERAGE mode of correction #4). Stop reuses each tool's existing E-stop. Add the trigger to **both** `dashboard.html` (WiFi) and `tools/espnow_drive.py` (ESP-NOW field) — same one command, cheap in both; calibration may happen on either link. Mirror `boat.proto` into both dashboards' `protoSchema` in the same commit (documented parity requirement).

**Storage — extend `CalibrationData`** (in `include/common.h`, persisted by `file_system.c` over NVS, exactly as gyro/mag bias already are) with a small fixed-capacity table: `uint8_t trim_count`, and `trim_points[N] = { float throttle_frac, float trim_diff }`. Bump the NVS struct version / handle absence so an old blob loads with an empty table (trim = 0 → today's behavior, nothing breaks).

**Runtime application — the ordering you specified, mapped exactly onto existing code:**
```
common throttle ─► trim table lookup(common) ─► balance L/R
                ─► + pilot's turn difference
                ─► per-ESC floor/curve (esc_map) ─► pulses
```
`control_arbiter` already keeps `throttle` and `rudder` separate before mixing, and `(L+R)/2` recovers the common throttle **losslessly** (the pilot's turn cancels in the average). So in `esc_driver_set_throttle(left, right)`: `common = (left+right)/2`; `trim = esc_trim_lookup(common)` (pure, host-testable linear interpolation between recorded points; clamp to the nearest endpoint outside the calibrated range); `left -= trim/2; right += trim/2`; then the existing `esc_map` floor/curve per side. Trim fixes the permanent bias on the common component; the pilot's turn stays fully intentional on top; the floor mapping stays last. **During calibration itself the saved-table lookup is bypassed** (the routine is *learning* the table, producing the balance directly) while the floor mapping still applies (motors always need the floor to spin) — a mode flag on the ESC output path.

---

## 9. Testing

Host-testable pure units (compile-and-run harness, the repo's existing pattern):
- **`esc_trim_cal.c`** — the integrator update + rolling-window accept logic + state transitions, fed synthetic yaw sequences (straight → converges & records; persistent yaw → drives trim then converges; runaway → clamp-abort; disturbance spike → excessive-yaw abort; never-settles → timeout). Same style as the `stability_control` and `arm_sequence` host tests.
- **`esc_trim_lookup`** — interpolation: exact at points, linear between, endpoint-clamp outside, empty table → 0.
- **Trim application math** — `common=(L+R)/2`, `left−=trim/2`, `right+=trim/2` round-trips the pilot turn unchanged; verify the ordering vs `esc_map`.

Not host-testable (build-gate + on-water): the ControlTask wiring and the actual convergence on a real hull — covered by `idf.py build` in both flag states and the tethered acceptance run (§7).

---

## 10. Scope, phasing, open items

- **v1 = single clean sweep, FRESH save.** AVERAGE-into-existing (both-heading, correction #4) ships in the same command but is exercised by running twice; a mid-sweep auto-reverse prompt is explicitly *not* built.
- **Continuous-throttle-ramp calibration** (learn a full curve instead of discrete points) is a deliberate non-goal for now — discrete-settle is safer and interpolation covers the gaps. Noted as a possible future refinement.
- **Default level list** (20/40/60/80 %) is a starting value to confirm on the water; it is config, not a commitment.
- **Requirements:** serves the straight-line-tracking foundation for R2 (autonomy) and the stable-platform goal for R3 (ML) — same lineage as the SAS spec.

---

## 11. Self-review
- **Placeholder scan:** none — every constant in §4 is either measured, a named reuse (`R_MAX`), a physically-motivated guard rail, or explicit config. No "TBD".
- **Consistency:** the "no magic numbers" claim (§1) is discharged item-by-item in §4; the scheduler model (§6) matches the "one tick, non-blocking" constraint; the runtime ordering (§8) matches the control-loop framing (§2) and the SAS suppression (§6).
- **Scope:** one subsystem (learn + store + apply a trim table), single implementation plan. AVERAGE mode and continuous-ramp explicitly bounded (§10).
- **Ambiguity:** trim is defined once as symmetric about the common throttle (`level ∓ trim_diff/2`), used identically in the loop (§2), the state machine (§5), and runtime application (§8).
