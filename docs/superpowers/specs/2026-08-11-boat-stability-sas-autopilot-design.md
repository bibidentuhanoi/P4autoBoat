# Boat Heading-Stability Autopilot — Concept & Plan

**Date:** 2026-08-11
**Status:** Concept / design (no code changes). Foundation for manual driving now, autonomy later.
**Owner:** Kiet
**Revised:** 2026-08-11 — post devil's-advocate; corrected for the water-jet catamaran reality (§2.0) and the yaw-rate data-path gap (§2.2). Fastest safe path to water is the A→B→C slice under §9.

---

## 1. Goal

Make the boat **go where you point it and hold straight without drifting**, while the pilot steers by moving the rudder, on a hull with **two rudders (one steering axis)** and **two ESCs with very different dead-bands** (left spins from ~5 %, right only from ~20 %).

The controller must **run underneath the pilot at all times** — the human commands a direction, the algorithm continuously *shapes and stabilises* the motion. Once manual driving is rock-solid, the identical interface becomes the mounting point for **GPS waypoint following (R2)** and **camera-ML obstacle avoidance (R3)**: the planner just replaces the human as the demand source. The boat's job is to be the *biggest, most stable, most predictable platform possible* so the ML model only has to decide *where to go*, never how to fight the boat.

**Explicitly out of scope for this doc:** writing code. This is concept + staged plan.

**Scope line, stated plainly so it can't be read as "SAS already controls both ESCs":**
- **SAS (this doc, §5.2, Tasks A/B/C — built)** = a real-time, per-tick **rudder-only** correction. It reads gyro yaw-rate and moves the rudder. It has never written to an ESC and does not know the motors exist.
- **ESC differential trim (sibling spec, `2026-08-12-esc-trim-autocal-design.md`)** = a **static, one-time-learned** per-throttle-level correction. It runs only when explicitly triggered, then bakes a fixed number into a lookup table applied at normal drive time. It is not a running controller and does not react to anything moment-to-moment.
- **Dynamic differential-thrust steering (§5.4 below)** = a *third*, **not-yet-built** thing: a real-time second steering channel that would use the ESCs continuously, alongside the rudder. It is design-only, deferred, and is a completely different mechanism from ESC trim above — trim removes a *permanent* bias once; this would be a *live* loop, forever.

Only the first of these three exists in the firmware today.

---

## 2. The setup as it actually is (ground truth from the firmware)

### 2.0 Hull & propulsion (from the CAD — this changes assumptions)
`boatMainReimagine.FCStd`: a **water-jet catamaran** — `nozzle`, `jetpumphead`, two `A2212 BLDC` motors, mirrored `theboathull`, `hullconnector`, `MG995` steering servos, `corner keel`. ≈ **0.5 m LOA, ~0.3 m hull-to-hull beam**. Consequences:
- **Water jets are unidirectional** (no reverse thrust — matches the arm-at-min ESC). Differential-thrust yaw is **one-sided** (you can only speed a jet *up*), and any yaw-by-thrust **adds** net forward thrust.
- **Wide beam (~0.15 m half-beam) = strong differential-thrust yaw authority**, even at low speed — a catamaran advantage a monohull lacks.
- **Rudder location CONFIRMED (2026-08-12, by the boat's designer):** the `MG995` rudders sit in front of each nozzle and deflect the **jet efflux** directly — rudder authority tracks *throttle*, not boat speed, and is available whenever the jet is thrusting, independent of hull speed through the water. §5.4's allocation logic is settled on this basis; the hull-wake branch is dropped.
- Small + light + jet on/off ⇒ likely **fast, twitchy yaw** → sample-rate matters (§6, §9 Phase 0).

### 2.1 Effectors
| Effector | Pin / bus | Driver fact |
|---|---|---|
| Left ESC | GPIO31, MCPWM grp 0 | Loaded pad → 40 mA drive. Unidirectional, arms-at-min, inverted pulse. |
| Right ESC | GPIO33, MCPWM grp 0 | Same operator, independent comparator. |
| Rudders ×2 | GPIO32, MCPWM grp 1 | **One** shared signal; both servos deflect together. Home = **full-right (+1.0)**. |
| Winch | GPIO35 | Already has `WINCH_DEADBAND_US` dead-band comp — the pattern we reuse for the ESCs. |
| Servo rail | GPIO36 | Auto-powers on first non-zero steer/winch; thrusters arm-gated (GPS-or-force). |

- `esc_driver.c :: throttle_to_us()` is **one symmetric linear map `[0,1] → [1000,2000] µs` for both motors — zero dead-band compensation.** This is the direct cause of drift under any differential command: a small differential moves only the left motor until the right crosses ~20 %.
- Two steering paths already coexist: the `MotorCommand` mix (`left = thr+rud`, `right = thr−rud` → differential thrust) **and** the dedicated `SteerCommand` → rudder servo. The design **unifies both into one yaw demand**.

### 2.2 Sensors and their failure modes
- **Yaw rate (gyro-Z):** the reliable, mag-independent stability signal — BUT **not currently exposed to the control loop.** `FusionResult = {pitch, roll, heading}` only; fusion computes `gz_rate` internally (`sensor_fusion.c:171`) but never publishes it, and returns no timestamp. Today the loop *differences heading* for rate (`motor_control.c:228`) — noisier and mag-contaminated. **Publishing `gz_rate` + a timestamp is prerequisite work (Phase 0).**
- **Heading (fused):** a complementary blend — gyro-Z integrated, corrected ~2 %/update toward mag when `mag_norm>800` and tilt<45° (`sensor_fusion.c:177–186`). The QMC mag can **silently freeze to standby** (`imu_mag_ok()` detects it). Two high-current BLDC + a CAD `Magnet` part near the `SensorsCASE` ⇒ heading may be **throttle-dependent** (dynamic hard-iron) — characterise before trusting heading-hold.
- **GPS:** speed + course-over-ground; needed later for *true* no-drift-over-ground (heading-hold holds heading, not ground track).
- **Pitch/roll:** not used by the yaw law today. **Open decision:** confirm unused, or use as a safety clamp (cut authority on bow-bury / hull-lift).

### 2.3 Current controller (the "dry-run PID with half-I")
`motor_control.c :: heading_assist_dry_run_tick()`, runs at **10 Hz** (every 10th cycle of the 100 Hz control loop), **dry-run (logs `CTRL_DRY,…`, actuates nothing)**:
- Law: `correction = Kp·p_error − Kd·yaw_rate + trim`.
- **P:** adaptive `Kp` (0.012–0.050), scheduled up when error grows, down on overshoot.
- **D:** `−Kd·yaw_rate`, derivative-on-measurement (good — no derivative kick).
- **"Half-I" =** `trim`: integrates only the *sign* of error at 0.002 units/s, clamped ±0.12 — a deliberately weak, bounded integrator for persistent bias.
- Captures a target heading only when the stick is centred; **any deflection disables it**. Outputs both a rudder assist (±0.25) and a small differential-thrust assist (±0.06).

**This is already the right *shape*.** The plan formalises and promotes it, not replaces it. **Data-path reality:** `fusion_get_result()` exposes no yaw-rate and no timestamp, so the promotion depends on the Phase-0 sensor-publish work below.

### 2.4 Task / core layout (critical for real-time)
`runtime_schedule.c` pins **Control to core 0, priority 10 (highest in the system), `critical`**, alongside SensorBus(8)/Fusion(7)/GPS(6)/ToFRead(5). **Core 1** holds all heavy/bursty work: Detect (camera-ML), CamDrain, Snapshot, ToFProc, WS_TX, Diagnostics, TrainingLog, StatusLED. `runtime_schedule_validate()` refuses to boot unless Control and SensorBus are core-0/critical. `RUNTIME_TASK_WAYPOINT` and `RUNTIME_TASK_ML_CONTROL` are **reserved but not yet scheduled** → the autonomy producers belong on core 1.

### 2.5 Safety model (must be preserved)
- Two power domains: thrusters (arm-gated, GPS-or-force) and servo rail (auto-power on first non-zero command; explicit PWR-OFF latches a cut).
- **400 ms control-link-silence failsafe:** zero throttle, centre rudder, cut rail. `heading_assist_reset()` clears controller state on failsafe/disarm.
- `control_arbiter` validates every command source; today only `MANUAL` is accepted.

---

## 3. Algorithm decision — PID, not LQR

**Verdict: PID — specifically a PD inner/outer cascade + a bounded, conditional integral (the "half-I" you already have) — structured as a Stability-Augmentation System. Not LQR (yet).**

Why, concretely for this boat:

1. **One controlled axis (yaw).** LQR earns its keep on coupled *multi-axis* plants. On a single axis it collapses to the *same* two-gain state feedback `u = −k₁·heading_error − k₂·yaw_rate` — which is exactly what the existing PD law already is. LQR would only give a principled way to *pick* those two gains, not a different controller.
2. **LQR needs a model you don't have.** It requires the boat's Nomoto steering model `ṙ = −r/T + (K/T)·δ` (gains `K`, `T`), obtained by an on-water zig-zag system-ID. PID can be tuned empirically without a model.
3. **Your dominant problems are nonlinear** — the asymmetric ESC dead-band, "rudder only works with water flow", saturation. LQR (linear) handles these *badly*; they must be solved *outside* the linear law (dead-band linearisation, control allocation, anti-windup) regardless of PID vs LQR.
4. **Embedded transparency.** Gain-scheduling, anti-windup, manual-override blending, and graceful sensor-loss degradation are all clearer and safer in the PID framework on an MCU.

**When LQR/LQI *does* become worth it:** Phase 5 (optional). After an on-water system-ID gives you `K`/`T`, an **LQI** (LQR + integral) yields the same feedback structure with model-justified gains and a clean effort/aggressiveness trade-off. It's a *tuning upgrade of the same architecture*, never a rewrite. Literature agrees: PID-on-Nomoto is the workhorse; fuzzy/LQR/MRAC are refinements layered on top.

---

## 4. Control architecture — layered SAS

The core idea (borrowed from fly-by-wire **rate-command / attitude-hold**): the pilot's rudder is **not a rudder angle — it is a steering demand (desired yaw rate)**. The stabiliser is always live; the pilot steers *through* it.

```
 MANUAL rudder ─┐
 GPS waypoint  ─┼─►[1 INTENT]  yaw-rate demand r_cmd  +  speed demand   (ONE interface, any source)
 ML planner    ─┘        │
                         ▼
                 [2 STABILISER]   cascade:
                     outer (mag-gated):  heading_err ──Kψ──► r_hold ─┐
                     inner (gyro, ALWAYS): r_err = (r_cmd or r_hold) − gyroZ
                     u_yaw = Kr·r_err − Kd·ẏaw + trim(bounded, conditional)
                         │  u_yaw = abstract "yaw effort", speed demand passes through
                         ▼
                 [3 SHAPING]   rate-limit demand · slew-limit outputs · LPF gyro · anti-windup
                         │
                         ▼
                 [4 ALLOCATION]   u_yaw → rudder_angle + thrust_differential
                         │         rudder-primary, speed-scheduled; ESC asymmetry cancelled here
                         ▼
                 [5 ACTUATORS]   steer_driver (rudder)  +  esc_driver (L/R, dead-band-linearised)
```

**Why this is the "stable ground for ML":** perception (Detect, core 1) → planner (ML_CONTROL, core 1) emits the *same* `r_cmd + speed` a human does. Layers 2–5 hide the boat's asymmetry and instability, so the model never learns "left motor is stronger" or "rudder is weak at low speed." Manual and ML feel identical to the hull.

---

## 5. Layer specifications (concept level)

### 5.1 Intent (demand) layer
- **Manual:** pilot steering input → `r_cmd = stick × R_MAX` (e.g. `R_MAX ≈ 30–45 °/s`, tuned). Centre (|stick| ≤ deadband) → `r_cmd = 0` → hold heading. Throttle stick → forward-speed demand.
- **Autopilot:** waypoint/ML producer emits the same `r_cmd + speed` through the arbiter. No actuator code changes — the arbiter already reserves `SOURCE_WAYPOINT` / `SOURCE_ML`.
- **Priority / override:** manual always outranks autonomy; a manual demand pre-empts an ML demand at the arbiter. (Arbitration policy is an explicit design item for the autopilot phase.)

### 5.2 Stabiliser (the algorithm)
**Cascade, so manual-steer and heading-hold share one inner loop:**
- **Inner loop — always on, gyro-only:** `r_err = r_target − gyroZ`; contributes `Kr·r_err − Kd·ẏaw`. Runs even if the mag is dead → the boat *stays damped and controllable* with no heading reference. This is the stability backbone. **(Depends on the Phase-0 yaw-rate publish; until `gz_rate` is exposed it rides differenced heading and does NOT cleanly survive mag standby.)**
- **Outer loop — heading hold, mag-gated:** when `r_cmd ≈ 0` **and** `imu_mag_ok()`, capture `ψ_target`; produce `r_hold = clamp(Kψ·wrap180(ψ_target − ψ))`, which *feeds the inner loop as its rate target*. When the pilot steers (`r_cmd ≠ 0`), `r_target = r_cmd`; when centred, `r_target = r_hold`; on re-centre, re-capture `ψ_target`.
- **Integral ("half-I") — bounded + conditional:** keep the slew-limited, ±-clamped trim. **Freeze it** while the pilot is actively steering (`|r_cmd| > 0`) or any actuator is saturated (anti-windup). It absorbs steady bias: residual thrust asymmetry, wind, current, rudder trim offset.
- Keep the existing adaptive-Kp / overshoot logic; it's a lightweight gain-schedule that already works in the logs.

### 5.3 Shaping ("auto input-shaping" — the stability recipe)
True ZV input-shaping is for lightly-damped oscillators; the boat's yaw is heavily water-damped, so the high-value shaping is:
1. **Reference shaping:** rate-limit (and lightly LPF) `r_cmd`/`ψ_target` → the boat follows a motion profile, never a step.
2. **Output slew-limiting:** cap rudder-angle rate and throttle rate (units/sec, scaled by *measured* dt). Protects mechanics, forbids impossible steps, doubles as jitter protection (§6).
3. **Filtered derivative:** first-order LPF on gyro-Z feeding the D term; **use gyro-Z, not heading-differencing** (less lag/noise, survives mag standby).
4. **Anti-windup:** conditional integration on the trim (above).
- *Ordering caveat:* if a true input-shaper is ever added, it must precede — not follow — the slew limiter, or the limiter degrades it.

### 5.4 Allocation (rudder-primary + throttle-scheduled thrust assist) — schedule confirmed, **mechanism NOT built**
**Not implemented.** "Confirmed" below means the *schedule variable* (throttle, not boat speed — settled 2026-08-12) if and when this is ever built — not that it exists. Tasks A/B/C built the rudder-only inner loop (§9 Phase 3) directly; this allocation step (§9 Phase 2) was skipped, not done first. Do not confuse this with the separate ESC-trim-autocal spec: that is a static table applied once per drive session; this would be a live per-tick loop, permanently. Neither exists yet.

`u_yaw → { rudder_angle, thrust_differential }`. **Rudder location is confirmed jet-efflux (§2.0)**, so the schedule is settled: rudder authority tracks **throttle**, not boat speed — it carries yaw at all speeds *whenever throttle is up*, independent of hull speed through the water. Differential thrust is **trim/assist only**, scheduled on **throttle** (added mainly when throttle is low, the regime where the jet stream is weakest and the rudder has the least to deflect), and is **unidirectional** (jets don't reverse) — model it as `thrust_i ≥ 0`; expect yaw-by-thrust to perturb forward speed (§5.5). Decide priority (hold-heading vs hold-speed) explicitly.
The differential component is made honest by §5.5 before it reaches the motors.

### 5.5 Actuator linearisation (the asymmetric-ESC fix)
Solve it **entirely below the controller** so neither human nor ML ever sees it — same pattern as `WINCH_DEADBAND_US`:
- **Per-motor spin-up floor:** remap each motor's `(0,1]` command so the smallest non-zero value lands just above that motor's real spin-up point (left floor ≈ 0.05, right ≈ 0.20, both **measured**). Zero still = off.
- **Optional upper-curve match:** measure a rough thrust-vs-command curve per motor and invert it, so equal commands give equal thrust across the band. (Start with just the floor; add the curve only if straight-line still pulls.)
- Result: `+0.1` yaw produces the *same* turn on either motor. Only after this is the ESC pair usable for steering or straight-line trim.
- **Preserve all hardware quirks:** GPIO31 = 40 mA; **do not** raise GPIO32/34 drive (bus coupling); keep `gpio_hold_dis` / arm-at-min / inverted pulse.

---

## 6. Real-time & scheduling robustness (the "it can stall on core 1" concern)

The control loop is on core 0 at top priority, so **core-1 load (ML/camera/WS/SD) cannot preempt it**. The residual risks are (a) *depending on* core-0 sensor tasks that can be late, and (b) *blocking on a lock* a core-1 task holds (SD↔WiFi mutex, I²C). Design so neither can hurt stability:

1. **Measured, clamped `dt` everywhere.** Integral, derivative, rate-limits, slew-limits all scale by the *real* elapsed time (already done in the dry-run). Clamp `dt` to a sane ceiling (~3× nominal) and *skip* the integral/derivative update on an absurd `dt`, so a scheduling hiccup never injects a huge step. Tolerate skipped ticks — a late tick is just a bigger (clamped) `dt`.
2. **Never block in the control tick.** No SD, no network, no `malloc`, no heavy logging inline. Telemetry (`CTRL_*`) goes over a queue to a **core-1, low-priority** consumer (TrainingLog already exists there) so the SDMMC/WiFi shared-mutex latency stays *off* the control path.
3. **Read sensors from a lock-free snapshot.** Publish `{heading, gyroZ, speed, timestamp, mag_ok}` into a double-buffered struct (generation counter — the pattern `s_status_buffers` already uses). The control loop reads the latest without holding a mutex a lower-priority producer could stall on (avoids priority inversion).
4. **Staleness gates.** Every input carries a timestamp. Heading stale or `!mag_ok` → drop the outer loop, keep inner gyro damping. Gyro/fusion stale beyond a threshold → hold last slew-limited output (no lurch) and reduce authority; badly stale → treat as sensor-loss and fall to the conservative state. Nothing ever acts on frozen data believing it's fresh.
5. **Decouple rates.** Stabiliser thinks at ~20 Hz (measured dt); the 100 Hz loop re-emits the last target through the slew limiter every tick. A late stabiliser tick still yields smooth motion.
6. **Decouple the demand producer (this is the real ML-stall answer).** The ML/waypoint task on core 1 feeds the loop **only through the timestamped, timeout-guarded arbiter queue** — never by calling actuators. A stalled or bursty ML just produces a *stale demand*, bounded by the existing 400 ms freshness timeout → hold/failsafe. **A slow model can never stall the boat's stability**, because stability lives in the always-on core-0 inner loop, not in the planner.
7. **Health signal already exists.** Keep feeding `RUNTIME_EVENT_DEADLINE_MISS`; the control task already resets its schedule on a miss. Watch it as the loop-health metric during tuning.

**Timing budget:** the actuation path is a few dozen float ops + 2–3 short MCPWM register writes (µs). Compute is never the risk on a 400 MHz core-0 slot; the risks above are all *coupling/jitter*, and every one is mitigated by measured-dt + non-blocking + snapshot + slew.

---

## 7. Safety integration & degradation ladder

The SAS lives *inside* the existing arm/failsafe model. It actuates only when **ARMED and control-link-alive**. Degradation, worst-case-safe:

| Condition | Behaviour |
|---|---|
| Normal | Full SAS: heading-hold + rate command + allocation. |
| Mag standby / heading stale | Drop outer loop; **keep gyro rate-damping** — boat stays stable and steerable, just no absolute-heading lock. |
| Fusion/gyro stale | Hold last slew-limited output, reduce authority; no lurch. |
| Very stale / sensor loss | Conservative: centre rudder, thrust to common-mode only. |
| Control-link loss (400 ms) | Existing failsafe: throttle 0, rudder centred, rail cut; `heading_assist_reset()`. |
| Disarm / PWR-OFF | Existing behaviour unchanged. |

**Manual override is guaranteed by construction:** the pilot *is* the setpoint, so the stabiliser never fights the stick; and a full-scale manual demand still commands directly. Mag-freeze robustness is the headline benefit of the inner/outer split.

---

## 8. Manual → Autopilot evolution

Because Intent is one interface, autopilot is *additive*:
1. **GPS waypoint follower (R2):** line-of-sight guidance → desired heading → the **outer loop already converts heading error to `r_cmd`**. Enable `SOURCE_WAYPOINT`; add a core-1 `RUNTIME_TASK_WAYPOINT` producer (slot reserved).
2. **ML obstacle avoidance (R3):** Detect (perception, core 1) → `ML_CONTROL` planner (core 1, slot reserved) → emits `r_cmd + speed` → same inner loop. The stabilised hull is the training/deployment platform.
3. **True no-drift-over-ground:** heading-hold holds *heading*, not ground track — current/wind still crab the hull. Swap/augment the outer-loop reference with **GPS course-over-ground** once heading-hold is solid. Same controller, better reference.
4. **Manual/Auto mode manager:** explicit arbitration (manual pre-empts auto), with the arm/failsafe guarantees intact.

---

## 9. Staged implementation plan (bench-gated, dry-run first)

| Phase | What | Gate |
|---|---|---|
| **0. Enable + measure the plant** | **Publish `gz_rate`+timestamp from fusion** (unblocks the inner loop); ~~confirm rudder location~~ **done — jet-efflux, §2.0**; measure per-motor spin-up floor + rough thrust curve; log heading-vs-throttle (mag distortion); observe yaw step-response (bandwidth → loop rate). | Yaw-rate live in `CTRL_*`; thresholds + mag/throttle + bandwidth known. |
| **1. Actuator linearisation** | Per-motor dead-band/curve comp in `throttle_to_us` (winch pattern). | Equal command → equal thrust; straight-line coast on bench/tub. |
| **2. Allocation + shaping (dry-run)** | `u_yaw → rudder + throttle-scheduled diff`; reference/output shaping; publish snapshot; extend `CTRL_DRY` logs. **Still logs-only.** | Logs show sane rudder/diff split vs throttle & error. |
| **3. Inner loop live (SAS)** | Actuate **yaw-rate damping only** (gyro); rudder = rate demand. Promote from dry-run for the inner loop. | On-water: boat feels planted, damped; no oscillation; manual override crisp. |
| **4. Outer loop live** | Add heading-hold (PD + bounded trim, mag-gated); auto-straighten on centre. | On-water: holds heading hands-off; rejects a nudge; degrades cleanly on mag-freeze. |
| **5. System-ID + optional LQI / speed-schedule** | Zig-zag → fit Nomoto `K`,`T`; schedule gains on GPS speed; optional LQI gains. | Measurable tracking improvement; only if wanted. |
| **6. Autopilot producers** | Enable `SOURCE_WAYPOINT` / `SOURCE_ML`; core-1 producer tasks; GPS-COG reference; mode manager. | R2 waypoint ≤ 2 m; R3 avoidance ≥ 1.5 m. |

Each phase is independently valuable and reversible.

**Getting to water fast (the point that matters): the run IS the measurement.** The fastest *safe* path is the **A→B→C slice**, and it goes on the water early — tethered:
- **A — Publish yaw-rate** (`gz_rate` + timestamp) from the fusion task into the snapshot the control loop reads. Small, no actuation. Unblocks everything.
- **B — ESC dead-band linearisation** (`throttle_to_us` per-motor floor + rough curve, winch pattern). Now equal command ≈ equal thrust; straight ≈ straight.
- **C — One capped live loop: an inner yaw-rate damper on the rudder**, behind a `Kconfig` flag, inside the existing ARM + 400 ms failsafe, authority caps low, `CTRL_*` logging ON. Pilot rudder = rate demand; centre = damp to zero yaw-rate (no mag/heading yet).
Put A→B→C on the water tethered with a hard kill in hand: that single run directly exercises Concerns 1–5 (rate-signal quality, dead-band fix, yaw bandwidth, surge/yaw coupling, one-sided thrust). **Heading-hold (outer loop) and gain tuning come after**, once heading-vs-throttle is characterised. Dry-run logging stays ON *alongside* the live loop — it is a data source, not a gate that delays water.

---

## 10. Parameters to measure / open questions
- Per-motor spin-up floors and thrust curves (Phase 0).
- `R_MAX` (yaw-rate the stick commands) — feel-tuned.
- `λ(v)` blend curve + speed floor where rudder becomes effective.
- Inner-loop `Kr`, `Kd`, LPF cutoff; outer-loop `Kψ` and `r_hold` clamp; trim rate/clamp (start from current dry-run values).
- Stabiliser rate (20 Hz proposed) and `dt` clamp ceiling.
- Telemetry-offload path (reuse TrainingLog vs new core-1 consumer).
- Manual↔auto arbitration policy (Phase 6).

## 11. Requirements traceability & sources
- **R2** (GPS waypoint ≤ 2 m, no continuous human control): Phase 6 waypoint producer + GPS-COG reference on the same outer loop.
- **R3** (obstacle detection/avoidance ≥ 1.5 m, edge inference): stabilised platform + ML producer on core 1.

**Sources:** Nomoto model & PID heading control (ScienceDirect S0029801813002916; ResearchGate 277497099); Fuzzy-LQR USV heading (ResearchGate 317385848); rudderless twin-thruster USV (PMC6539673); SAS / rate-command attitude-hold (Miltech MH-60 AFCS/SAS docs); ESC dead-band (KDE Direct); setpoint slew shaping (PX4 MC slew-rate trajectory); shaper-vs-rate-limiter interaction (US Patent 7,970,521).
