# Yaw-Rate PI and Heading-Hold Design

**Date:** 2026-09-18

## Problem

The current motor P-assist adds a filtered proportional correction to the
learned differential-thrust coefficient `c`. It cannot remove a persistent
disturbance, its correction was historically limited to `0.075` of `c`, and
recorded firmware data showed that the old implementation stopped correcting
when yaw exceeded 10 deg/s. At T40, a `0.075` correction changes each motor by
only three throttle percentage points. Afternoon runs also showed that equal or
weakly split motor commands can produce sustained right yaw greater than
10 deg/s, while a larger right-motor command can bring the average yaw back
toward zero.

The boat therefore needs a controller that reacts immediately to yaw rate,
builds enough steady correction to overcome a persistent motor or environmental
imbalance, and returns the boat to the direction captured when straight travel
began. The controller must respect the actual throttle headroom and must never
fight a deliberate manual turn.

## Measured Inputs and Conventions

- Positive gyro yaw and increasing fused heading mean a left turn. Negative
  yaw and decreasing heading mean a right turn.
- Increasing `c` gives the right motor more command and the left motor less:
  `left = throttle * (1 - c)` and `right = throttle * (1 + c)`.
- The measured plant is approximately 16.1 deg/s of yaw per unit of `c`, with
  a 1.10 s time constant and about 0.3 s of dead time.
- In the newest 40 BASE recordings, fused-heading sample changes had a median
  magnitude of 0.062 degrees and a 95th percentile of 0.195 degrees. Heading is
  suitable for a slower outer loop. Gyro yaw rate remains the input to the fast
  inner loop.
- Both recalibrated ESCs begin driving at 1050 us. The existing five-percent
  left and right minimum-throttle configuration remains in force.

## Goals

1. Start a correctly signed motor correction within two fresh fusion samples
   after unwanted yaw begins, including yaw magnitudes greater than 10 deg/s.
2. Use integral action to cancel persistent mismatch during a long straight
   run rather than depending on mechanical motor symmetry.
3. Capture and hold the direction of travel, so a period of nonzero yaw does
   not leave the boat travelling on a permanently changed heading.
4. Use all safe differential-thrust authority available at the current common
   throttle while preserving the requested average thrust.
5. Suspend correction during deliberate steering and recapture the new heading
   after the steering control is centred.
6. Record enough internal controller data to distinguish P response, I
   response, heading demand, and output saturation in both 3-second and
   30-second Lake-ID BASE tests.

## Controller Architecture

The implementation adds a pure, ESP-independent `yaw_heading_control` module.
`motor_control.c` owns one instance, supplies fresh sensor and command inputs,
and applies its requested differential coefficient through the existing motor
mixer. The module has two cascaded loops.

The outer heading loop captures the current fused heading as its target. It
computes the shortest signed error:

```text
heading_error = wrap_180(target_heading - measured_heading)
yaw_target = clamp(heading_kp * filtered_heading_error,
                   -max_yaw_target, +max_yaw_target)
```

The inner yaw-rate PI loop computes:

```text
rate_error = yaw_target - measured_yaw_rate
p = rate_kp * rate_error
i = anti_windup_integral(i, rate_ki * rate_error * dt)
dynamic_c = p + i
effective_c = clamp(feedforward_c + dynamic_c, -c_limit, +c_limit)
```

The sign follows directly from the measured boat convention: a right turn has
negative yaw, which creates positive rate error and positive `dynamic_c`, giving
the right motor more command.

The initial configuration is:

| Parameter | Value | Purpose |
| --- | ---: | --- |
| gyro low-pass time constant | 0.15 s | Reject gyro noise without hiding the first part of a 3-second event |
| rate P gain | 0.050 `c`/(deg/s) | Immediate strong correction; plant loop gain is about 0.81 |
| rate I gain | 0.020 `c`/(deg/s*s) | Cancel persistent mismatch over seconds |
| heading-error low-pass time constant | 0.35 s | Keep compass jitter out of motor commands |
| heading P gain | 0.80 (deg/s)/deg | Command a return rate proportional to accumulated heading error |
| maximum heading-requested yaw rate | 8.0 deg/s | Bound the outer loop while allowing a decisive return |
| minimum active throttle | 0.15 | Match the existing learner's trustworthy operating floor |
| centred steering threshold | 0.02 | Preserve the existing motor-assist steering gate |
| centred capture delay after a manual turn | 0.50 s | Capture the operator's new course after the turn ends |

These values are deliberately explicit so the first flash is reproducible.
They are Kconfig values with defensive parse bounds. Tuning changes require new
recorded evidence; code must not contain a second set of operational defaults.

## Differential-Thrust Authority and Anti-Windup

The fixed `0.075` correction ceiling is removed. For common throttle `T`, the
largest symmetric differential coefficient that preserves average thrust and
keeps both normalized motor commands in `[0, 1]` is:

```text
c_limit = min(1.0, (1.0 / T) - 1.0)
```

At T40 this permits `c` from -1.0 to +1.0, corresponding at the endpoints to
one motor at zero and the other at 0.8. At higher common throttle the bound
shrinks automatically because the high motor approaches 1.0. The final motor
commands are clamped defensively to `[0, 1]` after mixing.

The PI output bounds are computed relative to the current feed-forward trim:

```text
dynamic_min = -c_limit - feedforward_c
dynamic_max = +c_limit - feedforward_c
```

The integrator uses conditional integration. It may integrate while the
tentative output is inside these bounds, or while the current rate error would
move a saturated output back toward the available range. After every update,
the integral is clamped to the portion of the current dynamic range remaining
after the P term. This prevents stored correction that the motors cannot apply
and makes a throttle increase or decrease safe.

Saturation is an expected operating state and is logged. It does not disable
the controller. Physical saturation remains the hard limit: when one motor is
at zero or the other is at full command, software has no additional yaw
authority.

## Feed-Forward Trim and the Existing Learner

The current learned `c` remains the starting feed-forward balance. It is not
forced back to zero and is not discarded. While the new PI controller is
active, the slow `trim_learn` integrator is frozen so two integrators do not
learn the same error and fight each other. The new PI integral is temporary and
is never written to NVS.

When motor assist is disabled, the existing slow learner retains its current
behaviour. This keeps old A/B experiments possible. The runtime P-assist switch
becomes the runtime enable for the complete yaw-rate PI and heading-hold motor
controller; protocol compatibility retains the existing field name until a
separate protocol migration is justified.

## Activation and State Transitions

The controller is active only when all of the following are true:

- motor assist is enabled;
- the boat is armed and in normal drive or a BASE/BASE-LONG bench run;
- common throttle is at least 0.15;
- the steering command magnitude is at most 0.02;
- a new finite gyro sample has arrived and is fresh under the existing fusion
  freshness limit;
- calibration and LEFT/RIGHT bench routines do not own the motors.

On the first active sample, the gyro filter initializes from the measured yaw,
the rate integral starts at zero, and a valid fused heading is captured. The
rate loop therefore acts immediately; it does not wait for a heading error to
accumulate.

During deliberate manual steering, P and I output are removed so the controller
does not fight the turn. The integral value is frozen, preserving the estimated
motor imbalance. As soon as steering is centred, the rate loop resumes with a
zero yaw target. After 500 ms continuously centred, the current heading becomes
the new heading target and heading hold resumes. Any renewed steering resets
the capture timer.

Disarming, stale gyro data, non-finite data, low throttle, calibration, a
LEFT/RIGHT bench run, or an explicit motor-assist disable clears all dynamic
state and integral state. A BASE run captures heading from its first valid
driving sample without the manual-turn delay.

## Heading Validity and Fallback

`FusionResult` gains an internal `heading_valid` indication derived from the
fusion yaw initialization and existing magnetometer acceptance checks. Brief
magnetometer rejection does not invalidate the gyro-propagated fused heading;
heading becomes invalid only before yaw initialization or after the existing
magnetometer/IMU health path declares the heading source unusable.

If heading is invalid but gyro yaw remains fresh, the outer loop is suspended
and `yaw_target` is zero. The rate PI loop continues to suppress turning. When
heading becomes valid again, the controller captures the current heading as a
new target rather than steering toward a stale target.

## Integration with Existing Rudder Control

This controller owns differential thrust only. The existing assisted-rudder
SAS remains separate and its telemetry keeps its current meaning. The old
`heading_assist_dry_run` heuristic must not produce motor output alongside the
new controller. Its log-only implementation may remain for comparison, but the
new controller is the sole source of dynamic motor `c`.

`motor_control.c` applies the resulting `effective_c` before
`esc_driver_set_throttle()`. A new fusion sample or controller-state change
marks the motor output dirty so correction is applied even when the operator's
throttle packet has not changed.

## Lake-ID Tests and Observability

The firmware SD BASE CSV keeps its existing phase, yaw, command, and learned
trim fields and adds:

- measured heading and captured target heading;
- filtered heading error and requested yaw rate;
- filtered yaw rate and rate error;
- P term, I term, total dynamic correction, effective `c`, and `c_limit`;
- controller-active, heading-hold-active, and saturation flags.

The laptop BASE CSV continues to record fused heading and applied left/right
commands. The bench-status telemetry is extended with the compact controller
diagnostics needed by the laptop recorder: target heading, heading error,
requested yaw rate, P, I, total correction, effective `c`, limit, active flags,
and saturation. New protobuf fields use new tag numbers; existing tags retain
their meaning. Generated nanopb files and size assertions are updated together.

The Python Lake-ID summary reports net heading change, average and peak yaw,
time active, time saturated, peak absolute P and I, and final I. The existing
3-second BASE profile remains 3 seconds. The laptop-only `BASE TEST 30s`
profile remains 30 seconds. Firmware `BASE_LONG` timing remains unchanged by
this controller work.

## Verification

The pure controller receives deterministic C tests before integration. Tests
cover:

- right and left yaw produce correctly signed differential correction;
- correction remains active above 10 deg/s;
- nonzero P output appears within two fresh samples;
- persistent simulated disturbance grows I and brings average yaw toward zero;
- actuator saturation blocks outward integral growth and permits unwind;
- heading wrap works across 359/0 degrees;
- heading error requests a bounded return yaw rate;
- compass invalidity falls back to zero-yaw-rate control and recaptures safely;
- steering suspends output, freezes I, and recaptures after 500 ms centred;
- throttle-dependent headroom keeps both motor commands in `[0, 1]`;
- stale data, disarm, low throttle, and conflicting bench modes reset state.

Integration tests pin the ordering in `motor_control.c`, learner freeze rules,
BASE activation, telemetry meanings, CSV headers, and Python parsing. A replay
of the recorded `THEPTESTV3/T40_B_13.CSV` verifies that correction does not
disappear during samples below -10 deg/s. A first-order plant simulation uses
the measured 16.1 deg/s gain, 1.10 s time constant, and 0.3 s delay for both a
3-second disturbance test and a 30-second constant-bias test.

Before flashing, the focused C tests, the complete host test suite, protobuf
regeneration checks, and an ESP-IDF firmware build must all pass from the
Lake-ID worktree.

## Acceptance Criteria

1. In the deterministic 3-second simulation, the controller produces the
   correct sign within two fresh samples, never disables because yaw exceeds
   10 deg/s, and reduces absolute final yaw versus feed-forward-only control.
2. In the deterministic 30-second simulation, a constant disturbance is
   rejected by the I term, average yaw over the final 10 seconds is within
   0.5 deg/s of zero, and heading error is returning toward zero without
   sustained saturation.
3. Every applied motor command remains between zero and one, and average motor
   command equals requested common throttle except when final defensive clamps
   are exercised.
4. Manual steering never receives opposing differential correction from this
   controller. Straight control resumes and captures the new course after the
   centred delay.
5. Firmware SD and laptop BASE data expose whether the controller was active,
   what it requested, and whether it hit physical authority limits.

These software criteria establish controller behaviour and wiring. Actual lake
performance is confirmed only by the subsequent 3-second and 30-second boat
tests because wind, current, propeller loading, and hull response cannot be
proven by a host simulation.

## Non-Goals

- No derivative term is added; the gyro-rate measurement already supplies the
  damping signal and a derivative would amplify noise.
- No RPM or thrust feedback is claimed because the hardware has no actuator
  feedback.
- No rudder-controller retuning is included.
- No automatic gain adaptation is included in the first implementation.
- The controller does not promise correction beyond available motor authority.
