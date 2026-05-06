# ESC Motor Control Subsystem — Design Spec

**Date:** 2026-05-06
**Status:** Approved
**Hardware:** 2x XF Model-40A 40A brushless ESC, bidirectional mode
**Pins:** Left=GPIO31, Right=GPIO33 (via GPIO matrix)

## Requirements

- Bidirectional operation: 1000us (full reverse) — 1500us (neutral) — 2000us (full forward)
- Hybrid arming: auto-arm after WiFi connects at boot, dashboard can disarm/re-arm anytime
- Immediate motor stop when WebSocket connection drops (0 connected clients)
- Dashboard: direct left/right motor sliders with visual feedback
- Proto: keep throttle/rudder fields for future autopilot, add left/right for direct control
- Follow existing architecture: `drivers/` → logic layer → pipeline → dashboard

## Layer Architecture

```
dashboard.html          ← L/R sliders, arm button, status display
    ↕ protobuf over WebSocket
pipeline.c/h            ← publish_motor_status(), arm handler dispatch
    ↕ function calls
motor_control.c/h       ← state machine, input resolution, WS-drop watchdog
    ↕ function calls
drivers/esc_driver.c/h  ← MCPWM hardware: init, arm, set pulse, disarm
```

Separation: driver knows MCPWM only. Logic layer knows floats and state only. Swapping ESC hardware means changing only `esc_driver.c`.

## Files to Create

| File | Layer | Purpose |
|---|---|---|
| `main/drivers/esc_driver.h` | Driver | Public API: init, arm, disarm, set_throttle, get_state |
| `main/drivers/esc_driver.c` | Driver | MCPWM configuration, pulse mapping, state tracking |
| `main/motor_control.h` | Logic | Public API: init, arm, disarm |
| `main/motor_control.c` | Logic | Pipeline bridge, input resolution, watchdog, status publish |

## Files to Modify

| File | Changes |
|---|---|
| `main/proto/boat.proto` | Add left/right to MotorCommand, add MotorStatus, add ArmCommand, update BoatMessage |
| `main/proto/boat.pb.h` | Hand-edit: new structs, tags, fieldlists, size constants |
| `main/proto/boat.pb.c` | Hand-edit: new PB_BIND entries |
| `main/pipeline.h` | Add pipeline_publish_motor_status(), arm handler type + registration |
| `main/pipeline.c` | Implement publish_motor_status(), add arm_cmd dispatch case |
| `main/main.c` | Add motor_control include, init after NVS, arm after WiFi |
| `main/CMakeLists.txt` | Add esc_driver.c, motor_control.c to SRCS; add esp_driver_mcpwm to PRIV_REQUIRES |
| `main/dashboard.html` | Motor card CSS, HTML, JS; update inline proto schema; grid layout |

## ESC Driver Design

### MCPWM Resources

- Group 0 (unused by any existing code)
- Timer: 1MHz resolution (1us/tick), 20000-tick period = 50Hz
- One operator, two comparators (left/right), two generators
- Generator action: HIGH on timer empty, LOW on comparator match

### State Machine

```
DISARMED ──arm()──→ ARMING (hold neutral 3s) ──→ ARMED
    ↑                                               │
    └──────────────disarm()─────────────────────────┘
```

No ERROR state. RC PWM is one-way — no feedback wire to detect ESC faults.

### Pulse Mapping

Linear interpolation, clamped to [-1.0, 1.0]:

```
-1.0 → CONFIG_ESC_PULSE_MIN_US     (1000us, full reverse)
 0.0 → CONFIG_ESC_PULSE_NEUTRAL_US (1500us, dead band)
+1.0 → CONFIG_ESC_PULSE_MAX_US     (2000us, full forward)
```

XF 40A dead band is ~50us around neutral. Small float values near 0.0 naturally fall within it.

### Arming Sequence (XF Model-40A, Bidirectional)

1. `esc_driver_init()` — configure MCPWM, output 1500us immediately. Must run before ESCs see power.
2. ESC powers on, sees neutral → beep sequence (cell count, then armed tone), takes 2-4s.
3. `esc_driver_arm()` — hold neutral for 3s (blocking), then state = ARMED.
4. `esc_driver_set_throttle()` — only works in ARMED state.

### API

```c
esp_err_t   esc_driver_init(void);
esp_err_t   esc_driver_arm(void);
esp_err_t   esc_driver_set_throttle(float left, float right);
esp_err_t   esc_driver_disarm(void);
esc_state_t esc_driver_get_state(void);
void        esc_driver_get_throttle(float *left, float *right);
```

## Motor Control Logic Design

### Input Resolution

The dashboard always sends `left`+`right` fields. Future autopilot sends `throttle`+`rudder`. Since proto3 zero-values are indistinguishable from "not set", the logic layer uses a simple priority rule:

```
If MotorCommand.left != 0 OR MotorCommand.right != 0:
    → direct mode: pass left, right to esc_driver_set_throttle()
Else:
    → mixing mode: left = throttle + rudder, right = throttle - rudder
    → clamp with proportional scaling (preserve turn ratio)
```

Edge case: sending left=0, right=0 via direct mode is equivalent to sending throttle=0, rudder=0 via mixing mode — both produce (0.0, 0.0) — so the ambiguity is harmless. A dashboard "stop" command (both sliders at center) works correctly in either path.

### WS-Drop Watchdog

- `esp_timer` periodic callback at 250ms
- Check `ws_transport_client_count()` — if 0 clients AND armed AND last throttle non-zero → set both motors to 0.0
- Reacts to connection state, not command timeout

### Motor Status Publishing

- Every 4th watchdog tick (~1Hz)
- Publishes `MotorStatus { state, left_throttle, right_throttle }` through pipeline
- Dashboard updates arm indicator and bar gauges

### Boot Integration

1. `motor_control_init()` — right after NVS init, before camera/sensor init. Starts neutral PWM.
2. `motor_control_arm()` — after WiFi + HTTP server start. Blocking ~3s.

## Wire Protocol Changes

### Modified: MotorCommand

```proto
message MotorCommand {
  float throttle = 1;  // -1.0 to 1.0 (autopilot mode)
  float rudder   = 2;  // -1.0 to 1.0 (autopilot mode)
  float left     = 3;  // -1.0 to 1.0 (direct mode)
  float right    = 4;  // -1.0 to 1.0 (direct mode)
}
```

Backward compatible — existing tag numbers unchanged.

### New: MotorStatus

```proto
message MotorStatus {
  uint32 state           = 1;  // 0=DISARMED, 1=ARMING, 2=ARMED
  float  left_throttle   = 2;
  float  right_throttle  = 3;
}
```

### New: ArmCommand

```proto
message ArmCommand {
  bool arm = 1;  // true=arm, false=disarm
}
```

### Updated: BoatMessage

```proto
message BoatMessage {
  oneof payload {
    SensorSnapshot sensors       = 1;
    MotorCommand   motor         = 2;
    SystemStatus   status        = 3;
    DetectCommand  detect        = 4;
    GpsCoordinate  coord         = 5;
    MotorStatus    motor_status  = 6;
    ArmCommand     arm_cmd       = 7;
  }
}
```

## Dashboard Design

### Motor Card

Position: right column, row 3 (below ToF card). Camera card expands to span all 3 rows.

Components:
- State indicator: colored dot (grey=disarmed, yellow+pulse=arming, green+glow=armed) + text label
- ARM/DISARM toggle button: green border for arm, red border for disarm
- Two range sliders (-100% to +100%): one per motor, disabled when not armed, double-click to reset
- Two vertical bar gauges: driven by MotorStatus feedback, green=forward, red=reverse

### Behavior

- Sliders send `MotorCommand { left, right }` at ~10Hz while dragging (setInterval, not per input event)
- Final send on slider release, then interval clears
- ARM button sends `ArmCommand { arm: !currentState }` over WS
- Incoming `MotorStatus` updates dot, label, button, and bar gauges
- Inline proto schema string updated to include new messages and fields
