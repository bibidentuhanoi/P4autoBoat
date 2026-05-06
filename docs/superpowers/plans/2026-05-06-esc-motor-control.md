# ESC Motor Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add MCPWM-based ESC motor control with dashboard UI to the BoatEspP4 autonomous boat firmware.

**Architecture:** Driver layer (`esc_driver`) handles MCPWM hardware. Logic layer (`motor_control`) bridges pipeline commands to the driver, manages arm state, and implements WS-drop safety. Dashboard gets L/R sliders and an arm button. All connected through the existing protobuf pipeline.

**Tech Stack:** ESP-IDF v5.4 MCPWM API, nanopb protobuf (hand-edited), protobuf.js in dashboard

**Spec:** `docs/superpowers/specs/2026-05-06-esc-motor-control-design.md`

---

### Task 1: Protobuf — Update wire protocol

**Files:**
- Modify: `main/proto/boat.proto`
- Modify: `main/proto/boat.pb.h`
- Modify: `main/proto/boat.pb.c`

This task has no dependencies and can run in parallel with Task 2.

- [ ] **Step 1: Update boat.proto**

In `main/proto/boat.proto`, replace the MotorCommand message and everything after it with:

```proto
message MotorCommand {
  float throttle = 1;  // -1.0 to 1.0 (autopilot mode)
  float rudder   = 2;  // -1.0 to 1.0 (autopilot mode)
  float left     = 3;  // -1.0 to 1.0 (direct mode)
  float right    = 4;  // -1.0 to 1.0 (direct mode)
}

message MotorStatus {
  uint32 state           = 1;  // 0=DISARMED, 1=ARMING, 2=ARMED
  float  left_throttle   = 2;
  float  right_throttle  = 3;
}

message ArmCommand {
  bool arm = 1;  // true=arm, false=disarm
}

message SystemStatus {
  uint32 heap_free    = 1;
  int32  wifi_rssi    = 2;
  uint64 uptime_us    = 3;
}

// Envelope — every message on the wire is a BoatMessage
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

Note: The current working tree has uncommitted GpsFix/GpsCoordinate additions. If they are present in boat.proto, preserve them. If not (reverted), omit GpsCoordinate and coord from the BoatMessage — just add motor_status=6 and arm_cmd=7 after detect=4.

- [ ] **Step 2: Update boat.pb.h — struct definitions**

In `main/proto/boat.pb.h`, add two new fields to `boat_MotorCommand`:

```c
typedef struct _boat_MotorCommand {
    float throttle; /* -1.0 to 1.0 */
    float rudder; /* -1.0 to 1.0 */
    float left; /* -1.0 to 1.0 (direct mode) */
    float right; /* -1.0 to 1.0 (direct mode) */
} boat_MotorCommand;
```

Add two new structs after `boat_MotorCommand`:

```c
typedef struct _boat_MotorStatus {
    uint32_t state; /* 0=DISARMED, 1=ARMING, 2=ARMED */
    float left_throttle;
    float right_throttle;
} boat_MotorStatus;

typedef struct _boat_ArmCommand {
    bool arm; /* true=arm, false=disarm */
} boat_ArmCommand;
```

Add `boat_MotorStatus` and `boat_ArmCommand` to the `boat_BoatMessage` union:

```c
typedef struct _boat_BoatMessage {
    pb_size_t which_payload;
    union {
        boat_SensorSnapshot sensors;
        boat_MotorCommand motor;
        boat_SystemStatus status;
        boat_DetectCommand detect;
        boat_MotorStatus motor_status;
        boat_ArmCommand arm_cmd;
    } payload;
} boat_BoatMessage;
```

(If GpsCoordinate is present in the union, keep it and add motor_status + arm_cmd after it.)

- [ ] **Step 3: Update boat.pb.h — init macros**

Update the MotorCommand init macros (now 4 fields):

```c
#define boat_MotorCommand_init_default           {0, 0, 0, 0}
#define boat_MotorCommand_init_zero              {0, 0, 0, 0}
```

Add new init macros (insert near the other init macros):

```c
#define boat_MotorStatus_init_default            {0, 0, 0}
#define boat_MotorStatus_init_zero               {0, 0, 0}
#define boat_ArmCommand_init_default             {0}
#define boat_ArmCommand_init_zero                {0}
```

- [ ] **Step 4: Update boat.pb.h — field tags**

Add new field tags after the existing MotorCommand tags:

```c
#define boat_MotorCommand_left_tag               3
#define boat_MotorCommand_right_tag              4
#define boat_MotorStatus_state_tag               1
#define boat_MotorStatus_left_throttle_tag       2
#define boat_MotorStatus_right_throttle_tag      3
#define boat_ArmCommand_arm_tag                  1
#define boat_BoatMessage_motor_status_tag        6
#define boat_BoatMessage_arm_cmd_tag             7
```

(If coord_tag=5 exists, keep it. motor_status is always 6, arm_cmd always 7.)

- [ ] **Step 5: Update boat.pb.h — fieldlists**

Update `boat_MotorCommand_FIELDLIST` to include left and right:

```c
#define boat_MotorCommand_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    throttle,          1) \
X(a, STATIC,   SINGULAR, FLOAT,    rudder,            2) \
X(a, STATIC,   SINGULAR, FLOAT,    left,              3) \
X(a, STATIC,   SINGULAR, FLOAT,    right,             4)
#define boat_MotorCommand_CALLBACK NULL
#define boat_MotorCommand_DEFAULT NULL
```

Add new fieldlists after `boat_MotorCommand_FIELDLIST`:

```c
#define boat_MotorStatus_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   state,             1) \
X(a, STATIC,   SINGULAR, FLOAT,    left_throttle,     2) \
X(a, STATIC,   SINGULAR, FLOAT,    right_throttle,    3)
#define boat_MotorStatus_CALLBACK NULL
#define boat_MotorStatus_DEFAULT NULL

#define boat_ArmCommand_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     arm,               1)
#define boat_ArmCommand_CALLBACK NULL
#define boat_ArmCommand_DEFAULT NULL
```

Add to `boat_BoatMessage_FIELDLIST` (after the last existing ONEOF line):

```c
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,motor_status,payload.motor_status),   6) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,arm_cmd,payload.arm_cmd),   7)
```

Add MSGTYPE definitions:

```c
#define boat_BoatMessage_payload_motor_status_MSGTYPE boat_MotorStatus
#define boat_BoatMessage_payload_arm_cmd_MSGTYPE boat_ArmCommand
```

- [ ] **Step 6: Update boat.pb.h — extern declarations and size constants**

Add extern declarations:

```c
extern const pb_msgdesc_t boat_MotorStatus_msg;
extern const pb_msgdesc_t boat_ArmCommand_msg;
```

Add fields compatibility macros:

```c
#define boat_MotorStatus_fields &boat_MotorStatus_msg
#define boat_ArmCommand_fields &boat_ArmCommand_msg
```

Add size constants:

```c
#define boat_ArmCommand_size                     2
#define boat_MotorStatus_size                    17
```

Update `boat_MotorCommand_size` from 10 to 20 (added 2 floats = 10 more bytes max).

- [ ] **Step 7: Update boat.pb.c**

Add two new PB_BIND entries after `boat_MotorCommand`:

```c
PB_BIND(boat_MotorStatus, boat_MotorStatus, AUTO)


PB_BIND(boat_ArmCommand, boat_ArmCommand, AUTO)
```

- [ ] **Step 8: Commit**

```bash
git add main/proto/boat.proto main/proto/boat.pb.h main/proto/boat.pb.c
git commit -m "proto: add MotorStatus, ArmCommand, MotorCommand left/right fields"
```

---

### Task 2: ESC Driver — MCPWM hardware layer

**Files:**
- Create: `main/drivers/esc_driver.h`
- Create: `main/drivers/esc_driver.c`

This task has no dependencies and can run in parallel with Task 1.

- [ ] **Step 1: Create esc_driver.h**

Create `main/drivers/esc_driver.h`:

```c
#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESC_STATE_DISARMED = 0,
    ESC_STATE_ARMING   = 1,
    ESC_STATE_ARMED    = 2,
} esc_state_t;

esp_err_t   esc_driver_init(void);
esp_err_t   esc_driver_arm(void);
esp_err_t   esc_driver_set_throttle(float left, float right);
esp_err_t   esc_driver_disarm(void);
esc_state_t esc_driver_get_state(void);
void        esc_driver_get_throttle(float *left, float *right);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Create esc_driver.c**

Create `main/drivers/esc_driver.c`:

```c
#include "esc_driver.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "ESC_DRV";

#define TIMER_RESOLUTION_HZ  1000000
#define TIMER_PERIOD_TICKS   (1000000 / CONFIG_ESC_PWM_FREQ_HZ)

static mcpwm_timer_handle_t s_timer     = NULL;
static mcpwm_cmpr_handle_t  s_cmp_left  = NULL;
static mcpwm_cmpr_handle_t  s_cmp_right = NULL;

static esc_state_t s_state       = ESC_STATE_DISARMED;
static float       s_throttle_left  = 0.0f;
static float       s_throttle_right = 0.0f;

static uint32_t throttle_to_us(float t)
{
    if (t < -1.0f) t = -1.0f;
    if (t >  1.0f) t =  1.0f;
    uint32_t neutral = CONFIG_ESC_PULSE_NEUTRAL_US;
    if (t >= 0.0f) {
        return neutral + (uint32_t)(t * (float)(CONFIG_ESC_PULSE_MAX_US - neutral));
    }
    return neutral - (uint32_t)((-t) * (float)(neutral - CONFIG_ESC_PULSE_MIN_US));
}

esp_err_t esc_driver_init(void)
{
    if (s_timer) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMER_RESOLUTION_HZ,
        .period_ticks  = TIMER_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &s_timer), TAG, "timer");

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &oper), TAG, "operator");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(oper, s_timer), TAG, "connect");

    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &cmp_cfg, &s_cmp_left), TAG, "cmp_l");
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &cmp_cfg, &s_cmp_right), TAG, "cmp_r");

    mcpwm_gen_handle_t gen_left = NULL, gen_right = NULL;
    mcpwm_generator_config_t gen_cfg_l = { .gen_gpio_num = CONFIG_ESC_PWM_LEFT_PIN };
    mcpwm_generator_config_t gen_cfg_r = { .gen_gpio_num = CONFIG_ESC_PWM_RIGHT_PIN };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &gen_cfg_l, &gen_left), TAG, "gen_l");
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &gen_cfg_r, &gen_right), TAG, "gen_r");

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(gen_left,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
        TAG, "act_l_timer");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(gen_left,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_cmp_left, MCPWM_GEN_ACTION_LOW)),
        TAG, "act_l_cmp");

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(gen_right,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
        TAG, "act_r_timer");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(gen_right,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_cmp_right, MCPWM_GEN_ACTION_LOW)),
        TAG, "act_r_cmp");

    mcpwm_comparator_set_compare_value(s_cmp_left,  CONFIG_ESC_PULSE_NEUTRAL_US);
    mcpwm_comparator_set_compare_value(s_cmp_right, CONFIG_ESC_PULSE_NEUTRAL_US);

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), TAG, "enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP), TAG, "start");

    s_state = ESC_STATE_DISARMED;
    ESP_LOGI(TAG, "ESC PWM active (neutral %u us, %d Hz) L=GPIO%d R=GPIO%d",
             CONFIG_ESC_PULSE_NEUTRAL_US, CONFIG_ESC_PWM_FREQ_HZ,
             CONFIG_ESC_PWM_LEFT_PIN, CONFIG_ESC_PWM_RIGHT_PIN);
    return ESP_OK;
}

esp_err_t esc_driver_arm(void)
{
    if (s_state != ESC_STATE_DISARMED) {
        ESP_LOGW(TAG, "Cannot arm from state %d", (int)s_state);
        return ESP_ERR_INVALID_STATE;
    }

    s_state = ESC_STATE_ARMING;
    ESP_LOGI(TAG, "Arming — holding neutral for 3 s...");

    mcpwm_comparator_set_compare_value(s_cmp_left,  CONFIG_ESC_PULSE_NEUTRAL_US);
    mcpwm_comparator_set_compare_value(s_cmp_right, CONFIG_ESC_PULSE_NEUTRAL_US);
    vTaskDelay(pdMS_TO_TICKS(3000));

    s_state = ESC_STATE_ARMED;
    ESP_LOGI(TAG, "Armed");
    return ESP_OK;
}

esp_err_t esc_driver_set_throttle(float left, float right)
{
    if (s_state != ESC_STATE_ARMED) return ESP_ERR_INVALID_STATE;

    mcpwm_comparator_set_compare_value(s_cmp_left,  throttle_to_us(left));
    mcpwm_comparator_set_compare_value(s_cmp_right, throttle_to_us(right));
    s_throttle_left  = left;
    s_throttle_right = right;
    return ESP_OK;
}

esp_err_t esc_driver_disarm(void)
{
    mcpwm_comparator_set_compare_value(s_cmp_left,  CONFIG_ESC_PULSE_NEUTRAL_US);
    mcpwm_comparator_set_compare_value(s_cmp_right, CONFIG_ESC_PULSE_NEUTRAL_US);
    s_throttle_left  = 0.0f;
    s_throttle_right = 0.0f;
    s_state = ESC_STATE_DISARMED;
    ESP_LOGI(TAG, "Disarmed");
    return ESP_OK;
}

esc_state_t esc_driver_get_state(void)       { return s_state; }

void esc_driver_get_throttle(float *left, float *right)
{
    if (left)  *left  = s_throttle_left;
    if (right) *right = s_throttle_right;
}
```

- [ ] **Step 3: Commit**

```bash
git add main/drivers/esc_driver.h main/drivers/esc_driver.c
git commit -m "feat: add MCPWM ESC driver for XF Model-40A"
```

---

### Task 3: Motor Control — Logic layer

**Files:**
- Create: `main/motor_control.h`
- Create: `main/motor_control.c`

Depends on: Task 1 (proto types), Task 2 (esc_driver API)

- [ ] **Step 1: Create motor_control.h**

Create `main/motor_control.h`:

```c
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_control_init(void);
esp_err_t motor_control_arm(void);
esp_err_t motor_control_disarm(void);

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Create motor_control.c**

Create `main/motor_control.c`:

```c
#include "motor_control.h"
#include "drivers/esc_driver.h"
#include "pipeline.h"
#include "transports/ws_transport.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "MOTOR_CTL";

#define WATCHDOG_INTERVAL_US (250 * 1000)
#define STATUS_DIVIDER       4

static bool s_was_nonzero = false;
static esp_timer_handle_t s_watchdog = NULL;

static void motor_command_handler(const boat_MotorCommand *cmd)
{
    float left, right;

    if (cmd->left != 0.0f || cmd->right != 0.0f) {
        left  = cmd->left;
        right = cmd->right;
    } else {
        float throttle = cmd->throttle;
        float rudder   = cmd->rudder;
        left  = throttle + rudder;
        right = throttle - rudder;
        float max_abs = fmaxf(fabsf(left), fabsf(right));
        if (max_abs > 1.0f) {
            left  /= max_abs;
            right /= max_abs;
        }
    }

    s_was_nonzero = (left != 0.0f || right != 0.0f);
    esc_driver_set_throttle(left, right);
}

static void arm_command_handler(bool arm)
{
    if (arm) {
        ESP_LOGI(TAG, "Arm command received");
        if (esc_driver_get_state() == ESC_STATE_DISARMED) {
            motor_control_arm();
        }
    } else {
        ESP_LOGI(TAG, "Disarm command received");
        motor_control_disarm();
    }
}

static void publish_status(void)
{
    boat_MotorStatus ms = boat_MotorStatus_init_zero;
    ms.state = (uint32_t)esc_driver_get_state();
    esc_driver_get_throttle(&ms.left_throttle, &ms.right_throttle);
    pipeline_publish_motor_status(&ms);
}

static void watchdog_cb(void *arg)
{
    (void)arg;
    static int tick = 0;

    if (esc_driver_get_state() == ESC_STATE_ARMED && s_was_nonzero) {
        if (ws_transport_client_count() == 0) {
            ESP_LOGW(TAG, "WS disconnected — stopping motors");
            esc_driver_set_throttle(0.0f, 0.0f);
            s_was_nonzero = false;
        }
    }

    if (++tick % STATUS_DIVIDER == 0) {
        publish_status();
    }
}

esp_err_t motor_control_init(void)
{
    esp_err_t ret = esc_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESC driver init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    pipeline_register_motor_handler(motor_command_handler);
    pipeline_register_arm_handler(arm_command_handler);

    const esp_timer_create_args_t timer_args = {
        .callback = watchdog_cb,
        .name     = "motor_wd",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_watchdog), TAG, "timer_create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_watchdog, WATCHDOG_INTERVAL_US), TAG, "timer_start");

    ESP_LOGI(TAG, "Motor control initialized (status ~1 Hz)");
    return ESP_OK;
}

esp_err_t motor_control_arm(void)
{
    return esc_driver_arm();
}

esp_err_t motor_control_disarm(void)
{
    s_was_nonzero = false;
    return esc_driver_disarm();
}
```

- [ ] **Step 3: Commit**

```bash
git add main/motor_control.h main/motor_control.c
git commit -m "feat: add motor control logic layer with WS-drop watchdog"
```

---

### Task 4: Pipeline Integration

**Files:**
- Modify: `main/pipeline.h`
- Modify: `main/pipeline.c`

Depends on: Task 1 (proto types)

- [ ] **Step 1: Update pipeline.h**

Add after the existing `pipeline_register_motor_handler` declaration (after line 48):

```c
void pipeline_publish_motor_status(const boat_MotorStatus *mstatus);

typedef void (*arm_command_handler_fn)(bool arm);

void pipeline_register_arm_handler(arm_command_handler_fn handler);
```

- [ ] **Step 2: Update pipeline.c — add arm handler storage**

After the line `static motor_command_handler_fn s_motor_handler = NULL;` (line 22), add:

```c
static arm_command_handler_fn s_arm_handler = NULL;
```

In `pipeline_init()`, after `s_motor_handler = NULL;` (line 29), add:

```c
    s_arm_handler = NULL;
```

- [ ] **Step 3: Update pipeline.c — add registration and publish functions**

After the `pipeline_register_motor_handler` function (after line 54), add:

```c
void pipeline_register_arm_handler(arm_command_handler_fn handler)
{
    s_arm_handler = handler;
}
```

After the `pipeline_publish_status` function (after line 107), add:

```c
void pipeline_publish_motor_status(const boat_MotorStatus *mstatus)
{
    boat_BoatMessage msg = boat_BoatMessage_init_zero;
    msg.which_payload = boat_BoatMessage_motor_status_tag;
    msg.payload.motor_status = *mstatus;

    uint8_t buf[32];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));

    if (!pb_encode(&stream, boat_BoatMessage_fields, &msg)) {
        ESP_LOGE(TAG, "MotorStatus encode failed: %s", PB_GET_ERROR(&stream));
        return;
    }

    size_t len = stream.bytes_written;

    for (int i = 0; i < s_transport_count; i++) {
        esp_err_t ret = s_transports[i].send(buf, len, s_transports[i].ctx);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Transport %d motor_status send failed: %s", i, esp_err_to_name(ret));
        }
    }
}
```

- [ ] **Step 4: Update pipeline.c — add arm_cmd dispatch**

In `pipeline_handle_incoming()`, add a new case before the `default:` case (before line 134):

```c
    case boat_BoatMessage_arm_cmd_tag:
        ESP_LOGI(TAG, "Arm command received: %s", msg.payload.arm_cmd.arm ? "ARM" : "DISARM");
        if (s_arm_handler) {
            s_arm_handler(msg.payload.arm_cmd.arm);
        }
        break;
```

- [ ] **Step 5: Commit**

```bash
git add main/pipeline.h main/pipeline.c
git commit -m "feat: add motor status publish and arm command dispatch to pipeline"
```

---

### Task 5: Build Integration — CMakeLists and main.c

**Files:**
- Modify: `main/CMakeLists.txt`
- Modify: `main/main.c`

Depends on: Task 1, Task 2, Task 3, Task 4

- [ ] **Step 1: Update CMakeLists.txt — add source files**

In `main/CMakeLists.txt`, add `"drivers/esc_driver.c"` after `"drivers/camera_driver.c"` (after line 9) in the SRCS list:

```
        "drivers/esc_driver.c"
```

Add `"motor_control.c"` after `"pipeline.c"` (after line 13) in the SRCS list:

```
        "motor_control.c"
```

- [ ] **Step 2: Update CMakeLists.txt — add MCPWM dependency**

In the PRIV_REQUIRES list, add `esp_driver_mcpwm` after `driver` on line 19:

```
        spi_flash esp_timer nvs_flash driver esp_driver_mcpwm
```

- [ ] **Step 3: Update main.c — add include**

After the line `#include "detect_task.h"` (line 28), add:

```c
#include "motor_control.h"
```

- [ ] **Step 4: Update main.c — add motor_control_init after NVS**

After the `fs_init();` call (after line 58), add:

```c
    // 1b. Start ESC PWM at neutral — must be running before ESCs see power
    ESP_LOGI(TAG, "Initializing motor control (neutral PWM)...");
    ESP_ERROR_CHECK(motor_control_init());
```

- [ ] **Step 5: Update main.c — add motor_control_arm after HTTP server**

After the `ESP_ERROR_CHECK(http_server_start());` line (after line 165), add:

```c
        // 11b. Arm ESCs (blocking ~3 s — neutral PWM running since step 1b)
        ESP_LOGI(TAG, "Arming ESCs...");
        esp_err_t esc_ret = motor_control_arm();
        if (esc_ret != ESP_OK) {
            ESP_LOGW(TAG, "ESC arming failed (%s) — arm via dashboard later",
                     esp_err_to_name(esc_ret));
        }
```

Note: This is inside the `if (wifi_ret == ESP_OK)` block, so ESCs only auto-arm when WiFi is available.

- [ ] **Step 6: Build test**

Run: `idf.py build`

Expected: Successful compilation. If there are errors, fix them before committing.

- [ ] **Step 7: Commit**

```bash
git add main/CMakeLists.txt main/main.c
git commit -m "feat: integrate motor control into boot sequence"
```

---

### Task 6: Dashboard — Motor control UI

**Files:**
- Modify: `main/dashboard.html`

Depends on: Task 1 (proto schema must match)

- [ ] **Step 1: Add motor card CSS**

In `main/dashboard.html`, before the closing `</style>` tag (before line 131), add:

```css
  /* Motor control card */
  #motor-card { grid-column: 3; }
  .motor-controls { display: flex; flex-direction: column; gap: 8px; flex: 1; }
  .motor-state { display: flex; align-items: center; gap: 8px; }
  .state-dot { width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0; }
  .state-dot.disarmed { background: var(--dim); }
  .state-dot.arming { background: var(--warn); animation: pulse 1s infinite; }
  .state-dot.armed { background: var(--green); box-shadow: 0 0 6px var(--green); }
  @keyframes pulse { 0%,100% { opacity: 1; } 50% { opacity: 0.4; } }
  .btn-arm { padding: 4px 12px; border-radius: 4px; border: 1px solid var(--border); font-size: 10px; cursor: pointer; text-transform: uppercase; letter-spacing: 1px; margin-left: auto; }
  .btn-arm.arm { background: var(--bg); color: var(--green); border-color: var(--green); }
  .btn-arm.disarm { background: var(--bg); color: var(--danger); border-color: var(--danger); }
  .motor-slider-row { display: flex; align-items: center; gap: 8px; }
  .motor-slider-row label { font-size: 10px; color: var(--dim); min-width: 55px; text-transform: uppercase; letter-spacing: 1px; }
  .motor-slider-row input[type=range] { flex: 1; accent-color: var(--accent); }
  .motor-slider-row .val { font-size: 12px; color: var(--accent); min-width: 40px; text-align: right; font-weight: bold; }
  .motor-bars { display: flex; gap: 16px; justify-content: center; padding-top: 4px; }
  .motor-bar-wrap { text-align: center; }
  .motor-bar-label { font-size: 9px; color: var(--dim); letter-spacing: 1px; margin-bottom: 4px; text-transform: uppercase; }
  .motor-bar { width: 28px; height: 70px; background: var(--bg); border-radius: 4px; border: 1px solid var(--border); position: relative; overflow: hidden; }
  .motor-bar-fill { position: absolute; left: 0; right: 0; background: var(--accent); transition: height 0.15s, top 0.15s, bottom 0.15s; }
```

- [ ] **Step 2: Update grid layout CSS**

Change the `.grid` rule (line 34-39) from:

```css
    grid-template-rows: auto auto;
```

to:

```css
    grid-template-rows: 1fr 1fr auto;
```

Change the `#camera-card` rule (line 50) from:

```css
  #camera-card { grid-column: 1 / 3; grid-row: 1 / 3; }
```

to:

```css
  #camera-card { grid-column: 1 / 3; grid-row: 1 / 4; }
```

- [ ] **Step 3: Add motor card HTML**

After the closing `</div>` of the tof-card (after line 299, before `</div>` that closes the grid), add:

```html
  <div class="card" id="motor-card">
    <div class="card-title">Motor Control</div>
    <div class="motor-controls">
      <div class="motor-state">
        <div class="state-dot disarmed" id="motor-state-dot"></div>
        <span id="motor-state-label" style="font-size:11px;color:var(--dim);">DISARMED</span>
        <button class="btn-arm arm" id="arm-btn">ARM</button>
      </div>
      <div class="motor-slider-row">
        <label>Left</label>
        <input type="range" id="motor-left" min="-100" max="100" value="0" step="1" disabled />
        <span class="val" id="motor-left-val">0%</span>
      </div>
      <div class="motor-slider-row">
        <label>Right</label>
        <input type="range" id="motor-right" min="-100" max="100" value="0" step="1" disabled />
        <span class="val" id="motor-right-val">0%</span>
      </div>
      <div class="motor-bars">
        <div class="motor-bar-wrap">
          <div class="motor-bar-label">L</div>
          <div class="motor-bar" id="motor-bar-left"><div class="motor-bar-fill" id="motor-fill-left"></div></div>
        </div>
        <div class="motor-bar-wrap">
          <div class="motor-bar-label">R</div>
          <div class="motor-bar" id="motor-bar-right"><div class="motor-bar-fill" id="motor-fill-right"></div></div>
        </div>
      </div>
    </div>
  </div>
```

- [ ] **Step 4: Update inline proto schema**

Replace the `protoSchema` string (lines 309-319) with:

```javascript
const protoSchema = `
syntax = "proto3";
package boat;
message IMUData { float pitch = 1; float roll = 2; float heading = 3; }
message ToFGrid { bool valid = 1; repeated int32 distances = 2; repeated uint32 sigma = 3; repeated uint32 target_status = 4; repeated uint32 nb_target_detected = 5; }
message Detection { int32 category = 1; float score = 2; int32 x1 = 3; int32 y1 = 4; int32 x2 = 5; int32 y2 = 6; }
message SensorSnapshot { uint64 timestamp_us = 1; IMUData imu = 2; ToFGrid tof_a = 3; ToFGrid tof_b = 4; repeated Detection detections = 5; }
message MotorCommand { float throttle = 1; float rudder = 2; float left = 3; float right = 4; }
message MotorStatus { uint32 state = 1; float left_throttle = 2; float right_throttle = 3; }
message ArmCommand { bool arm = 1; }
message DetectCommand { }
message SystemStatus { uint32 heap_free = 1; int32 wifi_rssi = 2; uint64 uptime_us = 3; }
message BoatMessage { oneof payload { SensorSnapshot sensors = 1; MotorCommand motor = 2; SystemStatus status = 3; DetectCommand detect = 4; MotorStatus motor_status = 6; ArmCommand arm_cmd = 7; } }
`;
```

Note: If GpsCoordinate/coord (tag 5) are present in the current file, keep them in SensorSnapshot and BoatMessage.

- [ ] **Step 5: Add motor control JavaScript**

Before the `// Init` section (find the line `drawHorizon(0, 0);` — around line 1545), add the motor control JS block:

```javascript
// ═══════════════════════════════════════════════════════════════
// Motor Control
// ═══════════════════════════════════════════════════════════════
const MOTOR_STATES = ['DISARMED', 'ARMING', 'ARMED'];
const MOTOR_CLASSES = ['disarmed', 'arming', 'armed'];
let motorArmed = false;

function updateMotorStatus(ms) {
  const idx = ms.state || 0;
  const dot = $('motor-state-dot');
  dot.className = 'state-dot ' + (MOTOR_CLASSES[idx] || 'disarmed');
  $('motor-state-label').textContent = MOTOR_STATES[idx] || 'UNKNOWN';
  motorArmed = (idx === 2);

  const btn = $('arm-btn');
  if (motorArmed) {
    btn.textContent = 'DISARM';
    btn.className = 'btn-arm disarm';
  } else {
    btn.textContent = 'ARM';
    btn.className = 'btn-arm arm';
  }

  $('motor-left').disabled = !motorArmed;
  $('motor-right').disabled = !motorArmed;

  updateMotorBar('left', ms.leftThrottle || 0);
  updateMotorBar('right', ms.rightThrottle || 0);
}

function updateMotorBar(side, value) {
  const fill = $('motor-fill-' + side);
  const bar = $('motor-bar-' + side);
  const h = bar.clientHeight;
  const mid = h / 2;
  const px = Math.abs(value) * mid;
  if (value >= 0) {
    fill.style.bottom = mid + 'px';
    fill.style.top = 'auto';
    fill.style.height = px + 'px';
    fill.style.background = 'var(--green)';
  } else {
    fill.style.top = mid + 'px';
    fill.style.bottom = 'auto';
    fill.style.height = px + 'px';
    fill.style.background = 'var(--danger)';
  }
}

let _motorSendTimer = null;
function sendMotorCommand() {
  if (!ws || ws.readyState !== WebSocket.OPEN || !motorArmed) return;
  const left  = parseInt($('motor-left').value) / 100;
  const right = parseInt($('motor-right').value) / 100;
  const msg = BoatMessage.create({ motor: { left, right } });
  const buf = BoatMessage.encode(msg).finish();
  ws.send(buf);
}

['motor-left', 'motor-right'].forEach(id => {
  $(id).addEventListener('input', e => {
    const lbl = id === 'motor-left' ? 'motor-left-val' : 'motor-right-val';
    $(lbl).textContent = e.target.value + '%';
    if (!_motorSendTimer) {
      _motorSendTimer = setInterval(sendMotorCommand, 100);
    }
  });
  $(id).addEventListener('change', () => {
    sendMotorCommand();
    if (_motorSendTimer) { clearInterval(_motorSendTimer); _motorSendTimer = null; }
  });
  $(id).addEventListener('dblclick', () => {
    $(id).value = 0;
    const lbl = id === 'motor-left' ? 'motor-left-val' : 'motor-right-val';
    $(lbl).textContent = '0%';
    sendMotorCommand();
  });
});

$('arm-btn').addEventListener('click', () => {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const msg = BoatMessage.create({ armCmd: { arm: !motorArmed } });
  const buf = BoatMessage.encode(msg).finish();
  ws.send(buf);
});
```

- [ ] **Step 6: Add MotorStatus handling in WS onmessage**

In the `ws.onmessage` handler, after the `if (msg.status) { ... }` block (after line 1084), add:

```javascript
      if (msg.motorStatus) {
        updateMotorStatus(msg.motorStatus);
      }
```

- [ ] **Step 7: Commit**

```bash
git add main/dashboard.html
git commit -m "feat: add motor control card to dashboard"
```

---

### Task 7: Final Build Verification

**Files:** None (verification only)

Depends on: Task 5

- [ ] **Step 1: Full build**

Run: `idf.py build`

Expected: Clean compilation with no errors. Warnings about unused variables are acceptable but should be investigated.

- [ ] **Step 2: Verify binary size**

Run: `idf.py size`

Expected: The MCPWM driver and motor_control logic add ~2-4 KB to the binary. Dashboard HTML grows by ~3 KB. Total should be well within ESP32-P4 flash limits.

- [ ] **Step 3: Commit any build fixes**

If any fixes were needed, commit them:

```bash
git add -A
git commit -m "fix: resolve build issues from motor control integration"
```
