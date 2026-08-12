import shutil
import subprocess
import tempfile
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def application_sources():
    return {
        path.relative_to(ROOT).as_posix(): path.read_text()
        for path in (ROOT / "main").rglob("*")
        if path.suffix in {".c", ".h"}
    }


def function_containing(needle):
    matches = []
    function_start = re.compile(
        r"(?m)^[\w\s*]+\b(?P<name>[A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{"
    )
    for source in application_sources().values():
        for match in function_start.finditer(source):
            depth = 0
            for index in range(match.end() - 1, len(source)):
                if source[index] == "{":
                    depth += 1
                elif source[index] == "}":
                    depth -= 1
                    if depth == 0:
                        body = source[match.end():index]
                        if needle in body:
                            matches.append(match.group("name"))
                        break
    assert len(matches) == 1, f"expected one function containing {needle!r}, got {matches}"
    return matches[0]


STUB_HEADERS = {
    "motor_control.h": r"""
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "pipeline.h"
esp_err_t motor_control_init_hw(void);
esp_err_t motor_control_init(void);
void motor_control_notify_link_rx(int64_t received_us);
uint32_t motor_control_get_status(boat_MotorStatus *out);
""",
    "esp_err.h": r"""
#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_NO_MEM 0x101
static inline const char *esp_err_to_name(esp_err_t err) {(void)err; return "err";}
""",
    "esp_check.h": r"""
#pragma once
#define ESP_RETURN_ON_ERROR(expr, tag, msg) do { (void)(tag); (void)(msg); int rc_ = (expr); if (rc_ != ESP_OK) return rc_; } while (0)
""",
    "esp_log.h": r"""
#pragma once
void test_log(const char *tag, const char *format, ...);
#define ESP_LOGD(...) test_log(__VA_ARGS__)
#define ESP_LOGI(...) test_log(__VA_ARGS__)
#define ESP_LOGW(...) test_log(__VA_ARGS__)
#define ESP_LOGE(...) test_log(__VA_ARGS__)
""",
    "esp_timer.h": r"""
#pragma once
#include <stdint.h>
typedef void *esp_timer_handle_t;
typedef struct { void (*callback)(void *); const char *name; } esp_timer_create_args_t;
int64_t esp_timer_get_time(void);
esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us);
""",
    "freertos/FreeRTOS.h": r"""
#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef struct { int unused; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portMAX_DELAY ((TickType_t)-1)
""",
    "freertos/task.h": r"""
#pragma once
#include "freertos/FreeRTOS.h"
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
TickType_t xTaskGetTickCount(void);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait);
BaseType_t xTaskNotifyGive(TaskHandle_t task);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
""",
    "freertos/queue.h": r"""
#pragma once
#include "freertos/FreeRTOS.h"
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait);
BaseType_t xQueueSendToFront(QueueHandle_t queue, const void *item, TickType_t wait);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait);
BaseType_t xQueueReset(QueueHandle_t queue);
""",
    "freertos/semphr.h": "#pragma once\ntypedef void *SemaphoreHandle_t;\n",
    "esp_http_server.h": "#pragma once\ntypedef void *httpd_handle_t;\n",
    "drivers/esc_driver.h": r"""
#pragma once
#include "esp_err.h"
typedef enum { ESC_STATE_DISARMED, ESC_STATE_ARMING, ESC_STATE_ARMED } esc_state_t;
esp_err_t esc_driver_init(void);
esp_err_t esc_driver_arm_begin(void);
esp_err_t esc_driver_arm_complete(void);
esp_err_t esc_driver_set_throttle(float left, float right);
esp_err_t esc_driver_disarm(void);
esc_state_t esc_driver_get_state(void);
void esc_driver_get_throttle(float *left, float *right);
""",
    "drivers/winch_driver.h": r"""
#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t winch_driver_set_speed(float speed);
esp_err_t winch_driver_set_power(bool on);
bool winch_driver_get_power(void);
float winch_driver_get_speed(void);
""",
    "drivers/steer_driver.h": r"""
#pragma once
#include <stdint.h>
#include "esp_err.h"
esp_err_t steer_driver_set(float steer);
float steer_driver_get(void);
void steer_driver_reassert(void);
esp_err_t steer_driver_set_raw_us(uint32_t pulse_us);
""",
    "drivers/gps_driver.h": r"""
#pragma once
#include <stdbool.h>
typedef struct { float speed_mps; } gps_fix_t;
bool gps_driver_has_lock(void);
esp_err_t gps_driver_get_fix(gps_fix_t *fix);
""",
    "drivers/imu_driver.h": r"""
#pragma once
#include <stdbool.h>
bool imu_icm_ok(void);
bool imu_mag_ok(void);
""",
    "sensor_fusion.h": r"""
#pragma once
typedef struct { float pitch; float roll; float heading; } FusionResult;
void fusion_get_result(FusionResult *result);
""",
    "transports/ws_transport.h": "#pragma once\nint ws_transport_client_count(void);\n",
    "pipeline.h": r"""
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { float throttle; float rudder; float left; float right; } boat_MotorCommand;
typedef struct { float speed; } boat_WinchCommand;
typedef struct { float left; float right; } boat_SteerCommand;
typedef struct { uint32_t pulse_us; } boat_SteerRawCommand;
typedef struct { uint32_t state; float left_throttle; float right_throttle; float winch_speed; bool servo_power; } boat_MotorStatus;
#define boat_MotorStatus_init_zero {0, 0, 0, 0, 0}
typedef void (*motor_command_handler_fn)(const boat_MotorCommand *);
typedef void (*arm_command_handler_fn)(bool, bool);
typedef void (*winch_command_handler_fn)(const boat_WinchCommand *);
typedef void (*steer_command_handler_fn)(const boat_SteerCommand *);
typedef void (*servo_power_handler_fn)(bool);
typedef void (*steer_raw_command_handler_fn)(const boat_SteerRawCommand *);
void pipeline_register_motor_handler(motor_command_handler_fn handler);
void pipeline_register_arm_handler(arm_command_handler_fn handler);
void pipeline_register_winch_handler(winch_command_handler_fn handler);
void pipeline_register_steer_handler(steer_command_handler_fn handler);
void pipeline_register_servo_power_handler(servo_power_handler_fn handler);
void pipeline_register_steer_raw_handler(steer_raw_command_handler_fn handler);
void pipeline_publish_motor_status(const boat_MotorStatus *status);
bool pipeline_recent_command(int64_t max_age_us);
""",
}


HARNESS = r"""
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <string.h>
#include "motor_control.h"
#include "pipeline.h"
#include "runtime_task.h"
#include "runtime_metrics.h"
#include "drivers/esc_driver.h"
#include "drivers/gps_driver.h"
#include "drivers/steer_driver.h"
#include "drivers/winch_driver.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "sensor_fusion.h"

static motor_command_handler_fn motor_handler;
static arm_command_handler_fn arm_handler;
static winch_command_handler_fn winch_handler;
static steer_command_handler_fn steer_handler;
static servo_power_handler_fn power_handler;
static steer_raw_command_handler_fn raw_handler;

static unsigned throttle_writes;
static unsigned winch_writes;
static unsigned power_writes;
static unsigned steer_writes;
static unsigned raw_writes;
static unsigned transport_publishes;
static unsigned control_notifications;
static unsigned arm_begin_calls;
static unsigned arm_complete_calls;
static unsigned disarm_calls;
static int64_t now_us = 1000;
static float esc_left;
static float esc_right;
static float winch_speed;
static float steer_value = 1.0f;
static bool servo_power;
static bool gps_lock;
static esc_state_t esc_state = ESC_STATE_DISARMED;
static TaskFunction_t control_fn;
static TaskFunction_t arm_sequence_fn;
static runtime_task_id_t runtime_task_failure = RUNTIME_TASK_COUNT;
static jmp_buf control_wait;
static jmp_buf arm_sequence_wait;
static bool stop_at_wait;
static bool exercise_urgent_wake;
static unsigned wait_calls;
static TickType_t observed_waits[2];
static TickType_t fake_tick;
static TickType_t observed_arm_wait;
static unsigned arm_script_stage;
static bool run_arm_script;
static unsigned arm_expected_non_esc_writes;

typedef struct {
    size_t item_size;
    unsigned count;
    unsigned head;
    unsigned tail;
    unsigned char items[4][24];
} test_queue_t;

static test_queue_t test_queues[6];
static unsigned queue_create_count;

static void run_one_control_cycle(void);

static unsigned non_esc_write_count(void) {
    return throttle_writes + winch_writes + power_writes + steer_writes + raw_writes;
}

void pipeline_register_motor_handler(motor_command_handler_fn fn) { motor_handler = fn; }
void pipeline_register_arm_handler(arm_command_handler_fn fn) { arm_handler = fn; }
void pipeline_register_winch_handler(winch_command_handler_fn fn) { winch_handler = fn; }
void pipeline_register_steer_handler(steer_command_handler_fn fn) { steer_handler = fn; }
void pipeline_register_servo_power_handler(servo_power_handler_fn fn) { power_handler = fn; }
void pipeline_register_steer_raw_handler(steer_raw_command_handler_fn fn) { raw_handler = fn; }
void pipeline_publish_motor_status(const boat_MotorStatus *status) { (void)status; ++transport_publishes; }
bool pipeline_recent_command(int64_t max_age_us) { (void)max_age_us; return true; }
void test_log(const char *tag, const char *format, ...) { (void)tag; (void)format; }

esp_err_t esc_driver_init(void) { return ESP_OK; }
esp_err_t esc_driver_arm_begin(void) { assert(esc_state == ESC_STATE_DISARMED); esc_state = ESC_STATE_ARMING; ++arm_begin_calls; return ESP_OK; }
esp_err_t esc_driver_arm_complete(void) { assert(esc_state == ESC_STATE_ARMING); esc_state = ESC_STATE_ARMED; ++arm_complete_calls; return ESP_OK; }
esp_err_t esc_driver_disarm(void) { esc_state = ESC_STATE_DISARMED; esc_left = 0; esc_right = 0; ++disarm_calls; return ESP_OK; }
esp_err_t esc_driver_set_throttle(float left, float right) { esc_left = left; esc_right = right; ++throttle_writes; return ESP_OK; }
esc_state_t esc_driver_get_state(void) { return esc_state; }
void esc_driver_get_throttle(float *left, float *right) { *left = esc_left; *right = esc_right; }
esp_err_t winch_driver_set_speed(float speed) { winch_speed = speed; ++winch_writes; return ESP_OK; }
esp_err_t winch_driver_set_power(bool on) { servo_power = on; ++power_writes; return ESP_OK; }
bool winch_driver_get_power(void) { return servo_power; }
float winch_driver_get_speed(void) { return winch_speed; }
esp_err_t steer_driver_set(float steer) { steer_value = steer; ++steer_writes; return ESP_OK; }
float steer_driver_get(void) { return steer_value; }
void steer_driver_reassert(void) {}
esp_err_t steer_driver_set_raw_us(uint32_t pulse_us) { (void)pulse_us; ++raw_writes; return ESP_OK; }
bool gps_driver_has_lock(void) { return gps_lock; }
esp_err_t gps_driver_get_fix(gps_fix_t *fix) { (void)fix; return ESP_OK; }
bool imu_icm_ok(void) { return false; }
bool imu_mag_ok(void) { return false; }
void fusion_get_result(FusionResult *result) { (void)result; }
int ws_transport_client_count(void) { return 1; }
int64_t esp_timer_get_time(void) { return now_us; }
esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out) { (void)args; *out = (void *)1; return ESP_OK; }
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period) { (void)timer; (void)period; return ESP_OK; }
TickType_t xTaskGetTickCount(void) { return fake_tick; }
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait) {
    (void)clear;
    if (exercise_urgent_wake) {
        assert(wait_calls < 2);
        observed_waits[wait_calls++] = wait;
        if (wait_calls == 1) { fake_tick = 4; return 1; }
        longjmp(control_wait, 1);
    }
    if (stop_at_wait) longjmp(control_wait, 1);
    return 0;
}
BaseType_t xTaskNotifyGive(TaskHandle_t task) { (void)task; ++control_notifications; return pdPASS; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (TaskHandle_t)1; }
esp_err_t runtime_task_create(runtime_task_id_t id, TaskFunction_t fn, void *arg, TaskHandle_t *out) {
    (void)arg;
    if (id == runtime_task_failure) return ESP_ERR_NO_MEM;
    if (id == RUNTIME_TASK_CONTROL) {
        control_fn = fn;
        if (out) *out = (TaskHandle_t)2;
    } else {
        assert(id == RUNTIME_TASK_ARM_SEQUENCE);
        arm_sequence_fn = fn;
        if (out) *out = (TaskHandle_t)3;
    }
    return ESP_OK;
}
void runtime_metrics_count(runtime_task_id_t id, runtime_metric_event_t event) { (void)id; (void)event; }
void runtime_metrics_cycle_begin(runtime_task_id_t id, uint64_t scheduled, uint64_t started) { (void)id; (void)scheduled; (void)started; }
void runtime_metrics_cycle_end(runtime_task_id_t id, uint64_t finished) { (void)id; (void)finished; }

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size) {
    assert(length <= 4 && item_size <= sizeof(test_queues[0].items[0]));
    assert(queue_create_count < 6);
    test_queue_t *queue = &test_queues[queue_create_count++];
    queue->item_size = item_size;
    return queue;
}

static BaseType_t queue_push(test_queue_t *queue, const void *item, bool front) {
    if (queue->count == 4) return pdFALSE;
    if (front) {
        queue->head = (queue->head + 3) % 4;
        memcpy(queue->items[queue->head], item, queue->item_size);
    } else {
        memcpy(queue->items[queue->tail], item, queue->item_size);
        queue->tail = (queue->tail + 1) % 4;
    }
    ++queue->count;
    return pdTRUE;
}

BaseType_t xQueueSend(QueueHandle_t handle, const void *item, TickType_t wait) {
    (void)wait;
    return queue_push(handle, item, false);
}

BaseType_t xQueueSendToFront(QueueHandle_t handle, const void *item, TickType_t wait) {
    (void)wait;
    return queue_push(handle, item, true);
}

BaseType_t xQueueOverwrite(QueueHandle_t handle, const void *item) {
    test_queue_t *queue = handle;
    queue->count = queue->head = queue->tail = 0;
    return queue_push(queue, item, false);
}

BaseType_t xQueueReset(QueueHandle_t handle) {
    test_queue_t *queue = handle;
    queue->count = queue->head = queue->tail = 0;
    return pdPASS;
}

static BaseType_t queue_pop(test_queue_t *queue, void *item) {
    if (queue->count == 0) return pdFALSE;
    memcpy(item, queue->items[queue->head], queue->item_size);
    queue->head = (queue->head + 1) % 4;
    --queue->count;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t handle, void *item, TickType_t wait) {
    test_queue_t *queue = handle;
    if (queue_pop(queue, item) == pdTRUE) return pdTRUE;
    if (!run_arm_script || queue != &test_queues[0]) return pdFALSE;
    assert(non_esc_write_count() == arm_expected_non_esc_writes);

    switch (arm_script_stage++) {
    case 0:
        assert(wait == portMAX_DELAY);
        arm_handler(true, true);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        return queue_pop(queue, item);
    case 1:
        observed_arm_wait = wait;
        assert(arm_begin_calls == 0 && arm_complete_calls == 0 && disarm_calls == 0);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        assert(arm_begin_calls == 1 && arm_complete_calls == 0);
        now_us += 3000000;
        motor_handler(&(boat_MotorCommand){0});
        return pdFALSE;
    case 2:
        assert(wait == portMAX_DELAY);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        assert(arm_complete_calls == 1 && esc_state == ESC_STATE_ARMED);
        arm_handler(false, false);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        return queue_pop(queue, item);
    case 3:
        assert(wait == portMAX_DELAY);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        assert(disarm_calls == 1 && esc_state == ESC_STATE_DISARMED);
        arm_handler(true, true);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        return queue_pop(queue, item);
    case 4:
        observed_arm_wait = wait;
        assert(arm_begin_calls == 1);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        assert(arm_begin_calls == 2 && arm_complete_calls == 1);
        arm_handler(false, false);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        return queue_pop(queue, item);
    case 5:
        assert(wait == portMAX_DELAY);
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        assert(disarm_calls == 2 && esc_state == ESC_STATE_DISARMED);
        now_us += 4000000;
        run_one_control_cycle();
        arm_expected_non_esc_writes = non_esc_write_count();
        assert(arm_complete_calls == 1);
        longjmp(arm_sequence_wait, 1);
    default:
        assert(false);
        return pdFALSE;
    }
}

static bool close_enough(float actual, float expected) {
    return fabsf(actual - expected) < 0.0001f;
}

static void run_one_control_cycle(void) {
    stop_at_wait = true;
    if (setjmp(control_wait) == 0) control_fn(NULL);
    stop_at_wait = false;
}

static void run_scheduled_and_urgent_cycle(void) {
    exercise_urgent_wake = true;
    wait_calls = 0;
    if (setjmp(control_wait) == 0) control_fn(NULL);
    exercise_urgent_wake = false;
    assert(wait_calls == 2);
    assert(observed_waits[0] == 10 && observed_waits[1] == 6);
    fake_tick = 0;
}

int main(void) {
    assert(motor_control_init() == ESP_OK);
    assert(motor_handler && arm_handler && winch_handler && steer_handler && power_handler && raw_handler);
    assert(control_fn != NULL && arm_sequence_fn != NULL);

    motor_handler(&(boat_MotorCommand){.throttle = 0.4f, .rudder = 0.1f});
    assert(throttle_writes == 0);
    run_scheduled_and_urgent_cycle();
    assert(throttle_writes == 1);
    assert(close_enough(esc_left, 0.5f) && close_enough(esc_right, 0.3f));

    now_us += 1;
    motor_handler(&(boat_MotorCommand){.throttle = 1.0f, .rudder = 1.0f});
    run_one_control_cycle();
    assert(close_enough(esc_left, 1.0f) && close_enough(esc_right, 0.0f));

    now_us += 1;
    motor_handler(&(boat_MotorCommand){.left = 0.8f, .right = 0.2f});
    run_one_control_cycle();
    assert(close_enough(esc_left, 0.8f) && close_enough(esc_right, 0.2f));

    now_us += 1;
    unsigned before_winch = winch_writes;
    unsigned before_power = power_writes;
    winch_handler(&(boat_WinchCommand){.speed = 0.5f});
    assert(winch_writes == before_winch && power_writes == before_power);
    run_one_control_cycle();
    assert(close_enough(winch_speed, 0.5f) && servo_power);

    now_us += 1;
    unsigned before_steer = steer_writes;
    steer_handler(&(boat_SteerCommand){.left = -0.25f});
    assert(steer_writes == before_steer);
    run_one_control_cycle();
    assert(close_enough(steer_value, -0.25f));

    now_us += 1;
    unsigned before_raw = raw_writes;
    raw_handler(&(boat_SteerRawCommand){.pulse_us = 1500});
    assert(raw_writes == before_raw);
    run_one_control_cycle();
    assert(raw_writes == before_raw + 1);

    now_us += 1;
    unsigned notifications_before_off = control_notifications;
    before_power = power_writes;
    before_winch = winch_writes;
    before_steer = steer_writes;
    power_handler(false);
    assert(power_writes == before_power && winch_writes == before_winch && steer_writes == before_steer);
    assert(control_notifications == notifications_before_off + 1);
    run_one_control_cycle();
    assert(!servo_power && close_enough(winch_speed, 0.0f) && close_enough(steer_value, 1.0f));

    now_us += 1;
    steer_handler(&(boat_SteerCommand){.left = 0.4f});
    run_one_control_cycle();
    assert(!servo_power && close_enough(steer_value, 0.4f));

    now_us += 1;
    power_handler(true);
    run_one_control_cycle();
    assert(servo_power);

    now_us += 1;
    motor_handler(&(boat_MotorCommand){.left = 0.7f, .right = 0.6f});
    winch_handler(&(boat_WinchCommand){.speed = -0.3f});
    steer_handler(&(boat_SteerCommand){.left = -0.2f});
    run_one_control_cycle();
    assert(servo_power && close_enough(esc_left, 0.7f));

    now_us += 400000;
    run_one_control_cycle();
    assert(close_enough(esc_left, 0.0f) && close_enough(esc_right, 0.0f));
    assert(close_enough(winch_speed, 0.0f) && close_enough(steer_value, 0.0f));
    assert(!servo_power);

    now_us += 1;
    steer_handler(&(boat_SteerCommand){.left = 0.2f});
    run_one_control_cycle();
    assert(servo_power && close_enough(steer_value, 0.2f));

    boat_MotorStatus status;
    uint32_t generation = motor_control_get_status(&status);
    assert(generation != 0);
    assert(status.servo_power && close_enough(status.winch_speed, 0.0f));
    assert(transport_publishes == 0);

    esc_state = ESC_STATE_DISARMED;
    gps_lock = false;
    now_us += 1;
    arm_handler(true, false);
    assert(arm_begin_calls == 0 && arm_complete_calls == 0 && disarm_calls == 0);
    run_one_control_cycle();
    run_arm_script = true;
    arm_script_stage = 0;
    arm_expected_non_esc_writes = non_esc_write_count();
    if (setjmp(arm_sequence_wait) == 0) arm_sequence_fn(NULL);
    run_arm_script = false;
    assert(arm_script_stage == 6);
    assert(observed_arm_wait == pdMS_TO_TICKS(3000));
    assert(arm_begin_calls == 2 && arm_complete_calls == 1 && disarm_calls == 2);

    esc_state = ESC_STATE_ARMED;
    servo_power = true;
    runtime_task_failure = RUNTIME_TASK_ARM_SEQUENCE;
    unsigned disarms_before_failure = disarm_calls;
    assert(motor_control_init() == ESP_ERR_NO_MEM);
    assert(disarm_calls == disarms_before_failure + 1);
    assert(esc_state == ESC_STATE_DISARMED);
    assert(!servo_power);

    esc_state = ESC_STATE_ARMED;
    servo_power = true;
    runtime_task_failure = RUNTIME_TASK_CONTROL;
    disarms_before_failure = disarm_calls;
    assert(motor_control_init() == ESP_ERR_NO_MEM);
    assert(disarm_calls == disarms_before_failure + 1);
    assert(esc_state == ESC_STATE_DISARMED);
    assert(!servo_power);
    return 0;
}
"""


def test_pipeline_handlers_do_not_write_actuators():
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        for relative, content in STUB_HEADERS.items():
            path = tmpdir / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        shutil.copy(ROOT / "main" / "motor_control.c", tmpdir / "motor_control.c")
        (tmpdir / "harness.c").write_text(HARNESS)
        binary = tmpdir / "runtime_architecture_test"
        subprocess.run(
            [
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(tmpdir), "-I", str(ROOT / "main"),
                str(tmpdir / "motor_control.c"), str(ROOT / "main" / "control_arbiter.c"),
                str(ROOT / "main" / "arm_sequence.c"),
                str(ROOT / "main" / "esc_trim.c"),
                str(tmpdir / "harness.c"), "-lm", "-o", str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)


PIPELINE_HEADERS = {
    "pipeline.h": r"""
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#define PIPELINE_MAX_TRANSPORTS 4
typedef struct { int unused; } boat_SensorSnapshot;
typedef struct { int unused; } boat_SystemStatus;
typedef struct { float throttle; float rudder; float left; float right; } boat_MotorCommand;
typedef struct { float speed; } boat_WinchCommand;
typedef struct { float left; float right; } boat_SteerCommand;
typedef struct { uint32_t pulse_us; } boat_SteerRawCommand;
typedef struct { bool arm; bool force; } boat_ArmCommand;
typedef struct { bool on; } boat_ServoPowerCommand;
typedef struct { uint32_t state; float left_throttle; float right_throttle; float winch_speed; bool servo_power; } boat_MotorStatus;
typedef struct {
    int which_payload;
    union {
        boat_SensorSnapshot sensors;
        boat_SystemStatus status;
        boat_MotorStatus motor_status;
        boat_MotorCommand motor;
        boat_WinchCommand winch;
        boat_SteerCommand steer;
        boat_SteerRawCommand steer_raw;
        boat_ArmCommand arm_cmd;
        boat_ServoPowerCommand servo_power;
    } payload;
} boat_BoatMessage;
#define boat_BoatMessage_motor_tag 1
#define boat_BoatMessage_detect_tag 2
#define boat_BoatMessage_arm_cmd_tag 3
#define boat_BoatMessage_winch_tag 4
#define boat_BoatMessage_steer_tag 5
#define boat_BoatMessage_servo_power_tag 6
#define boat_BoatMessage_steer_raw_tag 7
#define boat_BoatMessage_sensors_tag 8
#define boat_BoatMessage_status_tag 9
#define boat_BoatMessage_motor_status_tag 10
#define boat_BoatMessage_training_log_tag 11
#define boat_BoatMessage_size 128
#define boat_SystemStatus_size 32
#define boat_BoatMessage_fields NULL
typedef esp_err_t (*transport_send_fn)(const uint8_t *, size_t, void *);
typedef void (*motor_command_handler_fn)(const boat_MotorCommand *);
typedef void (*arm_command_handler_fn)(bool, bool);
typedef void (*winch_command_handler_fn)(const boat_WinchCommand *);
typedef void (*steer_command_handler_fn)(const boat_SteerCommand *);
typedef void (*servo_power_handler_fn)(bool);
typedef void (*steer_raw_command_handler_fn)(const boat_SteerRawCommand *);
esp_err_t pipeline_init(void);
esp_err_t pipeline_register_transport(transport_send_fn send, void *ctx);
void pipeline_register_motor_handler(motor_command_handler_fn handler);
void pipeline_register_arm_handler(arm_command_handler_fn handler);
void pipeline_register_winch_handler(winch_command_handler_fn handler);
void pipeline_register_steer_handler(steer_command_handler_fn handler);
void pipeline_register_servo_power_handler(servo_power_handler_fn handler);
void pipeline_register_steer_raw_handler(steer_raw_command_handler_fn handler);
void pipeline_publish_sensors(const boat_SensorSnapshot *snap);
void pipeline_publish_status(const boat_SystemStatus *status);
void pipeline_publish_motor_status(const boat_MotorStatus *status);
void pipeline_handle_incoming(const uint8_t *buf, size_t len);
""",
    "esp_err.h": STUB_HEADERS["esp_err.h"],
    "esp_log.h": STUB_HEADERS["esp_log.h"],
    "detect_task.h": "#pragma once\nvoid detect_trigger(void);\n",
    "training_log_task.h": "#pragma once\n#include <stdbool.h>\nvoid training_log_trigger(void);\nextern volatile bool g_training_log_camera_active;\n",
    "esp_timer.h": "#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "freertos/FreeRTOS.h": STUB_HEADERS["freertos/FreeRTOS.h"] + "\n#define portMAX_DELAY ((TickType_t)-1)\n",
    "freertos/semphr.h": r"""
#pragma once
#include "freertos/FreeRTOS.h"
typedef void *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t wait);
BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore);
""",
    "pb_stub.h": r"""
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct { size_t bytes_written; } pb_ostream_t;
typedef struct { const uint8_t *buffer; size_t size; } pb_istream_t;
pb_ostream_t pb_ostream_from_buffer(uint8_t *buffer, size_t size);
pb_istream_t pb_istream_from_buffer(const uint8_t *buffer, size_t size);
bool pb_encode(pb_ostream_t *stream, const void *fields, const void *src);
bool pb_decode(pb_istream_t *stream, const void *fields, void *dest);
#define PB_GET_ERROR(stream) "test"
""",
    "pb_encode.h": "#pragma once\n#include \"pb_stub.h\"\n",
    "pb_decode.h": "#pragma once\n#include \"pb_stub.h\"\n",
}


PIPELINE_HARNESS = r"""
#include <assert.h>
#include <string.h>
#include "pipeline.h"
#include "pb_stub.h"
#include "freertos/semphr.h"

static int decoded_tag;
static unsigned detect_calls;
static unsigned manual_control_calls;
static unsigned training_log_calls;

void test_log(const char *tag, const char *format, ...) { (void)tag; (void)format; }
void detect_trigger(void) { ++detect_calls; }
void training_log_trigger(void) { ++training_log_calls; }
int64_t esp_timer_get_time(void) { return 0; }
volatile bool g_training_log_camera_active = false;
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (SemaphoreHandle_t)1; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t wait) {
    (void)semaphore; (void)wait; return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore) { (void)semaphore; return pdTRUE; }
pb_ostream_t pb_ostream_from_buffer(uint8_t *buffer, size_t size) {
    (void)buffer; (void)size; return (pb_ostream_t){0};
}
pb_istream_t pb_istream_from_buffer(const uint8_t *buffer, size_t size) {
    return (pb_istream_t){.buffer = buffer, .size = size};
}
bool pb_encode(pb_ostream_t *stream, const void *fields, const void *src) {
    (void)fields; (void)src; stream->bytes_written = 1; return true;
}
bool pb_decode(pb_istream_t *stream, const void *fields, void *dest) {
    (void)stream; (void)fields;
    boat_BoatMessage *message = dest;
    message->which_payload = decoded_tag;
    message->payload.motor.left = 0.2f;
    return true;
}
static void motor_handler(const boat_MotorCommand *command) {
    (void)command; ++manual_control_calls;
}
static void arm_handler(bool arm, bool force) { (void)arm; (void)force; ++manual_control_calls; }
static void winch_handler(const boat_WinchCommand *command) { (void)command; ++manual_control_calls; }
static void steer_handler(const boat_SteerCommand *command) { (void)command; ++manual_control_calls; }
static void power_handler(bool on) { (void)on; ++manual_control_calls; }
static void raw_handler(const boat_SteerRawCommand *command) { (void)command; ++manual_control_calls; }

int main(void) {
    const uint8_t input[] = {0};
    assert(pipeline_init() == ESP_OK);
    pipeline_register_motor_handler(motor_handler);
    pipeline_register_arm_handler(arm_handler);
    pipeline_register_winch_handler(winch_handler);
    pipeline_register_steer_handler(steer_handler);
    pipeline_register_servo_power_handler(power_handler);
    pipeline_register_steer_raw_handler(raw_handler);

    decoded_tag = boat_BoatMessage_detect_tag;
    pipeline_handle_incoming(input, sizeof(input));
    assert(detect_calls == 1);
    assert(manual_control_calls == 0);

    decoded_tag = boat_BoatMessage_training_log_tag;
    pipeline_handle_incoming(input, sizeof(input));
    assert(training_log_calls == 1);
    assert(manual_control_calls == 0);

    decoded_tag = boat_BoatMessage_motor_tag;
    pipeline_handle_incoming(input, sizeof(input));
    assert(manual_control_calls == 1);
    return 0;
}
"""


def test_non_motor_dispatch_does_not_enter_manual_control_ingress():
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        for relative, content in PIPELINE_HEADERS.items():
            path = tmpdir / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        shutil.copy(ROOT / "main" / "pipeline.c", tmpdir / "pipeline.c")
        (tmpdir / "harness.c").write_text(PIPELINE_HARNESS)
        binary = tmpdir / "pipeline_detect_test"
        subprocess.run(
            [
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(tmpdir), str(tmpdir / "pipeline.c"),
                str(tmpdir / "harness.c"), "-o", str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)


DIAGNOSTICS_HEADERS = {
    "esp_err.h": STUB_HEADERS["esp_err.h"],
    "esp_log.h": STUB_HEADERS["esp_log.h"],
    "freertos/FreeRTOS.h": STUB_HEADERS["freertos/FreeRTOS.h"],
    "freertos/task.h": STUB_HEADERS["freertos/task.h"] + r"""
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t task);
void vTaskDelay(TickType_t ticks);
""",
    "motor_control.h": r"""
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint32_t state; float left_throttle; float right_throttle; float winch_speed; bool servo_power; } boat_MotorStatus;
uint32_t motor_control_get_status(boat_MotorStatus *out);
""",
    "pipeline.h": r"""
#pragma once
#include "motor_control.h"
void pipeline_publish_motor_status(const boat_MotorStatus *status);
""",
    "drivers/gps_driver.h": r"""
#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef struct {
    uint32_t uart_fifo_overflows;
    uint32_t uart_buffer_full_events;
    uint32_t parser_line_overflows;
    uint32_t parse_errors;
    unsigned protocol_authority;
    int64_t last_frame_us;
    int64_t fix_age_us;
} gps_runtime_status_t;
esp_err_t gps_driver_get_runtime_status(gps_runtime_status_t *out);
""",
}


DIAGNOSTICS_HARNESS = r"""
#include <assert.h>
#include <setjmp.h>
#include <stddef.h>
#include "runtime_metrics.h"
#include "drivers/gps_driver.h"
#include "motor_control.h"

static unsigned motor_status_publishes;
static unsigned gps_status_reads;
static jmp_buf diagnostics_wait;

void test_log(const char *tag, const char *format, ...) { (void)tag; (void)format; }
uint32_t motor_control_get_status(boat_MotorStatus *out) {
    *out = (boat_MotorStatus){.state = 2, .left_throttle = 0.4f};
    return 7;
}
void pipeline_publish_motor_status(const boat_MotorStatus *status) {
    assert(status->state == 2);
    ++motor_status_publishes;
}
esp_err_t gps_driver_get_runtime_status(gps_runtime_status_t *out) {
    *out = (gps_runtime_status_t){.parse_errors = 3, .fix_age_us = 4000};
    ++gps_status_reads;
    return ESP_OK;
}
const runtime_task_spec_t *runtime_schedule_get(runtime_task_id_t id) {
    static runtime_task_spec_t specs[RUNTIME_TASK_COUNT];
    specs[id].name = "test";
    specs[id].core = id == RUNTIME_TASK_CONTROL ? 0 : 1;
    return &specs[id];
}
bool runtime_schedule_validate(void) { return true; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (TaskHandle_t)1; }
TickType_t xTaskGetTickCount(void) { return 0; }
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t task) { (void)task; return 100; }
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait) {
    (void)clear; (void)wait;
    longjmp(diagnostics_wait, 1);
}
BaseType_t xTaskNotifyGive(TaskHandle_t task) { (void)task; return pdPASS; }
void vTaskDelay(TickType_t ticks) { (void)ticks; longjmp(diagnostics_wait, 1); }

int main(void) {
    runtime_metrics_init();
    if (setjmp(diagnostics_wait) == 0) task_runtime_diagnostics(NULL);
    assert(motor_status_publishes == 1);
    assert(gps_status_reads == 1);
    return 0;
}
"""


def test_core1_diagnostics_publishes_motor_status():
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        for relative, content in DIAGNOSTICS_HEADERS.items():
            path = tmpdir / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        shutil.copy(ROOT / "main" / "runtime_metrics.c", tmpdir / "runtime_metrics.c")
        (tmpdir / "harness.c").write_text(DIAGNOSTICS_HARNESS)
        binary = tmpdir / "diagnostics_status_test"
        subprocess.run(
            [
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DESP_PLATFORM",
                "-I", str(tmpdir), "-I", str(ROOT / "main"),
                str(tmpdir / "runtime_metrics.c"), str(tmpdir / "harness.c"),
                "-o", str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)


def test_fusion_only_consumes_versioned_raw_samples():
    fusion_source = (ROOT / "main" / "sensor_fusion.c").read_text()
    raw_snapshot_source = (ROOT / "main" / "sample_snapshot.c").read_text()

    assert "sample_snapshot_read" in fusion_source
    assert "fusion_update_sample" in fusion_source
    assert "atomic_thread_fence(memory_order_seq_cst)" in fusion_source
    assert "atomic_thread_fence(memory_order_seq_cst)" in raw_snapshot_source
    for forbidden in (
        "imu_read_",
        "imu_recover_",
        "imu_reinit_",
        "g_i2c_mutex",
        "g_inference_active",
    ):
        assert forbidden not in fusion_source


def test_sensor_i2c_ownership_is_confined():
    """Each sensor type has exactly one task that ever touches its I2C
    device -- not "SensorBus owns everything" (that held until the
    2026-08-11 split: a single VL53L5CX read takes ~33ms, longer than
    SensorBus's own 20ms IMU period, so interleaving it there guaranteed a
    deadline miss on every cycle ToF was selected -- hw-confirmed ~38%
    sustained SensorBus miss rate. ToF moved to its own lower-priority task
    instead). The bus itself is still shared and still relies on ESP-IDF's
    own per-transaction lock, not an application-level mutex around either
    read -- g_i2c_mutex must still not exist, and neither should a new one."""
    sources = application_sources()
    all_application_sources = "\n".join(sources.values())

    assert "g_i2c_mutex" not in all_application_sources
    assert function_containing("sensor_read_imu_sample(") == "task_sensor_bus"
    assert function_containing("tof_read_grid(") == "task_tof_read"
    assert "g_inference_active" not in sources["main/sensor_task.c"]
    assert "g_inference_active" not in sources["main/sensor_fusion.c"]
