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
#include "esc_trim.h"
#include "pipeline.h"
esp_err_t motor_control_init_hw(void);
esp_err_t motor_control_init(void);
void motor_control_notify_link_rx(int64_t received_us);
void motor_control_set_esc_trim(const EscTrimPoint *pts, uint8_t count);
uint32_t motor_control_get_status(boat_MotorStatus *out);
uint32_t motor_control_get_calibrate_status(boat_CalibrateStatus *out);
uint32_t motor_control_get_bench_status(boat_BenchStatus *out);
void motor_control_bench_flush(void);
void motor_control_trimlearn_log(void);
float motor_control_trimlearn_c(void);
bool motor_control_p_assist_on(void);
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
uint32_t steer_driver_get_pulse_us(void);
void steer_driver_reassert(void);
esp_err_t steer_driver_set_raw_us(uint32_t pulse_us);
""",
    "drivers/gps_driver.h": r"""
#pragma once
#include <stdbool.h>
typedef struct { bool valid; float speed_mps; } gps_fix_t;
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
#include <stdint.h>
typedef struct { float pitch; float roll; float heading; float yaw_rate; uint32_t sequence; uint64_t captured_us; } FusionResult;
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
typedef struct { uint32_t state; float left_throttle; float right_throttle; float winch_speed; bool servo_power; float rudder_cmd; uint32_t rudder_pulse_us; bool rudder_saturated; bool assist_rudder; bool assist_motor_p; float yaw_target_dps; float yaw_filt_dps; uint32_t assist_request_id; } boat_MotorStatus;
#define boat_MotorStatus_init_zero {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
typedef struct { uint32_t state; uint32_t level_index; float level_throttle; float trim_diff; float yaw_avg_dps; bool making_way; uint32_t points_done; } boat_CalibrateStatus;
#define boat_CalibrateStatus_init_zero {0, 0, 0, 0, 0, 0, 0}
typedef struct { uint32_t state; uint32_t kind; float base; uint32_t samples; uint32_t file_index; float elapsed_s; float learn_c; bool p_on; } boat_BenchStatus;
#define boat_BenchStatus_init_zero {0, 0, 0, 0, 0, 0, 0, 0}
typedef struct { uint32_t kind; float base; float delta; float reset_c; } boat_BenchCommand;
typedef struct { bool p_on; bool rudder_assist; uint32_t request_id; } boat_AssistCommand;
typedef struct { float target_dps; } boat_SteerRateCommand;
#define boat_BoatMessage_steer_rate_tag 18
typedef void (*motor_command_handler_fn)(const boat_MotorCommand *);
typedef void (*arm_command_handler_fn)(bool, bool);
typedef void (*winch_command_handler_fn)(const boat_WinchCommand *);
typedef void (*steer_command_handler_fn)(const boat_SteerCommand *);
typedef void (*servo_power_handler_fn)(bool);
typedef void (*steer_raw_command_handler_fn)(const boat_SteerRawCommand *);
typedef void (*calibrate_command_handler_fn)(bool, bool);
typedef void (*bench_command_handler_fn)(uint32_t, float, float, float);
typedef void (*assist_command_handler_fn)(bool, bool, uint32_t);
typedef void (*steer_rate_command_handler_fn)(float);
void pipeline_register_steer_rate_handler(steer_rate_command_handler_fn handler);
void pipeline_register_motor_handler(motor_command_handler_fn handler);
void pipeline_register_arm_handler(arm_command_handler_fn handler);
void pipeline_register_winch_handler(winch_command_handler_fn handler);
void pipeline_register_steer_handler(steer_command_handler_fn handler);
void pipeline_register_servo_power_handler(servo_power_handler_fn handler);
void pipeline_register_steer_raw_handler(steer_raw_command_handler_fn handler);
void pipeline_register_calibrate_handler(calibrate_command_handler_fn handler);
void pipeline_register_bench_handler(bench_command_handler_fn handler);
void pipeline_register_assist_handler(assist_command_handler_fn handler);
void pipeline_publish_motor_status(const boat_MotorStatus *status);
void pipeline_publish_calibrate_status(const boat_CalibrateStatus *status);
void pipeline_publish_bench_status(const boat_BenchStatus *status);
bool pipeline_recent_command(int64_t max_age_us);
""",
    "file_system.h": r"""
#pragma once
#include <stddef.h>
#include "esc_trim.h"
bool fs_save_esc_trim(const EscTrimNvsBlob *blob);
#define ESP_ERR_NOT_FOUND 0x105
int snprintf(char *, size_t, const char *, ...);
bool fs_sdcard_ready(void);
esp_err_t fs_sdcard_read(const char *p, void *b, size_t c, size_t *o);
esp_err_t fs_sdcard_write(const char *p, const void *d, size_t l);
esp_err_t fs_sdcard_append(const char *p, const void *d, size_t l);
bool fs_load_esc_trim(EscTrimNvsBlob *blob);
""",
}


HARNESS = r"""
#ifndef ESP_ERR_NOT_FOUND
#define ESP_ERR_NOT_FOUND 0x105
#endif
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
#include "esc_trim.h"

static motor_command_handler_fn motor_handler;
static arm_command_handler_fn arm_handler;
static winch_command_handler_fn winch_handler;
static bench_command_handler_fn bench_handler;
static assist_command_handler_fn assist_handler;
static steer_command_handler_fn steer_handler;
static servo_power_handler_fn power_handler;
static steer_raw_command_handler_fn raw_handler;
static calibrate_command_handler_fn calibrate_handler;

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
void pipeline_register_calibrate_handler(calibrate_command_handler_fn fn) { calibrate_handler = fn; }
void pipeline_register_bench_handler(bench_command_handler_fn fn) { bench_handler = fn; }
void pipeline_register_assist_handler(assist_command_handler_fn fn) { assist_handler = fn; }
static steer_rate_command_handler_fn steer_rate_handler_cb;
void pipeline_register_steer_rate_handler(steer_rate_command_handler_fn fn) { steer_rate_handler_cb = fn; }
void pipeline_publish_motor_status(const boat_MotorStatus *status) { (void)status; ++transport_publishes; }
bool pipeline_recent_command(int64_t max_age_us) { (void)max_age_us; return true; }
bool fs_save_esc_trim(const EscTrimNvsBlob *blob) { (void)blob; return true; }
bool fs_sdcard_ready(void) { return true; }
esp_err_t fs_sdcard_read(const char *p, void *b, size_t c, size_t *o) { (void)p; (void)b; (void)c; if (o) *o = 0; return ESP_ERR_NOT_FOUND; }
esp_err_t fs_sdcard_write(const char *p, const void *d, size_t l) { (void)p; (void)d; (void)l; return ESP_OK; }
esp_err_t fs_sdcard_append(const char *p, const void *d, size_t l) { (void)p; (void)d; (void)l; return ESP_OK; }
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
uint32_t steer_driver_get_pulse_us(void) { return 1516u; }
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
                str(ROOT / "main" / "esc_trim_cal.c"),
                str(ROOT / "main" / "bench_run.c"),
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
typedef struct { uint32_t state; float left_throttle; float right_throttle; float winch_speed; bool servo_power; float rudder_cmd; uint32_t rudder_pulse_us; bool rudder_saturated; bool assist_rudder; bool assist_motor_p; float yaw_target_dps; float yaw_filt_dps; uint32_t assist_request_id; } boat_MotorStatus;
typedef struct { bool start; bool average_into_existing; } boat_CalibrateCommand;
typedef struct { uint32_t state; uint32_t level_index; float level_throttle; float trim_diff; float yaw_avg_dps; bool making_way; uint32_t points_done; } boat_CalibrateStatus;
typedef struct { uint32_t state; uint32_t kind; float base; uint32_t samples; uint32_t file_index; float elapsed_s; float learn_c; bool p_on; } boat_BenchStatus;
typedef struct { uint32_t kind; float base; float delta; float reset_c; } boat_BenchCommand;
typedef struct { bool p_on; bool rudder_assist; uint32_t request_id; } boat_AssistCommand;
typedef struct { float target_dps; } boat_SteerRateCommand;
#define boat_BoatMessage_steer_rate_tag 18
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
        boat_SteerRateCommand steer_rate;
        boat_ArmCommand arm_cmd;
        boat_ServoPowerCommand servo_power;
        boat_CalibrateCommand calibrate;
        boat_CalibrateStatus calibrate_status;
        boat_BenchCommand bench;
        boat_BenchStatus bench_status;
        boat_AssistCommand assist;
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
#define boat_BoatMessage_calibrate_tag 12
#define boat_BoatMessage_calibrate_status_tag 13
#define boat_BoatMessage_bench_tag 14
#define boat_BoatMessage_bench_status_tag 15
#define boat_BoatMessage_assist_tag 16
#define boat_BoatMessage_size 128
#define boat_SystemStatus_size 32
#define boat_CalibrateStatus_size 35
#define boat_BenchStatus_size 44
#define boat_BoatMessage_fields NULL
typedef esp_err_t (*transport_send_fn)(const uint8_t *, size_t, void *);
typedef void (*motor_command_handler_fn)(const boat_MotorCommand *);
typedef void (*arm_command_handler_fn)(bool, bool);
typedef void (*winch_command_handler_fn)(const boat_WinchCommand *);
typedef void (*steer_command_handler_fn)(const boat_SteerCommand *);
typedef void (*servo_power_handler_fn)(bool);
typedef void (*steer_raw_command_handler_fn)(const boat_SteerRawCommand *);
typedef void (*calibrate_command_handler_fn)(bool, bool);
typedef void (*bench_command_handler_fn)(uint32_t, float, float, float);
typedef void (*assist_command_handler_fn)(bool, bool, uint32_t);
typedef void (*steer_rate_command_handler_fn)(float);
void pipeline_register_steer_rate_handler(steer_rate_command_handler_fn handler);
esp_err_t pipeline_init(void);
esp_err_t pipeline_register_transport(transport_send_fn send, void *ctx);
void pipeline_register_motor_handler(motor_command_handler_fn handler);
void pipeline_register_arm_handler(arm_command_handler_fn handler);
void pipeline_register_winch_handler(winch_command_handler_fn handler);
void pipeline_register_steer_handler(steer_command_handler_fn handler);
void pipeline_register_servo_power_handler(servo_power_handler_fn handler);
void pipeline_register_steer_raw_handler(steer_raw_command_handler_fn handler);
void pipeline_register_calibrate_handler(calibrate_command_handler_fn handler);
void pipeline_register_bench_handler(bench_command_handler_fn handler);
void pipeline_register_assist_handler(assist_command_handler_fn handler);
void pipeline_publish_sensors(const boat_SensorSnapshot *snap);
void pipeline_publish_status(const boat_SystemStatus *status);
void pipeline_publish_motor_status(const boat_MotorStatus *status);
void pipeline_publish_calibrate_status(const boat_CalibrateStatus *status);
void pipeline_publish_bench_status(const boat_BenchStatus *status);
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
static unsigned calibrate_calls;

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
static void calibrate_handler(bool start, bool average) { (void)start; (void)average; ++calibrate_calls; }
static void bench_handler(uint32_t kind, float base, float delta, float reset_c)
{ (void)kind; (void)base; (void)delta; (void)reset_c; }
static void assist_handler(bool p_on, bool ra, uint32_t id) { (void)p_on; (void)ra; (void)id; }
static void steer_rate_handler(float d) { (void)d; }

int main(void) {
    const uint8_t input[] = {0};
    assert(pipeline_init() == ESP_OK);
    pipeline_register_motor_handler(motor_handler);
    pipeline_register_arm_handler(arm_handler);
    pipeline_register_winch_handler(winch_handler);
    pipeline_register_steer_handler(steer_handler);
    pipeline_register_servo_power_handler(power_handler);
    pipeline_register_steer_raw_handler(raw_handler);
    pipeline_register_calibrate_handler(calibrate_handler);
    pipeline_register_bench_handler(bench_handler);
    pipeline_register_assist_handler(assist_handler);
    pipeline_register_steer_rate_handler(steer_rate_handler);

    decoded_tag = boat_BoatMessage_detect_tag;
    pipeline_handle_incoming(input, sizeof(input));
    assert(detect_calls == 1);
    assert(manual_control_calls == 0);

    decoded_tag = boat_BoatMessage_training_log_tag;
    pipeline_handle_incoming(input, sizeof(input));
    assert(training_log_calls == 1);
    assert(manual_control_calls == 0);

    /* A calibrate command routes to its own handler, never manual-control
     * ingress -- calibration owns the actuators through the control task, not
     * the pipeline dispatch path. */
    decoded_tag = boat_BoatMessage_calibrate_tag;
    pipeline_handle_incoming(input, sizeof(input));
    assert(calibrate_calls == 1);
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
typedef struct { uint32_t state; float left_throttle; float right_throttle; float winch_speed; bool servo_power; float rudder_cmd; uint32_t rudder_pulse_us; bool rudder_saturated; bool assist_rudder; bool assist_motor_p; float yaw_target_dps; float yaw_filt_dps; uint32_t assist_request_id; } boat_MotorStatus;
typedef struct { uint32_t state; uint32_t level_index; float level_throttle; float trim_diff; float yaw_avg_dps; bool making_way; uint32_t points_done; } boat_CalibrateStatus;
typedef struct { uint32_t state; uint32_t kind; float base; uint32_t samples; uint32_t file_index; float elapsed_s; float learn_c; bool p_on; } boat_BenchStatus;
typedef struct { uint32_t kind; float base; float delta; float reset_c; } boat_BenchCommand;
typedef struct { bool p_on; bool rudder_assist; uint32_t request_id; } boat_AssistCommand;
typedef struct { float target_dps; } boat_SteerRateCommand;
#define boat_BoatMessage_steer_rate_tag 18
uint32_t motor_control_get_status(boat_MotorStatus *out);
uint32_t motor_control_get_calibrate_status(boat_CalibrateStatus *out);
uint32_t motor_control_get_bench_status(boat_BenchStatus *out);
void motor_control_bench_flush(void);
void motor_control_trimlearn_log(void);
float motor_control_trimlearn_c(void);
bool motor_control_p_assist_on(void);
""",
    "pipeline.h": r"""
#pragma once
#include "motor_control.h"
void pipeline_publish_motor_status(const boat_MotorStatus *status);
void pipeline_publish_calibrate_status(const boat_CalibrateStatus *status);
void pipeline_publish_bench_status(const boat_BenchStatus *status);
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
uint32_t motor_control_get_bench_status(boat_BenchStatus *out) {
    *out = (boat_BenchStatus){0};
    return 0;
}
void motor_control_bench_flush(void) { }
void motor_control_trimlearn_log(void) { }
float motor_control_trimlearn_c(void) { return 0.0f; }
bool motor_control_p_assist_on(void) { return false; }
void pipeline_publish_bench_status(const boat_BenchStatus *status) { (void)status; }
uint32_t motor_control_get_calibrate_status(boat_CalibrateStatus *out) {
    *out = (boat_CalibrateStatus){0};
    return 0;   /* generation 0 -> diagnostics publishes nothing (idle boat) */
}
void pipeline_publish_calibrate_status(const boat_CalibrateStatus *status) {
    (void)status;   /* never reached with generation 0 above */
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


def _function_body(src, signature):
    """Crude but sufficient: from the signature line to the first column-0 '}'."""
    start = src.index(signature)
    end = src.index("\n}", start)
    return src[start:end]


def test_bench_sd_write_never_happens_on_the_control_task():
    """The control loop is a 10 ms CRITICAL task. Writing a run to the SD card
    takes hundreds of milliseconds (tens of fopen/fclose on FAT, and the card
    shares a driver mutex with the C6 radio), so it must be handed to the
    core-1 diagnostics task instead. Doing it inline overruns the deadline by
    30-100x on the highest-priority safety task."""
    src = (ROOT / "main" / "motor_control.c").read_text()

    tick = _function_body(src, "static void bench_tick(int64_t now_us)")
    for forbidden in ("bench_write_csv", "fs_sdcard_write", "fs_sdcard_append"):
        assert forbidden not in tick, (
            "%s is called from bench_tick, which runs on the critical control "
            "task" % forbidden)

    flush = _function_body(src, "void motor_control_bench_flush(void)")
    assert "bench_write_csv" in flush, (
        "motor_control_bench_flush no longer performs the write")

    # and the diagnostics task must actually call it, or runs never get saved
    metrics = (ROOT / "main" / "runtime_metrics.c").read_text()
    assert "motor_control_bench_flush()" in metrics, (
        "nothing calls motor_control_bench_flush -- finished runs would never "
        "reach the card")



def _strip_c_comments(text):
    """Comments explaining WHY something is forbidden would otherwise trip the
    check for it. Test the code, not the prose about the code."""
    out, i, n = [], 0, len(text)
    while i < n:
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 2
        elif text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
        else:
            out.append(text[i]); i += 1
    return "".join(out)


def test_trim_learner_logging_never_happens_on_the_control_task():
    """One ESP_LOG line is ~100 bytes, and the UART writes it synchronously --
    about 9 ms at 115200 baud. That alone would blow the control task's 10 ms
    deadline, so the learner reports from the core-1 diagnostics task."""
    src = (ROOT / "main" / "motor_control.c").read_text()

    tick = _strip_c_comments(
        _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)"))
    assert "ESP_LOG" not in tick, (
        "trim_learn_tick logs, and it runs on the critical control task")

    metrics = (ROOT / "main" / "runtime_metrics.c").read_text()
    assert "motor_control_trimlearn_log()" in metrics, (
        "nothing calls motor_control_trimlearn_log -- the learner would run "
        "completely invisibly, with no way to tell on the water whether it "
        "moved, froze, or faulted")


def test_a_trim_change_reaches_the_motors_without_a_new_pilot_command():
    """The ESC write is gated on drive_changed. A learner that adjusts c while
    the pilot holds a steady throttle would then appear to run -- the log would
    show c moving -- while the motors kept the old split until the next command
    happened to arrive. The whole point is holding one throttle and watching
    the boat straighten, so that gate has to open on a trim move too."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    apply_ = _function_body(src, "static void control_apply_decision(control_decision_t *decision)")

    assert "if (decision->drive_changed || s_trim_moved || s_p_moved) {" in apply_, (
        "the drive block is gated on drive_changed alone -- an adapting trim "
        "would never reach the ESCs")
    assert "trim_learn_tick(decision);" in apply_, (
        "the learner is not ticked from the decision path")

    # ...and the decision path itself must run every cycle, not only when a
    # command arrives, or the learner ticks at the radio's rate instead of the
    # control loop's.
    cycle = _function_body(src, "static void run_control_cycle(bool scheduled, int64_t scheduled_us)")
    assert "control_apply_decision(&decision);" in cycle


def test_the_learner_is_frozen_unless_the_escs_are_actually_armed():
    """A throttle slider left up on a disarmed boat is not a measurement: the
    jets are dead and every degree the gyro reads is noise or someone carrying
    it. Integrating that is a random walk on c."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    assert "esc_driver_get_state() == ESC_STATE_ARMED" in tick
    assert "driving ? decision->throttle : 0.0f" in tick


def test_motorstatus_does_not_publish_at_the_control_rate():
    """MotorStatus is a STATUS channel, not a telemetry stream.

    The assisted rudder loop moves rudder_cmd / yaw_target_dps / yaw_filt_dps
    on every fresh fusion sample (~50 Hz). Comparing those in the
    publish-if-changed test put MotorStatus on the air at that rate, over a
    link already carrying ~20 Hz of telemetry. It congested, MotorStatus was
    what got dropped, and both assisted runs on 2026-09-02 aborted on a
    staleness gate while the boat was driving perfectly.

    The raw RUD_T20 runs recorded BEFORE that change completed cleanly at
    4.51 s with the same tool and the same gate, which is what pins the cause
    on the flood rather than on the link."""
    src = (ROOT / "main" / "motor_control.c").read_text()

    # The two comparisons must stay separate, with the fast floats OUT of the
    # discrete one -- putting any of them back restores the flood.
    disc = _function_body(src, "static bool motor_status_discrete_equal(")
    for fast in ("rudder_cmd", "yaw_target_dps", "yaw_filt_dps",
                 "rudder_pulse_us", "rudder_saturated"):
        assert fast not in disc, (
            "%s is back in the immediate-publish comparison -- MotorStatus "
            "will flood the radio again" % fast)
    for slow in ("state", "left_throttle", "right_throttle", "winch_speed",
                 "servo_power", "assist_rudder", "assist_motor_p"):
        assert slow in disc, "%s must still publish immediately" % slow

    cont = _function_body(src, "static bool motor_status_continuous_equal(")
    for fast in ("rudder_cmd", "yaw_target_dps", "yaw_filt_dps"):
        assert fast in cont

    # ...and the continuous path must actually be rate-limited.
    commit = _function_body(src, "static void status_commit_current(bool force)")
    assert "if (discrete_same) {" in commit
    # Pin the LIVE comparison, not merely a mention of the constant: an
    # `if (0 && ...)` still contains the name while publishing every time,
    # and an earlier version of this test accepted exactly that.
    assert ("            if (s_status_continuous_us != 0 &&\n"
            "                (esp_timer_get_time() - s_status_continuous_us)\n"
            "                    < MOTOR_STATUS_CONTINUOUS_MIN_INTERVAL_US) {\n"
            "                portEXIT_CRITICAL(&s_status_lock);\n"
            "                return;\n"
            "            }") in commit, (
        "the continuous rate limit is not a live guard that returns early")

    # The budget is stamped on EVERY publish, not only the continuous one.
    # Stamping it only inside the discrete_same branch leaves a stale mark
    # behind each discrete publish, so the next continuous change slips out
    # immediately -- which the first version of this fix did, while carrying a
    # comment claiming the opposite.
    # ...and outside the !force block, so a forced publish stamps it as well.
    forced = commit.split("if (!force) {", 1)[1]
    after = forced.split("\n    }\n", 1)[1]
    assert "s_status_continuous_us = esp_timer_get_time();" in after, (
        "the publish budget is not stamped on the discrete path, so a discrete "
        "change lets the next continuous one bypass the rate limit")

    # A rate that is not meaningfully slower than the source is no fix at all.
    m = re.search(r"#define MOTOR_STATUS_CONTINUOUS_MIN_INTERVAL_US\s+(\d+)", src)
    assert m, "the rate-limit interval is gone"
    assert int(m.group(1)) >= 50000, "faster than 20 Hz is back toward the flood"


def test_ordinary_forward_driving_is_not_bench_only():
    """The feature is "hold the throttle and the boat straightens out", not
    "press BASE and watch a number". Every other trim test drives the learner
    through a bench run, so if learning ever became conditional on one they
    would all still pass while the boat never trimmed itself on the water.

    Two things make ordinary driving work, and both are pinned here:
      1. the freeze is conditioned ONLY on calibration and non-BASE bench runs,
         so with neither active execution falls through to the update; and
      2. the throttle and steering handed to the learner come from the PILOT's
         decision, not from a bench field.

    tests/test_normal_driving_learns.c mirrors these exact lines to exercise
    the behaviour; this test is what stops that mirror drifting."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")

    # 1. the ONLY early return in the tick, and what it is conditioned on.
    freeze = "if (s_calibrating || (s_bench_active && !bench_learning)) {"
    assert freeze in tick
    assert tick.count("        return;") == 1, (
        "a second early return has appeared in trim_learn_tick -- ordinary "
        "driving may no longer reach trim_learn_update")

    # 2. the pilot's own command is what the learner measures.
    assert "bool steering = fabsf(decision->rudder) > 0.02f ||" in tick
    assert "float thr = driving ? decision->throttle : 0.0f;" in tick

    # 3. and those are the values actually handed over.
    assert ("s_trim_moved = trim_learn_update(&s_trim_learn, &s_trim_learn_cfg,\n"
            "                                     f.sequence, dt_s, yaw,\n"
            "                                     thr, steering, healthy);") in tick, (
        "the learner call no longer passes (thr, steering, healthy) -- the "
        "ordinary-driving mirror in test_normal_driving_learns.c is now wrong")

    # 4. the bench override is scoped to a bench run and nothing else, so it
    #    cannot quietly become the only way thr is ever non-zero.
    assert "if (bench_learning) {" in tick
    bench_branch = tick.split("if (bench_learning) {", 1)[1].split("    }", 1)[0]
    assert "thr = driving ? s_bench.base : 0.0f;" in bench_branch


def test_every_way_of_steering_the_boat_freezes_the_learner():
    """This boat turns three different ways, and the learner cannot tell a
    commanded turn from a motor imbalance. Miss one channel and it quietly
    learns the pilot's own steering into c and keeps it there.

      decision->rudder     differential thrust (folded into the drive command)
      decision->steer      the physical rudder servos -- a SEPARATE arbiter
                           channel, so a boat turning purely on its rudders has
                           decision->rudder == 0 and the old rudder-only test
                           saw a straight line
      decision->steer_raw  a raw microsecond pulse; submit_steer_raw() forces
                           value to 0.0f and carries only the pulse, so there
                           is no normalized position to threshold -- the flag
                           itself is the signal, the same call SAS makes"""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")

    assert ("bool steering = fabsf(decision->rudder) > 0.02f ||\n"
            "                    fabsf(decision->steer) > 0.02f ||\n"
            "                    decision->steer_raw;") in tick, (
        "the steering test no longer covers all three channels -- a turn on "
        "one of them would be learned as trim")

    # steer_raw must be a bare flag, never thresholded: its value is always 0.
    assert "fabsf(decision->steer_raw)" not in tick
    assert "decision->steer_raw > " not in tick

    # Both consumers must key off the same flag, or the slow and fast loops
    # would disagree about whether the boat is being steered.
    assert "thr, steering, healthy);" in tick, "the learner is not given `steering`"
    assert "!steering" in tick, "the P gate no longer excludes steering"


def test_a_left_or_right_run_never_teaches_the_learner():
    """A LEFT/RIGHT run drives the boat into a deliberate turn. That yaw is the
    perturbation the test applies, not an error -- learning from it would teach
    the trim to cancel the very thing being measured. Calibration owns the
    motors outright."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    assert "if (s_calibrating || (s_bench_active && !bench_learning)) {" in tick
    # pinned as the whole expression, not a substring: a `false &&` or a
    # negation slipped in front would leave every substring intact while
    # switching the learner off for BASE runs -- exactly the state this was
    # just fixed out of.
    assert ("const bool bench_learning = s_bench_active &&\n"
            "                                s_bench.kind == BENCH_KIND_BASE &&\n"
            "                                s_bench.state == BENCH_RUN;") in tick, (
        "bench_learning is no longer exactly "
        "`s_bench_active && kind == BASE && state == RUN`")
    assert "s_bench.state == BENCH_RUN" in tick, (
        "the learner would run through the motors-off baseline and coast, "
        "where the boat is not being driven at all")

    # ...and the freeze must forget when the last sample was. The step is
    # proportional to dt, so a remembered timestamp turns the pause itself into
    # a correction: 0.15 of c after a 30 s calibration -- into the clamp,
    # latched faulted, feature dead.
    freeze = tick.split(
        "if (s_calibrating || (s_bench_active && !bench_learning)) {",
        1)[1].split("}", 1)[0]
    assert "s_trim_last_capture_us = 0;" in freeze, (
        "the learner keeps a stale sample timestamp across the freeze, so the "
        "first sample afterwards carries the whole pause as dt")


def test_a_base_run_keeps_the_learner_live_and_feeds_it_the_bench_throttle():
    """A BASE run commands both jets equal, so it IS a straight-line demand and
    any yaw is exactly the error the learner removes. It has to stay live, or
    the one test the operator runs can never show the trim working.

    And it must read the BENCH's throttle: the tool zeroes the pilot throttle
    at the button press, so reading that would park the learner on the
    low-throttle gate for the whole run."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    assert "thr = driving ? s_bench.base : 0.0f;" in tick, (
        "the learner reads the pilot's throttle, which is 0 during a run")
    assert "steering = false;" in tick, (
        "a BASE run commands both jets equal -- it is not steering")

    # and the correction has to actually reach the jets during the run: it is
    # bench_tick that owns the ESCs then, so it must read c every tick
    bench = _function_body(src, "static void bench_tick(int64_t now_us)")
    assert "effective_trim_c(s_trim_learn.c)" in bench


def test_a_base_run_stays_comparable_with_every_earlier_run():
    """Run length is set by the POOL and by COMPARABILITY, never by the learner.

    Every run recorded before 2026-08-29 used 3 s. Changing it orphans ~200
    files, and being able to hold a new run against the old ones is the whole
    value of the BASE test.

    A 10 s run was tried so the learner could converge inside one run. Wrong
    twice: the hull reaches the wall at a median of 4 s, so 33% of the
    recording was post-contact against 19% at 3 s -- and it did not help
    either, because the learner keeps moving at ~50% of its maximum rate
    throughout but its steps cancel once the bouncing is symmetric.

        first 3 s: 0.0034 of c per second  |  next 7 s: 0.0010 per second

    c carries over between runs, so three short runs integrate like one long
    one."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    m = re.search(r"#define BENCH_RUN_US_BASE\s+(\d+)", src)
    assert m, "BENCH_RUN_US_BASE missing"
    base_s = int(m.group(1)) / 1e6
    assert base_s == 3.0, (
        "a BASE run is %.1f s, not the 3 s every earlier run used -- new files "
        "can no longer be compared against what is already on the card"
        % base_s)

    m2 = re.search(r"#define BENCH_RUN_US_SPLIT\s+(\d+)", src)
    assert int(m2.group(1)) == 3000000, "LEFT/RIGHT runs drifted off 3 s too"

    hdr = (ROOT / "main" / "bench_run.h").read_text()
    m = re.search(r"#define BENCH_MAX_SAMPLES\s+(\d+)", hdr)
    assert m
    need = int((0.5 + base_s + 1.0) * 100)      # 100 Hz control tick
    assert int(m.group(1)) >= need, (
        "the sample buffer holds %s but the profile is %d ticks -- the end of "
        "every run would be silently dropped" % (m.group(1), need))


def test_a_bench_run_measures_the_trim_the_boat_is_actually_running():
    """"Press BASE and see if it goes straight" has to be about the trim in
    force right now. With the learner on that is its current c, read fresh
    every tick so the correction reaches the jets DURING the run, and written
    into the CSV per sample so the file shows it moving."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void bench_tick(int64_t now_us)")
    assert "effective_trim_c(s_trim_learn.c)" in tick, (
        "a bench run would test the flashed trim while the boat drives the "
        "learned one")
    assert "esc_trim_lookup(s_esc_trim, s_esc_trim_count," in tick, (
        "the learner-off build must still take the static table")


def test_saving_a_run_does_not_hold_the_sd_lock_for_long():
    """Every flush in bench_write_csv is a full fopen/fwrite/fflush/fclose on
    FAT, and it takes the SD lock the C6 radio shares -- one mutex covers both
    slots in ESP-IDF's driver. The number of FLUSHES, not the byte total, sets
    how long the link goes quiet after a run, so the buffer has to grow with
    the profile or a longer run silently costs more radio downtime."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    hdr = (ROOT / "main" / "bench_run.h").read_text()

    chunk = int(re.search(r"static char chunk\[(\d+)\];", src).group(1))
    samples = int(re.search(r"#define BENCH_MAX_SAMPLES\s+(\d+)", hdr).group(1))

    row = 42          # "%.3f,%s,%.3f,%.3f,%.3f,%.4f\n" with an 8-char phase
    flushes = (samples * row) / float(chunk - 96)
    assert flushes <= 16, (
        "a full run takes ~%d SD flushes; each one blocks the radio's mutex, "
        "so keep it near a dozen" % flushes)


def test_the_learner_integrates_raw_gyro_not_a_baseline_corrected_one():
    """Subtracting the run's motors-off baseline was tried and made it worse.

    The baseline does not predict what the gyro does once the motors run --
    corr(raw, baseline) = -0.16 over one session and +0.05 over the next. So
    subtracting it removes no bias and adds a second noise source, and that one
    is a step held for a whole run rather than white noise, which is the worst
    thing to feed an integrator: sd went 0.40 -> 0.92, c's wander went
    0.129..0.206 -> 0.114..0.277, and it twice reached the clamp.

    Raw yaw is what actually tracks c: slope +11..+20 deg/s per unit c,
    r = +0.53..+0.83 across five level-fits."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    assert "yaw -= bench_baseline_yaw" not in tick, (
        "the baseline subtraction is back -- it was measured to double the "
        "noise and double c's wander")
    assert "const float yaw = f.yaw_rate;" in tick, (
        "the learner is no longer integrating the raw gyro")


def test_the_learner_reset_is_one_shot_and_only_on_an_accepted_run():
    """The reset exists for one experiment: start low, start high, see whether
    both reach the same c. Two properties make or break it.

    ONE SHOT: c carrying over between runs is the entire mechanism being
    measured. A reset that stayed latched would fire again on the next ordinary
    BASE run and wipe the convergence being observed.

    ONLY IF ACCEPTED: a refused start (disarmed, no SD, a run already going)
    must leave the learner untouched, or a button press that visibly did
    nothing would still have silently discarded everything it had learned."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void bench_tick(int64_t now_us)")

    # consumed where it is read, under the same lock that read it
    assert "s_bench_req_reset_c = 0.0f;             /* one shot, consumed here */" in tick, (
        "the reset request is not cleared when read -- it would fire again on "
        "the next ordinary BASE run")

    # the reset call must sit INSIDE the accepted-start branch
    accepted = tick.split("if (bench_start(&s_bench, (bench_kind_t)kind, base, delta, now_us)) {", 1)
    assert len(accepted) == 2, "bench_start acceptance branch not found"
    body = accepted[1].split("} else {", 1)[0]
    assert "trim_learn_reset(&s_trim_learn, &s_trim_learn_cfg" in body, (
        "the learner reset is not inside the accepted-start branch, so a "
        "refused run would still wipe the learner")

    # and the rejection path must not touch it
    rejected = tick.split("BENCH,start_rejected", 1)
    assert len(rejected) == 2
    assert "trim_learn_reset" not in rejected[1].split("}", 1)[0]


def test_an_ordinary_bench_run_never_resets_the_learner():
    """Every button except RESET must send 0, and the firmware must treat 0 as
    'leave it alone'. Otherwise each run restarts from the seed and the learner
    can never be seen to converge across runs."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void bench_tick(int64_t now_us)")
    assert "if (reset_c > 0.0f) {" in tick, (
        "the firmware does not treat 0 as 'no reset'")

    tool = (ROOT / "tools" / "espnow_drive.py").read_text()
    for kind in ("'left'", "'right'", "'both'"):
        assert "runBench(%s, 0)" % kind in tool, (
            "the %s button does not explicitly send reset_c 0" % kind)
    assert "reset_c=0.0" in tool, "send_bench does not default to no reset"


def test_the_learner_c_is_reported_back():
    """Without this the operator cannot tell what the boat is running, and the
    convergence experiment has no readout between runs."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    commit = _function_body(src, "static void bench_status_commit(void)")
    assert "st.learn_c    = motor_control_trimlearn_c();" in commit
    tool = (ROOT / "tools" / "espnow_drive.py").read_text()
    assert "'learn_c': float(bs.learn_c)" in tool
    assert "bench-learn-c" in tool


def test_the_p_assist_is_gated_on_every_condition_the_learner_uses():
    """The fast P correction must be as conservative as the slow learner about
    when it is allowed to act: armed, above minimum throttle, stick centred,
    gyro healthy, and the yaw below the impact-rejection threshold. A gate that
    is open wider than the learner's would let P act on samples the learner has
    already judged untrustworthy."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    for cond in ("s_p_assist_on", "healthy", "driving", "!steering",
                 "thr >= s_trim_learn_cfg.min_throttle",
                 "fabsf(yaw) <= TRIM_LEARN_REJECT_DPS"):
        assert cond in tick, "P gate is missing: %s" % cond
    # a closed gate must RESET, not decay -- otherwise OFF and gated-off differ
    assert "trim_assist_reset(&s_trim_assist);" in tick


def test_the_p_correction_never_touches_the_learned_c():
    """c is the boat's measured property and only the I learner may move it.
    P is added downstream, per sample, and is gone the moment its gate shuts."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    # the only writes to the learner's own c
    assert src.count("trim_learn_reset(&s_trim_learn") == 1
    assert src.count("trim_learn_init(&s_trim_learn") == 1
    # P reaches the mixer only through the sum
    assert "trim_assist_effective_c(learned_c, s_p_correction" in src
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    assert "s_trim_learn.c =" not in tick, "P is writing the learned c directly"


def test_a_changed_p_reaches_the_escs_without_a_new_pilot_command():
    """P changes on every fusion sample while the pilot holds a steady stick.
    Gated on drive_changed alone it would show in the log and never reach the
    motors -- the same hole the learner had."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    apply_ = _function_body(src, "static void control_apply_decision(control_decision_t *decision)")
    assert "if (decision->drive_changed || s_trim_moved || s_p_moved) {" in apply_
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    assert "if (s_p_correction != p_prev) s_p_moved = true;" in tick


def test_a_base_run_applies_p_but_a_split_run_does_not():
    """BASE commands both jets equal, so P's correction is meaningful there and
    the CSV records it. LEFT/RIGHT deliberately turn the boat; P must not fight
    the perturbation the test is applying."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    bench = _function_body(src, "static void bench_tick(int64_t now_us)")
    assert "effective_trim_c(s_trim_learn.c)" in bench, (
        "a BASE run drives the learned c without P")
    # P's gate rides on the learner's, which already bails for LEFT/RIGHT
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    assert "s_bench.kind == BENCH_KIND_BASE" in tick


def test_turning_p_off_returns_exactly_to_the_pre_p_path():
    """OFF is the control arm of the experiment. It has to be the same code
    path as before P existed, not a P that happens to output small numbers --
    so the correction is dropped on the transition rather than left to decay,
    and the clean value is pushed to the ESCs."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    # The RX task may only REQUEST. Every P state change belongs to the
    # control task, or a command can land mid-mix and leave the ESCs holding a
    # correction the gate has already revoked.
    h = _function_body(src,
                       "static void assist_command_handler(bool p_on, bool rudder_assist,")
    assert "s_p_assist_req_pending = true;" in h
    for forbidden in ("trim_assist_reset", "s_p_correction", "s_p_moved"):
        assert forbidden not in h, (
            "%s is written from the RX task; only the control task may touch "
            "P state" % forbidden)

    # ...and the control task applies it, clearing and remixing on an OFF
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    assert "s_p_assist_req_pending = false;" in tick
    assert "trim_assist_reset(&s_trim_assist);" in tick
    assert "if (s_p_correction != 0.0f) s_p_moved = true;" in tick, (
        "turning P off does not push the cleaned value out, so the boat would "
        "keep the last correction until the next pilot command")
    assert "static bool s_p_assist_on = false;" in src, "P must default OFF"


def test_p_is_cleared_on_every_path_that_takes_the_motors_away():
    """The bench/calibration early return sits BEFORE the P block, so without
    an explicit clear a LEFT/RIGHT run or a calibration keeps applying whatever
    correction was live when it started -- fighting the very perturbation the
    test is applying."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    tick = _function_body(src, "static void trim_learn_tick(const control_decision_t *decision)")
    early = tick.split('s_trim_why = "bench/cal owns the motors";', 1)
    assert len(early) == 2
    body = early[1].split("return;", 1)[0]
    assert "trim_assist_reset(&s_trim_assist);" in body, (
        "P is not cleared when a LEFT/RIGHT run or calibration takes the motors")
    assert "s_p_correction = 0.0f;" in body
    # Pinned with its CONDITION, not just the assignment: a `if (0)` in front
    # leaves every substring intact while silently dropping the remix.
    assert "if (s_p_correction != 0.0f) s_p_moved = true;" in \
        _strip_c_comments(body), "the cleared value is never remixed"


def test_the_bench_csv_records_what_p_did():
    """An A/B file has to be self-describing: whether P was on, what it saw,
    what it contributed and whether it was against its cap. None of that is
    recoverable from the motor commands alone."""
    src = (ROOT / "main" / "motor_control.c").read_text()
    assert "t_s,phase,yaw_dps,left,right,c,p_on,p_yaw,c_learn,p_corr,split,at_cap" in src
    write = _function_body(src, "static void bench_write_csv(void)")
    assert "smp->c_learn" in write, (
        "learned c is derived rather than logged -- c - p_corr only holds "
        "while neither the learner nor P is against a clamp, and both can be")
    assert "BENCH_P_AT_CAP" in write
