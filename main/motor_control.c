#include "motor_control.h"
#include "arm_sequence.h"
#include "drivers/esc_driver.h"
#include "drivers/winch_driver.h"
#include "drivers/steer_driver.h"
#include "drivers/gps_driver.h"
#include "drivers/imu_driver.h"
#include "sensor_fusion.h"
#include "pipeline.h"
#include "control_arbiter.h"
#include "runtime_metrics.h"
#include "runtime_task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>

static const char *TAG = "MOTOR_CTL";

#define CONTROL_PERIOD_MS    10
#define HEADING_DIVIDER      10
#define STATUS_DIVIDER       10
#define ARMING_DURATION_US   3000000LL
#define ARM_REQUEST_QUEUE_LENGTH 4
#define ARM_ACTION_QUEUE_LENGTH 4

/* How long accepted manual-control traffic may go silent before failsafe.
 * Detect and all other decodable protobuf traffic are deliberately excluded. */
#define CONTROL_LINK_TIMEOUT_US CONTROL_DRIVE_TIMEOUT_US

/* New Kconfig symbols are absent until sdkconfig is regenerated. Keep safe
 * fallback defaults so this branch builds and dry-run works immediately. */
#ifndef CONFIG_HEADING_ASSIST_DRY_RUN
#define CONFIG_HEADING_ASSIST_DRY_RUN 1
#endif
#ifndef CONFIG_HEADING_ASSIST_BAND_DEG
#define CONFIG_HEADING_ASSIST_BAND_DEG "4.0"
#endif
#ifndef CONFIG_HEADING_ASSIST_CAPTURE_DELAY_MS
#define CONFIG_HEADING_ASSIST_CAPTURE_DELAY_MS 500
#endif
#ifndef CONFIG_HEADING_ASSIST_RUDDER_DEADBAND
#define CONFIG_HEADING_ASSIST_RUDDER_DEADBAND "0.08"
#endif
#ifndef CONFIG_HEADING_ASSIST_MAX_RUDDER
#define CONFIG_HEADING_ASSIST_MAX_RUDDER "0.25"
#endif
#ifndef CONFIG_HEADING_ASSIST_MAX_DIFF
#define CONFIG_HEADING_ASSIST_MAX_DIFF "0.06"
#endif

#define ASSIST_KP_MIN       0.012f
#define ASSIST_KP_BASE      0.022f
#define ASSIST_KP_MAX       0.050f
#define ASSIST_KD           0.030f
#define ASSIST_TRIM_RATE    0.002f   /* rudder units/sec while persistently outside band */
#define ASSIST_TRIM_MAX     0.120f
#define ASSIST_DIFF_GAIN    0.150f

/* ── Manual-driving control & safety model (settle it here, don't let it drift) ─
 *  Two independent power domains:
 *    • Thrusters (ESC pins 31/33): live ONLY in ARMED. Arming needs a GPS lock
 *      unless the dashboard sends force=true (bench override — OFF by default,
 *      a deliberate tick, so nothing arms unattended without a fix).
 *    • Servo rail (pin 36 → winch + both rudders): auto-powers on the first
 *      NON-ZERO winch/steer command, so manual driving has zero friction.
 *  Rail power rules:
 *    • Auto-power is suppressed after an explicit PWR-OFF (s_rail_cut) until an
 *      explicit PWR-ON — the operator's kill actually holds.
 *    • A zero/stop command never powers the rail (spring-back-to-0 must not
 *      re-energise it).
 *    • ARM powers the rail and clears the cut; DISARM cuts rail power.
 *    • Control-link-loss failsafe (10 ms control loop) zeroes throttle + winch,
 *      centres rudders, and DE-ENERGISES the rail — loss of the control link
 *      returns to a safe, unpowered state, not a hot rail holding torque
 *      forever. A failsafe rail cut is not latched; only an explicit PWR-OFF
 *      suppresses later non-zero steering/winch auto-power.
 * ───────────────────────────────────────────────────────────────────────────── */

static control_arbiter_t s_arbiter;
static portMUX_TYPE s_arbiter_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t s_last_control_rx_us = 0;
static bool s_control_failsafe = true;
static TaskHandle_t s_control_task = NULL;
static TaskHandle_t s_arm_sequence_task = NULL;
static QueueHandle_t s_arm_request_queue = NULL;
static QueueHandle_t s_arm_action_queue = NULL;

static boat_MotorStatus s_status_buffers[2];
static uint32_t s_status_generation;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static atomic_uintptr_t s_status_reader_task;

static bool s_arm_power_allowed;

typedef struct {
    arm_request_t request;
    bool force;
    int64_t requested_us;
} arm_request_message_t;

static inline bool control_link_alive(void)
{
    int64_t last;
    portENTER_CRITICAL(&s_arbiter_lock);
    last = s_last_control_rx_us;
    portEXIT_CRITICAL(&s_arbiter_lock);
    return last != 0 && esp_timer_get_time() - last < CONTROL_LINK_TIMEOUT_US;
}

/* True after an explicit PWR-OFF: suppresses auto-power until an explicit PWR-ON
 * (or ARM). Without it the rail re-energised on the very next command — even a
 * spring-back-to-zero winch stop — so the operator's kill never held. */
static bool s_rail_cut = false;

/* Last manual inputs, used only by the heading-assist dry-run logger for now.
 * The actual actuator path remains unchanged until dry-run is explicitly
 * promoted to active control. */
static float s_manual_left = 0.0f;
static float s_manual_right = 0.0f;
static float s_manual_rudder = 0.0f;

/* Heading assist dry-run state. */
static bool s_assist_cfg_loaded = false;
static float s_assist_band_deg = 4.0f;
static float s_assist_rudder_deadband = 0.08f;
static float s_assist_max_rudder = 0.25f;
static float s_assist_max_diff = 0.06f;
static bool s_assist_hold = false;
static bool s_assist_have_last_heading = false;
static float s_assist_target = 0.0f;
static float s_assist_last_heading = 0.0f;
static float s_assist_last_error = 0.0f;
static float s_assist_kp = ASSIST_KP_BASE;
static float s_assist_trim = 0.0f;
static int64_t s_assist_center_since_us = 0;
static int64_t s_assist_last_us = 0;

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static float wrap180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float parse_cfg_float(const char *s, float fallback, float lo, float hi)
{
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s || !isfinite(v)) {
        return fallback;
    }
    return clampf(v, lo, hi);
}

static void heading_assist_load_cfg_once(void)
{
    if (s_assist_cfg_loaded) return;
    s_assist_cfg_loaded = true;
    s_assist_band_deg = parse_cfg_float(CONFIG_HEADING_ASSIST_BAND_DEG, 4.0f, 0.5f, 30.0f);
    s_assist_rudder_deadband = parse_cfg_float(CONFIG_HEADING_ASSIST_RUDDER_DEADBAND, 0.08f, 0.01f, 0.50f);
    s_assist_max_rudder = parse_cfg_float(CONFIG_HEADING_ASSIST_MAX_RUDDER, 0.25f, 0.01f, 1.0f);
    s_assist_max_diff = parse_cfg_float(CONFIG_HEADING_ASSIST_MAX_DIFF, 0.06f, 0.0f, 0.5f);
}

static void heading_assist_reset(void)
{
    s_assist_hold = false;
    s_assist_center_since_us = 0;
    s_assist_have_last_heading = false;
    s_assist_last_us = 0;
    s_assist_last_error = 0.0f;
    s_assist_kp = ASSIST_KP_BASE;
    s_assist_trim = 0.0f;
}

static void heading_assist_dry_run_tick(void)
{
#if CONFIG_HEADING_ASSIST_DRY_RUN
    heading_assist_load_cfg_once();

    static uint32_t log_div = 0;
    const int64_t now = esp_timer_get_time();
    const bool log_now = (++log_div % STATUS_DIVIDER) == 0;

    if (!control_link_alive()) {
        heading_assist_reset();
        return;
    }

    if (!imu_icm_ok()) {
        heading_assist_reset();
        if (log_now) {
            ESP_LOGI(TAG, "CTRL_DRY,disabled,reason=imu_bad");
        }
        return;
    }

    FusionResult fusion = {0};
    fusion_get_result(&fusion);

    float dt = 0.0f;
    float yaw_rate = 0.0f;
    if (s_assist_last_us != 0) {
        dt = (float)(now - s_assist_last_us) / 1000000.0f;
        if (dt > 0.001f && dt < 1.0f && s_assist_have_last_heading) {
            yaw_rate = wrap180(fusion.heading - s_assist_last_heading) / dt;
        }
    }
    s_assist_last_us = now;
    s_assist_last_heading = fusion.heading;
    s_assist_have_last_heading = true;

    const bool mag_ok = imu_mag_ok();
    const bool centered = fabsf(s_manual_rudder) <= s_assist_rudder_deadband;

    if (!mag_ok) {
        s_assist_hold = false;
    } else if (!centered) {
        s_assist_hold = false;
        s_assist_center_since_us = 0;
    } else {
        if (s_assist_center_since_us == 0) {
            s_assist_center_since_us = now;
        }
        if (!s_assist_hold &&
            (now - s_assist_center_since_us) >=
                (int64_t)CONFIG_HEADING_ASSIST_CAPTURE_DELAY_MS * 1000LL) {
            s_assist_target = fusion.heading;
            s_assist_last_error = 0.0f;
            s_assist_hold = true;
            ESP_LOGI(TAG, "CTRL_DRY,capture,target=%.1f,band=+/-%.1f",
                     s_assist_target, s_assist_band_deg);
        }
    }

    float error = s_assist_hold ? wrap180(s_assist_target - fusion.heading) : 0.0f;
    float p_error = 0.0f;
    if (fabsf(error) > s_assist_band_deg) {
        p_error = copysignf(fabsf(error) - s_assist_band_deg, error);
    }

    if (s_assist_hold && mag_ok && dt > 0.001f && dt < 1.0f) {
        if (p_error != 0.0f) {
            s_assist_trim += ASSIST_TRIM_RATE * dt * copysignf(1.0f, p_error);
            s_assist_trim = clampf(s_assist_trim, -ASSIST_TRIM_MAX, ASSIST_TRIM_MAX);

            if (fabsf(error) > fabsf(s_assist_last_error) + 0.2f &&
                (error * s_assist_last_error) >= 0.0f) {
                s_assist_kp = clampf(s_assist_kp + 0.002f * dt, ASSIST_KP_MIN, ASSIST_KP_MAX);
            } else if ((error * s_assist_last_error) < 0.0f) {
                s_assist_kp = clampf(s_assist_kp - 0.006f, ASSIST_KP_MIN, ASSIST_KP_MAX);
                s_assist_trim *= 0.98f;
            }
        }
        s_assist_last_error = error;
    }

    const float correction = s_assist_hold
        ? (s_assist_kp * p_error) - (ASSIST_KD * yaw_rate) + s_assist_trim
        : 0.0f;
    const float rudder_assist = clampf(correction, -s_assist_max_rudder, s_assist_max_rudder);
    const float diff_assist = clampf(correction * ASSIST_DIFF_GAIN, -s_assist_max_diff, s_assist_max_diff);
    const float left_would = clampf(s_manual_left + diff_assist, 0.0f, 1.0f);
    const float right_would = clampf(s_manual_right - diff_assist, 0.0f, 1.0f);
    const float rudder_would = clampf(s_manual_rudder + rudder_assist, -1.0f, 1.0f);

    gps_fix_t gps = {0};
    gps_driver_get_fix(&gps);

    if (log_now) {
        ESP_LOGI(TAG,
                 "CTRL_DRY,%s,tgt=%.1f,hdg=%.1f,err=%.1f,yaw=%.1f,band=%.1f,kp=%.3f,trim=%.3f,user_rud=%.2f,rud_would=%.2f,diff=%.3f,L=%.2f,R=%.2f,mag=%d,gps_v=%.2f",
                 s_assist_hold ? "hold" : (centered ? "capture_wait" : "override"),
                 s_assist_target, fusion.heading, error, yaw_rate, s_assist_band_deg,
                 s_assist_kp, s_assist_trim, s_manual_rudder, rudder_would,
                 diff_assist, left_would, right_would, mag_ok ? 1 : 0, gps.speed_mps);
    }
#endif
}

static bool motor_status_equal(const boat_MotorStatus *a, const boat_MotorStatus *b)
{
    return a->state == b->state &&
           a->left_throttle == b->left_throttle &&
           a->right_throttle == b->right_throttle &&
           a->winch_speed == b->winch_speed &&
           a->servo_power == b->servo_power;
}

static void status_commit_current(bool force)
{
    boat_MotorStatus status = boat_MotorStatus_init_zero;
    status.state = (uint32_t)esc_driver_get_state();
    esc_driver_get_throttle(&status.left_throttle, &status.right_throttle);
    status.winch_speed = winch_driver_get_speed();
    status.servo_power = winch_driver_get_power();

    portENTER_CRITICAL(&s_status_lock);
    uint32_t generation = s_status_generation;
    if (!force && motor_status_equal(&status, &s_status_buffers[generation & 1U])) {
        portEXIT_CRITICAL(&s_status_lock);
        return;
    }

    uint32_t next = generation + 1U;
    s_status_buffers[next & 1U] = status;
    s_status_generation = next;
    portEXIT_CRITICAL(&s_status_lock);

    TaskHandle_t reader = (TaskHandle_t)atomic_load_explicit(&s_status_reader_task,
                                                             memory_order_acquire);
    if (reader) xTaskNotifyGive(reader);
}

uint32_t motor_control_get_status(boat_MotorStatus *out)
{
    if (!out) return 0;
    atomic_store_explicit(&s_status_reader_task, (uintptr_t)xTaskGetCurrentTaskHandle(),
                          memory_order_release);

    portENTER_CRITICAL(&s_status_lock);
    uint32_t generation = s_status_generation;
    *out = s_status_buffers[generation & 1U];
    portEXIT_CRITICAL(&s_status_lock);
    return generation;
}

static void notify_link_rx_locked(int64_t received_us)
{
    if (received_us > s_last_control_rx_us) s_last_control_rx_us = received_us;

    control_drive_proposal_t *drive = &s_arbiter.drive[CONTROL_SOURCE_MANUAL];
    if (drive->valid && received_us > drive->rx_us) drive->rx_us = received_us;
}

void motor_control_notify_link_rx(int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
}

static void count_submit_result(control_submit_result_t result, bool overwrote)
{
    if (result == CONTROL_ACCEPTED && overwrote) {
        runtime_metrics_count(RUNTIME_TASK_CONTROL, RUNTIME_EVENT_COMMAND_OVERWRITE);
    } else if (result != CONTROL_ACCEPTED) {
        runtime_metrics_count(RUNTIME_TASK_CONTROL, RUNTIME_EVENT_INVALID_COMMAND);
    }
}

/* This module is the manual transport boundary. Source-less discrete arbiter
 * events are reachable only through this static wrapper, never from a future
 * waypoint or ML producer. */
static control_submit_result_t manual_transport_control_arbiter_submit_drive(
    float throttle, float rudder, int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool overwrote = s_arbiter.drive[CONTROL_SOURCE_MANUAL].changed;
    control_submit_result_t result = control_arbiter_submit_drive(
        &s_arbiter, CONTROL_SOURCE_MANUAL, throttle, rudder, received_us);
    if (result == CONTROL_ACCEPTED) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    count_submit_result(result, overwrote);
    return result;
}

static control_submit_result_t manual_transport_control_arbiter_submit_winch(
    float winch, int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool overwrote = s_arbiter.winch.changed;
    control_submit_result_t result = control_arbiter_submit_winch(
        &s_arbiter, CONTROL_SOURCE_MANUAL, winch, received_us);
    if (result == CONTROL_ACCEPTED) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    count_submit_result(result, overwrote);
    return result;
}

static control_submit_result_t manual_transport_control_arbiter_submit_steer(
    float steer, int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool overwrote = s_arbiter.steer.changed;
    control_submit_result_t result = control_arbiter_submit_steer(
        &s_arbiter, CONTROL_SOURCE_MANUAL, steer, received_us);
    if (result == CONTROL_ACCEPTED) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    count_submit_result(result, overwrote);
    return result;
}

static control_submit_result_t manual_transport_control_arbiter_submit_steer_raw(
    uint32_t pulse_us, int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool overwrote = s_arbiter.steer.changed;
    control_submit_result_t result = control_arbiter_submit_steer_raw(
        &s_arbiter, CONTROL_SOURCE_MANUAL, pulse_us, received_us);
    if (result == CONTROL_ACCEPTED) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    count_submit_result(result, overwrote);
    return result;
}

static bool manual_transport_control_arbiter_push_event(control_event_kind_t event,
                                                        int64_t received_us)
{
    portENTER_CRITICAL(&s_arbiter_lock);
    bool accepted = control_arbiter_push_event(&s_arbiter, event);
    if (accepted) notify_link_rx_locked(received_us);
    portEXIT_CRITICAL(&s_arbiter_lock);
    if (!accepted) runtime_metrics_count(RUNTIME_TASK_CONTROL, RUNTIME_EVENT_INVALID_COMMAND);
    return accepted;
}

static void motor_command_handler(const boat_MotorCommand *cmd)
{
    float left = cmd->left != 0.0f || cmd->right != 0.0f
               ? cmd->left : cmd->throttle + cmd->rudder;
    float right = cmd->left != 0.0f || cmd->right != 0.0f
                ? cmd->right : cmd->throttle - cmd->rudder;
    if (fmaxf(fabsf(left), fabsf(right)) > 1.0f) {
        float scale = fmaxf(fabsf(left), fabsf(right));
        left /= scale;
        right /= scale;
    }

    float throttle = 0.5f * (left + right);
    float rudder = 0.5f * (left - right);
    manual_transport_control_arbiter_submit_drive(throttle, rudder,
                                                   esp_timer_get_time());
}

static void winch_command_handler(const boat_WinchCommand *cmd)
{
    manual_transport_control_arbiter_submit_winch(cmd->speed, esp_timer_get_time());
}

static void steer_command_handler(const boat_SteerCommand *cmd)
{
    float steer = 0.0f;
    if (cmd->left != 0.0f && cmd->right != 0.0f) {
        steer = 0.5f * (cmd->left + cmd->right);
    } else {
        steer = (cmd->left != 0.0f) ? cmd->left : cmd->right;
    }
    manual_transport_control_arbiter_submit_steer(steer, esp_timer_get_time());
}

static void steer_raw_command_handler(const boat_SteerRawCommand *cmd)
{
    manual_transport_control_arbiter_submit_steer_raw(cmd->pulse_us,
                                                       esp_timer_get_time());
}

static void servo_power_command_handler(bool on)
{
    control_event_kind_t event = on ? CONTROL_EVENT_SERVO_POWER_ON
                                    : CONTROL_EVENT_SERVO_POWER_OFF;
    if (manual_transport_control_arbiter_push_event(event, esp_timer_get_time()) &&
        !on && s_control_task) {
        xTaskNotifyGive(s_control_task);
    }
}

static void arm_command_handler(bool arm, bool force)
{
    ESP_LOGI(TAG, "%s command received%s", arm ? "Arm" : "Disarm",
             (arm && force) ? " (GPS override)" : "");
    control_event_kind_t event = arm
        ? (force ? CONTROL_EVENT_FORCE_ARM : CONTROL_EVENT_ARM)
        : CONTROL_EVENT_DISARM;
    if (manual_transport_control_arbiter_push_event(event, esp_timer_get_time()) &&
        !arm && s_control_task) {
        xTaskNotifyGive(s_control_task);
    }
}

static bool submit_arm_request(arm_request_t request, bool force, int64_t requested_us)
{
    arm_request_message_t message = {
        .request = request,
        .force = force,
        .requested_us = requested_us,
    };

    BaseType_t queued;
    if (request == ARM_REQUEST_DISARM) {
        xQueueReset(s_arm_request_queue);
        queued = xQueueSendToFront(s_arm_request_queue, &message, 0);
    } else {
        queued = xQueueSend(s_arm_request_queue, &message, 0);
    }
    if (queued != pdTRUE) {
        runtime_metrics_count(RUNTIME_TASK_ARM_SEQUENCE,
                              RUNTIME_EVENT_INVALID_COMMAND);
        ESP_LOGE(TAG, "ArmSeq request queue full");
        return false;
    }
    return true;
}

static void publish_arm_action(arm_action_t action)
{
    if (action == ARM_ACTION_REJECT_NO_GPS) {
        ESP_LOGW(TAG, "Arm refused — waiting for GPS lock (use override to bypass)");
        return;
    }
    if (action == ARM_ACTION_NONE) return;

    BaseType_t queued;
    if (action == ARM_ACTION_DISARM) {
        xQueueReset(s_arm_action_queue);
        queued = xQueueSendToFront(s_arm_action_queue, &action, 0);
    } else {
        queued = xQueueSend(s_arm_action_queue, &action, 0);
    }
    if (queued != pdTRUE) {
        runtime_metrics_count(RUNTIME_TASK_ARM_SEQUENCE,
                              RUNTIME_EVENT_INVALID_COMMAND);
        ESP_LOGE(TAG, "ArmSeq action queue full");
        return;
    }
    if (s_control_task) xTaskNotifyGive(s_control_task);
}

static TickType_t arm_sequence_wait_ticks(const arm_sequence_t *sequence,
                                          int64_t now_us)
{
    if (sequence->state != ARM_SEQUENCE_ARMING) return portMAX_DELAY;
    if (sequence->deadline_us <= now_us) return 0;

    uint64_t remaining_ms = (uint64_t)(sequence->deadline_us - now_us + 999) / 1000;
    TickType_t ticks = pdMS_TO_TICKS(remaining_ms);
    return ticks == 0 ? 1 : ticks;
}

static void task_arm_sequence(void *arg)
{
    (void)arg;
    arm_sequence_t sequence;
    arm_sequence_init(&sequence, ARMING_DURATION_US);

    for (;;) {
        arm_request_message_t message;
        int64_t before_wait_us = esp_timer_get_time();
        TickType_t wait = arm_sequence_wait_ticks(&sequence, before_wait_us);
        if (xQueueReceive(s_arm_request_queue, &message, wait) == pdTRUE) {
            arm_sequence_request(&sequence, message.request, message.force,
                                 message.requested_us);
        }

        int64_t now_us = esp_timer_get_time();
        bool gps_locked = false;
        if (sequence.state == ARM_SEQUENCE_ARM_PENDING) {
            gps_locked = sequence.force || gps_driver_has_lock();
        }
        publish_arm_action(arm_sequence_step(&sequence, now_us, gps_locked));
    }
}

static bool control_apply_arm_action(control_decision_t *decision, bool safe_stop,
                                     bool *changed)
{
    arm_action_t action;
    if (xQueueReceive(s_arm_action_queue, &action, 0) != pdTRUE) return false;

    switch (action) {
    case ARM_ACTION_BEGIN:
        if (safe_stop || !s_arm_power_allowed) {
            submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
        } else if (esc_driver_arm_begin() == ESP_OK) {
            *changed = true;
        } else {
            submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
        }
        break;

    case ARM_ACTION_COMPLETE:
        if (safe_stop || !s_arm_power_allowed) {
            if (esc_driver_get_state() != ESC_STATE_DISARMED) {
                esc_driver_disarm();
                *changed = true;
            }
            submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
        } else if (esc_driver_arm_complete() == ESP_OK) {
            decision->servo_power_on = true;
            *changed = true;
        } else {
            submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
        }
        break;

    case ARM_ACTION_DISARM:
        if (esc_driver_get_state() != ESC_STATE_DISARMED) {
            esc_driver_disarm();
            *changed = true;
        }
        decision->arm = false;
        decision->force_arm = false;
        decision->disarm = true;
        return true;

    case ARM_ACTION_NONE:
    case ARM_ACTION_REJECT_NO_GPS:
    default:
        break;
    }
    return false;
}

static void control_apply_decision(control_decision_t *decision)
{
    bool changed = false;
    bool explicit_off = decision->servo_power_off;
    bool safe_stop = explicit_off || decision->disarm || decision->failsafe;
    bool internal_disarm = control_apply_arm_action(decision, safe_stop, &changed);
    safe_stop = explicit_off || decision->disarm || decision->failsafe;

    if (explicit_off) {
        s_rail_cut = true;
        s_arm_power_allowed = false;
        if (winch_driver_get_power()) {
            winch_driver_set_power(false);
            changed = true;
        }
        if (winch_driver_get_speed() != 0.0f) {
            winch_driver_set_speed(0.0f);
            changed = true;
        }
        if (steer_driver_get() != 1.0f) {
            steer_driver_set(1.0f);
            changed = true;
        }
        s_manual_rudder = 1.0f;
        heading_assist_reset();
    } else if (decision->disarm || decision->failsafe) {
        s_arm_power_allowed = false;
        float left;
        float right;
        esc_driver_get_throttle(&left, &right);
        if (left != 0.0f || right != 0.0f) {
            esc_driver_set_throttle(0.0f, 0.0f);
            changed = true;
        }
        if (winch_driver_get_speed() != 0.0f) {
            winch_driver_set_speed(0.0f);
            changed = true;
        }
        if (steer_driver_get() != 0.0f) {
            steer_driver_set(0.0f);
            changed = true;
        }
        if (winch_driver_get_power()) {
            winch_driver_set_power(false);
            changed = true;
        }
        s_manual_left = 0.0f;
        s_manual_right = 0.0f;
        s_manual_rudder = 0.0f;
        heading_assist_reset();
        if (decision->failsafe && changed) {
            ESP_LOGW(TAG, "Control link lost — throttle 0, winch 0, rudders centred, servo rail cut");
        }
    }

    if (decision->disarm && !internal_disarm) {
        submit_arm_request(ARM_REQUEST_DISARM, false, esp_timer_get_time());
    }

    if (!safe_stop) {
        if (decision->arm || decision->force_arm) {
            s_arm_power_allowed = true;
            submit_arm_request(ARM_REQUEST_ARM, decision->force_arm,
                               esp_timer_get_time());
        }

        if (decision->servo_power_on) {
            s_rail_cut = false;
            s_arm_power_allowed = true;
            if (!winch_driver_get_power()) {
                steer_driver_reassert();
                winch_driver_set_power(true);
                changed = true;
            }
        }

        if (decision->drive_changed) {
            float left;
            float right;
            esc_driver_get_throttle(&left, &right);
            if (left != decision->left || right != decision->right) {
                esc_driver_set_throttle(decision->left, decision->right);
                changed = true;
            }
            s_manual_left = clampf(decision->left, 0.0f, 1.0f);
            s_manual_right = clampf(decision->right, 0.0f, 1.0f);
        }

        if (decision->winch_changed) {
            if (decision->winch != 0.0f && !s_rail_cut && !winch_driver_get_power()) {
                steer_driver_reassert();
                winch_driver_set_power(true);
                changed = true;
            }
            if (winch_driver_get_speed() != decision->winch) {
                winch_driver_set_speed(decision->winch);
                changed = true;
            }
        }

        if (decision->steer_changed) {
            bool needs_power = decision->steer_raw || decision->steer != 0.0f;
            if (needs_power && !s_rail_cut && !winch_driver_get_power()) {
                steer_driver_reassert();
                winch_driver_set_power(true);
                changed = true;
            }
            if (decision->steer_raw) {
                steer_driver_set_raw_us(decision->steer_raw_us);
                changed = true;
            } else if (steer_driver_get() != decision->steer) {
                steer_driver_set(decision->steer);
                changed = true;
            }
            s_manual_rudder = decision->steer_raw
                ? s_manual_rudder : clampf(decision->steer, -1.0f, 1.0f);
        }
    }

    status_commit_current(changed);
}

static void run_control_cycle(bool scheduled, int64_t scheduled_us)
{
    static uint8_t heading_divider = 0;
    int64_t start_us = esp_timer_get_time();
    control_decision_t decision;

    runtime_metrics_cycle_begin(RUNTIME_TASK_CONTROL, (uint64_t)scheduled_us,
                                (uint64_t)start_us);
    portENTER_CRITICAL(&s_arbiter_lock);
    control_arbiter_decide(&s_arbiter, start_us, &decision);
    bool failsafe = s_last_control_rx_us == 0 ||
                    start_us - s_last_control_rx_us >= CONTROL_LINK_TIMEOUT_US;
    if (failsafe != s_control_failsafe) decision.drive_changed = true;
    decision.failsafe = failsafe;
    s_control_failsafe = failsafe;
    portEXIT_CRITICAL(&s_arbiter_lock);

    control_apply_decision(&decision);
    status_commit_current(false);

    if (scheduled && ++heading_divider == HEADING_DIVIDER) {
        heading_divider = 0;
        heading_assist_dry_run_tick();
    }
    runtime_metrics_cycle_end(RUNTIME_TASK_CONTROL, (uint64_t)esp_timer_get_time());
}

static void task_control(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    TickType_t next = xTaskGetTickCount();
    int64_t scheduled_us = esp_timer_get_time();

    for (;;) {
        run_control_cycle(true, scheduled_us);
        next += period;
        scheduled_us += CONTROL_PERIOD_MS * 1000LL;

        for (;;) {
            TickType_t now = xTaskGetTickCount();
            if (next <= now) {
                if (next < now) {
                    runtime_metrics_count(RUNTIME_TASK_CONTROL,
                                          RUNTIME_EVENT_DEADLINE_MISS);
                    next = now;
                    scheduled_us = esp_timer_get_time();
                }
                break;
            }
            if (ulTaskNotifyTake(pdTRUE, next - now) == 0) break;
            run_control_cycle(false, esp_timer_get_time());
        }
    }
}

esp_err_t motor_control_init_hw(void)
{
    esp_err_t ret = esc_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESC driver init failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

void motor_control_disarm(void)
{
    s_arm_power_allowed = false;
    (void)esc_driver_disarm();
    (void)winch_driver_set_speed(0.0f);
    (void)winch_driver_set_power(false);
    (void)steer_driver_set(0.0f);
}

esp_err_t motor_control_init(void)
{
    control_arbiter_init(&s_arbiter);
    s_last_control_rx_us = 0;
    s_control_failsafe = true;
    s_rail_cut = false;
    s_status_generation = 0;
    atomic_store(&s_status_reader_task, (uintptr_t)NULL);
    s_arm_power_allowed = false;
    s_arm_request_queue = xQueueCreate(ARM_REQUEST_QUEUE_LENGTH,
                                       sizeof(arm_request_message_t));
    s_arm_action_queue = xQueueCreate(ARM_ACTION_QUEUE_LENGTH,
                                      sizeof(arm_action_t));
    if (!s_arm_request_queue || !s_arm_action_queue) return ESP_ERR_NO_MEM;
    status_commit_current(true);

    pipeline_register_motor_handler(motor_command_handler);
    pipeline_register_arm_handler(arm_command_handler);
    pipeline_register_winch_handler(winch_command_handler);
    pipeline_register_steer_handler(steer_command_handler);
    pipeline_register_servo_power_handler(servo_power_command_handler);
    pipeline_register_steer_raw_handler(steer_raw_command_handler);

    esp_err_t task_error = runtime_task_create(RUNTIME_TASK_ARM_SEQUENCE,
                                               task_arm_sequence, NULL,
                                               &s_arm_sequence_task);
    if (task_error != ESP_OK) {
        motor_control_disarm();
        (void)winch_driver_set_power(false);
        ESP_LOGE(TAG, "critical task failed to start: ArmSeq");
        return task_error;
    }
    task_error = runtime_task_create(RUNTIME_TASK_CONTROL, task_control,
                                     NULL, &s_control_task);
    if (task_error != ESP_OK) {
        motor_control_disarm();
        (void)winch_driver_set_power(false);
        ESP_LOGE(TAG, "critical task failed to start: Control");
        return task_error;
    }

    ESP_LOGI(TAG, "Motor control initialized (ControlTask 10 ms, persistent ArmSeq)");
    return ESP_OK;
}
