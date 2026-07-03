#include "motor_control.h"
#include "drivers/esc_driver.h"
#include "drivers/winch_driver.h"
#include "drivers/steer_driver.h"
#include "drivers/gps_driver.h"
#include "pipeline.h"
#include "transports/ws_transport.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>

static const char *TAG = "MOTOR_CTL";

#define WATCHDOG_INTERVAL_US (250 * 1000)
#define STATUS_DIVIDER       4

static bool s_was_nonzero = false;
static esp_timer_handle_t s_watchdog = NULL;

/* Set when motor/servo state changes so the next 250ms watchdog tick publishes
 * status immediately (vs the ~1s cadence). Only the timer publishes it: doing so
 * from an incoming-command handler would deadlock — that path holds the shared
 * pipeline envelope mutex that publish also takes. */
static volatile bool s_status_dirty = false;

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

/* Winch + rudders share the pin-36 servo rail. Zero-friction manual driving:
 * the first command auto-powers the rail, so a servo moves the instant the user
 * touches a control — no arming, no separate power switch to find first. */
static void ensure_servo_rail(void)
{
    if (!winch_driver_get_power()) {
        winch_driver_set_power(true);
        s_status_dirty = true;   /* reflect PWR-on to the dashboard next tick */
    }
}

static void winch_command_handler(const boat_WinchCommand *cmd)
{
    ensure_servo_rail();
    winch_driver_set_speed(cmd->speed);
}

static void steer_command_handler(const boat_SteerCommand *cmd)
{
    ensure_servo_rail();
    steer_driver_set(cmd->left, cmd->right);
}

/* Explicit servo-rail switch — mainly to CUT power (a command auto-powers it on).
 * Turning it off also zeroes the commanded winch/steer so reported state stays
 * honest. */
static void servo_power_command_handler(bool on)
{
    if (winch_driver_set_power(on) == ESP_OK && !on) {
        winch_driver_set_speed(0.0f);   /* rail off ⇒ make commanded state match */
        steer_driver_set(0.0f, 0.0f);
    }
    s_status_dirty = true;
}

static void arm_task_fn(void *arg)
{
    intptr_t v = (intptr_t)arg;
    bool do_arm = (v & 1) != 0;
    bool force  = (v & 2) != 0;
    if (do_arm) {
        motor_control_arm(force);
    } else {
        motor_control_disarm();
    }
    vTaskDelete(NULL);
}

static void arm_command_handler(bool arm, bool force)
{
    ESP_LOGI(TAG, "%s command received%s", arm ? "Arm" : "Disarm",
             (arm && force) ? " (GPS override)" : "");
    if (arm && esc_driver_get_state() != ESC_STATE_DISARMED) return;
    if (!arm && esc_driver_get_state() == ESC_STATE_DISARMED) return;
    intptr_t v = (arm ? 1 : 0) | (force ? 2 : 0);
    xTaskCreate(arm_task_fn, "esc_arm", 2048, (void *)v, 5, NULL);
}

static void publish_status(void)
{
    boat_MotorStatus ms = boat_MotorStatus_init_zero;
    ms.state = (uint32_t)esc_driver_get_state();
    esc_driver_get_throttle(&ms.left_throttle, &ms.right_throttle);
    ms.winch_speed = winch_driver_get_speed();
    ms.servo_power = winch_driver_get_power();
    pipeline_publish_motor_status(&ms);
}

static void watchdog_cb(void *arg)
{
    (void)arg;
    static int tick = 0;

    /* Failsafe on WS loss: covers ARMED, and also the bench case where the
     * servo rail was switched on manually while disarmed. */
    if ((esc_driver_get_state() == ESC_STATE_ARMED || winch_driver_get_power()) &&
        ws_transport_client_count() == 0) {
        bool acted = false;
        if (s_was_nonzero) {
            esc_driver_set_throttle(0.0f, 0.0f);
            s_was_nonzero = false;
            acted = true;
        }
        if (winch_driver_get_speed() != 0.0f) {
            winch_driver_set_speed(0.0f);
            acted = true;
        }
        if (steer_driver_get_left() != 0.0f || steer_driver_get_right() != 0.0f) {
            steer_driver_set(0.0f, 0.0f);
            acted = true;
        }
        if (acted) {
            ESP_LOGW(TAG, "WS disconnected — stopping motors + winch, centering rudder");
        }
    }

    ++tick;
    if (s_status_dirty || (tick % STATUS_DIVIDER) == 0) {
        s_status_dirty = false;
        publish_status();
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

esp_err_t motor_control_init(void)
{
    pipeline_register_motor_handler(motor_command_handler);
    pipeline_register_arm_handler(arm_command_handler);
    pipeline_register_winch_handler(winch_command_handler);
    pipeline_register_steer_handler(steer_command_handler);
    pipeline_register_servo_power_handler(servo_power_command_handler);

    const esp_timer_create_args_t timer_args = {
        .callback = watchdog_cb,
        .name     = "motor_wd",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_watchdog), TAG, "timer_create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_watchdog, WATCHDOG_INTERVAL_US), TAG, "timer_start");

    ESP_LOGI(TAG, "Motor control initialized (status ~1 Hz)");
    return ESP_OK;
}

esp_err_t motor_control_arm(bool force)
{
    if (!force && !gps_driver_has_lock()) {
        ESP_LOGW(TAG, "Arm refused — waiting for GPS lock (use override to bypass)");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = esc_driver_arm();
    if (ret == ESP_OK) {
        winch_driver_set_power(true);   /* arming always powers the servo rail */
    }
    s_status_dirty = true;
    return ret;
}

esp_err_t motor_control_disarm(void)
{
    s_was_nonzero = false;
    winch_driver_set_speed(0.0f);       /* stop the winch */
    winch_driver_set_power(false);      /* cut servo power */
    steer_driver_set(0.0f, 0.0f);       /* rail is off — command state to match */
    s_status_dirty = true;
    return esc_driver_disarm();
}
