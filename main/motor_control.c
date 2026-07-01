#include "motor_control.h"
#include "drivers/esc_driver.h"
#include "drivers/winch_driver.h"
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

static void winch_command_handler(const boat_WinchCommand *cmd)
{
    winch_driver_set_speed(cmd->speed);
}

static void arm_task_fn(void *arg)
{
    bool do_arm = (bool)(intptr_t)arg;
    if (do_arm) {
        motor_control_arm();
    } else {
        motor_control_disarm();
    }
    vTaskDelete(NULL);
}

static void arm_command_handler(bool arm)
{
    ESP_LOGI(TAG, "%s command received", arm ? "Arm" : "Disarm");
    if (arm && esc_driver_get_state() != ESC_STATE_DISARMED) return;
    if (!arm && esc_driver_get_state() == ESC_STATE_DISARMED) return;
    xTaskCreate(arm_task_fn, "esc_arm", 2048, (void *)(intptr_t)arm, 5, NULL);
}

static void publish_status(void)
{
    boat_MotorStatus ms = boat_MotorStatus_init_zero;
    ms.state = (uint32_t)esc_driver_get_state();
    esc_driver_get_throttle(&ms.left_throttle, &ms.right_throttle);
    ms.winch_speed = winch_driver_get_speed();
    pipeline_publish_motor_status(&ms);
}

static void watchdog_cb(void *arg)
{
    (void)arg;
    static int tick = 0;

    if (esc_driver_get_state() == ESC_STATE_ARMED && ws_transport_client_count() == 0) {
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
        if (acted) {
            ESP_LOGW(TAG, "WS disconnected — stopping motors + winch");
        }
    }

    if (++tick % STATUS_DIVIDER == 0) {
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
    esp_err_t ret = esc_driver_arm();
    if (ret == ESP_OK) {
        winch_driver_set_power(true);   /* servo rail live only while armed */
    }
    return ret;
}

esp_err_t motor_control_disarm(void)
{
    s_was_nonzero = false;
    winch_driver_set_speed(0.0f);       /* stop the winch */
    winch_driver_set_power(false);      /* cut servo power */
    return esc_driver_disarm();
}
