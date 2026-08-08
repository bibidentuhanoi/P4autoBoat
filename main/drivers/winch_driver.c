#include "winch_driver.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sdkconfig.h"

static const char *TAG = "WINCH";

/* Kconfig `bool default n` leaves the symbol UNDEFINED (not 0) when disabled,
 * which is fine in #if but breaks a runtime C expression. Normalise to 0/1. */
#ifdef CONFIG_SERVO_ENABLE_ACTIVE_LOW
#define WINCH_SERVO_ACTIVE_LOW 1
#else
#define WINCH_SERVO_ACTIVE_LOW 0
#endif

/* Own MCPWM timer/operator in group 0 (ESC uses group 0 too, but a separate
 * timer/operator index — MCPWM has 3 of each per group). 1 MHz => 1 us/tick. */
#define WINCH_MCPWM_GROUP      0
#define WINCH_TIMER_RESOLUTION 1000000U
#define WINCH_PERIOD_TICKS     (WINCH_TIMER_RESOLUTION / CONFIG_WINCH_PWM_FREQ_HZ)

static mcpwm_timer_handle_t s_timer  = NULL;
static mcpwm_oper_handle_t  s_oper   = NULL;
static mcpwm_cmpr_handle_t  s_cmp    = NULL;
static mcpwm_gen_handle_t   s_gen    = NULL;
static SemaphoreHandle_t    s_mutex  = NULL;
static float                s_speed  = 0.0f;
static bool                 s_power  = false;
static bool                 s_inited = false;

/* Map speed [-1,1] -> pulse us. 0 -> neutral (stop), +1 -> max (CW/down), -1 -> min (CCW/up).
 * Dead-band compensated: a continuous-rotation servo ignores pulses within ~db us of
 * neutral (the winch sat dead until ~9% slider). So exactly 0 = neutral, but ANY non-zero
 * command jumps straight past the dead-band — the winch responds at the very bottom of the
 * slider and the whole |speed| 0..1 maps to real motion instead of wasting the first ~9%. */
static uint32_t speed_to_us(float s)
{
    if (s < -1.0f) s = -1.0f;
    if (s >  1.0f) s =  1.0f;
    uint32_t neutral = CONFIG_WINCH_PULSE_NEUTRAL_US;
    uint32_t min_us  = CONFIG_WINCH_PULSE_MIN_US;
    uint32_t max_us  = CONFIG_WINCH_PULSE_MAX_US;
    uint32_t db      = CONFIG_WINCH_DEADBAND_US;

    if (s > 0.0f) {
        return (uint32_t)(neutral + db + s * (float)(max_us - neutral - db) + 0.5f);
    }
    if (s < 0.0f) {
        return (uint32_t)(neutral - db + s * (float)(neutral - db - min_us) + 0.5f);
    }
    return neutral;
}

static void servo_power_write(bool on)
{
    int level = on ? 1 : 0;
#if WINCH_SERVO_ACTIVE_LOW
    level = !level;
#endif
    gpio_set_level(CONFIG_SERVO_ENABLE_PIN, level);
}

esp_err_t winch_driver_init(void)
{
    if (s_inited) {
        ESP_LOGW(TAG, "winch_driver_init called twice");
        return ESP_ERR_INVALID_STATE;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    /* Servo power pin: output, inactive (unpowered) by default. */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_SERVO_ENABLE_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "servo-enable gpio_config");
    /* Max drive strength (40mA vs 20mA default) -- same reasoning as the ESC
     * pin (GPIO31, see esc_driver.c): this board has repeatedly turned out to
     * electrically load pins that looked plain on schematic alone. This pin
     * switches real downstream current (winch + both rudder servos) through
     * whatever gate/switch sits on CONFIG_SERVO_ENABLE_PIN's net -- a weak
     * drive means a slow, not-quite-clean transition into that load exactly
     * at power-on, which is the single moment most likely to glitch anything
     * sharing the same rail. Harmless if the net turns out to be unloaded. */
    gpio_set_drive_capability(CONFIG_SERVO_ENABLE_PIN, GPIO_DRIVE_CAP_3);
    servo_power_write(false);

    /* MCPWM timer. */
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = WINCH_MCPWM_GROUP,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = WINCH_TIMER_RESOLUTION,
        .period_ticks  = WINCH_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &s_timer), TAG, "new_timer");

    mcpwm_operator_config_t oper_cfg = { .group_id = WINCH_MCPWM_GROUP };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &s_oper), TAG, "new_operator");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_oper, s_timer), TAG, "connect_timer");

    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_oper, &cmp_cfg, &s_cmp), TAG, "new_comparator");

    /* GPIO35 boots HELD + sleep-isolated (see esc_driver.c); it's also the boot
     * button, doubling the stale state. Release the hold + clean-reset before MCPWM
     * attaches, then exclude the pad from sleep-switching after so the winch signal
     * survives WiFi modem-sleep. */
    gpio_hold_dis(CONFIG_WINCH_PWM_PIN);
    gpio_reset_pin(CONFIG_WINCH_PWM_PIN);
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = CONFIG_WINCH_PWM_PIN };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_oper, &gen_cfg, &s_gen), TAG, "new_generator");
    gpio_sleep_sel_dis(CONFIG_WINCH_PWM_PIN);

    /* HIGH at period start, LOW at compare match. */
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            s_gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)),
        TAG, "gen action timer");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           s_cmp,
                                           MCPWM_GEN_ACTION_LOW)),
        TAG, "gen action cmp");

    /* Neutral before starting so the servo never sees garbage. */
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp, CONFIG_WINCH_PULSE_NEUTRAL_US),
                        TAG, "init neutral");
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), TAG, "timer_enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP),
                        TAG, "timer_start");

    s_speed  = 0.0f;
    s_inited = true;
    ESP_LOGI(TAG, "winch init OK (PWM=GPIO%d %uHz neutral=%uus, EN=GPIO%d%s)",
             CONFIG_WINCH_PWM_PIN, (unsigned)CONFIG_WINCH_PWM_FREQ_HZ,
             (unsigned)CONFIG_WINCH_PULSE_NEUTRAL_US, CONFIG_SERVO_ENABLE_PIN,
             WINCH_SERVO_ACTIVE_LOW ? " (active-low)" : "");
    return ESP_OK;
}

esp_err_t winch_driver_set_speed(float speed)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_speed = (speed < -1.0f) ? -1.0f : (speed > 1.0f ? 1.0f : speed);
    uint32_t us = speed_to_us(s_speed);
    esp_err_t ret = mcpwm_comparator_set_compare_value(s_cmp, us);
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "set_speed: speed=%.2f -> %uus, rail=%s, mcpwm_ret=%s",   /* DIAG */
             s_speed, (unsigned)us, s_power ? "ON" : "OFF", esp_err_to_name(ret));
    return ret;
}

esp_err_t winch_driver_set_power(bool on)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    servo_power_write(on);
    s_power = on;
    ESP_LOGI(TAG, "servo rail %s (GPIO%d)", on ? "ON" : "OFF", CONFIG_SERVO_ENABLE_PIN);
    return ESP_OK;
}

bool winch_driver_get_power(void)
{
    return s_power;
}

float winch_driver_get_speed(void)
{
    return s_speed;
}
