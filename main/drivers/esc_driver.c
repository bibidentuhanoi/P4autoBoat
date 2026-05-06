#include "esc_driver.h"

#include "driver/mcpwm_prelude.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sdkconfig.h"
#include <string.h>

static const char *TAG = "ESC";

/* ---------- Compile-time constants from Kconfig ---------- */

#define ESC_MCPWM_GROUP      0
#define ESC_TIMER_RESOLUTION 1000000U   /* 1 MHz → 1 µs per tick */
#define ESC_PERIOD_TICKS     (ESC_TIMER_RESOLUTION / CONFIG_ESC_PWM_FREQ_HZ)

#define ESC_ARMING_DELAY_MS  3000

/* ---------- Static state ---------- */

static mcpwm_timer_handle_t   s_timer     = NULL;
static mcpwm_oper_handle_t    s_operator  = NULL;
static mcpwm_cmpr_handle_t    s_cmp_left  = NULL;
static mcpwm_cmpr_handle_t    s_cmp_right = NULL;
static mcpwm_gen_handle_t     s_gen_left  = NULL;
static mcpwm_gen_handle_t     s_gen_right = NULL;

static volatile esc_state_t   s_state     = ESC_STATE_DISARMED;
static float                  s_thr_left  = 0.0f;
static float                  s_thr_right = 0.0f;
static SemaphoreHandle_t      s_mutex     = NULL;
static bool                   s_inited    = false;

/* ---------- Helpers ---------- */

/**
 * Map float throttle [-1.0, 1.0] → pulse width in microseconds.
 * 0.0 maps to CONFIG_ESC_PULSE_NEUTRAL_US (centre of the range).
 * Values outside [-1, 1] are clamped.
 */
static uint32_t throttle_to_us(float t)
{
    if (t < -1.0f) t = -1.0f;
    if (t >  1.0f) t =  1.0f;

    /* neutral is the midpoint between min and max in config */
    uint32_t neutral = CONFIG_ESC_PULSE_NEUTRAL_US;
    uint32_t min_us  = CONFIG_ESC_PULSE_MIN_US;
    uint32_t max_us  = CONFIG_ESC_PULSE_MAX_US;

    uint32_t us;
    if (t >= 0.0f) {
        /* 0 → neutral, +1 → max */
        us = (uint32_t)(neutral + t * (float)(max_us - neutral) + 0.5f);
    } else {
        /* 0 → neutral, -1 → min */
        us = (uint32_t)(neutral + t * (float)(neutral - min_us) + 0.5f);
    }
    return us;
}

/** Write a pulse width directly to both comparators. */
static esp_err_t set_pulse_us(uint32_t left_us, uint32_t right_us)
{
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp_left,  left_us),
                        TAG, "cmp_left set");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp_right, right_us),
                        TAG, "cmp_right set");
    return ESP_OK;
}

/* ---------- Arming ---------- */

/* ---------- Public API ---------- */

esp_err_t esc_driver_init(void)
{
    if (s_inited) {
        ESP_LOGW(TAG, "esc_driver_init called twice");
        return ESP_ERR_INVALID_STATE;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    /* Timer */
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = ESC_MCPWM_GROUP,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = ESC_TIMER_RESOLUTION,
        .period_ticks  = ESC_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &s_timer),
                        TAG, "new_timer");

    /* Operator */
    mcpwm_operator_config_t oper_cfg = {
        .group_id = ESC_MCPWM_GROUP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &s_operator),
                        TAG, "new_operator");

    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_operator, s_timer),
                        TAG, "connect_timer");

    /* Comparators */
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_operator, &cmp_cfg, &s_cmp_left),
                        TAG, "new_cmp_left");
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_operator, &cmp_cfg, &s_cmp_right),
                        TAG, "new_cmp_right");

    /* Generators */
    mcpwm_generator_config_t gen_left_cfg = {
        .gen_gpio_num = CONFIG_ESC_PWM_LEFT_PIN,
    };
    mcpwm_generator_config_t gen_right_cfg = {
        .gen_gpio_num = CONFIG_ESC_PWM_RIGHT_PIN,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_operator, &gen_left_cfg,  &s_gen_left),
                        TAG, "new_gen_left");
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_operator, &gen_right_cfg, &s_gen_right),
                        TAG, "new_gen_right");

    /* Generator actions: HIGH on timer empty (start of period), LOW on comparator match */
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            s_gen_left,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)),
        TAG, "gen_left action timer");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_gen_left,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                            s_cmp_left,
                                            MCPWM_GEN_ACTION_LOW)),
        TAG, "gen_left action cmp");

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            s_gen_right,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)),
        TAG, "gen_right action timer");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_gen_right,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                            s_cmp_right,
                                            MCPWM_GEN_ACTION_LOW)),
        TAG, "gen_right action cmp");

    /* Output neutral before starting the timer so the ESC never sees garbage. */
    uint32_t neutral_us = CONFIG_ESC_PULSE_NEUTRAL_US;
    ESP_RETURN_ON_ERROR(set_pulse_us(neutral_us, neutral_us), TAG, "init neutral");

    /* Start timer (continuous). */
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), TAG, "timer_enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP),
                        TAG, "timer_start");

    s_thr_left  = 0.0f;
    s_thr_right = 0.0f;
    s_state     = ESC_STATE_DISARMED;
    s_inited    = true;

    ESP_LOGI(TAG, "ESC init OK (L=GPIO%d R=GPIO%d %uHz neutral=%uus)",
             CONFIG_ESC_PWM_LEFT_PIN, CONFIG_ESC_PWM_RIGHT_PIN,
             (unsigned)CONFIG_ESC_PWM_FREQ_HZ, (unsigned)neutral_us);
    return ESP_OK;
}

esp_err_t esc_driver_arm(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_state != ESC_STATE_DISARMED) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "arm() ignored — state=%d", (int)s_state);
        return ESP_ERR_INVALID_STATE;
    }
    /* Ensure neutral while arming. */
    uint32_t neutral_us = CONFIG_ESC_PULSE_NEUTRAL_US;
    set_pulse_us(neutral_us, neutral_us);
    s_thr_left  = 0.0f;
    s_thr_right = 0.0f;
    s_state     = ESC_STATE_ARMING;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "ESC arming (holding neutral %d ms)...", ESC_ARMING_DELAY_MS);

    vTaskDelay(pdMS_TO_TICKS(ESC_ARMING_DELAY_MS));

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state = ESC_STATE_ARMED;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "ESC armed");
    return ESP_OK;
}

esp_err_t esc_driver_set_throttle(float left, float right)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_state != ESC_STATE_ARMED) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = set_pulse_us(throttle_to_us(left), throttle_to_us(right));
    if (err == ESP_OK) {
        s_thr_left  = left;
        s_thr_right = right;
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t esc_driver_disarm(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t neutral_us = CONFIG_ESC_PULSE_NEUTRAL_US;
    set_pulse_us(neutral_us, neutral_us);
    s_thr_left  = 0.0f;
    s_thr_right = 0.0f;
    s_state     = ESC_STATE_DISARMED;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "ESC disarmed");
    return ESP_OK;
}

esc_state_t esc_driver_get_state(void)
{
    return s_state;
}

void esc_driver_get_throttle(float *left, float *right)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (left)  *left  = s_thr_left;
    if (right) *right = s_thr_right;
    xSemaphoreGive(s_mutex);
}
