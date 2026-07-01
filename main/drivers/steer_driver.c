#include "steer_driver.h"

#include "driver/mcpwm_prelude.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sdkconfig.h"

static const char *TAG = "STEER";

/* Group 1 — ESC and winch use MCPWM group 0. Both rudder servos share one
 * timer + operator in group 1 (two comparators/generators), so a single steer
 * command drives them together. 1 MHz => 1 us/tick. */
#define STEER_MCPWM_GROUP      1
#define STEER_TIMER_RESOLUTION 1000000U
#define STEER_PERIOD_TICKS     (STEER_TIMER_RESOLUTION / CONFIG_STEER_PWM_FREQ_HZ)

/* Kconfig bools are undefined (not 0) when 'n' — normalise for C expressions. */
#ifdef CONFIG_STEER_LEFT_REVERSE
#define STEER_LEFT_REV 1
#else
#define STEER_LEFT_REV 0
#endif
#ifdef CONFIG_STEER_RIGHT_REVERSE
#define STEER_RIGHT_REV 1
#else
#define STEER_RIGHT_REV 0
#endif

static mcpwm_timer_handle_t s_timer = NULL;
static mcpwm_oper_handle_t  s_oper  = NULL;
static mcpwm_cmpr_handle_t  s_cmp_l = NULL;
static mcpwm_cmpr_handle_t  s_cmp_r = NULL;
static mcpwm_gen_handle_t   s_gen_l = NULL;
static mcpwm_gen_handle_t   s_gen_r = NULL;
static SemaphoreHandle_t    s_mutex = NULL;
static float                s_left  = 0.0f;
static float                s_right = 0.0f;
static bool                 s_inited = false;

/* Map steer [-1,1] -> pulse us. 0 -> neutral, +1 -> max (right), -1 -> min (left). */
static uint32_t steer_to_us(float s)
{
    if (s < -1.0f) s = -1.0f;
    if (s >  1.0f) s =  1.0f;
    uint32_t neutral = CONFIG_STEER_PULSE_NEUTRAL_US;
    uint32_t min_us  = CONFIG_STEER_PULSE_MIN_US;
    uint32_t max_us  = CONFIG_STEER_PULSE_MAX_US;
    if (s >= 0.0f) {
        return (uint32_t)(neutral + s * (float)(max_us - neutral) + 0.5f);
    }
    return (uint32_t)(neutral + s * (float)(neutral - min_us) + 0.5f);
}

/* Create one comparator + generator on the shared operator for a servo output. */
static esp_err_t steer_make_output(mcpwm_cmpr_handle_t *cmp, mcpwm_gen_handle_t *gen, int gpio)
{
    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_oper, &cmp_cfg, cmp), TAG, "new_comparator");

    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = gpio };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_oper, &gen_cfg, gen), TAG, "new_generator");

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            *gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)),
        TAG, "gen action timer");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            *gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           *cmp,
                                           MCPWM_GEN_ACTION_LOW)),
        TAG, "gen action cmp");
    return ESP_OK;
}

esp_err_t steer_driver_init(void)
{
    if (s_inited) {
        ESP_LOGW(TAG, "steer_driver_init called twice");
        return ESP_ERR_INVALID_STATE;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    mcpwm_timer_config_t timer_cfg = {
        .group_id      = STEER_MCPWM_GROUP,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = STEER_TIMER_RESOLUTION,
        .period_ticks  = STEER_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &s_timer), TAG, "new_timer");

    mcpwm_operator_config_t oper_cfg = { .group_id = STEER_MCPWM_GROUP };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &s_oper), TAG, "new_operator");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_oper, s_timer), TAG, "connect_timer");

    ESP_RETURN_ON_ERROR(steer_make_output(&s_cmp_l, &s_gen_l, CONFIG_STEER_LEFT_PIN),  TAG, "left output");
    ESP_RETURN_ON_ERROR(steer_make_output(&s_cmp_r, &s_gen_r, CONFIG_STEER_RIGHT_PIN), TAG, "right output");

    /* Center both before starting so the servos never see garbage. */
    uint32_t neutral = CONFIG_STEER_PULSE_NEUTRAL_US;
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp_l, neutral), TAG, "L center");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp_r, neutral), TAG, "R center");
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), TAG, "timer_enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP), TAG, "timer_start");

    s_left   = 0.0f;
    s_right  = 0.0f;
    s_inited = true;
    ESP_LOGI(TAG, "steer init OK (L=GPIO%d%s R=GPIO%d%s %uHz neutral=%uus)",
             CONFIG_STEER_LEFT_PIN,  STEER_LEFT_REV  ? " rev" : "",
             CONFIG_STEER_RIGHT_PIN, STEER_RIGHT_REV ? " rev" : "",
             (unsigned)CONFIG_STEER_PWM_FREQ_HZ, (unsigned)neutral);
    return ESP_OK;
}

esp_err_t steer_driver_set(float left, float right)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_left  = (left  < -1.0f) ? -1.0f : (left  > 1.0f ? 1.0f : left);
    s_right = (right < -1.0f) ? -1.0f : (right > 1.0f ? 1.0f : right);
    float sl = STEER_LEFT_REV  ? -s_left  : s_left;
    float sr = STEER_RIGHT_REV ? -s_right : s_right;
    esp_err_t r1 = mcpwm_comparator_set_compare_value(s_cmp_l, steer_to_us(sl));
    esp_err_t r2 = mcpwm_comparator_set_compare_value(s_cmp_r, steer_to_us(sr));
    xSemaphoreGive(s_mutex);
    return (r1 == ESP_OK) ? r2 : r1;
}

float steer_driver_get_left(void)  { return s_left; }
float steer_driver_get_right(void) { return s_right; }
