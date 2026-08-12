#include "esc_driver.h"
#include "esc_map.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sdkconfig.h"

/* New Kconfig symbols are absent until sdkconfig is regenerated. Keep the
 * measured defaults safe for an existing generated configuration. */
#ifndef CONFIG_ESC_MIN_THR_LEFT_PCT
#define CONFIG_ESC_MIN_THR_LEFT_PCT 5
#endif
#ifndef CONFIG_ESC_MIN_THR_RIGHT_PCT
#define CONFIG_ESC_MIN_THR_RIGHT_PCT 20
#endif

static const char *TAG = "ESC";

/* ---------------------------------------------------------------------------
 * Driven with MCPWM, not LEDC.
 *
 * History: LEDC drives GPIO31 fine in a bare Arduino sketch, but inside THIS
 * firmware the LEDC signal on GPIO31 came out corrupted (the ESC never saw a
 * clean arm pulse and kept warning-beeping) once the rest of the system was up —
 * for a reason no GPIO/LEDC register dump could show (pad, duty=62259, routing
 * and channel were all byte-identical to the working GPIO33). The winch and
 * steering servos run cleanly on MCPWM on neighbouring pins in the same firmware,
 * so the ESC uses MCPWM too and the problem is gone.
 *
 * HW-517: needs an INVERTED (active-low) servo pulse — LOW for `pulse_us`, HIGH
 * for the rest of the 20ms frame (matches the old LEDC MAX-d inversion).
 * Unidirectional, arms at MINIMUM (1000us). Own MCPWM timer+operator in group 0;
 * one operator, two comparators/generators, so left and right are independent.
 * Arming is split into nonblocking begin/complete operations; ControlTask owns
 * the three-second timing through the persistent ArmSeq state machine.
 * ------------------------------------------------------------------------- */

#define ESC_MCPWM_GROUP        0
#define ESC_TIMER_RESOLUTION   1000000U   /* 1 MHz => 1 us/tick */
#define ESC_PERIOD_TICKS       (ESC_TIMER_RESOLUTION / CONFIG_ESC_PWM_FREQ_HZ)   /* 20000 @ 50Hz */
static mcpwm_timer_handle_t s_timer  = NULL;
static mcpwm_oper_handle_t  s_oper   = NULL;
static mcpwm_cmpr_handle_t  s_cmp_l  = NULL;
static mcpwm_cmpr_handle_t  s_cmp_r  = NULL;
static mcpwm_gen_handle_t   s_gen_l  = NULL;
static mcpwm_gen_handle_t   s_gen_r  = NULL;

static volatile esc_state_t s_state     = ESC_STATE_DISARMED;
static float                s_thr_left  = 0.0f;
static float                s_thr_right = 0.0f;
static SemaphoreHandle_t    s_mutex     = NULL;
static bool                 s_inited    = false;

/* Unidirectional ESC (HW-517 arms at min): 0 = MIN (off/stop), 1 = MAX (full).
 * No reverse — negative throttle clamps to stop. */
static uint32_t throttle_to_us(float t, float floor_frac)
{
    return esc_throttle_to_us(t, CONFIG_ESC_PULSE_MIN_US,
                               CONFIG_ESC_PULSE_MAX_US, floor_frac);
}

/* Set both channels' pulse width (us == MCPWM ticks at 1 MHz). */
static esp_err_t set_pulse_us(uint32_t left_us, uint32_t right_us)
{
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp_l, left_us),  TAG, "left");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp_r, right_us), TAG, "right");
    return ESP_OK;
}

/* Create one comparator + generator on the shared operator for a channel.
 * INVERTED (active-low) for the HW-517: LOW at period start (timer empty), HIGH at
 * the compare match — so the pad is LOW for `pulse_us` then HIGH for the rest. */
static esp_err_t esc_make_output(mcpwm_cmpr_handle_t *cmp, mcpwm_gen_handle_t *gen, int gpio)
{
    /* GPIO31 boots HELD + sleep-isolated on this board (see the long saga). Release
     * the hold + clean-reset BEFORE attaching, exclude from sleep-switching AFTER. */
    gpio_hold_dis(gpio);
    gpio_reset_pin(gpio);

    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_oper, &cmp_cfg, cmp), TAG, "new_comparator");

    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = gpio };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_oper, &gen_cfg, gen), TAG, "new_generator");
    gpio_sleep_sel_dis(gpio);
    /* Max drive strength (40mA vs 20mA default): GPIO31 is loaded on this board once
     * the full system powers up; the strongest pad driver gives the best shot at a
     * clean edge into that load. Harmless on the unloaded right pin. */
    gpio_set_drive_capability(gpio, GPIO_DRIVE_CAP_3);

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            *gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_LOW)),
        TAG, "gen action timer");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            *gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           *cmp,
                                           MCPWM_GEN_ACTION_HIGH)),
        TAG, "gen action cmp");
    return ESP_OK;
}

esp_err_t esc_driver_init(void)
{
    if (s_inited) {
        ESP_LOGW(TAG, "esc_driver_init called twice");
        return ESP_ERR_INVALID_STATE;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    mcpwm_timer_config_t timer_cfg = {
        .group_id      = ESC_MCPWM_GROUP,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = ESC_TIMER_RESOLUTION,
        .period_ticks  = ESC_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &s_timer), TAG, "new_timer");

    mcpwm_operator_config_t oper_cfg = { .group_id = ESC_MCPWM_GROUP };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &s_oper), TAG, "new_operator");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_oper, s_timer), TAG, "connect_timer");

    ESP_RETURN_ON_ERROR(esc_make_output(&s_cmp_l, &s_gen_l, CONFIG_ESC_PWM_LEFT_PIN),  TAG, "left output");
    ESP_RETURN_ON_ERROR(esc_make_output(&s_cmp_r, &s_gen_r, CONFIG_ESC_PWM_RIGHT_PIN), TAG, "right output");

    /* Output MINIMUM (off) before starting — the unidirectional ESC arms at min and
     * treats min as off (neutral would be half throttle on this ESC). */
    uint32_t off_us = CONFIG_ESC_PULSE_MIN_US;
    ESP_RETURN_ON_ERROR(set_pulse_us(off_us, off_us), TAG, "init min");
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), TAG, "timer_enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP), TAG, "timer_start");

    s_thr_left  = 0.0f;
    s_thr_right = 0.0f;
    s_state     = ESC_STATE_DISARMED;
    s_inited    = true;

    ESP_LOGI(TAG, "ESC init OK (MCPWM L=GPIO%d R=GPIO%d %uHz off=%uus, inverted/unidirectional)",
             CONFIG_ESC_PWM_LEFT_PIN, CONFIG_ESC_PWM_RIGHT_PIN,
             (unsigned)CONFIG_ESC_PWM_FREQ_HZ, (unsigned)off_us);
    return ESP_OK;
}

esp_err_t esc_driver_arm_begin(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_state != ESC_STATE_DISARMED) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "arm begin ignored — state=%d", (int)s_state);
        return ESP_ERR_INVALID_STATE;
    }
    /* Arm at MINIMUM throttle on BOTH channels (the HW-517 arm point). */
    uint32_t off_us = CONFIG_ESC_PULSE_MIN_US;
    esp_err_t err = set_pulse_us(off_us, off_us);
    if (err == ESP_OK) {
        s_thr_left  = 0.0f;
        s_thr_right = 0.0f;
        s_state     = ESC_STATE_ARMING;
    }
    xSemaphoreGive(s_mutex);

    if (err == ESP_OK) ESP_LOGI(TAG, "ESC arming (holding minimum throttle)");
    return err;
}

esp_err_t esc_driver_arm_complete(void)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_state != ESC_STATE_ARMING) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "arm complete ignored — state=%d", (int)s_state);
        return ESP_ERR_INVALID_STATE;
    }
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
    esp_err_t err = set_pulse_us(
        throttle_to_us(left, CONFIG_ESC_MIN_THR_LEFT_PCT / 100.0f),
        throttle_to_us(right, CONFIG_ESC_MIN_THR_RIGHT_PCT / 100.0f));
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
    uint32_t off_us = CONFIG_ESC_PULSE_MIN_US;
    set_pulse_us(off_us, off_us);
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
