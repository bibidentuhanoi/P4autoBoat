#include "steer_driver.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sdkconfig.h"

static const char *TAG = "STEER";

/* ONE unified rudder output. A boat has a single steering axis, so both rudder
 * servos hang off this one signal (wired in parallel — each keeps its own + and
 * GND) and always deflect together. The old split control (two independent
 * rudders on GPIO32/34) is gone: it added a degree of freedom nothing actually
 * steers with, and a planner only ever wants one steering value. Both servos
 * are wired to GPIO32 (CONFIG_STEER_PIN) on this board — GPIO34 generated a
 * provably correct signal (right math, ESP_OK on every write) but nothing was
 * physically connected to it; moved to 32 2026-08-04.
 *
 * Group 1 — the ESC and winch use MCPWM group 0. 1 MHz => 1 us/tick. */
#define STEER_MCPWM_GROUP      1
#define STEER_TIMER_RESOLUTION 1000000U
#define STEER_PERIOD_TICKS     (STEER_TIMER_RESOLUTION / CONFIG_STEER_PWM_FREQ_HZ)

/* Kconfig bools are undefined (not 0) when 'n' — normalise for C expressions. */
#ifdef CONFIG_STEER_REVERSE
#define STEER_REV 1
#else
#define STEER_REV 0
#endif

static mcpwm_timer_handle_t s_timer  = NULL;
static mcpwm_oper_handle_t  s_oper   = NULL;
static mcpwm_cmpr_handle_t  s_cmp    = NULL;
static mcpwm_gen_handle_t   s_gen    = NULL;
static SemaphoreHandle_t    s_mutex  = NULL;
static float                s_steer  = 0.0f;
static bool                 s_inited = false;

/* The operator's physical resting reference (hand-turned mechanical stop,
 * before every power-up) is the 1805us end-stop. Measured on the boat
 * 2026-08-31, sighting from BEHIND the hull toward the bow:
 *
 *     1805us = full LEFT      1516us = centre      1195us = full RIGHT
 *
 * so that stop is full LEFT, and the canonical value for it is -1.
 *
 * This #define went from +1 to -1 in the same change that turned
 * CONFIG_STEER_REVERSE on, and the two cancel: boot_s below flips -1 back to
 * +1, steer_to_us(+1) is still MAX_US, and the servo still powers up at
 * 1805us. The physical boot position is UNCHANGED — only the name it is
 * called by is now the true one.
 *
 * Earlier revisions called 1805us "full RIGHT" and set this to +1. That was
 * never checked against the hull; it was inferred from MAX_US being the
 * larger number. Everything downstream inherited the error. */
#define STEER_HOME (-1.0f)

/* Map steer [-1,1] -> pulse us: 0 -> neutral, +1 -> MAX_US, -1 -> MIN_US.
 *
 * This is the RAW pulse mapping and knows nothing about left or right.
 * CONFIG_STEER_REVERSE is applied by the CALLERS (init/set/reassert), never
 * here, so this function stays a pure MIN/NEUTRAL/MAX interpolation.
 *
 * On this board MAX_US 1805 is physical LEFT and MIN_US 1195 is physical
 * RIGHT, so with reverse ON a caller's +1 ("right", canonical) arrives here
 * as -1 and lands on 1195us. That is the whole reason reverse is on. */
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

    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_oper, &cmp_cfg, &s_cmp), TAG, "new_comparator");

    /* Actuator pads boot HELD + sleep-isolated on this board (see esc_driver.c).
     * Release the hold + clean-reset before MCPWM attaches, then exclude the pad
     * from sleep-switching after so the servo signal survives WiFi modem-sleep. */
    gpio_hold_dis(CONFIG_STEER_PIN);
    gpio_reset_pin(CONFIG_STEER_PIN);
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = CONFIG_STEER_PIN };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_oper, &gen_cfg, &s_gen), TAG, "new_generator");
    gpio_sleep_sel_dis(CONFIG_STEER_PIN);

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

    /* Home to STEER_HOME (full-LEFT, 1805us) before starting, matching the
     * operator's confirmed hand-positioned reference — the standard fix for
     * "servo has no position feedback" (manual positioning to match the boot
     * target), not a guess. Reuse steer_to_us() so this can never drift from
     * what steer_driver_set(STEER_HOME) would itself compute. */
    s_steer = STEER_HOME;
    float boot_s = STEER_REV ? -s_steer : s_steer;
    uint32_t boot_us = steer_to_us(boot_s);
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_cmp, boot_us), TAG, "home");
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), TAG, "timer_enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP),
                        TAG, "timer_start");

    s_inited = true;
    ESP_LOGI(TAG, "steer init OK (rudder=GPIO%d%s %uHz home=%uus full-left, both servos linked)",
             CONFIG_STEER_PIN, STEER_REV ? " rev" : "",
             (unsigned)CONFIG_STEER_PWM_FREQ_HZ, (unsigned)boot_us);
    return ESP_OK;
}

esp_err_t steer_driver_set(float steer)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_steer = (steer < -1.0f) ? -1.0f : (steer > 1.0f ? 1.0f : steer);
    float s = STEER_REV ? -s_steer : s_steer;
    uint32_t us = steer_to_us(s);
    esp_err_t err = mcpwm_comparator_set_compare_value(s_cmp, us);
    xSemaphoreGive(s_mutex);
    ESP_LOGD(TAG, "set: steer=%.2f -> %uus, mcpwm_ret=%s",   /* DIAG */
             s_steer, (unsigned)us, esp_err_to_name(err));
    return err;
}

/* Defensive re-assertion, NOT a new command: re-clears any hold that may have
 * re-latched on this pad since boot (this board's actuator pins boot HELD;
 * something during the ~5s WiFi/SDIO bring-up is suspected of re-applying it —
 * see the app_main cleanup comment) and rewrites the CURRENT s_steer value
 * fresh, so the physical pad is guaranteed to actually match the peripheral's
 * register right before the rail gets power — not just once, early, at boot,
 * and hoping it survives untouched until power arrives tens of seconds later.
 * Call this immediately before energising the servo rail. */
void steer_driver_reassert(void)
{
    if (!s_inited) return;
    gpio_hold_dis(CONFIG_STEER_PIN);   /* cheap, idempotent — safe to call anytime */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float s = STEER_REV ? -s_steer : s_steer;
    uint32_t us = steer_to_us(s);
    esp_err_t err = mcpwm_comparator_set_compare_value(s_cmp, us);
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "reassert: steer=%.2f -> %uus, mcpwm_ret=%s", s_steer, (unsigned)us, esp_err_to_name(err));
}

/* Calibration escape hatch: writes pulse_us straight to the comparator,
 * bypassing steer_to_us()/MIN/NEUTRAL/MAX entirely, so the operator can probe
 * past the currently configured range to find a servo's true mechanical stop.
 * Clamped to 400-2600us (generous extended hobby-servo spec) purely to stop a
 * typo from sending something nonsensical — not a calibration boundary. */
esp_err_t steer_driver_set_raw_us(uint32_t pulse_us)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (pulse_us < 400) pulse_us = 400;
    if (pulse_us > 2600) pulse_us = 2600;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    esp_err_t err = mcpwm_comparator_set_compare_value(s_cmp, pulse_us);
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "set_raw: %uus, mcpwm_ret=%s (calibration — steer_driver_get() unchanged)",
             (unsigned)pulse_us, esp_err_to_name(err));
    return err;
}

float steer_driver_get(void)
{
    if (!s_mutex) return s_steer;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float steer = s_steer;
    xSemaphoreGive(s_mutex);
    return steer;
}
