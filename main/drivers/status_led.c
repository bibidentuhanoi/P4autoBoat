#include "drivers/status_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "STATUS_LED";

/* Kconfig `bool default n` leaves the symbol undefined (not 0) when disabled,
 * which is fine in #if but breaks a runtime C expression. Normalise to 0/1. */
#ifdef CONFIG_STATUS_LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 1
#else
#define LED_ACTIVE_LOW 0
#endif

/* Plain volatile write/read is atomic for an enum on this target — the setter
 * can be called from any task without a lock; the pattern task samples it. */
static volatile status_led_state_t s_state = STATUS_LED_OFF;
static bool s_inited = false;

static inline void led_write(bool on) {
    int level = on ? 1 : 0;
#if LED_ACTIVE_LOW
    level = !level;
#endif
    gpio_set_level(CONFIG_STATUS_LED_PIN, level);
}

/* On for on_ms, off for off_ms, polled in 10ms steps so a state change aborts
 * the pulse promptly instead of finishing a long blink. Returns false if the
 * state changed mid-pulse (so callers can short-circuit multi-pulse patterns). */
static bool pulse(status_led_state_t owner, int on_ms, int off_ms) {
    led_write(true);
    for (int t = 0; t < on_ms && s_state == owner; t += 10) vTaskDelay(pdMS_TO_TICKS(10));
    led_write(false);
    for (int t = 0; t < off_ms && s_state == owner; t += 10) vTaskDelay(pdMS_TO_TICKS(10));
    return s_state == owner;
}

static void status_led_task(void *arg) {
    (void)arg;
    while (1) {
        status_led_state_t st = s_state;
        switch (st) {
            case STATUS_LED_CAL_WINDOW:                 /* fast even blink — "press now" */
                pulse(st, 100, 100);
                break;
            case STATUS_LED_CAL_STILL:                  /* solid — "hold flat & still" */
                led_write(true);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
            case STATUS_LED_CAL_MOVE:                   /* 3 blips + gap — "keep moving" */
                (void)(pulse(st, 70, 70) && pulse(st, 70, 70) && pulse(st, 70, 400));
                break;
            case STATUS_LED_CAL_POINT:                  /* slow blink — "aim & settle" */
                pulse(st, 500, 500);
                break;
            case STATUS_LED_CAL_DONE_OK:                /* 3 slow flashes, then dark */
                pulse(st, 300, 200); pulse(st, 300, 200); pulse(st, 300, 200);
                if (s_state == STATUS_LED_CAL_DONE_OK) s_state = STATUS_LED_OFF;
                break;
            case STATUS_LED_CAL_DONE_FAIL:              /* rapid flutter ~1.5s, then dark */
                for (int i = 0; i < 12 && s_state == st; i++) pulse(st, 60, 60);
                if (s_state == STATUS_LED_CAL_DONE_FAIL) s_state = STATUS_LED_OFF;
                break;
            case STATUS_LED_OFF:
            default:
                led_write(false);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
        }
    }
}

esp_err_t status_led_init(void) {
    if (s_inited) return ESP_OK;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_STATUS_LED_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(GPIO%d) failed: %s", CONFIG_STATUS_LED_PIN, esp_err_to_name(err));
        return err;
    }
    led_write(false);

    if (xTaskCreate(status_led_task, "status_led", 2048, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }
    s_inited = true;
    ESP_LOGI(TAG, "status LED on GPIO%d%s", CONFIG_STATUS_LED_PIN,
             LED_ACTIVE_LOW ? " (active-low)" : "");
    return ESP_OK;
}

void status_led_set(status_led_state_t state) {
    s_state = state;
}
