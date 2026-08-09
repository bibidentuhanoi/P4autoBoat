#include "wifi_manager.h"
#include "test_esp_idf.h"

#include <stdarg.h>
#include <stdio.h>

const char test_wifi_event_base[] = "WIFI_EVENT";
const char test_ip_event_base[] = "IP_EVENT";

static struct test_event_group s_event_group;
static esp_event_handler_t s_wifi_handler;
static void *s_wifi_handler_arg;
static esp_event_handler_t s_ip_handler;
static void *s_ip_handler_arg;
static bool s_fake_netif_started;

static void post_wifi_event(int32_t event_id, void *event_data)
{
    if (s_wifi_handler) {
        s_wifi_handler(s_wifi_handler_arg, WIFI_EVENT, event_id, event_data);
    }
}

static void post_ip_event(int32_t event_id, void *event_data)
{
    if (s_ip_handler) {
        s_ip_handler(s_ip_handler_arg, IP_EVENT, event_id, event_data);
    }
}

const char *esp_err_to_name(esp_err_t error)
{
    (void)error;
    return "test error";
}

void test_log_sink(const char *tag, const char *format, ...)
{
    (void)tag;
    (void)format;
}

esp_err_t esp_netif_init(void)
{
    return ESP_OK;
}

esp_netif_t *esp_netif_create_default_wifi_sta(void)
{
    static esp_netif_t netif;
    return &netif;
}

esp_err_t esp_event_loop_create_default(void)
{
    return ESP_OK;
}

esp_err_t esp_event_handler_instance_register(esp_event_base_t event_base,
                                               int32_t event_id,
                                               esp_event_handler_t handler,
                                               void *handler_arg,
                                               esp_event_handler_instance_t *instance)
{
    (void)event_id;
    *instance = handler;
    if (event_base == WIFI_EVENT) {
        s_wifi_handler = handler;
        s_wifi_handler_arg = handler_arg;
    } else if (event_base == IP_EVENT) {
        s_ip_handler = handler;
        s_ip_handler_arg = handler_arg;
    } else {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{
    (void)config;
    return ESP_OK;
}

esp_err_t esp_wifi_set_mode(wifi_mode_t mode)
{
    (void)mode;
    return ESP_OK;
}

esp_err_t esp_wifi_set_config(wifi_interface_t interface, wifi_config_t *config)
{
    (void)interface;
    (void)config;
    return ESP_OK;
}

esp_err_t esp_wifi_start(void)
{
    s_fake_netif_started = true;
    post_wifi_event(WIFI_EVENT_STA_START, NULL);
    return ESP_OK;
}

esp_err_t esp_wifi_set_max_tx_power(int8_t power)
{
    (void)power;
    return ESP_OK;
}

esp_err_t esp_wifi_get_max_tx_power(int8_t *power)
{
    *power = 80;
    return ESP_OK;
}

esp_err_t esp_wifi_set_ps(wifi_ps_type_t type)
{
    (void)type;
    return ESP_OK;
}

esp_err_t esp_wifi_set_protocol(wifi_interface_t interface, uint8_t bitmap)
{
    (void)interface;
    (void)bitmap;
    s_fake_netif_started = false;
    post_wifi_event(WIFI_EVENT_STA_STOP, NULL);
    return ESP_OK;
}

esp_err_t esp_wifi_connect(void)
{
    if (s_fake_netif_started) {
        ip_event_got_ip_t got_ip = {0};
        post_ip_event(IP_EVENT_STA_GOT_IP, &got_ip);
    }
    return ESP_OK;
}

esp_err_t esp_wifi_disconnect(void)
{
    return ESP_OK;
}

EventGroupHandle_t xEventGroupCreate(void)
{
    s_event_group.bits = 0;
    return &s_event_group;
}

EventBits_t xEventGroupSetBits(EventGroupHandle_t event_group, EventBits_t bits)
{
    event_group->bits |= bits;
    return event_group->bits;
}

EventBits_t xEventGroupWaitBits(EventGroupHandle_t event_group,
                                EventBits_t bits_to_wait_for,
                                BaseType_t clear_on_exit,
                                BaseType_t wait_for_all,
                                TickType_t ticks_to_wait)
{
    (void)clear_on_exit;
    (void)wait_for_all;
    (void)ticks_to_wait;
    return event_group->bits & bits_to_wait_for;
}

void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
}

int main(void)
{
    if (wifi_start_radio() != ESP_OK) {
        fprintf(stderr, "wifi_start_radio failed\n");
        return 1;
    }
    if (wifi_connect() != ESP_OK) {
        fprintf(stderr, "wifi_connect did not reach got-IP state\n");
        return 2;
    }
    return 0;
}
