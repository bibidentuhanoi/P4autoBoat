#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CONFIG_WIFI_SSID "test-network"
#define CONFIG_WIFI_PASSWORD "test-password"

typedef int esp_err_t;

#define ESP_OK                         0
#define ESP_FAIL                      -1
#define ESP_ERR_NO_MEM                 0x101
#define ESP_ERR_INVALID_STATE          0x103
#define ESP_ERR_TIMEOUT                0x107
#define ESP_ERR_WIFI_NOT_STARTED       0x3001
#define ESP_ERR_WIFI_NOT_CONNECT       0x3002

const char *esp_err_to_name(esp_err_t error);
void test_log_sink(const char *tag, const char *format, ...);

#define ESP_LOGE(...) test_log_sink(__VA_ARGS__)
#define ESP_LOGW(...) test_log_sink(__VA_ARGS__)
#define ESP_LOGI(...) test_log_sink(__VA_ARGS__)
#define ESP_ERROR_CHECK(expression) do { \
    esp_err_t test_error = (expression); \
    if (test_error != ESP_OK) { \
        abort(); \
    } \
} while (0)

typedef const char *esp_event_base_t;
extern const char test_wifi_event_base[];
extern const char test_ip_event_base[];

#define WIFI_EVENT test_wifi_event_base
#define IP_EVENT test_ip_event_base
#define ESP_EVENT_ANY_ID (-1)

enum {
    WIFI_EVENT_STA_START = 0,
    WIFI_EVENT_STA_STOP = 1,
    WIFI_EVENT_STA_CONNECTED = 2,
    WIFI_EVENT_STA_DISCONNECTED = 3,
};

enum {
    IP_EVENT_STA_GOT_IP = 0,
};

typedef struct {
    uint32_t addr;
} esp_ip4_addr_t;

typedef struct {
    esp_ip4_addr_t ip;
    esp_ip4_addr_t netmask;
    esp_ip4_addr_t gw;
} esp_netif_ip_info_t;

typedef struct {
    void *esp_netif;
    esp_netif_ip_info_t ip_info;
} ip_event_got_ip_t;

#define IPSTR "%u.%u.%u.%u"
#define IP2STR(ip) \
    (unsigned)((ip)->addr & 0xffU), \
    (unsigned)(((ip)->addr >> 8) & 0xffU), \
    (unsigned)(((ip)->addr >> 16) & 0xffU), \
    (unsigned)(((ip)->addr >> 24) & 0xffU)

typedef void (*esp_event_handler_t)(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data);
typedef void *esp_event_handler_instance_t;

esp_err_t esp_event_loop_create_default(void);
esp_err_t esp_event_handler_instance_register(esp_event_base_t event_base,
                                               int32_t event_id,
                                               esp_event_handler_t handler,
                                               void *handler_arg,
                                               esp_event_handler_instance_t *instance);

typedef struct {
    int unused;
} esp_netif_t;

esp_err_t esp_netif_init(void);
esp_netif_t *esp_netif_create_default_wifi_sta(void);

typedef enum {
    WIFI_IF_STA = 0,
} wifi_interface_t;

typedef enum {
    WIFI_MODE_STA = 1,
} wifi_mode_t;

typedef enum {
    WIFI_AUTH_OPEN = 0,
    WIFI_AUTH_WPA2_PSK = 3,
} wifi_auth_mode_t;

typedef enum {
    WIFI_PS_NONE = 0,
} wifi_ps_type_t;

typedef struct {
    int unused;
} wifi_init_config_t;

#define WIFI_INIT_CONFIG_DEFAULT() ((wifi_init_config_t){0})

typedef struct {
    struct {
        uint8_t ssid[32];
        uint8_t password[64];
        struct {
            wifi_auth_mode_t authmode;
        } threshold;
    } sta;
} wifi_config_t;

#define WIFI_PROTOCOL_11B  (1U << 0)
#define WIFI_PROTOCOL_11G  (1U << 1)
#define WIFI_PROTOCOL_11N  (1U << 2)
#define WIFI_PROTOCOL_11AX (1U << 5)

esp_err_t esp_wifi_init(const wifi_init_config_t *config);
esp_err_t esp_wifi_set_mode(wifi_mode_t mode);
esp_err_t esp_wifi_set_config(wifi_interface_t interface, wifi_config_t *config);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_set_max_tx_power(int8_t power);
esp_err_t esp_wifi_get_max_tx_power(int8_t *power);
esp_err_t esp_wifi_set_ps(wifi_ps_type_t type);
esp_err_t esp_wifi_set_protocol(wifi_interface_t interface, uint8_t bitmap);
esp_err_t esp_wifi_connect(void);
esp_err_t esp_wifi_disconnect(void);

typedef uint32_t TickType_t;
typedef uint32_t EventBits_t;
typedef int BaseType_t;

typedef struct test_event_group {
    EventBits_t bits;
} *EventGroupHandle_t;

#define BIT0 (1U << 0)
#define BIT1 (1U << 1)
#define pdFALSE 0
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(milliseconds) ((TickType_t)(milliseconds))

EventGroupHandle_t xEventGroupCreate(void);
EventBits_t xEventGroupSetBits(EventGroupHandle_t event_group, EventBits_t bits);
EventBits_t xEventGroupWaitBits(EventGroupHandle_t event_group,
                                EventBits_t bits_to_wait_for,
                                BaseType_t clear_on_exit,
                                BaseType_t wait_for_all,
                                TickType_t ticks_to_wait);
void vTaskDelay(TickType_t ticks);
