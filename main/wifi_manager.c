#include "wifi_manager.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

static const char *TAG = "WIFI";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_CONNECT_TIMEOUT_MS 30000

static EventGroupHandle_t s_wifi_events;
static int s_retry_count;
static volatile bool s_stop_reconnect = false;
static volatile bool s_connect_enabled = false;
#define MAX_RETRIES 5

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        /* Only chase an AP once wifi_connect() has asked us to. Starting the
         * radio must NOT trigger a scan: ESP-NOW probing runs on the started-
         * but-unconnected radio, and a scan would hop it off the ESP-NOW
         * channel. */
        if (s_connect_enabled) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Field mode owns the radio channel — never reconnect (and never scan)
         * behind ESP-NOW's back. See wifi_manager_stop_reconnect(). */
        if (s_stop_reconnect) {
            return;
        }
        s_retry_count++;
        /* Fast retries first, then back off to 10s — never give up */
        int delay_ms = (s_retry_count <= MAX_RETRIES) ? 1000 : 10000;
        ESP_LOGI(TAG, "WiFi disconnected, retry %d in %dms...", s_retry_count, delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_start_radio(void)
{
    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t wifi_err = esp_wifi_init(&cfg);
    if (wifi_err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s (esp_hosted slave down?)",
                 esp_err_to_name(wifi_err));
        return wifi_err;
    }

    esp_event_handler_instance_t inst_any_id;
    esp_event_handler_instance_t inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                         &wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    /* If a password is set, require WPA2 minimum */
    if (strlen(CONFIG_WIFI_PASSWORD) > 0) {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    ESP_ERROR_CHECK(esp_wifi_start());

    /* Max TX power: 20 dBm (param is in 0.25 dBm units → 80) */
    esp_err_t pwr_err = esp_wifi_set_max_tx_power(80);
    if (pwr_err != ESP_OK) {
        ESP_LOGW(TAG, "set_max_tx_power: %s (may need wifi started first)", esp_err_to_name(pwr_err));
    } else {
        int8_t actual;
        esp_wifi_get_max_tx_power(&actual);
        ESP_LOGI(TAG, "TX power set to %.1f dBm", actual * 0.25);
    }

    /* Disable modem power save on the C6 slave (RPC via esp_wifi_remote).
     * The old CONFIG_ESP_WIFI_PS_NONE line in sdkconfig.defaults was a dead
     * symbol on esp_hosted — this runtime call is what actually applies it.
     * PS off = lower latency + far fewer drops for WS control + MJPEG. */
    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_err != ESP_OK) {
        ESP_LOGW(TAG, "set_ps(NONE): %s", esp_err_to_name(ps_err));
    } else {
        ESP_LOGI(TAG, "WiFi power save disabled (PS_NONE)");
    }

    ESP_LOGI(TAG, "WiFi radio started (not connected)");
    return ESP_OK;
}

esp_err_t wifi_connect(void)
{
    if (!s_wifi_events) {
        ESP_LOGE(TAG, "wifi_connect() before wifi_start_radio()");
        return ESP_ERR_INVALID_STATE;
    }

    /* Undo any LR protocol bitmap the ESP-NOW probe may have set on the C6
     * (tools/slave_firmware/espnow_bridge.c's init_cb runs unconditionally
     * at the START of every probe attempt, before we know whether a ground
     * station will answer -- see main.c's boot sequence). Espressif
     * documents BGNLR as negotiating down cleanly with a normal router, but
     * this path has nothing to do with ESP-NOW/LR at all -- don't rely on
     * that claim holding on this exact hardware. Restore the chip's own
     * documented default (BGNAX -- the C6 supports 802.11ax) before this
     * interface is ever used for a real AP connection. */
    esp_err_t proto_err = esp_wifi_set_protocol(WIFI_IF_STA,
            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX);
    if (proto_err != ESP_OK) {
        ESP_LOGW(TAG, "wifi_connect: esp_wifi_set_protocol(reset) failed: %s",
                 esp_err_to_name(proto_err));
    }

    ESP_LOGI(TAG, "Connecting to SSID: %s", CONFIG_WIFI_SSID);

    /* Arm the handler and kick off the first attempt; the radio is already
     * started, so WIFI_EVENT_STA_START has long since fired. */
    s_connect_enabled = true;
    esp_err_t conn_err = esp_wifi_connect();
    if (conn_err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(conn_err));
        return conn_err;
    }

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi failed to connect");
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "WiFi connection timed out");
        return ESP_ERR_TIMEOUT;
    }
}

void wifi_manager_stop_reconnect(void)
{
    s_stop_reconnect = true;

    /* Abort any connect/scan already in flight, otherwise it keeps hopping
     * channels for a few more seconds after we hand the radio to ESP-NOW. */
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED && err != ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGW(TAG, "stop_reconnect: disconnect returned %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "WiFi auto-reconnect disabled — radio channel released for ESP-NOW");
}
