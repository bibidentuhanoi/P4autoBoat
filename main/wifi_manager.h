#pragma once
#include "esp_err.h"
#include <string.h>
/**
 * @brief Connect to WiFi AP configured via Kconfig (CONFIG_WIFI_SSID / CONFIG_WIFI_PASSWORD).
 *
 * Blocks until IP is assigned or a 30s timeout elapses.
 */
esp_err_t wifi_init(void);

/**
 * @brief Stop the auto-reconnect loop and park the station.
 *
 * MUST be called before espnow_transport_init() in field mode.
 *
 * wifi_init()'s disconnect handler retries esp_wifi_connect() forever. With no
 * AP present each retry makes the co-processor SCAN ALL CHANNELS for the SSID,
 * which drags the radio off the ESP-NOW channel — and the channel is never
 * restored afterwards, because esp_wifi_set_channel() is only applied once when
 * the ESP-NOW session is set up. The bridge then reports "ready" while every
 * packet leaves on the wrong channel.
 */
void wifi_manager_stop_reconnect(void);
