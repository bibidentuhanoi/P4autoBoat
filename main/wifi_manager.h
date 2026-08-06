#pragma once
#include "esp_err.h"
#include <string.h>
/**
 * @brief Initialise netif/driver and START the radio, WITHOUT connecting.
 *
 * Split out from connecting because the co-processor requires esp_wifi_start()
 * before esp_now_init(), so ESP-NOW can be probed on a started-but-idle radio.
 * Deliberately does not scan or associate — a scan would hop the radio off the
 * ESP-NOW channel.
 */
esp_err_t wifi_start_radio(void);

/**
 * @brief Associate with the AP from Kconfig (CONFIG_WIFI_SSID / _PASSWORD).
 *
 * Requires wifi_start_radio() first. Blocks until an IP is assigned or the
 * 30s timeout elapses, and enables the auto-reconnect loop.
 */
esp_err_t wifi_connect(void);

/**
 * @brief Stop the auto-reconnect loop and park the station.
 *
 * MUST be called before espnow_transport_activate() in field mode.
 *
 * The disconnect handler retries esp_wifi_connect() forever. With no
 * AP present each retry makes the co-processor SCAN ALL CHANNELS for the SSID,
 * which drags the radio off the ESP-NOW channel — and the channel is never
 * restored afterwards, because esp_wifi_set_channel() is only applied once when
 * the ESP-NOW session is set up. The bridge then reports "ready" while every
 * packet leaves on the wrong channel.
 */
void wifi_manager_stop_reconnect(void);
