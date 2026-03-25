#pragma once
#include "esp_err.h"

/**
 * @brief Connect to WiFi AP configured via Kconfig (CONFIG_WIFI_SSID / CONFIG_WIFI_PASSWORD).
 *
 * Blocks until IP is assigned or a 30s timeout elapses.
 */
esp_err_t wifi_init(void);
