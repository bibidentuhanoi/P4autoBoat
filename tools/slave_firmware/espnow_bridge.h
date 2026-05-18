#pragma once
#include "esp_err.h"

/**
 * @brief Initialize the ESP-NOW bridge.
 *
 * Creates the TX queue, starts the TX task, and registers custom data
 * callbacks for PEER_MSG_VIDEO, PEER_MSG_COMMAND, and PEER_MSG_INIT
 * from the P4 host via esp_hosted.
 *
 * Call once from app_main() before the system enters its main loop.
 *
 * @return ESP_OK on success, or an error code if resource allocation fails.
 */
esp_err_t espnow_bridge_init(void);
