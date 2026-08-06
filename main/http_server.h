#pragma once
#include "esp_err.h"

/**
 * @brief Start the dashboard + WebSocket HTTP server on port 80.
 *        Registers: GET / (dashboard HTML), WS /ws (protobuf transport).
 *        Also initializes ws_transport and registers it with the pipeline.
 *        Call after pipeline_init() and wifi_connect().
 */
esp_err_t http_server_start(void);
