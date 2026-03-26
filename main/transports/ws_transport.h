#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Initialize the WebSocket transport and register it with the pipeline.
 *        Must be called after pipeline_init() and after httpd is started.
 * @param server  The httpd handle to register the /ws URI on.
 */
esp_err_t ws_transport_init(httpd_handle_t server);
