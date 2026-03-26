#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Start the MJPEG HTTP stream server on CONFIG_HTTP_STREAM_PORT (default 81).
 *
 * Self-contained — does not share its httpd handle. The API/dashboard
 * server runs separately on CONFIG_HTTP_API_PORT (default 80).
 */
esp_err_t camera_stream_server_start(void);
