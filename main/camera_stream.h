#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Start the MJPEG HTTP stream server.
 *
 * Serves MJPEG at http://<ip>/stream using esp_http_server.
 * Must be called after wifi_init().
 *
 * @param[out] out_handle  Returns the httpd_handle_t for registering additional URI handlers.
 */
esp_err_t camera_stream_server_start(httpd_handle_t *out_handle);
