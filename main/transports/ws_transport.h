#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief Initialize the WebSocket transport and register it with the pipeline.
 *        Must be called after pipeline_init() and after httpd is started.
 * @param server  The httpd handle to register the /ws URI on.
 */
esp_err_t ws_transport_init(httpd_handle_t server);

/** httpd close callback — removes stale WS clients on any socket closure */
void ws_transport_close_fd(httpd_handle_t hd, int fd);

/** Returns the current number of connected WebSocket clients */
int ws_transport_client_count(void);
