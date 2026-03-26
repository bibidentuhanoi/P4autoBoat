#include "http_server.h"
#include "transports/ws_transport.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "sdkconfig.h"

static const char *TAG = "HTTP_SERVER";

/* Embedded dashboard HTML (main/dashboard.html via EMBED_TXTFILES) */
extern const uint8_t dashboard_html_start[] asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[]   asm("_binary_dashboard_html_end");

static esp_err_t dashboard_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)dashboard_html_start,
                           dashboard_html_end - dashboard_html_start);
}

esp_err_t http_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = CONFIG_HTTP_API_PORT;   /* 80 */
    cfg.ctrl_port        = 32768;
    cfg.stack_size       = 8192;
    cfg.max_open_sockets = 7;
    cfg.max_uri_handlers = 4;

    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &cfg), TAG, "httpd_start failed");

    /* Dashboard */
    static const httpd_uri_t dashboard_uri = {
        .uri = "/", .method = HTTP_GET,
        .handler = dashboard_handler, .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &dashboard_uri),
                        TAG, "register / failed");

    /* WebSocket transport (registers /ws and hooks into pipeline) */
    ESP_RETURN_ON_ERROR(ws_transport_init(server), TAG, "ws_transport_init failed");

    ESP_LOGI(TAG, "HTTP server ready on port %d: / (dashboard), /ws (protobuf)",
             CONFIG_HTTP_API_PORT);
    return ESP_OK;
}
