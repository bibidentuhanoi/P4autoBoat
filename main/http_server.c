#include "http_server.h"
#include "transports/ws_transport.h"
#include "drivers/camera_driver.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <unistd.h>

static const char *TAG = "HTTP_SERVER";

/* Embedded dashboard HTML (main/dashboard.html via EMBED_TXTFILES) */
extern const uint8_t dashboard_html_start[] asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[]   asm("_binary_dashboard_html_end");

static esp_err_t dashboard_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    /* no-store: the dashboard is embedded in the firmware image — a cached
     * copy silently survives reflashes and runs STALE JS against new firmware
     * (seen on hardware: controls dead because the cached page predated the
     * bench-override / proto changes). Page is ~80KB over LAN; refetch is cheap. */
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)dashboard_html_start,
                           dashboard_html_end - dashboard_html_start);
}

static esp_err_t snapshot_handler(httpd_req_t *req)
{
    void *buf = NULL;
    size_t len = 0;
    esp_err_t ret = ESP_FAIL;
    /* Retry to survive contention with MJPEG stream / detect capture.
     * Pattern mirrors detect_task.cpp for consistency. */
    for (int attempt = 0; attempt < 10; attempt++) {
        ret = camera_capture_frame(&buf, &len, NULL, NULL, NULL);
        if (ret == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "capture failed");
        return ret;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    ret = httpd_resp_send(req, (const char *)buf, (ssize_t)len);
    camera_release_frame();
    return ret;
}

esp_err_t http_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = CONFIG_HTTP_API_PORT;
    cfg.ctrl_port        = 32768;
    cfg.stack_size       = 8192;
    cfg.max_open_sockets = 7;
    cfg.max_uri_handlers = 4;
    cfg.recv_wait_timeout  = 30;   /* WebSocket clients are long-lived, don't kill after 5s */
    cfg.send_wait_timeout  = 10;
    cfg.keep_alive_enable  = true;
    cfg.keep_alive_idle    = 5;     /* start probing after 5s idle */
    cfg.keep_alive_interval = 3;    /* probe every 3s */
    cfg.keep_alive_count   = 3;     /* drop after 3 missed probes (~14s total) */
    cfg.close_fn           = ws_transport_close_fd;  /* clean up stale WS clients on any socket close */

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const httpd_uri_t dashboard_uri = {
        .uri = "/", .method = HTTP_GET,
        .handler = dashboard_handler, .user_ctx = NULL,
    };
    ret = httpd_register_uri_handler(server, &dashboard_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register / failed: %s", esp_err_to_name(ret));
        httpd_stop(server);
        return ret;
    }

    const httpd_uri_t snapshot_uri = {
        .uri = "/snapshot", .method = HTTP_GET,
        .handler = snapshot_handler, .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &snapshot_uri);

    ret = ws_transport_init(server);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ws_transport_init failed: %s", esp_err_to_name(ret));
        httpd_stop(server);
        return ret;
    }

    ESP_LOGI(TAG, "HTTP server ready on port %d: / /snapshot /ws",
             CONFIG_HTTP_API_PORT);
    return ESP_OK;
}
