#include "ws_transport.h"
#include "pipeline.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <unistd.h>

static const char *TAG = "WS_TRANSPORT";

#define WS_MAX_CLIENTS 4

/* ---- Client tracking (mutex-protected) ---- */

static httpd_handle_t s_server = NULL;
static int s_client_fds[WS_MAX_CLIENTS];
static int s_client_count = 0;
static SemaphoreHandle_t s_client_mutex = NULL;

static void add_client(int fd)
{
    xSemaphoreTake(s_client_mutex, portMAX_DELAY);

    /* Deduplicate — browser refresh reuses the same fd without a CLOSE frame */
    for (int i = 0; i < s_client_count; i++) {
        if (s_client_fds[i] == fd) {
            ESP_LOGI(TAG, "Client reconnected (fd=%d, total=%d)", fd, s_client_count);
            xSemaphoreGive(s_client_mutex);
            return;
        }
    }

    if (s_client_count < WS_MAX_CLIENTS) {
        s_client_fds[s_client_count++] = fd;
        ESP_LOGI(TAG, "Client connected (fd=%d, total=%d)", fd, s_client_count);
    } else {
        ESP_LOGW(TAG, "Max WS clients reached, rejecting fd=%d", fd);
    }
    xSemaphoreGive(s_client_mutex);
}

static void remove_client(int fd)
{
    xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    bool found = false;
    for (int i = 0; i < s_client_count; i++) {
        if (s_client_fds[i] == fd) {
            s_client_fds[i] = s_client_fds[s_client_count - 1];
            s_client_count--;
            ESP_LOGI(TAG, "Client removed (fd=%d, total=%d)", fd, s_client_count);
            found = true;
            break;
        }
    }
    if (!found) {
        ESP_LOGW(TAG, "remove_client: fd=%d not found (already removed or recycled)", fd);
    }
    xSemaphoreGive(s_client_mutex);
}

/* ---- Transport send (called from sensor task) ---- */

static esp_err_t ws_transport_send(const uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;

    /* Snapshot client list under mutex */
    int fds[WS_MAX_CLIENTS];
    int count;
    xSemaphoreTake(s_client_mutex, portMAX_DELAY);
    count = s_client_count;
    memcpy(fds, s_client_fds, count * sizeof(int));
    xSemaphoreGive(s_client_mutex);

    if (count == 0) return ESP_OK;

    httpd_ws_frame_t frame = {
        .type    = HTTPD_WS_TYPE_BINARY,
        .payload = (uint8_t *)buf,
        .len     = len,
        .final   = true,
    };

    for (int i = 0; i < count; i++) {
        esp_err_t ret = httpd_ws_send_frame_async(s_server, fds[i], &frame);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Send failed fd=%d: %s — removing stale client",
                     fds[i], esp_err_to_name(ret));
            remove_client(fds[i]);
        }
    }

    return ESP_OK;
}

/* ---- WebSocket handler (runs in httpd task) ---- */

static esp_err_t ws_handler(httpd_req_t *req)
{
    /* New connection */
    if (req->method == HTTP_GET) {
        add_client(httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    /* Incoming frame */
    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_BINARY };

    /* First call: get frame type and length */
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    /* Handle close frame — remove client immediately */
    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        remove_client(httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    if (frame.fragmented) {
        ESP_LOGW(TAG, "Fragmented WS frame ignored (fd=%d)", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    if (frame.len == 0) return ESP_OK;

    /* Bounded stack buffer for incoming commands (MotorCommand ~8 bytes).
     * Reject oversized frames to prevent stack issues. */
    if (frame.len > 128) {
        ESP_LOGW(TAG, "WS frame too large (%d bytes), ignoring", (int)frame.len);
        return ESP_OK;
    }

    uint8_t payload[128];
    frame.payload = payload;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK && frame.type == HTTPD_WS_TYPE_BINARY) {
        pipeline_handle_incoming(payload, frame.len);
    }

    return ret;
}

/* ---- Public API ---- */

esp_err_t ws_transport_init(httpd_handle_t server)
{
    s_server = server;
    if (s_client_mutex != NULL) {
        vSemaphoreDelete(s_client_mutex);
        s_client_mutex = NULL;
    }
    s_client_count = 0;
    memset(s_client_fds, -1, sizeof(s_client_fds));

    s_client_mutex = xSemaphoreCreateMutex();
    if (!s_client_mutex) {
        ESP_LOGE(TAG, "Failed to create client mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Register WS URI */
    static const httpd_uri_t ws_uri = {
        .uri            = "/ws",
        .method         = HTTP_GET,
        .handler        = ws_handler,
        .user_ctx       = NULL,
        .is_websocket   = true,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &ws_uri),
                        TAG, "register /ws failed");

    /* Disconnect cleanup relies on send-failure detection in ws_transport_send().
     * ESP-IDF does not reliably deliver close events for all disconnect scenarios
     * (client crash, network drop), so stale fds are removed when send fails. */

    /* Register with pipeline */
    ESP_RETURN_ON_ERROR(pipeline_register_transport(ws_transport_send, NULL),
                        TAG, "pipeline register failed");

    ESP_LOGI(TAG, "WebSocket transport ready on /ws (max %d clients)", WS_MAX_CLIENTS);
    return ESP_OK;
}
