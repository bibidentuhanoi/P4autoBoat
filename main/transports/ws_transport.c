#include "ws_transport.h"
#include "pipeline.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static const char *TAG = "WS_TRANSPORT";

#define WS_MAX_CLIENTS 4
#define WS_SLOT_SIZE   13000
#define WS_TX_STACK    8192
#define WS_TX_PRIORITY 3  /* below sensor tasks (4), above httpd (1) — consumer yields to producers */

/* ---- Client tracking (mutex-protected) ---- */

static httpd_handle_t s_server = NULL;
static int s_client_fds[WS_MAX_CLIENTS];
static int s_client_count = 0;
static SemaphoreHandle_t s_client_mutex = NULL;

/* ---- Double-buffer slot: sensor writes one, TX task reads the other ---- */

static uint8_t  s_slot_buf[2][WS_SLOT_SIZE];
static size_t   s_slot_len[2];
static volatile int s_write_idx = 0;
static TaskHandle_t s_tx_task = NULL;

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
        /* Cap send blocking to 50ms — prevents esp_hosted spinlock from
         * holding interrupts long enough to trigger the 300ms WDT */
        struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

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
/* Writes into the double-buffer slot and notifies the TX task.
 * Never touches the WiFi stack — returns immediately. */

static esp_err_t ws_transport_send(const uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;
    if (len > WS_SLOT_SIZE) return ESP_ERR_INVALID_SIZE;

    int idx = s_write_idx;
    memcpy(s_slot_buf[idx], buf, len);
    s_slot_len[idx] = len;

    /* Flip: TX task will read this buffer; next sensor write goes to the other */
    s_write_idx = idx ^ 1;

    /* Wake TX task */
    if (s_tx_task) {
        xTaskNotifyGive(s_tx_task);
    }

    return ESP_OK;
}

/* ---- TX task: drains the latest slot over WiFi ---- */

static void ws_tx_task(void *arg)
{
    (void)arg;

    /* Local copy — sensor can freely overwrite the slot while we send */
    static uint8_t tx_buf[WS_SLOT_SIZE];

    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        /* Snapshot the completed buffer before sensor can overwrite it */
        int read_idx = s_write_idx ^ 1;
        size_t len = s_slot_len[read_idx];
        if (len == 0) continue;
        memcpy(tx_buf, s_slot_buf[read_idx], len);

        /* Snapshot client list under mutex */
        int fds[WS_MAX_CLIENTS];
        int count;
        xSemaphoreTake(s_client_mutex, portMAX_DELAY);
        count = s_client_count;
        memcpy(fds, s_client_fds, count * sizeof(int));
        xSemaphoreGive(s_client_mutex);

        if (count == 0) continue;

        httpd_ws_frame_t frame = {
            .type    = HTTPD_WS_TYPE_BINARY,
            .payload = tx_buf,
            .len     = len,
            .final   = true,
        };

        for (int i = 0; i < count; i++) {
            /* Verify fd is still a valid httpd session before async send.
             * Between snapshot and here, the socket may have been closed and
             * the fd recycled — sending to a recycled fd corrupts heap. */
            if (httpd_sess_update_lru_counter(s_server, fds[i]) == ESP_ERR_NOT_FOUND) {
                remove_client(fds[i]);
                continue;
            }
            esp_err_t ret = httpd_ws_send_frame_async(s_server, fds[i], &frame);
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "Send failed fd=%d: %s", fds[i], esp_err_to_name(ret));
                remove_client(fds[i]);
            }
        }
    }
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

/* httpd close callback — fires for ALL socket closures (clean or dirty) */
void ws_transport_close_fd(httpd_handle_t hd, int fd)
{
    remove_client(fd);
    close(fd);
}

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

    /* Start TX task — owns all WiFi sends, decoupled from sensor task */
    BaseType_t ret_task = xTaskCreate(ws_tx_task, "WS_TX", WS_TX_STACK,
                                      NULL, WS_TX_PRIORITY, &s_tx_task);
    if (ret_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create WS TX task");
        return ESP_ERR_NO_MEM;
    }

    /* Register with pipeline */
    ESP_RETURN_ON_ERROR(pipeline_register_transport(ws_transport_send, NULL),
                        TAG, "pipeline register failed");

    ESP_LOGI(TAG, "WebSocket transport ready on /ws (max %d clients)", WS_MAX_CLIENTS);
    return ESP_OK;
}
