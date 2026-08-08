/*
 * espnow_bridge.c
 *
 * ESP-NOW bridge running on the ESP32-C6 slave coprocessor.
 *
 * Receives data from the P4 host via esp_hosted custom data callbacks and
 * forwards it over ESP-NOW to the remote peer, and vice-versa.
 *
 * Two send paths:
 *  - Video / bulk data (PEER_MSG_VIDEO):  queued → espnow_tx_task fragments
 *    into 244-byte chunks and sends via esp_now_send().
 *  - Commands (PEER_MSG_COMMAND):  sent directly from the callback to avoid
 *    any latency added by the queue.
 *
 * Initialisation (PEER_MSG_INIT):  the P4 host sends channel (1 byte) followed
 * by the peer MAC address (6 bytes).  The callback calls esp_now_init() and
 * adds the peer before enabling the receive path.
 */

#include "espnow_bridge.h"

#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_hosted_peer_data.h"

/* -------------------------------------------------------------------------
 * Constants – replicated from the shared protocol header since this is a
 * separate build from the P4 project.
 * ---------------------------------------------------------------------- */

#define PEER_MSG_VIDEO    1u
#define PEER_MSG_COMMAND  2u
#define PEER_MSG_INIT     3u
#define PEER_MSG_UPSTREAM 4u

/* ESP-NOW packet layout (ESPNowCam-style fragmentation):
 *   [uint32_t total_len][up to FRAG_DATA_SIZE bytes of payload]
 *
 *   total_len == 0           → intermediate chunk
 *   total_len == frame_size  → last chunk of the frame
 *
 * The outer ESP-NOW frame is limited to 250 bytes; we use 244 to leave room
 * for a small header should the receiving side need it.
 */
#define ESPNOW_MAX_PAYLOAD 244u
#define FRAG_HDR_SIZE      4u                             /* sizeof(uint32_t) */
#define FRAG_DATA_SIZE     (ESPNOW_MAX_PAYLOAD - FRAG_HDR_SIZE)  /* 240 */

/* TX queue depth – enough to buffer a couple of JPEG frames */
#define TX_QUEUE_DEPTH     8u
#define TX_TASK_STACK      4096u
#define TX_TASK_PRIORITY   5u

/* Delay between successive ESP-NOW sends to avoid back-pressure */
#define INTER_CHUNK_DELAY_MS 1u

static const char *TAG = "espnow_bridge";

/* -------------------------------------------------------------------------
 * Forward declarations
 * ---------------------------------------------------------------------- */
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len);

/* -------------------------------------------------------------------------
 * Static state
 * ---------------------------------------------------------------------- */
typedef struct {
    uint8_t  *data;
    size_t    len;
} tx_item_t;

static uint8_t         s_peer_mac[ESP_NOW_ETH_ALEN];
static volatile bool   s_espnow_ready = false;
static QueueHandle_t   s_tx_queue     = NULL;
static TaskHandle_t    s_tx_task      = NULL;

/* -------------------------------------------------------------------------
 * espnow_frag_send
 *
 * Fragments `data` (length `total_len`) into FRAG_DATA_SIZE chunks and sends
 * each as a separate ESP-NOW packet.  A 1 ms delay is inserted between
 * consecutive sends so the driver does not drop frames under load.
 *
 * Packet format per chunk:
 *   bytes [0..3]  uint32_t  0            if this is NOT the last chunk
 *                           total_len    if this IS the last chunk
 *   bytes [4..]   up to FRAG_DATA_SIZE bytes of payload
 * ---------------------------------------------------------------------- */
static void espnow_frag_send(const uint8_t *data, size_t total_len)
{
    uint8_t  pkt[ESPNOW_MAX_PAYLOAD];
    size_t   offset = 0;

    while (offset < total_len) {
        size_t   chunk = total_len - offset;
        bool     last  = (chunk <= FRAG_DATA_SIZE);

        if (chunk > FRAG_DATA_SIZE) {
            chunk = FRAG_DATA_SIZE;
        }

        /* Header: 0 for intermediate chunks, total_len for the last one */
        uint32_t hdr = last ? (uint32_t)total_len : 0u;
        memcpy(pkt, &hdr, FRAG_HDR_SIZE);
        memcpy(pkt + FRAG_HDR_SIZE, data + offset, chunk);

        size_t pkt_len = FRAG_HDR_SIZE + chunk;
        esp_err_t err = esp_now_send(s_peer_mac, pkt, pkt_len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_send failed (offset=%zu): %s",
                     offset, esp_err_to_name(err));
        }

        offset += chunk;

        if (!last) {
            vTaskDelay(pdMS_TO_TICKS(INTER_CHUNK_DELAY_MS));
        }
    }
}

/* -------------------------------------------------------------------------
 * espnow_tx_task
 *
 * FreeRTOS task that drains the video TX queue.  Each item is a heap-
 * allocated buffer; the task is responsible for freeing it after sending.
 * ---------------------------------------------------------------------- */
static void espnow_tx_task(void *arg)
{
    tx_item_t item;

    for (;;) {
        if (xQueueReceive(s_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (s_espnow_ready) {
                espnow_frag_send(item.data, item.len);
            } else {
                ESP_LOGW(TAG, "tx_task: ESP-NOW not ready, dropping frame");
            }
            free(item.data);
        }
    }
}

/* -------------------------------------------------------------------------
 * video_cb
 *
 * Invoked by esp_hosted in the RPC RX thread for PEER_MSG_VIDEO messages.
 * Copies the payload onto the heap and enqueues a tx_item_t.  If the queue
 * is full the frame is dropped and the buffer freed immediately.
 *
 * Must return quickly — no blocking operations here.
 *
 * Uses xQueueSend() with a zero timeout, not xQueueSendFromISR(): the RPC RX
 * thread is a normal FreeRTOS task, not an ISR (the doc comment above always
 * said so — the ISR-safe API was simply the wrong call for that context).
 * The ISR variant skips the scheduler bookkeeping xQueueSend() does on this
 * queue's OTHER (correct, task-context) callers, so calling it here-only,
 * repeatedly, under sustained traffic, risked corrupting whatever internal
 * queue state those callers rely on being consistent. A zero timeout keeps
 * the exact same non-blocking, drop-if-full behaviour as before.
 * ---------------------------------------------------------------------- */
static void video_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;

    uint8_t *buf = malloc(data_len);
    if (!buf) {
        ESP_LOGE(TAG, "video_cb: malloc failed (len=%zu)", data_len);
        return;
    }

    memcpy(buf, data, data_len);

    tx_item_t item = { .data = buf, .len = data_len };

    if (xQueueSend(s_tx_queue, &item, 0) != pdTRUE) {
        /* Queue full — drop this frame */
        ESP_LOGW(TAG, "video_cb: tx_queue full, dropping frame (len=%zu)",
                 data_len);
        free(buf);
    }
}

/* -------------------------------------------------------------------------
 * command_cb
 *
 * Invoked for PEER_MSG_COMMAND.  Commands are small and latency-sensitive so
 * they bypass the queue and go directly to esp_now_send().
 * ---------------------------------------------------------------------- */
static void command_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;

    if (!s_espnow_ready) {
        ESP_LOGW(TAG, "command_cb: ESP-NOW not ready, dropping command");
        return;
    }

    if (data_len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "command_cb: oversized command (%zu > %u), truncating",
                 data_len, ESPNOW_MAX_PAYLOAD);
        data_len = ESPNOW_MAX_PAYLOAD;
    }

    esp_err_t err = esp_now_send(s_peer_mac, data, data_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "command_cb: esp_now_send failed: %s",
                 esp_err_to_name(err));
    }
}

/* -------------------------------------------------------------------------
 * init_cb
 *
 * Payload layout (7 bytes):
 *   byte  0      : WiFi channel (1–13)
 *   bytes 1..6   : peer MAC address
 *
 * Initialises ESP-NOW, adds the peer, and registers the receive callback.
 * Idempotent: if called again esp_now_deinit() is called first to reset state.
 * ---------------------------------------------------------------------- */
static void init_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;

    if (data_len < 1 + ESP_NOW_ETH_ALEN) {
        ESP_LOGE(TAG, "init_cb: payload too short (%zu bytes, need %d)",
                 data_len, 1 + ESP_NOW_ETH_ALEN);
        return;
    }

    uint8_t channel = data[0];
    memcpy(s_peer_mac, data + 1, ESP_NOW_ETH_ALEN);

    ESP_LOGI(TAG, "init_cb: channel=%u, peer=" MACSTR, channel,
             MAC2STR(s_peer_mac));

    /* Tear down any previous session */
    if (s_espnow_ready) {
        s_espnow_ready = false;
        esp_now_deinit();
    }

    /* WiFi must be started before esp_now_init(); caller is responsible for
     * calling esp_wifi_start().  Set the channel here. */
    esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "init_cb: esp_wifi_set_channel failed: %s",
                 esp_err_to_name(err));
        /* Non-fatal — proceed anyway; the channel the AP set may be fine */
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: esp_now_init failed: %s",
                 esp_err_to_name(err));
        return;
    }

    /* Register the receive callback before adding the peer so no packets
     * are missed in the narrow window between the two calls. */
    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: register recv_cb failed: %s",
                 esp_err_to_name(err));
        esp_now_deinit();
        return;
    }

    esp_now_peer_info_t peer_info = {
        .channel = channel,
        .ifidx   = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer_info.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);

    err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init_cb: esp_now_add_peer failed: %s",
                 esp_err_to_name(err));
        esp_now_deinit();
        return;
    }

    s_espnow_ready = true;
    ESP_LOGI(TAG, "ESP-NOW bridge ready");
}

/* -------------------------------------------------------------------------
 * espnow_recv_cb
 *
 * ESP-NOW receive callback (called from the ESP-NOW task context).
 * Forwards received data back to the P4 host via esp_hosted.
 * ---------------------------------------------------------------------- */
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    if (len <= 0 || !data) {
        return;
    }

    ESP_LOGD(TAG, "espnow_recv_cb: %d bytes from " MACSTR,
             len, MAC2STR(info->src_addr));

    esp_err_t err = esp_hosted_send_custom_data(PEER_MSG_UPSTREAM,
                                                data, (size_t)len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "espnow_recv_cb: esp_hosted_send_custom_data failed: %s",
                 esp_err_to_name(err));
    }
}

/* -------------------------------------------------------------------------
 * espnow_bridge_init  (public API)
 * ---------------------------------------------------------------------- */
esp_err_t espnow_bridge_init(void)
{
    /* Create the video TX queue */
    s_tx_queue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_item_t));
    if (!s_tx_queue) {
        ESP_LOGE(TAG, "espnow_bridge_init: failed to create tx_queue");
        return ESP_ERR_NO_MEM;
    }

    /* Start the TX task */
    BaseType_t ret = xTaskCreate(espnow_tx_task, "espnow_tx",
                                 TX_TASK_STACK, NULL,
                                 TX_TASK_PRIORITY, &s_tx_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "espnow_bridge_init: failed to create tx_task");
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Register hosted callbacks */
    esp_err_t err;

    err = esp_hosted_register_custom_callback(PEER_MSG_VIDEO, video_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register video_cb failed: %s", esp_err_to_name(err));
        goto fail;
    }

    err = esp_hosted_register_custom_callback(PEER_MSG_COMMAND, command_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register command_cb failed: %s", esp_err_to_name(err));
        goto fail;
    }

    err = esp_hosted_register_custom_callback(PEER_MSG_INIT, init_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register init_cb failed: %s", esp_err_to_name(err));
        goto fail;
    }

    ESP_LOGI(TAG, "espnow_bridge_init: callbacks registered, waiting for INIT");
    return ESP_OK;

fail:
    vTaskDelete(s_tx_task);
    s_tx_task = NULL;
    vQueueDelete(s_tx_queue);
    s_tx_queue = NULL;
    return err;
}
