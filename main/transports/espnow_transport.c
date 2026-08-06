#include "espnow_transport.h"
#include "espnow_protocol.h"
#include "pipeline.h"
#include "esp_hosted_misc.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "proto/boat.pb.h"

static const char *TAG = "ESPNOW_TRANSPORT";

/* ---------------------------------------------------------------------------
 * peer_data msg_id constants
 * -------------------------------------------------------------------------*/
#define PEER_MSG_VIDEO    1u   /* P4→C6: JPEG + sensor data (queued on C6) */
#define PEER_MSG_COMMAND  2u   /* P4→C6: commands (fast path on C6) */
#define PEER_MSG_INIT     3u   /* P4→C6: ESP-NOW init */
#define PEER_MSG_UPSTREAM 4u   /* C6→P4: commands from laptop */
#define PEER_DATA_MAX  8166u   /* max bytes per esp_hosted_send_custom_data call */

/* Maximum JPEG payload that fits in one peer_data frame after the header */
#define JPEG_CHUNK_MAX  (PEER_DATA_MAX - ESPNOW_HDR_SIZE)

/* Static send buffer shared by espnow_send_fn (sensor path, single-task) */
static uint8_t s_send_buf[PEER_DATA_MAX];

/* Ground-station presence, set from upstream_cb (RPC RX thread) */
#define GROUND_SEEN_BIT  BIT0
static EventGroupHandle_t s_ground_evt = NULL;
static volatile bool      s_ground_seen = false;

/* ---------------------------------------------------------------------------
 * parse_mac_string — "AA:BB:CC:DD:EE:FF" → 6 bytes; falls back to broadcast
 * -------------------------------------------------------------------------*/
static void parse_mac_string(const char *str, uint8_t mac[6])
{
    if (!str || strlen(str) < 17) {
        goto broadcast;
    }
    unsigned v[6];
    if (sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        goto broadcast;
    }
    for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)v[i];
    }
    return;

broadcast:
    ESP_LOGW(TAG, "Invalid peer MAC \"%s\", falling back to broadcast", str ? str : "(null)");
    memset(mac, 0xFF, 6);
}

/* ---------------------------------------------------------------------------
 * upstream_cb — registered for PEER_MSG_UPSTREAM (C6→P4 commands)
 * -------------------------------------------------------------------------*/
static void upstream_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;

    if (!data || data_len < ESPNOW_HDR_SIZE) {
        ESP_LOGW(TAG, "upstream_cb: short frame (%u bytes)", (unsigned)data_len);
        return;
    }

    const espnow_pkt_hdr_t *hdr = (const espnow_pkt_hdr_t *)data;
    uint16_t payload_len = hdr->payload_len;

    /* Ground-station beacon: presence signal only, never application data. */
    if (hdr->msg_type == MSG_GROUND_HELLO) {
        if (!s_ground_seen) {
            ESP_LOGI(TAG, "Ground station detected (MSG_GROUND_HELLO)");
        }
        s_ground_seen = true;
        if (s_ground_evt) {
            xEventGroupSetBits(s_ground_evt, GROUND_SEEN_BIT);
        }
        return;
    }

    if ((size_t)(ESPNOW_HDR_SIZE + payload_len) > data_len) {
        ESP_LOGW(TAG, "upstream_cb: payload_len %u overflows frame %u",
                 (unsigned)payload_len, (unsigned)data_len);
        return;
    }

    const uint8_t *payload = data + ESPNOW_HDR_SIZE;
    pipeline_handle_incoming(payload, payload_len);
}

/* ---------------------------------------------------------------------------
 * espnow_send_fn — transport_send_fn registered with the pipeline
 * Wraps nanopb-encoded BoatMessage bytes with espnow_pkt_hdr_t (MSG_SENSOR)
 * and sends via PEER_MSG_VIDEO.
 * -------------------------------------------------------------------------*/
static esp_err_t espnow_send_fn(const uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;

    if (ESPNOW_HDR_SIZE + len > PEER_DATA_MAX) {
        ESP_LOGW(TAG, "espnow_send_fn: payload %u bytes exceeds PEER_DATA_MAX", (unsigned)len);
        return ESP_ERR_INVALID_SIZE;
    }

    espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)s_send_buf;
    hdr->msg_type    = MSG_SENSOR;
    hdr->payload_len = (uint16_t)len;
    hdr->seq         = 0;  /* sensor frames are standalone; seq unused */

    memcpy(s_send_buf + ESPNOW_HDR_SIZE, buf, len);

    esp_err_t ret = esp_hosted_send_custom_data(PEER_MSG_VIDEO,
                                                s_send_buf,
                                                ESPNOW_HDR_SIZE + len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "espnow_send_fn: send failed (%s)", esp_err_to_name(ret));
    }
    return ret;
}

/* ---------------------------------------------------------------------------
 * espnow_transport_send_jpeg — fragments JPEG into ≤JPEG_CHUNK_MAX chunks
 * -------------------------------------------------------------------------*/
esp_err_t espnow_transport_send_jpeg(const uint8_t *jpg, size_t len)
{
    if (!jpg || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    static uint8_t seq = 0;
    size_t offset = 0;

    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > JPEG_CHUNK_MAX) {
            chunk = JPEG_CHUNK_MAX;
        }

        espnow_pkt_hdr_t hdr = {
            .msg_type    = MSG_JPEG_CHUNK,
            .payload_len = (uint16_t)chunk,
            .seq         = seq++,
        };

        /* Use stack-local buf for header + this chunk (fits because chunk ≤ JPEG_CHUNK_MAX) */
        static uint8_t chunk_buf[PEER_DATA_MAX];
        memcpy(chunk_buf, &hdr, ESPNOW_HDR_SIZE);
        memcpy(chunk_buf + ESPNOW_HDR_SIZE, jpg + offset, chunk);

        esp_err_t ret = esp_hosted_send_custom_data(PEER_MSG_VIDEO,
                                                    chunk_buf,
                                                    ESPNOW_HDR_SIZE + chunk);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "send_jpeg: chunk send failed at offset %u (%s)",
                     (unsigned)offset, esp_err_to_name(ret));
            return ret;
        }

        offset += chunk;
    }

    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * espnow_transport_probe — bring ESP-NOW up, then listen for the S3 beacon
 * -------------------------------------------------------------------------*/
esp_err_t espnow_transport_probe(uint32_t timeout_ms)
{
    esp_err_t ret;

    if (!s_ground_evt) {
        s_ground_evt = xEventGroupCreate();
        if (!s_ground_evt) {
            return ESP_ERR_NO_MEM;
        }
    }

    /* --- Register upstream callback for C6→P4 commands --- */
    ret = esp_hosted_register_custom_callback(PEER_MSG_UPSTREAM, upstream_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register upstream callback (%s)", esp_err_to_name(ret));
        return ret;
    }

    /* --- Build and send MSG_ESPNOW_INIT --- */
    /*
     * Payload layout:
     *   [0..3]  espnow_pkt_hdr_t  (4 bytes)
     *   [4]     channel            (1 byte)
     *   [5..10] peer MAC           (6 bytes)
     */
    {
        uint8_t init_buf[ESPNOW_HDR_SIZE + 1 + 6];

        espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)init_buf;
        hdr->msg_type    = MSG_ESPNOW_INIT;
        hdr->payload_len = 1 + 6;
        hdr->seq         = 0;

        uint8_t channel = (uint8_t)CONFIG_ESPNOW_CHANNEL;
        init_buf[ESPNOW_HDR_SIZE] = channel;

        uint8_t peer_mac[6];
        parse_mac_string(CONFIG_ESPNOW_PEER_MAC, peer_mac);
        memcpy(init_buf + ESPNOW_HDR_SIZE + 1, peer_mac, 6);

        ret = esp_hosted_send_custom_data(PEER_MSG_INIT, init_buf, sizeof(init_buf));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send ESPNOW_INIT (%s)", esp_err_to_name(ret));
            return ret;
        }
    }

    /* --- Build and send MSG_ESPNOW_CONFIG --- */
    /*
     * Payload layout:
     *   [0..3]  espnow_pkt_hdr_t  (4 bytes)
     *   [4..5]  width  uint16_t   (little-endian)
     *   [6..7]  height uint16_t   (little-endian)
     */
    {
        uint8_t cfg_buf[ESPNOW_HDR_SIZE + 2 + 2];

        espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)cfg_buf;
        hdr->msg_type    = MSG_ESPNOW_CONFIG;
        hdr->payload_len = 2 + 2;
        hdr->seq         = 0;

        uint16_t width  = 800;
        uint16_t height = 640;
        memcpy(cfg_buf + ESPNOW_HDR_SIZE,     &width,  2);
        memcpy(cfg_buf + ESPNOW_HDR_SIZE + 2, &height, 2);

        ret = esp_hosted_send_custom_data(PEER_MSG_VIDEO, cfg_buf, sizeof(cfg_buf));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send ESPNOW_CONFIG (%s)", esp_err_to_name(ret));
            return ret;
        }
    }

    /* --- Listen for the ground station -------------------------------------
     * The S3 bridge broadcasts MSG_GROUND_HELLO every ESPNOW_HELLO_INTERVAL_MS.
     * Hearing one proves the whole chain is live: C6 radio on the right
     * channel, S3 powered and in range. If nothing arrives the caller falls
     * back to WiFi, so no pipeline transport is registered here. */
    ESP_LOGI(TAG, "Listening %ums for a ground station (ch=%d)...",
             (unsigned)timeout_ms, CONFIG_ESPNOW_CHANNEL);

    EventBits_t bits = xEventGroupWaitBits(s_ground_evt, GROUND_SEEN_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(timeout_ms));
    if (!(bits & GROUND_SEEN_BIT)) {
        ESP_LOGW(TAG, "No ground station beacon within %ums", (unsigned)timeout_ms);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * espnow_transport_activate — attach ESP-NOW to the pipeline
 * -------------------------------------------------------------------------*/
esp_err_t espnow_transport_activate(void)
{
    esp_err_t ret = pipeline_register_transport(espnow_send_fn, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register pipeline transport (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW transport ready (ch=%d peer=" CONFIG_ESPNOW_PEER_MAC ")",
             CONFIG_ESPNOW_CHANNEL);
    return ESP_OK;
}
