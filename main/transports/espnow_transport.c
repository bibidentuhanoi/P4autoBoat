#include "espnow_transport.h"
#include "espnow_protocol.h"
#include "pipeline.h"
#include "esp_hosted_misc.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
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

/* Per-chunk sub-header inside a MSG_JPEG_CHUNK payload:
 *   [0] frame_id   same for every chunk of one image
 *   [1] chunk_idx  0-based order
 *   [2] n_chunks   total for this image
 */
#define JPEG_SUBHDR_SIZE      3
/* Deliberately well under PEER_DATA_MAX (8166). The first attempt sized chunks
 * at exactly 8166 total — sitting precisely on a documented limit — and nothing
 * ever reached the air. 4 KB keeps a wide margin at the cost of a few more
 * chunks. */
#define JPEG_CHUNK_BYTES      4096u
/* Gap between chunks so telemetry can interleave (see send_jpeg). */
#define JPEG_INTER_CHUNK_MS   40

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
/* Tell the ground station what a JPEG attempt did: rc 0 = sent. */
static void espnow_report_jpeg(uint8_t rc, uint16_t size, uint8_t n_chunks)
{
    uint8_t buf[ESPNOW_HDR_SIZE + 4];
    espnow_pkt_hdr_t *h = (espnow_pkt_hdr_t *)buf;
    h->msg_type = MSG_JPEG_STATUS;
    h->payload_len = 4;
    h->seq = 0;
    buf[ESPNOW_HDR_SIZE + 0] = rc;
    buf[ESPNOW_HDR_SIZE + 1] = n_chunks;
    memcpy(&buf[ESPNOW_HDR_SIZE + 2], &size, 2);
    esp_hosted_send_custom_data(PEER_MSG_VIDEO, buf, sizeof(buf));
}

esp_err_t espnow_transport_send_jpeg(const uint8_t *jpg, size_t len)
{
    if (!jpg || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Payload of every MSG_JPEG_CHUNK is:
     *     [frame_id][chunk_idx][n_chunks] then JPEG bytes
     *
     * The old scheme put a rolling counter in espnow_pkt_hdr_t.seq and
     * incremented it PER CHUNK, while the receiver treated a changed seq as
     * "new frame — reset buffer". Every chunk therefore wiped the previous one
     * and a multi-chunk JPEG could never reassemble. frame_id now stays
     * constant across one image, chunk_idx orders them, and n_chunks lets the
     * receiver know when it is complete (and detect a missing chunk instead of
     * decoding garbage). */
    static uint8_t frame_id = 0;
    frame_id++;

    const size_t payload_max = JPEG_CHUNK_BYTES;
    const size_t n_chunks    = (len + payload_max - 1) / payload_max;

    if (n_chunks > 255) {
        ESP_LOGW(TAG, "send_jpeg: %u bytes needs %u chunks (max 255)",
                 (unsigned)len, (unsigned)n_chunks);
        return ESP_ERR_INVALID_SIZE;
    }

    static uint8_t chunk_buf[PEER_DATA_MAX];
    size_t offset = 0;

    for (size_t i = 0; i < n_chunks; i++) {
        size_t chunk = len - offset;
        if (chunk > payload_max) {
            chunk = payload_max;
        }

        espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)chunk_buf;
        hdr->msg_type    = MSG_JPEG_CHUNK;
        hdr->payload_len = (uint16_t)(JPEG_SUBHDR_SIZE + chunk);
        hdr->seq         = frame_id;      /* same for every chunk of this image */

        uint8_t *sub = chunk_buf + ESPNOW_HDR_SIZE;
        sub[0] = frame_id;
        sub[1] = (uint8_t)i;
        sub[2] = (uint8_t)n_chunks;
        memcpy(sub + JPEG_SUBHDR_SIZE, jpg + offset, chunk);

        esp_err_t ret = esp_hosted_send_custom_data(
            PEER_MSG_VIDEO, chunk_buf,
            ESPNOW_HDR_SIZE + JPEG_SUBHDR_SIZE + chunk);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "send_jpeg: chunk %u/%u failed (%s)",
                     (unsigned)(i + 1), (unsigned)n_chunks, esp_err_to_name(ret));
            espnow_report_jpeg((uint8_t)(i + 1), (uint16_t)len, (uint8_t)n_chunks);
            return ret;
        }

        offset += chunk;

        /* Breathe between chunks. Each ~8 KB chunk becomes ~34 ESP-NOW
         * fragments on the co-processor; firing several back-to-back is what
         * previously buried telemetry in the C6's 8-deep TX queue. Yielding
         * here lets 20 Hz snapshots interleave instead of queueing behind the
         * whole image. */
        if (i + 1 < n_chunks) {
            vTaskDelay(pdMS_TO_TICKS(JPEG_INTER_CHUNK_MS));
        }
    }

    espnow_report_jpeg(0, (uint16_t)len, (uint8_t)n_chunks);
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

    /* --- Build and send the ESP-NOW init payload ----------------------------
     * RAW payload, NO espnow_pkt_hdr_t. The C6's init_cb reads:
     *     channel  = data[0]
     *     peer_mac = data[1..6]
     * so it must receive exactly {channel, mac[6]} = 7 bytes.
     *
     * This previously sent a 4-byte header first, so the C6 read
     * channel = MSG_ESPNOW_INIT = 0x10 = 16 (invalid; the valid range is
     * 0-14) and peer_mac = the rest of the header (07:00:00:06:FF:FF).
     * esp_now_add_peer() then rejected .channel = 16 with ESP_ERR_ESPNOW_ARG,
     * and the C6's error path calls esp_now_deinit() — leaving the
     * co-processor with ESP-NOW torn down and completely deaf. The P4 never
     * saw it because init_cb is a void callback: the RPC reports success as
     * long as the handler ran.
     */
    {
        uint8_t init_buf[1 + 6];

        init_buf[0] = (uint8_t)CONFIG_ESPNOW_CHANNEL;
        parse_mac_string(CONFIG_ESPNOW_PEER_MAC, &init_buf[1]);

        ret = esp_hosted_send_custom_data(PEER_MSG_INIT, init_buf, sizeof(init_buf));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send ESPNOW_INIT (%s)", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Sent ESPNOW_INIT (ch=%d peer=" CONFIG_ESPNOW_PEER_MAC ")",
                 CONFIG_ESPNOW_CHANNEL);
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

    /* --- Own the channel from THIS side, and verify it -----------------------
     * The C6's init_cb is a void callback: it calls esp_wifi_set_channel() and
     * esp_now_init(), logs any failure to its own console (invisible to us),
     * and returns nothing. The RPC reports success merely because the handler
     * ran, so "MSG_ESPNOW_INIT sent OK" proves nothing about ESP-NOW actually
     * being up on the right channel.
     *
     * esp_wifi_set/get_channel() are proxied to the co-processor by
     * esp_wifi_remote and DO return a status, so drive the channel here and
     * read it back. Requires the station to be started but NOT associated —
     * which is exactly the state wifi_start_radio() leaves it in. */
    esp_err_t ch_err = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (ch_err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_channel(%d) failed: %s",
                 CONFIG_ESPNOW_CHANNEL, esp_err_to_name(ch_err));
    }

    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
        if (primary != CONFIG_ESPNOW_CHANNEL) {
            ESP_LOGE(TAG, "CHANNEL MISMATCH: co-processor is on %d, ESP-NOW needs %d "
                          "— the ground station will not be heard",
                     primary, CONFIG_ESPNOW_CHANNEL);
        } else {
            ESP_LOGI(TAG, "Co-processor confirmed on channel %d", primary);
        }
    } else {
        ESP_LOGW(TAG, "Could not read back the co-processor channel");
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
