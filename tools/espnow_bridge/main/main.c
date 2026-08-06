#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "tinyusb.h"                /* -> tusb.h -> class/cdc/cdc_device.h
                                       (declares tud_cdc_n_connected) */
#include "tinyusb_default_config.h" /* TINYUSB_DEFAULT_CONFIG() */
#include "tinyusb_cdc_acm.h"

#include "cobs.h"

static const char *TAG = "bridge";

/* ---- Constants ---------------------------------------------------------- */
#define ESPNOW_CHANNEL      6
#define FRAG_HDR_SIZE       4
#define REASSEMBLY_BUF_SIZE (64 * 1024)   /* 64 KB — enough for a JPEG frame */

/* COBS overhead: at most 1 extra byte per 254 payload bytes + 1 delimiter */
#define COBS_MAX_OVERHEAD(n) ((n) / 254 + 2)

/* USB TX queue depth */
#define USB_TX_QUEUE_DEPTH  8

/* Per-flush timeout while draining the CDC TX FIFO.  FreeRTOS tick is 100 Hz
 * here, so the flush helper's internal vTaskDelay(1) costs 10 ms per stall;
 * this bounds a stalled frame instead of blocking the task indefinitely. */
#define USB_TX_FLUSH_TIMEOUT_MS 100

/* USB → ESP-NOW accumulator */
#define USB_RX_ACCUM_SIZE   512

/* Ground-station beacon. Broadcast so the boat can detect that a ground
 * station is present and choose ESP-NOW field mode over WiFi at boot.
 * Must match espnow_msg_type_t / ESPNOW_HELLO_INTERVAL_MS in
 * main/transports/espnow_protocol.h (separate build, so duplicated here). */
#define MSG_GROUND_HELLO         0x12
#define HELLO_INTERVAL_MS        500

/* ---- USB self-test ------------------------------------------------------- *
 * TEMPORARY BENCH AID.  Set to 0 for normal operation.
 *
 * When 1, a task emits synthetic COBS-framed frames at real telemetry sizes
 * through the SAME usb_tx_enqueue() -> usb_write_all() path production uses, so
 * it genuinely exercises the TX-FIFO truncation fix without needing the P4, the
 * C6, or the radio.  Verify on the laptop with:
 *     .venv/bin/python tools/usb_selftest_verify.py /dev/ttyACM0
 * ------------------------------------------------------------------------- */
#define USB_SELFTEST 0

/* ---- Types --------------------------------------------------------------- */
typedef struct {
    uint8_t *data;   /* heap-allocated; freed by usb_tx_task after write */
    size_t   len;
} usb_tx_item_t;

/* ---- Globals ------------------------------------------------------------- */
static QueueHandle_t s_usb_tx_queue;

/* Reassembly state for fragmented ESP-NOW packets */
static uint8_t  s_reassembly_buf[REASSEMBLY_BUF_SIZE];
static size_t   s_reassembly_len = 0;

/* USB RX accumulator (looks for 0x00 COBS delimiter) */
static uint8_t  s_usb_rx_accum[USB_RX_ACCUM_SIZE];
static size_t   s_usb_rx_accum_len = 0;

/* ---- Broadcast peer MAC -------------------------------------------------- */
static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] =
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ========================================================================== */
/*  USB TX task                                                                */
/* ========================================================================== */

/*
 * usb_write_all — write the WHOLE buffer to USB CDC, or give up cleanly.
 *
 * Why this is not a single write_queue() call:
 *   tinyusb_cdcacm_write_queue() queues only MIN(len, tud_cdc_n_write_available())
 *   and RETURNS how much it took.  The TX FIFO is CONFIG_TINYUSB_CDC_TX_BUFSIZE
 *   bytes, so any frame larger than the free FIFO space is silently truncated if
 *   the return value is ignored.  A COBS frame that loses its tail also loses its
 *   0x00 delimiter, so the host would glue the next frame onto the partial one and
 *   corrupt BOTH.  We therefore loop on the return value, and on failure emit a
 *   lone delimiter so the host resynchronises on the next frame.
 *
 * Returns true if every byte was queued and flushed.
 */
static bool usb_write_all(const uint8_t *data, size_t len)
{
    /* No host has the port open: flush can never drain (TinyUSB drops data when
     * the terminal is not connected), so writing would just burn the timeout on
     * every frame.  Drop instead — telemetry is best-effort. */
    if (!tud_cdc_n_connected(TINYUSB_CDC_ACM_0)) {
        return false;
    }

    size_t sent = 0;
    while (sent < len) {
        size_t n = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0,
                                              data + sent, len - sent);
        sent += n;

        if (sent < len) {
            /* FIFO full — block until it drains before queuing the rest.
             * A timeout here means the host stopped reading; abandon the frame. */
            if (tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0,
                                            pdMS_TO_TICKS(USB_TX_FLUSH_TIMEOUT_MS)) != ESP_OK) {
                return false;
            }
        }
    }

    /* Push the tail out of the FIFO. */
    return tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0,
                                       pdMS_TO_TICKS(USB_TX_FLUSH_TIMEOUT_MS)) == ESP_OK;
}

static void usb_tx_task(void *arg)
{
    usb_tx_item_t item;
    while (1) {
        if (xQueueReceive(s_usb_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (!usb_write_all(item.data, item.len)) {
                /* Partial or skipped frame: the COBS delimiter may not have gone
                 * out.  Emit a bare 0x00 (best effort) so the host flushes its
                 * accumulator and the NEXT frame decodes cleanly. */
                if (tud_cdc_n_connected(TINYUSB_CDC_ACM_0)) {
                    const uint8_t delim = 0x00;
                    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, &delim, 1);
                    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
                    ESP_LOGW(TAG, "USB TX incomplete — dropped frame (%u bytes)",
                             (unsigned)item.len);
                }
            }
            free(item.data);
        }
    }
}

/* Helper: enqueue a buffer for USB TX.  Takes ownership of buf (must be
   heap-allocated); frees it on enqueue failure. */
static void usb_tx_enqueue(uint8_t *buf, size_t len)
{
    usb_tx_item_t item = {.data = buf, .len = len};
    if (xQueueSend(s_usb_tx_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "USB TX queue full — dropping %u bytes", (unsigned)len);
        free(buf);
    }
}

#if USB_SELFTEST
/* ========================================================================== */
/*  USB self-test emitter (temporary — see USB_SELFTEST above)                 */
/* ========================================================================== */

/* Frame layout before COBS encoding:
 *   [0..3]   magic "TST1"
 *   [4..7]   seq        (uint32 LE)
 *   [8..11]  total_len  (uint32 LE)  <- receiver checks this vs bytes received
 *   [12..]   body, body[i] = (uint8_t)(i * 7 + seq)
 *
 * Sizes sweep the range that used to break: 300 B fit the old 512 B FIFO,
 * 1126 B is a trimmed snapshot, 2299 B a full one, 4000 B stresses the new
 * 4096 B FIFO.  Anything above 512 would have been truncated before the fix.
 */
static void usb_selftest_task(void *arg)
{
    static const size_t sizes[] = {300, 1126, 2299, 4000};
    const size_t nsizes = sizeof(sizes) / sizeof(sizes[0]);
    uint32_t seq = 0;

    while (1) {
        const size_t n = sizes[seq % nsizes];

        uint8_t *payload = malloc(n);
        if (payload) {
            memcpy(payload, "TST1", 4);
            uint32_t seq_le = seq, len_le = (uint32_t)n;
            memcpy(payload + 4, &seq_le, 4);
            memcpy(payload + 8, &len_le, 4);
            for (size_t i = 12; i < n; i++) {
                payload[i] = (uint8_t)(i * 7 + seq);
            }

            /* COBS worst case: +1 byte per 254, +1 code byte, +1 delimiter */
            uint8_t *cobs = malloc(n + n / 254 + 2);
            if (cobs) {
                size_t clen = cobs_encode(payload, n, cobs);
                cobs[clen++] = 0x00;
                usb_tx_enqueue(cobs, clen);   /* takes ownership of cobs */
            }
            free(payload);
        }

        seq++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif /* USB_SELFTEST */

/* ========================================================================== */
/*  Ground-station beacon                                                      */
/* ========================================================================== */

/*
 * Periodically broadcast a bare espnow_pkt_hdr_t{MSG_GROUND_HELLO, 0, seq}.
 *
 * The boat listens for this at boot: heard -> run ESP-NOW field mode, silence
 * -> fall back to connecting to WiFi. Sent raw (no fragment header), which is
 * what the C6 bridge forwards verbatim to the P4 as PEER_MSG_UPSTREAM.
 *
 * Beacons unconditionally, whether or not a laptop has the USB port open —
 * the S3 being powered IS the ground station being present.
 */
static void hello_beacon_task(void *arg)
{
    uint8_t seq = 0;
    while (1) {
        const uint8_t hello[4] = {
            MSG_GROUND_HELLO,
            0x00, 0x00,          /* payload_len = 0 (little-endian uint16) */
            seq++,
        };
        esp_err_t err = esp_now_send(BROADCAST_MAC, hello, sizeof(hello));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "hello beacon send failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(HELLO_INTERVAL_MS));
    }
}

/* ========================================================================== */
/*  ESP-NOW → USB path                                                         */
/* ========================================================================== */

/*
 * Fragment header layout (4 bytes, little-endian uint32_t):
 *   total_len == 0  → intermediate fragment; append payload to reassembly buf
 *   total_len >  0  → last fragment; append payload then ship total_len bytes
 *
 * The ESP-NOW MTU is 250 bytes, so the header eats 4 bytes, leaving 246 bytes
 * of payload per fragment.
 */
static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int data_len)
{
    if (data_len < (int)FRAG_HDR_SIZE) {
        ESP_LOGW(TAG, "Short ESP-NOW packet (%d bytes) — ignored", data_len);
        return;
    }

    uint32_t total_len;
    memcpy(&total_len, data, sizeof(total_len));   /* little-endian uint32_t */

    const uint8_t *payload     = data + FRAG_HDR_SIZE;
    size_t         payload_len = (size_t)(data_len - FRAG_HDR_SIZE);

    /* Append payload to reassembly buffer */
    if (s_reassembly_len + payload_len > REASSEMBLY_BUF_SIZE) {
        ESP_LOGE(TAG, "Reassembly buffer overflow — discarding %u bytes",
                 (unsigned)(s_reassembly_len + payload_len));
        s_reassembly_len = 0;
        return;
    }
    memcpy(s_reassembly_buf + s_reassembly_len, payload, payload_len);
    s_reassembly_len += payload_len;

    if (total_len == 0) {
        /* Intermediate fragment — wait for more */
        return;
    }

    /* Last fragment: total_len tells us the expected reassembled length */
    if (s_reassembly_len != total_len) {
        ESP_LOGW(TAG, "Reassembly length mismatch: got %u, expected %u",
                 (unsigned)s_reassembly_len, (unsigned)total_len);
        s_reassembly_len = 0;
        return;
    }

    /* COBS-encode the reassembled frame and append 0x00 delimiter */
    size_t cobs_max = total_len + COBS_MAX_OVERHEAD(total_len);
    uint8_t *cobs_buf = malloc(cobs_max + 1);   /* +1 for delimiter */
    if (!cobs_buf) {
        ESP_LOGE(TAG, "malloc failed for COBS buffer");
        s_reassembly_len = 0;
        return;
    }

    size_t cobs_len = cobs_encode(s_reassembly_buf, total_len, cobs_buf);
    cobs_buf[cobs_len] = 0x00;   /* COBS frame delimiter */
    cobs_len++;

    usb_tx_enqueue(cobs_buf, cobs_len);   /* takes ownership */

    /* Reset for next reassembly */
    s_reassembly_len = 0;
}

/* ========================================================================== */
/*  USB → ESP-NOW path                                                         */
/* ========================================================================== */

static void cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    uint8_t rx_buf[64];
    size_t  rx_size = 0;

    esp_err_t err = tinyusb_cdcacm_read(itf, rx_buf, sizeof(rx_buf), &rx_size);
    if (err != ESP_OK || rx_size == 0) {
        return;
    }

    for (size_t i = 0; i < rx_size; i++) {
        uint8_t byte = rx_buf[i];

        if (byte == 0x00) {
            /* End of COBS frame — decode and send via ESP-NOW */
            if (s_usb_rx_accum_len > 0) {
                /* Decoded payload is always <= accumulated length */
                uint8_t decoded[USB_RX_ACCUM_SIZE];
                size_t decoded_len = cobs_decode(s_usb_rx_accum,
                                                  s_usb_rx_accum_len,
                                                  decoded);
                if (decoded_len > 0) {
                    /* Commands are <250 bytes — send as a single ESP-NOW packet */
                    if (decoded_len > ESP_NOW_MAX_DATA_LEN) {
                        ESP_LOGW(TAG, "Command too large (%u bytes) — dropping",
                                 (unsigned)decoded_len);
                    } else {
                        esp_err_t send_err = esp_now_send(BROADCAST_MAC,
                                                          decoded,
                                                          decoded_len);
                        if (send_err != ESP_OK) {
                            ESP_LOGW(TAG, "esp_now_send failed: %s",
                                     esp_err_to_name(send_err));
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "COBS decode failed");
                }
                s_usb_rx_accum_len = 0;
            }
        } else {
            /* Accumulate byte */
            if (s_usb_rx_accum_len < USB_RX_ACCUM_SIZE) {
                s_usb_rx_accum[s_usb_rx_accum_len++] = byte;
            } else {
                ESP_LOGW(TAG, "USB RX accumulator overflow — discarding frame");
                s_usb_rx_accum_len = 0;
            }
        }
    }
}

/* ========================================================================== */
/*  WiFi / ESP-NOW init                                                        */
/* ========================================================================== */

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Fix the channel — must match CONFIG_ESPNOW_CHANNEL on P4 side */
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    /* Disable power-save for lowest latency */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Add broadcast peer so we can send to it */
    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

/* ========================================================================== */
/*  USB CDC init                                                               */
/* ========================================================================== */

static void usb_cdc_init(void)
{
    /* Must be TINYUSB_DEFAULT_CONFIG(), not `{ 0 }`.  esp_tinyusb 2.x validates
     * the driver task config and rejects a zeroed one with
     * "Task size can't be 0" -> ESP_ERR_INVALID_ARG.  The macro fills in
     * .port / .phy / .task (TINYUSB_DEFAULT_TASK_SIZE) / .descriptor. */
    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx      = &cdc_rx_callback,
        .callback_rx_wanted_char     = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
}

/* ========================================================================== */
/*  app_main                                                                   */
/* ========================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-NOW ↔ USB CDC bridge starting");

    /* NVS — required by WiFi driver */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* WiFi + ESP-NOW */
    wifi_init();
    espnow_init();

    /* USB TX queue + task */
    s_usb_tx_queue = xQueueCreate(USB_TX_QUEUE_DEPTH, sizeof(usb_tx_item_t));
    assert(s_usb_tx_queue != NULL);
    xTaskCreate(usb_tx_task, "usb_tx", 4096, NULL, 5, NULL);

    /* USB CDC */
    usb_cdc_init();

    /* Announce our presence so the boat prefers ESP-NOW over WiFi at boot. */
    xTaskCreate(hello_beacon_task, "hello_beacon", 2560, NULL, 4, NULL);

    ESP_LOGI(TAG, "Bridge ready — channel %d, beaconing every %d ms",
             ESPNOW_CHANNEL, HELLO_INTERVAL_MS);

#if USB_SELFTEST
    ESP_LOGW(TAG, "***********************************************************");
    ESP_LOGW(TAG, "*** USB SELF-TEST ACTIVE — emitting synthetic frames    ***");
    ESP_LOGW(TAG, "*** Set USB_SELFTEST to 0 before real telemetry testing ***");
    ESP_LOGW(TAG, "***********************************************************");
    xTaskCreate(usb_selftest_task, "usb_selftest", 4096, NULL, 4, NULL);
#endif
}
