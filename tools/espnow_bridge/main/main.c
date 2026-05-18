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

#include "tinyusb.h"
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

/* USB → ESP-NOW accumulator */
#define USB_RX_ACCUM_SIZE   512

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

static void usb_tx_task(void *arg)
{
    usb_tx_item_t item;
    while (1) {
        if (xQueueReceive(s_usb_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, item.data, item.len);
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(100));
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
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,   /* use default */
        .string_descriptor = NULL,
        .external_phy      = false,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev  = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx      = &cdc_rx_callback,
        .callback_rx_wanted_char     = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
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

    ESP_LOGI(TAG, "Bridge ready — channel %d", ESPNOW_CHANNEL);
}
