#include "espnow_transport.h"
#include "espnow_protocol.h"
#include "pipeline.h"
#include "esp_hosted_misc.h"
#include "esp_log.h"
#include "esp_timer.h"
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
#define PEER_MSG_VIDEO    1u   /* P4→C6: sensor telemetry (queued on C6) */
#define PEER_MSG_COMMAND  2u   /* P4→C6: commands (fast path on C6) */
#define PEER_MSG_INIT     3u   /* P4→C6: ESP-NOW init */
#define PEER_MSG_UPSTREAM 4u   /* C6→P4: commands from laptop */
#define PEER_MSG_INIT_STATUS 5u /* C6→P4: one-shot LR rate-config result */
/* Sized from the nanopb worst case, not a hardcoded number -- PEER_DATA_MAX
 * used to be 8166 (leftover from when PEER_MSG_VIDEO carried JPEG frames,
 * before that feature was reverted). boat_BoatMessage_size grew to 13270
 * once PEER_MSG_VIDEO was repurposed for pure SensorSnapshot telemetry (dual
 * 8x8 ToF grids + IMU + GPS + detections), but this ceiling was never
 * revisited -- so any real snapshot big enough to exceed the old 8166 got
 * silently rejected by the size check in espnow_send_fn() below, worse
 * exactly when there was more real data to report (active driving, ToF
 * detections, GPS lock), which is the opposite of when you want telemetry
 * to drop. Same class of bug as the two encode-buffer sizes fixed elsewhere
 * in this file's history -- see pipeline.c's buffer comments. */
#define PEER_DATA_MAX  (boat_BoatMessage_size + ESPNOW_HDR_SIZE)

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
 * send_espnow_init — the {channel, peer_mac} handshake that brings the C6's
 * ESP-NOW stack up. Extracted so both the initial probe() and the
 * self-healing watchdog below can call it identically.
 * -------------------------------------------------------------------------*/
static esp_err_t send_espnow_init(void)
{
    uint8_t init_buf[1 + 6];
    init_buf[0] = (uint8_t)CONFIG_ESPNOW_CHANNEL;
    parse_mac_string(CONFIG_ESPNOW_PEER_MAC, &init_buf[1]);

    esp_err_t ret = esp_hosted_send_custom_data(PEER_MSG_INIT, init_buf, sizeof(init_buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send ESPNOW_INIT (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Sent ESPNOW_INIT (ch=%d peer=" CONFIG_ESPNOW_PEER_MAC ")",
                 CONFIG_ESPNOW_CHANNEL);
    }
    return ret;
}

/* ---------------------------------------------------------------------------
 * Self-healing re-init: if the C6 crashes/reboots under sustained ESP-NOW
 * load, its stack comes back up completely uninitialised -- init_cb only
 * ever runs once, triggered by the MSG_ESPNOW_INIT this side sends during
 * probe(). Nothing on the C6 asks for it again on its own, so a transient
 * C6-side fault otherwise becomes PERMANENT: the boat stops responding and
 * stays that way until the P4 itself is manually reset (which happens to
 * re-run probe() from scratch). Detect prolonged silence from upstream and
 * re-send the same handshake automatically instead.
 *
 * MSG_GROUND_HELLO is the signal to watch, not application data: the S3
 * broadcasts it every ESPNOW_HELLO_INTERVAL_MS regardless of whether anyone
 * is actively driving, so its absence is a clean heartbeat that doesn't
 * depend on traffic volume or operator activity.
 * -------------------------------------------------------------------------*/
#define UPSTREAM_SILENCE_TIMEOUT_US ((int64_t)3 * ESPNOW_HELLO_INTERVAL_MS * 1000LL)
#define REINIT_WATCHDOG_PERIOD_US   (1000LL * 1000LL)

/* A reinit tears the C6's ESP-NOW down and rebuilds it -- disruptive, and it
 * can't complete faster than the next hello cycle proves it worked. If the
 * underlying fault is PERSISTENT rather than a one-time crash, an uncapped
 * watchdog fires roughly every UPSTREAM_SILENCE_TIMEOUT_US forever, which
 * never gives the link more than one timeout window to prove itself before
 * being torn down again -- indistinguishable from a permanent stall, and
 * potentially worse than doing nothing. Bound the damage: try a few times,
 * then go quiet and wait for a human, same fallback as before this existed. */
#define MAX_CONSECUTIVE_REINIT_ATTEMPTS 3

static volatile int64_t   s_last_upstream_us = 0;
static volatile int       s_reinit_attempts  = 0;
static esp_timer_handle_t s_reinit_watchdog  = NULL;
static volatile int8_t    s_lr_status        = -1;  /* -1 = not yet reported */

static void reinit_watchdog_cb(void *arg)
{
    (void)arg;
    int64_t last = s_last_upstream_us;
    if (last == 0) {
        return;   /* never heard anything yet -- probe()'s own timeout owns that case */
    }
    if (esp_timer_get_time() - last > UPSTREAM_SILENCE_TIMEOUT_US) {
        if (s_reinit_attempts >= MAX_CONSECUTIVE_REINIT_ATTEMPTS) {
            return;   /* already gave up this round -- see the ESP_LOGE below */
        }
        s_reinit_attempts++;
        ESP_LOGW(TAG, "No upstream traffic (hello or data) for over %lldms -- "
                      "re-sending ESPNOW_INIT, attempt %d/%d (the C6 may have reset)",
                 (long long)(UPSTREAM_SILENCE_TIMEOUT_US / 1000),
                 s_reinit_attempts, MAX_CONSECUTIVE_REINIT_ATTEMPTS);
        if (s_reinit_attempts >= MAX_CONSECUTIVE_REINIT_ATTEMPTS) {
            ESP_LOGE(TAG, "Giving up after %d attempts -- this fault is persistent, "
                          "not transient. Auto-recovery can't fix that class of problem; "
                          "the link needs a manual reset (and the real cause needs "
                          "investigating, not more retries).",
                     MAX_CONSECUTIVE_REINIT_ATTEMPTS);
        }
        send_espnow_init();
        /* Stamp now so a still-dead C6 gets re-poked once per timeout window,
         * not once per REINIT_WATCHDOG_PERIOD_US tick while silent. */
        s_last_upstream_us = esp_timer_get_time();
    }
}

/* ---------------------------------------------------------------------------
 * upstream_cb — registered for PEER_MSG_UPSTREAM (C6→P4 commands)
 * -------------------------------------------------------------------------*/
static void upstream_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;
    s_last_upstream_us = esp_timer_get_time();
    s_reinit_attempts  = 0;   /* real traffic heard -- link is genuinely alive again */

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
 * init_status_cb — registered for PEER_MSG_INIT_STATUS (C6→P4, one-shot LR
 * rate-config result; see tools/slave_firmware/espnow_bridge.c's init_cb).
 * -------------------------------------------------------------------------*/
static void init_status_cb(uint32_t msg_id, const uint8_t *data, size_t data_len)
{
    (void)msg_id;
    if (!data || data_len < 1) {
        return;
    }
    s_lr_status = (int8_t)data[0];
    ESP_LOGI(TAG, "Co-processor reports LR rate config: %s",
             s_lr_status ? "OK" : "FAILED");
}

/* ---------------------------------------------------------------------------
 * espnow_transport_lr_status — LR rate-config result reported by the C6.
 * -------------------------------------------------------------------------*/
int8_t espnow_transport_lr_status(void)
{
    return s_lr_status;
}

/* ---------------------------------------------------------------------------
 * espnow_send_fn — transport_send_fn registered with the pipeline
 * Wraps nanopb-encoded BoatMessage bytes with espnow_pkt_hdr_t and sends via
 * PEER_MSG_VIDEO. Used identically for pipeline_publish_sensors(),
 * pipeline_publish_status(), and pipeline_publish_motor_status() -- the
 * pipeline's transport_send_fn interface is generic (buf, len, ctx) and
 * carries no "what kind of message is this" hint, so espnow_pkt_hdr_t.msg_type
 * has to be derived from the encoded bytes themselves.
 * -------------------------------------------------------------------------*/
static esp_err_t espnow_send_fn(const uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;

    if (ESPNOW_HDR_SIZE + len > PEER_DATA_MAX) {
        ESP_LOGW(TAG, "espnow_send_fn: payload %u bytes exceeds PEER_DATA_MAX", (unsigned)len);
        return ESP_ERR_INVALID_SIZE;
    }

    /* buf[0] is the wire tag of BoatMessage's oneof member -- reliable
     * because `oneof payload` is the ENTIRE message (no other top-level
     * field ever accompanies it), so nanopb always encodes exactly one
     * length-delimited field, at the very start. tag = (field_num << 3) | 2:
     *   sensors=1 -> 0x0A, status=3 -> 0x1A, motor_status=6 -> 0x32
     * This was hardcoded to MSG_SENSOR unconditionally before, so every
     * MotorStatus/SystemStatus publish over ESP-NOW was mislabeled as sensor
     * telemetry on the wire -- silently dropped by any receiver that (correctly)
     * checked HasField('sensors') before trusting msg_type, which is exactly
     * why this went unnoticed: it fails at the same "nothing happens" level
     * as every other silent fault in this transport. */
    espnow_msg_type_t msg_type = MSG_SENSOR;
    if (len > 0) {
        switch (buf[0]) {
        case 0x1A: msg_type = MSG_STATUS;       break;  /* SystemStatus */
        case 0x32: msg_type = MSG_MOTOR_STATUS; break;  /* MotorStatus */
        default:   msg_type = MSG_SENSOR;       break;  /* SensorSnapshot (0x0A), or unknown -> safe default */
        }
    }

    espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)s_send_buf;
    hdr->msg_type    = msg_type;
    hdr->payload_len = (uint16_t)len;
    hdr->seq         = 0;  /* sensor frames are standalone; seq unused */

    memcpy(s_send_buf + ESPNOW_HDR_SIZE, buf, len);

    /* Timed, not just pass/fail: esp_hosted_send_custom_data() is a
     * synchronous RPC that blocks the caller (the sensor task, for
     * PEER_MSG_VIDEO) until the C6's response arrives or the RPC layer's own
     * timeout fires. A slow-but-eventually-OK call and a full timeout both
     * return -- the sensor task's overrun warning can't tell them apart, it
     * only knows the 50ms budget was blown. Logging only when this actually
     * exceeds 20ms keeps this silent in the normal case (matches the fast
     * command path) while giving an exact number the moment it isn't. */
    int64_t t0 = esp_timer_get_time();
    esp_err_t ret = esp_hosted_send_custom_data(PEER_MSG_VIDEO,
                                                s_send_buf,
                                                ESPNOW_HDR_SIZE + len);
    int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    if (elapsed_ms > 20) {
        ESP_LOGW(TAG, "espnow_send_fn: esp_hosted_send_custom_data took %lldms "
                      "for %u-byte payload (msg_type=%d, ret=%s)",
                 (long long)elapsed_ms, (unsigned)(ESPNOW_HDR_SIZE + len),
                 (int)msg_type, esp_err_to_name(ret));
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "espnow_send_fn: send failed (%s)", esp_err_to_name(ret));
    }
    return ret;
}

/* ---------------------------------------------------------------------------
 * espnow_transport_send_telemetry — field-mode-only compact IMU+GPS send.
 * Bypasses pipeline.c/boat.proto entirely; see espnow_telemetry_t for why.
 * Called directly from sensor_task.c, not registered as a pipeline
 * transport_send_fn -- this is not a boat_BoatMessage, there is nothing for
 * fanout_locked()'s pb_encode() to do with it.
 * -------------------------------------------------------------------------*/
esp_err_t espnow_transport_send_telemetry(const espnow_telemetry_t *t)
{
    uint8_t buf[ESPNOW_HDR_SIZE + sizeof(espnow_telemetry_t)];
    espnow_pkt_hdr_t *hdr = (espnow_pkt_hdr_t *)buf;
    hdr->msg_type    = MSG_FIELD_TELEMETRY;
    hdr->payload_len = (uint16_t)sizeof(espnow_telemetry_t);
    hdr->seq         = 0;
    memcpy(buf + ESPNOW_HDR_SIZE, t, sizeof(espnow_telemetry_t));

    /* Same timing instrumentation as espnow_send_fn, for the same reason:
     * this path is EXPECTED to always be fast now (one packet, no
     * fragmentation) -- if it ever isn't, that's worth knowing exactly,
     * not guessing at. */
    int64_t t0 = esp_timer_get_time();
    esp_err_t ret = esp_hosted_send_custom_data(PEER_MSG_VIDEO, buf, sizeof(buf));
    int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    if (elapsed_ms > 20) {
        ESP_LOGW(TAG, "espnow_transport_send_telemetry: esp_hosted_send_custom_data "
                      "took %lldms for %u-byte payload (ret=%s)",
                 (long long)elapsed_ms, (unsigned)sizeof(buf), esp_err_to_name(ret));
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "espnow_transport_send_telemetry: send failed (%s)", esp_err_to_name(ret));
    }
    return ret;
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

    /* --- Register the one-shot LR-status callback --- */
    ret = esp_hosted_register_custom_callback(PEER_MSG_INIT_STATUS, init_status_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register init-status callback (%s)", esp_err_to_name(ret));
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
    ret = send_espnow_init();
    if (ret != ESP_OK) {
        return ret;
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

    /* --- Verify Long Range mode took, if the C6 build has it enabled --------
     * esp_wifi_set_protocol()/esp_now_set_peer_rate_config() run ON THE C6
     * (tools/slave_firmware/espnow_bridge.c's init_cb), not here -- the P4
     * can't set them, only verify. esp_wifi_get_protocol() IS proxied
     * through esp_wifi_remote like the channel check above, so it genuinely
     * reflects the co-processor's state. Log-only: a mismatch doesn't fail
     * the probe (the link still works at normal range), it just means the
     * range test won't show a gain -- confirm that here, at boot, rather
     * than discovering it 300m into a field test. */
    uint8_t protocol_bitmap = 0;
    if (esp_wifi_get_protocol(WIFI_IF_STA, &protocol_bitmap) == ESP_OK) {
        if (protocol_bitmap & WIFI_PROTOCOL_LR) {
            ESP_LOGI(TAG, "Co-processor LR protocol bit is SET (bitmap=0x%02x)",
                     protocol_bitmap);
        } else {
            ESP_LOGW(TAG, "Co-processor LR protocol bit NOT set (bitmap=0x%02x) "
                          "-- ESPNOW_LR_ENABLED may be 0 on the C6 build, or "
                          "esp_wifi_set_protocol failed there (check its own log)",
                     protocol_bitmap);
        }
    } else {
        ESP_LOGW(TAG, "Could not read back the co-processor's WiFi protocol bitmap");
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

    /* probe() already heard at least one MSG_GROUND_HELLO (that's what makes
     * activate() get called at all), so s_last_upstream_us is already fresh
     * via upstream_cb -- just start the periodic check from here on. */
    const esp_timer_create_args_t reinit_args = {
        .callback = reinit_watchdog_cb,
        .name     = "espnow_reinit",
    };
    ret = esp_timer_create(&reinit_args, &s_reinit_watchdog);
    if (ret == ESP_OK) {
        ret = esp_timer_start_periodic(s_reinit_watchdog, REINIT_WATCHDOG_PERIOD_US);
    }
    if (ret != ESP_OK) {
        /* Non-fatal: the link still works, it just won't self-heal from a
         * C6 crash without a manual P4 reset -- same as before this existed. */
        ESP_LOGW(TAG, "Could not start ESP-NOW self-heal watchdog (%s)", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "ESP-NOW transport ready (ch=%d peer=" CONFIG_ESPNOW_PEER_MAC ")",
             CONFIG_ESPNOW_CHANNEL);
    return ESP_OK;
}
