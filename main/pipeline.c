#include "pipeline.h"
#include "esp_log.h"
#include <string.h>
#include <pb_encode.h>
#include <pb_decode.h>

static const char *TAG = "PIPELINE";

/* ---- Transport registry ---- */

typedef struct {
    transport_send_fn send;
    void *ctx;
} transport_entry_t;

static transport_entry_t s_transports[PIPELINE_MAX_TRANSPORTS];
static int s_transport_count = 0;

/* ---- Command handler ---- */

static motor_command_handler_fn s_motor_handler = NULL;

/* ---- Public API ---- */

esp_err_t pipeline_init(void)
{
    s_transport_count = 0;
    s_motor_handler = NULL;
    ESP_LOGI(TAG, "Pipeline initialized (max %d transports)", PIPELINE_MAX_TRANSPORTS);
    return ESP_OK;
}

esp_err_t pipeline_register_transport(transport_send_fn send, void *ctx)
{
    if (!send) {
        ESP_LOGE(TAG, "Transport send function cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (s_transport_count >= PIPELINE_MAX_TRANSPORTS) {
        ESP_LOGE(TAG, "Max transports reached (%d)", PIPELINE_MAX_TRANSPORTS);
        return ESP_ERR_NO_MEM;
    }
    s_transports[s_transport_count].send = send;
    s_transports[s_transport_count].ctx = ctx;
    s_transport_count++;
    ESP_LOGI(TAG, "Transport registered (%d/%d)", s_transport_count, PIPELINE_MAX_TRANSPORTS);
    return ESP_OK;
}

void pipeline_register_motor_handler(motor_command_handler_fn handler)
{
    s_motor_handler = handler;
}

void pipeline_publish_sensors(const boat_SensorSnapshot *snap)
{
    /* Wrap in BoatMessage envelope */
    boat_BoatMessage msg = boat_BoatMessage_init_zero;
    msg.which_payload = boat_BoatMessage_sensors_tag;
    msg.payload.sensors = *snap;

    /* Encode to stack buffer.
     * boat_BoatMessage_size (nanopb-computed max) = 1449 bytes.
     * 1500 bytes gives a small safety margin.
     * Sensor task stack must be large enough to accommodate this (>=8KB recommended). */
    uint8_t buf[1500];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));

    if (!pb_encode(&stream, boat_BoatMessage_fields, &msg)) {
        ESP_LOGE(TAG, "Encode failed: %s", PB_GET_ERROR(&stream));
        return;
    }

    size_t len = stream.bytes_written;

    /* Fan out to all registered transports */
    for (int i = 0; i < s_transport_count; i++) {
        esp_err_t ret = s_transports[i].send(buf, len, s_transports[i].ctx);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Transport %d send failed: %s", i, esp_err_to_name(ret));
        }
    }
}

void pipeline_publish_status(const boat_SystemStatus *status)
{
    boat_BoatMessage msg = boat_BoatMessage_init_zero;
    msg.which_payload = boat_BoatMessage_status_tag;
    msg.payload.status = *status;

    uint8_t buf[64];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));

    if (!pb_encode(&stream, boat_BoatMessage_fields, &msg)) {
        ESP_LOGE(TAG, "Status encode failed: %s", PB_GET_ERROR(&stream));
        return;
    }

    size_t len = stream.bytes_written;

    for (int i = 0; i < s_transport_count; i++) {
        esp_err_t ret = s_transports[i].send(buf, len, s_transports[i].ctx);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Transport %d status send failed: %s", i, esp_err_to_name(ret));
        }
    }
}

void pipeline_handle_incoming(const uint8_t *buf, size_t len)
{
    boat_BoatMessage msg = boat_BoatMessage_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(buf, len);

    if (!pb_decode(&stream, boat_BoatMessage_fields, &msg)) {
        ESP_LOGW(TAG, "Decode failed: %s", PB_GET_ERROR(&stream));
        return;
    }

    switch (msg.which_payload) {
    case boat_BoatMessage_motor_tag:
        if (s_motor_handler) {
            s_motor_handler(&msg.payload.motor);
        } else {
            ESP_LOGW(TAG, "Motor command received but no handler registered");
        }
        break;
    default:
        ESP_LOGW(TAG, "Unhandled message type: %d", (int)msg.which_payload);
        break;
    }
}
