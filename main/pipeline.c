#include "pipeline.h"
#include "detect_task.h"
#include "esp_log.h"
#include <string.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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
static arm_command_handler_fn s_arm_handler = NULL;
static winch_command_handler_fn s_winch_handler = NULL;
static steer_command_handler_fn s_steer_handler = NULL;

/* ---- Shared protobuf envelope ----
 * boat_BoatMessage is a ~7KB union (SensorSnapshot member holds the 256-entry
 * ToF arrays): too big for small task stacks (esp_timer is 3.5KB, httpd 4KB),
 * and per-call-site statics cost 7KB of internal RAM EACH — that starved
 * xTaskCreate of contiguous internal heap (detect task failed to spawn).
 * So: ONE shared static envelope, serialized by a mutex. All uses are short
 * (encode/decode + dispatch), and publishers run at <= 20 Hz. */
static boat_BoatMessage  s_msg;
static SemaphoreHandle_t s_msg_mutex = NULL;

/* Encode s_msg (caller set which_payload/payload under the mutex) and fan out. */
static void fanout_locked(uint8_t *buf, size_t bufsize, const char *what)
{
    pb_ostream_t stream = pb_ostream_from_buffer(buf, bufsize);
    if (!pb_encode(&stream, boat_BoatMessage_fields, &s_msg)) {
        ESP_LOGE(TAG, "%s encode failed: %s", what, PB_GET_ERROR(&stream));
        return;
    }
    size_t len = stream.bytes_written;
    for (int i = 0; i < s_transport_count; i++) {
        esp_err_t ret = s_transports[i].send(buf, len, s_transports[i].ctx);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Transport %d %s send failed: %s", i, what, esp_err_to_name(ret));
        }
    }
}

/* ---- Public API ---- */

esp_err_t pipeline_init(void)
{
    s_transport_count = 0;
    s_motor_handler = NULL;
    s_arm_handler = NULL;
    s_winch_handler = NULL;
    s_steer_handler = NULL;
    if (!s_msg_mutex) {
        s_msg_mutex = xSemaphoreCreateMutex();
        if (!s_msg_mutex) {
            ESP_LOGE(TAG, "Failed to create envelope mutex");
            return ESP_ERR_NO_MEM;
        }
    }
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

void pipeline_register_arm_handler(arm_command_handler_fn handler)
{
    s_arm_handler = handler;
}

void pipeline_register_winch_handler(winch_command_handler_fn handler)
{
    s_winch_handler = handler;
}

void pipeline_register_steer_handler(steer_command_handler_fn handler)
{
    s_steer_handler = handler;
}

void pipeline_publish_sensors(const boat_SensorSnapshot *snap)
{
    if (!s_msg_mutex) return;

    /* Encode buffer — static to avoid stack overflow.
     * boat_BoatMessage_size (nanopb-computed max) = 11053 bytes (4 targets/zone × 256 entries).
     * 12000 bytes gives a small safety margin. Guarded by s_msg_mutex like s_msg. */
    static uint8_t buf[12000];

    xSemaphoreTake(s_msg_mutex, portMAX_DELAY);
    /* No memset needed: which_payload selects the only member nanopb reads,
     * and we overwrite that member entirely. */
    s_msg.which_payload = boat_BoatMessage_sensors_tag;
    s_msg.payload.sensors = *snap;
    fanout_locked(buf, sizeof(buf), "sensors");
    xSemaphoreGive(s_msg_mutex);
}

void pipeline_publish_status(const boat_SystemStatus *status)
{
    if (!s_msg_mutex) return;

    uint8_t buf[64];
    xSemaphoreTake(s_msg_mutex, portMAX_DELAY);
    s_msg.which_payload = boat_BoatMessage_status_tag;
    s_msg.payload.status = *status;
    fanout_locked(buf, sizeof(buf), "status");
    xSemaphoreGive(s_msg_mutex);
}

void pipeline_publish_motor_status(const boat_MotorStatus *mstatus)
{
    if (!s_msg_mutex) return;

    uint8_t buf[32];
    xSemaphoreTake(s_msg_mutex, portMAX_DELAY);
    s_msg.which_payload = boat_BoatMessage_motor_status_tag;
    s_msg.payload.motor_status = *mstatus;
    fanout_locked(buf, sizeof(buf), "motor_status");
    xSemaphoreGive(s_msg_mutex);
}

void pipeline_handle_incoming(const uint8_t *buf, size_t len)
{
    if (!s_msg_mutex) return;

    xSemaphoreTake(s_msg_mutex, portMAX_DELAY);

    /* Decode needs a zeroed struct (repeated-field counts etc.). */
    memset(&s_msg, 0, sizeof(s_msg));
    pb_istream_t stream = pb_istream_from_buffer(buf, len);

    if (!pb_decode(&stream, boat_BoatMessage_fields, &s_msg)) {
        ESP_LOGW(TAG, "Decode failed: %s", PB_GET_ERROR(&stream));
        xSemaphoreGive(s_msg_mutex);
        return;
    }

    switch (s_msg.which_payload) {
    case boat_BoatMessage_motor_tag:
        if (s_motor_handler) {
            s_motor_handler(&s_msg.payload.motor);
        } else {
            ESP_LOGW(TAG, "Motor command received but no handler registered");
        }
        break;
    case boat_BoatMessage_detect_tag:
        ESP_LOGI(TAG, "Detect command received");
        detect_trigger();
        break;
    case boat_BoatMessage_arm_cmd_tag:
        ESP_LOGI(TAG, "Arm command received: %s%s",
                 s_msg.payload.arm_cmd.arm ? "ARM" : "DISARM",
                 s_msg.payload.arm_cmd.force ? " (force)" : "");
        if (s_arm_handler) {
            s_arm_handler(s_msg.payload.arm_cmd.arm, s_msg.payload.arm_cmd.force);
        }
        break;
    case boat_BoatMessage_winch_tag:
        if (s_winch_handler) {
            s_winch_handler(&s_msg.payload.winch);
        } else {
            ESP_LOGW(TAG, "Winch command received but no handler registered");
        }
        break;
    case boat_BoatMessage_steer_tag:
        if (s_steer_handler) {
            s_steer_handler(&s_msg.payload.steer);
        } else {
            ESP_LOGW(TAG, "Steer command received but no handler registered");
        }
        break;
    default:
        ESP_LOGW(TAG, "Unhandled message type: %d", (int)s_msg.which_payload);
        break;
    }

    xSemaphoreGive(s_msg_mutex);
}
