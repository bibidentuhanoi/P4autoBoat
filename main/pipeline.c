#include "pipeline.h"
#include "detect_task.h"
#include "training_log_task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* pipeline_publish_sensors() should be sub-millisecond (protobuf encode of
 * a ~1-2KB struct + a non-blocking memcpy into the WS slot buffer) -- hw
 * has shown it taking 107ms during a TrainingLog capture, well past
 * task_sensor_snapshot's 50ms budget, cause not yet isolated to either
 * waiting for s_msg_mutex (someone else holding it) or the work inside the
 * lock. This threshold is deliberately far below the 50ms deadline so it
 * fires before the deadline miss it's diagnosing, and split into wait vs
 * work so the log states which one it actually was. */
#define PIPELINE_PUBLISH_SLOW_US 10000U

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
static servo_power_handler_fn s_servo_power_handler = NULL;
static steer_raw_command_handler_fn s_steer_raw_handler = NULL;
static calibrate_command_handler_fn s_calibrate_handler = NULL;

/* ---- Shared protobuf envelopes ----
 * boat_BoatMessage is a ~7KB union (SensorSnapshot member holds the 256-entry
 * ToF arrays): too big for small task stacks (esp_timer is 3.5KB, httpd 4KB),
 * and per-call-site statics cost 7KB of internal RAM EACH — that starved
 * xTaskCreate of contiguous internal heap (detect task failed to spawn).
 * So: static envelopes serialized by mutexes, instead of per-call-site
 * stack copies.
 *
 * TWO envelopes, not one. This used to be a single s_msg/s_msg_mutex shared
 * between publish (encode+send) and pipeline_handle_incoming (decode+
 * dispatch). fanout_locked() calls each transport's send_fn *inside* that
 * mutex, and the ESP-NOW transport's send_fn blocks on esp_hosted_send_
 * custom_data() waiting for an RPC response. esp_hosted delivers that
 * response and dispatches inbound custom-RPC events from the SAME single
 * task (rpc_rx_thread) — so an inbound event arriving while a publish's
 * send was in flight would make rpc_rx_thread block taking s_msg_mutex for
 * the decode, stalling the very task needed to deliver the response the
 * publish call was waiting on. Deadlock, broken only by the RPC call's own
 * timeout: observed on the P4 console as "Timeout waiting for Resp"
 * alternating with esp_hosted's "H_SDIO_DRV task still writing Rx data to
 * queue" (its RX pipeline backing up behind the wedged rpc_rx_thread), and
 * on the ESP-NOW ground station as telemetry going completely silent even
 * though the C6 itself was never actually crashing.
 * Fix: RX gets its own envelope/mutex so a stuck TX send can never again
 * block RX decode+dispatch. Costs one extra ~7KB static buffer. */
static boat_BoatMessage  s_msg;
static SemaphoreHandle_t s_msg_mutex = NULL;

static boat_BoatMessage  s_rx_msg;
static SemaphoreHandle_t s_rx_msg_mutex = NULL;

/* Encode s_msg (caller set which_payload/payload under the mutex) and fan out.
 * hw-confirmed (2026-08-11): publish_sensors's "work" phase hit 30-115ms with
 * transports=1 and negligible mutex_wait, meaning the delay is in here --
 * either pb_encode() or the one registered transport's own send(). Split so
 * the next occurrence names which. */
static void fanout_locked(uint8_t *buf, size_t bufsize, const char *what)
{
    uint64_t t_encode_start = (uint64_t)esp_timer_get_time();
    pb_ostream_t stream = pb_ostream_from_buffer(buf, bufsize);
    if (!pb_encode(&stream, boat_BoatMessage_fields, &s_msg)) {
        ESP_LOGE(TAG, "%s encode failed: %s", what, PB_GET_ERROR(&stream));
        return;
    }
    size_t len = stream.bytes_written;
    uint64_t encode_us = (uint64_t)esp_timer_get_time() - t_encode_start;
    if (encode_us > PIPELINE_PUBLISH_SLOW_US) {
        ESP_LOGW(TAG, "%s pb_encode slow: %lluus (len=%u, camera_active=%d)",
                 what, (unsigned long long)encode_us, (unsigned)len,
                 (int)g_training_log_camera_active);
    }

    for (int i = 0; i < s_transport_count; i++) {
        uint64_t t_send_start = (uint64_t)esp_timer_get_time();
        esp_err_t ret = s_transports[i].send(buf, len, s_transports[i].ctx);
        uint64_t send_us = (uint64_t)esp_timer_get_time() - t_send_start;
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Transport %d %s send failed: %s", i, what, esp_err_to_name(ret));
        }
        if (send_us > PIPELINE_PUBLISH_SLOW_US) {
            ESP_LOGW(TAG, "Transport %d %s send slow: %lluus", i, what, (unsigned long long)send_us);
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
    s_servo_power_handler = NULL;
    s_steer_raw_handler = NULL;
    s_calibrate_handler = NULL;
    if (!s_msg_mutex) {
        s_msg_mutex = xSemaphoreCreateMutex();
        if (!s_msg_mutex) {
            ESP_LOGE(TAG, "Failed to create TX envelope mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    if (!s_rx_msg_mutex) {
        s_rx_msg_mutex = xSemaphoreCreateMutex();
        if (!s_rx_msg_mutex) {
            ESP_LOGE(TAG, "Failed to create RX envelope mutex");
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

void pipeline_register_servo_power_handler(servo_power_handler_fn handler)
{
    s_servo_power_handler = handler;
}

void pipeline_register_steer_raw_handler(steer_raw_command_handler_fn handler)
{
    s_steer_raw_handler = handler;
}

void pipeline_register_calibrate_handler(calibrate_command_handler_fn handler)
{
    s_calibrate_handler = handler;
}

void pipeline_publish_sensors(const boat_SensorSnapshot *snap)
{
    if (!s_msg_mutex) return;

    /* Encode buffer — static to avoid stack overflow. Sized from the
     * nanopb-computed worst case so it can never drift below the schema
     * again (a hardcoded 12000 sat under a 13270 max after GpsFix grew the
     * snapshot — worst-case frames would have been silently dropped).
     * Guarded by s_msg_mutex like s_msg. */
    static uint8_t buf[boat_BoatMessage_size + 16];

    uint64_t t0 = (uint64_t)esp_timer_get_time();
    xSemaphoreTake(s_msg_mutex, portMAX_DELAY);
    uint64_t t1 = (uint64_t)esp_timer_get_time();
    /* No memset needed: which_payload selects the only member nanopb reads,
     * and we overwrite that member entirely. */
    s_msg.which_payload = boat_BoatMessage_sensors_tag;
    s_msg.payload.sensors = *snap;
    fanout_locked(buf, sizeof(buf), "sensors");
    xSemaphoreGive(s_msg_mutex);
    uint64_t t2 = (uint64_t)esp_timer_get_time();

    uint64_t wait_us = t1 - t0, work_us = t2 - t1;
    if (wait_us + work_us > PIPELINE_PUBLISH_SLOW_US) {
        ESP_LOGW(TAG, "publish_sensors slow: mutex_wait=%lluus work=%lluus (transports=%d)",
                 (unsigned long long)wait_us, (unsigned long long)work_us, s_transport_count);
    }
}

void pipeline_publish_status(const boat_SystemStatus *status)
{
    if (!s_msg_mutex) return;

    /* Sized from the nanopb-computed worst case, same reasoning as the
     * sensors buffer above: a hardcoded 64 sat under SystemStatus's actual
     * 90-byte worst case once the GPS diagnostic fields grew it, and every
     * status publish was silently failing pb_encode() as a result. */
    uint8_t buf[boat_SystemStatus_size + 16];
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

void pipeline_publish_calibrate_status(const boat_CalibrateStatus *status)
{
    if (!s_msg_mutex) return;

    uint8_t buf[boat_CalibrateStatus_size + 16];
    xSemaphoreTake(s_msg_mutex, portMAX_DELAY);
    s_msg.which_payload = boat_BoatMessage_calibrate_status_tag;
    s_msg.payload.calibrate_status = *status;
    fanout_locked(buf, sizeof(buf), "calibrate_status");
    xSemaphoreGive(s_msg_mutex);
}

void pipeline_handle_incoming(const uint8_t *buf, size_t len)
{
    if (!s_rx_msg_mutex) return;

    xSemaphoreTake(s_rx_msg_mutex, portMAX_DELAY);

    /* Decode needs a zeroed struct (repeated-field counts etc.). */
    memset(&s_rx_msg, 0, sizeof(s_rx_msg));
    pb_istream_t stream = pb_istream_from_buffer(buf, len);

    if (!pb_decode(&stream, boat_BoatMessage_fields, &s_rx_msg)) {
        ESP_LOGW(TAG, "Decode failed: %s", PB_GET_ERROR(&stream));
        xSemaphoreGive(s_rx_msg_mutex);
        return;
    }

    switch (s_rx_msg.which_payload) {
    case boat_BoatMessage_motor_tag:
        ESP_LOGD(TAG, "RX motor: L=%.2f R=%.2f thr=%.2f rud=%.2f",
                 s_rx_msg.payload.motor.left, s_rx_msg.payload.motor.right,
                 s_rx_msg.payload.motor.throttle, s_rx_msg.payload.motor.rudder);   /* DIAG */
        if (s_motor_handler) {
            s_motor_handler(&s_rx_msg.payload.motor);
        } else {
            ESP_LOGW(TAG, "Motor command received but no handler registered");
        }
        break;
    case boat_BoatMessage_detect_tag:
        ESP_LOGI(TAG, "Detect command received");
        detect_trigger();
        break;
    case boat_BoatMessage_training_log_tag:
        ESP_LOGI(TAG, "TrainingLog command received");
        training_log_trigger();
        break;
    case boat_BoatMessage_arm_cmd_tag:
        ESP_LOGI(TAG, "Arm command received: %s%s",
                 s_rx_msg.payload.arm_cmd.arm ? "ARM" : "DISARM",
                 s_rx_msg.payload.arm_cmd.force ? " (force)" : "");
        if (s_arm_handler) {
            s_arm_handler(s_rx_msg.payload.arm_cmd.arm, s_rx_msg.payload.arm_cmd.force);
        }
        break;
    case boat_BoatMessage_winch_tag:
        ESP_LOGD(TAG, "RX winch: speed=%.2f", s_rx_msg.payload.winch.speed);   /* DIAG */
        if (s_winch_handler) {
            s_winch_handler(&s_rx_msg.payload.winch);
        } else {
            ESP_LOGW(TAG, "Winch command received but no handler registered");
        }
        break;
    case boat_BoatMessage_steer_tag:
        ESP_LOGD(TAG, "RX steer: L=%.2f R=%.2f",
                 s_rx_msg.payload.steer.left, s_rx_msg.payload.steer.right);   /* DIAG */
        if (s_steer_handler) {
            s_steer_handler(&s_rx_msg.payload.steer);
        } else {
            ESP_LOGW(TAG, "Steer command received but no handler registered");
        }
        break;
    case boat_BoatMessage_servo_power_tag:
        ESP_LOGI(TAG, "Servo power command: %s", s_rx_msg.payload.servo_power.on ? "ON" : "OFF");
        if (s_servo_power_handler) {
            s_servo_power_handler(s_rx_msg.payload.servo_power.on);
        } else {
            ESP_LOGW(TAG, "Servo power command received but no handler registered");
        }
        break;
    case boat_BoatMessage_steer_raw_tag:
        ESP_LOGI(TAG, "RX steer_raw: pulse_us=%u", (unsigned)s_rx_msg.payload.steer_raw.pulse_us);
        if (s_steer_raw_handler) {
            s_steer_raw_handler(&s_rx_msg.payload.steer_raw);
        } else {
            ESP_LOGW(TAG, "SteerRaw command received but no handler registered");
        }
        break;
    case boat_BoatMessage_calibrate_tag:
        ESP_LOGI(TAG, "Calibrate command: %s%s",
                 s_rx_msg.payload.calibrate.start ? "START" : "STOP",
                 s_rx_msg.payload.calibrate.average_into_existing ? " (average)" : "");
        if (s_calibrate_handler) {
            s_calibrate_handler(s_rx_msg.payload.calibrate.start,
                                s_rx_msg.payload.calibrate.average_into_existing);
        } else {
            ESP_LOGW(TAG, "Calibrate command received but no handler registered");
        }
        break;
    default:
        ESP_LOGW(TAG, "Unhandled message type: %d", (int)s_rx_msg.which_payload);
        break;
    }

    xSemaphoreGive(s_rx_msg_mutex);
}
