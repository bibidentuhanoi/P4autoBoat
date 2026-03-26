#pragma once
#include "esp_err.h"
#include "transport.h"
#include "proto/boat.pb.h"

/**
 * @brief Initialize the pipeline. Call once from app_main before registering transports.
 */
esp_err_t pipeline_init(void);

/**
 * @brief Register a transport adapter for outgoing data.
 * @param send  Function pointer called for each published message.
 * @param ctx   Opaque context passed to send().
 */
esp_err_t pipeline_register_transport(transport_send_fn send, void *ctx);

/**
 * @brief Encode a SensorSnapshot to protobuf and fan out to all registered transports.
 *        Called from sensor task context (5Hz).
 */
void pipeline_publish_sensors(const boat_SensorSnapshot *snap);

/**
 * @brief Decode incoming raw protobuf bytes and dispatch to registered handlers.
 *        Called from transport receive context (e.g., WS httpd task).
 */
void pipeline_handle_incoming(const uint8_t *buf, size_t len);

/**
 * @brief Motor command handler callback type.
 */
typedef void (*motor_command_handler_fn)(const boat_MotorCommand *cmd);

/**
 * @brief Register a handler for incoming MotorCommand messages.
 */
void pipeline_register_motor_handler(motor_command_handler_fn handler);
