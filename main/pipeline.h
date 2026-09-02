#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "transport.h"
#include "proto/boat.pb.h"

/**
 * @brief Initialize the pipeline. Call once from app_main before registering transports.
 */
esp_err_t pipeline_init(void);

/**
 * @brief Register a transport adapter for outgoing data.
 * @param send  Function pointer called for each published message. Must not be NULL.
 * @param ctx   Opaque context passed to send().
 * @note  MUST be called before starting any FreeRTOS tasks that call
 *        pipeline_publish_sensors() or pipeline_handle_incoming().
 *        Registration is not thread-safe and must complete during init.
 */
esp_err_t pipeline_register_transport(transport_send_fn send, void *ctx);

/**
 * @brief Encode a SensorSnapshot to protobuf and fan out to all registered transports.
 *        Called from sensor task context (20Hz).
 */
void pipeline_publish_sensors(const boat_SensorSnapshot *snap);

/**
 * @brief Encode a SystemStatus to protobuf and fan out to all registered transports.
 *        Called from sensor task context (~1Hz).
 */
void pipeline_publish_status(const boat_SystemStatus *status);

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
 * @note  MUST be called before starting any FreeRTOS tasks.
 */
void pipeline_register_motor_handler(motor_command_handler_fn handler);

void pipeline_publish_motor_status(const boat_MotorStatus *mstatus);

typedef void (*arm_command_handler_fn)(bool arm, bool force);

void pipeline_register_arm_handler(arm_command_handler_fn handler);

/**
 * @brief Winch command handler callback type.
 */
typedef void (*winch_command_handler_fn)(const boat_WinchCommand *cmd);

/**
 * @brief Register a handler for incoming WinchCommand messages.
 * @note  MUST be called before starting any FreeRTOS tasks.
 */
void pipeline_register_winch_handler(winch_command_handler_fn handler);

/**
 * @brief Steering command handler callback type.
 */
typedef void (*steer_command_handler_fn)(const boat_SteerCommand *cmd);

/**
 * @brief Register a handler for incoming SteerCommand messages.
 * @note  MUST be called before starting any FreeRTOS tasks.
 */
void pipeline_register_steer_handler(steer_command_handler_fn handler);

/**
 * @brief Servo rail power handler callback type.
 */
typedef void (*servo_power_handler_fn)(bool on);

/**
 * @brief Register a handler for incoming ServoPowerCommand messages.
 * @note  MUST be called before starting any FreeRTOS tasks.
 */
void pipeline_register_servo_power_handler(servo_power_handler_fn handler);

/**
 * @brief Raw steer calibration command handler callback type.
 */
typedef void (*steer_raw_command_handler_fn)(const boat_SteerRawCommand *cmd);

/**
 * @brief Register a handler for incoming SteerRawCommand messages.
 * @note  MUST be called before starting any FreeRTOS tasks.
 */
void pipeline_register_steer_raw_handler(steer_raw_command_handler_fn handler);

/**
 * @brief ESC trim auto-calibration command handler callback type.
 *        start=false is an explicit stop/abort (E-stop path).
 */
typedef void (*calibrate_command_handler_fn)(bool start, bool average_into_existing);

/**
 * @brief Register a handler for incoming CalibrateCommand messages.
 * @note  MUST be called before starting any FreeRTOS tasks.
 */
void pipeline_register_calibrate_handler(calibrate_command_handler_fn handler);

/* Bench throttle-mismatch test: the boat runs the profile and records to its
 * own SD card, so the link is only used to press the button. */
/* reset_c > 0 asks the trim learner to restart from that c, ONCE, and only if
 * the run is actually accepted. 0 leaves the learner untouched, which is what
 * every ordinary run sends. */
typedef void (*bench_command_handler_fn)(uint32_t kind, float base, float delta,
                                         float reset_c);
void pipeline_register_bench_handler(bench_command_handler_fn handler);

/* Runtime P-assist switch. Runtime so both arms of an A/B run the same
 * firmware -- a rebuild between arms could pass a build difference off as a
 * result. */
typedef void (*assist_command_handler_fn)(bool p_on, bool rudder_assist,
                                          uint32_t request_id);
/* Yaw-RATE demand for the assisted loop, deg/s. Its own handler because it is
 * its own message: it must never reach the raw rudder path. */
typedef void (*steer_rate_command_handler_fn)(float target_dps);
void pipeline_register_steer_rate_handler(steer_rate_command_handler_fn handler);
void pipeline_register_assist_handler(assist_command_handler_fn handler);
void pipeline_publish_bench_status(const boat_BenchStatus *status);

/**
 * @brief Encode a CalibrateStatus to protobuf and fan out to all transports.
 *        Called from the control task while a calibration sweep runs.
 */
void pipeline_publish_calibrate_status(const boat_CalibrateStatus *status);
