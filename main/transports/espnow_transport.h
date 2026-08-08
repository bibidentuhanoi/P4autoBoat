#pragma once
#include "esp_err.h"
#include "espnow_protocol.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief Bring ESP-NOW up on the co-processor and listen for a ground station.
 *
 * Performs the C6 handshake (channel + peer + callbacks) and then waits up to
 * @p timeout_ms for a MSG_GROUND_HELLO beacon from the S3 USB bridge.
 *
 * WiFi must already be STARTED (not necessarily connected) — the co-processor
 * requires esp_wifi_start() before esp_now_init().
 *
 * Does NOT register the pipeline transport; call espnow_transport_activate()
 * for that, so a failed probe leaves no dead transport attached.
 *
 * @return ESP_OK           ground station heard — use field mode
 *         ESP_ERR_NOT_FOUND no beacon within the window — fall back to WiFi
 *         other            the co-processor handshake itself failed
 */
esp_err_t espnow_transport_probe(uint32_t timeout_ms);

/**
 * @brief Register ESP-NOW as a pipeline transport. Call only after a
 *        successful espnow_transport_probe().
 */
esp_err_t espnow_transport_activate(void);

/**
 * @brief Send field-mode telemetry (IMU + GPS only) as a compact,
 *        non-protobuf struct — bypasses the pipeline/boat.proto path
 *        entirely. Called directly by sensor_task.c when g_field_mode is
 *        true, instead of pipeline_publish_sensors(). See espnow_telemetry_t
 *        in espnow_protocol.h for why.
 */
esp_err_t espnow_transport_send_telemetry(const espnow_telemetry_t *t);

/**
 * @brief LR rate-config result reported by the C6 (see init_cb in
 *        tools/slave_firmware/espnow_bridge.c): 1 = esp_now_set_peer_rate_config
 *        succeeded, 0 = it failed, -1 = not yet reported (LR disabled on the
 *        C6 build, or the report hasn't arrived yet).
 */
int8_t espnow_transport_lr_status(void);
