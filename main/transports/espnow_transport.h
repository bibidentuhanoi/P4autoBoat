#pragma once
#include "esp_err.h"
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
 * @brief Fragment and send a JPEG over the field link.
 *
 * Was defined in espnow_transport.c but never declared here, so nothing could
 * call it — the reason field mode had no video despite the code existing.
 * Chunks into <=JPEG_CHUNK_MAX pieces tagged MSG_JPEG_CHUNK.
 */
esp_err_t espnow_transport_send_jpeg(const uint8_t *jpg, size_t len);
