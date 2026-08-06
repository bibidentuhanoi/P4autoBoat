#pragma once
#include "esp_err.h"

/**
 * @brief Start periodic JPEG capture -> ESP-NOW in field mode.
 *
 * Call AFTER espnow_transport_activate() and only when g_field_mode is set.
 * A no-op (returns ESP_OK) when CONFIG_ESPNOW_JPEG_INTERVAL_MS is 0.
 *
 * BANDWIDTH WARNING — read before raising the rate.
 * A JPEG at the sensor's native 800x640 is roughly 15-25 KB, i.e. ~15-20x a
 * trimmed ToF telemetry snapshot (~1.1 KB), and it shares the co-processor's
 * TX queue with telemetry. Measured on hardware, ToF snapshots were ALREADY
 * being dropped upstream of the ground station before any video existed, so
 * video is deliberately rate-limited rather than streamed: it is a periodic
 * still, not a feed. Raising the rate will cost sensor frames.
 */
esp_err_t espnow_video_start(void);
