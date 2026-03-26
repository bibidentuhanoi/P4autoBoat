#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#define PIPELINE_MAX_TRANSPORTS 4

/**
 * @brief Transport send function pointer.
 *        Called by pipeline to push encoded protobuf bytes to a transport.
 *        Implementations must be thread-safe (called from sensor task context).
 */
typedef esp_err_t (*transport_send_fn)(const uint8_t *buf, size_t len, void *ctx);
