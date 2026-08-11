#pragma once
#include "esp_err.h"
#include <stdbool.h>

#define SD_MOUNT_POINT "/sdcard"

/**
 * @brief Power up and mount the microSD card (SDMMC slot 0, 4-line).
 *
 * Soft-optional, like GPS: a missing or faulty card logs a warning and returns
 * an error, it never blocks boot. The boat must still fly without a card.
 *
 * IMPORTANT ordering: call AFTER esp-hosted/WiFi is up. The C6 co-processor
 * uses SDMMC slot 1 and the card uses slot 0 — the same peripheral. Espressif's
 * own host_sdcard_with_hosted example documents an interaction between the two
 * (ESP-IDF issue 16233), so the card is brought up last, and a failure here
 * must never take the radio down with it.
 */
esp_err_t sd_card_init(void);

/** @brief True once the card is mounted and writable. */
bool sd_card_ready(void);

/** @brief Free space in MB, or 0 when not mounted. */
uint64_t sd_card_free_mb(void);
