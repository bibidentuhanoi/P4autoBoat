/*
 * sd_card.c — microSD (TF) card on SDMMC slot 0.
 *
 * Board wiring (ESP32-P4 Function EV Board / this build):
 *     D0 GPIO39   D1 GPIO40   D2 GPIO41   D3 GPIO42
 *     CLK GPIO43  CMD GPIO44  EN  GPIO45
 *
 * Slot 0 pins are fixed by the IO MUX, so they are not configured individually
 * — only the width, and the card power.
 *
 * Two separate power concerns, both handled here:
 *   1. GPIO45 EN — board-level MOSFET on the card's 3.3V supply.
 *   2. On-chip LDO channel 4 — VDD_SDIO rail. Slot 0 is the UHS-I capable
 *      slot and needs controllable IO voltage; without this the card either
 *      fails to enumerate or is stuck at low speed.
 */
#include "sd_card.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <sys/stat.h>

static const char *TAG = "SD";

#define SD_EN_GPIO        GPIO_NUM_45
#define SD_LDO_CHAN_ID    4          /* VDD_SDIO on-chip LDO channel */
#define SD_MAX_FILES      8

static sdmmc_card_t          *s_card = NULL;
static sd_pwr_ctrl_handle_t   s_pwr  = NULL;
static bool                   s_ready = false;

esp_err_t sd_card_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    /* 1. Board power-enable. Give the card a moment to come up before probing;
     *    probing a rail that is still rising looks like a missing card. */
    gpio_config_t en = {
        .pin_bit_mask = 1ULL << SD_EN_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&en), TAG, "EN gpio_config");
    gpio_set_level(SD_EN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 2. VDD_SDIO via the on-chip LDO — required for slot 0. */
    sd_pwr_ctrl_ldo_config_t ldo_cfg = { .ldo_chan_id = SD_LDO_CHAN_ID };
    esp_err_t err = sd_pwr_ctrl_new_on_chip_ldo(&ldo_cfg, &s_pwr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "on-chip LDO init failed: %s", esp_err_to_name(err));
        gpio_set_level(SD_EN_GPIO, 0);
        return err;
    }

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot             = SDMMC_HOST_SLOT_0;   /* slot 1 belongs to the C6 */
    host.max_freq_khz     = SDMMC_FREQ_HIGHSPEED;
    host.pwr_ctrl_handle  = s_pwr;

    /* Slot 0 is IO MUX: pins are fixed, only width is ours to choose. */
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    slot.cd    = SDMMC_SLOT_NO_CD;
    slot.wp    = SDMMC_SLOT_NO_WP;

    const esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        /* Never reformat: a mount failure is far more likely to be a loose card
         * or a bad rail than a bad filesystem, and silently erasing a dataset
         * the user has already collected would be unforgivable. */
        .format_if_mount_failed = false,
        .max_files              = SD_MAX_FILES,
        .allocation_unit_size   = 64 * 1024,   /* large clusters: fewer, bigger
                                                * writes for image logging */
    };

    err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);
    if (err != ESP_OK) {
        if (err == ESP_FAIL) {
            ESP_LOGW(TAG, "mount failed — card not formatted FAT? (not reformatting)");
        } else {
            ESP_LOGW(TAG, "card init failed: %s (inserted? EN=GPIO%d wired?)",
                     esp_err_to_name(err), SD_EN_GPIO);
        }
        sd_pwr_ctrl_del_on_chip_ldo(s_pwr);
        s_pwr = NULL;
        gpio_set_level(SD_EN_GPIO, 0);
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG, "mounted at %s", SD_MOUNT_POINT);
    sdmmc_card_print_info(stdout, s_card);
    ESP_LOGI(TAG, "free space: %llu MB", sd_card_free_mb());
    return ESP_OK;
}

bool sd_card_ready(void)
{
    return s_ready;
}

uint64_t sd_card_free_mb(void)
{
    if (!s_ready) {
        return 0;
    }
    /* ESP-IDF newlib has no statvfs; FATFS exposes its own space query. */
    uint64_t total = 0, freeb = 0;
    if (esp_vfs_fat_info(SD_MOUNT_POINT, &total, &freeb) != ESP_OK) {
        return 0;
    }
    return freeb / (1024ULL * 1024ULL);
}
