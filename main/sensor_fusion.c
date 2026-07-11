#include "sensor_fusion.h"
#include "detect_task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>

static const char *FUSION_TAG = "FUSION";

// Configuration macros will be provided by Kconfig in the future,
// for now we use the ones defined in sdkconfig or default if missing
#include "sdkconfig.h"

// Filter Constants
#define MAG_LPF_ALPHA       0.2f
#define ACCEL_EPSILON       0.0001f // Prevents Div-by-Zero at 0G

// Globals
static CalibrationData* calib;
static SemaphoreHandle_t fusion_mutex;
static FusionResult current_fusion;

// Helper
static float mag_filt[3] = {0};
static float pitch = 0.0f;
static float roll = 0.0f;
static float alpha;

void fusion_init(CalibrationData* calib_data) {
    calib = calib_data;
    fusion_mutex = xSemaphoreCreateMutex();

    // Parse alpha once — was strtof() every 20ms tick (Bug 4)
    alpha = strtof(CONFIG_FUSION_COMPLEMENTARY_ALPHA, NULL);
    if (alpha <= 0.0f || alpha >= 1.0f) {
        alpha = 0.96f;
    }

    // Initialize state with tares
    pitch = calib->pitch_tare;
    roll = calib->roll_tare;
}

void fusion_get_result(FusionResult* res) {
    if (xSemaphoreTake(fusion_mutex, portMAX_DELAY) == pdTRUE) {
        *res = current_fusion;
        xSemaphoreGive(fusion_mutex);
    }
}

void task_imu_fusion(void *pvParameters) {
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    int16_t raw_mx, raw_my, raw_mz;

    int64_t last_time = esp_timer_get_time();

    while(1) {
        while (g_inference_active) vTaskDelay(pdMS_TO_TICKS(10));
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;

        // --- I2C reads under shared bus mutex ---
        // Failure visibility: a dead IMU used to freeze the outputs at zero
        // with no trace in the logs. Count failures and report ~every 2s.
        static uint32_t ag_fails = 0, mag_fails = 0;
        static bool bus_scanned = false;   /* one-shot ground-truth scan on first sustained fault */
        bool mag_ok = false;
        esp_err_t mag_err = ESP_FAIL;   /* real I2C error, surfaced in the failure log */
        esp_err_t ag_err  = ESP_FAIL;

        bool read_success = false;
        /* The ToF task holds this SAME bus mutex for its slow 8x8 grid read.
         * A 50ms give-up let the fusion abandon the bus mid-contention and then
         * (below) mislabel it as a read failure — the ESP_FAIL on BOTH chips at
         * once was this timeout default, never a real chip fault. Wait well past
         * a ToF read; if we still can't get the bus, SKIP the tick (hold last
         * values) — bus contention is not a sensor problem, don't false-alarm. */
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        // MAG READ
        bool    mag_diag_now = false;   /* emit a raw/cal/ctrl diagnostic line this tick */
        uint8_t mag_ctrl     = 0xFF;    /* QMC control reg 0x09 read-back (0x05 = continuous) */
        static float diag_heading = 0.0f;  /* last heading stored — mirrors the UI value for the diag line */
        mag_err = imu_read_mag(&raw_mx, &raw_my, &raw_mz);
        mag_ok  = (mag_err == ESP_OK);
        if (mag_ok) {
            /* Standby detection: a QMC that ACKs but stopped measuring returns
             * byte-identical data forever — heading freezes with no error. Real
             * readings always flicker >=1 LSB, so ~1s of identical samples means
             * it dropped to standby (boot config lost on a marginal bus). Re-arm
             * continuous; escalate to a full soft-reset if the gentle re-arm does
             * not take. The bus mutex is already held here. */
            static int16_t l_mx = 0, l_my = 0, l_mz = 0;
            static uint32_t mag_static = 0, mag_rearm = 0;
            static bool mag_frozen = false;
            if (raw_mx == l_mx && raw_my == l_my && raw_mz == l_mz) {
                if (++mag_static >= 50) {
                    mag_static = 0;
                    /* Frozen ≥1s while still ACKing = standby: the heading is NOT
                     * trustworthy even though reads "succeed". Drop the health flag
                     * so the dashboard's COMPASS / QMC5883L indicator lights up —
                     * the stall shows on the UI, not only in the serial log.
                     * Restored below the instant raw data moves again. */
                    if (!mag_frozen) { imu_set_mag_ok(false); mag_frozen = true; }
                    if (++mag_rearm >= 3) {
                        imu_reinit_mag();   /* escalate: soft reset + verified reconfig */
                        mag_rearm = 0;
                        ESP_LOGW(FUSION_TAG, "mag still frozen after re-arm — full reset (check wiring if this repeats)");
                    } else {
                        imu_mag_ensure_continuous();
                        ESP_LOGW(FUSION_TAG, "mag data frozen (standby) — re-asserted continuous mode");
                    }
                }
            } else {
                /* Raw moved ⇒ genuinely measuring (a live mag always dithers ≥1 LSB
                 * from noise, even on a still boat), so the heading is trustworthy. */
                if (mag_frozen) { imu_set_mag_ok(true); mag_frozen = false; }
                mag_static = 0;
                mag_rearm  = 0;
                l_mx = raw_mx; l_my = raw_my; l_mz = raw_mz;
            }

            /* Periodic health readout (~every 5s): re-assert continuous as a
             * belt-and-suspenders and grab the control register. The post-mutex
             * log then prints raw mag + ctrl so the bench test is unambiguous —
             * rotate the boat: raw moving = sensor + wiring good; raw stuck with
             * ctrl09=0x05 = chip stopped measuring; ctrl09!=0x05 = config not
             * landing (wiring/solder). */
            static uint32_t mag_diag = 0;
            if (++mag_diag >= 250) {
                mag_diag = 0;
                imu_mag_ensure_continuous();
                imu_mag_read_ctrl(&mag_ctrl);
                mag_diag_now = true;
            }

            float mx_cal = ((float)raw_mx - calib->m_bias[0]) * calib->m_scale[0];
            float my_cal = ((float)raw_my - calib->m_bias[1]) * calib->m_scale[1];
            float mz_cal = ((float)raw_mz - calib->m_bias[2]) * calib->m_scale[2];

            mag_filt[0] += MAG_LPF_ALPHA * (mx_cal - mag_filt[0]);
            mag_filt[1] += MAG_LPF_ALPHA * (my_cal - mag_filt[1]);
            mag_filt[2] += MAG_LPF_ALPHA * (mz_cal - mag_filt[2]);
        }

        // ACCEL/GYRO READ
        ag_err = imu_read_accel_gyro(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        read_success = (ag_err == ESP_OK);
        xSemaphoreGive(g_i2c_mutex);

        if (!mag_ok) {
            ++mag_fails;
            if (mag_fails == 50) imu_set_mag_ok(false);   /* ~1s dead → report it */
            /* Ground-truth scan on first sustained mag fault too (not just ICM),
             * so a dead mag on a working bus is settled, not guessed. */
            if (mag_fails == 50 && !bus_scanned &&
                xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                bus_scanned = true;
                imu_bus_scan();
                xSemaphoreGive(g_i2c_mutex);
            }
            if ((mag_fails % 100) == 1) {
                ESP_LOGW(FUSION_TAG, "QMC5883L mag read failing (%lu fails, %s) — heading frozen",
                         (unsigned long)mag_fails, esp_err_to_name(mag_err));
            }
            /* Sustained NACK never returns ESP_OK, so the recover-after-success
             * path below can't fire. Force a full re-add + reconfigure every
             * ~2s so a chip that dropped off the bus can rejoin. */
            if ((mag_fails % 100) == 0 &&
                xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                esp_err_t rec = imu_recover_mag();
                xSemaphoreGive(g_i2c_mutex);
                if (rec == ESP_OK) {
                    ESP_LOGI(FUSION_TAG, "mag recovered after %lu fails — re-added + reconfigured",
                             (unsigned long)mag_fails);
                    imu_set_mag_ok(true);
                    mag_fails = 0;
                }
            }
        } else if (mag_fails) {
            /* Only reconfigure after a genuine ~1s outage (mag_fails>=50). A brief
             * 1-2 read glitch recovers on its own; the old unconditional soft-reset
             * fired on EVERY glitch (~360ms) and the read right after it grabbed
             * garbage — that churn is what killed the heading on the dashboard. */
            if (mag_fails >= 50) {
                if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    imu_reinit_mag();
                    xSemaphoreGive(g_i2c_mutex);
                }
                ESP_LOGI(FUSION_TAG, "mag recovered after %lu fails — reconfigured",
                         (unsigned long)mag_fails);
                imu_set_mag_ok(true);
            }
            mag_fails = 0;
        }
        if (!read_success) {
            ++ag_fails;
            if (ag_fails == 50) imu_set_icm_ok(false);    /* ~1s dead → report it */
            /* Once, when the bus is known-good at runtime (mag reading fine),
             * dump who actually ACKs — settles chip-absent vs read-path bug. */
            if (ag_fails == 50 && !bus_scanned &&
                xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                bus_scanned = true;
                imu_bus_scan();
                xSemaphoreGive(g_i2c_mutex);
            }
            if ((ag_fails % 100) == 1) {
                ESP_LOGW(FUSION_TAG, "ICM20948 accel/gyro read failing (%lu fails, %s) — pitch/roll frozen",
                         (unsigned long)ag_fails, esp_err_to_name(ag_err));
            }
            /* Sustained NACK never returns ESP_OK, so the recover-after-success
             * path below can't fire. Force a full re-probe (both straps) +
             * reconfigure every ~2s so a reset / address-moved chip can return. */
            if ((ag_fails % 100) == 0 &&
                xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                esp_err_t rec = imu_recover_accel_gyro();
                xSemaphoreGive(g_i2c_mutex);
                if (rec == ESP_OK) {
                    ESP_LOGI(FUSION_TAG, "accel/gyro recovered after %lu fails — reprobed + reconfigured",
                             (unsigned long)ag_fails);
                    imu_set_icm_ok(true);
                    ag_fails = 0;
                }
            }
        } else if (ag_fails) {
            /* Same as the mag: only re-wake/reconfigure after a real ~1s outage,
             * not on every brief glitch — no need to churn a chip that self-recovered. */
            if (ag_fails >= 50) {
                if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    imu_reinit_accel_gyro();
                    xSemaphoreGive(g_i2c_mutex);
                }
                ESP_LOGI(FUSION_TAG, "accel/gyro recovered after %lu fails — reconfigured",
                         (unsigned long)ag_fails);
                imu_set_icm_ok(true);
            }
            ag_fails = 0;
        }

        // --- FUSION ---
        if (read_success) {
            float gx_rate = ((float)raw_gx - calib->g_bias[0]) / GYRO_SCALE_250DPS;
            float gy_rate = ((float)raw_gy - calib->g_bias[1]) / GYRO_SCALE_250DPS;

            float acc_roll  = atan2f(raw_ay, raw_az) * RAD_TO_DEG;

            float acc_denom = sqrtf((float)raw_ay * raw_ay + (float)raw_az * raw_az);
            if (acc_denom < ACCEL_EPSILON) {
                acc_denom = ACCEL_EPSILON;
            }
            float acc_pitch = atan2f(-raw_ax, acc_denom) * RAD_TO_DEG;

            // Simple complementary filter (alpha parsed once in fusion_init)

            roll  = alpha * (roll  + gx_rate * dt) + (1.0f - alpha) * acc_roll;
            pitch = alpha * (pitch + gy_rate * dt) + (1.0f - alpha) * acc_pitch;

            // --- COMPASS ---
            float p_rad = pitch * DEG_TO_RAD;
            float r_rad = roll * DEG_TO_RAD;

            float Xh = mag_filt[0] * cosf(p_rad) + mag_filt[2] * sinf(p_rad);
            float Yh = mag_filt[0] * sinf(r_rad) * sinf(p_rad) + mag_filt[1] * cosf(r_rad) - mag_filt[2] * sinf(r_rad) * cosf(p_rad);

            float heading = atan2f(Yh, Xh) * RAD_TO_DEG;

            heading -= calib->heading_tare;

            if (heading < 0.0f) {
                heading += 360.0f;
            }
            if (heading >= 360.0f) {
                heading -= 360.0f;
            }

            // --- THREAD SAFE WRITE ---
            if (xSemaphoreTake(fusion_mutex, portMAX_DELAY) == pdTRUE) {
                current_fusion.roll = roll - calib->roll_tare;
                current_fusion.pitch = pitch - calib->pitch_tare;
                current_fusion.heading = heading;
                xSemaphoreGive(fusion_mutex);
            }
            diag_heading = heading;   /* what the UI will show this tick */
        }

        /* Wiring-vs-firmware oracle (~5s), logged with the bus released. Rotate the
         * boat and read ONE line to localise the fault:
         *   raw moves + cal moves + hdg moves → mag, wiring and software all good
         *   raw moves + cal frozen/near-zero  → software (m_bias/m_scale), NOT wiring
         *   raw frozen + ctrl09==0x05         → chip stopped measuring (standby)
         *   raw frozen + ctrl09!=0x05         → config not landing (wiring/solder/pull-ups)
         * cal[] is the calibrated+filtered mag that DIRECTLY drives hdg, so the
         * raw-vs-cal split is the clean line between a wiring fault and a firmware one. */
        if (mag_diag_now) {
            ESP_LOGI(FUSION_TAG,
                     "MAG DIAG raw=(%d,%d,%d) cal=(%.0f,%.0f,%.0f) hdg=%.1f ctrl09=0x%02X — rotate boat; raw+cal+hdg must ALL move",
                     raw_mx, raw_my, raw_mz,
                     mag_filt[0], mag_filt[1], mag_filt[2],
                     diag_heading, mag_ctrl);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
