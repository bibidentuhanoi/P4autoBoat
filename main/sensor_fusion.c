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

/* ---- DIAGNOSTIC: magnetometer locus capture -----------------------------
 * Set to 1 to stream the RAW magnetometer + computed heading as compact CSV
 * ("MLOC,idx,mx,my,mz,hdg") at ~10Hz. Rotate the boat LEVEL through a full
 * turn and paste the MLOC block; fitting the (mx,my) locus offline shows
 * whether the field traces a clean CENTRED CIRCLE (calibration good, sticking
 * is elsewhere) or an OFF-CENTRE / ROTATED ELLIPSE (per-axis scale can't fix
 * it → ellipsoid fit needed). Set back to 0 for normal operation. */
#define MAG_LOCUS_CAPTURE   0

/* ---- Gyro + mag complementary YAW fusion --------------------------------
 * Heading was pure instantaneous magnetometer — jittery, and on water the
 * tilt-comp turns wave rock into heading swings. Fuse the gyro Z rate (smooth,
 * tilt-immune, short-term) with the mag bearing (absolute, anchors north over
 * ~seconds), same idea as the pitch/roll complementary filter. Gyro carries the
 * heading through mag disturbances (motors/tilt); mag kills gyro drift. */
#define YAW_GYRO_SIGN        (+1.0f)   /* flip to -1.0f if heading runs BACKWARDS vs the turn */
#define YAW_MAG_TILT_LIMIT   45.0f     /* deg: above this, tilt-comp unreliable -> gyro-only */
#define YAW_MAG_NORM_MIN     800.0f    /* LSB: reject a near-zero horizontal field (bad/disturbed) */

// Globals
static CalibrationData* calib;
static SemaphoreHandle_t fusion_mutex;
static FusionResult current_fusion;

// Helper
static float mag_filt[3] = {0};
static float pitch = 0.0f;
static float roll = 0.0f;
static float alpha;
static float yaw_alpha;            /* gyro/mag complementary weight for heading */
static float declination_deg;      /* magnetic -> true north correction (deg) */
static float heading_yaw = 0.0f;   /* fused absolute yaw state (deg) */
static bool  yaw_init = false;     /* snap heading_yaw to mag on first valid fix */

void fusion_init(CalibrationData* calib_data) {
    calib = calib_data;
    fusion_mutex = xSemaphoreCreateMutex();

    // Parse alpha once — was strtof() every 20ms tick (Bug 4)
    alpha = strtof(CONFIG_FUSION_COMPLEMENTARY_ALPHA, NULL);
    if (alpha <= 0.0f || alpha >= 1.0f) {
        alpha = 0.96f;
    }

    // Yaw complementary weight (higher = trust gyro more short-term) + declination.
    yaw_alpha = strtof(CONFIG_FUSION_YAW_ALPHA, NULL);
    if (yaw_alpha <= 0.0f || yaw_alpha >= 1.0f) {
        yaw_alpha = 0.98f;
    }
    declination_deg = strtof(CONFIG_HEADING_DECLINATION_DEG, NULL);
    yaw_init = false;

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
        if (dt > 0.1f) dt = 0.1f;   /* clamp: after an inference stall, don't let the integrators jump */
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
        static float diag_heading = 0.0f;  /* last FUSED heading — mirrors the UI value for the diag line */
#if MAG_LOCUS_CAPTURE
        static float diag_mag_only = 0.0f; /* mag-only bearing (pre-fusion) for the diag comparison */
        static float diag_gz = 0.0f;       /* gyro Z rate (deg/s) — lets us verify YAW_GYRO_SIGN */
#endif
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

            /* Periodic health readout (~every 30s): re-assert continuous as a
             * belt-and-suspenders and grab the control register. The post-mutex
             * log then prints raw mag + ctrl so the bench test is unambiguous —
             * rotate the boat: raw moving = sensor + wiring good; raw stuck with
             * ctrl09=0x05 = chip stopped measuring; ctrl09!=0x05 = config not
             * landing (wiring/solder). */
            static uint32_t mag_diag = 0;
            if (++mag_diag >= 1500) {
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

            // --- COMPASS: gyro + mag complementary yaw fusion ---
            float p_rad = pitch * DEG_TO_RAD;
            float r_rad = roll * DEG_TO_RAD;

            // Tilt-compensated horizontal mag components (NXP AN4248).
            float Xh = mag_filt[0] * cosf(p_rad) + mag_filt[2] * sinf(p_rad);
            float Yh = mag_filt[0] * sinf(r_rad) * sinf(p_rad) + mag_filt[1] * cosf(r_rad) - mag_filt[2] * sinf(r_rad) * cosf(p_rad);
            float mag_hdg = atan2f(Yh, Xh) * RAD_TO_DEG;   // absolute magnetic bearing [-180,180]

            // Gyro Z yaw-rate (deg/s): smooth, tilt-immune short-term heading.
            float gz_rate = YAW_GYRO_SIGN * ((float)raw_gz - calib->g_bias[2]) / GYRO_SCALE_250DPS;
#if MAG_LOCUS_CAPTURE
            diag_gz = gz_rate;
#endif

            if (!yaw_init) {
                heading_yaw = mag_hdg;               // snap to mag on first fix (no startup ramp)
                yaw_init = true;
            } else {
                heading_yaw += gz_rate * dt;         // (1) gyro predicts every tick
                if (mag_ok) {
                    float err = mag_hdg - heading_yaw;   // wrap-safe innovation
                    while (err >  180.0f) err -= 360.0f;
                    while (err < -180.0f) err += 360.0f;
                    // (2) mag corrects slowly — gated so a tilted/disturbed mag can't yank it.
                    float mag_norm = sqrtf(Xh * Xh + Yh * Yh);
                    if (mag_norm > YAW_MAG_NORM_MIN &&
                        fabsf(pitch) < YAW_MAG_TILT_LIMIT &&
                        fabsf(roll)  < YAW_MAG_TILT_LIMIT) {
                        heading_yaw += (1.0f - yaw_alpha) * err;
                    }
                }
            }
            while (heading_yaw >= 360.0f) heading_yaw -= 360.0f;
            while (heading_yaw <    0.0f) heading_yaw += 360.0f;

            // Zero reference (point-zero / mounting offset) + declination -> true bow heading.
            float heading = heading_yaw - calib->heading_tare + declination_deg;
            while (heading >= 360.0f) heading -= 360.0f;
            while (heading <    0.0f) heading += 360.0f;

            // Mag-only bearing (diagnostic: compare fused-vs-raw-mag to verify the fusion + gyro sign).
            float mag_only = mag_hdg - calib->heading_tare + declination_deg;
            while (mag_only >= 360.0f) mag_only -= 360.0f;
            while (mag_only <    0.0f) mag_only += 360.0f;
#if MAG_LOCUS_CAPTURE
            diag_mag_only = mag_only;
#endif

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

#if MAG_LOCUS_CAPTURE
        /* Dense raw-locus stream for offline circle-vs-ellipse fitting (see the
         * MAG_LOCUS_CAPTURE note near the top). ~10Hz, only when this tick's mag
         * read is fresh so mx/my/mz are real. */
        static uint32_t mloc_ctr = 0, mloc_idx = 0;
        if (mag_ok && ++mloc_ctr >= 5) {
            mloc_ctr = 0;
            ESP_LOGI(FUSION_TAG, "MLOC,%lu,%d,%d,%d,%.1f,%.1f,%.2f",
                     (unsigned long)mloc_idx++, raw_mx, raw_my, raw_mz,
                     diag_heading, diag_mag_only, diag_gz);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
