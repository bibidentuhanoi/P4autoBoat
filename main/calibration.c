#include "calibration.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "sdkconfig.h"
#include "drivers/status_led.h"

static const char* TAG = "CALIB";

// ============================================================
// DYNAMIC CALIBRATION THRESHOLDS
// Tune these for your environment if needed
// ============================================================

// Phase 1: Gyro/Level
#define STILLNESS_ACCEL_VARIANCE_THRESHOLD  8000.0f   // LSB^2 — max variance to be considered "still"
#define STILLNESS_WINDOW_SAMPLES            50        // samples to measure variance over
#define STILLNESS_TIMEOUT_MS                10000     // give up waiting for stillness after this
#define GYRO_BIAS_SANITY_MAX                2000.0f   // LSB — if bias > this, sensor was likely disturbed

// Phase 2: Mag figure-8
#define MAG_MIN_DURATION_MS                 8000      // never stop before this even if converged
#define MAG_MAX_DURATION_MS                 60000     // give up and save best data after this
#define MAG_CONVERGENCE_WINDOW_MS           4000      // no new min/max in this window = converged
#define MAG_MIN_AXIS_RANGE_LSB              400       // minimum per-axis chord to accept as covered
#define MAG_SCALE_SANITY_MIN                0.3f      // scale factor below this = bad calibration
#define MAG_SCALE_SANITY_MAX                3.0f      // scale factor above this = bad calibration

// Phase 3: Alignment
#define ALIGN_STABILITY_SAMPLES             100
#define ALIGN_STABILITY_MAX_STDDEV          5.0f      // degrees — heading must be this stable to accept


// ============================================================
// INTERNAL TYPES
// ============================================================

typedef struct {
    int   score;
    bool  gyro_ok;
    bool  mag_coverage_ok;
    bool  mag_scale_ok;
    bool  align_ok;
    float mag_chord[3];
    float gyro_bias_magnitude;
} CalibQuality;


// ============================================================
// HELPERS
// ============================================================

static float compute_variance(float* arr, int n) {
    if (n <= 1) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < n; i++) { sum += arr[i]; }
    float mean = sum / n;
    float var  = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = arr[i] - mean;
        var += d * d;
    }
    return var / n;
}

/* Solve the 4x4 linear system A x = b in place (Gaussian elimination + partial
 * pivot). Returns false if singular. Used by the magnetometer sphere fit. */
static bool solve4(double A[4][4], double b[4], double x[4]) {
    for (int col = 0; col < 4; col++) {
        int piv = col;
        for (int r = col + 1; r < 4; r++)
            if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
        if (fabs(A[piv][col]) < 1e-9) return false;
        if (piv != col) {
            for (int c = 0; c < 4; c++) { double t = A[col][c]; A[col][c] = A[piv][c]; A[piv][c] = t; }
            double t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        for (int r = 0; r < 4; r++) {
            if (r == col) continue;
            double f = A[r][col] / A[col][col];
            for (int c = col; c < 4; c++) A[r][c] -= f * A[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int i = 0; i < 4; i++) x[i] = b[i] / A[i][i];
    return true;
}

static void countdown(int seconds) {
    for (int i = seconds; i > 0; i--) {
        ESP_LOGI(TAG, "%d...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void print_quality_report(const CalibQuality* q) {
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "   CALIBRATION QUALITY REPORT");
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "Overall Score: %d / 100", q->score);
    ESP_LOGI(TAG, " ");

    // Gyro
    if (q->gyro_ok) {
        ESP_LOGI(TAG, "[GYRO BIAS]  OK  (magnitude: %.1f LSB)", q->gyro_bias_magnitude);
    } else {
        ESP_LOGW(TAG, "[GYRO BIAS]  WARNING - HIGH BIAS  (magnitude: %.1f LSB)", q->gyro_bias_magnitude);
        ESP_LOGW(TAG, "  >> Board may not have been still during Phase 1.");
        ESP_LOGW(TAG, "  >> Check for vibration or nearby magnetic interference.");
    }

    // Mag coverage
    if (q->mag_coverage_ok) {
        ESP_LOGI(TAG, "[MAG AXES]   OK");
    } else {
        ESP_LOGW(TAG, "[MAG AXES]   WARNING - POOR AXIS COVERAGE");
        ESP_LOGW(TAG, "  >> Next time, tilt the sensor MORE aggressively in all directions.");
    }
    ESP_LOGI(TAG, "  >> Chord  X: %d  Y: %d  Z: %d  LSB  (min needed: %d)",
             (int)q->mag_chord[0], (int)q->mag_chord[1], (int)q->mag_chord[2],
             MAG_MIN_AXIS_RANGE_LSB);

    // Mag scale
    if (q->mag_scale_ok) {
        ESP_LOGI(TAG, "[MAG SCALE]  OK");
    } else {
        ESP_LOGW(TAG, "[MAG SCALE]  WARNING - EXTREME SCALE FACTORS DETECTED");
        ESP_LOGW(TAG, "  >> Severe magnetic distortion near sensor.");
        ESP_LOGW(TAG, "  >> Check for nearby ferromagnetic objects or high-current cables.");
    }

    // Alignment
    if (q->align_ok) {
        ESP_LOGI(TAG, "[ALIGNMENT]  OK");
    } else {
        ESP_LOGW(TAG, "[ALIGNMENT]  WARNING - HEADING WAS UNSTABLE");
        ESP_LOGW(TAG, "  >> Sensor may have moved during Phase 3. Zero reference may be off.");
    }

    ESP_LOGI(TAG, "================================================");

    if (q->score < 60) {
        ESP_LOGW(TAG, ">>> RESULT: Score below 60. Please recalibrate before trusting output.");
    } else if (q->score < 80) {
        ESP_LOGI(TAG, ">>> RESULT: Acceptable. For best results, recalibrate in final mounted position.");
    } else {
        ESP_LOGI(TAG, ">>> RESULT: Good calibration. You're ready to go!");
    }
    ESP_LOGI(TAG, "================================================");
}


// ============================================================
// PHASE 1: GYRO & LEVEL
// ============================================================

static bool phase1_gyro_level(CalibrationData* out, CalibQuality* quality) {
    status_led_set(STATUS_LED_CAL_STILL);   // LED solid: hold flat & still
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "################################################");
    ESP_LOGI(TAG, "   PHASE 1: GYRO & LEVEL");
    ESP_LOGI(TAG, "################################################");
    ESP_LOGI(TAG, "Place the board FLAT and STATIONARY on a level surface.");
    ESP_LOGI(TAG, "Waiting for stillness...");

    // --- Wait for the board to actually be still before sampling ---
    float   az_window[STILLNESS_WINDOW_SAMPLES];
    bool    is_still        = false;
    int64_t stillness_start = esp_timer_get_time();

    while (!is_still) {
        for (int i = 0; i < STILLNESS_WINDOW_SAMPLES; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            if (imu_read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
                az_window[i] = (float)az;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        float   var     = compute_variance(az_window, STILLNESS_WINDOW_SAMPLES);
        int64_t elapsed = (esp_timer_get_time() - stillness_start) / 1000;

        if (var < STILLNESS_ACCEL_VARIANCE_THRESHOLD) {
            is_still = true;
            ESP_LOGI(TAG, ">> Board is STILL! (variance: %.1f)  Starting measurement...", var);
        } else {
            ESP_LOGW(TAG, "   Still moving... (variance: %.1f)  Put it down and hold still!  [%lldms elapsed]",
                     var, elapsed);
        }

        if (elapsed > STILLNESS_TIMEOUT_MS) {
            ESP_LOGW(TAG, "!! Timeout waiting for stillness. Proceeding with best available data.");
            is_still = true;
        }
    }

    // --- Sample gyro and accel ---
    long g_sum[3] = {0};
    long a_sum[3] = {0};
    int  samples  = CONFIG_CALIB_GYRO_SAMPLES;
    int  valid    = 0;

    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (imu_read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            a_sum[0] += ax; a_sum[1] += ay; a_sum[2] += az;
            g_sum[0] += gx; g_sum[1] += gy; g_sum[2] += gz;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "!! ERROR: No valid IMU readings in Phase 1! Check I2C connection.");
        return false;
    }

    out->g_bias[0] = (float)g_sum[0] / valid;
    out->g_bias[1] = (float)g_sum[1] / valid;
    out->g_bias[2] = (float)g_sum[2] / valid;

    float ax_avg = (float)a_sum[0] / valid;
    float ay_avg = (float)a_sum[1] / valid;
    float az_avg = (float)a_sum[2] / valid;

    float acc_denom = sqrtf(ay_avg * ay_avg + az_avg * az_avg);
    if (acc_denom < 0.0001f) { acc_denom = 0.0001f; }

    out->roll_tare  = atan2f(ay_avg, az_avg)     * RAD_TO_DEG;
    out->pitch_tare = atan2f(-ax_avg, acc_denom) * RAD_TO_DEG;

    // --- Quality assessment ---
    float bias_mag = sqrtf(out->g_bias[0] * out->g_bias[0] +
                           out->g_bias[1] * out->g_bias[1] +
                           out->g_bias[2] * out->g_bias[2]);

    quality->gyro_bias_magnitude = bias_mag;
    quality->gyro_ok = (bias_mag < GYRO_BIAS_SANITY_MAX);

    ESP_LOGI(TAG, ">> Gyro Bias:  X=%.1f  Y=%.1f  Z=%.1f  LSB  (magnitude: %.1f)",
             out->g_bias[0], out->g_bias[1], out->g_bias[2], bias_mag);
    ESP_LOGI(TAG, ">> Level Tare: Pitch=%.2f deg  Roll=%.2f deg",
             out->pitch_tare, out->roll_tare);

    if (!quality->gyro_ok) {
        ESP_LOGW(TAG, "!! Gyro bias magnitude is HIGH (%.1f LSB). Was the board really still?", bias_mag);
    } else {
        ESP_LOGI(TAG, ">> Phase 1 complete!");
    }

    return true;
}


// ============================================================
// PHASE 2: MAG FIGURE-8
// ============================================================

static bool phase2_mag(CalibrationData* out, CalibQuality* quality) {
    status_led_set(STATUS_LED_CAL_MOVE);    // LED blips: pick up & figure-8
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "################################################");
    ESP_LOGI(TAG, "   PHASE 2: COMPASS FIGURE-8");
    ESP_LOGI(TAG, "################################################");
    ESP_LOGI(TAG, "Pick up the board and rotate it in a Figure-8.");
    ESP_LOGI(TAG, "IMPORTANT: Tilt in ALL directions -- not just flat spinning!");
    ESP_LOGI(TAG, "The routine stops automatically once your data is good.");
    ESP_LOGI(TAG, "Max time: %d seconds.", MAG_MAX_DURATION_MS / 1000);

    countdown(3);
    ESP_LOGI(TAG, ">> GO!  Keep moving!");

    int16_t m_min[3] = { 30000,  30000,  30000};
    int16_t m_max[3] = {-30000, -30000, -30000};

    /* Least-squares sphere-fit accumulators. Doubles: samples ~1e4, summed over
     * thousands of points => ~1e11, well within double precision. */
    double Sx=0, Sy=0, Sz=0, Sxx=0, Syy=0, Szz=0, Sxy=0, Sxz=0, Syz=0;
    double Sxw=0, Syw=0, Szw=0, Sw=0;
    long   Nfit=0;

    int64_t start_time       = esp_timer_get_time();
    int64_t last_update_time = start_time;
    int64_t last_print_time  = start_time;
    int     total_samples    = 0;
    bool    converged        = false;

    while (true) {
        int64_t now_us          = esp_timer_get_time();
        int64_t elapsed_ms      = (now_us - start_time)       / 1000;
        int64_t since_update_ms = (now_us - last_update_time) / 1000;

        bool min_time_met   = (elapsed_ms      >= MAG_MIN_DURATION_MS);
        bool max_time_hit   = (elapsed_ms      >= MAG_MAX_DURATION_MS);
        bool data_converged = (since_update_ms >= MAG_CONVERGENCE_WINDOW_MS);

        if (max_time_hit) {
            ESP_LOGW(TAG, ">> Max time reached. Saving best data collected.");
            break;
        }
        if (min_time_met && data_converged) {
            converged = true;
            ESP_LOGI(TAG, ">> Data converged! No new extremes in %.1f s.  Done!",
                     MAG_CONVERGENCE_WINDOW_MS / 1000.0f);
            break;
        }

        // --- Read mag ---
        int16_t mx, my, mz;
        bool updated = false;
        if (imu_read_mag(&mx, &my, &mz) == ESP_OK) {
            total_samples++;
            /* Feed the sphere fit (uses every sample, not just the extremes). */
            double x=mx, y=my, z=mz, w=x*x + y*y + z*z;
            Sx+=x; Sy+=y; Sz+=z;
            Sxx+=x*x; Syy+=y*y; Szz+=z*z; Sxy+=x*y; Sxz+=x*z; Syz+=y*z;
            Sxw+=x*w; Syw+=y*w; Szw+=z*w; Sw+=w; Nfit++;
            if (mx < m_min[0]) { m_min[0] = mx; updated = true; }
            if (mx > m_max[0]) { m_max[0] = mx; updated = true; }
            if (my < m_min[1]) { m_min[1] = my; updated = true; }
            if (my > m_max[1]) { m_max[1] = my; updated = true; }
            if (mz < m_min[2]) { m_min[2] = mz; updated = true; }
            if (mz > m_max[2]) { m_max[2] = mz; updated = true; }
            if (updated) { last_update_time = now_us; }
        }

        // --- Live status every second ---
        if ((now_us - last_print_time) >= 1000000) {
            last_print_time = now_us;

            int chord_x = m_max[0] - m_min[0];
            int chord_y = m_max[1] - m_min[1];
            int chord_z = m_max[2] - m_min[2];

            // Uppercase = axis well covered, lowercase = still needs work
            char cov_x = (chord_x >= MAG_MIN_AXIS_RANGE_LSB) ? 'X' : 'x';
            char cov_y = (chord_y >= MAG_MIN_AXIS_RANGE_LSB) ? 'Y' : 'y';
            char cov_z = (chord_z >= MAG_MIN_AXIS_RANGE_LSB) ? 'Z' : 'z';

            int stable_sec   = (int)(since_update_ms / 1000);
            int min_sec_left = (int)((MAG_MIN_DURATION_MS - elapsed_ms) / 1000);
            if (min_sec_left < 0) { min_sec_left = 0; }

            if (min_sec_left > 0) {
                ESP_LOGI(TAG, "  t=%llds | Axes [%c%c%c] | Range X:%d Y:%d Z:%d | Min time left: %ds",
                         elapsed_ms / 1000, cov_x, cov_y, cov_z,
                         chord_x, chord_y, chord_z, min_sec_left);
            } else {
                ESP_LOGI(TAG, "  t=%llds | Axes [%c%c%c] | Range X:%d Y:%d Z:%d | Stable for: %ds / %.0fs",
                         elapsed_ms / 1000, cov_x, cov_y, cov_z,
                         chord_x, chord_y, chord_z,
                         stable_sec, MAG_CONVERGENCE_WINDOW_MS / 1000.0f);
            }

            // Targeted hints for whichever axes still need coverage
            if (chord_x < MAG_MIN_AXIS_RANGE_LSB) { ESP_LOGW(TAG, "  >> Tilt LEFT and RIGHT more!"); }
            if (chord_y < MAG_MIN_AXIS_RANGE_LSB) { ESP_LOGW(TAG, "  >> Tilt FORWARD and BACK more!"); }
            if (chord_z < MAG_MIN_AXIS_RANGE_LSB) { ESP_LOGW(TAG, "  >> Tilt UPSIDE-DOWN and back!"); }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // --- Robust centre (bias) via least-squares SPHERE FIT over ALL samples ---
    // Min/max trusts only the 6 extreme points, so one noisy sweep => wrong centre
    // (the "heading sticks then jumps" saga). A sphere fit uses every sample and is
    // stable even with poor Z-tilt coverage (validated: cx,cy within ~1 LSB). The
    // field on this boat is a clean sphere (no soft-iron), so the model is centre +
    // unit scale; per-axis m_scale stays 1.0. Min/max is kept as a fallback only.
    quality->mag_chord[0] = (float)(m_max[0] - m_min[0]);
    quality->mag_chord[1] = (float)(m_max[1] - m_min[1]);
    quality->mag_chord[2] = (float)(m_max[2] - m_min[2]);

    bool sphere_ok = false;
    if (Nfit >= 20) {
        double A[4][4] = { {Sxx,Sxy,Sxz,Sx}, {Sxy,Syy,Syz,Sy},
                           {Sxz,Syz,Szz,Sz}, {Sx ,Sy ,Sz ,(double)Nfit} };
        double b[4] = { Sxw, Syw, Szw, Sw };
        double s[4];
        if (solve4(A, b, s)) {
            float cx = (float)(s[0] * 0.5), cy = (float)(s[1] * 0.5), cz = (float)(s[2] * 0.5);
            double r2 = s[3] + (double)cx*cx + (double)cy*cy + (double)cz*cz;
            /* Sane only if the centre sits inside the sampled cloud and R>0. */
            if (r2 > 1.0 &&
                cx > m_min[0] - 1 && cx < m_max[0] + 1 &&
                cy > m_min[1] - 1 && cy < m_max[1] + 1) {
                out->m_bias[0] = cx; out->m_bias[1] = cy; out->m_bias[2] = cz;
                out->m_scale[0] = out->m_scale[1] = out->m_scale[2] = 1.0f;
                sphere_ok = true;
                ESP_LOGI(TAG, ">> Sphere fit OK: centre=(%.0f,%.0f,%.0f) R=%.0f  (%ld pts)",
                         cx, cy, cz, sqrt(r2), Nfit);
            }
        }
    }
    if (!sphere_ok) {
        ESP_LOGW(TAG, ">> Sphere fit rejected (thin/degenerate data) — using min/max centre");
        out->m_bias[0] = (m_max[0] + m_min[0]) / 2.0f;
        out->m_bias[1] = (m_max[1] + m_min[1]) / 2.0f;
        out->m_bias[2] = (m_max[2] + m_min[2]) / 2.0f;
        out->m_scale[0] = out->m_scale[1] = out->m_scale[2] = 1.0f;
    }

    bool coverage_ok = (quality->mag_chord[0] >= MAG_MIN_AXIS_RANGE_LSB) &&
                       (quality->mag_chord[1] >= MAG_MIN_AXIS_RANGE_LSB) &&
                       (quality->mag_chord[2] >= MAG_MIN_AXIS_RANGE_LSB);

    quality->mag_coverage_ok = coverage_ok;
    quality->mag_scale_ok    = sphere_ok;   /* robust fit succeeded (vs min/max fallback) */

    ESP_LOGI(TAG, ">> Mag Bias:    X=%.1f  Y=%.1f  Z=%.1f",
             out->m_bias[0], out->m_bias[1], out->m_bias[2]);
    ESP_LOGI(TAG, ">> Mag Scale:   X=%.4f  Y=%.4f  Z=%.4f",
             out->m_scale[0], out->m_scale[1], out->m_scale[2]);
    ESP_LOGI(TAG, ">> Samples: %d | Converged: %s",
             total_samples, converged ? "YES" : "NO (max time hit)");

    return true;
}


// ============================================================
// PHASE 3: ALIGNMENT
// ============================================================

static bool phase3_alignment(CalibrationData* out, CalibQuality* quality) {
    status_led_set(STATUS_LED_CAL_POINT);   // LED slow blink: point at bow & hold
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "################################################");
    ESP_LOGI(TAG, "   PHASE 3: ALIGNMENT (Set Zero Heading)");
    ESP_LOGI(TAG, "################################################");
    ESP_LOGI(TAG, "Point the sensor exactly FORWARD (toward the bow).");
    ESP_LOGI(TAG, "Hold it PERFECTLY STILL. This sets your zero reference.");

    countdown(5);
    ESP_LOGI(TAG, ">> Capturing heading reference...");

    float headings[ALIGN_STABILITY_SAMPLES];
    int   valid = 0;

    for (int i = 0; i < ALIGN_STABILITY_SAMPLES; i++) {
        int16_t mx, my, mz;
        if (imu_read_mag(&mx, &my, &mz) == ESP_OK) {
            float mx_cal = ((float)mx - out->m_bias[0]) * out->m_scale[0];
            float my_cal = ((float)my - out->m_bias[1]) * out->m_scale[1];

            float h = atan2f(my_cal, mx_cal) * RAD_TO_DEG;
            if (h < 0.0f) { h += 360.0f; }
            headings[valid++] = h;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "!! ERROR: No valid mag readings in Phase 3! Check I2C connection.");
        quality->align_ok = false;
        return false;
    }

    // Mean heading via sin/cos averaging — handles 0/360 wraparound correctly
    float sin_sum = 0.0f, cos_sum = 0.0f;
    for (int i = 0; i < valid; i++) {
        sin_sum += sinf(headings[i] * DEG_TO_RAD);
        cos_sum += cosf(headings[i] * DEG_TO_RAD);
    }

    float ref_heading = atan2f(sin_sum / valid, cos_sum / valid) * RAD_TO_DEG;
    if (ref_heading < 0.0f) { ref_heading += 360.0f; }

    // Circular std dev — true stability measure for angular data
    float R           = sqrtf((sin_sum / valid) * (sin_sum / valid) +
                              (cos_sum / valid) * (cos_sum / valid));
    float circ_stddev = (R > 0.0001f) ? sqrtf(-2.0f * logf(R)) * RAD_TO_DEG : 999.0f;

    ESP_LOGI(TAG, ">> Zero Reference Heading: %.2f deg", ref_heading);
    ESP_LOGI(TAG, ">> Heading Stability (std dev): %.2f deg  (max allowed: %.1f deg)",
             circ_stddev, ALIGN_STABILITY_MAX_STDDEV);

    quality->align_ok = (circ_stddev <= ALIGN_STABILITY_MAX_STDDEV);

    if (!quality->align_ok) {
        ESP_LOGW(TAG, "!! Heading was not stable during capture. Did the sensor move?");
        ESP_LOGW(TAG, "   Saving best reference anyway, but consider redoing Phase 3.");
    } else {
        ESP_LOGI(TAG, ">> Phase 3 complete! Zero heading locked in.");
    }

    out->heading_tare = ref_heading;
    return true;
}


// ============================================================
// PUBLIC ENTRY POINT
// ============================================================

void perform_calibration_routine(CalibrationData* output_calib) {
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "################################################");
    ESP_LOGI(TAG, "   ENTERING CALIBRATION MODE");
    ESP_LOGI(TAG, "   (Data-driven -- stops when quality is met)");
    ESP_LOGI(TAG, "################################################");

    CalibQuality quality = {0};

    bool ok1 = phase1_gyro_level(output_calib, &quality);
    bool ok2 = phase2_mag(output_calib, &quality);
    bool ok3 = phase3_alignment(output_calib, &quality);

    // --- Score ---
    int score = 100;
    if (!ok1 || !quality.gyro_ok)  { score -= 20; }
    if (!ok2)                       { score -= 40; }
    if (!quality.mag_coverage_ok)  { score -= 20; }
    if (!quality.mag_scale_ok)     { score -= 15; }
    if (!ok3 || !quality.align_ok) { score -= 15; }
    if (score < 0) { score = 0; }

    quality.score = score;

    // LED verdict (auto-returns to OFF after the flash): steady triple = good,
    // rapid flutter = redo. 60 mirrors print_quality_report's "recalibrate" line.
    status_led_set(score >= 60 ? STATUS_LED_CAL_DONE_OK : STATUS_LED_CAL_DONE_FAIL);

    print_quality_report(&quality);
}