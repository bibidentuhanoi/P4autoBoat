#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>

#include "common.h"
#include "esp_err.h"

/* IMU + compass calibration, run as its own task once the boat is fully up
 * (WiFi, sensors, fusion), so the dashboard can show every step.
 *
 *   1. HOLD STILL  gyro drift + level reference; ends when the boat IS still
 *   2. SPIN        turn the boat slowly, flat, about two full circles; ends
 *                  when every direction is covered and the gyro counted 1.5+
 *   3. CHECK       ellipse fit + quality gate (mag_cal.h)
 *   PASS  -> saved to flash and applied at once
 *   FAIL  -> nothing changes; the previous calibration stays in use
 *
 * Started at power-on (BOOT held, or nothing valid stored) or from the
 * dashboard's Calibrate button (CompassCalCommand), which also cancels.
 * Arming is refused for as long as it runs; a start is refused while armed. */

/* CompassCalStatus.state */
typedef enum {
    COMPASS_CAL_IDLE = 0,
    COMPASS_CAL_STILL = 1,
    COMPASS_CAL_SPIN = 2,
    COMPASS_CAL_CHECKING = 3,
    COMPASS_CAL_PASS = 4,
    COMPASS_CAL_FAIL = 5,
} compass_cal_state_t;

/* CompassCalStatus.reason.  10 + v is mag_cal_verdict_t v (the fit gates). */
typedef enum {
    COMPASS_CAL_REASON_NONE = 0,
    COMPASS_CAL_REASON_STILL_TIMEOUT = 1,   /* never held still for 60 s */
    COMPASS_CAL_REASON_GYRO_BIAS = 2,       /* gyro drift implausibly large */
    COMPASS_CAL_REASON_SPIN_TIMEOUT = 3,    /* circle not completed in 3 min */
    COMPASS_CAL_REASON_NO_MEMORY = 4,
    COMPASS_CAL_REASON_SAVE_FAILED = 5,     /* flash write failed: not applied */
    COMPASS_CAL_REASON_NO_IMU = 6,          /* no IMU readings at all */
    COMPASS_CAL_REASON_SPIN_NOT_STARTED = 7,/* no turning within 30 s: cancelled */
    COMPASS_CAL_REASON_ARMED = 8,           /* dashboard start refused: motors armed */
    COMPASS_CAL_REASON_CANCELLED = 9,       /* dashboard Cancel */
    COMPASS_CAL_REASON_FIT_BASE = 10,
} compass_cal_reason_t;

/* Starts the CompassCal task.  With run_now it walks through the steps above;
 * either way it afterwards reports the calibration's health once a second
 * (WiFi mode only).  On PASS `live` is updated to the new calibration.
 * stored_valid: `live` came from flash.  When false and only the spin fails,
 * the step-1 gyro drift + level are still saved (compass NOT calibrated). */
esp_err_t compass_cal_start(CalibrationData *live, bool run_now, bool stored_valid);

/* True while a calibration runs (boot or dashboard) until PASS/FAIL. */
bool compass_cal_active(void);

#endif // CALIBRATION_H
