#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     valid;           // Most recent fix was usable (RMC 'A' or GGA quality > 0).
    double   latitude;        // WGS84 degrees (+ = North).
    double   longitude;       // WGS84 degrees (+ = East).
    float    altitude_m;      // Mean sea level altitude (metres, from GGA).
    float    speed_mps;       // Ground speed (m/s, converted from RMC knots).
    float    course_deg;      // Course over ground (deg true, 0..360).
    float    speed_acc_mps;   // Speed accuracy estimate (m/s, UBX sAcc); 0 = unknown.
    float    vel_n_mps;       // NED velocity North (m/s, UBX NAV-PVT).
    float    vel_e_mps;       // NED velocity East  (m/s, UBX NAV-PVT).
    float    vel_d_mps;       // NED velocity Down  (m/s, UBX NAV-PVT).
    uint8_t  fix_quality;     // Fix type/quality (UBX fixType, or NMEA GGA quality).
    uint8_t  satellites;      // Satellites in use.
    float    hdop;            // Horizontal dilution of precision.
    uint64_t utc_ms;          // ms since Unix epoch (UTC), 0 = unknown.
    int64_t  last_update_us;  // esp_timer_get_time() when any field was last updated.
} gps_fix_t;

/* Stale threshold — if no NMEA update within this window, get_fix() forces valid=false. */
#define GPS_STALE_US  (5LL * 1000 * 1000)

/**
 * Initialize GPS driver: configures UART + internal RX/parse task.
 * Idempotent; returns ESP_ERR_INVALID_STATE on a second call.
 *
 * @param uart_num  ESP-IDF UART port (e.g., UART_NUM_1).
 * @param rx_pin    GPIO connected to GPS module TX (MCU receives).
 * @param tx_pin    GPIO connected to GPS module RX (MCU transmits, used for UBX config).
 * @param baud      Baud rate (NEO-8M stock = 9600).
 */
esp_err_t gps_driver_init(int uart_num, int rx_pin, int tx_pin, int baud);

/**
 * Copy the most recent fix snapshot. Always fills *out; valid=false if no recent fix.
 */
esp_err_t gps_driver_get_fix(gps_fix_t *out);

/**
 * True when there is a usable, fresh fix (valid + >= min satellites) — the
 * gate for arming propulsion. Bench/indoor use the manual override instead.
 */
bool gps_driver_has_lock(void);

#ifdef __cplusplus
}
#endif
