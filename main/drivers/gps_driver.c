#include "gps_driver.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sdkconfig.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "GPS";

#define NMEA_MAX_LINE     100   /* NMEA 0183 caps at 82 chars incl CRLF; 100 is safe. */
#define UART_RX_BUF       2048
#define GPS_TASK_STACK    4096
#define GPS_TASK_PRIORITY 3

#define UBX_SYNC1         0xB5
#define UBX_SYNC2         0x62
#define UBX_CLASS_NAV     0x01
#define UBX_ID_NAV_PVT    0x07
#define UBX_NAV_PVT_LEN   92
#define GPS_LOCK_MIN_SATS 5     /* valid fix + this many sats = "locked" for arming */

static int              s_uart_num = -1;
static SemaphoreHandle_t s_mutex   = NULL;
static TaskHandle_t     s_task     = NULL;
static gps_fix_t        s_fix      = {0};
/* When NAV-PVT is flowing it is authoritative; NMEA is ignored while fresh. */
static int64_t          s_last_ubx_us = 0;
/* Link-alive tracking — bumped on ANY checksum-valid sentence/frame, even one
 * we don't otherwise parse (e.g. GSV). Deliberately separate from s_fix's own
 * last_update_us: that one only moves on sentences that carry usable fix
 * fields, this one only means "the module is talking to the UART at all". */
static int64_t          s_last_rx_us       = 0;
static bool              s_logged_first_talk = false;
static bool              s_logged_first_bytes = false;   /* ANY byte at all, valid or not */
/* Config ACK results — written ONCE, synchronously, inside gps_configure_ublox()
 * during app_main()'s boot sequence, before gps_task or any other reader could
 * possibly exist yet. Never modified again after that. No mutex needed: this
 * is effectively immutable post-boot, not actively-mutated shared state like
 * s_fix/s_last_rx_us are. */
static bool              s_cfg_prt_acked  = false;
static bool              s_cfg_rate_acked = false;
static bool              s_cfg_msg_acked  = false;
/* ASCII-with-dots preview of the first raw bytes seen (non-printable -> '.'),
 * populated once alongside the hex-dump log line, so a human can judge
 * text-vs-garbage from the dashboard without a serial monitor attached. */
static char               s_raw_preview[17] = {0};   /* 16 visible chars + NUL */
static int                s_raw_printable_pct = -1;  /* -1 = no bytes seen yet */
/* True when the first bytes start with the UBX sync sequence (0xB5 0x62) —
 * a correctly-framed BINARY protocol, not garbage. Printable-ASCII% is only
 * a valid text-vs-garbage signal for NMEA; a genuine UBX frame is *expected*
 * to score low on that metric since it's binary, not text. Without this
 * check, working UBX traffic reads as a false-positive "GARBAGE" verdict. */
static bool                s_raw_looks_ubx = false;
/* Auto-baud scan result — written once, synchronously, before gps_task exists.
 * s_detected_baud always ends up non-zero (falls back to CONFIG_GPS_BAUD if
 * the scan found nothing) — s_baud_confirmed is what actually distinguishes
 * "this baud is verified working" from "this is just the fallback guess". */
static uint32_t           s_detected_baud = 0;
static bool                s_baud_confirmed = false;

/* ---------- little-endian readers ---------- */
static inline uint16_t rd_u16(const uint8_t *p){ return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd_u32(const uint8_t *p){
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t  rd_i32(const uint8_t *p){ return (int32_t)rd_u32(p); }

/* ---------- NMEA helpers ---------- */

static uint8_t nmea_xor(const char *begin, const char *end) {
    uint8_t cs = 0;
    for (const char *c = begin; c < end; c++) cs ^= (uint8_t)*c;
    return cs;
}

/* Validate "$...*HH" envelope. On success, null-terminates line at '*' and
 * returns pointer to payload (character after '$'). Returns NULL on failure. */
static char *nmea_validate(char *line, size_t len) {
    if (len < 4 || line[0] != '$') return NULL;
    char *star = memchr(line, '*', len);
    if (!star || (star + 2) >= (line + len)) return NULL;
    uint8_t want = (uint8_t)strtoul(star + 1, NULL, 16);
    uint8_t got  = nmea_xor(line + 1, star);
    if (want != got) return NULL;
    *star = 0;         /* strip checksum for easier parsing */
    return line + 1;   /* payload start (past '$') */
}

/* strsep-style: returns current field, advances *cursor to next field or NULL. */
static char *next_field(char **cursor) {
    if (!*cursor) return NULL;
    char *f = *cursor;
    char *comma = strchr(f, ',');
    if (comma) {
        *comma = 0;
        *cursor = comma + 1;
    } else {
        *cursor = NULL;
    }
    return f;
}

/* NMEA lat/lon format is "ddmm.mmmm" (lat) or "dddmm.mmmm" (lon).
 * Integer part / 100 = degrees, remainder = minutes. */
static double nmea_to_deg(const char *field, char hem) {
    if (!field || !*field) return 0.0;
    double raw = strtod(field, NULL);
    int deg = (int)(raw / 100.0);
    double minutes = raw - (double)deg * 100.0;
    double val = (double)deg + minutes / 60.0;
    if (hem == 'S' || hem == 'W') val = -val;
    return val;
}

/* Leap-year predicate. */
static int is_leap(int y) {
    return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

/* UTC y/m/d h/m/s (+ fractional seconds) → ms since Unix epoch. */
static uint64_t utc_to_epoch_ms(int year, int mon, int day,
                                 int hh, int mm, int ss, double frac) {
    static const int md[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (year < 1970 || mon < 1 || mon > 12 || day < 1 || day > 31) return 0;
    uint64_t days = 0;
    for (int y = 1970; y < year; y++) days += 365u + (uint64_t)is_leap(y);
    for (int m = 0; m < mon - 1; m++) {
        days += (uint64_t)md[m];
        if (m == 1 && is_leap(year)) days += 1;
    }
    days += (uint64_t)(day - 1);
    uint64_t secs = days * 86400ull
                  + (uint64_t)hh * 3600ull
                  + (uint64_t)mm * 60ull
                  + (uint64_t)ss;
    return secs * 1000ull + (uint64_t)(frac * 1000.0);
}

/* ---------- UBX NAV-PVT parser (primary) ---------- */

static void parse_navpvt(const uint8_t *p) {
    uint8_t  fixType = p[20];
    uint8_t  flags   = p[21];
    uint8_t  numSV   = p[23];
    int32_t  lon     = rd_i32(p + 24);   /* 1e-7 deg */
    int32_t  lat     = rd_i32(p + 28);   /* 1e-7 deg */
    int32_t  hMSL    = rd_i32(p + 36);   /* mm */
    int32_t  velN    = rd_i32(p + 48);   /* mm/s */
    int32_t  velE    = rd_i32(p + 52);   /* mm/s */
    int32_t  velD    = rd_i32(p + 56);   /* mm/s */
    int32_t  gSpeed  = rd_i32(p + 60);   /* mm/s ground speed */
    int32_t  headMot = rd_i32(p + 64);   /* 1e-5 deg */
    uint32_t sAcc    = rd_u32(p + 68);   /* mm/s speed accuracy */
    uint16_t pDOP    = rd_u16(p + 76);   /* 0.01 */
    bool     fixOk   = (flags & 0x01) != 0;

    uint16_t year = rd_u16(p + 4);
    int mon = p[6], day = p[7], hh = p[8], mm = p[9], ss = p[10];

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_fix.valid         = fixOk && (fixType == 2 || fixType == 3);
    s_fix.latitude      = (double)lat * 1e-7;
    s_fix.longitude     = (double)lon * 1e-7;
    s_fix.altitude_m    = (float)hMSL / 1000.0f;
    s_fix.speed_mps     = (float)gSpeed / 1000.0f;
    s_fix.course_deg    = (float)headMot * 1e-5f;
    s_fix.speed_acc_mps = (float)sAcc / 1000.0f;
    s_fix.vel_n_mps     = (float)velN / 1000.0f;
    s_fix.vel_e_mps     = (float)velE / 1000.0f;
    s_fix.vel_d_mps     = (float)velD / 1000.0f;
    s_fix.fix_quality   = fixType;
    s_fix.satellites    = numSV;
    s_fix.hdop          = (float)pDOP * 0.01f;   /* position DOP as a proxy */
    uint64_t ms = utc_to_epoch_ms(year, mon, day, hh, mm, ss, 0.0);
    if (ms) s_fix.utc_ms = ms;
    s_fix.last_update_us = esp_timer_get_time();
    s_last_ubx_us = s_fix.last_update_us;
    xSemaphoreGive(s_mutex);
}

/* ---------- NMEA sentence parsers (fallback; caller holds s_mutex) ---------- */

static void parse_gga(char *body, gps_fix_t *fix) {
    char *p = body;
    next_field(&p);                    /* talker+"GGA" */
    next_field(&p);                    /* time hhmmss.ss (date comes from RMC) */
    char *lat  = next_field(&p);
    char *ns   = next_field(&p);
    char *lon  = next_field(&p);
    char *ew   = next_field(&p);
    char *qual = next_field(&p);
    char *sats = next_field(&p);
    char *hdop = next_field(&p);
    char *alt  = next_field(&p);

    uint8_t q = qual ? (uint8_t)strtoul(qual, NULL, 10) : 0;
    fix->fix_quality = q;
    if (sats && *sats) fix->satellites = (uint8_t)strtoul(sats, NULL, 10);
    if (hdop && *hdop) fix->hdop       = strtof(hdop, NULL);
    if (alt  && *alt ) fix->altitude_m = strtof(alt, NULL);

    if (q > 0 && lat && *lat && ns && *ns && lon && *lon && ew && *ew) {
        fix->latitude  = nmea_to_deg(lat, *ns);
        fix->longitude = nmea_to_deg(lon, *ew);
        fix->valid = true;
    } else if (q == 0) {
        fix->valid = false;
    }
}

static void parse_rmc(char *body, gps_fix_t *fix) {
    char *p = body;
    next_field(&p);                    /* talker+"RMC" */
    char *time    = next_field(&p);    /* hhmmss.ss */
    char *status  = next_field(&p);    /* A=valid, V=warning */
    char *lat     = next_field(&p);
    char *ns      = next_field(&p);
    char *lon     = next_field(&p);
    char *ew      = next_field(&p);
    char *spd_kn  = next_field(&p);    /* knots */
    char *course  = next_field(&p);    /* deg true */
    char *date    = next_field(&p);    /* ddmmyy */

    bool ok = (status && *status == 'A');
    fix->valid = ok;

    if (ok) {
        if (lat && *lat && ns && *ns && lon && *lon && ew && *ew) {
            fix->latitude  = nmea_to_deg(lat, *ns);
            fix->longitude = nmea_to_deg(lon, *ew);
        }
        if (spd_kn && *spd_kn) fix->speed_mps  = strtof(spd_kn, NULL) * 0.514444f;
        if (course && *course) fix->course_deg = strtof(course, NULL);
    }

    if (date && strlen(date) == 6 && time && strlen(time) >= 6) {
        int d  = (date[0] - '0') * 10 + (date[1] - '0');
        int mo = (date[2] - '0') * 10 + (date[3] - '0');
        int y  = (date[4] - '0') * 10 + (date[5] - '0') + 2000;
        int hh = (time[0] - '0') * 10 + (time[1] - '0');
        int mm = (time[2] - '0') * 10 + (time[3] - '0');
        int ss = (time[4] - '0') * 10 + (time[5] - '0');
        double frac = (time[6] == '.') ? strtod(time + 6, NULL) : 0.0;
        uint64_t ms = utc_to_epoch_ms(y, mo, d, hh, mm, ss, frac);
        if (ms) fix->utc_ms = ms;
    }
}

/* Dispatch a validated, null-terminated NMEA payload (no '$', no '*cs'). */
static void dispatch_sentence(char *body) {
    if (strlen(body) < 5) return;
    /* NAV-PVT is authoritative while fresh — ignore NMEA to avoid fighting. */
    if (s_last_ubx_us != 0 && (esp_timer_get_time() - s_last_ubx_us) < 2000000) return;

    const char *sent = body + 2;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if      (!strncmp(sent, "GGA", 3)) parse_gga(body, &s_fix);
    else if (!strncmp(sent, "RMC", 3)) parse_rmc(body, &s_fix);
    else {
        xSemaphoreGive(s_mutex);
        return;
    }
    s_fix.last_update_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
}

/* ---------- u-blox UBX configuration ---------- */

static void ubx_send(int uart, uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len) {
    uint8_t hdr[6] = { UBX_SYNC1, UBX_SYNC2, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6; i++) { a += hdr[i]; b += a; }
    for (int i = 0; i < len; i++) { a += pl[i]; b += a; }
    uint8_t ck[2] = { a, b };
    uart_write_bytes(uart, hdr, 6);
    if (len) uart_write_bytes(uart, pl, len);
    uart_write_bytes(uart, ck, 2);
    uart_wait_tx_done(uart, pdMS_TO_TICKS(100));
}

/* Blocking wait (up to timeout_ms) for a UBX-ACK-ACK(0x05,0x01)/ACK-NAK(0x05,0x00)
 * frame acknowledging (want_cls, want_id). Simple linear byte scan — fine for a
 * one-shot, synchronous, init-time check, not the main parse loop. Does not
 * re-verify the ACK frame's own checksum (diagnostic only, not safety-critical:
 * worst case is a false positive on a corrupt ACK, not a bad nav command).
 * Logs and returns true=ACK'd, false=NAK'd or timed out (module never replied). */
static bool ubx_wait_ack(int uart, uint8_t want_cls, uint8_t want_id, int timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    int state = 0;
    uint8_t ack_id_field = 0, pl_cls = 0, pl_id = 0;
    while (esp_timer_get_time() < deadline_us) {
        uint8_t b;
        if (uart_read_bytes(uart, &b, 1, pdMS_TO_TICKS(20)) != 1) continue;
        switch (state) {
        case 0: state = (b == UBX_SYNC1) ? 1 : 0; break;
        case 1: state = (b == UBX_SYNC2) ? 2 : 0; break;
        case 2: state = (b == 0x05) ? 3 : 0; break;              /* class must be ACK(0x05) */
        case 3: ack_id_field = b; state = (b == 0x01 || b == 0x00) ? 4 : 0; break;
        case 4: state = (b == 0x02) ? 5 : 0; break;               /* len low = 2 */
        case 5: state = (b == 0x00) ? 6 : 0; break;               /* len high = 0 */
        case 6: pl_cls = b; state = 7; break;
        case 7: pl_id = b; state = 8; break;
        case 8: state = 9; break;                                 /* CK_A — not re-verified */
        case 9:
            state = 0;
            if (pl_cls == want_cls && pl_id == want_id) {
                bool acked = (ack_id_field == 0x01);
                ESP_LOGI(TAG, "UBX %s: class=0x%02X id=0x%02X", acked ? "ACK" : "NAK", want_cls, want_id);
                return acked;
            }
            break;
        }
    }
    ESP_LOGW(TAG, "UBX ACK timeout: no response for class=0x%02X id=0x%02X within %dms",
             want_cls, want_id, timeout_ms);
    return false;
}

/* Common GPS/UART baud rates, most-likely-first. 9600 is the overwhelming
 * stock default for u-blox and most NMEA modules; the rest cover modules
 * shipped pre-configured differently or left at a previous session's rate. */
static const uint32_t BAUD_CANDIDATES[] = { 9600, 4800, 38400, 57600, 115200, 19200 };
#define BAUD_CANDIDATES_COUNT (sizeof(BAUD_CANDIDATES) / sizeof(BAUD_CANDIDATES[0]))
/* Must exceed one full NMEA emission cycle, or a window can land entirely in
 * the silent gap between bursts and falsely conclude a correct baud doesn't
 * work. Most GPS modules (this one's stock default included) burst NMEA once
 * per second at power-on settings, THEN go quiet until the next second — a
 * shorter window has no guarantee of ever overlapping a burst, regardless of
 * whether the baud is actually right. 1100ms guarantees catching at least one
 * full 1Hz cycle with margin for scheduling jitter. Exits immediately on a
 * hit (doesn't wait out the rest of the window), so this is a worst case, not
 * a typical case — ~6.6s only if every wrong candidate is tried in full. */
#define BAUD_SCAN_WINDOW_MS   1100

/* Tries each candidate baud in turn: reconfigure the UART, listen for at
 * least one full NMEA cycle (see BAUD_SCAN_WINDOW_MS) for a checksum-valid
 * sentence (reuses nmea_validate(), the same check the main parser uses),
 * stop on the first hit. Bounded total time — this must NEVER hang boot
 * indefinitely, matching this driver's existing soft-optional philosophy (a
 * GPS problem must never block the rest of the boat). Returns the detected
 * baud, or 0 if no candidate produced valid data.
 * Buffers are static (BSS, not stack) — same pattern gps_task() already uses
 * for upayload — so this carries zero stack-budget risk regardless of how
 * much of app_main()'s stack happens to already be in use when this runs. */
static uint32_t gps_autobaud_scan(int uart) {
    static uint8_t rx[128];
    static char    line[NMEA_MAX_LINE];

    for (size_t ci = 0; ci < BAUD_CANDIDATES_COUNT; ci++) {
        uint32_t candidate = BAUD_CANDIDATES[ci];
        uart_set_baudrate(uart, candidate);
        uart_flush_input(uart);

        int64_t deadline_us = esp_timer_get_time() + (int64_t)BAUD_SCAN_WINDOW_MS * 1000;
        size_t line_len = 0;
        bool   in_nmea = false;

        while (esp_timer_get_time() < deadline_us) {
            int n = uart_read_bytes(uart, rx, sizeof(rx), pdMS_TO_TICKS(50));
            for (int i = 0; i < n; i++) {
                uint8_t b = rx[i];
                if (b == '$') { in_nmea = true; line_len = 0; line[line_len++] = '$'; continue; }
                if (!in_nmea) continue;
                if (b == '\r' || b == '\n') {
                    line[line_len] = 0;
                    if (nmea_validate(line, line_len)) {
                        ESP_LOGI(TAG, "Autobaud: HIT at %u baud", (unsigned)candidate);
                        return candidate;
                    }
                    in_nmea = false; line_len = 0;
                } else if (line_len < sizeof(line) - 1) {
                    line[line_len++] = (char)b;
                } else {
                    in_nmea = false; line_len = 0;   /* overflow — reset */
                }
            }
        }
        ESP_LOGI(TAG, "Autobaud: no valid NMEA at %u baud, trying next", (unsigned)candidate);
    }
    ESP_LOGW(TAG, "Autobaud: no candidate baud produced valid NMEA — giving up, staying at %u",
             (unsigned)BAUD_CANDIDATES[0]);
    return 0;
}

/* Switch the module to the target baud + rate and enable NAV-PVT. Best-effort:
 * if it fails the module keeps emitting NMEA and the fallback parser copes.
 * Now checks the actual UBX ACK/NAK for each command instead of assuming. */
static void gps_configure_ublox(int uart) {
    uint32_t hb = (uint32_t)CONFIG_GPS_HIGH_BAUD;

    /* CFG-PRT (0x06 0x00): UART1, 8N1, target baud, in/out = UBX + NMEA.
     * Its ACK, if the module sends one at all, arrives at the OLD baud —
     * BEFORE the flush+switch below, or it'd be thrown away unread. Some
     * u-blox firmware doesn't ACK a port-config change at all, so a timeout
     * here isn't conclusive by itself — CFG-RATE/CFG-MSG below (fully at the
     * post-switch baud) are the more reliable signal of whether it followed. */
    uint8_t prt[20] = {0};
    prt[0]  = 0x01;                       /* portID = UART1 */
    prt[4]  = 0xD0; prt[5] = 0x08;        /* mode 0x000008D0 (8 data, no parity, 1 stop) */
    prt[8]  = (uint8_t)(hb & 0xFF);
    prt[9]  = (uint8_t)((hb >> 8) & 0xFF);
    prt[10] = (uint8_t)((hb >> 16) & 0xFF);
    prt[11] = (uint8_t)((hb >> 24) & 0xFF);
    prt[12] = 0x03;                       /* inProtoMask  = UBX + NMEA */
    prt[14] = 0x03;                       /* outProtoMask = UBX + NMEA */
    ubx_send(uart, 0x06, 0x00, prt, 20);
    s_cfg_prt_acked = ubx_wait_ack(uart, 0x06, 0x00, 200);

    /* Follow the module to the new baud regardless — if it silently switched
     * anyway we still need to follow; if it didn't, the ACK checks below will
     * make that unambiguous instead of us silently guessing either way. */
    uart_flush_input(uart);
    uart_set_baudrate(uart, hb);
    vTaskDelay(pdMS_TO_TICKS(60));

    /* CFG-RATE (0x06 0x08): measRate ms, navRate 1, timeRef = GPS(1). */
    uint16_t meas = (uint16_t)(1000 / CONFIG_GPS_MEAS_RATE_HZ);
    uint8_t rate[6] = { (uint8_t)(meas & 0xFF), (uint8_t)(meas >> 8), 0x01, 0x00, 0x01, 0x00 };
    ubx_send(uart, 0x06, 0x08, rate, 6);
    s_cfg_rate_acked = ubx_wait_ack(uart, 0x06, 0x08, 200);

    /* CFG-MSG (0x06 0x01): enable NAV-PVT at rate 1 on the current port. */
    uint8_t msg[3] = { UBX_CLASS_NAV, UBX_ID_NAV_PVT, 0x01 };
    ubx_send(uart, 0x06, 0x01, msg, 3);
    s_cfg_msg_acked = ubx_wait_ack(uart, 0x06, 0x01, 200);

    ESP_LOGI(TAG, "u-blox configured: %u baud, %d Hz, NAV-PVT enabled — ACKs: PRT=%s RATE=%s MSG=%s%s",
             (unsigned)hb, CONFIG_GPS_MEAS_RATE_HZ,
             s_cfg_prt_acked ? "OK" : "no-response",
             s_cfg_rate_acked ? "OK" : "no-response",
             s_cfg_msg_acked ? "OK" : "no-response",
             (!s_cfg_prt_acked && !s_cfg_rate_acked && !s_cfg_msg_acked)
                 ? " — module not responding at all (check wiring/power)" : "");
}

/* ---------- link-alive tracking + one-shot "first talk" diagnostic ---------- */

/* Called on every checksum-valid NMEA sentence, whatever its talker/type. */
static void mark_alive_nmea(const char *body) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_last_rx_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
    if (!s_logged_first_talk) {
        s_logged_first_talk = true;
        ESP_LOGI(TAG, "GPS module responding on UART%d — first NMEA sentence: $%s",
                 s_uart_num, body);
    }
}

/* Called on every checksum-valid UBX frame, whatever its class/ID. */
static void mark_alive_ubx(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_last_rx_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
    if (!s_logged_first_talk) {
        s_logged_first_talk = true;
        char hex[3 * 16 + 1] = {0};
        int n = (len < 16) ? (int)len : 16;
        for (int i = 0; i < n; i++) {
            snprintf(hex + i * 3, sizeof(hex) - (size_t)(i * 3), "%02X ", pl[i]);
        }
        ESP_LOGI(TAG, "GPS module responding on UART%d — first UBX frame: "
                      "class=0x%02X id=0x%02X len=%u payload[0:%d]=%s",
                 s_uart_num, cls, id, (unsigned)len, n, hex);
    }
}

/* ---------- UART task: hybrid UBX (primary) + NMEA (fallback) ---------- */

static void gps_task(void *arg) {
    uint8_t rx[256];
    /* NMEA line accumulator */
    char   line[NMEA_MAX_LINE];
    size_t line_len = 0;
    bool   in_nmea = false;
    /* UBX frame state machine */
    enum { U_IDLE, U_S2, U_CLS, U_ID, U_L1, U_L2, U_PL, U_CKA, U_CKB } ust = U_IDLE;
    uint8_t  ucls = 0, uid = 0, ucka = 0, uckb = 0, rcka = 0;
    uint16_t ulen = 0, uidx = 0;
    static uint8_t upayload[128];

    ESP_LOGI(TAG, "GPS task started on UART%d", s_uart_num);

    while (true) {
        int n = uart_read_bytes(s_uart_num, rx, sizeof(rx), pdMS_TO_TICKS(200));
        if (n <= 0) continue;

        /* Fires on ANY byte at all, valid or garbage — the most fundamental
         * "is the wire carrying anything" check, independent of whether it
         * ever parses as valid NMEA/UBX. If this never prints, no bytes are
         * reaching the UART at all (wiring/power). If THIS prints but
         * "module responding" (mark_alive_*) never does, bytes are arriving
         * but not validating — most likely a baud mismatch. */
        if (!s_logged_first_bytes) {
            s_logged_first_bytes = true;
            char hex[3 * 16 + 1] = {0};
            int m = (n < 16) ? n : 16;
            int printable = 0;
            for (int i = 0; i < m; i++) {
                snprintf(hex + i * 3, sizeof(hex) - (size_t)(i * 3), "%02X ", rx[i]);
                bool is_printable = (rx[i] >= 0x20 && rx[i] < 0x7F);
                s_raw_preview[i] = is_printable ? (char)rx[i] : '.';
                if (is_printable) printable++;
            }
            s_raw_preview[m] = 0;
            s_raw_printable_pct = (m > 0) ? (printable * 100 / m) : 0;
            /* Binary UBX check FIRST: a real UBX frame starts with the fixed
             * 0xB5 0x62 sync bytes and will naturally score low on printable%
             * (it's binary, not text) — that low score must not be read as
             * "garbage" when the sync bytes prove it's a correctly-framed
             * protocol. Only fall back to the printable-ASCII heuristic
             * (>=70% = text-like NMEA, below = scrambled bit-timing from a
             * baud mismatch) when the sync check doesn't already settle it. */
            s_raw_looks_ubx = (m >= 2 && rx[0] == UBX_SYNC1 && rx[1] == UBX_SYNC2);
            const char *verdict = s_raw_looks_ubx ? "looks like valid UBX binary"
                                : (s_raw_printable_pct >= 70) ? "looks like TEXT"
                                : "looks like GARBAGE";
            ESP_LOGI(TAG, "GPS UART%d: first raw bytes seen (%d bytes this read, up to 16 shown) "
                          "hex=[%s] ascii=[%s] — %d%% printable (%s)",
                     s_uart_num, n, hex, s_raw_preview, s_raw_printable_pct, verdict);
        }

        for (int i = 0; i < n; i++) {
            uint8_t b = rx[i];

            /* --- inside a UBX frame: consume until complete --- */
            if (ust != U_IDLE) {
                switch (ust) {
                case U_S2:  ust = (b == UBX_SYNC2) ? U_CLS : U_IDLE; break;
                case U_CLS: ucls = b; ucka = b; uckb = b; ust = U_ID;  break;
                case U_ID:  uid  = b; ucka += b; uckb += ucka; ust = U_L1; break;
                case U_L1:  ulen = b; ucka += b; uckb += ucka; ust = U_L2; break;
                case U_L2:
                    ulen |= (uint16_t)b << 8; ucka += b; uckb += ucka; uidx = 0;
                    ust = (ulen == 0) ? U_CKA
                        : (ulen <= sizeof(upayload) ? U_PL : U_IDLE);
                    break;
                case U_PL:
                    upayload[uidx++] = b; ucka += b; uckb += ucka;
                    if (uidx >= ulen) ust = U_CKA;
                    break;
                case U_CKA: rcka = b; ust = U_CKB; break;
                case U_CKB:
                    if (rcka == ucka && b == uckb) {
                        mark_alive_ubx(ucls, uid, upayload, ulen);
                        if (ucls == UBX_CLASS_NAV && uid == UBX_ID_NAV_PVT &&
                            ulen >= UBX_NAV_PVT_LEN) {
                            parse_navpvt(upayload);
                        }
                    }
                    ust = U_IDLE;
                    break;
                default: ust = U_IDLE; break;
                }
                continue;
            }

            /* --- idle: look for a UBX sync or an NMEA '$' --- */
            if (b == UBX_SYNC1) { ust = U_S2; in_nmea = false; line_len = 0; continue; }
            if (b == '$')       { in_nmea = true; line_len = 0; line[line_len++] = '$'; continue; }

            if (in_nmea) {
                if (b == '\r' || b == '\n') {
                    line[line_len] = 0;
                    char *body = nmea_validate(line, line_len);
                    if (body) {
                        mark_alive_nmea(body);
                        dispatch_sentence(body);
                    }
                    in_nmea = false; line_len = 0;
                } else if (line_len < sizeof(line) - 1) {
                    line[line_len++] = (char)b;
                } else {
                    in_nmea = false; line_len = 0;   /* overflow — reset */
                }
            }
        }
    }
}

/* ---------- Public API ---------- */

esp_err_t gps_driver_init(int uart_num, int rx_pin, int tx_pin, int baud) {
    if (s_uart_num >= 0) {
        ESP_LOGW(TAG, "gps_driver_init called twice");
        return ESP_ERR_INVALID_STATE;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    const uart_config_t cfg = {
        .baud_rate           = baud,          /* module's power-on baud (9600) */
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(uart_num, UART_RX_BUF, 0, 0, NULL, 0),
                        TAG, "uart_driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(uart_num, &cfg),
                        TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(uart_num, tx_pin, rx_pin,
                                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");

    s_uart_num = uart_num;

    /* Scan common baud rates for a hit before assuming CONFIG_GPS_BAUD is
     * right — it wasn't, on this exact board, for this exact reason. Bounded
     * (~6.6s worst case, typically much faster on the first correct hit —
     * see BAUD_SCAN_WINDOW_MS), never blocks boot indefinitely. */
    s_detected_baud = gps_autobaud_scan(uart_num);
    s_baud_confirmed = (s_detected_baud != 0);
    if (!s_baud_confirmed) {
        s_detected_baud = (uint32_t)baud;   /* nothing hit — fall back to the configured default */
        uart_set_baudrate(uart_num, s_detected_baud);
    }

    /* Configure the u-blox module: switch to high baud + rate, enable NAV-PVT.
     * Best-effort — on failure the NMEA fallback parser keeps a fix flowing.
     * Starts from whatever baud the scan above actually confirmed works. */
    gps_configure_ublox(uart_num);

    BaseType_t ok = xTaskCreate(gps_task, "GPS_Task", GPS_TASK_STACK,
                                 NULL, GPS_TASK_PRIORITY, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "GPS init OK (UART%d RX=%d TX=%d, %d->%d baud, %d Hz NAV-PVT)",
             uart_num, rx_pin, tx_pin, baud, CONFIG_GPS_HIGH_BAUD, CONFIG_GPS_MEAS_RATE_HZ);
    return ESP_OK;
}

esp_err_t gps_driver_get_fix(gps_fix_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;

    if (!s_mutex) {
        memset(out, 0, sizeof(*out));
        return ESP_OK;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_fix;
    xSemaphoreGive(s_mutex);

    /* Force invalid + clear stale-looking fields if the fix is old (receiver
     * unplugged / lost reception) — otherwise a dead link still shows the last
     * satellite count/quality frozen, which reads as "still receiving". */
    int64_t now = esp_timer_get_time();
    if (out->last_update_us == 0 ||
        (now - out->last_update_us) > GPS_STALE_US) {
        out->valid       = false;
        out->satellites  = 0;
        out->fix_quality = 0;
        out->hdop        = 0.0f;
    }
    return ESP_OK;
}

bool gps_driver_has_lock(void) {
    gps_fix_t f;
    gps_driver_get_fix(&f);   /* copies + applies staleness */
    return f.valid && f.satellites >= GPS_LOCK_MIN_SATS;
}

bool gps_driver_is_alive(void) {
    if (!s_mutex) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int64_t last = s_last_rx_us;
    xSemaphoreGive(s_mutex);
    if (last == 0) return false;
    return (esp_timer_get_time() - last) <= GPS_STALE_US;
}

/* baud: the value in effect (either a confirmed scan hit or the fallback).
 * confirmed (may be NULL): true only if the scan actually got a checksum-valid
 * NMEA hit at that rate — false means it's just the configured fallback guess,
 * not a verified fact, and callers should present it as such. */
uint32_t gps_driver_get_detected_baud(bool *confirmed) {
    if (confirmed) *confirmed = s_baud_confirmed;
    return s_detected_baud;
}
