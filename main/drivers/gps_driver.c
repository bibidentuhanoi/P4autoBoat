#include "gps_driver.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdlib.h>
#include <string.h>

static const char *TAG = "GPS";

#define NMEA_MAX_LINE     100   /* NMEA 0183 caps at 82 chars incl CRLF; 100 is safe. */
#define UART_RX_BUF       1024
#define GPS_TASK_STACK    4096
#define GPS_TASK_PRIORITY 3

static int              s_uart_num = -1;
static SemaphoreHandle_t s_mutex   = NULL;
static TaskHandle_t     s_task     = NULL;
static gps_fix_t        s_fix      = {0};

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

/* ---------- Sentence parsers (caller holds s_mutex). ---------- */

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
        /* GGA alone is enough to mark valid; RMC may override. */
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

    /* Combine date + time into UTC epoch. */
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

/* Dispatch a validated, null-terminated payload (no '$', no '*cs'). */
static void dispatch_sentence(char *body) {
    /* Talker is 2 chars (GP/GN/GL/GA/BD/QZ...), sentence id is next 3. */
    if (strlen(body) < 5) return;
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

/* ---------- UART task ---------- */

static void gps_task(void *arg) {
    uint8_t rx_chunk[128];
    char    line[NMEA_MAX_LINE];
    size_t  line_len = 0;

    ESP_LOGI(TAG, "GPS task started on UART%d", s_uart_num);

    while (true) {
        int n = uart_read_bytes(s_uart_num, rx_chunk, sizeof(rx_chunk),
                                pdMS_TO_TICKS(200));
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            char c = (char)rx_chunk[i];
            if (c == '\r' || c == '\n') {
                if (line_len > 0) {
                    line[line_len] = 0;
                    char *body = nmea_validate(line, line_len);
                    if (body) dispatch_sentence(body);
                    line_len = 0;
                }
            } else if (line_len < sizeof(line) - 1) {
                line[line_len++] = c;
            } else {
                /* overflow — reset accumulator */
                line_len = 0;
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
        .baud_rate           = baud,
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

    BaseType_t ok = xTaskCreate(gps_task, "GPS_Task", GPS_TASK_STACK,
                                 NULL, GPS_TASK_PRIORITY, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "GPS init OK (UART%d RX=%d TX=%d @ %d baud)",
             uart_num, rx_pin, tx_pin, baud);
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

    /* Force invalid if fix is stale (receiver unplugged / lost reception). */
    int64_t now = esp_timer_get_time();
    if (out->last_update_us == 0 ||
        (now - out->last_update_us) > GPS_STALE_US) {
        out->valid = false;
    }
    return ESP_OK;
}
