#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

/* Single user-LED (default GPIO3), a no-terminal guide: a heartbeat blink
 * from power-on until the system is running (the board is alive and
 * booting), the boot-button window, and which calibration step is running.
 * Dark once the boat is up and not calibrating. A small internal task
 * renders the blink pattern for the current state. */
typedef enum {
    STATUS_LED_OFF = 0,         /* dark — not calibrating */
    STATUS_LED_CAL_WINDOW,      /* fast blink — press BOOT now to force recalibration */
    STATUS_LED_CAL_STILL,       /* solid on — Phase 1: hold flat & still */
    STATUS_LED_CAL_MOVE,        /* triple-blip bursts — Phase 2: figure-8, keep moving */
    STATUS_LED_CAL_POINT,       /* slow blink — Phase 3: point at the bow & hold */
    STATUS_LED_CAL_DONE_OK,     /* three slow flashes then OFF — calibration good */
    STATUS_LED_CAL_DONE_FAIL,   /* rapid flutter then OFF — calibration poor, redo */
    STATUS_LED_BOOTING,         /* heartbeat blink — powered, alive, still booting */
} status_led_state_t;

/* Configure the LED GPIO and start the pattern task. Idempotent. */
esp_err_t status_led_init(void);

/* Set the current pattern. Atomic; safe to call from any task/context. */
void status_led_set(status_led_state_t state);

#endif // STATUS_LED_H
