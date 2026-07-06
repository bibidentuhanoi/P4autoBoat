#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "esp_err.h"

/* Single user-LED (default GPIO3) used purely as a no-terminal calibration
 * guide: it signals when the boot-button window is open and which of the three
 * calibration phases is running, so the operator knows what to do without a
 * serial console. Dark whenever the boat is not calibrating. A small internal
 * task renders the blink pattern for the current state. */
typedef enum {
    STATUS_LED_OFF = 0,         /* dark — not calibrating */
    STATUS_LED_CAL_WINDOW,      /* fast blink — press BOOT now to force recalibration */
    STATUS_LED_CAL_STILL,       /* solid on — Phase 1: hold flat & still */
    STATUS_LED_CAL_MOVE,        /* triple-blip bursts — Phase 2: figure-8, keep moving */
    STATUS_LED_CAL_POINT,       /* slow blink — Phase 3: point at the bow & hold */
    STATUS_LED_CAL_DONE_OK,     /* three slow flashes then OFF — calibration good */
    STATUS_LED_CAL_DONE_FAIL,   /* rapid flutter then OFF — calibration poor, redo */
} status_led_state_t;

/* Configure the LED GPIO and start the pattern task. Idempotent. */
esp_err_t status_led_init(void);

/* Set the current pattern. Atomic; safe to call from any task/context. */
void status_led_set(status_led_state_t state);

#endif // STATUS_LED_H
