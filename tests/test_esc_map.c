#include <assert.h>

#include "esc_map.h"

int main(void)
{
    /* A regression in the off clamp would make a unidirectional ESC run. */
    assert(esc_throttle_to_us(0.0f, 1000, 2000, 0.20f) == 1000);
    assert(esc_throttle_to_us(-0.5f, 1000, 2000, 0.20f) == 1000);

    /* A regression in the top clamp must not exceed the calibrated full pulse. */
    assert(esc_throttle_to_us(1.0f, 1000, 2000, 0.20f) == 2000);
    assert(esc_throttle_to_us(5.0f, 1000, 2000, 0.20f) == 2000);

    /* A nonzero command must skip the motor-specific start dead-band. */
    unsigned tiny = esc_throttle_to_us(0.001f, 1000, 2000, 0.20f);
    assert(tiny >= 1195 && tiny <= 1205);

    /* floor + command * remaining span: .20 + .50 * .80 = .60. */
    assert(esc_throttle_to_us(0.5f, 1000, 2000, 0.20f) == 1600);
    return 0;
}
