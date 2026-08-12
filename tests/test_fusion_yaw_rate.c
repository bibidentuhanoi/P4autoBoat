#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common.h"
#include "imu_sample.h"
#include "sensor_fusion.h"

/* Feed one settled sample, then a second sample with a known gyro-Z; assert the
 * published yaw_rate reflects gz / 131.0 and sequence advances. */
int main(void)
{
    static CalibrationData calib = { .m_scale = {1, 1, 1} };
    fusion_init(&calib);

    imu_sample_t s = {0};
    s.accel_gyro_valid = true;
    s.az = 1000;            /* ~1g down so tilt math is finite */
    s.gz = 1310;            /* 1310 / 131.0 = 10.0 deg/s */
    s.captured_us = 1000000;
    s.sequence = 1;
    fusion_update_sample(&s);

    s.captured_us = 1020000; /* +20 ms */
    s.sequence = 2;
    fusion_update_sample(&s);

    FusionResult r;
    fusion_get_result(&r);
    assert(r.sequence == 2);
    assert(fabsf(r.yaw_rate - 10.0f) < 0.5f);
    return 0;
}
