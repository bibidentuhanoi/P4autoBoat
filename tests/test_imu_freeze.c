#include <assert.h>
#include <stdio.h>
#include "imu_freeze.h"

int main(void)
{
    imu_freeze_t f = {0};
    int16_t live[6] = {100, -50, 16300, 12, -40, 7};
    /* noisy live data never freezes */
    for (int i = 0; i < 1000; i++) {
        live[0] = (int16_t)(100 + (i % 7)); live[5] = (int16_t)(7 - (i % 3));
        assert(imu_freeze_update(&f, live, 25) == IMU_FREEZE_LIVE);
    }
    /* the asleep chip: zeros forever -> frozen after 25 identical, not before */
    int16_t zero[6] = {0};
    assert(imu_freeze_update(&f, zero, 25) == IMU_FREEZE_LIVE);      /* first zero: differs */
    for (int i = 1; i < 25; i++) assert(imu_freeze_update(&f, zero, 25) == IMU_FREEZE_LIVE);
    assert(imu_freeze_update(&f, zero, 25) == IMU_FREEZE_JUST_FROZE);
    assert(f.frozen);
    for (int i = 0; i < 100; i++) assert(imu_freeze_update(&f, zero, 25) == IMU_FREEZE_STILL_FROZEN);
    /* woke up again */
    assert(imu_freeze_update(&f, live, 25) == IMU_FREEZE_JUST_RECOVERED);
    assert(!f.frozen);
    assert(imu_freeze_update(&f, zero, 25) == IMU_FREEZE_LIVE);
    /* a stuck but non-zero value (wrong register bank) is caught the same way */
    imu_freeze_t g = {0};
    int16_t junk[6] = {257, 257, 1, 0, 3, 3};
    int froze_at = -1;
    for (int i = 0; i < 40; i++) if (imu_freeze_update(&g, junk, 25) == IMU_FREEZE_JUST_FROZE) froze_at = i;
    assert(froze_at == 25);
    printf("imu_freeze: all tests passed\n");
    return 0;
}
