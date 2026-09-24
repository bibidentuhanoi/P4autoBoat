#pragma once

/* Frozen-IMU detector.  A live ICM20948 never returns the same six
 * accel/gyro values twice in a row for long: its noise alone moves them by
 * tens of counts every read.  An asleep chip, or one left on the wrong
 * register bank, still ACKs every read and returns the same bytes forever --
 * zeros when asleep (hw 2026-09-24: "accel NOT changing at boot (0,0,0)"),
 * so pitch/roll froze and the gyro read a constant yaw rate.  Pure C, no
 * ESP-IDF, so it is host-tested (tests/test_imu_freeze.c). */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    int16_t last[6];
    uint16_t same;     /* consecutive samples equal to `last` */
    bool have;
    bool frozen;
} imu_freeze_t;

typedef enum {
    IMU_FREEZE_LIVE = 0,
    IMU_FREEZE_JUST_FROZE,      /* crossed the limit on this sample */
    IMU_FREEZE_STILL_FROZEN,
    IMU_FREEZE_JUST_RECOVERED,  /* values moved again after being frozen */
} imu_freeze_event_t;

/* Feed one valid accel/gyro read (ax, ay, az, gx, gy, gz).  `limit` is how
 * many identical samples in a row count as frozen. */
static inline imu_freeze_event_t imu_freeze_update(imu_freeze_t *f, const int16_t v[6],
                                                   uint16_t limit)
{
    bool equal = f->have && memcmp(f->last, v, sizeof(f->last)) == 0;
    memcpy(f->last, v, sizeof(f->last));
    f->have = true;
    if (!equal) {
        f->same = 0;
        if (f->frozen) {
            f->frozen = false;
            return IMU_FREEZE_JUST_RECOVERED;
        }
        return IMU_FREEZE_LIVE;
    }
    if (f->same < UINT16_MAX) f->same++;
    if (f->frozen) return IMU_FREEZE_STILL_FROZEN;
    if (f->same >= limit) {
        f->frozen = true;
        return IMU_FREEZE_JUST_FROZE;
    }
    return IMU_FREEZE_LIVE;
}
