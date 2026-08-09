#pragma once

#include <stdbool.h>
#include <stdint.h>

/* A single raw IMU acquisition.  The sequence is assigned by the acquisition
 * task and identifies the complete set of accel/gyro and magnetometer values. */
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t mx;
    int16_t my;
    int16_t mz;
    bool accel_gyro_valid;
    bool mag_valid;
    uint64_t captured_us;
    uint32_t sequence;
} imu_sample_t;
