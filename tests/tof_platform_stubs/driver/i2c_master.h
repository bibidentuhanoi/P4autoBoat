#pragma once

#include <stddef.h>
#include <stdint.h>

typedef void *i2c_master_dev_handle_t;

typedef struct {
    int unused;
} i2c_master_bus_config_t;

typedef struct {
    const uint8_t *write_buffer;
    size_t buffer_size;
} i2c_master_transmit_multi_buffer_info_t;

int i2c_master_multi_buffer_transmit(
    i2c_master_dev_handle_t handle,
    i2c_master_transmit_multi_buffer_info_t *buffers,
    size_t buffer_count,
    int timeout_ms);

int i2c_master_transmit_receive(i2c_master_dev_handle_t handle,
                                const uint8_t *write_buffer,
                                size_t write_size,
                                uint8_t *read_buffer,
                                size_t read_size,
                                int timeout_ms);
