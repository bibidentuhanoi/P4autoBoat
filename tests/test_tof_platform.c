#include "platform.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define EXPECTED_MAX_READ 64U
#define MAX_CALLS 8U

static size_t s_call_count;
static uint16_t s_addresses[MAX_CALLS];
static size_t s_sizes[MAX_CALLS];
static size_t s_fail_call;

int i2c_master_transmit_receive(i2c_master_dev_handle_t handle,
                                const uint8_t *write_buffer,
                                size_t write_size,
                                uint8_t *read_buffer,
                                size_t read_size,
                                int timeout_ms)
{
    (void)handle;
    (void)timeout_ms;
    assert(write_size == 2U);
    assert(s_call_count < MAX_CALLS);

    uint16_t address = ((uint16_t)write_buffer[0] << 8) | write_buffer[1];
    s_addresses[s_call_count] = address;
    s_sizes[s_call_count] = read_size;
    ++s_call_count;

    if (s_fail_call != 0U && s_call_count == s_fail_call) {
        return 7;
    }

    for (size_t i = 0; i < read_size; ++i) {
        read_buffer[i] = (uint8_t)(address + i);
    }
    return 0;
}

int i2c_master_multi_buffer_transmit(i2c_master_dev_handle_t handle,
                                     i2c_master_transmit_multi_buffer_info_t *buffers,
                                     size_t buffer_count,
                                     int timeout_ms)
{
    (void)handle;
    (void)buffers;
    (void)buffer_count;
    (void)timeout_ms;
    return 0;
}

int gpio_set_direction(gpio_num_t gpio, int mode)
{
    (void)gpio;
    (void)mode;
    return 0;
}

int gpio_set_level(gpio_num_t gpio, int level)
{
    (void)gpio;
    (void)level;
    return 0;
}

void vTaskDelay(uint32_t ticks)
{
    (void)ticks;
}

static void reset_calls(void)
{
    s_call_count = 0U;
    s_fail_call = 0U;
    memset(s_addresses, 0, sizeof(s_addresses));
    memset(s_sizes, 0, sizeof(s_sizes));
}

static void test_large_read_releases_bus_between_bounded_chunks(void)
{
    VL53L5CX_Platform platform = {0};
    uint8_t out[150] = {0};

    reset_calls();
    assert(VL53L5CX_RdMulti(&platform, 0x1200U, out, sizeof(out)) == 0U);

    assert(s_call_count == 3U);
    assert(s_addresses[0] == 0x1200U && s_sizes[0] == EXPECTED_MAX_READ);
    assert(s_addresses[1] == 0x1240U && s_sizes[1] == EXPECTED_MAX_READ);
    assert(s_addresses[2] == 0x1280U && s_sizes[2] == 22U);
    for (size_t i = 0; i < sizeof(out); ++i) {
        assert(out[i] == (uint8_t)(0x1200U + i));
    }
}

static void test_read_stops_at_first_i2c_error(void)
{
    VL53L5CX_Platform platform = {0};
    uint8_t out[150] = {0};

    reset_calls();
    s_fail_call = 2U;
    assert(VL53L5CX_RdMulti(&platform, 0x2000U, out, sizeof(out)) == 7U);
    assert(s_call_count == 2U);
}

static void test_empty_read_does_not_touch_bus(void)
{
    VL53L5CX_Platform platform = {0};
    uint8_t out = 0;

    reset_calls();
    assert(VL53L5CX_RdMulti(&platform, 0x3000U, &out, 0U) == 0U);
    assert(s_call_count == 0U);
}

int main(void)
{
    test_large_read_releases_bus_between_bounded_chunks();
    test_read_stops_at_first_i2c_error();
    test_empty_read_does_not_touch_bus();
    return 0;
}
