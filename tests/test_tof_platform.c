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
static int s_timeouts[MAX_CALLS];
static int s_write_timeout;
static size_t s_write_bytes;

int i2c_master_transmit_receive(i2c_master_dev_handle_t handle,
                                const uint8_t *write_buffer,
                                size_t write_size,
                                uint8_t *read_buffer,
                                size_t read_size,
                                int timeout_ms)
{
    (void)handle;
    assert(write_size == 2U);
    assert(s_call_count < MAX_CALLS);
    s_timeouts[s_call_count] = timeout_ms;

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
    s_write_bytes = 0U;
    for (size_t i = 0; i < buffer_count; ++i) s_write_bytes += buffers[i].buffer_size;
    s_write_timeout = timeout_ms;
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

/* Every transfer is bounded. -1 (wait forever) is how a marginal bus wedged
 * the whole boot on hardware (2026-07-02: hung mid ToF-B bring-up; the IMU got
 * bounded timeouts then, this side did not). The bound must still fit the
 * longest legit transfer at the slowest bus this build may run: 3x its wire
 * time at 100 kHz, plus 100 ms. */
static int wire_ms_at_100k(size_t bytes) { return (int)((bytes + 4U) * 9U * 1000U / 100000U + 1U); }

static void test_every_transfer_is_bounded(void)
{
    VL53L5CX_Platform platform = {0};
    uint8_t out[150] = {0};

    reset_calls();
    assert(VL53L5CX_RdMulti(&platform, 0x1200U, out, sizeof(out)) == 0U);
    for (size_t i = 0; i < s_call_count; ++i) {
        assert(s_timeouts[i] > 0);                                   /* never "forever" */
        assert(s_timeouts[i] >= 100 + 3 * wire_ms_at_100k(s_sizes[i] + 2U));
        assert(s_timeouts[i] <= 1000);                               /* a read gives the bus back */
    }

    /* the firmware upload: one 32 KB write */
    static uint8_t firmware[0x8000];
    assert(VL53L5CX_WrMulti(&platform, 0x0000U, firmware, sizeof(firmware)) == 0U);
    assert(s_write_bytes == sizeof(firmware) + 2U);
    assert(s_write_timeout > 0);
    assert(s_write_timeout >= 100 + 3 * wire_ms_at_100k(s_write_bytes));
    assert(s_write_timeout <= 20000);

    /* a single register byte */
    uint8_t v = 0;
    assert(VL53L5CX_WrByte(&platform, 0x7FFFU, 0x02U) == 0U);
    assert(s_write_timeout > 0 && s_write_timeout <= 1000);
    reset_calls();
    assert(VL53L5CX_RdByte(&platform, 0x0000U, &v) == 0U);
    assert(s_timeouts[0] > 0 && s_timeouts[0] <= 1000);
}

int main(void)
{
    test_every_transfer_is_bounded();
    test_large_read_releases_bus_between_bounded_chunks();
    test_read_stops_at_first_i2c_error();
    test_empty_read_does_not_touch_bus();
    return 0;
}
