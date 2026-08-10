#include <assert.h>
#include <stddef.h>

#include "gps_uart_events.h"

typedef struct {
    size_t bytes_requested;
    unsigned data_calls;
    unsigned flush_calls;
    unsigned queue_reset_calls;
    unsigned parser_reset_calls;
} observed_t;

static void read_available(void *context, size_t available)
{
    observed_t *observed = context;
    observed->bytes_requested += available;
    ++observed->data_calls;
}

static void flush_input(void *context)
{
    ++((observed_t *)context)->flush_calls;
}

static void reset_queue(void *context)
{
    ++((observed_t *)context)->queue_reset_calls;
}

static void reset_parser(void *context)
{
    ++((observed_t *)context)->parser_reset_calls;
}

int main(void)
{
    gps_runtime_status_t status = {0};
    observed_t observed = {0};
    const gps_uart_event_ops_t ops = {
        .read_available = read_available,
        .flush_input = flush_input,
        .reset_queue = reset_queue,
        .reset_parser = reset_parser,
    };

    gps_uart_event_handle(&status, GPS_UART_EVENT_DATA, 513, &ops, &observed);
    assert(observed.data_calls == 1);
    assert(observed.bytes_requested == 513);
    assert(observed.flush_calls == 0);

    gps_uart_event_handle(&status, GPS_UART_EVENT_FIFO_OVERFLOW, 0, &ops, &observed);
    assert(status.uart_fifo_overflows == 1);
    assert(status.uart_buffer_full_events == 0);
    assert(observed.flush_calls == 1);
    assert(observed.queue_reset_calls == 1);
    assert(observed.parser_reset_calls == 1);

    gps_uart_event_handle(&status, GPS_UART_EVENT_BUFFER_FULL, 0, &ops, &observed);
    assert(status.uart_fifo_overflows == 1);
    assert(status.uart_buffer_full_events == 1);
    assert(observed.flush_calls == 2);
    assert(observed.queue_reset_calls == 2);
    assert(observed.parser_reset_calls == 2);

    gps_uart_event_handle(&status, GPS_UART_EVENT_OTHER, 99, &ops, &observed);
    assert(observed.data_calls == 1);
    assert(observed.flush_calls == 2);
    return 0;
}
