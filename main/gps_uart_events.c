#include "gps_uart_events.h"

void gps_uart_event_handle(gps_runtime_status_t *status,
                           gps_uart_event_kind_t kind, size_t available,
                           const gps_uart_event_ops_t *ops, void *context)
{
    if (!status || !ops) return;

    if (kind == GPS_UART_EVENT_DATA) {
        if (available && ops->read_available) {
            ops->read_available(context, available);
        }
        return;
    }

    if (kind != GPS_UART_EVENT_FIFO_OVERFLOW &&
        kind != GPS_UART_EVENT_BUFFER_FULL) {
        return;
    }

    if (kind == GPS_UART_EVENT_FIFO_OVERFLOW) {
        ++status->uart_fifo_overflows;
    } else {
        ++status->uart_buffer_full_events;
    }
    if (ops->flush_input) ops->flush_input(context);
    if (ops->reset_queue) ops->reset_queue(context);
    if (ops->reset_parser) ops->reset_parser(context);
}
