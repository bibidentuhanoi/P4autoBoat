#pragma once

#include <stddef.h>

#include "drivers/gps_driver.h"

typedef enum {
    GPS_UART_EVENT_DATA,
    GPS_UART_EVENT_FIFO_OVERFLOW,
    GPS_UART_EVENT_BUFFER_FULL,
    GPS_UART_EVENT_OTHER,
} gps_uart_event_kind_t;

typedef struct {
    void (*read_available)(void *context, size_t available);
    void (*flush_input)(void *context);
    void (*reset_queue)(void *context);
    void (*reset_parser)(void *context);
} gps_uart_event_ops_t;

void gps_uart_event_handle(gps_runtime_status_t *status,
                           gps_uart_event_kind_t kind, size_t available,
                           const gps_uart_event_ops_t *ops, void *context);
