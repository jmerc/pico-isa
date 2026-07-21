#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "isa.h"

// Software ring buffer depth for bytes drained out of the UART's RHR by the interrupt
// handler. This is independent of (and much larger than) the UART's own 1-byte holding
// register, since FIFO mode is disabled and interrupts must be serviced promptly.
#define UART_RX_RING_SIZE 64

typedef struct {
    isa_controller_t *isa;
    uint16_t base_addr;
    bool available;
    uint8_t rx_ring[UART_RX_RING_SIZE];
    size_t rx_head;
    size_t rx_tail;
} uart_controller_t;

void uart_controller_init(uart_controller_t *uart, isa_controller_t *isa, uint16_t base_addr);
size_t uart_try_read(uart_controller_t *uart, char *buffer, size_t buffer_size);
bool uart_try_write(uart_controller_t *uart, uint8_t value);

// Interrupt handler compatible with isa_irq_handler_fn (context must be a uart_controller_t*).
// Reads IIR to check whether this UART is the interrupt source; if so, drains all currently
// available bytes from RHR into the software ring buffer. Returns false if this UART had no
// interrupt pending, so isa_irq can move on to the next registered device on a shared line.
bool uart_handle_interrupt(void *context);

#endif
