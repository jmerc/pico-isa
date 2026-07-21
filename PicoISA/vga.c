/*
#include "uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pico/stdlib.h"

#define UART_REG_DATA         0x00u
#define UART_REG_IER          0x01u
#define UART_REG_ISR          0x02u  // Read-only
#define UART_REG_FCR          0x02u  // Write-only
#define UART_REG_LCR          0x03u
#define UART_REG_MCR          0x04u
#define UART_REG_LSR          0x05u
#define UART_REG_MSR          0x06u
#define UART_REG_SCR          0x07u

#define UART_REG_DLL          0x00u
#define UART_REG_DLM          0x01u

#define UART_LSR_DATA_READY   0x01u
#define UART_LSR_THR_EMPTY    0x20u

#define UART_MSR_CTS          0x10u
#define UART_MSR_DSR          0x20u

#define UART_IER_RX_DATA_AVAILABLE   0x01u
#define UART_IER_THR_EMPTY           0x02u   // Reserved for a future TX queue/THR-empty interrupt.

// IIR bit 0 is inverted: 1 means no interrupt is pending on this UART.
#define UART_IIR_NO_INTERRUPT   0x01u
#define UART_IIR_ID_MASK        0x0Eu
#define UART_IIR_ID_THR_EMPTY   0x02u
#define UART_IIR_ID_RX_DATA     0x04u
#define UART_IIR_ID_RX_TIMEOUT  0x0Cu   // FIFO mode only; not expected with FIFO disabled.


static uint8_t uart_read_reg(uart_controller_t *uart, uint16_t reg)
{
    return isa_io_read(uart->isa, (uint16_t)(uart->base_addr + reg));
}

static void uart_write_reg(uart_controller_t *uart, uint16_t reg, uint8_t value)
{
    isa_io_write(uart->isa, (uint16_t)(uart->base_addr + reg), value);
}

// Pushes a byte into the software RX ring buffer, dropping the oldest unread byte to make
// room if the ring is full (favors keeping the newest data over silently ignoring bytes
// already accepted from the wire).
static void uart_rx_ring_push(uart_controller_t *uart, uint8_t byte)
{
    size_t next_head = (uart->rx_head + 1) % UART_RX_RING_SIZE;
    if (next_head == uart->rx_tail) {
        uart->rx_tail = (uart->rx_tail + 1) % UART_RX_RING_SIZE;
    }

    uart->rx_ring[uart->rx_head] = byte;
    uart->rx_head = next_head;
}

void uart_controller_init(uart_controller_t *uart, isa_controller_t *isa, uint16_t base_addr)
{
    uart->isa = isa;
    uart->base_addr = base_addr;
    uart->available = false;
    uart->rx_head = 0;
    uart->rx_tail = 0;

    uart_write_reg(uart, UART_REG_SCR, 0xA5u);    
    isa_io_write(uart->isa, 0, 0);  // Clears the ISA data lines if they're floating
    if (uart_read_reg(uart, UART_REG_SCR) != 0xA5u) {
        return;
    }

    uart_write_reg(uart, UART_REG_SCR, 0x5Au);
    isa_io_write(uart->isa, 0, 0);

    if (uart_read_reg(uart, UART_REG_SCR) != 0x5Au) {
        return;
    }

    uart->available = true;

    uart_write_reg(uart, UART_REG_LCR, 0x80u);   // DLAB=1
    uart_write_reg(uart, UART_REG_DLL, 0x0Cu);   // Baud @ 1.8432 MHz - 0x01: 115200, 0x0C: 9600, 0x30: 2400
    uart_write_reg(uart, UART_REG_DLM, 0x00u);   // divisor latch high byte

    uart_write_reg(uart, UART_REG_LCR, 0x03u);   // 8N1, DLAB=0

    // FIFO disabled for now: interrupts fire per-byte (IIR ID 0x04, "received data
    // available") instead of coalescing into 16550-style FIFO/character-timeout
    // behavior. Revisit once FIFO's earlier data-eviction issue is root-caused.
    uart_write_reg(uart, UART_REG_FCR, 0x00u);

    // Only the "receive data available" interrupt is enabled for now. THR-empty (bit 1)
    // is left disabled but documented above for a future TX queue that writes a byte at a
    // time as THR-empty interrupts arrive, instead of the current burst-write-and-poll
    // uart_try_write approach.
    uart_write_reg(uart, UART_REG_IER, UART_IER_RX_DATA_AVAILABLE);

    // OUT2 (bit 3) must be set for this UART to actually drive its IRQ output pin -- on
    // 8250/16450/16550-style parts OUT2 gates the interrupt output, so leaving it low would
    // enable interrupts internally but never assert the shared ISA IRQ line.
    uart_write_reg(uart, UART_REG_MCR, 0x0Fu);   // OUT1=1, OUT2=1, DTR=1, RTS=1
}

size_t uart_try_read(uart_controller_t *uart, char *buffer, size_t buffer_size)
{
    if (!uart->available || buffer == NULL || buffer_size == 0) {
        return 0;
    }

    size_t count = 0;
    while (count < buffer_size && uart->rx_tail != uart->rx_head) {
        buffer[count++] = (char)uart->rx_ring[uart->rx_tail];
        uart->rx_tail = (uart->rx_tail + 1) % UART_RX_RING_SIZE;
    }

    return count;
}

bool uart_try_write(uart_controller_t *uart, uint8_t value)
{
    if (!uart->available) {
        return false;
    }

    uint8_t lsr = uart_read_reg(uart, UART_REG_LSR);
    if ((lsr & UART_LSR_THR_EMPTY) == 0) {
        return false;
    }

    uart_write_reg(uart, UART_REG_DATA, value);
    return true;
}

bool uart_handle_interrupt(void *context)
{
    uart_controller_t *uart = (uart_controller_t *)context;
    if (uart == NULL || !uart->available) {
        return false;
    }

    uint8_t iir = uart_read_reg(uart, UART_REG_ISR);
    if ((iir & UART_IIR_NO_INTERRUPT) != 0) {
        // This UART is not the source of the currently asserted IRQ line.
        return false;
    }

    uint8_t identification = iir & UART_IIR_ID_MASK;
    if (identification == UART_IIR_ID_RX_DATA || identification == UART_IIR_ID_RX_TIMEOUT) {
        // Reading IIR does not clear an RX-data-available cause; draining RHR via LSR does.
        // Loop here since more than one byte may have arrived by the time we service this.
        while (uart_read_reg(uart, UART_REG_LSR) & UART_LSR_DATA_READY) {
            uart_rx_ring_push(uart, uart_read_reg(uart, UART_REG_DATA));
        }
        return true;
    }

    // Any other cause (THR empty, modem status, line status) is not yet handled here;
    // reading LSR/MSR as appropriate would be required to clear those once implemented.
    return false;
}
*/