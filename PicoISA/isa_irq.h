#ifndef ISA_IRQ_H
#define ISA_IRQ_H

#include <stdbool.h>

#include "isa.h"

// Maximum number of devices that can share the single ISA IRQ line routed to the Pico.
#define ISA_IRQ_MAX_HANDLERS 8

// A registered handler inspects its own device's interrupt-identification register and
// services it if (and only if) it was the source. Return true if this device claimed and
// serviced the interrupt, false if it had nothing pending (so the dispatcher can ask the
// next registered device).
typedef bool (*isa_irq_handler_fn)(void *context);

// Configures the ISA IRQ GPIO pin (level-high, since ISA IRQ lines are driven high and held
// there by the card until the interrupt source is serviced) and installs the shared GPIO IRQ
// callback. Call once, after isa_gpio_init.
//
// Dispatch to registered handlers happens synchronously inside the GPIO ISR itself (see
// isa_irq.c) so pending data (e.g. a UART's 1-deep RHR with FIFO disabled) is drained within
// microseconds of the interrupt, even if the main loop is blocked in a long-running
// operation. Keep registered handlers fast for this reason -- they run in interrupt context.
void isa_irq_init(isa_controller_t *isa);

// Registers a device handler. Order matters only in that handlers are polled in registration
// order each pass; every handler is still given a chance every pass regardless of others'
// results, since multiple devices can share one line.
bool isa_irq_register(isa_irq_handler_fn handler, void *context);

#endif
