#include "isa_irq.h"

#include <stddef.h>

#include "hardware/gpio.h"

typedef struct {
    isa_irq_handler_fn handler;
    void *context;
} isa_irq_registration_t;

static isa_irq_registration_t s_handlers[ISA_IRQ_MAX_HANDLERS];
static size_t s_handler_count = 0;
static uint32_t s_irq_pin = 0;

// Defensive re-entrancy guard. All GPIO IRQs on RP2040 share a single NVIC line
// (IO_IRQ_BANK0), so this callback cannot genuinely be preempted by itself in practice --
// this only protects against an unexpected recursive call path (e.g. a handler that itself
// causes GPIO IRQ processing to run again before this pass completes).
static volatile bool s_dispatching = false;

static void isa_irq_gpio_callback(uint gpio, uint32_t events)
{
    if (gpio != s_irq_pin || s_dispatching) {
        return;
    }

    s_dispatching = true;

    // Service the interrupt synchronously, right here in interrupt context, instead of
    // deferring to the main loop. This guarantees pending data (e.g. a UART's RHR, only
    // 1 byte deep with FIFO disabled) is drained within microseconds of IRQ assertion, even
    // if the main loop is stuck in a long-running operation (a blocking printf/USB write,
    // etc). The trade-off: this callback holds off all other GPIO interrupts for its
    // duration, so registered handlers must stay fast (register reads and a short drain
    // loop -- no long busy-waits or blocking calls).
    while (gpio_get(s_irq_pin)) {
        bool any_handled = false;

        for (size_t i = 0; i < s_handler_count; ++i) {
            if (s_handlers[i].handler(s_handlers[i].context)) {
                any_handled = true;
            }
        }

        if (!any_handled) {
            // Nobody claimed it despite the line still reading high -- avoid spinning
            // forever on a stuck/unrelated line.
            break;
        }
    }

    s_dispatching = false;
}

void isa_irq_init(isa_controller_t *isa)
{
    s_irq_pin = isa->irq_pin;
    s_handler_count = 0;
    s_dispatching = false;

    gpio_set_irq_enabled_with_callback(s_irq_pin, GPIO_IRQ_LEVEL_HIGH, true, &isa_irq_gpio_callback);
}

bool isa_irq_register(isa_irq_handler_fn handler, void *context)
{
    if (handler == NULL || s_handler_count >= ISA_IRQ_MAX_HANDLERS) {
        return false;
    }

    s_handlers[s_handler_count].handler = handler;
    s_handlers[s_handler_count].context = context;
    s_handler_count++;
    return true;
}
