#include "isa.h"

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

static void isa_set_address(isa_controller_t *isa, uint32_t address)
{
    gpio_put_masked(isa->addr_mask, ((uint32_t)(address & 0xFFu)) << isa->addr_base_pin);

    // The following section writes A8-A19 through a serial shift register. This is done to reduce 
    // the number of GPIO pins used on the Pico. Since this operation can take longer, track the last
    // address write to avoid unnecessary writes to the shift register if the upper address bits haven't changed.
    // Additional improvements are possible since the IO range is only 10 bits.

    // Strip the lower 8 bits to detect if the upper bits have changed since the last address set
    address &= 0xffff00u;
    
    if (address == isa->last_high_address)
    {
        return;
    }

    isa->last_high_address = address;

    // drop lower 8 bits
    uint16_t shifted_high_address = (address >> 8) & 0xFFFFu;

    // Shift out the high address bits to the serial pins
    for (int bit = (19 - 8); bit >= (8 - 8); --bit) {
        gpio_put(isa->ser_data_pin, (shifted_high_address >> bit) & 0x1u);
        gpio_put(isa->ser_clk_pin, 1);
        gpio_put(isa->ser_clk_pin, 0);
    }

    // Trigger serclk one more time to latch the output of the shift register to the output pins
    gpio_put(isa->ser_clk_pin, 1);
    gpio_put(isa->ser_clk_pin, 0);
}

static inline uint8_t isa_reverse_bits8(uint8_t value)
{
    // Swaps groups of bits reducing in size until we're complete
    value = (uint8_t)((value & 0xF0u) >> 4) | (uint8_t)((value & 0x0Fu) << 4);
    value = (uint8_t)((value & 0xCCu) >> 2) | (uint8_t)((value & 0x33u) << 2);
    value = (uint8_t)((value & 0xAAu) >> 1) | (uint8_t)((value & 0x55u) << 1);
    return value;
}

static void isa_set_data_out(isa_controller_t *isa, uint8_t value)
{
    value = isa_reverse_bits8(value);
    gpio_put_masked(isa->data_mask, ((uint32_t)value) << isa->data_base_pin);
}

static uint8_t isa_read_data_bus(isa_controller_t *isa)
{
    uint8_t value = (uint8_t)((sio_hw->gpio_in & isa->data_mask) >> isa->data_base_pin);
    return isa_reverse_bits8(value);
}

void isa_gpio_init(isa_controller_t *isa,
                   uint32_t data_base_pin,
                   uint32_t addr_base_pin,
                   uint32_t ready_pin,
                   uint32_t irq_pin,
                   uint32_t memw_pin,
                   uint32_t memr_pin,
                   uint32_t iow_pin,
                   uint32_t ior_pin,
                   uint32_t dir_pin,
                   uint32_t ser_data_pin,
                   uint32_t ser_clk_pin)
{
    isa->data_base_pin = data_base_pin;
    isa->addr_base_pin = addr_base_pin;
    isa->ready_pin = ready_pin;
    isa->irq_pin = irq_pin;
    isa->memw_pin = memw_pin;
    isa->memr_pin = memr_pin;
    isa->iow_pin = iow_pin;
    isa->ior_pin = ior_pin;
    isa->dir_pin = dir_pin;
    isa->ser_data_pin = ser_data_pin;
    isa->ser_clk_pin = ser_clk_pin;
    isa->last_high_address = 0xFFFFFFFFu;
    isa->data_mask = (0xFFu << isa->data_base_pin);
    isa->addr_mask = (0xFFu << isa->addr_base_pin);

    for (uint32_t pin = isa->data_base_pin; pin <= isa->data_base_pin + 7; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
    }

    for (uint32_t pin = isa->addr_base_pin; pin <= isa->addr_base_pin + 7; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    gpio_init(isa->ready_pin);
    gpio_put(isa->ready_pin, 1);
    gpio_set_dir(isa->ready_pin, GPIO_OUT);

    gpio_init(isa->irq_pin);
    gpio_disable_pulls(isa->irq_pin);
    gpio_set_dir(isa->irq_pin, GPIO_IN);

    gpio_init(isa->memw_pin);
    gpio_set_dir(isa->memw_pin, GPIO_OUT);
    gpio_put(isa->memw_pin, 1);

    gpio_init(isa->memr_pin);
    gpio_set_dir(isa->memr_pin, GPIO_OUT);
    gpio_put(isa->memr_pin, 1);

    gpio_init(isa->iow_pin);
    gpio_set_dir(isa->iow_pin, GPIO_OUT);
    gpio_put(isa->iow_pin, 1);

    gpio_init(isa->ior_pin);
    gpio_set_dir(isa->ior_pin, GPIO_OUT);
    gpio_put(isa->ior_pin, 1);

    gpio_init(isa->dir_pin);
    gpio_set_dir(isa->dir_pin, GPIO_OUT);
    gpio_put(isa->dir_pin, 1);

    gpio_init(isa->ser_data_pin);
    gpio_set_dir(isa->ser_data_pin, GPIO_OUT);
    gpio_put(isa->ser_data_pin, 0);

    gpio_init(isa->ser_clk_pin);
    gpio_set_dir(isa->ser_clk_pin, GPIO_OUT);
    gpio_put(isa->ser_clk_pin, 0);
    
    // Enable isa bus after a short delay to give the ISA devices time to reset and initialize
    sleep_ms(100);
    gpio_put(isa->ready_pin, 0);
    sleep_ms(100);
}

void isa_io_write(isa_controller_t *isa, uint32_t address, uint8_t value)
{
    // The full sequence below (address/direction setup, strobe assert/deassert, data hold)
    // is not atomic with respect to GPIO state. If the ISA IRQ fires mid-sequence and its
    // handler also performs an ISA bus transaction, the two would interleave on the same
    // physical pins and corrupt both. Disabling interrupts for this short (~1us) window
    // guarantees the transaction completes uninterrupted.
    uint32_t save = save_and_disable_interrupts();

    // Set direction: Pico -> ISA card
    gpio_put(isa->dir_pin, 0);

    // Set data bus to output
    for (uint32_t pin = isa->data_base_pin; pin <= isa->data_base_pin + 7; ++pin) {
        gpio_set_dir(pin, GPIO_OUT);
    }

    // Put address on bus
    isa_set_address(isa, address);

    // Put data on bus
    isa_set_data_out(isa, value);

    // Data setup time before IOW# low (>= 30 ns)
    busy_wait_at_least_cycles(8);   // ~64 ns

    // Assert IOW# Low
    gpio_put(isa->iow_pin, 0);

    // Hold IOW# low long enough for ISA card to latch data (>= 150–375 ns)
    //busy_wait_at_least_cycles(20);  // ~160 ns
    busy_wait_at_least_cycles(40);  // ~320 ns

    // Desassert IOW#
    gpio_put(isa->iow_pin, 1);

    // Data hold time after IOW# high (>= 30–50 ns)
    busy_wait_at_least_cycles(8);   // ~64 ns

    // Reset data bus to input
    isa_set_data_out(isa, 0);  // used to prevent floating values on next read
    for (uint32_t pin = isa->data_base_pin; pin <= isa->data_base_pin + 7; ++pin) {
        gpio_set_dir(pin, GPIO_IN);
    }

    // Set direction back to ISA card -> Pico
    gpio_put(isa->dir_pin, 1);

    restore_interrupts(save);
}

uint8_t isa_io_read(isa_controller_t *isa, uint32_t address)
{
    uint32_t save = save_and_disable_interrupts();

    // Put address on bus
    isa_set_address(isa, address);

    // Address setup time before IOR# low (>= 30–50 ns)
    busy_wait_at_least_cycles(4);   // ~32 ns

    // Assert IOR# low
    gpio_put(isa->ior_pin, 0);

    // Wait for ISA card to drive valid data (>= 50–125 ns)
    busy_wait_at_least_cycles(8);  // ~64 ns

    // Sample data
    uint8_t value = isa_read_data_bus(isa);

    // Deassert IOR# high (strobe width >= 150–375 ns)
    busy_wait_at_least_cycles(20);  // ~160 ns
    gpio_put(isa->ior_pin, 1);

    // Address hold time (>= 30–50 ns)
    busy_wait_at_least_cycles(4);   // ~32 ns

    restore_interrupts(save);
    return value;
}

void isa_mem_write(isa_controller_t *isa, uint32_t address, uint8_t value)
{
    uint32_t save = save_and_disable_interrupts();

    // Set direction: Pico -> ISA card
    gpio_put(isa->dir_pin, 0);
    // Set data bus to output
    for (uint32_t pin = isa->data_base_pin; pin <= isa->data_base_pin + 7; ++pin) {
        gpio_set_dir(pin, GPIO_OUT);
    }

    // Put address on bus
    isa_set_address(isa, address);

    // Put data on bus
    isa_set_data_out(isa, value);

    // Data setup time before MEMW# low (>= 30 ns)
    busy_wait_at_least_cycles(4);  // ~32 ns
    gpio_put(isa->memw_pin, 0);
    // Hold MEMW# low long enough for ISA card to latch data (>= 150–375 ns)
    busy_wait_at_least_cycles(20);  // ~160 ns
    gpio_put(isa->memw_pin, 1);
    // Data hold time after MEMW# high (>= 30–50 ns)
    busy_wait_at_least_cycles(4);  // ~32 ns

    // Reset data bus to input
    for (uint32_t pin = isa->data_base_pin; pin <= isa->data_base_pin + 7; ++pin) {
        gpio_set_dir(pin, GPIO_IN);
    }

    // Set direction back to ISA card -> Pico
    gpio_put(isa->dir_pin, 1);

    restore_interrupts(save);
}

uint8_t isa_mem_read(isa_controller_t *isa, uint32_t address)
{
    uint32_t save = save_and_disable_interrupts();
    
    // Put address on bus
    isa_set_address(isa, address);

    // Address setup time before MEMR# low (>= 30–50 ns)
    busy_wait_at_least_cycles(4);   // ~32 ns
    gpio_put(isa->memr_pin, 0);

    // Wait for ISA card to drive valid data (>= 50–125 ns)
    busy_wait_at_least_cycles(8);  // ~64 ns
    uint8_t value = isa_read_data_bus(isa);

    // Deassert MEMR# high (strobe width >= 150–375 ns)
    busy_wait_at_least_cycles(20);  // ~160 ns
    gpio_put(isa->memr_pin, 1);

    // Address hold time (>= 30–50 ns)
    busy_wait_at_least_cycles(4);   // ~32 ns

    restore_interrupts(save);
    return value;
}
