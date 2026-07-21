#ifndef ISA_H
#define ISA_H

#include <stdint.h>

typedef struct {
    uint32_t data_mask;
    uint32_t addr_mask;
    uint32_t data_base_pin;
    uint32_t addr_base_pin;
    uint32_t ready_pin;
    uint32_t irq_pin;
    uint32_t memw_pin;
    uint32_t memr_pin;
    uint32_t iow_pin;
    uint32_t ior_pin;
    uint32_t dir_pin;
    uint32_t ser_data_pin;
    uint32_t ser_clk_pin;
    uint32_t last_high_address;
} isa_controller_t;

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
                   uint32_t ser_clk_pin);

void isa_io_write(isa_controller_t *isa, uint32_t address, uint8_t value);
uint8_t isa_io_read(isa_controller_t *isa, uint32_t address);
void isa_mem_write(isa_controller_t *isa, uint32_t address, uint8_t value);
uint8_t isa_mem_read(isa_controller_t *isa, uint32_t address);

#endif
