#ifndef VGA_H
#define VGA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "isa.h"

typedef struct {
    isa_controller_t *isa;
    bool available;
} vga_controller_t;

void vga_controller_init(vga_controller_t *vga, isa_controller_t *isa);

uint8_t vga_read_reg(vga_controller_t *vga, uint16_t index_addr, uint8_t index, uint16_t data_addr);
void vga_write_reg(vga_controller_t *vga, uint16_t index_addr, uint8_t index, uint16_t data_addr, uint8_t value);


#endif
