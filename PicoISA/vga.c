
#include "vga.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pico/stdlib.h"

#define VGA_POST_ADDRESS 0x46E8u
#define VGA_BUS_ENABLE 0x102u

uint8_t vga_read_reg(vga_controller_t *vga, uint16_t index_addr, uint8_t index, uint16_t data_addr)
{
    isa_io_write(vga->isa, index_addr, index);
    return isa_io_read(vga->isa, data_addr);
}

void vga_write_reg(vga_controller_t *vga, uint16_t index_addr, uint8_t index, uint16_t data_addr, uint8_t value)
{
    isa_io_write(vga->isa, index_addr, index);
    isa_io_write(vga->isa, data_addr, value);
}


void vga_controller_init(vga_controller_t *vga, isa_controller_t *isa)
{
    vga->isa = isa;
    vga->available = false;

    // Initialization:
    // Enable VGA bus decoding
    isa_io_write(vga->isa, VGA_POST_ADDRESS, 0x10); // Enter setup mode
    isa_io_write(vga->isa, VGA_BUS_ENABLE, 0x01);
    //isa_io_write(vga->isa, VGA_BUS_ENABLE + 1, 0x00);
    isa_io_write(vga->isa, VGA_POST_ADDRESS, 0x07); // Enter operation mode




    vga->available = true;

}
