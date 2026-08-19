#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "isa.h"
#include "isa_irq.h"
#include "uart.h"
#include "vga.h"

#define TX_BUFFER_SIZE 128

typedef struct {
    uint8_t data[TX_BUFFER_SIZE];
    size_t length;
    size_t next_index;
} tx_buffer_t;

// Quiet period (no new bytes appended) after which the stdio queue auto-flushes to printf.
// Chosen well above the observed ~1 ms/character spacing at 9600 baud, but short enough
// to feel responsive once a response burst finishes.
#define STDIO_QUIET_MS 20
#define STDIO_BUFFER_SIZE 256

typedef struct {
    char data[STDIO_BUFFER_SIZE];
    size_t length;
    absolute_time_t last_update;
} stdio_queue_t;

static void stdio_queue_init(stdio_queue_t *stdio)
{
    stdio->length = 0;
    stdio->last_update = get_absolute_time();
}

static void stdio_queue_flush(stdio_queue_t *stdio)
{
    if (stdio->length == 0) {
        return;
    }

    stdio->data[stdio->length] = '\0';
    printf("%s", stdio->data);
    stdio->length = 0;
}

// Appends a formatted, null-terminated string to the stdio queue. Flushes first if the
// incoming string would overflow the buffer, so continuous/high-volume responses
// (e.g. AT$ help text) still get printed incrementally instead of being dropped or
// blocking on a full buffer.
static void stdio_queue_append(stdio_queue_t *stdio, const char *text)
{
    size_t len = strlen(text);

    if (stdio->length + len > STDIO_BUFFER_SIZE - 1) {
        stdio_queue_flush(stdio);
    }

    for (size_t i = 0; i < len && stdio->length < STDIO_BUFFER_SIZE - 1; ++i) {
        stdio->data[stdio->length++] = text[i];
    }
    stdio->last_update = get_absolute_time();
}

// Flushes the queue once no new bytes have arrived for STDIO_QUIET_MS, deferring the
// (blocking) printf call off the critical uart_try_read polling path.
static void stdio_queue_maybe_flush(stdio_queue_t *stdio)
{
    if (stdio->length == 0) {
        return;
    }

    if (time_reached(delayed_by_ms(stdio->last_update, STDIO_QUIET_MS))) {
        stdio_queue_flush(stdio);
    }
}

#define ISA_DATA_BASE_PIN 0
#define ISA_ADDR_BASE_PIN 8
 // Used for A8-A19
#define ISA_ADDR_SER_DATA_PIN 16
#define ISA_ADDR_SER_CLK_PIN 17

#define ISA_IOR_PIN 18
#define ISA_IOW_PIN 19
#define ISA_MEMR_PIN 20
#define ISA_MEMW_PIN 21
#define ISA_READY_PIN 22

#define ISA_DIR_PIN 27

#define ISA_IRQ_PIN 28

static void print_value_details(uint8_t value)
{
    char ascii = (value >= 32 && value <= 126) ? (char)value : ' ';

    // Hex, decimal, ASCII, binary — aligned columns
    printf("0x%02X   %3u   %c   ", value, value, ascii);

    for (int bit = 7; bit >= 0; --bit) {
        putchar((value & (1u << bit)) ? '1' : '0');
    }

    putchar('\n');
}

static void handle_command(isa_controller_t *isa, uart_controller_t *uart, vga_controller_t *vga, tx_buffer_t *tx_buffer, char *line)
{
    unsigned int address = 0;
    unsigned int value = 0;

    // ISA IO Write command: "o <address_hex> <value_hex>"
    if (strncmp(line, "o ", 2) == 0 && sscanf(line, "o %x %x", &address, &value) == 2) {
        if (address >  0xFFFFFu || value > 0xFFu) {
            printf("invalid io address/value\n");
            return;
        }

        isa_io_write(isa, address, (uint8_t)value);
        printf("o 0x%03X 0x%02X\n", address, value);
        return;
    }

    // ISA IO Read command: "i <address_hex>"
    if (strncmp(line, "i ", 2) == 0 && sscanf(line, "i %x", &address) == 1) {
        if (address >  0xFFFFFu) {
            printf("invalid io address\n");
            return;
        }

        uint8_t read_value = isa_io_read(isa, address);
        printf("i 0x%03X -> ", address);
        print_value_details(read_value);
        return;
    }

    // ISA Memory Write command: "e <address_hex> <value_hex>"
    if (strncmp(line, "e ", 2) == 0 && sscanf(line, "e %x %x", &address, &value) == 2) {
        if (address > 0xFFFFFu || value > 0xFFu) {
            printf("invalid memory address/value\n");
            return;
        }

        isa_mem_write(isa, address, (uint8_t)value);
        printf("e 0x%05X 0x%02X\n", address, value);
        return;
    }

    // ISA Memory Read command: "d <address_hex>"
    if (strncmp(line, "d ", 2) == 0 && sscanf(line, "d %x", &address) == 1) {
        if (address > 0xFFFFFu) {
            printf("invalid memory address\n");
            return;
        }

        uint8_t read_value = isa_mem_read(isa, address);
        printf("d 0x%05X -> ", address);
        print_value_details(read_value);
        return;
    }

    // ISA reset command: "reset"
    if (strcmp(line, "reset") == 0) {
        gpio_put(isa->ready_pin, 1);
        sleep_ms(100);
        gpio_put(isa->ready_pin, 0);
        printf("reset complete\n");
        return;
    }

    // UART Init command: "uart init"
    if (strcmp(line, "uart init") == 0) {
        uart_controller_init(uart, isa, 0x3F8u);
        printf("uart init complete. Available: %s\n", uart->available ? "yes" : "no");
        return;
    }

    // UART Output command: "out <string>"
    if (strncmp(line, "out ", 4) == 0) {
        const char *text = line + 4;
        size_t index = 0;
        tx_buffer->length = 0;
        tx_buffer->next_index = 0;

        while (text[index] != '\0' && tx_buffer->length < TX_BUFFER_SIZE) {
            tx_buffer->data[tx_buffer->length++] = (uint8_t)text[index++];
        }

        if (tx_buffer->length < TX_BUFFER_SIZE) {
            tx_buffer->data[tx_buffer->length++] = '\r';
        }

        if (tx_buffer->length < TX_BUFFER_SIZE) {
            tx_buffer->data[tx_buffer->length++] = '\n';
        }

        printf("out queued: %s\n", text);
        return;
    }

    // VGA Init command: "vga init"
    if (strcmp(line, "vga init") == 0) {
        vga_controller_init(vga, isa);
        printf("vga init complete. Available: %s\n", vga->available ? "yes" : "no");
        return;
    }

    // VGA Indexed Read command: "vi <addr_hex> <data_hex> <index_hex>"
    if (strncmp(line, "vi ", 3) == 0) {
        unsigned int index_addr = 0;
        unsigned int data_addr = 0;
        unsigned int index = 0;
        if (sscanf(line, "vi %x %x %x", &index_addr, &data_addr, &index) == 3) {
            uint8_t read_value = vga_read_reg(vga, index_addr, data_addr, index);
            printf("vi 0x%03X 0x%03X 0x%02X -> ", index_addr, data_addr, index);
            print_value_details(read_value);
            return;
        }
    }

    // VGA Indexed Write command: "vo <addr_hex> <data_hex> <index_hex> <value_hex>"
    if (strncmp(line, "vo ", 3) == 0) {
        unsigned int index_addr = 0;
        unsigned int data_addr = 0;
        unsigned int index = 0;
        unsigned int value = 0;
        if (sscanf(line, "vo %x %x %x %x", &index_addr, &data_addr, &index, &value) == 4) {
            vga_write_reg(vga, index_addr, data_addr, index, value);
            printf("vo 0x%03X 0x%03X 0x%02X 0x%02X\n", index_addr, data_addr, index, value);
            return;
        }
    }

    // Help command: "help"
    if (strcmp(line, "help") == 0) {
        printf("Commands:");
        printf("ISA: i <addr_hex>, o <addr_hex> <value_hex>, mem: d <addr_hex>, e <addr_hex> <value_hex>, reset\n");
        printf("UART: uart init, out <string>\n");
        printf("VGA: vga init, vi <addr_hex> <data_hex> <index_hex>, vo <addr_hex> <data_hex> <index_hex> <value_hex>\n");
        
        return;
    }

    if (line[0] != '\0') {
        printf("unknown command: %s\n", line);
    }
}

int main(void)
{
    isa_controller_t isa;
    uart_controller_t uart;
    vga_controller_t vga;
    tx_buffer_t tx_buffer = {0};
    stdio_queue_t stdio_queue;
    char uart_chunk[17];

    stdio_queue_init(&stdio_queue);

    stdio_init_all();
    isa_gpio_init(&isa,
                  ISA_DATA_BASE_PIN,
                  ISA_ADDR_BASE_PIN,
                  ISA_READY_PIN,
                  ISA_IRQ_PIN,
                  ISA_MEMW_PIN,
                  ISA_MEMR_PIN,
                  ISA_IOW_PIN,
                  ISA_IOR_PIN,
                  ISA_DIR_PIN,
                  ISA_ADDR_SER_DATA_PIN,
                  ISA_ADDR_SER_CLK_PIN);
    
    uart_controller_init(&uart, &isa, 0x3F8u);
    isa_irq_init(&isa);
    isa_irq_register(uart_handle_interrupt, &uart);

    vga_controller_init(&vga, &isa);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    char line[64];
    size_t line_index = 0;
    absolute_time_t next_blink = make_timeout_time_ms(1000);
    absolute_time_t next_uart = make_timeout_time_ms(1);
    bool prompted = false;


    while (true) 
    {
        // Welcome prompt
        if (!prompted && stdio_usb_connected())
        {
            printf("\nPicoISA ready. Type 'help' for commands.\n");
            printf("Uart available: %s\n", uart.available ? "yes" : "no");
            prompted = true;
        }

        if (time_reached(next_blink)) {
            gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
            next_blink = delayed_by_ms(next_blink, (prompted ? 1000 : 250));
        }

        int ch = getchar_timeout_us(0);

        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\r' || ch == '\n') {
                line[line_index] = '\0';
                if (line_index > 0) {
                    handle_command(&isa, &uart, &vga, &tx_buffer, line);
                }
                line_index = 0;
            } else if (line_index < sizeof(line) - 1) {
                line[line_index++] = (char)ch;
            }
        } 
        
        if (time_reached(next_uart))
        {         
            // Read in any available data from the uart module   
            size_t count = uart_try_read(&uart, uart_chunk, 16);
            if (count > 0) {
                uart_chunk[count] = '\0';
                char formatted[64 + sizeof(uart_chunk)];
                
                //snprintf(formatted, sizeof(formatted), "RX: %llu %u - %s\n", time_us_64(), (unsigned int)count, uart_chunk);
                stdio_queue_append(&stdio_queue, uart_chunk);
            }
        
            // Transmit data if available
            if (uart.available && tx_buffer.next_index < tx_buffer.length) 
            {
                if (uart_try_write(&uart, tx_buffer.data[tx_buffer.next_index])) 
                {
                    tx_buffer.next_index++;
                }            
            }
            next_uart = delayed_by_ms(next_uart, 1);
        }

        stdio_queue_maybe_flush(&stdio_queue);

        sleep_ms(10);
    }

}
