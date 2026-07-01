/**
 * @file console.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Interactive serial console over the UART (COM5). See console.h.
 *
 * Command input is interrupt-driven in ble_cus.c (uart_event_handle assembles a
 * line and calls data_handler_command). This module only prints the banner and
 * echoes replies, both over the UART via serial_write.
 */

#include "console.h"
#include "serial_io.h"
#include <string.h>

void console_init(void)
{
    static const char banner[] =
        "\r\nEPX console ready. Type 'h' for the command list.\r\n";
    serial_write((const uint8_t *)banner, sizeof(banner) - 1);
}

void console_print(const uint8_t *data, uint16_t len)
{
    serial_write(data, len);
    serial_write((const uint8_t *)"\r\n", 2);
}
