/**
 * @file console.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Interactive serial console over the UART (COM5).
 *
 * Command input is interrupt-driven (uart_event_handle in ble_cus.c assembles a
 * line and feeds the data_handler parser, same grammar as BLE NUS). This module
 * prints the banner and echoes replies over the UART.
 */

#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

/** Print a startup banner on the UART console. */
void console_init(void);

/** Echo a reply line to the UART console (used by nus_data_send). */
void console_print(const uint8_t *data, uint16_t len);

#endif // CONSOLE_H
