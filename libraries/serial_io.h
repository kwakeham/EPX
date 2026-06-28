/**
 * @file serial_io.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Minimal UART byte-output hook. Implemented in ble_cus.c (which owns the
 *        app_uart instance); used by the console and telemetry so they don't need
 *        the full BLE header. RX is interrupt-driven (uart_event_handle).
 */

#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#include <stdint.h>

/** Blocking-ish write of len bytes to the UART (COM5). Safe before init (no-op). */
void serial_write(const uint8_t *data, uint16_t len);

#endif // SERIAL_IO_H
