/**
 * @file console.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Interactive serial console over SEGGER RTT (bidirectional, no pins).
 *
 * Reads newline-terminated commands from the RTT down-channel and feeds them to
 * the existing data_handler command parser (same grammar as BLE NUS). Command
 * replies are echoed back on the RTT terminal channel so the dev kit can be
 * driven entirely from a J-Link RTT terminal with no hardware setup.
 */

#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

/** Print a startup banner on the RTT terminal. */
void console_init(void);

/** Poll the RTT down-channel for input. Call from the main loop. */
void console_poll(void);

/** Echo a reply line to the RTT terminal (used by nus_data_send). */
void console_print(const uint8_t *data, uint16_t len);

#endif // CONSOLE_H
