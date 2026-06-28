/**
 * @file telemetry.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief CSV time-series telemetry over the UART (COM5).
 *
 * Logs stay on RTT (NRF_LOG); telemetry goes out the UART so a serial capture of
 * COM5 is a clean, directly-plottable CSV. Values are scaled to integers
 * (centi-degrees) to avoid pulling in float printf. Output is decimated so the
 * stream fits inside 115200 baud.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/** Initialise (resets state). Call once at startup. */
void telemetry_init(void);

/**
 * @brief Enable/disable streaming and set the decimation.
 * @param divider emit every Nth tick (1 = full 256 Hz, 4 ~= 64 Hz). 0 disables.
 *        Enabling prints a fresh CSV header.
 */
void telemetry_set_rate(uint16_t divider);

/** Convenience: enable at the default rate (divider 4) or disable. */
void telemetry_set_enabled(bool enabled);

bool telemetry_is_enabled(void);

/** Emit one CSV row (subject to decimation). Call once per control tick. */
void telemetry_tick(float target, float current, float drive, float integral,
                    int state, int16_t isense, bool fault);

#endif // TELEMETRY_H
