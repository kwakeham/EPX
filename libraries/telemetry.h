/**
 * @file telemetry.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief CSV time-series telemetry over a dedicated SEGGER RTT up-channel.
 *
 * Logs stay on RTT channel 0; telemetry is emitted on channel 1 so a capture of
 * channel 1 is a clean, directly-plottable CSV. Values are scaled to integers
 * (centi-degrees) to avoid pulling in float printf support.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>

/** Configure the dedicated RTT up-channel. Call once at startup. */
void telemetry_init(void);

/** Enable/disable streaming. Enabling resets the time base and prints a header. */
void telemetry_set_enabled(bool enabled);

bool telemetry_is_enabled(void);

/** Emit one CSV row. No-op when disabled. Call once per control tick. */
void telemetry_tick(float target, float current, float drive, float integral, int state);

#endif // TELEMETRY_H
