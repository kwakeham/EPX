/**
 * @file evt_log.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Tagged lifecycle-event output for hardware-in-the-loop testing.
 *
 * Separate from the per-tick CSV telemetry (telemetry.c): this emits discrete,
 * machine-parseable "#tag,key=value" lines at boot / calibration / flash-save /
 * turn-crossing / fault moments. Lines go out the same UART the bench harness
 * reads, prefixed with '#' so they are trivially separable from CSV rows (which
 * lead with a digit) and from human-readable command replies.
 *
 * Each event is gated by a category bit, so instrumentation can stay compiled in
 * and is silenced by masking the category off (console 'e' command). The mask is
 * RAM-only; it resets to EVT_LOG_DEFAULT_MASK on boot.
 */

#ifndef EVT_LOG_H
#define EVT_LOG_H

#include <stdint.h>

/** Event categories. Keep in sync with the 'e' command legend in data_handler.c. */
typedef enum {
    EVT_BOOT  = 1u << 0,  // boot position reconstruction (fires once, first tick)
    EVT_CAL   = 1u << 1,  // guided calibration FSM steps
    EVT_SAVE  = 1u << 2,  // flash persistence (config / position records)
    EVT_TURN  = 1u << 3,  // multi-turn boundary crossings
    EVT_FAULT = 1u << 4,  // overcurrent / driver fault latch
    EVT_SHIFT = 1u << 5,  // shift commit
} evt_cat_t;

/* Categories enabled at startup. All on while this is a HIL/bench build so the
 * one-shot boot event (which fires before any console command can set the mask)
 * is captured by default. Set to 0 for a quiet production default. */
#define EVT_LOG_DEFAULT_MASK 0x3Fu

/** Reset the category mask to EVT_LOG_DEFAULT_MASK. Call once at startup. */
void evt_log_init(void);

/** Enable/disable categories (bitmask of evt_cat_t). */
void evt_log_set_mask(uint32_t mask);

/** Current category mask. */
uint32_t evt_log_get_mask(void);

/**
 * @brief Emit one tagged event line over the UART, iff (cat & mask) is set.
 * @param cat one evt_cat_t bit gating this line.
 * @param fmt printf-style format for the line body (no trailing newline; one is
 *            appended). Pass the full "#tag,..." text.
 */
void evt_log(uint32_t cat, const char *fmt, ...);

#endif // EVT_LOG_H
