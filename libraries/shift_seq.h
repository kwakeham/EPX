/**
 * @file shift_seq.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief SDK-free overshift/dwell shift sequencer.
 *
 * A shift can overtravel past the target gear (to force the chain over), dwell
 * there briefly, then settle back to the gear's rest position. This sequences
 * the sub-targets the position controller chases. overshift == 0 means a plain
 * single move to the gear with no dwell.
 *
 * Pure logic: it emits sub-targets and consumes an "arrived" signal, so it is
 * driven identically by the firmware and the host SIL.
 */

#ifndef SHIFT_SEQ_H
#define SHIFT_SEQ_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SEQ_IDLE,
    SEQ_TO_OVERSHIFT,
    SEQ_DWELL,
    SEQ_TO_FINAL,
    SEQ_DONE
} shift_phase_t;

typedef struct
{
    shift_phase_t phase;
    int32_t       final_pos;
    int32_t       overshift_pos;
    uint16_t      dwell_ticks;
    uint16_t      dwell_count;
} shift_seq_t;

/** Reset to IDLE (no sequence running). */
void shift_seq_init(shift_seq_t *s);

/**
 * @brief Begin a shift to final_pos.
 * @param signed_overshift overtravel in the shift direction; 0 => go straight to
 *        final with no overshift and no dwell.
 * @param dwell_ticks ticks to dwell at the overshift waypoint.
 * @param out_target receives the first sub-target to drive to.
 */
void shift_seq_start(shift_seq_t *s, int32_t final_pos, int16_t signed_overshift,
                     uint16_t dwell_ticks, int32_t *out_target);

/**
 * @brief Advance one tick.
 * @param arrived true when the controller has reached the current sub-target.
 * @param out_target receives the new sub-target when the return is true.
 * @return true if the sub-target changed this tick.
 */
bool shift_seq_step(shift_seq_t *s, bool arrived, int32_t *out_target);

/** True while a sequence is running (not IDLE and not DONE). */
bool shift_seq_active(const shift_seq_t *s);

#endif // SHIFT_SEQ_H
