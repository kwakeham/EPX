/**
 * @file shift_seq.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief SDK-free overshift/dwell shift sequencer. See shift_seq.h.
 */

#include "shift_seq.h"

void shift_seq_init(shift_seq_t *s)
{
    s->phase         = SEQ_IDLE;
    s->final_pos     = 0;
    s->overshift_pos = 0;
    s->dwell_ticks   = 0;
    s->dwell_count   = 0;
}

void shift_seq_start(shift_seq_t *s, int32_t final_pos, int16_t signed_overshift,
                     uint16_t dwell_ticks, int32_t *out_target)
{
    s->final_pos   = final_pos;
    s->dwell_ticks = dwell_ticks;
    s->dwell_count = 0;

    if (signed_overshift != 0)
    {
        s->overshift_pos = final_pos + signed_overshift;
        s->phase         = SEQ_TO_OVERSHIFT;
        *out_target      = s->overshift_pos;
    }
    else
    {
        // No overtravel and no dwell: straight to the gear.
        s->overshift_pos = final_pos;
        s->phase         = SEQ_TO_FINAL;
        *out_target      = final_pos;
    }
}

bool shift_seq_step(shift_seq_t *s, bool arrived, int32_t *out_target)
{
    switch (s->phase)
    {
    case SEQ_TO_OVERSHIFT:
        if (arrived)
        {
            s->phase       = SEQ_DWELL;
            s->dwell_count = 0;
        }
        return false;

    case SEQ_DWELL:
        s->dwell_count++;
        if (s->dwell_count >= s->dwell_ticks)
        {
            s->phase    = SEQ_TO_FINAL;
            *out_target = s->final_pos;
            return true;
        }
        return false;

    case SEQ_TO_FINAL:
        if (arrived)
        {
            s->phase = SEQ_DONE;
        }
        return false;

    case SEQ_IDLE:
    case SEQ_DONE:
    default:
        return false;
    }
}

bool shift_seq_active(const shift_seq_t *s)
{
    return (s->phase != SEQ_IDLE) && (s->phase != SEQ_DONE);
}
