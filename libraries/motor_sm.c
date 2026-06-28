/**
 * @file motor_sm.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief SDK-free motor-driver sleep state machine. See motor_sm.h.
 */

#include "motor_sm.h"
#include <math.h>

void motor_sm_init(motor_sm_t *sm, float threshold, uint16_t settle_ticks)
{
    sm->state        = MOTOR_MOVING;   // home to the loaded target on boot
    sm->settle_count = 0;
    sm->threshold    = threshold;
    sm->settle_ticks = settle_ticks;
}

bool motor_sm_step(motor_sm_t *sm, float target, float current, motor_action_t *act)
{
    act->nsleep_changed = false;
    act->nsleep_value   = false;
    act->save_position  = false;
    act->pid_reset      = false;

    bool in_band = fabsf(target - current) < sm->threshold;

    if (sm->state == MOTOR_MOVING)
    {
        if (in_band)
        {
            sm->settle_count++;
            if (sm->settle_count >= sm->settle_ticks)
            {
                // Settled at target: stop, sleep the driver, persist position.
                sm->state           = MOTOR_HOLDING;
                sm->settle_count    = 0;
                act->nsleep_changed = true;
                act->nsleep_value   = false;  // driver asleep
                act->save_position  = true;
                return false;                 // output zero on the stopping tick
            }
        }
        else
        {
            sm->settle_count = 0;
        }
        return true;  // keep driving
    }

    // MOTOR_HOLDING
    if (!in_band)
    {
        // Pushed off target (new gear command or external force): wake and move.
        sm->state           = MOTOR_MOVING;
        sm->settle_count    = 0;
        act->nsleep_changed = true;
        act->nsleep_value   = true;   // driver awake
        act->pid_reset      = true;   // fresh integral/derivative for this move
    }
    return false;  // output zero this tick; PID drives from the next tick
}
