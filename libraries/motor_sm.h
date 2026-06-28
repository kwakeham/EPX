/**
 * @file motor_sm.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief SDK-free motor-driver sleep state machine.
 *
 * Replaces the old derailleur_moving / sleep_count flags. The motor is either
 * actively MOVING toward a target or HOLDING (driver asleep) at it. Transition
 * to HOLDING happens only once the position has *settled* inside the band for a
 * debounce window, not merely because drive strength fell low mid-travel.
 *
 * The step function is pure: it makes the decision and reports the side effects
 * to perform (driver sleep, position save, PID reset) so the caller wires them
 * to hardware. This lets the host SIL harness exercise it without the SDK.
 */

#ifndef MOTOR_SM_H
#define MOTOR_SM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { MOTOR_HOLDING, MOTOR_MOVING } motor_state_t;

typedef struct
{
    motor_state_t state;
    uint16_t      settle_count;   // consecutive ticks inside the band
    float         threshold;      // position band half-width (same units as angle)
    uint16_t      settle_ticks;   // ticks inside band required before HOLDING
} motor_sm_t;

typedef struct
{
    bool nsleep_changed;  // driver sleep line should change this tick
    bool nsleep_value;    // value to write when nsleep_changed (1 = awake)
    bool save_position;   // persist position to flash this tick
    bool pid_reset;       // controller should be reset this tick
} motor_action_t;

/**
 * @brief Initialise; starts MOVING so the motor homes to the loaded target.
 * @param threshold    position band half-width.
 * @param settle_ticks ticks inside band before sleeping the driver.
 */
void motor_sm_init(motor_sm_t *sm, float threshold, uint16_t settle_ticks);

/**
 * @brief Advance the state machine one tick.
 * @param act  out: side effects for the caller to apply.
 * @return true if the motor should be actively driven this tick (apply PID
 *         output); false if it should output zero (holding or just transitioned).
 */
bool motor_sm_step(motor_sm_t *sm, float target, float current, motor_action_t *act);

#endif // MOTOR_SM_H
