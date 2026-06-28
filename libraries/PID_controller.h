/**
 * @file PID_controller.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Reusable, SDK-free PID controller.
 *
 * The controller keeps its state in a caller-owned struct so it can be reset
 * between moves and instantiated multiple times (e.g. the host SIL harness).
 * It reads its gains through live pointers, so updating the stored Kp/Ki/Kd
 * (via the command parser) takes effect on the next update with no relinking.
 *
 * @version 0.2
 * @date 2023-08-17
 *
 * @copyright Copyright (c) 2023
 */

#ifndef PID_CONTROLLER__
#define PID_CONTROLLER__

#include <stdbool.h>

typedef struct
{
    float integral;          // accumulated error * dt
    float prev_measurement;  // for derivative-on-measurement
    bool  primed;            // false until the first update seeds prev_measurement

    const float *kp;         // live gains (point at the stored configuration)
    const float *ki;
    const float *kd;

    float out_min, out_max;  // output (drive) clamp
    float i_min, i_max;      // integral clamp (back-stop for the anti-windup logic)
} pid_ctrl_t;               // not "pid_t": that collides with POSIX <sys/types.h>

/**
 * @brief Bind a PID instance to its (live) gains and output/integral limits.
 *        Leaves the controller primed=false; call pid_reset() before first use.
 */
void pid_init(pid_ctrl_t *pid, const float *kp, const float *ki, const float *kd,
              float out_min, float out_max, float i_min, float i_max);

/**
 * @brief Clear accumulated state. Call on every new move / motor wake so stale
 *        windup from the previous target cannot drive overshoot.
 * @param measurement current measured value, used to seed the derivative term.
 */
void pid_reset(pid_ctrl_t *pid, float measurement);

/**
 * @brief Run one control step.
 *
 * Derivative is taken on the measurement (no setpoint kick), the integral uses
 * conditional integration (anti-windup), and gains are scaled by dt so the same
 * gains behave identically at any loop rate.
 *
 * @param dt loop period in seconds.
 * @return control signal clamped to [out_min, out_max].
 */
float pid_update(pid_ctrl_t *pid, float setpoint, float measurement, float dt);

#endif
