/**
 * @file PID_controller.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Reusable, SDK-free PID controller. See PID_controller.h.
 * @version 0.2
 * @date 2023-08-17
 *
 * @copyright Copyright (c) 2023
 */

#include "PID_controller.h"

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

void pid_init(pid_ctrl_t *pid, const float *kp, const float *ki, const float *kd,
              float out_min, float out_max, float i_min, float i_max)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->primed           = false;
    pid->kp               = kp;
    pid->ki               = ki;
    pid->kd               = kd;
    pid->out_min          = out_min;
    pid->out_max          = out_max;
    pid->i_min            = i_min;
    pid->i_max            = i_max;
}

void pid_reset(pid_ctrl_t *pid, float measurement)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = measurement;
    pid->primed           = true;
}

float pid_update(pid_ctrl_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // Derivative on measurement (rate of change of the plant), not on error, so
    // a step change in setpoint does not produce a derivative kick.
    float deriv = 0.0f;
    if (pid->primed)
    {
        deriv = (measurement - pid->prev_measurement) / dt;
    }
    pid->prev_measurement = measurement;
    pid->primed           = true;

    // Output from the integral as it stands going into this step.
    float output = (*pid->kp) * error
                 + (*pid->ki) * pid->integral
                 - (*pid->kd) * deriv;

    // Anti-windup via conditional integration: only accumulate when we are not
    // saturated, or when the error would pull the output back out of saturation.
    bool above = (output >= pid->out_max);
    bool below = (output <= pid->out_min);
    if ((!above && !below) ||
        (above && error < 0.0f) ||
        (below && error > 0.0f))
    {
        pid->integral += error * dt;
        pid->integral  = clampf(pid->integral, pid->i_min, pid->i_max);
    }

    return clampf(output, pid->out_min, pid->out_max);
}
