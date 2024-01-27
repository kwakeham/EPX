/**
 * @file PID_controller.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief 
 * @version 0.1
 * @date 2023-08-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "PID_controller.h"
#include "nrf_fstorage_sd.h"
// #include "nrf_fstorage.h" //no soft device
#include "nrf_log.h"
#include "app_error.h"
#include "nrf_delay.h"


// PID Controller parameters
float Kp = 10.0;   // Proportional gain
float Ki = 1.5;  // Integral gain
float Kd = 3;  // Derivative gain

//At 128hz kp = 9
//At 256hz (128 repeat) kp = 18
//AT 512hz kp = 35-40


// PID Controller state variables
float previousError = 0.0;
float integral = 0.0;

// Integral limits
float integralMax = 300.0;
float integralMin = -300.0;

// Control limits
float ControlMax = 400;
float ControlMin = -400;

// Dead band?
float DeadMax = 50;
float DeadMin = -50;

static epx_configuration_t *link_epx_values = NULL;

void pid_update_gains(void)
{
    Kp =link_epx_values->Kp;
    Ki =link_epx_values->Ki;
    Kd =link_epx_values->Kd;
}

void pid_link_memory(epx_configuration_t *temp_link_epx_values)
{
    link_epx_values = temp_link_epx_values;
}


// PID Controller function
float pidController(float setpoint, float measuredValue) {

    // Calculate the error
    float error = setpoint - measuredValue;
    
    // Calculate the integral term
    integral += error;
    
    // Apply integral limits
    if (integral > integralMax) {
        integral = integralMax;
    } else if (integral < integralMin) {
        integral = integralMin;
    }
    
    // Calculate the derivative term
    float derivative = error - previousError;
    
    // Calculate the control signal
    float controlSignal = Kp * error + Ki * integral + Kd * derivative;
    
    // Update state variables
    previousError = error;

    // Apply drive limits
    if (controlSignal > ControlMax) {
        controlSignal = ControlMax;
    } else if (controlSignal < ControlMin) {
        controlSignal = ControlMin;
    }

    // Apply drive limits
    if (controlSignal < DeadMax && controlSignal > DeadMin) {
        controlSignal = 0;
    }
    
    return controlSignal;
}