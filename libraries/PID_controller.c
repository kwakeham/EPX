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
float Ki = 0;  // Integral gain
float Kd = 0;  // Derivative gain

//At 256hz (128 repeat) kp = 18
//At 128hz kp = 9
//AT 512hz kp = 35-40


// PID Controller state variables
float previousError = 0.0;
float integral = 0.0;

// Integral limits
float integralMax = 20.0;
float integralMin = -20.0;

// Control limits
float ControlMax = 400;
float ControlMin = -400;

void update_Kp(float temp_Kp)
{
    Kp = temp_Kp;
}

void update_Ki(float temp_Ki)
{
    Ki = temp_Ki;
}

void update_Kd(float temp_Kd)
{
    Kd = temp_Kd;
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
    
    return controlSignal;
}