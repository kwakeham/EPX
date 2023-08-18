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
#define Kp 1.0   // Proportional gain
#define Ki 0.1   // Integral gain
#define Kd 0.01  // Derivative gain

// PID Controller state variables
double previousError = 0.0;
double integral = 0.0;

// Integral limits
double integralMin = -10.0;
double integralMax = 10.0;

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
    
    return controlSignal;
}