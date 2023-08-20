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
#define Kp 20.0   // Proportional gain
#define Ki 0   // Integral gain
#define Kd 0  // Derivative gain

// PID Controller state variables
double previousError = 0.0;
double integral = 0.0;

// Integral limits
double integralMax = 20.0;
double integralMin = -20.0;

// Control limits
double ControlMax = 300;
double ControlMin = -300;

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