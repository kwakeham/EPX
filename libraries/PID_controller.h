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

#ifndef PID_CONTROLLER__
#define PID_CONTROLLER__

/**
 * @brief update the kp from default
 * 
 * @param temp_Kp 
 */
void update_Kp(float temp_Kp);

/**
 * @brief update the ki from default
 * 
 * @param temp_Ki 
 */
void update_Ki(float temp_Ki);

/**
 * @brief update the ki from default
 * 
 * @param temp_Kd
 */
void update_Kd(float temp_Kd);

float pidController(float setpoint, float measuredValue);

#endif