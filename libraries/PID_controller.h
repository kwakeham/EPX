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

#include "titan_mem.h"


void pid_link_memory(epx_configuration_t *temp_link_epx_values);

float pidController(float setpoint, float measuredValue);

#endif