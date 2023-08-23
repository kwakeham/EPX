/**
 * Copyright (c) 2019 - 2022, TITAN LAB INC
 *
 * All rights reserved.
 * 
 *
 */

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#ifndef __DATAHANDLER_H__
#define __DATAHANDLER_H__


void data_handler_command(const char* p_chars, uint32_t length);

void data_handler_command_processor(void);

float data_handler_command_float_return(uint8_t offset);

int32_t data_handler_command_number_return(uint8_t offset);

void data_handler_force_save(char command);

void data_handler_command_gear_value(void);

void data_handler_show_gains(void);

void data_handler_sch_execute(void);

void data_handler_get_flash_values(void);

bool data_handler_averaging(void);

int32_t data_handler_averaging_count(void);

#endif