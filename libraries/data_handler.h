/**
 * Copyright (c) 2019 - 2022, TITAN LAB INC
 *
 * All rights reserved.
 * 
 *
 */


#ifndef __DATAHANDLER_H__
#define __DATAHANDLER_H__

#include <stdbool.h>
#include "multi_btn.h"

void data_handler_button_event_handler(multibtn_event_t evt);

void data_handler_command(const char* p_chars, uint32_t length);

void data_handler_command_processor(void);

float data_handler_command_float_return(uint8_t offset);

int32_t data_handler_command_number_return(uint8_t offset);

void data_handler_force_save(char command);

void data_handler_shift_gear_handler(bool command, int shift_count);

/**
 * @brief Function that handles long presses and waits for a few before doing somethings
 * 
 * @param long_or_not_release true for long, false for release
 */
void data_handler_long_mode_handler(bool long_or_not_release);

void data_handler_shift_mode_handler(bool command, bool mode);

void data_handler_command_gear_value(void);

void data_handler_show_gains(void);

void data_handler_sch_execute(void);

void data_handler_get_flash_values(void);

void data_handler_req_update_position_flash(void);

#endif