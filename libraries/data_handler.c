/**
 * Copyright (c) 2019 - 2022, TITAN LAB INC
 *
 * All rights reserved.
 * 
 *
 */
#include <stdio.h>
#include <ctype.h>
#include "data_handler.h"
#include "boards.h"
#include "math.h"
#include "ble_cus.h"
#include <string.h>
#include "titan_mem.h"
#include "mpos.h"
#include "PID_controller.h"

#define NRF_LOG_MODULE_NAME dh
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();


#define CHAR_LENGTH 10
#define DATAOUTENABLE

static char command_message[10] = {}; 

bool data_process_command = false;

epx_configuration_t epx_values;

//nus buffers
char buff1[50];
char buff2[50];

bool update_flash = false; //Update the falsh memory from the main loop

bool shift_mode = true; //if true then we are in a gear mode, if false we're in an angle mode 

uint32_t dh_debug_counter = 0;

void data_handler_button_event_handler(multibtn_event_t evt)
{
    NRF_LOG_INFO("button_event_handler %d", evt);
	switch (evt)
	{
	case MULTI_BTN_EVENT_NOTHING:
		break;

	case MULTI_BTN_EVENT_CH1_PUSH:
		break;

	case MULTI_BTN_EVENT_CH2_PUSH:
		break;

	case MULTI_BTN_EVENT_CH3_PUSH:
		break;

	case MULTI_BTN_EVENT_CH4_PUSH:
		break;

	case MULTI_BTN_EVENT_CH1_LONG:
		break;

	case MULTI_BTN_EVENT_CH2_LONG:
		break;

	case MULTI_BTN_EVENT_CH3_LONG:
		break;

	case MULTI_BTN_EVENT_CH4_LONG:
		break;

	default:
		break;
	}

}

void data_handler_command(const char* p_chars, uint32_t length)
{
    memset(command_message, 0, sizeof(command_message));
    memcpy(command_message, p_chars, length);
    data_process_command = true;
}

void data_handler_command_processor(void)
{
    command_message[0] = tolower(command_message[0]);
    command_message[1] = tolower(command_message[1]);
    // NRF_LOG_INFO("command processor: %s", command_message);
    switch (command_message[0])
    {
    
    case 0x66: //f force save coefficients
        NRF_LOG_INFO("little f");
        data_handler_force_save(command_message[1]);
        break;

    case 0x67: //g Set the position for each Gear
        NRF_LOG_INFO("little g");
        data_handler_command_gear_value();
        break;
    
    case 0x6B: //k List Gains
        NRF_LOG_INFO("little k");
        data_handler_show_gains();
        break;

    case 0x6D: //g Set the position for each Gear
        NRF_LOG_INFO("little m");
        data_handler_shift_mode_handler();
        break;

    case 0x70: //p Set Kp
        NRF_LOG_INFO("little P");
        epx_values.Kp =  data_handler_command_float_return(1);
        pid_update_gains();
        data_handler_show_gains();
        break;

    case 0x69: //i set Ki
        NRF_LOG_INFO("little i");
        epx_values.Ki =  data_handler_command_float_return(1);
        pid_update_gains();
        data_handler_show_gains();
        break;

    case 0x64: //d Set Kd
        NRF_LOG_INFO("little d");
        epx_values.Kd =  data_handler_command_float_return(1);
        pid_update_gains();
        data_handler_show_gains();
        break;

    case 0x73: //s 
        NRF_LOG_INFO("little s");
        if (shift_mode)
        {
            data_handler_shift_geat_handler();
        }
        break;

    case 0x74: //t Target Angle in degrees
        NRF_LOG_INFO("little t");
        if (!shift_mode)
        {
            mpos_update_angle(data_handler_command_float_return(1));
        }
        break;
        
    default:
        break;
    }
    
}

float data_handler_command_float_return(uint8_t offset)
{
    char temp_array[CHAR_LENGTH];
    strncpy(temp_array, command_message+offset, sizeof(command_message)-1);
    float calibration_coefficient = strtof(temp_array, NULL);
    return(calibration_coefficient);
}

int32_t data_handler_command_number_return(uint8_t offset)
{
    char temp_array[CHAR_LENGTH];
    int32_t x;
    strncpy(temp_array, command_message+offset, sizeof(command_message)-1);
    x = atoi(temp_array);
    NRF_LOG_INFO("value, %ld", x);
    return(x);
}

void data_handler_force_save(char command)
{
    if (command == 0x53 || command == 0x73)
    {
        update_flash = true;
    }
    NRF_LOG_INFO("Force save");
}

void data_handler_shift_geat_handler(void)
{
    switch (command_message[1])
    {
    case 0x2B: //+
        epx_values.current_gear++;
        break;
    case 0x2D: //-
        epx_values.current_gear--;
        break;
    default:
        epx_values.current_gear = data_handler_command_number_return(1);
        break;
    }

    //guards
    if(epx_values.current_gear > epx_values.num_gears-1)
    {
        epx_values.current_gear = epx_values.num_gears-1;
    }

    if(epx_values.current_gear < 0)
    {
        epx_values.current_gear = 0;
    }

    NRF_LOG_INFO("Current gear: %ld Angle: %ld",epx_values.current_gear, epx_values.gear_pos[(epx_values.current_gear)]);
    mpos_update_angle((float)epx_values.gear_pos[(epx_values.current_gear)]);

}

void data_handler_shift_mode_handler(void)
{
    switch (command_message[1])
    {
    case 0x61: //a
        shift_mode = false;
        sprintf(buff1, "Angle Mode");
        break;

    case 0x67: //g
        shift_mode = true;
        sprintf(buff1, "Gear Mode");
        break; 

    default:
        break;
    }
    NRF_LOG_INFO(" %s %d" , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
    
}

void data_handler_command_gear_value(void)
{
    update_flash = true;

    switch (command_message[1])
    {
    case 0x31: //1
        epx_values.gear_pos[0] = data_handler_command_number_return(2);
        break;
    case 0x32: //2
        epx_values.gear_pos[1] = data_handler_command_number_return(2);
        break;
    case 0x33: //3
        epx_values.gear_pos[2] = data_handler_command_number_return(2);
        break;
    case 0x34: //4
        epx_values.gear_pos[3] = data_handler_command_number_return(2);
        break;
    case 0x35: //5
        epx_values.gear_pos[4] = data_handler_command_number_return(2);
        break;
    case 0x36: //6
        epx_values.gear_pos[5] = data_handler_command_number_return(2);
        break;
    case 0x37: //7
        epx_values.gear_pos[6] = data_handler_command_number_return(2);
        break;
    case 0x38: //8
        epx_values.gear_pos[7] = data_handler_command_number_return(2);
        break;
    case 0x39: //9
        epx_values.gear_pos[8] = data_handler_command_number_return(2);
        break;
    case 0x61: //10 - a
        epx_values.gear_pos[9] = data_handler_command_number_return(2);
        break;
    case 0x62: //11 - b
        epx_values.gear_pos[10] = data_handler_command_number_return(2);
        break;
    case 0x63: //12 - c
        epx_values.gear_pos[11] = data_handler_command_number_return(2);
        break;
    case 0x64: //13 - d
        epx_values.gear_pos[12] = data_handler_command_number_return(2);
        break;
    case 0x65: //14 - e
        epx_values.gear_pos[12] = data_handler_command_number_return(2);
        break;
    case 0x66: //f
        epx_values.num_gears = data_handler_command_number_return(2);
        NRF_LOG_INFO(" %d" , epx_values.num_gears);
        break;
    default:
        update_flash = false; //if it wasn't the other cases, don't update the flash memory
        break;
    }
    sprintf(buff1, "Gear 1: %ld, %ld, %ld, %ld, %ld, %ld",epx_values.gear_pos[0], epx_values.gear_pos[1], epx_values.gear_pos[2], epx_values.gear_pos[3], epx_values.gear_pos[4], epx_values.gear_pos[5]);
    sprintf(buff2, "Gear 7: %ld, %ld, %ld, %ld, %ld, %ld",epx_values.gear_pos[6], epx_values.gear_pos[7], epx_values.gear_pos[8], epx_values.gear_pos[9], epx_values.gear_pos[10], epx_values.gear_pos[11]);

    NRF_LOG_INFO(" %s %d" , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
    nus_data_send((uint8_t *)buff2, strlen(buff2));
}

void data_handler_show_gains(void)
{
    sprintf(buff1, "Gains: Kp: %.3f, Ki: %.3f, Kd: %.3f",epx_values.Kp, epx_values.Ki, epx_values.Kd);
    NRF_LOG_INFO(" %s " , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
}

void data_handler_sch_execute(void)
{

    if (data_process_command)
    {
        data_handler_command_processor();
        data_process_command = false;
        NRF_LOG_INFO("data_process_command, %d", data_process_command);
    }

    if(update_flash)
    {
        update_flash = false;
        mem_epx_update(epx_values);
    }

}

void data_handler_get_flash_values(void)
{
    epx_values = tm_fds_epx_config();
    link_memory(&epx_values);
}
