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
#include "gears.h"
#include "telemetry.h"

#define NRF_LOG_MODULE_NAME datahandler
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();


#define CHAR_LENGTH 10
#define DATAOUTENABLE

// Two-point gear calibration: capture these two gears, interpolate the rest.
// Indices are 0-based (gear 2 -> index 1, gear 10 -> index 9).
#define GEAR_REF_LO_IDX 1
#define GEAR_REF_HI_IDX 9

static int32_t ref_lo = 0, ref_hi = 0;
static bool    ref_lo_set = false, ref_hi_set = false;

static char command_message[10] = {};

bool data_process_command = false;

epx_configuration_t epx_configuration;
epx_position_configuration_t epx_position;

//nus buffers
char buff1[50];
char buff2[50];

bool update_config_flash = false; //Update the falsh memory from the main loop
bool update_pos_flash = false; //Update the falsh memory from the main loop

bool shift_mode = true; //if true then we are in a gear mode, if false we're in an angle mode 
uint8_t long_mode_count = 0;

uint32_t dh_debug_counter = 0;

void data_handler_button_event_handler(multibtn_event_t evt)
{
    NRF_LOG_INFO("button_event_handler %d", evt);
	switch (evt)
	{
	case MULTI_BTN_EVENT_NOTHING:
		break;

	case MULTI_BTN_EVENT_CH1_PUSH:
        data_handler_shift_gear_handler(false, -1);
		break;

	case MULTI_BTN_EVENT_CH2_PUSH:
        data_handler_shift_gear_handler(false, 1);
		break;

	case MULTI_BTN_EVENT_CH3_PUSH:
        data_handler_shift_mode_handler(false, true); //temporary, switch back to gear mode
		break;

	case MULTI_BTN_EVENT_CH4_PUSH:
		break;

	case MULTI_BTN_EVENT_CH1_LONG:
        data_handler_shift_gear_handler(false, -1);
		break;

	case MULTI_BTN_EVENT_CH2_LONG:
        data_handler_shift_gear_handler(false, 1);
		break;

	case MULTI_BTN_EVENT_CH3_LONG:
        // data_handler_shift_mode_handler(false, false);
        data_handler_long_mode_handler(true);
		break;

	case MULTI_BTN_EVENT_CH4_LONG:
		break;

	case MULTI_BTN_EVENT_CH1_RELEASE:
		break;

	case MULTI_BTN_EVENT_CH2_RELEASE:
		break;

	case MULTI_BTN_EVENT_CH3_RELEASE:
        data_handler_long_mode_handler(false);
		break;

	case MULTI_BTN_EVENT_CH4_RELEASE:
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
        data_handler_shift_mode_handler(true, false); // if the command (first) is true the mode (2nd) is ignored
        break;

    case 0x70: //p Set Kp
        NRF_LOG_INFO("little P");
        epx_configuration.Kp =  data_handler_command_float_return(1);
        data_handler_show_gains();
        break;

    case 0x69: //i set Ki
        NRF_LOG_INFO("little i");
        epx_configuration.Ki =  data_handler_command_float_return(1);
        data_handler_show_gains();
        break;

    case 0x64: //d Set Kd
        NRF_LOG_INFO("little d");
        epx_configuration.Kd =  data_handler_command_float_return(1);
        data_handler_show_gains();
        break;

    case 0x73: //s 
        NRF_LOG_INFO("little s");
        if (shift_mode)
        {
            data_handler_shift_gear_handler(true, 0);
        }
        break;

    case 0x74: //t Target Angle in degrees
        NRF_LOG_INFO("little t");
        if (!shift_mode)
        {
            mpos_update_angle(true, data_handler_command_float_return(1));
        }
        break;

    case 0x79: //y Telemetry stream: y1 = on, y0 = off
        NRF_LOG_INFO("little y");
        telemetry_set_enabled(command_message[1] == '1');
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
    if (command == 0x53 || command == 0x73) //if 's' or 'S'
    {
        update_config_flash = true;
    }
    NRF_LOG_INFO("Force save");
}

void data_handler_shift_gear_handler(bool command, int shift_count)
{
    if(shift_mode) //check if we're in shift mode
    {
        if (command) //if there is a command then process and decode
        {
            switch (command_message[1])
            {
            case 0x2B: //+
                epx_position.current_gear++;
                break;
            case 0x2D: //-
                epx_position.current_gear--;
                break;
            default:
                epx_position.current_gear = data_handler_command_number_return(1);
                break;
            }
        } else //no command then process the shift count
        {
            epx_position.current_gear += shift_count;
        }

        //guards to ensure it stays within number of gears
        if(epx_position.current_gear > epx_configuration.num_gears-1)
        {
            epx_position.current_gear = epx_configuration.num_gears-1;
        }

        if(epx_position.current_gear < 0)
        {
            epx_position.current_gear = 0;
        }

        NRF_LOG_INFO("Current gear: %ld Angle: %ld",epx_position.current_gear, epx_configuration.gear_pos[(epx_position.current_gear)]);
        mpos_update_angle(true,(float)epx_configuration.gear_pos[(epx_position.current_gear)]);
    } else //if we're in angle mode
    {
        mpos_update_angle(false, (float)shift_count*5);
    }

    
}

void data_handler_shift_mode_handler(bool command, bool mode)
{
    if (command) //if we are dealing with a text command we'll do some decode
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
    } else // we're looking at a direct mode switch
    {
        shift_mode = mode;
    }

    NRF_LOG_INFO(" %s %d" , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
}

void data_handler_long_mode_handler(bool long_press)
{
    if (long_press)
    {
        long_mode_count++;
    } else //reset on release
    {
        long_mode_count = 0;
    }

    if(long_mode_count > 4)
    {
        long_mode_count = 0;
        data_handler_shift_mode_handler(false, false); //change modes
    }
}

void data_handler_command_gear_value(void)
{
    update_config_flash = true;

    switch (command_message[1])
    {
    case 0x31: //1
        epx_configuration.gear_pos[0] = data_handler_command_number_return(2);
        break;
    case 0x32: //2
        epx_configuration.gear_pos[1] = data_handler_command_number_return(2);
        break;
    case 0x33: //3
        epx_configuration.gear_pos[2] = data_handler_command_number_return(2);
        break;
    case 0x34: //4
        epx_configuration.gear_pos[3] = data_handler_command_number_return(2);
        break;
    case 0x35: //5
        epx_configuration.gear_pos[4] = data_handler_command_number_return(2);
        break;
    case 0x36: //6
        epx_configuration.gear_pos[5] = data_handler_command_number_return(2);
        break;
    case 0x37: //7
        epx_configuration.gear_pos[6] = data_handler_command_number_return(2);
        break;
    case 0x38: //8
        epx_configuration.gear_pos[7] = data_handler_command_number_return(2);
        break;
    case 0x39: //9
        epx_configuration.gear_pos[8] = data_handler_command_number_return(2);
        break;
    case 0x61: //10 - a
        epx_configuration.gear_pos[9] = data_handler_command_number_return(2);
        break;
    case 0x62: //11 - b
        epx_configuration.gear_pos[10] = data_handler_command_number_return(2);
        break;
    case 0x63: //12 - c
        epx_configuration.gear_pos[11] = data_handler_command_number_return(2);
        break;
    case 0x64: //13 - d
        epx_configuration.gear_pos[12] = data_handler_command_number_return(2);
        break;
    case 0x65: //14 - e
        epx_configuration.gear_pos[13] = data_handler_command_number_return(2);
        break;
    case 0x66: //f
        epx_configuration.num_gears = data_handler_command_number_return(2);
        NRF_LOG_INFO(" %d" , epx_configuration.num_gears);
        break;
    case 0x6C: //l capture current angle as the LOW reference (gear 2)
        update_config_flash = false; //reference is RAM-only until interpolation runs
        ref_lo = (int32_t)mpos_last_angle();
        ref_lo_set = true;
        sprintf(buff1, "Low ref (gear %d): %ld", GEAR_REF_LO_IDX + 1, ref_lo);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        return;
    case 0x68: //h capture current angle as the HIGH reference (gear 10)
        update_config_flash = false;
        ref_hi = (int32_t)mpos_last_angle();
        ref_hi_set = true;
        sprintf(buff1, "High ref (gear %d): %ld", GEAR_REF_HI_IDX + 1, ref_hi);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        return;
    case 0x69: //i interpolate all gears from the two captured references
        data_handler_compute_gears();
        return;
    default:
        update_config_flash = false; //if it wasn't the other cases, don't update the flash memory
        break;
    }
    data_handler_command_gear_value_print();
}

void data_handler_command_gear_value_print(void)
{
    sprintf(buff1, "Gear 1: %ld, %ld, %ld, %ld, %ld, %ld",epx_configuration.gear_pos[0], epx_configuration.gear_pos[1], epx_configuration.gear_pos[2], epx_configuration.gear_pos[3], epx_configuration.gear_pos[4], epx_configuration.gear_pos[5]);
    sprintf(buff2, "Gear 7: %ld, %ld, %ld, %ld, %ld, %ld",epx_configuration.gear_pos[6], epx_configuration.gear_pos[7], epx_configuration.gear_pos[8], epx_configuration.gear_pos[9], epx_configuration.gear_pos[10], epx_configuration.gear_pos[11]);

    NRF_LOG_INFO(" %s %d" , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
    nus_data_send((uint8_t *)buff2, strlen(buff2));
}

void data_handler_compute_gears(void)
{
    if (!ref_lo_set || !ref_hi_set)
    {
        sprintf(buff1, "Capture gl and gh first");
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        return;
    }

    bool ok = gears_interpolate(epx_configuration.gear_pos,
                                epx_configuration.num_gears,
                                GEAR_REF_LO_IDX, ref_lo,
                                GEAR_REF_HI_IDX, ref_hi);
    if (!ok)
    {
        sprintf(buff1, "Interp failed (num_gears %ld?)", epx_configuration.num_gears);
        nus_data_send((uint8_t *)buff1, strlen(buff1));
        return;
    }

    update_config_flash = true; // persist the computed positions
    data_handler_command_gear_value_print();
}

void data_handler_show_gains(void)
{
    sprintf(buff1, "Gains: Kp: %.3f, Ki: %.3f, Kd: %.3f",epx_configuration.Kp, epx_configuration.Ki, epx_configuration.Kd);
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

    if(update_config_flash)
    {
        NRF_LOG_INFO("data_handler_sch_execute config flash update");
        update_config_flash = false;
        mem_epx_config_update(epx_configuration);
    }

    if(update_pos_flash)
    {
        NRF_LOG_INFO("data_handler_sch_execute position flash update");
        update_pos_flash = false;
        mem_epx_position_update(epx_position);
    }

}

void data_handler_get_flash_values(void)
{
    epx_configuration = tm_fds_epx_config(); //get configuration from titanmem
    epx_position = tm_fds_epx_position(); //get position data from titanmem
    //link the live gains to the PID (owned by mpos) so it reads the stored config directly
    mpos_link_gains(&epx_configuration.Kp, &epx_configuration.Ki, &epx_configuration.Kd);
    mpos_link_memory(&epx_position);//link the local value to the mpos controller
}

void data_handler_req_update_position_flash(void)
{
    update_pos_flash = true;
}



