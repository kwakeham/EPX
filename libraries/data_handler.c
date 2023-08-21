/**
 * Copyright (c) 2019 - 2022, TITAN LAB INC
 *
 * All rights reserved.
 * 
 *
 */

#include "data_handler.h"
#include "boards.h"
#include "math.h"
// #include "arm_math.h"
#include "ble_cus.h"
#include <string.h>
#include "titan_mem.h"
#include "mpos.h"
#include "PID_controller.h"
// #include "hf_time.h"
// #include "linearinterpolator.h"

#define DEFAULT_AVERAGE_SAMPLES 250

#define MAX_ADC_CHANNELS 4
#define DATA_BUFFER_LENGTH 4
#define CHAR_LENGTH 10
#define DATAOUTENABLE

static char command_message[10] = "c55525";
char write_message[240] = "5n51000001,5.1,2122\n";

// static char nus_message[128];

//dataoutput variables
// static float torque_L[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// static float torque_R[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// static uint32_t sample_time[6] = {0, 0, 0, 0, 0, 0};
// static float gyro_angle[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// static float gyro_z[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

bool average_data = false;
bool zero_request = false;
bool new_adc_data = false;
int32_t average_count = 0;
int32_t max_average_count = 0;

bool data_process_command = false;

epx_configuration_t epx_values;


//temp
char buff1[50];
char buff2[50];
bool update_flash = false; //did something change, should we update the falsh memory

uint32_t dh_debug_counter = 0;

void data_handler_command(const char* p_chars, uint32_t length)
{
    memset(command_message, 0, sizeof(command_message));
    memcpy(command_message, p_chars, length);
    data_process_command = true;
}

void data_handler_command_processor(void)
{
    // NRF_LOG_INFO("command processor: %s", command_message);
    switch (command_message[0])
    {
    case 0x5A: //Z Zero
        NRF_LOG_INFO("big Z");
        average_data = true;
        zero_request = true;
        max_average_count = 255;
        break;
    case 0x7A: //z Zero
        NRF_LOG_INFO("little z");
        average_data = true;
        zero_request = true;
        max_average_count = 255;
        break;

    case 0x41: //A
        average_data = true;
        max_average_count = data_handler_command_number_return(1);
        NRF_LOG_INFO("Big A %d",max_average_count);
        break;
    case 0x61: //ac 
        average_data = true;
        max_average_count = data_handler_command_number_return(1);
        NRF_LOG_INFO("little a %d",max_average_count);
        break;

    case 0x43: //C Calibration coefficient
        NRF_LOG_INFO("big C");
        data_handler_command_gear_value();
        break;
    case 0x63: //c Calibration coefficient
        NRF_LOG_INFO("little c");
        data_handler_command_gear_value();
        break;

    case 0x42: //B Calibration coefficient
        NRF_LOG_INFO("big B");
        break;
    case 0x62: //b Calibration coefficient
        NRF_LOG_INFO("little b");
        break;

    case 0x47: //G Gear offset
        NRF_LOG_INFO("big G");
        data_handler_command_gear_value();
        break;
    case 0x67: //g Gear offset
        NRF_LOG_INFO("little g");
        data_handler_command_gear_value();
        break;

    case 0x4D: //M Force Output
        NRF_LOG_INFO("big M");
        break;
    case 0x6D: //m Force Output
        NRF_LOG_INFO("little m");
        break;

    case 0x50: //P Set Kp
        NRF_LOG_INFO("big P");
        update_Kp(data_handler_command_float_return(1));
        break;
    case 0x70: //p Set Kp
        NRF_LOG_INFO("little P");
        update_Kp(data_handler_command_float_return(1));
        break;

    case 0x49: //I set Ki
        NRF_LOG_INFO("big I");
        update_Ki(data_handler_command_float_return(1));
        break;
    case 0x69: //i set Ki
        NRF_LOG_INFO("little i");
        update_Ki(data_handler_command_float_return(1));
        break;

    case 0x44: //D Set Kd
        NRF_LOG_INFO("big D");
        update_Kd(data_handler_command_float_return(1));
        break;
    case 0x64: //d Set Kd
        NRF_LOG_INFO("little d");
        update_Kd(data_handler_command_float_return(1));
        break;

    case 0x52: //R Raw Output
        NRF_LOG_INFO("big R");
        break;
    case 0x72: //r Raw Output
        NRF_LOG_INFO("little r");
        break;

    case 0x54: //T Tared ouput
        NRF_LOG_INFO("big T");
        mpos_update_angle(data_handler_command_float_return(1));
    case 0x74: //t Tared ouput
        NRF_LOG_INFO("little t");
        mpos_update_angle(data_handler_command_float_return(1));
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

void data_handler_command_gear_value(void)
{
    update_flash = true;
    switch (command_message[1])
    {
    case 0x31: //1
        epx_values.gear1_pos = data_handler_command_number_return(2);
        break;
    case 0x32: //2
        epx_values.gear2_pos = data_handler_command_number_return(2);
        break;
    case 0x33: //3
        epx_values.gear3_pos = data_handler_command_number_return(2);
        break;
    case 0x34: //4
        epx_values.gear4_pos = data_handler_command_number_return(2);
        break;
    case 0x35: //5
        epx_values.gear5_pos = data_handler_command_number_return(2);
        break;
    case 0x36: //6
        epx_values.gear6_pos = data_handler_command_number_return(2);
        break;
    case 0x37: //7
        epx_values.gear7_pos = data_handler_command_number_return(2);
        break;
    case 0x38: //8
        epx_values.gear8_pos = data_handler_command_number_return(2);
        break;
    case 0x39: //9
        epx_values.gear9_pos = data_handler_command_number_return(2);
        break;
    case 0x61: //a
        epx_values.gear10_pos = data_handler_command_number_return(2);
        break;
    case 0x62: //b
        epx_values.gear11_pos = data_handler_command_number_return(2);
        break;
    case 0x63: //c
        epx_values.gear12_pos = data_handler_command_number_return(2);
        break;
    case 0x64: //d
        epx_values.gear13_pos = data_handler_command_number_return(2);
        break;
    default:
        update_flash = false; //if it wasn't the other cases, don't update the flash memory
        break;
    }
    sprintf(buff1, "Gear 1: %ld, %ld, %ld, %ld",epx_values.gear1_pos,epx_values.gear2_pos,epx_values.gear3_pos,epx_values.gear4_pos);
    // sprintf(buff2, "y: %.6f, %.6f, %.6f, %.6f",epx_values.C1y_cal,epx_values.C2y_cal,epx_values.C3y_cal,epx_values.C4y_cal);
    // sprintf(buff2, "y: %.6f, %.6f, %.6f, %.6f",epx_values.C1y_cal,epx_values.C2y_cal,epx_values.C3y_cal,epx_values.C4y_cal);
    NRF_LOG_INFO(" %s " , buff1);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
    // nus_data_send((uint8_t *)buff2, strlen(buff2));
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

}

bool data_handler_averaging(void)
{
    return average_data;
}

int32_t data_handler_averaging_count(void)
{
    return average_count;
}