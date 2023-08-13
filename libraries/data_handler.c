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
// #include "hf_time.h"
// #include "linearinterpolator.h"

#define DEFAULT_AVERAGE_SAMPLES 250

#define MAX_ADC_CHANNELS 4
#define DATA_BUFFER_LENGTH 4
#define CHAR_LENGTH 10
#define DATAOUTENABLE

static char command_message[10] = "c55525";
char write_message[240] = "5n51000001,5.1,2122\n";

static int32_t CHdata_raw[MAX_ADC_CHANNELS]; //Data storage for the channel data that was just read
static int32_t CHdata_average[MAX_ADC_CHANNELS]; //Data storage for the channel data that was just read

data_handler_raw_configuration_t data_handler_CHdata_raw; //holds the last two samples
// data_handler_interpolated_configuration_t data_handler_interpolated; //holds the interpolated data

static int32_t accel_average[4];
uint16_t accel_average_count = 0;

static int32_t gyro_zero[3];
static int32_t gyro_average[3];
uint16_t gyro_average_count = 0;

static int16_t tmp117_temperature;
static char nus_message[128];


static float torque[2] = {0.0f, 0.0f};
static int32_t gyro_rpm = 0;
static int32_t raw_integrated_z_angle = 0;

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

epx_configuration_t cal_factors;
//calculated active zero
int32_t active_zero[6] = {0,0,0,0,0,0};
bool calculate_az = false; //should we calculate the active zero. True = yes.

//temp
char buff1[50];
char buff2[50];
bool update_flash = false; //did something change, should we update the falsh memory

uint32_t adc_ms_time_old = 0;
uint16_t missedcount = 0;

bool safe_write_time = false;
uint32_t dh_debug_counter = 0;
int32_t sample_time_difference = 0;


void data_handler_strain_data(int32_t *raw_data, uint8_t first_channel, uint8_t channel_offset)
{
    //guard against too many channels
    if((first_channel + channel_offset) > MAX_ADC_CHANNELS)
    {
        NRF_LOG_ERROR("Too many channels");
        return;
    }

    for (size_t i = 0; i < channel_offset; i++)
    {
        CHdata_raw[i+first_channel] = raw_data[i];
    }
    new_adc_data = true;
}

void data_handler_calculate_active_zero(void)
{
    active_zero[0] = (int32_t)(tmp117_temperature*cal_factors.C1_thermal_m) + cal_factors.CH1_thermal_b;
    active_zero[1] = (int32_t)(tmp117_temperature*cal_factors.C2_thermal_m) + cal_factors.CH2_thermal_b;
    active_zero[2] = (int32_t)(tmp117_temperature*cal_factors.C3_thermal_m) + cal_factors.CH3_thermal_b;
    active_zero[3] = (int32_t)(tmp117_temperature*cal_factors.C4_thermal_m) + cal_factors.CH4_thermal_b;
}

void data_handler_calculate_torque(void)
{
    torque[0] = (CHdata_raw[0] - active_zero[0] - cal_factors.CH1_zero)*cal_factors.C1x_cal
                + (CHdata_raw[1] - active_zero[1] - cal_factors.CH2_zero)*cal_factors.C2x_cal
                + (CHdata_raw[2] - active_zero[2] - cal_factors.CH3_zero)*cal_factors.C3x_cal
                + (CHdata_raw[3] - active_zero[2] - cal_factors.CH4_zero)*cal_factors.C4x_cal;

    torque[1] = (CHdata_raw[0] - active_zero[0] - cal_factors.CH1_zero)*cal_factors.C1y_cal
                + (CHdata_raw[1] - active_zero[1] - cal_factors.CH2_zero)*cal_factors.C2y_cal
                + (CHdata_raw[2] - active_zero[2] - cal_factors.CH3_zero)*cal_factors.C3y_cal
                + (CHdata_raw[3]  - active_zero[3] - cal_factors.CH4_zero)*cal_factors.C4y_cal;

    sprintf(buff1, "%.3f, %.3f \n",torque[0], torque[1]);  
    NRF_LOG_RAW_INFO("Torque %s", buff1);            
    nus_data_send((uint8_t *)buff1, strlen(buff1));
}

void data_handler_accel_data(int16_t *raw_accel, uint8_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        accel_average[i] += raw_accel[i];
    }
    accel_average_count++;
    // NRF_LOG_RAW_INFO("ACCEL: %d, %d, %d, %d\n", raw_accel[0], raw_accel[1], raw_accel[2], raw_accel[3]);
}

void data_handler_gyro_data(int16_t *raw_gyro)
{
    for (size_t i = 0; i < 3; i++)
    {
        gyro_average[i] += raw_gyro[i];
    }
    gyro_average_count++;
}

void data_handler_gyro_average()
{
    if(gyro_average_count>0)
    {
        for (size_t i = 0; i < 3; i++)
        {
            gyro_average[i] = gyro_average[i]/gyro_average_count;
        }
        gyro_average_count = 0;
        gyro_rpm = gyro_average[2];
        raw_integrated_z_angle += (gyro_average[2] - gyro_zero[2]);
        memset(gyro_average, 0, sizeof(gyro_average));
    }
}

void data_handler_gyro_z_angle_reset()
{
    raw_integrated_z_angle = 0;
}


void data_handler_adc_average(void)
{
    if(new_adc_data) //secondary check
    {
        for (size_t i = 0; i < MAX_ADC_CHANNELS; i++)
        {
            CHdata_average[i]+= CHdata_raw[i];
        }
        average_count++;

        if (average_count >= max_average_count) //we maxed out time to calculate average
        {
            //calculate the averages
            data_handler_adc_average_calculate();
            data_handler_accel_average_calculate();

            //Send the averages over the uart
            data_handler_nus_send_average();

            //Reset the handlers
            data_handler_adc_average_reset();
            data_handler_accel_average_reset();
            gyro_zero[2] = gyro_rpm; //quick get gyro for zero
        }
    }

}


void data_handler_adc_average_calculate(void)
{
    for (size_t i = 0; i < MAX_ADC_CHANNELS; i++)
    {
        CHdata_average[i]= CHdata_average[i]/average_count; //average
    }
    if(!zero_request)
    {
        NRF_LOG_RAW_INFO("AVG: %d, %d, %d, ", CHdata_average[0], CHdata_average[1], CHdata_average[2]);
        NRF_LOG_RAW_INFO("%d\n ", CHdata_average[3]);
    }
    average_count = 0; //reset the counter
    average_data = false; //reset the flag
    if (zero_request)
    {
        zero_request = false;
        cal_factors.CH1_zero = CHdata_average[0] - active_zero[0];
        cal_factors.CH2_zero = CHdata_average[1] - active_zero[1];
        cal_factors.CH3_zero = CHdata_average[2] - active_zero[2];
        cal_factors.CH4_zero = CHdata_average[3] - active_zero[3];
        update_flash = true;
        NRF_LOG_RAW_INFO("AVG: %d, %d, %d, ", cal_factors.CH1_zero, cal_factors.CH2_zero, cal_factors.CH3_zero);
        NRF_LOG_RAW_INFO("%d\n ", cal_factors.CH4_zero);
    }
    
}

void data_handler_accel_average_calculate(void)
{
    for (size_t i = 0; i < 4; i++)
    {
        accel_average[i]= accel_average[i]/accel_average_count; //average
    }
    NRF_LOG_RAW_INFO("AAVG: %d, %d, %d, ", accel_average[0], accel_average[1], accel_average[2]);
    accel_average_count = 0;
    
}

void data_handler_adc_average_reset(void)
{
    memset(CHdata_average, 0, sizeof(CHdata_average));
}

void data_handler_accel_average_reset(void)
{
    memset(accel_average, 0, sizeof(accel_average));
}

void data_handler_nus_send_average(void)
{
    memset(nus_message, 0, sizeof(nus_message)); //Clear the bugger

    for (size_t i = 0; i < MAX_ADC_CHANNELS; i++)
    {
        sprintf(nus_message+strlen(nus_message),"%ld,",CHdata_average[i]);
    }
    // sprintf(nus_message+strlen(nus_message),"\n");
      // NRF_LOG_RAW_INFO("%s", nus_message);
#ifdef DATAOUTENABLE            
    nus_data_send((uint8_t *)nus_message, strlen(nus_message)); //ignore the nulls!
#endif

    memset(nus_message, 0, sizeof(nus_message)); //Clear the bugger
    sprintf(nus_message+strlen(nus_message),"%ld, %ld,",accel_average[0],accel_average[1]);
    sprintf(nus_message+strlen(nus_message),"%d\n",tmp117_temperature);
#ifdef DATAOUTENABLE            
    nus_data_send((uint8_t *)nus_message, strlen(nus_message)); //ignore the nulls!
#endif  
}

void data_handler_zero_request(void)
{
    average_data = true;
    zero_request = true;
    max_average_count = 255;
}


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
        // data_handler_command_number_return(2);
        data_handler_command_calibration_value();
        break;
    case 0x63: //c Calibration coefficient
        NRF_LOG_INFO("little c");
        // data_handler_command_number_return(2);
        data_handler_command_calibration_value();
        break;

    case 0x42: //B Calibration coefficient
        NRF_LOG_INFO("big B");
        // data_handler_command_number_return(2);
        data_handler_command_temp_offset_value();
        break;
    case 0x62: //b Calibration coefficient
        NRF_LOG_INFO("little b");
        // data_handler_command_number_return(2);
        data_handler_command_temp_offset_value();
        break;

    case 0x4D: //F Force Output
        NRF_LOG_INFO("big M");
        data_handler_command_temp_slope_value();
        break;
    case 0x6D: //f Force Output
        NRF_LOG_INFO("little m");
        data_handler_command_temp_slope_value();
        break;

    case 0x52: //R Raw Output
        NRF_LOG_INFO("big R");
        break;
    case 0x72: //r Raw Output
        NRF_LOG_INFO("little ");
        break;

    case 0x54: //T Tared ouput
        NRF_LOG_INFO("big T");
        data_handler_calculate_torque();
        break;
    case 0x74: //t Tared ouput
        NRF_LOG_INFO("little t");
        data_handler_calculate_torque();
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

void data_handler_command_calibration_value(void)
{
    update_flash = true;
    switch (command_message[1])
    {
    case 0x31: //1
        cal_factors.C1x_cal = data_handler_command_float_return(2);
        break;
    case 0x32: //2
        cal_factors.C2x_cal = data_handler_command_float_return(2);
        /* code */
        break;
    case 0x33: //3
        cal_factors.C3x_cal = data_handler_command_float_return(2);
        /* code */
        break;
    case 0x34: //4
        cal_factors.C4x_cal = data_handler_command_float_return(2);
        /* code */
        break;
    case 0x35: //5
        cal_factors.C1y_cal = data_handler_command_float_return(2);
        /* code */
        break;
    case 0x36: //6
        cal_factors.C2y_cal = data_handler_command_float_return(2);
        /* code */
        break;
    case 0x37: //7
        cal_factors.C3y_cal = data_handler_command_float_return(2);
        /* code */
        break;
    case 0x38: //8
        cal_factors.C4y_cal = data_handler_command_float_return(2);
        /* code */
        break;
    default:
        update_flash = false; //if it wasn't the other cases, don't update the flash memory
        break;
    }
    sprintf(buff1, "CAL x: %.6f, %.6f, %.6f, %.6f",cal_factors.C1x_cal,cal_factors.C2x_cal,cal_factors.C3x_cal,cal_factors.C4x_cal);
    sprintf(buff2, "y: %.6f, %.6f, %.6f, %.6f",cal_factors.C1y_cal,cal_factors.C2y_cal,cal_factors.C3y_cal,cal_factors.C4y_cal);
    NRF_LOG_INFO(" %s %s" , buff1, buff2);
    nus_data_send((uint8_t *)buff1, strlen(buff1));
    nus_data_send((uint8_t *)buff2, strlen(buff2));
}


void data_handler_command_temp_slope_value(void)
{
    update_flash = true;
    switch (command_message[1])
    {
    case 0x31: //1
        cal_factors.C1_thermal_m = data_handler_command_float_return(2);
        break;
    case 0x32: //2
        cal_factors.C2_thermal_m = data_handler_command_float_return(2);
        /* code */
        break;
    case 0x33: //3
        cal_factors.C3_thermal_m = data_handler_command_float_return(2);
        /* code */
        break;
    case 0x34: //4
        cal_factors.C4_thermal_m = data_handler_command_float_return(2);
        /* code */
        break;
    default:
        update_flash = false; //if it wasn't the other cases, don't update the flash memory
        break;
    }
    sprintf(buff1, "%.4f, %.4f, %.4f",cal_factors.C1_thermal_m,cal_factors.C2_thermal_m,cal_factors.C3_thermal_m);
    sprintf(buff2, "%.4f,",cal_factors.C4_thermal_m);
    NRF_LOG_INFO("Temp m: %s%s" , buff1, buff2);

}

void data_handler_command_temp_offset_value(void)
{
    update_flash = true;
    switch (command_message[1])
    {
    case 0x31: //1
        cal_factors.CH1_thermal_b = data_handler_command_number_return(2);
        break;
    case 0x32: //2
        cal_factors.CH2_thermal_b = data_handler_command_number_return(2);
        /* code */
        break;
    case 0x33: //3
        cal_factors.CH3_thermal_b = data_handler_command_number_return(2);
        /* code */
        break;
    case 0x34: //4
        cal_factors.CH4_thermal_b = data_handler_command_number_return(2);
        /* code */
        break;
    default:
        update_flash = false; //if it wasn't the other cases, don't update the flash memory
        break;
    }
    sprintf(buff1, "%ld, %ld, %ld",cal_factors.CH1_thermal_b,cal_factors.CH2_thermal_b,cal_factors.CH3_thermal_b);
    sprintf(buff2, ", %ld, ",cal_factors.CH4_thermal_b);
    NRF_LOG_INFO("Temp B : %s%s" , buff1, buff2);

}

void data_handler_safe_write_time(void)
{
    safe_write_time = true;
}

void data_handler_sch_execute(void)
{
    if (average_data) //this needs to be before the new_adc_data because it depends on it.
    {
        data_handler_adc_average();
    }

    if (new_adc_data)
    {
        new_adc_data = false;
        data_handler_gyro_average();
        // data_handler_upsample();
    }

    if (data_process_command)
    {
        data_handler_command_processor();
        data_process_command = false;
        NRF_LOG_INFO("data_process_command, %d", data_process_command);
    }

    if(calculate_az)
    {
        calculate_az = false;
        data_handler_calculate_active_zero();
    }


    if(update_flash)
    {
        update_flash = false;
        mem_epx_update(cal_factors);
    }

}

void data_handler_get_flash_calibration(void)
{
    cal_factors = tm_fds_epx_config();

}

bool data_handler_averaging(void)
{
    return average_data;
}

int32_t data_handler_averaging_count(void)
{
    return average_count;
}

void data_handler_tmp117_handler(int16_t p_temp_data)
{
    calculate_az = true;
    tmp117_temperature = p_temp_data;
}