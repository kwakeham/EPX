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

/* A  structure to store raw samples for interpolation */
typedef struct
{
    int32_t CH1L[3];
    int32_t CH2L[3];
    int32_t CH3L[3];
    int32_t CH1R[3];
    int32_t CH2R[3];
    int32_t CH3R[3];
    uint32_t time_ms[3];
    int32_t angular_velocity[3];
    int32_t integrated_angle[3];
} data_handler_raw_configuration_t;

typedef struct
{
    int32_t CH1L[36];
    int32_t CH2L[36];
    int32_t CH3L[36];
    int32_t CH1R[36];
    int32_t CH2R[36];
    int32_t CH3R[36];
    uint32_t time_ms[36];
} data_handler_interpolated_configuration_t;


/**
 * @brief This handles the incoming data and sorts it into the right location
 * Adjust MAX_ADC_CHANNELS define to set max
 * 
 * @param raw_data pointer to the raw 32bit ints
 * @param firstchannel this is the first channel
 * @param channel_offset this is the offset for the channels 
 */
void data_handler_strain_data(int32_t *raw_data, uint8_t first_channel, uint8_t channel_offset);


/**
 * @brief Handles the gyro data input for integration.
 * 
 * @param raw_gyro_data 
 */

void data_handler_accel_data(int16_t *raw_accel, uint8_t length);


/**
 * @brief Handles the gyro data input for integration.
 * 
 * @param raw_gyro 
 */
void data_handler_gyro_data(int16_t *raw_gyro);

void data_handler_adc_average(void);

void data_handler_gyro_z_angle_reset(void);

void data_handler_adc_average_calculate(void);

void data_handler_calculate_active_zero(void);

void data_handler_calculate_torque_sd_card_interpolated(void);

void data_handler_calculate_torque_sd_card(void);

void data_handler_calculate_torque(void);

void data_handler_accel_average_calculate(void);

void data_handler_adc_average_reset(void);

void data_handler_accel_average_reset(void);

void data_handler_nus_send_average(void);

void data_handler_zero_request(void);

void data_handler_command(const char* p_chars, uint32_t length);

void data_handler_command_processor(void);

float data_handler_command_float_return(uint8_t offset);

int32_t data_handler_command_number_return(uint8_t offset);

void data_handler_command_calibration_value(void);

void data_handler_command_temp_slope_value(void);

void data_handler_command_temp_offset_value(void);

void data_handler_safe_write_time(void);

void data_handler_sch_execute(void);

void data_handler_get_flash_calibration(void);

bool data_handler_averaging(void);

int32_t data_handler_averaging_count(void);

void data_handler_tmp117_handler(int16_t p_temp_data);

#endif