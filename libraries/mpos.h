/**
 * @file mpos.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief header for mpos.c
 * @version 0.1
 * @date 2023-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MPOS_H
#define MPOS_H

#include <stdint.h>
#include <math.h>
#include <inttypes.h>
#include "nrfx_saadc.h"
#include "app_error.h"
#include "titan_mem.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 
 * 
 */
typedef void (* voidfunctionptr_t)(void);


void saadc_callback(nrfx_saadc_evt_t const * p_event);

/**
 * @brief Initialize the SAADC and setup oversampling and burst
 * 
 */
void mpos_init(voidfunctionptr_t pos_save_callback);

/**
 * @brief Will be removed, test conversion and write
 * 
 * @return int16_t 
 */
int16_t mpos_test_convert(void);

/**
 * @brief Request a conversion request
 * 
 */
void mpos_convert(void);

/**
 * @brief 
 * 
 * @param min 
 * @param max 
 * @param defaultValue 
 * @return int 
 */
int16_t mpos_average(int16_t min, int16_t max, int16_t range, int16_t defaultValue);

/**
 * @brief Calculates and updates the min/max values
 * 
 */
void mpos_min_max(void);

float angle(int16_t hall_0, int16_t hall_1);

/**
 * @brief temporary to update the target angle from BLE
 * 
 * @param direct is this the direct angle (true) or is it incremental (false)
 * @param new_target_angle The direct angle or the increment
 */
void mpos_update_angle(bool direct, float new_target_angle);

/**
 * @brief 
 * 
 * @return float of the current angle
 */
float mpos_calculate_angle(void);

/**
 * @brief 
 * 
 * @param drive 
 * @param current_angle 
 */
void manage_motor_state(float *drive, float current_angle);

/**
 * @brief 
 * 
 * @param drive 
 */
void handle_motor_movement_mode(float *drive);

/**
 * @brief 
 * 
 * @param drive 
 * @param current_angle 
 */
void handle_motor_idle_mode(float *drive, float current_angle);

/**
 * @brief 
 * 
 * @param drive 
 * @return true 
 * @return false 
 */
bool should_sleep_motor(float drive);

/**
 * @brief 
 * 
 */
void sleep_motor(void);

/**
 * @brief 
 * 
 */
void reset_sleep_count(void);

/**
 * @brief 
 * 
 */
void wake_motor(void);

/**
 * @brief 
 * 
 */
bool mpos_angle_in_threshold(float current_angle);

void mpos_motor_drive(void);

void move_deraileur(float drive);

/**
 * @brief Returns if the angle is within +/- threshold of Define ANGLE_THRESHOLD
 *
 * @param ref_angle
 * @return true ref_angle is less than threshold
 * @return false re_angle greater than threshold
 */
bool mpos_angle_in_threshold(float ref_angle);

/**
 * @brief link memory so that the mpos will know motor information
 * 
 * @param temp_link_epx_values 
 */
void mpos_link_memory(epx_position_configuration_t *temp_link_epx_values);

void mpos_sincos_debug(void);

void mpos_wake_debug(void);

#ifdef __cplusplus
}
#endif

#endif // MPOS_H