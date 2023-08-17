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

#ifdef __cplusplus
extern "C" {
#endif

void saadc_callback(nrfx_saadc_evt_t const * p_event);

/**
 * @brief Initialize the SAADC and setup oversampling and burst
 * 
 */
void mpos_init(void);

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

void mpos_display_value(void);

#ifdef __cplusplus
}
#endif

#endif // MPOS_H