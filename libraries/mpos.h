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

void mpos_init(void);

int16_t mpos_test_convert(void);

void mpos_convert(void);

float angle(int16_t hall_0, int16_t hall_1);

void mpos_display_value(void);

#ifdef __cplusplus
}
#endif

#endif // MPOS_H