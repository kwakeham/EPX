/**
 *
 * Flash memory interface Library
 * TITANLAB INC 2019
 * Keith Wakeham
 * 
 * 
 */

#ifndef TITAN_MEM__
#define TITAN_MEM__
#include "nrf.h"
#include "app_timer.h"
#include "nrf_fstorage_sd.h"
#include "nrf_fstorage.h"

/* A dummy structure to save in flash. */
typedef struct
{
    //Configuration
    int32_t num_gears;
    int32_t gear1_pos;

    //sleep restore data
    int32_t sleep_rotations;
    int32_t sleep_angle;

    //Historic data
    uint32_t upshifts;
    uint32_t downshifts;

    int16_t sin_min;
    int16_t sin_max;
    int16_t cos_min;
    int16_t cos_max;
    int32_t CH1_zero;
    int32_t CH2_zero;
    int32_t CH3_zero;
    int32_t CH4_zero;
    float C1x_cal;
    float C2x_cal;
    float C3x_cal;
    float C4x_cal;
    float C1y_cal;
    float C2y_cal;
    float C3y_cal;
    float C4y_cal;
    int32_t CH1_thermal_b;
    int32_t CH2_thermal_b;
    int32_t CH3_thermal_b;
    int32_t CH4_thermal_b;
    float C1_thermal_m;
    float C2_thermal_m;
    float C3_thermal_m;
    float C4_thermal_m;
} epx_configuration_t;

epx_configuration_t tm_fds_epx_config ();

void tm_fds_init();

void tm_fds_test_write();

void tm_fds_test_retrieve();
void tm_fds_test_delete();

void tm_fds_gc();

void tm_fds_config_init();

void tm_fds_config_update();

void mem_epx_update(epx_configuration_t config_towrite);

#endif