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
    
    //gear settings
    int32_t gear_pos[14];
  
    //PID Gains
    float Kp;
    float Ki;
    float Kd;

    //current information for sleep restore data
    int32_t current_rotations;
    int32_t current_angle;
    uint8_t current_gear;

    //Historic data
    uint32_t upshifts;
    uint32_t downshifts;

    int16_t sin_min; //averages can be calculated from this
    int16_t sin_max;
    int16_t cos_min;
    int16_t cos_max;

} epx_configuration_t;

/* A dummy structure to save in flash. */
typedef struct
{
    //current information for sleep restore data
    int32_t current_rotations;
    int32_t current_angle;
    uint8_t current_gear;

    //Historic data
    uint32_t upshifts;
    uint32_t downshifts;

} epx_sleep_configuration_t;

epx_configuration_t tm_fds_epx_config ();

void tm_fds_init();

void tm_fds_test_write();

void tm_fds_test_retrieve();

void tm_fds_test_delete();

/**
 * @brief If there are more than 60 dirty records it'll do a cleanup that takes a little while.
 * Why 60, seemed like a fine number. Depending on record length and area this might need to be changed
 * 
 */
void tm_fds_gc();

/**
 * @brief This will do the actual memory system writing and will check the garbage collection tm_fds_gc()
 * to see if it needs to run
 * 
 */
void tm_fds_config_init();

void tm_fds_config_update();

/**
 * @brief This will trigger a new Flash memory write of the epx_configuration file
 * 
 * This updates the titan_mem variables, kept seperate from other variables, and requests a write from tm_fds_config_update
 * which is what will actually do the update
 * 
 * @param config_towrite 
 */
void mem_epx_update(epx_configuration_t config_towrite);

#endif