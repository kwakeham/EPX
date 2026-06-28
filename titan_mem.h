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
#include "derailleur.h"

/* Bump when the layout/semantics of epx_configuration_t change. On load, a
 * mismatch triggers a reset to defaults (see tm_fds_config_init). */
#define CONFIG_VERSION 2

/* A dummy structure to save in flash. */
typedef struct
{
    uint32_t config_version; //must be first; gates migration

    //Configuration
    int32_t num_gears;

    //gear settings (first NUM_REAR_GEARS used; array kept at 14 for layout stability)
    int32_t gear_pos[14];

    //captured calibration reference points (so a re-fit needs no re-jog)
    int32_t ref_lo;
    int32_t ref_hi;
    uint8_t ref_lo_idx;
    uint8_t ref_hi_idx;

    //PID Gains
    float Kp;
    float Ki;
    float Kd;

    int16_t sin_min; //averages can be calculated from this
    int16_t sin_max;
    int16_t cos_min;
    int16_t cos_max;

    //overshift/dwell per rear gear, per front position, per direction (up/down)
    overshift_t rear_overshift[NUM_REAR_GEARS][NUM_FRONT_POS][NUM_DIRS];

    //front derailleur provisioning (data only; no actuation yet)
    int32_t     front_pos[NUM_FRONT_POS];
    overshift_t front_overshift[NUM_FRONT_POS][NUM_DIRS];

    //overcurrent protection
    int16_t  isense_limit;        //raw SAADC counts; 0 disables the check
    uint16_t isense_fault_count;  //consecutive over-limit samples before faulting

} epx_configuration_t;

/* A dummy structure to save in flash. */
typedef struct
{
    //current information for sleep restore data
    int32_t current_rotations;
    int32_t target_angle;
    int8_t current_gear;
    int8_t current_front;   //selected chainring (0/1); provision

    //Historic data
    uint32_t upshifts;
    uint32_t downshifts;

} epx_position_configuration_t;

epx_configuration_t tm_fds_epx_config();

epx_position_configuration_t tm_fds_epx_position();

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

/**
 * @brief Force a update to the config file on flash
 * run garbage collection, check if the record exists, write it
 * 
 */
void tm_fds_config_update();

/**
 * @brief Force a update to the position file on flash
 * run garbage collection, check if the record exists, write it
 * 
 */
void tm_fds_position_update();

/**
 * @brief This will trigger a new Flash memory write of the epx_configuration file
 * 
 * This updates the titan_mem variables, kept seperate from the position variables, and requests a write from tm_fds_config_update
 * which is what will actually do the update
 * 
 * @param config_towrite 
 */
void mem_epx_config_update(epx_configuration_t config_towrite);

/**
 * @brief This will trigger a new Flash memory write of the epx_position_configuration_t file
 * 
 * This updates the titan_mem positions, kept seperate from the configuration variables, and requests a write from tm_fds_position_update
 * which is what will actually do the update
 * 
 * @param config_towrite 
 */
void mem_epx_position_update(epx_position_configuration_t position_config_towrite);

#endif