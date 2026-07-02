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
 * @brief Run one control tick: read position, drive the motor, advance the
 *        sleep state machine, and emit telemetry. Call from the main loop.
 */
void mpos_motor_drive(void);

/**
 * @brief Bind the PID controller to its live gains. Call before mpos_init().
 */
void mpos_link_gains(const float *kp, const float *ki, const float *kd);

/**
 * @brief Bind the overcurrent guard to the live ISENSE limit / fault-count.
 */
void mpos_link_overcurrent(const int16_t *limit, const uint16_t *count);

/**
 * @brief Shift to a gear with optional overtravel then dwell then settle-back.
 * @param final_pos        resting position for the gear (persisted as target).
 * @param signed_overshift overtravel in the shift direction; 0 = direct, no dwell.
 * @param dwell_ticks      ticks to dwell at the overshift waypoint.
 */
void mpos_shift_to(int32_t final_pos, int16_t signed_overshift, uint16_t dwell_ticks);

/**
 * @brief Command raw open-loop drive for a bounded window (bench system-ID).
 *        Bypasses the PID; @p ticks is a watchdog (in 256 Hz control ticks) that
 *        reverts to holding the current position when it expires. @p drive is
 *        clamped to the PWM range. Any t/s command also exits open-loop.
 */
void mpos_set_open_loop(int16_t drive, uint16_t ticks);

/**
 * @brief Clamp the motor target to [lo, hi] (centi... whole degrees, absolute) so a
 *        bad target/overshoot can't drive past an end stop. hi <= lo disables it.
 *        Set from the calibrated gear table (+ margin) after calibration and boot.
 */
void mpos_set_travel_limits(int32_t lo, int32_t hi);

/** Last raw current-sense (ISENSE/AIN1) count. */
int16_t mpos_isense(void);

/** Motor sleep state (0 = HOLDING, 1 = MOVING). */
int mpos_state(void);

/** Position the controller is currently chasing (overshift sub-target). */
int32_t mpos_subtarget(void);

/** True while an overcurrent/driver fault is latched (motion inhibited). */
bool mpos_is_faulted(void);

/** Clear a latched fault and re-enable motion. */
void mpos_clear_fault(void);

/** Pace the human-readable debug monitor: print every `divider` ticks (0 = off). */
void mpos_set_monitor(uint16_t divider);

/** True (once) when a monitor line is due; clears the flag. Poll from main loop. */
bool mpos_monitor_due(void);

/**
 * @brief Last angle computed by mpos_motor_drive(), with no side effects.
 *        Used by the calibration commands to capture a reference position.
 */
float mpos_last_angle(void);

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