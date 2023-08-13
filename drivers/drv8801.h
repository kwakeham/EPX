/**
 * Copyright (c) 2018 Keith Wakeham
 *
 * All rights reserved.
 *
 *
 */

#ifndef DRV8801_H
#define DRV8801_H

#include <stdint.h>
#include "nrfx_pwm.h"
#include "nrf_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief Sensor driver usage.
 *
 *PINS

 *  PHASE   ENABLE  MODE 1  MODE 2  nSLEEP  OUT+    OUT-        Description
 *  1       1       X       X       1       H       L           Forward
 *  0       1       X       X       1       L       H           Reverse
 *  X       0       1       0       1       L       L           Brake (slow decay)
 *  X       0       1       1       1       H       H           Brake (slow decay)
 *  1       0       0       X       1       L       H           Fast-decay synchronous rectification(2)
 *  0       0       0       X       1       H       L           Fast-decay synchronous rectification(2)
 *  X       X       X       X       0       Z       Z           Sleep mode
 */

/**
 * @brief Function for initializing drv8801 Motor Controller.
 *
 * @note This function will initialize the PWM0 and therefore must be set in the config.h
 *
 *
 * @param .
 * @return
 */
void drv8801_init(void);

void drv8801_test(void);

void drv8801_test2(void);

void drv8801_drive(int16_t drv8801_duty);

#ifdef __cplusplus
}
#endif

#endif // DRV8801_H
