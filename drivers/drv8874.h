/**
 * Copyright (c) 2018 Keith Wakeham
 *
 * All rights reserved.
 *
 *
 */

#ifndef DRV8874_H
#define DRV8874_H

#include <stdint.h>
#include "nrfx_pwm.h"
#include "nrf_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief Sensor driver usage in PWM mode
 *
 *PINS

 *  nSleep  IN1     IN2     OUT1    OUT2    Description
 *  0       X       X       Hi-Z    Hi-Z    Sleep, (H-Bridge Hi-Z)
 *  1       0       0       Hi-Z    Hi-Z    Coast, (H-Bridge Hi-Z)
 *  1       0       1       L       H       Reverse (OUT2 -> OUT1)
 *  1       1       0       H       L       Forward (OUT1 -> OUT2)
 *  1       1       1       L       L       Brake
 */

/**
 * @brief Function for initializing drv8874 Motor Controller.
 *
 * @note This function will initialize the PWM0 and therefore must be set in the config.h
 * 
 */
void drv8874_init(void);

void drv8874_drive(int16_t drv8874_duty);

void drv8874_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // DRV8801_H
