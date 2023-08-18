/**
 * @file drv8874.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief This is a general motor driver for use with Nordic semiconductor and most TI DRV series, specifically 8874
 * @version 0.1
 * @date 2023-08-17
 * 
 * @copyright Copyright (c) 2023
 * 
 * MOAR POWERRRRRRRRR (I don't need it but why not)
 */

#include "drv8874.h"
#include "boards.h"


void drv8874_init(void)
{
    uint32_t err_code;

    nrf_gpio_cfg_output(M_PHASE);

    nrf_gpio_pin_set(M_PHASE);

    nrfx_pwm_config_t const config0 =
    {
        .output_pins =
        {
            M_EN| NRFX_PWM_PIN_INVERTED,         // channel 0
            NRFX_PWM_PIN_NOT_USED,               // channel 1
            NRFX_PWM_PIN_NOT_USED,               // channel 2
            NRFX_PWM_PIN_NOT_USED,               // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 1000,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrfx_pwm_init(&m_pwm0, &config0, NULL);
    if (err_code != NRF_SUCCESS)
    {
        // Initialization failed. Take recovery action.
    }
    return;
}