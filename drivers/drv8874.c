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

static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);

static uint16_t const              pwm_top_period  = 400;//25 us for 16 MHz clock
static nrf_pwm_values_individual_t pwm_values;
static nrf_pwm_sequence_t const    pwm_playback =
{
    .values.p_individual = &pwm_values,
    .length              = NRF_PWM_VALUES_LENGTH(pwm_values),
    .repeats             = 0,
    .end_delay           = 0
};

void drv8874_init(void)
{
    uint32_t err_code;

    //zero out all the starting values so they don't do squat
    pwm_values.channel_0 = 0;
    pwm_values.channel_1 = 0;
    pwm_values.channel_2 = 0;
    pwm_values.channel_3 = 0;

    nrfx_pwm_config_t const config0 =
    {
        .output_pins =
        {
            M_IN1| NRFX_PWM_PIN_INVERTED,        // channel 0
            M_IN2| NRFX_PWM_PIN_INVERTED,        // channel 1
            NRFX_PWM_PIN_NOT_USED,               // channel 2
            NRFX_PWM_PIN_NOT_USED,               // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = pwm_top_period,
        .load_mode    = PWM_DECODER_LOAD_Individual,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrfx_pwm_init(&m_pwm0, &config0, NULL);
    if (err_code != NRF_SUCCESS)
    {
        // Initialization failed. Take recovery action.
    }

    nrfx_pwm_simple_playback(&m_pwm0, &pwm_playback, 1,NRFX_PWM_FLAG_LOOP);

    //Set to PWM mode
    nrf_gpio_cfg_output(M_PMode);
    nrf_gpio_pin_set(M_PMode);

    //Wake up the Motor controller
    nrf_gpio_cfg_output(M_nSleep);
    nrf_gpio_pin_set(M_nSleep);

    return;
}

void drv8874_drive(int16_t drv8874_duty){

    if (drv8874_duty > 0)
    {
        pwm_values.channel_0 = 0;
        pwm_values.channel_1 = drv8874_duty;
    } else
    {
        pwm_values.channel_0 = -drv8874_duty;
        pwm_values.channel_1 = 0;
    }
}

void drv8874_nsleep(bool nsleep_req)
{
    nrf_gpio_pin_write(M_nSleep, nsleep_req);
}