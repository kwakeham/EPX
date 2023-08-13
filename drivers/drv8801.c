/**
 * Copyright (c) 2018 Keith Wakeham
 *
 */

#include "drv8801.h"
#include "cx.h"

static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_common_t seq_values[] = {100, 100};
static nrf_pwm_values_common_t seq_values2[] = {0,100,200,300,400,500,600,700,800,900,1000,1000,1000,800,600,400,200,0};

// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
#define USED_PWM(idx) (1UL << idx)
// static uint8_t m_used = 0;

void drv8801_init(void)
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

void drv8801_test(void){
    nrf_gpio_pin_set(M_PHASE);
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 0,
        .end_delay       = 0
    };

    nrfx_pwm_simple_playback(&m_pwm0, &seq, 2000, NRFX_PWM_FLAG_STOP);
    // nrfx_pwm_stop(&m_pwm0, 0);
}

void drv8801_test2(void){
    nrf_gpio_pin_clear(M_PHASE);
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values2,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values2),
        .repeats         = 50,
        .end_delay       = 0
    };

    nrfx_pwm_simple_playback(&m_pwm0, &seq, 4, NRFX_PWM_FLAG_STOP);
    // nrfx_pwm_stop(&m_pwm0, 0);
}

void drv8801_drive(int16_t drv8801_duty){
    uint16_t temp_duty[1] = {0};
    if (drv8801_duty > 0)
    {
        nrf_gpio_pin_clear(M_PHASE);
        temp_duty[0] = drv8801_duty;
        nrf_pwm_sequence_t const seq_temp =
        {
            .values.p_common = temp_duty,
            .length          = 1,
            .repeats         = 50,
            .end_delay       = 0
        };
    nrfx_pwm_simple_playback(&m_pwm0, &seq_temp, 1, 0);
    } else
    {
        nrf_gpio_pin_set(M_PHASE);
        // drv8801_duty = 
        temp_duty[0] = -drv8801_duty;
        // nrf_pwm_values_common_t seq_values_temp[] = {1000};
        nrf_pwm_sequence_t const seq_temp =
        {
            .values.p_common = temp_duty,
            .length          = 1,
            .repeats         = 50,
            .end_delay       = 0
        };
        nrfx_pwm_simple_playback(&m_pwm0, &seq_temp, 1, 0);
    }
}