/**
 * @file mpos.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief This uses the ADC to get the motor positions but auxillary function of getting the the motor current
 * @version 0.1
 * @date 2023-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "mpos.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "app_timer.h"

// #define count_offset 2350 //3.3v
#define count_offset 2048 //Half?
static nrf_saadc_value_t m_buffer_pool[3];
static uint8_t rotation_count = 0;
static double angle_old;

APP_TIMER_DEF(m_repeat_action);

void saadc_callback(nrfx_saadc_evt_t const * p_event)
{

    if (p_event->type == NRFX_SAADC_EVT_DONE) //Capture offset calibration complete event
    {
 
    }
    else if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
    {
        
    }
}

void mpos_timer_handler(void *p_context)
{
    mpos_convert(); //do a converstion
}


void mpos_init(void)
{
    NRF_LOG_INFO("MPOS init");
    ret_code_t err_code;
    nrfx_saadc_config_t saadc_config;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT; //need to manually set the resolution or else it'll default to 8 bit
    // saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED; // default is 4 sample over sampling so need to override that.
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_4X; // default is 4 sample over sampling so need to override that.
    saadc_config.interrupt_priority = 5; //Hmmm?
    saadc_config.low_power_mode = false;

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t chan_config = {
        NRF_SAADC_RESISTOR_DISABLED,                //P
        NRF_SAADC_RESISTOR_DISABLED,                //N
        NRF_SAADC_GAIN1_4,
        NRF_SAADC_REFERENCE_VDD4,
        NRF_SAADC_ACQTIME_40US,
        NRF_SAADC_MODE_SINGLE_ENDED,
        NRF_SAADC_BURST_ENABLED,                    //required to for auto - oversampling
        NRF_SAADC_INPUT_DISABLED,   
        NRF_SAADC_INPUT_DISABLED
    };

    //Some interesting behaviour here. If you use identical configurations and init channels, you can't oversample using burst mode
    //But if you use the same channel config you can. Remember BURST = 1 and Multiple channels to capture averaged.
    chan_config.pin_p = NRF_SAADC_INPUT_AIN2; //sin
    nrfx_saadc_channel_init(0, &chan_config);
    APP_ERROR_CHECK(err_code);

    chan_config.pin_p = NRF_SAADC_INPUT_AIN0; //cos
    nrfx_saadc_channel_init(1, &chan_config);
    APP_ERROR_CHECK(err_code);

    chan_config.pin_p = NRF_SAADC_INPUT_AIN1; //isense
    nrfx_saadc_channel_init(3, &chan_config);
    APP_ERROR_CHECK(err_code);

    //mpositoin timer
    err_code = app_timer_create(&m_repeat_action, APP_TIMER_MODE_REPEATED, mpos_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_repeat_action, 3277, NULL); 
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(S_HALL_EN);
    nrf_gpio_pin_clear(S_HALL_EN);
   
}

int16_t mpos_test_convert(void)
{
    nrf_saadc_value_t duckman;
    ret_code_t err_code;
    err_code = nrfx_saadc_sample_convert(0, &duckman);
    
    if (err_code == NRFX_SUCCESS)
    {
        return(duckman);
    } else
    {
        return (-8008);
    }
    return (-8007);
}

void mpos_convert(void)
{
    ret_code_t err_code;
    err_code = nrfx_saadc_buffer_convert(m_buffer_pool, 3);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_sample();
    APP_ERROR_CHECK(err_code);
    if (err_code == NRFX_ERROR_INVALID_STATE)
    {
        NRF_LOG_ERROR("Event did not complete \r\n");
    }
}

float angle(int16_t hall_0, int16_t hall_1)
{
    float rotation_angle;
    rotation_angle = (atan2f((float)(hall_0-count_offset),(float)(hall_1-count_offset))*180/3.14159265359)+180 ;
    if (angle_old > rotation_angle)
    {
        if ((angle_old- rotation_angle) > 180.0)
        {
            // rotation_count=rotation_count+1;
            rotation_count++;
        }
    } else if (angle_old < rotation_angle)
    {
        if ((rotation_angle-angle_old) > 180.0)
        {
            rotation_count--;
        }
    }
    angle_old = rotation_angle;
    return(rotation_angle);
}

void mpos_display_value(void)
{
    // angle(m_buffer_pool[0], m_buffer_pool(1));
    double temp_angle = angle(m_buffer_pool[0], m_buffer_pool[1]);
    NRF_LOG_INFO("%d, %d, %i, " NRF_LOG_FLOAT_MARKER, m_buffer_pool[0], m_buffer_pool[1],m_buffer_pool[2],NRF_LOG_FLOAT(temp_angle));
    NRF_LOG_FLUSH();
}
