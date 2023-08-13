/**
 * Copyright (c) 2018-2023 Titan Lab Inc.
 *
 * All rights reserved.
 *
 *
 */

#include "mpos.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"

#define count_offset 2350 //3.3v
static nrf_saadc_value_t m_buffer_pool[3];
static uint8_t rotation_count = 0;
static double angle_old;

void saadc_callback(nrfx_saadc_evt_t const * p_event)
{

    if (p_event->type == NRFX_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
 
    }
    else if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
    {
        
    }
}


void mpos_init(void)
{
    NRF_LOG_INFO("MPOS init");
    ret_code_t err_code;
    nrfx_saadc_config_t saadc_config;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT; //need to manually set the resolution or else it'll default to 8 bit
    // saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED; // default is 4 sample over sampling so need to override that.
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_8X; // default is 4 sample over sampling so need to override that.
    saadc_config.interrupt_priority = 5; //Hmmm?
    saadc_config.low_power_mode = false;

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t channel_config_sin = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0); //Real = Ain4
    channel_config_sin.gain = NRF_SAADC_GAIN1_5; // this is measured against either vdd/4 or vcore = 0.6v.

    nrf_saadc_channel_config_t channel_config_cos = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2); //Real = Ain5
    channel_config_cos.gain = NRF_SAADC_GAIN1_5; // this is measured against either vdd/4 or vcore = 0.6v.

    nrf_saadc_channel_config_t channel_config_isense = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1); //Real = Ain5
    channel_config_cos.gain = NRF_SAADC_GAIN1_5; // this is measured against either vdd/4 or vcore = 0.6v.

    nrfx_saadc_channel_init(0, &channel_config_sin);
    APP_ERROR_CHECK(err_code);

    nrfx_saadc_channel_init(1, &channel_config_cos);
    APP_ERROR_CHECK(err_code);

    nrfx_saadc_channel_init(3, &channel_config_isense);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool, 3);
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

void display_value(void)
{
    // angle(m_buffer_pool[0], m_buffer_pool(1));
    double temp_angle = angle(m_buffer_pool[0], m_buffer_pool[1]);
    NRF_LOG_INFO("%d, %d, %i, " NRF_LOG_FLOAT_MARKER, m_buffer_pool[0], m_buffer_pool[1],rotation_count,NRF_LOG_FLOAT(temp_angle));
    NRF_LOG_FLUSH();
}
