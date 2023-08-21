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
#include "nrf_delay.h"
#include "drv8874.h"
#include "PID_controller.h"

uint32_t mpos_debug_counter = 0;

// #define count_offset 2350 //3.3v
#define default_sin_cos_offset 2078 //Half?
#define default_range 50
static bool update_position = false;
static nrf_saadc_value_t m_buffer_pool[3];

static nrf_saadc_value_t sin_cos[2]; //stores the current value of sin in sin_cos[0] and cos in sin_cos[1]

//these are broken out to individual values because I think it'll be easier to understand
static nrf_saadc_value_t sin_min = 32767;
static nrf_saadc_value_t sin_max;
static nrf_saadc_value_t cos_min = 32767;
static nrf_saadc_value_t cos_max;

static nrf_saadc_value_t sin_avg;
static nrf_saadc_value_t cos_avg;

static int8_t rotation_count = 0;
static double angle_old;

APP_TIMER_DEF(m_repeat_action);

float ble_angle = 180.0f;

void saadc_callback(nrfx_saadc_evt_t const * p_event)
{

    if (p_event->type == NRFX_SAADC_EVT_DONE) //Capture offset calibration complete event
    {
        update_position = true;
    }
    else if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
    {
        NRF_LOG_INFO("Calibration complete");
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
        NRF_SAADC_ACQTIME_10US,
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

    nrfx_saadc_calibrate_offset();
    nrf_delay_ms(20);

    //motor position timer, this is 10hz but really this will be 100 - 200 hz... or more likely 128 or 256, because 2^ maths
    err_code = app_timer_create(&m_repeat_action, APP_TIMER_MODE_REPEATED, mpos_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_repeat_action, 256, NULL); 
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(S_HALL_EN);
    nrf_gpio_pin_clear(S_HALL_EN);
   
}

int16_t mpos_test_convert(void)
{
    nrf_saadc_value_t tempvalue;
    ret_code_t err_code;
    err_code = nrfx_saadc_sample_convert(0, &tempvalue);
    
    if (err_code == NRFX_SUCCESS)
    {
        return(tempvalue);
    } else
    {
        return (-8008);
    }
    return (-8007);
}

void mpos_convert(void)
{
    ret_code_t err_code;
    err_code = nrfx_saadc_buffer_convert(m_buffer_pool, 3); //setup the buffer to convert
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_sample(); //actually sample with interrupt
    APP_ERROR_CHECK(err_code);
    if (err_code == NRFX_ERROR_INVALID_STATE)
    {
        NRF_LOG_ERROR("Event did not complete \r\n");
    }
}

int16_t mpos_average(int16_t min, int16_t max, int16_t range, int16_t defaultValue) {
    int16_t temp_average = (min + max + 1)/2; //average out and hold in temp
    int16_t minRange = defaultValue - range;
    int16_t maxRange = defaultValue + range;
    if (temp_average < minRange || temp_average > maxRange) {
        return defaultValue;
    }
    return temp_average;
}

void mpos_min_max(void)
{
    //finds new min or max
    if(sin_cos[0]>sin_max)
    {
        sin_max = sin_cos[0];
    }
    if(sin_cos[0]<sin_min)
    {
        sin_min = sin_cos[0];
    }
    if(sin_cos[1]>cos_max)
    {
        cos_max = sin_cos[1];
    }
    if(sin_cos[1]<cos_min)
    {
        cos_min = sin_cos[1];
    }

    sin_avg = mpos_average(sin_min, sin_max, default_range, default_sin_cos_offset);
    cos_avg = mpos_average(cos_min, cos_max, default_range, default_sin_cos_offset);
    
}


float angle(int16_t hall_0, int16_t hall_1)
{
    float rotation_angle;
    rotation_angle = (atan2f((float)(hall_0-sin_avg),(float)(hall_1-cos_avg))*180/3.14159265359)+180 ;
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

void mpos_update_angle(float target_angle)
{
    ble_angle = target_angle;
}

void mpos_display_value(void)
{
    if (update_position)
    {
        update_position = false;
        // angle(m_buffer_pool[0], m_buffer_pool(1));
        float temp_angle = angle(m_buffer_pool[0], m_buffer_pool[1]);
        sin_cos[0] = m_buffer_pool[0];
        sin_cos[1] = m_buffer_pool[1];
        // temp_angle += 1;
        // double temp_angle = angle(m_buffer_pool[0], m_buffer_pool[1]);
        mpos_min_max();
        temp_angle += rotation_count*360;
        float drive = pidController(ble_angle,(float)temp_angle);
        // int16_t drive_int = (int16_t)drive;
        mpos_debug_counter++;
        // if (mpos_debug_counter %20 == 0)
        // {
        //     NRF_LOG_INFO("%d, %d, %d, " NRF_LOG_FLOAT_MARKER, m_buffer_pool[0], m_buffer_pool[1],drive_int,NRF_LOG_FLOAT(temp_angle));
        // }
        NRF_LOG_RAW_INFO( NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(temp_angle));
        // NRF_LOG_INFO("%d, %d, %d, %d, %d, %d", m_buffer_pool[0], m_buffer_pool[1], sin_max, sin_min, sin_avg, cos_avg);
        // NRF_LOG_FLUSH();
        // float drive = pidController(180,(float)temp_angle);
        drv8874_drive((int16_t)drive);
    }

}
