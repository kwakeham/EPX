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
#include "boards.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "drv8874.h"
#include "PID_controller.h"

#define NRF_LOG_MODULE_NAME mpos
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

uint32_t mpos_debug_counter = 0;

// #define count_offset 2350 //3.3v
#define default_sin_cos_offset 2078 //Half?
#define default_range 50

#define SLEEP_THRESHOLD 600
#define ANGLE_THRESHOLD 10
static bool update_position = false; // Flag to know if a new position has been acquired
static bool derailleur_moving = true;
static uint16_t sleep_count = 0;

static nrf_saadc_value_t m_buffer_pool[3]; //temporary Adc storage in sin, cos, isense order

static nrf_saadc_value_t sin_cos[2]; //stores the current value of sin in sin_cos[0] and cos in sin_cos[1]

//these are broken out to individual values because I think it'll be easier to understand
//<info> mpos: sin max, 2515, min, 1522                                        
//<info> mpos: cos max, 2640, min, 1492
static nrf_saadc_value_t sin_min = 1550;
static nrf_saadc_value_t sin_max = 2500;
static nrf_saadc_value_t cos_min = 1500;
static nrf_saadc_value_t cos_max = 2600;

static nrf_saadc_value_t sin_avg = 2030;
static nrf_saadc_value_t cos_avg = 2030;

// static int8_t rotation_count = 0; //TODO get this from epx sleep configuration
static epx_position_configuration_t *link_epx_pos = NULL;
static float angle_old; // last angle to keep track of if we need to add or subtract an angle

static void _default_pos_save_callback(void) {}
static voidfunctionptr_t  m_registered_pos_save_callback = &_default_pos_save_callback;

APP_TIMER_DEF(m_repeat_action);

uint16_t wake_debug = 0;

// float ble_angle = 180.0f;

void saadc_callback(nrfx_saadc_evt_t const * p_event)
{

    if (p_event->type == NRFX_SAADC_EVT_DONE) //Capture offset calibration complete event
    {
        update_position = true; //set the flag to update the position
        nrf_gpio_pin_set(S_HALL_EN);  //Turn off the Hall effect sensors to save power
    }
    else if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
    {
        NRF_LOG_INFO("Calibration complete");
    }
}

void mpos_timer_handler(void *p_context)
{
    // ret_code_t err_code;
    nrf_gpio_pin_clear(S_HALL_EN); //Enable the hall effect sensors, this starts drawing current
    mpos_convert(); //do a converstion
}

void mpos_init(voidfunctionptr_t pos_save_callback) //Initialize the SAADC and timers for sampling
{
    NRF_LOG_INFO("MPOS init");
    ret_code_t err_code;

    m_registered_pos_save_callback = pos_save_callback;

    nrfx_saadc_config_t saadc_config;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT; //need to manually set the resolution or else it'll default to 8 bit
    // saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED; // default is 4 sample over sampling so need to override that.
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_16X; // default is 4 sample over sampling so need to override that.
    saadc_config.interrupt_priority = 5; //Hmmm?
    saadc_config.low_power_mode = false;

    err_code = nrfx_saadc_init(&saadc_config, saadc_callback); //this callback handles the peripheral when it's done
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

    err_code = app_timer_start(m_repeat_action, 128, NULL); //This starts the sampling timer for mpos_timer_handler -- 32768/128 = 256 hz
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

void mpos_convert(void) //this queues the peripheral to sample the data autonomously
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
        if ((angle_old - rotation_angle) > 180.0)
        {
            link_epx_pos->current_rotations++;
        }
    } else if (angle_old < rotation_angle)
    {
        if ((rotation_angle - angle_old) > 180.0)
        {
            link_epx_pos->current_rotations--;
        }
    }
    angle_old = rotation_angle;
    return(rotation_angle);
}

void mpos_update_angle(bool direct, float new_target_angle)
{
    if(direct)
    {
        link_epx_pos->target_angle = new_target_angle;
    } else
    {
        link_epx_pos->target_angle = link_epx_pos->target_angle + new_target_angle;
    }

}

float mpos_calculate_angle(void)
{
        sin_cos[0] = m_buffer_pool[0]; //move the buffer_pools to the sin_cos storage
        sin_cos[1] = m_buffer_pool[1];
        mpos_min_max(); // store min max for average offset
        float current_angle = angle(sin_cos[0], sin_cos[1]); //return angle
        current_angle += (link_epx_pos->current_rotations)*360; //find the total current angle
        return current_angle;
}



//This needs a name update TBD
void mpos_motor_drive(void)
{
    if (!update_position) return;

    update_position = false; //unset flag
    float current_angle = mpos_calculate_angle();
    float drive = pidController((link_epx_pos->target_angle),(float)current_angle);

    if (derailleur_moving) //if we are derailleur_moving
    {
        should_sleep_motor(drive);
    }
    else //not derailleur_moving
    {
        drive = 0.0f;                                // Override and set the drive strength of the motor to 0 just in case
        if (!mpos_angle_in_threshold(current_angle)) // Power up if the derailleur moved while sleeping for some reason
        {
            NRF_LOG_INFO("Wake up the motor driver"); // debug statement for testing
            derailleur_moving = true;                 // if the drive strength is large then on the next
            drv8874_nsleep(1);                        // wake the motor driver since the next time around we'll have to drive it.
        }
    }
    drv8874_drive((int16_t)drive);
}

void should_sleep_motor(float drive)
{
    if (fabs(drive) < 20.0f) // is our drive strength low
    {
        // if we're basically at the location because drive strength is low
        sleep_count++;                     // sleep counter
        if (sleep_count > SLEEP_THRESHOLD) // if we're above the threshold then we successfully moved the derailleur to position and it's stable, ready to sleep the motor driver and leave movement mode
        {
            sleep_motor();
        }
    }
    else // if we're outside, like in adjustment mode we don't want to sleep, so if drive strength too high reset the sleep count
    {
        sleep_count = 0;
    }
}

void sleep_motor(void)
{
    NRF_LOG_INFO("Sleep the motor driver");
    derailleur_moving = false;
    drv8874_nsleep(0);
    sleep_count = 0;
    m_registered_pos_save_callback();
}

bool mpos_angle_in_threshold(float ref_angle)
{   
    int16_t angle_difference = link_epx_pos->target_angle - ref_angle;
    return (abs(angle_difference) < ANGLE_THRESHOLD); //
}

void mpos_link_memory(epx_position_configuration_t *temp_link_epx_values)
{
    link_epx_pos = temp_link_epx_values;
    angle_old = (link_epx_pos->target_angle) % 360;
}

void mpos_sincos_debug(void)
{
    NRF_LOG_INFO("sin max, %d, min, %d",sin_max, sin_min);
    NRF_LOG_INFO("cos max, %d, min, %d",cos_max, cos_min);
}

void mpos_wake_debug(void)
{
    wake_debug++;
    if(wake_debug<5)
    {
        NRF_LOG_INFO("%d, %d, %d", link_epx_pos->target_angle, link_epx_pos->current_gear, link_epx_pos->current_rotations);
    }

}