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
#include "nrf_gpio.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "drv8874.h"
#include "PID_controller.h"
#include "motor_sm.h"
#include "shift_seq.h"
#include "telemetry.h"

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

#define ANGLE_THRESHOLD 10        // position band half-width (degrees)
#define MPOS_SETTLE_TICKS 64      // ticks in band before sleeping the driver (~0.25 s @ 256 Hz)
#define MPOS_ARRIVE_TICKS 8       // ticks in band before a shift sub-target counts as reached
#define MPOS_DT (1.0f / 256.0f)   // control loop period (seconds), matches sample timer

static bool update_position = false; // Flag to know if a new position has been acquired

// Control + sleep state
static pid_ctrl_t  m_pid;
static motor_sm_t  m_sm;
static shift_seq_t m_seq;                 // overshift/dwell sequencer
static int32_t     m_subtarget = 0;       // position the PID currently chases
static uint16_t    m_arrive_count = 0;    // ticks in band at the current sub-target
static float       m_last_angle = 0.0f;   // last computed angle, for out-of-loop reads
static const float *m_kp = NULL;          // live gains, bound via mpos_link_gains()
static const float *m_ki = NULL;
static const float *m_kd = NULL;

// Overcurrent protection
static const int16_t  *m_isense_limit = NULL;   // live limit, bound via mpos_link_overcurrent()
static const uint16_t *m_isense_count = NULL;
static uint16_t        m_over_count = 0;         // consecutive over-limit samples
static int16_t         m_isense = 0;             // last raw current-sense count
static bool            m_fault = false;          // latched overcurrent/driver fault

static nrf_saadc_value_t m_buffer_pool[3]; //temporary Adc storage in sin, cos, isense order

static nrf_saadc_value_t sin_cos[2]; //stores the current value of sin in sin_cos[0] and cos in sin_cos[1]

//these are broken out to individual values because I think it'll be easier to understand
// Sin/cos calibration reference (restore these defaults after a flash wipe).
// Defaults below; observed-on-hardware ranges from old logs:
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

void mpos_link_gains(const float *kp, const float *ki, const float *kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void mpos_link_overcurrent(const int16_t *limit, const uint16_t *count)
{
    m_isense_limit = limit;
    m_isense_count = count;
}

int16_t mpos_isense(void)
{
    return m_isense;
}

bool mpos_is_faulted(void)
{
    return m_fault;
}

void mpos_clear_fault(void)
{
    m_fault      = false;
    m_over_count = 0;
}

// Latch a fault if ISENSE exceeds the configured limit for enough consecutive
// samples, or the DRV8874 nFault line is asserted (active low). Returns m_fault.
static bool mpos_check_fault(void)
{
    if (m_isense_limit != NULL && *m_isense_limit > 0 && m_isense > *m_isense_limit)
    {
        m_over_count++;
        uint16_t limit_count = (m_isense_count != NULL) ? *m_isense_count : 1;
        if (m_over_count >= limit_count)
        {
            if (!m_fault) NRF_LOG_WARNING("FAULT overcurrent: isense %d", m_isense);
            m_fault = true;
        }
    }
    else
    {
        m_over_count = 0;
    }

    if (nrf_gpio_pin_read(M_nFault) == 0) // DRV8874 fault output, active low
    {
        if (!m_fault) NRF_LOG_WARNING("FAULT driver nFault asserted");
        m_fault = true;
    }

    return m_fault;
}

float mpos_last_angle(void)
{
    return m_last_angle;
}

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

    // DRV8874 fault output (active low) as input for the overcurrent guard.
    nrf_gpio_cfg_input(M_nFault, NRF_GPIO_PIN_PULLUP);

    // Control + sleep state machine. Gains are bound earlier via mpos_link_gains().
    // Recovered tuning reference (pre-rewrite PID_controller.c, re-tune from here):
    //   Kp ~ 9 @128Hz, 18 @256Hz, 35-40 @512Hz  (old PID summed error; this one
    //   folds dt in, so those numbers are only a starting point -- re-tune).
    //   old limits: integral +/-300, output +/-400, deadband +/-50 (deadband removed).
    //   old motor-sleep was SLEEP_THRESHOLD 600 (~2.3s of low drive); replaced by
    //   settle-on-position MPOS_SETTLE_TICKS (64 ticks ~0.25s in band).
    pid_init(&m_pid, m_kp, m_ki, m_kd, -400.0f, 400.0f, -2000.0f, 2000.0f);
    motor_sm_init(&m_sm, (float)ANGLE_THRESHOLD, MPOS_SETTLE_TICKS);
    shift_seq_init(&m_seq);
    m_subtarget = link_epx_pos->target_angle;  // chase the restored target on boot
    telemetry_init();
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
    if (m_fault) return; // motion inhibited until the fault is cleared

    if(direct)
    {
        link_epx_pos->target_angle = new_target_angle;
    } else
    {
        link_epx_pos->target_angle = link_epx_pos->target_angle + new_target_angle;
    }

    // A direct angle move has no overshift sequence: chase the target directly.
    shift_seq_init(&m_seq);
    m_subtarget    = link_epx_pos->target_angle;
    m_arrive_count = 0;
}

void mpos_shift_to(int32_t final_pos, int16_t signed_overshift, uint16_t dwell_ticks)
{
    if (m_fault) return; // motion inhibited until the fault is cleared

    link_epx_pos->target_angle = final_pos; // persist the resting position

    int32_t first_target;
    shift_seq_start(&m_seq, final_pos, signed_overshift, dwell_ticks, &first_target);
    m_subtarget    = first_target;
    m_arrive_count = 0;
}

float mpos_calculate_angle(void)
{
        sin_cos[0] = m_buffer_pool[0]; //move the buffer_pools to the sin_cos storage
        sin_cos[1] = m_buffer_pool[1];
        m_isense   = m_buffer_pool[2]; //current sense (AIN1), used by the overcurrent guard
        mpos_min_max(); // store min max for average offset
        float current_angle = angle(sin_cos[0], sin_cos[1]); //return angle
        current_angle += (link_epx_pos->current_rotations)*360; //find the total current angle
        return current_angle;
}



void mpos_motor_drive(void)
{
    if (!update_position) return;
    update_position = false; //unset flag

    float current = mpos_calculate_angle();
    m_last_angle  = current;

    // Overcurrent guard: if faulted, kill drive, sleep the driver, abort any
    // shift, and inhibit motion until the fault is cleared.
    if (mpos_check_fault())
    {
        drv8874_drive(0);
        drv8874_nsleep(0);
        shift_seq_init(&m_seq);
        telemetry_tick((float)m_subtarget, current, 0.0f, m_pid.integral, (int)m_sm.state, m_isense, true);
        return;
    }

    // Track arrival at the current sub-target (in-band, debounced).
    if (fabsf((float)m_subtarget - current) < (float)ANGLE_THRESHOLD)
    {
        if (m_arrive_count < 0xFFFF) m_arrive_count++;
    }
    else
    {
        m_arrive_count = 0;
    }
    bool arrived = (m_arrive_count >= MPOS_ARRIVE_TICKS);

    // Advance the overshift/dwell sequence; it may retarget us to the final gear.
    if (shift_seq_active(&m_seq))
    {
        int32_t next_target;
        if (shift_seq_step(&m_seq, arrived, &next_target))
        {
            m_subtarget    = next_target;
            m_arrive_count = 0;
        }
    }

    float target = (float)m_subtarget;
    float drive  = 0.0f;

    if (shift_seq_active(&m_seq))
    {
        // While sequencing, keep driving to the sub-target and never sleep.
        if (m_sm.state != MOTOR_MOVING)
        {
            m_sm.state = MOTOR_MOVING;
            drv8874_nsleep(1);
            pid_reset(&m_pid, current);
        }
        m_sm.settle_count = 0;
        drive = pid_update(&m_pid, target, current, MPOS_DT);
    }
    else
    {
        // Idle/done: normal PID + sleep state machine manages the driver.
        if (m_sm.state == MOTOR_MOVING)
        {
            drive = pid_update(&m_pid, target, current, MPOS_DT);
        }

        motor_action_t act;
        bool driving = motor_sm_step(&m_sm, target, current, &act);
        if (!driving) drive = 0.0f;

        if (act.pid_reset)      pid_reset(&m_pid, current);
        if (act.nsleep_changed) drv8874_nsleep(act.nsleep_value);
        if (act.save_position)  m_registered_pos_save_callback();
    }

    drv8874_drive((int16_t)drive);

    telemetry_tick(target, current, drive, m_pid.integral, (int)m_sm.state, m_isense, m_fault);
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