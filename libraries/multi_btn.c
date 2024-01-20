/**
 * @file multi_btn.c
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Rewrite of a other internal libraries from Titan lab related to buttons
 * @version 0.1
 * @date 2024-01-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "app_error.h"
#include "app_timer.h"
#include "app_button.h"
#include "boards.h"
// #include "nrf_log.h"
#include "sdk_config.h"
#include "multi_btn.h"
#include "app_util_platform.h"



#define NRF_LOG_MODULE_NAME multibut
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

//Button intervals
#define LONGPRESS_INTERVAL_MS        500UL
#define LONGPRESS_TIMER_INTERVAL_MS        10UL //10ms makes it simple
#define LONGPRESS_TICKS LONGPRESS_INTERVAL_MS/LONGPRESS_TIMER_INTERVAL_MS
#define DEBOUNCE_TIME_MS        20UL

//Buttons defined via SDK in the customboard.h, to change
#define MULTI_BTN_PIN_CH1      BUTTON_1
#define MULTI_BTN_PIN_CH2      BUTTON_2
#define MULTI_BTN_PIN_CH3      BUTTON_3
#define MULTI_BTN_PIN_CH4      BUTTON_4

//Pullup will be 10 - 15k, this is okay for some stuff but not others.
//For this it's fine but it means 0.2ma drain per button, that's kinda meaningful for coin but less for this
static app_button_cfg_t  button_cfg[BUTTONS_NUMBER]= {
	{MULTI_BTN_PIN_CH1, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_callback}, 
	{MULTI_BTN_PIN_CH2, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_callback},
	{MULTI_BTN_PIN_CH3, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_callback},
	{MULTI_BTN_PIN_CH4, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_callback},
};

static void _default_callback(multibtn_event_t evt) {}

static multibtn_event_callback_t   m_registered_callback = &_default_callback;

//static app_button_cfg_t  button_cfg;
static uint8_t btn_hold_count[4] = {0,0,0,0};
static bool button_pressed[4] = {0,0,0,0};
static bool run_long_timer = false; //Should we run the timer for checking longs NOTE: I think this can be removed

static uint8_t is_button_init=0;

APP_TIMER_DEF(m_button_timer);
bool m_button_timer_running = false;


void button_timeout_handler (void * p_context)
{
	// uint32_t err_code;

	//Loop buttons and check if they are pushed
	for (size_t i = 0; i < 4; i++)
	{
		button_pressed[i] = app_button_is_pushed(i);
		//If the button is pushed 
		if (button_pressed[i])
		{
			run_long_timer = true; //if any button is pressed ensure to run the timer
			btn_hold_count[i]++;
			if(btn_hold_count[i]>(LONGPRESS_TICKS))
			{
				btn_hold_count[i] -= LONGPRESS_TICKS;
				NRF_LOG_DEBUG("LONG PRESS CH%d",(i+1));
				m_registered_callback(MULTI_BTN_EVENT_CH1_LONG + i); //
			}

		} else
		{
			btn_hold_count[i] = 0; //if the button is released, reset the count
		}
	}
	multi_button_timer_run_check();
}

// called by the app_button only
void button_callback(uint8_t pin_no, uint8_t button_action)
{
	if (button_action == APP_BUTTON_PUSH) {

		// run timer to check it's status
		run_long_timer = true;
		switch (pin_no) {

			case MULTI_BTN_PIN_CH1:
				button_pressed[0] = true;
				m_registered_callback(MULTI_BTN_EVENT_CH1_PUSH);
				NRF_LOG_DEBUG("CH1 Press");
				break;

			case MULTI_BTN_PIN_CH2:
				button_pressed[1] = true;
				m_registered_callback(MULTI_BTN_EVENT_CH2_PUSH);
				NRF_LOG_DEBUG("CH2 Press");
				break;

			case MULTI_BTN_PIN_CH3:
				button_pressed[2] = true;
				m_registered_callback(MULTI_BTN_EVENT_CH3_PUSH);
				NRF_LOG_DEBUG("CH3 Press");
				break;

			case MULTI_BTN_PIN_CH4:
				button_pressed[3] = true;
				m_registered_callback(MULTI_BTN_EVENT_CH4_PUSH);
				NRF_LOG_DEBUG("CH4 Press");
				break;

			default:
			break;
		}
	}

	if (button_action == APP_BUTTON_RELEASE)
	{
		switch (pin_no) {

			case MULTI_BTN_PIN_CH1:
				button_pressed[0] = false;
				m_registered_callback(MULTI_BTN_EVENT_CH1_RELEASE);
				NRF_LOG_DEBUG("CH1 Released");
				break;

			case MULTI_BTN_PIN_CH2:
				button_pressed[1] = false;
				m_registered_callback(MULTI_BTN_EVENT_CH2_RELEASE);
				NRF_LOG_DEBUG("CH2 Released");
				break;

			case MULTI_BTN_PIN_CH3:
				button_pressed[2] = false;
				m_registered_callback(MULTI_BTN_EVENT_CH3_RELEASE);
				NRF_LOG_DEBUG("CH3 Released");
				break;

			case MULTI_BTN_PIN_CH4:
				button_pressed[3] = false;
				m_registered_callback(MULTI_BTN_EVENT_CH4_RELEASE);
				NRF_LOG_DEBUG("CH4 Released");
				break;

			default:
			break;
		}
	}
	multi_button_timer_run_check();

}

void multi_buttons_init(multibtn_event_callback_t callback)
{
	uint32_t err_code;

	// Register a callback
	m_registered_callback = callback;

	// If button isn't initialized then initialize, to prevent re-initializing the buttons except at startup
	if (is_button_init == 0) {
		is_button_init++;
		err_code = app_button_init(button_cfg, sizeof(button_cfg) / sizeof(button_cfg[0]), APP_TIMER_TICKS(DEBOUNCE_TIME_MS));
		APP_ERROR_CHECK(err_code);
	}

	err_code = app_button_enable();
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("MULTI app button init");
	NRF_LOG_INFO("%d", APP_TIMER_TICKS(LONGPRESS_TIMER_INTERVAL_MS));

	// reset state variables
	multi_reset_buttons();

	//this is the timer for checking long presses
    err_code = app_timer_create(&m_button_timer, APP_TIMER_MODE_REPEATED, button_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void multi_reset_buttons(void) 
{
	for (size_t i = 0; i < 4; i++)
	{
		btn_hold_count[i] = 0;
		button_pressed[i] = 0;
	}
	run_long_timer = false; //clear flag
	app_timer_stop(m_button_timer);
	m_button_timer_running = false;
}

void multi_button_timer_run_check(void)
{
	ret_code_t err_code;
	if(!button_pressed[0] && !button_pressed[1] && !button_pressed[2] && !button_pressed[3])//if all buttons are not pressed...
	{
		NRF_LOG_DEBUG("no buttons multi_reset_buttons()");
		multi_reset_buttons(); //reset everything, it'll overwrite button_pressed but that's fine, a wasted few clocks
	} else
	{
		if (!m_button_timer_running) //if the timer isn't runnings
		{	
			NRF_LOG_DEBUG("timerrun");
			m_button_timer_running = true;
			err_code = app_timer_start(m_button_timer, APP_TIMER_TICKS(LONGPRESS_TIMER_INTERVAL_MS), NULL); //This starts timer to check if Long press is active
			APP_ERROR_CHECK(err_code);
			
		}
	}
}

void multi_buttons_tasks(void) {

	// if (run_long_timer) {
	// 	button_timeout_handler(NULL);
	// }
}

void multi_buttons_disable()
{
	uint32_t err_code;
	err_code = app_button_disable();
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("MULTI buttons disable");
}
