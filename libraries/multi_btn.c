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
#include "nrf_log.h"
#include "sdk_config.h"
#include "multi_btn.h"
#include "app_util_platform.h"


#define MULTI_PRESS_INTERVAL_MS        600UL
#define DEBOUNCE_TIME_MS        20UL

//Buttons defined via SDK, to change
#define MULTI_BTN_PIN_CH1      BUTTON_1
#define MULTI_BTN_PIN_CH2      BUTTON_2
#define MULTI_BTN_PIN_CH3      BUTTON_3
#define MULTI_BTN_PIN_CH4      BUTTON_4

//Pullup will be 10 - 15k, this is okay for some stuff but not others
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



static bool timer_run = false;
static uint8_t is_button_init=0;

APP_TIMER_DEF(m_button_timer);

void button_timeout_handler (void * p_context)
{
	// uint32_t err_code;
	bool BTN_pushed[4];

	//Loop buttons and check if they are pushed
	for (size_t i = 0; i < 4; i++)
	{
		BTN_pushed[i] = app_button_is_pushed(i);
		//If the button is pushed 
		if (BTN_pushed[i])
		{

		}
	}

}

// called by the app_button only
void button_callback(uint8_t pin_no, uint8_t button_action)
{

	if (button_action == APP_BUTTON_PUSH) {

		// run timer to check it's status
		timer_run = true;

		// NRF_LOG_INFO("Button %d pushed !", pin_no);

		switch (pin_no) {

			case MULTI_BTN_PIN_CH1:
				// m_registered_callback(MULTI_BTN_EVENT_CH1_PUSH);
				NRF_LOG_INFO("CH1 Press");
				break;

			case MULTI_BTN_PIN_CH2:
				// m_registered_callback(MULTI_BTN_EVENT_CH2_PUSH);
				NRF_LOG_INFO("CH2 Press");
				break;

			case MULTI_BTN_PIN_CH3:
				// m_registered_callback(MULTI_BTN_EVENT_CH3_PUSH);
				NRF_LOG_INFO("CH3 Press");
				break;

			case MULTI_BTN_PIN_CH4:
				// m_registered_callback(MULTI_BTN_EVENT_CH4_PUSH);
				NRF_LOG_INFO("CH4 Press");
				break;

			default:
			break;
		}
	}

	if (button_action == APP_BUTTON_RELEASE)
	{
		switch (pin_no) {

			case MULTI_BTN_PIN_CH1:
				// m_registered_callback(MULTI_BTN_EVENT_CH1_PUSH);
				NRF_LOG_INFO("CH1 Released");
				break;

			case MULTI_BTN_PIN_CH2:
				// m_registered_callback(MULTI_BTN_EVENT_CH2_PUSH);
				NRF_LOG_INFO("CH2 Released");
				break;

			case MULTI_BTN_PIN_CH3:
				// m_registered_callback(MULTI_BTN_EVENT_CH3_PUSH);
				NRF_LOG_INFO("CH3 Released");
				break;

			case MULTI_BTN_PIN_CH4:
				// m_registered_callback(MULTI_BTN_EVENT_CH4_PUSH);
				NRF_LOG_INFO("CH4 Released");
				break;

			default:
			break;
		}
	}


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

	// reset state variables
	for (size_t i = 0; i < 4; i++)
	{
		btn_hold_count[i] = 0;
		button_held_cnt[i] = 0;
	}
	timer_run = false;

	    //motor position timer, this is 10hz but really this will be 100 - 200 hz... or more likely 128 or 256, because 2^ maths
    err_code = app_timer_create(&m_button_timer, APP_TIMER_MODE_REPEATED, button_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void multi_buttons_tasks(void) {

	if (timer_run) {
		button_timeout_handler(NULL);
	}
}

void multi_buttons_disable()
{
	uint32_t err_code;

	err_code = app_button_disable();
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("MULTI buttons disable");
}
