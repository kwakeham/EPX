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
#include "sdk_config.h"
#include "multi_btn.h"
#include "app_util_platform.h"
#include "segger_wrapper.h"


#define BUTTON_STATE_POLL_INTERVAL_MS  10UL
#define MULTI_PRESS_INTERVAL_MS        600UL

//Buttons defined via SDK, to change
#define MULTI_BTN_PIN_CH1      BUTTON_2
#define MULTI_BTN_PIN_CH2      BUTTON_1
#define MULTI_BTN_PIN_CH3      BUTTON_3
#define MULTI_BTN_PIN_CH4      BUTTON_4

#define LONG_PRESS(MS)      (uint32_t)(MS)/BUTTON_STATE_POLL_INTERVAL_MS

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
static uint8_t btn_CH1_press_count = 0;
static uint8_t btn_CH2_press_count = 0;
static uint8_t btn_CH3_press_count = 0;
static uint8_t btn_CH4_press_count = 0;
static bool long_press_active_CH1;
static bool long_press_active_CH2;
static bool long_press_active_CH3;
static bool long_press_active_CH4;
static int32_t cnt_CH1 = 0;
static int32_t cnt_CH2 = 0;
static int32_t cnt_CH3 = 0;
static int32_t cnt_CH4 = 0;

typedef struct {
	app_button_cfg_t  button_cfg[BUTTONS_NUMBER];
	uint8_t btn_CH1_press_count;
	uint8_t btn_CH2_press_count;
	uint8_t btn_CH3_press_count;
	uint8_t btn_CH4_press_count;
	bool long_press_active_CH1;
	bool long_press_active_CH2;
	bool long_press_active_CH3;
	bool long_press_active_CH4;
	int32_t cnt_CH1;
	int32_t cnt_CH2;
	int32_t cnt_CH3;
	int32_t cnt_CH4;
} sButtonPairDescr;

static bool timer_run = false;
static bool repeat_run = false;
static uint16_t repeat_counter = 0;
static uint8_t is_button_init=0;

void button_timeout_handler (void * p_context)
{
	// uint32_t err_code;
	bool CH1_pushed = app_button_is_pushed(0);
	bool CH2_pushed = app_button_is_pushed(1);
	bool CH3_pushed = app_button_is_pushed(2);
	bool CH4_pushed = app_button_is_pushed(3);

	// disable by default
	timer_run = false;

	if (CH2_pushed && CH1_pushed)
	{
		cnt_CH1++;
		cnt_CH2++;
		if ( cnt_CH2 >= LONG_PRESS(1000))
		{
			cnt_CH2 = cnt_CH2 - LONG_PRESS(1000);
			if (!long_press_active_CH2)
			{
				long_press_active_CH2 = true;
				m_registered_callback(MULTI_BTN_EVENT_CH2_LONG);
			} else
			{
				m_registered_callback(MULTI_BTN_EVENT_CH2_LONG_CONT);
			}
			btn_CH2_press_count = 0;
			// btn_CH3_press_count = 0;
			// btn_CH4_press_count = 0;
		}

		if (cnt_CH1 >= LONG_PRESS(1000))
		{
			cnt_CH1 = cnt_CH1 - LONG_PRESS(1000);
			if (!long_press_active_CH1)
			{
				long_press_active_CH1 = true;
				m_registered_callback(MULTI_BTN_EVENT_CH1_LONG);
			} else
			{
				m_registered_callback(MULTI_BTN_EVENT_CH1_LONG_CONT);
			}
			btn_CH1_press_count = 0;
		}
		timer_run = true;
	}
	else if (CH1_pushed)
	{
		cnt_CH1++;
		if ( cnt_CH1 >= LONG_PRESS(1000))
		{
			cnt_CH1 = cnt_CH1 - LONG_PRESS(1000);
			if (!long_press_active_CH1)
			{
				long_press_active_CH1 = true;
				m_registered_callback(MULTI_BTN_EVENT_CH1_LONG);

			} else
			{
				m_registered_callback(MULTI_BTN_EVENT_CH1_LONG_CONT);
			}
			btn_CH1_press_count = 0;
		}
		timer_run = true;
		cnt_CH2 = 0;
		cnt_CH3 = 0;
		cnt_CH4 = 0;
	}
	else if (CH2_pushed)
	{
		cnt_CH2++;
		if ( cnt_CH2 >= LONG_PRESS(1000))
		{
			cnt_CH2 = cnt_CH2 - LONG_PRESS(1000);
			if (!long_press_active_CH2)
			{
				long_press_active_CH2 = true;

				m_registered_callback(MULTI_BTN_EVENT_CH2_LONG);
			} else
			{
				m_registered_callback(MULTI_BTN_EVENT_CH2_LONG_CONT);
			}
			btn_CH2_press_count = 0;
		}
		timer_run = true;
		cnt_CH1 = 0;
		cnt_CH3 = 0;
		cnt_CH4 = 0;
	}
	else if (CH3_pushed)
	{
		cnt_CH3++;
		if ( cnt_CH3 >= LONG_PRESS(1000))
		{
			cnt_CH3 = cnt_CH3 - LONG_PRESS(1000);
			if (!long_press_active_CH3)
			{
				long_press_active_CH3 = true;

				m_registered_callback(MULTI_BTN_EVENT_CH3_LONG);
			} else
			{
				m_registered_callback(MULTI_BTN_EVENT_CH3_LONG_CONT);
			}
			btn_CH3_press_count = 0;
		}
		timer_run = true;
		cnt_CH2 = 0;
		cnt_CH1 =  0;
		cnt_CH4 = 0;
	}
	else if (CH4_pushed)
	{
		cnt_CH4++;
		if ( cnt_CH4 >= LONG_PRESS(1000))
		{
			cnt_CH4 = cnt_CH4 - LONG_PRESS(1000);
			if (!long_press_active_CH4)
			{
				long_press_active_CH4 = true;

				m_registered_callback(MULTI_BTN_EVENT_CH4_LONG);
			} else
			{
				m_registered_callback(MULTI_BTN_EVENT_CH4_LONG_CONT);
			}
			btn_CH4_press_count = 0;
		}
		timer_run = true;
		cnt_CH2 = 0;
		cnt_CH1 = 0;
		cnt_CH3 = 0;
	}

	else // no button is pressed
	{
		if (long_press_active_CH1)
		{
			long_press_active_CH1 = false;
			m_registered_callback(MULTI_BTN_EVENT_CH1_LONG_RELEASE);
		}
		else if (!long_press_active_CH1 && cnt_CH1 >= (LONG_PRESS(600)))  // This fixes the edge case of between 600ms time out and the 1000ms long
		{
			m_registered_callback(MULTI_BTN_EVENT_CH1);
			// LOG_INFO("Edgecase");
		}

		if (long_press_active_CH2)
		{
			long_press_active_CH2 = false;
			m_registered_callback(MULTI_BTN_EVENT_CH2_LONG_RELEASE);
		}
		else if (!long_press_active_CH2 && cnt_CH2 >= (LONG_PRESS(600)))  // This fixes the edge case of between 600ms time out and the 1000ms long
		{
			m_registered_callback(MULTI_BTN_EVENT_CH2);
		}
		
		if (long_press_active_CH3)
		{
			long_press_active_CH3 = false;
			m_registered_callback(MULTI_BTN_EVENT_CH3_LONG_RELEASE);
		}
		else if (!long_press_active_CH3 && cnt_CH3 >= (LONG_PRESS(600)))  // This fixes the edge case of between 600ms time out and the 1000ms long
		{
			m_registered_callback(MULTI_BTN_EVENT_CH3);
		}

		if (long_press_active_CH4)
		{
			long_press_active_CH4 = false;
			m_registered_callback(MULTI_BTN_EVENT_CH4_LONG_RELEASE);
		}
		else if (!long_press_active_CH4 && cnt_CH4 >= (LONG_PRESS(600)))  // This fixes the edge case of between 600ms time out and the 1000ms long
		{
			m_registered_callback(MULTI_BTN_EVENT_CH4);
		}
		cnt_CH1 = 0; // Reset the counter
		cnt_CH2 = 0;
		cnt_CH3 = 0;
		cnt_CH4 = 0;
	}

}

void repeat_timeout_handler(void * p_context)
{
	if (!long_press_active_CH1  && (cnt_CH1 < (LONG_PRESS(600)-1) ))
	{
		// LOG_INFO("repeat_timeout_handler: btn 1 pressed %d with cnt = %d", btn_CH1_press_count, cnt_CH1 );
		if (btn_CH1_press_count == 1)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH1);
			// LOG_INFO("Edgecase");
		} else if (btn_CH1_press_count == 2)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH1_DOUBLE);
		} else if (btn_CH1_press_count == 3)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH1_TRIPLE);
		}
		btn_CH1_press_count = 0;
	}

	if (!long_press_active_CH2 && (cnt_CH2 < (LONG_PRESS(600)-1)))
	{
		//Button 2 stuff
		if (btn_CH2_press_count == 1)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH2);
		} else if (btn_CH2_press_count == 2)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH2_DOUBLE);
		} else if (btn_CH2_press_count == 3)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH2_TRIPLE);
		}

		btn_CH2_press_count = 0;
	}

	if (!long_press_active_CH3 && (cnt_CH3 < (LONG_PRESS(600)-1)))
	{
		//Button 2 stuff
		if (btn_CH3_press_count == 1)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH3);
		} else if (btn_CH3_press_count == 2)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH3_DOUBLE);
		} else if (btn_CH3_press_count == 3)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH3_TRIPLE);
		}
		btn_CH3_press_count = 0;
	}

	if (!long_press_active_CH4 && (cnt_CH4 < (LONG_PRESS(600)-1)))
	{
		//Button 2 stuff
		if (btn_CH4_press_count == 1)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH4);
		} else if (btn_CH4_press_count == 2)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH4_DOUBLE);
		} else if (btn_CH4_press_count == 3)
		{
			m_registered_callback(MULTI_BTN_EVENT_CH4_TRIPLE);
		}

		btn_CH4_press_count = 0;
	}
	repeat_run = false;
	if (btn_CH5_pressed) {
		btn_CH5_pressed = 0;
		m_registered_callback(MULTI_BTN_EVENT_CH5);
	}
	if (btn_CH6_pressed) {
		btn_CH6_pressed = 0;
		m_registered_callback(MULTI_BTN_EVENT_CH6);
	}
}

// called by the app_button only
// seems like it starts the timers
void button_callback(uint8_t pin_no, uint8_t button_action)
{

	if (button_action == APP_BUTTON_PUSH) {

		// let the timeout run
		timer_run = true;
		repeat_run = true;
		repeat_counter = 0;

		//NRF_LOG_WARNING("Button %d pushed !", pin_no);

		switch (pin_no) {

			case MULTI_BTN_PIN_CH1:
				btn_CH1_press_count++;
				break;

			case MULTI_BTN_PIN_CH2:
				btn_CH2_press_count++;
				break;

			case MULTI_BTN_PIN_CH3:
				btn_CH3_press_count++;
				break;

			case MULTI_BTN_PIN_CH4:
				btn_CH4_press_count++;
				break;

#ifdef MULTI_BTN_PIN_CH6
			case MULTI_BTN_PIN_CH5:
				btn_CH5_pressed = 1;
				break;

			case MULTI_BTN_PIN_CH6:
				btn_CH6_pressed = 1;
				break;
#endif

			default:
			break;
		}

	}

}



void multi_buttons_init(multibtn_event_callback_t callback)
{
	uint32_t err_code;

	//   button_cfg
	m_registered_callback = callback;

	if (is_button_init == 0) {
		is_button_init++;
		err_code = app_button_init(button_cfg, sizeof(button_cfg) / sizeof(button_cfg[0]), APP_TIMER_TICKS(40));
		APP_ERROR_CHECK(err_code);
	}

	err_code = app_button_enable();
	APP_ERROR_CHECK(err_code);

	LOG_INFO("MULTI app button init");

	// reset state variables
	btn_CH1_press_count = 0;
	btn_CH2_press_count = 0;
	btn_CH3_press_count = 0;
	btn_CH4_press_count = 0;

	btn_CH5_pressed = 0;
	btn_CH6_pressed = 0;

	long_press_active_CH1 = false;
	long_press_active_CH2 = false;
	long_press_active_CH3 = false;
	long_press_active_CH4 = false;

	cnt_CH1 = 0;
	cnt_CH2 = 0;
	cnt_CH3 = 0;
	cnt_CH4 = 0;

	timer_run = false;
	repeat_run = false;
	repeat_counter = 0;
}

void multi_buttons_tasks(void) {

	if (timer_run) {
		button_timeout_handler(NULL);
	}

	if (++repeat_counter >= MULTI_PRESS_INTERVAL_MS / BUTTON_STATE_POLL_INTERVAL_MS &&
			repeat_run) {

		repeat_timeout_handler(NULL);
		repeat_run = false;
		repeat_counter = 0;
	}
}

void multi_buttons_disable()
{
	uint32_t err_code;

	err_code = app_button_disable();
	APP_ERROR_CHECK(err_code);
	LOG_INFO("MULTI buttons disable");
}
