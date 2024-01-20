/**
 * @file multi_btn.h
 * @author Keith Wakeham (keith@titanlab.co)
 * @brief Rewrite of a other internal libraries from Titan lab related to buttons
 * @version 0.1
 * @date 2024-01-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MULTI_BTN_H_
#define MULTI_BTN_H_

#include <stdint.h>

typedef enum
{
    MULTI_BTN_EVENT_NOTHING = 0,
    MULTI_BTN_EVENT_CH1_PUSH,
    MULTI_BTN_EVENT_CH2_PUSH,
    MULTI_BTN_EVENT_CH3_PUSH,
    MULTI_BTN_EVENT_CH4_PUSH,
    MULTI_BTN_EVENT_CH1_LONG,
    MULTI_BTN_EVENT_CH2_LONG,
    MULTI_BTN_EVENT_CH3_LONG,
    MULTI_BTN_EVENT_CH4_LONG,
} multibtn_event_t;


#ifdef	__cplusplus
extern "C" {
#endif

/**@brief MULTIBTN module event callback function type.
 *
 * @details     Upon an event in the BSP module, this callback function will be called to notify
 *              the application about the event.
 *
 * @param[in]   bsp_event_t BSP event type.
 */
typedef void (* multibtn_event_callback_t)(multibtn_event_t);


/**@brief button_timeout_handler.
 *
 * @details     Upon an event in the BSP module, this callback function will be called to notify
 *              the application about the event.
 *
 * @param[in]   p_context .
 */
// void button_timeout_handler(void * p_context);
void button_timeout_handler(void * p_context);



/**@brief button_callback function type.
 *
 * @details     Upon an event in the BSP module, this callback function will be called to notify
 *              the application about the event.
 *
 * @param[in]   pin_no  which pin caused the callback
 * @param[in]   button_action  ?????
 */
void button_callback(uint8_t pin_no, uint8_t button_action);


/**@brief Function for initializing MULTI Button
 *
 * @details     The function initializes the MULTI button module to allow state indication and
 *              button reaction.
 *
 * @param[in]   none
 */
void multi_buttons_init(multibtn_event_callback_t callback);


/**
 * @brief resets the variables and timer that holds the state of the buttons
 * 
 */
void multi_reset_buttons(void);


/**
 * @brief check if any button is pressed and ensure timer is running
 * 
 */
void multi_button_timer_run_check(void);

/**
 * @brief This runs tasks for the buttons that should run not during an
 * interrupt but form main after an interrupt usually based on flags
 * 
 */
void multi_buttons_tasks(void);


/**@brief Function for disabling MULTI Button
 *
 * @details     The function initializes the MULTI button module to allow state indication and
 *              button reaction.
 *
 * @param[in]   none
 */
void multi_buttons_disable();


#ifdef	__cplusplus
}
#endif

#endif /* SOURCE_SENSORS_MULTI_BTN_H_ */
