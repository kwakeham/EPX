/**
 * Copyright (c) 2019 - 2023, TITAN LAB Inc.
 *
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

//not in use
#define HWFC           true

//TEST

#define M_EN 6
#define M_IN1 6
#define M_Phase 7
#define M_IN2 7
#define M_PMode 8 //this is called P-mode in the docs, thats important
#define M_nSleep 9
#define M_nFault 5 // this is read

#define M_ISENSE    3 //AIN1

#define S_HALL_EN   31
#define S_SIN       4 //AIN2
#define S_COS       2 //AIN0


#define BATT_SDA 14
#define BATT_SCL 10


#define LIS2DTW_CS 17
#define LIS2DTW_SCLK 18
#define LIS2DTW_SDI 15
#define LIS2DTW_SDO 16
#define LIS2DTW_INT1 19

#define GAGE_CTRL 19

#define TX_PIN_NUMBER 23

#define TP1 27
#define TP2 28

#define nRST 21

#define MAG_DIGITAL 22
#define MAG_ANALOG 31

#define LED_B 23
#define LEG_R 24


#define BUTTONS_NUMBER 3 //because not in order? TODO: Check this
#define BUTTON_1 20
#define BUTTON_2 13
#define BUTTON_3 12
#define BUTTON_4 11

#define BSP_BUTTON_0            BUTTON_2
#define BSP_BUTTON_1            BUTTON_3
#define BSP_BUTTON_2            BUTTON_4

#define BUTTON_PULL             NRF_GPIO_PIN_PULLUP
#define BUTTONS_ACTIVE_STATE    0

#define BUTTONS_LIST            { BUTTON_2, BUTTON_3, BUTTON_4}

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
