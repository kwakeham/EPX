/**
 * Copyright (c) 2019 - 2022, TITAN LAB Inc.
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

#define MAX_DRDY 2
#define MAX_SCK_PIN  4
#define MAX_MISO_PIN 25
#define MAX_nRST 26
#define MAX_SS_PIN 27
#define MAX_MOSI_PIN 30

#define MXC_SDA 5
#define MXC_SCL 7

#define CRM_SCLK 6
#define CRM_CS 8

#define CRM_DIN 10
#define CRM_DOUT 11
#define CRN_nRST 12

#define LSM6DSM_CS 13
#define LSM6DSM_SCLK 15
#define LSM6DSM_MISO 16
#define LSM6DSM_MOSI 17
#define LSM6DSM_INT1 18

#define GAGE_CTRL 19

#define TX_PIN_NUMBER 20

#define TP1 9
#define TP2 14
#define nRST 21

#define MAG_DIGITAL 22
#define MAG_ANALOG 31

#define LED_B 23
#define LEG_R 24




#define BUTTONS_NUMBER 2
#define BUTTON_1 TP1
#define BUTTON_2 TP2

#define BSP_BUTTON_0            BUTTON_1
#define BSP_BUTTON_1            BUTTON_2

#define BUTTON_PULL             NRF_GPIO_PIN_PULLUP
#define BUTTONS_ACTIVE_STATE    0

#define BUTTONS_LIST            { BUTTON_1 , BUTTON_2}

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
