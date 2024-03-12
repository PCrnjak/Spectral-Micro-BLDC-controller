/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iodefs.h
  * @brief   This file provides code for pin declarations
  * @author Petar Crnjak
  ******************************************************************************
  * @attention
  *
  * Copyright (c) Source robotics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IODEFS_H
#define IODEFS_H

#include <Arduino.h>
#include <stdio.h>

#define PWM_CH1 PA_2 //Driver IN1 ===> PA3 sense; SENSE2
#define PWM_CH2 PA_1 //Driver IN2 ==> No sense resistor
#define PWM_CH3 PA_0 //Driver IN3 ==> PA4 sense; SENSE1

#define EN1 PA_9
#define EN2 PA_8
#define EN3 PA_10

#define RESET PB_14
#define SLEEP PB_15

#define HAL1 PB_2
#define HAL2 PB_1
#define HAL3 PA_0

#define FAULT PA_11

#define MOSI PB_5
#define MISO PB_4
#define CLK PB_3
#define CSN PA_15

#define EEPROM_SCL PB_10
#define EEPROM_SDA PB_11
#define EEPROM_WP PB_12

//#define LED PA_7 //VERSION 170 BATCH

#define LED PB_13

#define ADDITIONAL PA_7

#define SENSE1 PA_4 // CHANNEL 4
#define SENSE2  PA_3 // CHANNEL 3
#define VBUS_VOLTAGE PA_6 // CHANNEL 6
#define TEMPERATURE PA_5 // CHANNEL 5

#define TX_COM PB_6
#define RX_COM PB_7

// CAN bus
//#define RX_COM PB8
//#define TX_COM PB9



#endif