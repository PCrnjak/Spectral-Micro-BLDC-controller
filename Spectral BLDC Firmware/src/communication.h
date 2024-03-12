/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    communication.h
  * @brief   This file provides code for all the function prototypes for communication.cpp
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/


#ifndef COMS_H
#define COMS_H

#include "constants.h"
#include <stdint.h>
#include "hw_init.h"
#include "pwm_init.h"
#include "utils.h"
#include "stm32f1xx_hal.h"
#include "adc_init.h"
#include "iodefs.h"
#include "qfplib-m3.h"
#include "SerialPacketParser.h"
#include "constants.h"
#include "motor_control.h"
#include <SPI.h>
#include "common.h"
#include "foc.h"
#include "temperature_table.h"
#include "CAN.h"
#include <Wire.h>
#include <I2C_eeprom.h>
#include "EEPROM.h"


void UART_protocol(Stream &Serialport);

void Cyclic_UART(Stream &Serialport ,uint32_t ms);


#endif