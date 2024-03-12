/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    common.h
  * @brief   This file provides code for all the function prototypes for EEPROM.cpp
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


#ifndef EEPROM_H
#define EEPROM_H

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

// 16 Kbit EEPROM
#define EEPROM 16384 / 8

// address of EEPORM
#define DEVICEADDRESS (0x50)

//I2C_eeprom eeprom(DEVICEADDRESS, EEPROM);

void writeFloat(unsigned int pageAddress, float data);

float readFloat(unsigned int pageAddress);

int32_t readInt(unsigned int pageAddress);

void writeInt(unsigned int pageAddress, int32_t data);

int8_t readInt_8t(unsigned int pageAddress);

void writeInt_8t(unsigned int pageAddress, int8_t data);

void Init_EEPROM();

void read_config();

void Write_config();

void Set_Default_config();



#endif