/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hw_init.h
  * @brief   This file contains all the function prototypes for the hw_init.cpp file
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
#ifndef HW_SETUP_H
#define HW_SETUP_H

#include <Arduino.h>
#include <stdio.h>
#include "iodefs.h"
#include <Arduino.h>
#include "stm32f1xx_hal.h"
#include "common.h"



void Init_Digital_Inputs(void);
void Init_Digital_Outputs(void);

void TM_SystemClock_Config(int multiplier);
void SystemClock_Config(void);

void DRV_ERROR_INTERRUPT();

//static void MX_ADC1_Init(void);

#endif