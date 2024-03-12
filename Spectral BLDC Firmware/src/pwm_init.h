/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    pwm_init.h
  * @brief   This file provides code for all the function prototypes for pwm_init.cpp
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
#ifndef PWM_H
#define PWM_H

#include <Arduino.h>
#include <stdio.h>
#include "iodefs.h"
#include "stm32f1xx_hal.h"

HardwareTimer* pwm_high(int ulPin, uint32_t PWM_freq);
HardwareTimer* pwm_freq(int ulPin, uint32_t PWM_freq, void (*int_callback)());
HardwareTimer* pwm_freq_2(int ulPin, uint32_t PWM_freq, void (*int_callback)());
HardwareTimer* pwm_low(int ulPin, uint32_t PWM_freq);
void pwm_set(int ulPin, uint32_t value, int resolution);
void pwm_align(HardwareTimer *HT2);


#endif