/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    foc.h
  * @brief   This file contains all the function prototypes for
  *          the foc.cpp file
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
#ifndef FOC_H
#define FOC_H

#include <Arduino.h>
#include <stdio.h>
#include "iodefs.h"
#include <Arduino.h>
#include "stm32f1xx_hal.h"
#include "common.h"

void dq0_abc_variables(float theta);

void abc_fast(float d, float q, volatile float *a, volatile float *b, volatile float *c);

// Used to get Iq and Id from Ia Ib and Ic
void dq0_fast(float a, float b, float c, float *d, float *q);

void abc( float theta, float d, float q,  float *a,  float *b,  float *c);
    
void dq0(float theta, float a, float b, float c, float *d, float *q);

void dq0_fast_int( int a, int b, int c, volatile  float *d, volatile float *q);

void sinusoidal_commutation(float v_bus, float u, float v, float w, volatile float *u_normalized, volatile float *v_normalized,volatile  float *w_normalized);

void space_vector_commutation(float v_bus, float u, float v, float w, volatile float *u_normalized, volatile float *v_normalized, volatile float *w_normalized);

#endif