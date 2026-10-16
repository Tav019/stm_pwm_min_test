/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/*
 * Mapeo de timers del firmware unificado (STM32F446RETx), fuente de verdad
 * para las cuatro ruedas (ver tabla en tim.c):
 * - htim1  : encoder FR  (CH1=PA8, CH2=PA9)
 * - htim3  : encoder FL  (CH1=PB4, CH2=PA7)
 * - htim5  : encoder BR  (CH1=PA0, CH2=PA1)
 * - htim8  : encoder BL  (CH1=PC6, CH2=PC7)
 * - htim2  : PWM FR + FL (CH1=PA15 FR-Adelante, CH4=PB2 FR-Atras,
 *                          CH2=PB9 FL-Adelante, CH3=PB10 FL-Atras)
 * - htim12 : PWM BR      (CH1=PB14 Adelante, CH2=PB15 Atras)
 * - htim10 : PWM BL Adelante (CH1=PB8)
 * - htim13 : PWM BL Atras    (CH1=PA6)
 * - htim6  : base de tiempo HAL (reemplaza a TIM1, ahora ocupado por el
 *            encoder FR)
 */
extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim5;

extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim10;

extern TIM_HandleTypeDef htim12;

extern TIM_HandleTypeDef htim13;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_TIM10_Init(void);
void MX_TIM12_Init(void);
void MX_TIM13_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

