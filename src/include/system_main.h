/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __SYSTEM_MAIN_H
#define __SYSTEM_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void SystemClock_Config_8MHz_HSE(void);
void SystemClock_Config_8MHZ_HSI(void);
void SystemClock_Config_48MHz_HSE_ADC(void);
void SystemClock_Config_56MHz_HSE_ADC(void);
void SystemClock_Config_72MHz(void);
void SystemClock_Config_72MHz_2(void);
void Error_Handler(void);
void init_system(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_MAIN_H */
