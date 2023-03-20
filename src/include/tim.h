/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "system_main.h"

extern TIM_HandleTypeDef htim2;
extern uint32_t tim2_uptime;
extern uint8_t tim2_flag;

extern TIM_HandleTypeDef htim3;
extern uint32_t tim3_uptime;
extern uint8_t tim3_flag_1sec;

void tim2_init(void);
void tim3_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

