/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim3;
uint32_t tim3_uptime = 0;
uint8_t tim3_flag_1sec = 0;

/* TIM3 init function */
void tim3_init(void)
{
	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	// TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	// TIM_MasterConfigTypeDef sMasterConfig = {0};

	// /* USER CODE BEGIN TIM3_Init 1 */

	// /* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		printf("tim3 error 0\n");
		Error_Handler();
	}

	// sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	// if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	// {
	// 	printf("tim3 error 1\n");
	// 	Error_Handler();
	// }

	// sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	// sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	// if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	// {
	// 	printf("tim3 error 2\n");
	// 	Error_Handler();
	// }


/*	TIM1_CR1:
	9:8		CKD[1:0]
	7		ARPE
	6:5		CMS[1:0]
	4		DIR
	3		OPM
	2		URS
	1		UDIS
	0		CEN
*/
//	TIM3 -> CR1 |=  TIM_CR1_CEN;		// TIM1 Counter Enable;
	TIM3 -> CR1 &= ~TIM_CR1_DIR;		// Upconting mode DIR = 0;
//	TIM3 -> CR1 |=  TIM_CKD_DIV1;

/*	TIM1_SMCR:
	15		ETP
	7
	2:0		SMS[2:0]
*/
	TIM3 -> SMCR &= ~TIM_SMCR_SMS;		// Internal clock select;

/*	TIM1_DIER: DMA/interrupt enable register
	14		TDE	- Trigger DMA request enable
	8		UDE
	6		TIE
	0		UIE	- Uptade Interrupt enable
*/
	// TIM1 -> DIER |= TIM_DIER_UIE;

/* Prescaler configuration */
//	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM3->PSC = 8000;				// Divide 8MHz by 8000 = 1000 Hz;
	TIM3->EGR |= (1<<0);			// Set or reset the UG Bit

	TIM3->ARR = 1000-1;				// Auto reload count to 1000. 1 second.

	/*	Interruption set*/
	// if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	// {
	// 	/* Starting Error */
	// 	Error_Handler();
	// }
	TIM3->DIER |= TIM_DIER_UIE;		// update interrupt enable

	/* TIM3 enable counter */
	TIM3 -> CR1 |=  TIM_CR1_CEN;	// TIM1 Counter Enable;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	if(tim_baseHandle->Instance==TIM3)
	{
		/* USER CODE BEGIN TIM3_MspInit 0 */

		/* USER CODE END TIM3_MspInit 0 */
		/* TIM3 clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();

		/* TIM3 interrupt Init */
		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
		/* USER CODE BEGIN TIM3_MspInit 1 */

		/* USER CODE END TIM3_MspInit 1 */
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
