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

// For TIM2
TIM_HandleTypeDef htim2;
uint32_t tim2_uptime = 0;
uint8_t tim2_flag = 0;

// For TIM3
TIM_HandleTypeDef htim3;
uint32_t tim3_uptime = 0;
uint8_t tim3_flag_1sec = 0;

void tim2_config_2(void) {
/************** STEPS TO FOLLOW *****************
	1. Enable Timer clock
	2. Set the prescalar and the ARR
	3. Enable the Timer, and wait for the update Flag to set
************************************************/
	
	RCC->APB1ENR |= (1<<0); // enable clock for TIM2 
/*	
	TIM2->CR1 = 0;  				// reset all
	TIM2->CR1 |= (0<<1); 		// UDIS=0; UEV Enabled. The Update event will generate
	TIM2->CR1 |= (0<<4);  	// DIR=1; select UP COUNTER mode
	TIM2->CR1 |= (0<<5);  	// CMS=0;  Edge-aligned mode, up or down count depends on DIR
	TIM2->CR1 |= (0<<8);  	// CKD=0; No clock DIVISION
	TIM2->CR1 |= (0<<7);  	// ARPE=0; ARR Auto Reload Disabled
	TIM2->RCR |= 0;         // Repetition Counter 0
*/	
	TIM2->ARR = 0xffff-1;  	// ARR value
	TIM2->PSC = 72-1;      	// Prescalar value
	
	TIM2->CR1 |= (1<<0);  	// enable timer
	while (!(TIM2->SR & (1<<0)));  // UIF: Update interrupt flag..  This bit is set by hardware when the registers are updated
}
void Delay_us (uint16_t us)
{
	/************** STEPS TO FOLLOW *****************
	1. RESET the Counter
	2. Wait for the Counter to reach the entered value. As each count will take 1 us, 
		 the total waiting time will be the required us delay
	************************************************/
	TIM2->CNT = 0;
	while (TIM2->CNT < us);
}
void Delay_ms (uint16_t ms)
{
	for (uint16_t i=0; i<ms; i++)
	{
		Delay_us (1000); // delay of 1 ms
	}
}
void tim2_init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 4000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{	
		printf("TIM2: init error 0\n");
		Error_Handler();
	} else {
		printf("TIM2: initialized!\n");
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		printf("TIM2: clock config error!\n");
		Error_Handler();
	}

	// if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	// {
	// 	Error_Handler();
	// }

	// TIM2->SMCR &= ~TIM_SMCR_SMS;	// Internal clock select;

	// TIM2->PSC = 8000;				// Divide 8MHz by 8000 = 1000 Hz;
	// TIM2->EGR |= (1<<0);			// Set or reset the UG Bit

	// TIM2->ARR = 1000-1;				// Auto reload count to 1000. 1 second.

	HAL_TIM_Base_Start_IT(&htim2);		// update interrupt enable;
	// TIM2->DIER |= TIM_DIER_UIE;		

	// TIM2->CR1 &= ~TIM_CR1_DIR;		// Upconting mode DIR = 0;
	// TIM2->CR1 |=  TIM_CR1_CEN;		// TIM1 Counter Enable;


	// sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	// sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	// if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	// {
	// 	Error_Handler();
	// }

	// sConfigOC.OCMode = TIM_OCMODE_TIMING;
	// sConfigOC.Pulse = 0;
	// sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	// sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	// if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	// {
	// 	Error_Handler();
	// }
	// __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
}
void tim3_init(void) {

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1000;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		printf("TIM3: init error\n");
		Error_Handler();
	} else {
		printf("TIM3: initialized!\n");
	}

	TIM_ClockConfigTypeDef sClockSourceConfig;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		printf("TIM3: clock config error!\n");
		Error_Handler();
	}

	TIM_MasterConfigTypeDef sMasterConfig;
	// sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		printf("tim3 error 2\n");
		Error_Handler();
	}


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
	// TIM3 -> CR1 &= ~TIM_CR1_DIR;		// Upconting mode DIR = 0;
//	TIM3 -> CR1 |=  TIM_CKD_DIV1;

/*	TIM1_SMCR:
	15		ETP
	7
	2:0		SMS[2:0]
*/
	// TIM3 -> SMCR &= ~TIM_SMCR_SMS;		// Internal clock select;

/*	TIM1_DIER: DMA/interrupt enable register
	14		TDE	- Trigger DMA request enable
	8		UDE
	6		TIE
	0		UIE	- Uptade Interrupt enable
*/
	HAL_TIM_Base_Start_IT(&htim3);		// update interrupt enable;
	// TIM1 -> DIER |= TIM_DIER_UIE;

/* Prescaler configuration */
	// TIM3->PSC = 8000;				// Divide 8MHz by 8000 = 1000 Hz;
	// TIM3->EGR |= (1<<0);			// Set or reset the UG Bit

	// TIM3->ARR = 1000-1;				// Auto reload count to 1000. 1 second.

	/*	Interruption set*/
	// if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	// {
	// 	/* Starting Error */
	// 	Error_Handler();
	// }
	// TIM3->DIER |= TIM_DIER_UIE;		// update interrupt enable

	/* TIM3 enable counter */
	// TIM3 -> CR1 |=  TIM_CR1_CEN;	// TIM1 Counter Enable;
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	if(tim_baseHandle->Instance==TIM3)
	{
		/* TIM3 clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();

		/* TIM3 interrupt Init */
		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}

	if(tim_baseHandle->Instance==TIM2)
	{
		/* TIM2 clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();

		/* TIM2 interrupt Init */
		HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}
}
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
	if(tim_baseHandle->Instance==TIM2)
	{
		/* USER CODE BEGIN TIM2_MspDeInit 0 */

		/* USER CODE END TIM2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();

		/* TIM3 interrupt Deinit */
		HAL_NVIC_DisableIRQ(TIM2_IRQn);
		/* USER CODE BEGIN TIM2_MspDeInit 1 */

		/* USER CODE END TIM2_MspDeInit 1 */
	}

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
