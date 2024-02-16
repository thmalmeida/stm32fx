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

// For TIM1
TIM_HandleTypeDef htim1 = {0};
uint8_t tim1_flag = 0;
uint32_t tim1_cnt = 0;

// For TIM2
TIM_HandleTypeDef htim2 = {0};
uint8_t tim2_flag = 0;
uint32_t tim2_cnt = 0;

// For TIM3
TIM_HandleTypeDef htim3 = {0};
uint8_t tim3_flag = 0;
uint32_t tim3_cnt = 0;

// For TIM4
TIM_HandleTypeDef htim4 = {0};
uint8_t tim4_flag = 0;
uint32_t tim4_cnt = 0;


/* TIM3 init function */
void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit_2(&htim3);

	if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	} else {
		printf("TIM3 PWM start!\n");
	}

}

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

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 4000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{	
		printf("TIM2: init error\n");
		Error_Handler();
	} else {
		printf("TIM2: initialized!\n");
	}

	HAL_TIM_Base_Start_IT(&htim2);		// update interrupt enable;

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		printf("TIM2: clock config error!\n");
		Error_Handler();
	}

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};	

	// if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	// {
	// 	Error_Handler();
	// }


	// TIM2->SMCR &= ~TIM_SMCR_SMS;		// Internal clock select;
	// TIM2->PSC = 8000;				// Divide 8MHz by 8000 = 1000 Hz;
	// TIM2->EGR |= (1<<0);				// Set or reset the UG Bit
	// TIM2->ARR = 1000-1;				// Auto reload count to 1000. 1 second.
	// TIM2->DIER |= TIM_DIER_UIE;		// update interrupt enable;
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
void tim2_pwm_init(uint32_t freq) {
	uint32_t gu32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);

	printf("freq: %d\n", HAL_RCC_GetHCLKFreq());

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10000-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 4000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit_2(&htim2);

	if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	} else {
		printf("TIM2 PWM init\n");
	}

}
void tim3_init(void) {

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 8000;									// 16 bit size value TIMx_PSC register
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;					// TIMx_CR1 register - Bit 4 (DIR)
	htim3.Init.Period = 1000-1;										// 12 bit size value
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	// htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		printf("TIM3: init error\n");
		Error_Handler();
	} else {
		printf("TIM3: initialized!\n");
	}

	printf("TESTE 01\n");
	TIM_ClockConfigTypeDef sClockSourceConfig;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	printf("TESTE 02\n");
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		printf("TIM3: clock config error!\n");
		Error_Handler();
	} else {
		printf("TIM3: clock configured!\n");
	}
	printf("TESTE 03\n");

	// TIM_MasterConfigTypeDef sMasterConfig;
	// // sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	// sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
	/*	Interruption Enable*/
	printf("TESTE 04\n");
	__HAL_RCC_TIM3_CLK_ENABLE();

	/* TIM3 interrupt Init */
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		/* Starting Error */
		printf("TIM3: isr Error!\n");
		Error_Handler();
	} else {
		printf("TIM3: interrupt enable");
	}
	// TIM3->DIER |= TIM_DIER_TIE;		// Trigger interrupt enable
	printf("TESTE 05\n");
	// TIM3->DIER |= TIM_DIER_UIE;		// update interrupt enable
	printf("TESTE 06\n");
/* Prescaler configuration */
	// TIM3->PSC = 8000;				// Divide 8MHz by 8000 = 1000 Hz;
	// TIM3->EGR |= (1<<0);			// Set or reset the UG Bit

	// TIM3->ARR = 1000-1;				// Auto reload count to 1000. 1 second.


	/* TIM3 enable counter */
	TIM3 -> CR1 |=  TIM_CR1_CEN;	// TIM1 Counter Enable;
	printf("TESTE 07\n");
}
void pwm_tim3_ch1(void) {
	
	// TIM3 and channel 1 - pin PA6
	// TIM_TypeDef *TIMX_;				// TIM module number;
	// TIMX_ = TIM3;

	// Enable TIM3 clock
	RCC->APB1ENR |= (1<<1);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	/* TIM3 GPIO Configuration - TIM3_CH1 --> PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	/* ----- CR1 register ----- */
	// CEN bit 0. Counter Enable. 0 disable; 1 enable.
	TIM3->CR1 &= ~(1<<0);

	// CKD[1:0] bits 8:9. Clock division: (div 1) 00: t_dts = t_ck_int; (div 2) 01: t_dts = t_ck_int/2; (div 4) 10: t_dts = t_ck_int/4.
	TIM3->CR1 &= ~(3<<8);

	// ARPE bit 7. Auto-reload preload enable. 1: ARR register is buffered.
	TIM3->CR1 |= (1<<7);

	// CMS[1:0] bits 6:5. Center-aligned mode selection. 00 Depends direction bit;
	TIM3->CR1 &= ~(3<<5);

	// DIR bit 4. 0 upcounter; 1 downcounter.
	TIM3->CR1 &= ~(1<<4);

	// OPM bit 3. One-pulse mode. 0 disable; 1 enable.
	TIM3->CR1 &= ~(1<<3);

	// URS bit 2. Update request source. 0 any event generate interrupt; 1 only ovr generate interrupt.
	TIM3->CR1 &= ~(1<<2);

	// UDIS bit 1. Update disable. 0 update enable; 1 update disable;
	TIM3->CR1 &= ~(1<<1);


	/* ----- CR2 register ----- */
	// TIM3->CR2 

	/* ----- Slave Mode Control Register (SMRC) ----- */
	
	/* ----- Event Generation Register (EGR) ----- */
	/* UG bit 0. Update generation. 0 no action; 1 Re-initialize the couter.
	*
	* As the preload registers are transferred to the shadow registers only
	* when an update event occurs, before starting the counter, the user has to
	* initialize all the registers by setting the UG bit in the TIM3EGR register.
	*/
	TIM3->EGR |= (1<<0);

	/* ----- Capture/compare mode register (CCMR1) ----- */
	// Using TIM3 and CH1 on pin PA6
	// OC1M bits [6:4]. Output compare 1 mode. 110: PWM mode 1 - in upcounting.
	// TIM3->CCMR1 &= ~(7<<4);
	// TIM3->CCMR1 |=  (6<<4);

	// Channel 1 - to output CC1S[1:0] = 00
	TIM3->CCMR1 &= ~(3<<0);

	// Channel 1 - OCxM[2:0] at bit 4. Select PWM mode 1 = 110 or PWM mode 2 = 111;
	TIM3->CCMR1 &= ~(7<<4);
	TIM3->CCMR1 |=  (6<<4);

	// Channel 1 - OCxPE bit 3. Output compare 1 preload enable. 0 disable; 1 Enable R/W operations to CCR1.
	TIM3->CCMR1 |=  (1<<3);

	// Channel 1 - Ouput compare clear enable OC1CE on bit 7
	TIM3->CCMR1 |= (1<<7);

	// OC1FE bit 2. Output compare 1 fast eanble. 0 fast disable.
	TIM3->CCMR1 &= ~(1<<2);


	/* ----- Capture/compare enable register (CCER) ----- */
	// CC1P bit 1. Capture/compare 1 output polarity. 0 active high; 1 active low
	TIM3->CCER &= ~(1<<1);

	// CC1E bit 0. Campture/Compare 1 output enable. 0 disable; 1 enable.
	TIM3->CCER |= (1<<0);

	/* ----- Counter register (CNT) ----- */
	// CNT. Reset counter.
	TIM3->CNT = 0;

	/* ----- Prescale (PSC) ----- */
	// 16 bit value
	TIM3->PSC = 2;

	/* ----- Auto-Reload Register (ARR) ----- */
	// 16 bit. Determines de PWM frequency;
	TIM3->ARR = 8000-1;

	/* ----- Capture/Compare register ch1 (CCR1) ----- */
	// 16 bit for duty cycle - It is linked with OCxPE on CCMRx register;
	TIM3->CCR1 = 4000;

	// CEN bit 0. Counter Enable. 0 disable; 1 enable.
	TIM3->CR1 |= (1<<0);


	// htim3.Instance = TIM3;
	// htim3.Init.Prescaler = 1;
	// htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	// htim3.Init.Period = 8000-1;
	// htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	// htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	// if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
	// 	Error_Handler();
	// }

	// TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	// sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	// if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
	// 	Error_Handler();
	// }
	// if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
	// 	Error_Handler();
	// }

	// TIM_MasterConfigTypeDef sMasterConfig = {0};
	// sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	// sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	// if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
	// 	Error_Handler();
	// }

	// TIM_OC_InitTypeDef sConfigOC = {0};
	// sConfigOC.OCMode = TIM_OCMODE_PWM1;
	// sConfigOC.Pulse = 2000;
	// sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	// sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	// if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	// {
	// 	Error_Handler();
	// }

	// if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
	// 	Error_Handler();
	// } else {
	// 	printf("TIM3 PWM start!\n");
	// }

}
void tim4_init(void) {

	uint32_t gu32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);


	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 10;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000-1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		printf("TIM4: init error\n");		
		Error_Handler();
	} else {
		printf("TIM4 init\n");
	}

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	} else {
		printf("TIM4 clock config.\n");
	}

	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	} else {
		printf("TIM4 pwm init.\n");
	}

	TIM_MasterConfigTypeDef sMasterConfig = {0};	
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // TIM_TRGO_OC1
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		printf("TIM4 sync Error\n");
		Error_Handler();
	} else {
		printf("TIM4 sync\n");
	}

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 10000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		printf("TIM4 channel config Error\n");
		Error_Handler();
	} else {
		printf("TIM4 channel config.\n");
	}

	HAL_TIM_MspPostInit_2(&htim4);

	TIM4->CCR1 = 300;
	if(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) {
		printf("TIM4 pwm start Error!\n");
	} else {
		printf("TIM4 pwm started!\n");
	}
}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle) {
  if(tim_pwmHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit_2(TIM_HandleTypeDef* timHandle) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(timHandle->Instance==TIM4) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/*TIM4 GPIO Configuration
		* PB7     ------> TIM4_CH1
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	
	if(timHandle->Instance==TIM3) {
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	
	if(timHandle->Instance==TIM2) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/*TIM2 GPIO Configuration
		 * PA2     ------> TIM2_CH3
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle) {
  if(tim_pwmHandle->Instance==TIM4) {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
}
void HAL_TIM_Base_MspInit_(TIM_HandleTypeDef* tim_baseHandle) {
	if(tim_baseHandle->Instance==TIM3) {
		/* USER CODE BEGIN TIM3_MspInit 0 */

		/* USER CODE END TIM3_MspInit 0 */
		/* TIM3 clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();
		/* USER CODE BEGIN TIM3_MspInit 1 */

		/* USER CODE END TIM3_MspInit 1 */
	}
}
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle) {
	if(tim_baseHandle->Instance==TIM2) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();

		/* TIM3 interrupt Deinit */
		HAL_NVIC_DisableIRQ(TIM2_IRQn);
	} else if(tim_baseHandle->Instance==TIM3) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();

		/* TIM3 interrupt Deinit */
		// HAL_NVIC_DisableIRQ(TIM3_IRQn);
	}
}

