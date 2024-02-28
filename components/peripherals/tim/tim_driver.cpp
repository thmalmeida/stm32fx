#include "tim_driver.hpp"

// ----- STM32F specifics -----
TIM_HandleTypeDef htim1_;
uint8_t tim1_flag_;
uint32_t tim1_cnt_;

TIM_HandleTypeDef htim2_;
uint8_t tim2_flag_;
uint32_t tim2_cnt_;

TIM_HandleTypeDef htim3_;
uint8_t tim3_flag_;
uint32_t tim3_cnt_;

TIM_HandleTypeDef htim4_;
uint8_t tim4_flag_;
uint32_t tim4_cnt_;

extern "C" {
// STM32F weak initializers functions
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {
	if(tim_baseHandle->Instance == TIM1) {
		/* TIM1 clock enable */
		__HAL_RCC_TIM1_CLK_ENABLE();

		HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

	} else if(tim_baseHandle->Instance == TIM2) {
		/* TIM2 clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();

		/* TIM2 interrupt Init */
		HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	} else if(tim_baseHandle->Instance == TIM3) {
		/* TIM3 clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();

		/* TIM3 interrupt Init */
		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	} else if(tim_baseHandle->Instance == TIM4) {
		/* TIM4 clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE();

		/* TIM4 interrupt Init */
		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
	}
}
// STM32F TIM interruptions functions
void TIM1_UP_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim1_);
	tim1_flag_ = 1;
}
/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim1_);
	// tim1_cnt_++;
}
void TIM2_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim2_);
	tim2_flag_ = 1;
	// tim2_cnt_++;
}
void TIM3_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim3_);
	tim3_flag_ = 1;
	// tim3_cnt_++;
}
void TIM4_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim4_);
	tim4_flag_ = 1;
	// tim4_cnt_++;
}
}
void tim4_pwm_reg_config(void) {
	
	/* ----- CR1 register ----- */
	// CEN bit 0. Counter Enable. 0 disable; 1 enable.
	TIM4->CR1 &= ~(1<<0);

	// CKD[1:0] bits 8:9. Clock division: (1x) 00: t_dts = t_ck_int.
	TIM4->CR1 &= ~(3<<8);

	// ARPE bit 7. Auto-reload preload enable. 1: ARR register is buffered.
	TIM4->CR1 |= (1<<7);

	// CMS[1:0] bits 6:5. Center-aligned mode selection. 00 Depends direction bit;
	TIM4->CR1 &= ~(3<<5);

	// DIR bit 4. 0 upcounter; 1 downcounter.
	TIM4->CR1 &= ~(1<<4);

	// OPM bit 3. One-pulse mode. 0 disable; 1 enable.
	TIM4->CR1 &= ~(1<<3);

	// URS bit 2. Update request source. 0 any event generate interrupt; 1 only ovr generate interrupt.
	TIM4->CR1 &= ~(1<<2);

	// UDIS bit 1. Update disable. 0 update enable; 1 update disable;
	TIM4->CR1 &= ~(1<<1);


	/* ----- CR2 register ----- */
	// TIM4->CR2 

	/* ----- Slave Mode Control Register (SMRC) ----- */
	
	/* ----- Event Generation Register (EGR) ----- */
	// UG bit 0. Update generation. 0 no action; 1 Re-initialize the couter.
	TIM4->EGR |= (1<<0);

	/* ----- Capture/compare mode register (CCMR1) ----- */
	// OC1M bits [6:4]. Output compare 1 mode. 110: PWM mode 1 - in upcounting.
	TIM4->CCMR1 &= ~(7<<4);
	TIM4->CCMR1 |=  (6<<4);

	// OC1PE bit 3. Output compare 1 preload enable. 0 disable; 1 Enable R/W operations to CCR1.
	TIM4->CCMR1 |=  (1<<3);

	// OC1FE bit 2. Output compare 1 fast eanble. 0 fast disable.
	TIM4->CCMR1 &= ~(1<<2);

	// CC1S[1:0] bit [1:0]. Capture compare. 00 CC1 channel is configured as output
	TIM4->CCMR1 &= ~(3<<0);


	/* ----- Capture/compare mode register (CCMR2) ----- */
	TIM4->CCMR2 = 0;

	/* ----- Capture/compare enable register (CCER) ----- */
	// CC1P bit 1. Capture/compare 1 output polarity. 0 active high; 1 active low
	TIM4->CCER &= ~(1<<1);

	// CC1E bit 0. Campture/Compare 1 output enable. 0 disable; 1 enable.
	TIM4->CCER |= (1<<0);

	/* ----- Counter register (CNT) ----- */
	// CNT. Reset counter.
	TIM4->CNT = 0;

	/* ----- Prescale (PSC) ----- */
	// 16 bit value
	TIM4->PSC = 8000-1;

	/* ----- Auto-Reload Register (ARR) ----- */
	// 16 bit. Determines de PWM frequency;
	TIM4->ARR = 1000-1;

	/* ----- Capture/Compare register 1 (CCR1) ----- */
	// 16 bit for duty cycle;
	TIM4->CCR1 = 1000;


	// CEN bit 0. Counter Enable. 0 disable; 1 enable.
	TIM4->CR1 |= (1<<0);
}
