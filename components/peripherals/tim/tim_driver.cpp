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
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	if(tim_baseHandle->Instance==TIM1) {
		/* TIM1 clock enable */
		__HAL_RCC_TIM1_CLK_ENABLE();

		HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	} else if(tim_baseHandle->Instance==TIM2) {
		/* TIM2 clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();

		/* TIM2 interrupt Init */
		HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	} else if(tim_baseHandle->Instance==TIM3) {
		/* TIM3 clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();

		/* TIM3 interrupt Init */
		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	} else if(tim_baseHandle->Instance==TIM4) {
		/* TIM4 clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE();

		printf("TIM4 CLOCK ENABLE!\n");

		/* TIM4 interrupt Init */
		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
	}
}
// STM32F TIM interruptions functions
void TIM1_UP_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_UP_IRQn 0 */

	/* USER CODE END TIM1_UP_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1_);
	tim1_flag_ = 1;
	/* USER CODE BEGIN TIM1_UP_IRQn 1 */

	/* USER CODE END TIM1_UP_IRQn 1 */
}
/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_CC_IRQn 0 */

	/* USER CODE END TIM1_CC_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1_);
	/* USER CODE BEGIN TIM1_CC_IRQn 1 */

	/* USER CODE END TIM1_CC_IRQn 1 */
}
void TIM2_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim2_);
	tim2_flag_ = 1;
}
void TIM3_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim3_);
	tim3_flag_ = 1;
}
void TIM4_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim4_);
	tim4_flag_ = 1;
}
}