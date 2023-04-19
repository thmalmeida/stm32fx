/*
 * tim_driver.hpp
 *
 *  Created on: 13 de abr de 2023
 *      Author: titi
 */


/* Briefly description of ADC behavior's

	1
*/
#ifndef TIM_DRIVER_HPP__
#define TIM_DRIVER_HPP__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// STM32----------------------
#include "system_main.h"
#include "stm32_log.h"
// ---------------------------

enum class timer_mode {
    timer_default,
	timer_interrupt,
    pwm_output
};

/*
    Implemented modes:
	General Purpose TIMERS. TIM2 to TIM5;

    1- Timer with interrupt
    2- PWM with output predefined

	Predefined setup options

	1)  timer 1 - TIM1

		a) Timer:

		b) PWM:

			TIM1_CH1:
				- PA8 pin 29;
			TIM1_CH2:
				- PA9 pin 30;
			TIM1_CH3:
				- PA10 pin 31;
			TIM1_CH4
				- PA11 pin 32;

			TIM1_CH1N:
				- PB13 pin 26; 
				- PA7 pin 17 (Remap);
			TIM1_CH2N:
				- PB0 pin 18 (Remap);
				- PB14 pin 27;
			TIM1_CH3N:
				- PB1 pin 19 (Remap);
				- PB15;

	2) timer 2 - TIM2
		a) Timer:
	
		b) PWM:
			TIM2_CH1:
				- PA0 pin 10;
				- PA15 pin 38 (REMAP);
			TIM2_CH2:
				- PA1 pin 11;
				- PB3 (traceswo) pin 39 (REMAP);
			TIM2_CH3:
				- PA2 pin 12;
				- PB10 pin 21 (REMAP);
			TIM2_CH4:
				- PA3 pin 13;
				- PB11 pin 22 (REMAP);

	3) timer 3 - TIM3
		a) Timer:

		b) PWM:
			TIM3_CH1:
				- PA6 pin 16;
				- PB4 pin 40 (Remap) (Default JNTRST);
			TIM3_CH2:
				- PA7 pin 17;
				- PB5 pin 41 (Remap).
			TIM3_CH3:
				- PB0 pin 18;
			TIM3_CH4:
				- PB1 pin 19;

	4) timer 4 - TIM4
		b) PWM:
			TIM4_CH1:
				- PB6 pin 42 (I2C1_SCL);
			TIM4_CH2:
				- PB7 pin 42 (I2C1_SDA);
			TIM4_CH3:
				- PB8 pin 45;
			TIM4_CH4:
				- PB9 pin 45;
*/

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
// -----------------------------

class TIM_driver {
public:

	TIM_driver(int timer_num, int freq, timer_mode mode, int channel = 1) : timer_num_(timer_num), channel_(channel) {

		switch(timer_num) {
			case 1: {
				TIMX_ = TIM1;
				htimX_ = &htim1_;
				break;
			}
			case 2: {
				TIMX_ = TIM2;
				htimX_ = &htim2_;
				break;
			}
			case 3: {
				TIMX_ = TIM3;
				htimX_ = &htim3_;
				break;
			}
			case 4: {
				TIMX_ = TIM4;
				htimX_ = &htim4_;
				break;
			}
		}

		init(freq, mode);
	}
	~TIM_driver(void) {}

	void init(int freq, timer_mode mode) {

		sys_clock_ = HAL_RCC_GetHCLKFreq();

		uint32_t div1 = 1;
		uint32_t div2 = sys_clock_/freq;

		// uint32_t div_remain = sys_clock_/freq;

		if(div2 > 65535) {
			div1 = 1000;
			div2 = sys_clock_/div1/freq;
		}

		duty_max_ = div2-1;
		printf("sys_clock_: %lu, div1:%lu, freq:%d, div2:%lu", sys_clock_, div1, freq, div2);
		printf("Duty max: %lu\n", duty_max_);

		htimX_->Instance = TIMX_;
		htimX_->Init.Prescaler = div1-1;
		htimX_->Init.CounterMode = TIM_COUNTERMODE_UP;
		htimX_->Init.Period = div2-1;
		htimX_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htimX_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

		if (HAL_TIM_Base_Init(htimX_) != HAL_OK)
		{	
			printf("TIM%d: init error\n", timer_num_);
			Error_Handler();
		} else {
			printf("TIM%d: initialized!\n", timer_num_);
		}

		TIM_ClockConfigTypeDef sClockSourceConfig;
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(htimX_, &sClockSourceConfig) != HAL_OK)
		{
			printf("TIM%d: clock config error!\n", timer_num_);
			Error_Handler();
		} else {
			printf("TIM%d: clock config!\n", timer_num_);
		}

		switch(mode) {
			case timer_mode::timer_interrupt: {
				if(HAL_TIM_Base_Start_IT(htimX_) != HAL_OK) {	// update interrupt enable;
					printf("TIM%d: interrupt error!\n", timer_num_);
					Error_Handler();
				} else {
					printf("TIM%d: interrupt init!\n", timer_num_);
				}
				break;
			}
			case timer_mode::pwm_output: {

				switch (channel_) {
					case 1:
						channel_addr_ = TIM_CHANNEL_1;
						break;
					case 2:
						channel_addr_ = TIM_CHANNEL_2;
						break;
					case 3:
						channel_addr_ = TIM_CHANNEL_3;
						break;
					case 4:
						channel_addr_ = TIM_CHANNEL_4;
						break;
					default:
						channel_addr_ = TIM_CHANNEL_1;
						break;
				}


				if (HAL_TIM_PWM_Init(htimX_) != HAL_OK) {
					printf("TIM%d: PWM init error!\n", timer_num_);					
					Error_Handler();
				} else {
					printf("TIM%d PWM init\n", timer_num_);
				}

				TIM_MasterConfigTypeDef sMasterConfig;
				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(htimX_, &sMasterConfig) != HAL_OK) {
					printf("TIM%d: PWM sync error!\n", timer_num_);										
					Error_Handler();
				}

				TIM_OC_InitTypeDef sConfigOC;
				sConfigOC.OCMode = TIM_OCMODE_PWM1;
				sConfigOC.Pulse = 0;
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
				sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
				sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

				if (HAL_TIM_PWM_ConfigChannel(htimX_, &sConfigOC, channel_addr_) != HAL_OK)
				{
					printf("TIM%d: PWM channel config error!\n", timer_num_);					
					Error_Handler();
				}

				HAL_TIM_MspPostInit_(htimX_, channel_);

				if(HAL_TIM_PWM_Start(htimX_, channel_addr_) != HAL_OK) {
					printf("TIM%d: PWM init error!\n", timer_num_);					
					Error_Handler();
				} else {
					printf("TIM%d PWM channel %d init\n", timer_num_, channel_);
				}
				break;
			}
			default:
				break;
		}
	}
	void reset_cnt(void) {
		TIMX_->CNT = 0;
	}
	void set_cnt(uint32_t value) {
		TIMX_->CNT = value;
	}
	void set_duty_cycle(uint32_t value) {
		uint32_t pulse_width = static_cast<uint32_t>(value*duty_max_/100.0);
		__HAL_TIM_SET_COMPARE(htimX_, channel_addr_, pulse_width);
	}
	void set_frequency(uint32_t freq) {
		uint32_t value = freq*2;
		__HAL_TIM_SET_AUTORELOAD(htimX_, value);
	}
	int get_tim_number(void) {
		return timer_num_;
	}

private:
	// General timer parameters
	int timer_num_;
	int channel_;
	uint32_t duty_max_;
    uint32_t sys_clock_;
	uint32_t freq_;
	uint32_t duty_cycle_;

	// STM32F103 specifics
	TIM_HandleTypeDef *htimX_;
	// TIM_HandleTypeDef htimY_;
    TIM_TypeDef *TIMX_;
	uint32_t channel_addr_;

	void HAL_TIM_MspPostInit_(TIM_HandleTypeDef* timHandle, int channel) {
		GPIO_InitTypeDef GPIO_InitStruct;
		
		if(timHandle->Instance==TIM1) {
			switch(channel) {
				case 1: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_8;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);					
					break;
				}
				case 2: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_9;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					break;
				}
				case 3: {
					__HAL_RCC_GPIOA_CLK_ENABLE();					
					GPIO_InitStruct.Pin = GPIO_PIN_10;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					break;
				}
				case 4: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_11;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					break;
				}
				default:
					printf("TIM PWM channel error\n");
					break;
			}			
		} else if(timHandle->Instance==TIM2) {
			switch(channel) {
				case 1: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_0;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);					
					break;
				}
				case 2: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_1;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					break;
				}
				case 3: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_2;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					break;
				}
				case 4: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_3;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					printf("HOLLA!\n");
					break;
				}
				default:
					printf("TIM PWM channel error\n");
					break;
			}
		} else if(timHandle->Instance==TIM3) {
			switch(channel) {
				case 1: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_6;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);					
					break;
				}
				case 2: {
					__HAL_RCC_GPIOA_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_7;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					break;
				}
				case 3: {
					__HAL_RCC_GPIOB_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_0;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					break;
				}
				case 4: {
					__HAL_RCC_GPIOB_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_1;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					break;
				}
				default:
					printf("TIM PWM channel error\n");
					break;
			}
		} else if(timHandle->Instance==TIM4) {
			switch(channel) {
				case 1: {
					__HAL_RCC_GPIOB_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_6;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);					
					break;
				}
				case 2: {
					__HAL_RCC_GPIOB_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_7;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					printf("TIM4_CH2!!!!\n");
					break;
				}
				case 3: {
					__HAL_RCC_GPIOB_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_8;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					break;
				}
				case 4: {
					__HAL_RCC_GPIOB_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_9;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					break;
				}
				default:
					printf("TIM PWM channel error\n");
					break;
			}
		}
	}
};


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
#endif /* __TIM_DRIVER_HPP__ */
