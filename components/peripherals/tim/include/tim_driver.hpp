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

#include "tim.h"
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

		a)

		b)

	2) timer 2 - TIM2

	3) timer 3 - TIM3

	4) timer 4 - TIM4
*/


class TIM_driver {
public:
	uint8_t channel;

	TIM_driver(int timer_num, int freq, timer_mode mode) : timer_num_(timer_num) {

		switch(timer_num) {
			case 1: {
				TIMX_ = TIM1;
				htimX_ = &htim1;
				break;
			}
			case 2: {
				TIMX_ = TIM2;
				htimX_ = &htim2;
				break;
			}
			case 3: {
				TIMX_ = TIM3;
				htimX_ = &htim3;
				break;
			}
			case 4: {
				TIMX_ = TIM4;
				htimX_ = &htim4;
				break;
			}
		}

		init(freq, mode);
	}
	~TIM_driver(void) {}

	void init(int freq, timer_mode mode) {

		sys_clock_ = (HAL_RCC_GetHCLKFreq() / 1000000);

		uint32_t div1 = 1;
		uint32_t div2 = sys_clock_/freq;

		uint32_t div_remain = sys_clock_/freq;

		if(div2 > 65535) {
			div1 = 1000;
			div2 = sys_clock_/div1/freq;
		}

		htimX_->Instance = TIM3; //TIMX_;
		htimX_->Init.Prescaler = 8000;//div1-1;
		htimX_->Init.CounterMode = TIM_COUNTERMODE_UP;
		htimX_->Init.Period = 1000-1;//div2-1;
		htimX_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htimX_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

		if (HAL_TIM_Base_Init(htimX_) != HAL_OK)
		{	
			printf("TIM%d: init error\n", timer_num_);
			Error_Handler();
		} else {
			printf("TIM%d: initialized!\n", timer_num_);
		}

		TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
				if (HAL_TIM_PWM_Init(htimX_) != HAL_OK)
				{
					printf("TIM%d: PWM init error!\n", timer_num_);					
					Error_Handler();
				}

				TIM_MasterConfigTypeDef sMasterConfig = {0};
				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(htimX_, &sMasterConfig) != HAL_OK)
				{
					printf("TIM%d: PWM sync error!\n", timer_num_);										
					Error_Handler();
				}

				TIM_OC_InitTypeDef sConfigOC = {0};
				sConfigOC.OCMode = TIM_OCMODE_PWM1;
				sConfigOC.Pulse = 2;
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

				if (HAL_TIM_PWM_ConfigChannel(htimX_, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
				{
					printf("TIM%d: PWM channel config error!\n", timer_num_);					
					Error_Handler();
				}

				HAL_TIM_MspPostInit_(htimX_);

				if(HAL_TIM_PWM_Start(htimX_, TIM_CHANNEL_3) != HAL_OK) {
					printf("TIM%d: PWM init error!\n", timer_num_);					
					Error_Handler();
				} else {
					printf("TIM%d PWM init\n", timer_num_);
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
		TIMX_->CCR3 = value;
	}
	void set_frequency(uint32_t freq) {

	}

	int get_tim_number(void) {
		return timer_num_;
	}

private:
	// General timer parameters
	int timer_num_;
    uint32_t sys_clock_;
	uint32_t freq_;
	uint32_t duty_cycle_;

	// STM32F103 specifics
	TIM_HandleTypeDef *htimX_;
	TIM_HandleTypeDef htimY_;
    TIM_TypeDef *TIMX_;
	
	void HAL_TIM_MspPostInit_(TIM_HandleTypeDef* timHandle) {
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		
		if(timHandle->Instance==TIM1) {
			
		} else if(timHandle->Instance==TIM2)
		{
			__HAL_RCC_GPIOA_CLK_ENABLE();

			/**TIM2 GPIO Configuration
			PA2     ------> TIM2_CH3
			*/
			GPIO_InitStruct.Pin = GPIO_PIN_2;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		} else if(timHandle->Instance==TIM3) {

		} else if(timHandle->Instance==TIM4) 
		{
			__HAL_RCC_GPIOB_CLK_ENABLE();
			/**TIM4 GPIO Configuration
			PB7     ------> TIM4_CH1
			*/
			GPIO_InitStruct.Pin = GPIO_PIN_7;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}
	}
};

// STM32F TIM interruptions functions
extern "C" {

void TIM1_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim1);
	tim1_flag = 1;
}
void TIM2_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim2);
	tim2_flag = 1;
}
void TIM3_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim3);
	tim3_flag = 1;
}
void TIM4_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim4);
	tim4_flag = 1;
}

}
#endif /* __TIM_DRIVER_HPP__ */
