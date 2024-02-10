/*
 * tim_driver.hpp
 *
 *  Created on: 13 de abr de 2023
 *      Author: titi
 */


/* Briefly description of ADC behavior's

	1
*/
#ifndef _TIM_DRIVER_HPP__
#define _TIM_DRIVER_HPP__

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
	timer_counter_only,
    pwm_output
};

/*
    Implemented modes:
	General Purpose TIMERS. TIM2 to TIM5;

    - Timer with interrupt
    - PWM with output predefined

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

	Maximum frequency tested was 2.6 MHz (2.666666 MHz)
	with f_clk = 8*10^6 MHz

	With f_clk = 72 MHz
	TIM1 on channel 1 and duty 50% achieve 14400000 Hz. But, it was a senoidal signal.
*/

// ----- STM32F specifics -----
extern TIM_HandleTypeDef htim1_;
extern uint8_t tim1_flag_;
extern uint32_t tim1_cnt_;

extern TIM_HandleTypeDef htim2_;
extern uint8_t tim2_flag_;
extern uint32_t tim2_cnt_;

extern TIM_HandleTypeDef htim3_;
extern uint8_t tim3_flag_;
extern uint32_t tim3_cnt_;

extern TIM_HandleTypeDef htim4_;
extern uint8_t tim4_flag_;
extern uint32_t tim4_cnt_;
// -----------------------------

/* Timer can work into two ways now
* 
* 1- Interrupt
*
* 2- PWM output
*/

class TIM_DRIVER {
public:

	TIM_DRIVER(int timer_num, int freq, timer_mode mode, int channel = 1) : timer_num_(timer_num), channel_(channel) {

		switch(timer_num) {
			case 1: {
				TIMX_ = TIM1;
				htimX_ = &htim1_;
				tim_isr_flag_ = &tim1_flag_;
				tim_cnt_ = &tim1_cnt_;
				break;
			}
			case 2: {
				TIMX_ = TIM2;
				htimX_ = &htim2_;
				tim_isr_flag_ = &tim2_flag_;
				tim_cnt_ = &tim2_cnt_;
				break;
			}
			case 3: {
				TIMX_ = TIM3;
				htimX_ = &htim3_;
				tim_isr_flag_ = &tim3_flag_;
				tim_cnt_ = &tim3_cnt_;
				break;
			}
			case 4: {
				TIMX_ = TIM4;
				htimX_ = &htim4_;
				tim_isr_flag_ = &tim4_flag_;
				tim_cnt_ = &tim4_cnt_;
				break;
			}
		}

		init(freq, mode);
	}
	~TIM_DRIVER(void) {}

	void init(int freq, timer_mode mode) {

		sys_clock_ = HAL_RCC_GetHCLKFreq();

		uint16_t div1 = sys_clock_/freq;
		uint16_t div2 = 0;

		if(mode ==  timer_mode::timer_counter_only) {
			div1 = sys_clock_/freq;
			div2 = 65536;
		} else {
			if(div2 > 65535) {
				div1 = 1000;
				div2 = sys_clock_/div1/freq;
			} else {
				div2 = sys_clock_/freq;
				// uint32_t div_remain = sys_clock_/freq;
			}
		}

		duty_max_ = div2-1;
		printf("sys_clock_: %lu, div1:%u, freq:%d, div2(ARR):%u ", sys_clock_, div1, freq, div2);
		printf("Duty max: %lu\n", duty_max_);

		htimX_->Instance = TIMX_;
		htimX_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;			// TIMx_CR1. Set CKD[1:0] bits
		htimX_->Init.Prescaler = div1-1;								// TIMx_PSC 16 bit register (clk division)
		htimX_->Init.Period = div2-1;									// TIMx_ARR register (count up to this register value)
		htimX_->Init.CounterMode = TIM_COUNTERMODE_UP;					// TIMx_CR1 set DIR bit
		htimX_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;// TIMx_CR1 set ARPE bit 7

		// htimX_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

		if (HAL_TIM_Base_Init(htimX_) != HAL_OK) {
			printf("TIM%d: base init error!\n", timer_num_);
			Error_Handler();
		} else {
			printf("TIM%d: base init\n", timer_num_);
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
			case timer_mode::timer_counter_only: {

				// HAL_Delay(1000);
				// get_TIM_ARR_();
				// get_TIM_CR1_();
				// get_TIM_CR2_();
				// get_TIM_DIER_();
				// get_TIM_EGR_();
				// get_TIM_PSC_();
				// get_TIM_SR_();
				// printf("TIM%d: interrupt without interrupt!\n", timer_num_);
				enable_cnt();
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
	uint32_t get_cnt(void) {
		return TIMX_->CNT;
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
	uint32_t get_uptime(void) {
		return *tim_cnt_;
	}
	void enable_cnt(void) {
		TIMX_->CR1 |= (1<<0);
	}
	// This attribute force compile include this functions avoid optimization
	int __attribute__((__used__)) get_isr_flag(void) {
		if(*tim_isr_flag_) {
			*tim_isr_flag_ = 0;
			return 1;
		} else
			return 0;
	}
	void set_isr_flag(void) {
		*tim_isr_flag_ = 1;
	}

private:
	// General timer parameters
	int timer_num_;
	int channel_;
	uint32_t duty_max_;
    uint32_t sys_clock_;
	uint32_t freq_;
	uint32_t freq_cnt_;
	uint32_t duty_cycle_;
	uint32_t* tim_cnt_;		// The instant counter register value;
	uint8_t* tim_isr_flag_;	// Handle flag to connect with ISR. Become 1 when interrupt occurs;
	uint32_t cnt_clk_;		// The frequency of update the cnt register;
	uint32_t timer_clk_;	// The frequency of timer. 1/T. T is the time to full cycle from 0 to max value;


	/* ----- STM32F103 specifics ----- */
	// TIM_HandleTypeDef htimY_;
	TIM_HandleTypeDef *htimX_;
    TIM_TypeDef *TIMX_;
	uint32_t channel_addr_;

	// These prescalers are configured on RCC clock tree. MUST BE EQUAL ON REGISTER TREE!
	uint8_t prescale_AHB_ = 1;
	uint8_t prescale_APB1_ = 2;
	uint8_t prescale_APB2_ = 2;

	// STM32F103 16-bit timer module registers (stm32f101x6.h file)
	uint16_t TIMX_CR1_, TIMX_CR2_;
	uint16_t TIMX_DIER_, TIMX_EGR_, TIMX_SR_;
	uint16_t TIMX_CNT_, TIMX_ARR_, TIMX_PSC_;

	void update_registers_(void) {
		TIMX_CR1_ = static_cast<uint16_t>(TIMX_->CR1);
		TIMX_CR2_ = static_cast<uint16_t>(TIMX_->CR2);
		TIMX_SR_ = static_cast<uint16_t>(TIMX_->SR);
		TIMX_PSC_ = static_cast<uint16_t>(TIMX_->PSC);
		TIMX_EGR_ = static_cast<uint16_t>(TIMX_->EGR);
	}

	// STM32F103 16-bit Reset and Clock Control 16-bit registers
	uint16_t RCC_CR_;
	uint16_t RCC_AHBENR_;
	uint16_t RCC_APB2ENR_;
	uint32_t RCC_APB1RSTR_, RCC_APB1ENR_;
	// RCC_TypeDef RCCx;

	void get_TIM_CNT_(void) {
		TIMX_CNT_ = static_cast<uint16_t>(TIMX_->CNT);
		printf("TIMX_CNT_:%u\n", TIMX_CNT_);
	}
	void get_TIM_ARR_(void) {
		TIMX_ARR_ = static_cast<uint16_t>(TIMX_->ARR);
		printf("TIMX_ARR_:%u\n", TIMX_ARR_);
	}
	void get_TIM_PSC_(void) {
		TIMX_PSC_ = static_cast<uint16_t>(TIMX_->PSC);
		printf("TIMX_PSC_:%u\n", TIMX_PSC_);
	}
	void get_TIM_CR1_(void) {
		TIMX_CR1_ = static_cast<uint16_t>(TIMX_->CR1);
		printf("TIM%d_CR1:0x%04x\n", timer_num_, TIMX_CR1_);
	}
	void get_TIM_CR2_(void) {
		TIMX_CR2_ = static_cast<uint16_t>(TIMX_->CR2);
		printf("TIMX_CR2:0x%04x\n", TIMX_CR2_);
	}
	void get_TIM_EGR_(void) {
		TIMX_EGR_ = static_cast<uint16_t>(TIMX_->EGR);
		printf("TIMX_EGR:0x%04x\n", TIMX_EGR_);
	}
	void get_TIM_SR_(void) {
		TIMX_SR_ = static_cast<uint16_t>(TIMX_->SR);
		printf("TIMX_SR_:0x%04x\n", TIMX_SR_);
	}
	void get_TIM_DIER_(void) {
		TIMX_DIER_ = static_cast<uint16_t>(TIMX_->DIER);
		printf("TIMX_DIER_:0x%04x\n", TIMX_DIER_);
	}
	void get_RCC_APB1RSTR_(void) {
		RCC_APB1RSTR_ = RCC->APB1RSTR;
		printf("RCC_APB1RSTR_:0x%08lx\n", RCC_APB1RSTR_);
	}
	void get_RCC_APB1ENR_(void) {
		RCC_APB1ENR_ = RCC->APB1ENR;
		printf("RCC_APB1ENR_:0x%08lx\n", RCC_APB1ENR_);
	}

	void set_TIMx_EGR_UG(void) {
		TIMX_->EGR |= (1<<0);
	}

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
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
// STM32F TIM interruptions functions
void TIM1_UP_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
}

void tim4_pwm_reg_config(void);

#endif /* __TIM_DRIVER_HPP__ */
