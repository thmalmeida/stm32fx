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

class TIM_driver {
public:

	TIM_driver(int timer_num, int freq, timer_mode mode, int channel = 1) : timer_num_(timer_num), channel_(channel) {

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
	uint32_t get_uptime(void) {
		return *tim_cnt_;
	}
	int get_isr_flag(void) {
		if(*tim_isr_flag_) {
			*tim_isr_flag_ = 0;
			return 1;
		} else
			return 0;
	}

private:
	// General timer parameters
	int timer_num_;
	int channel_;
	uint32_t duty_max_;
    uint32_t sys_clock_;
	uint32_t freq_;
	uint32_t duty_cycle_;
	uint32_t* tim_cnt_;
	uint8_t* tim_isr_flag_;

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

// void tim4_pwm_reg_config(void) {
	
// 	/* ----- CR1 register ----- */
// 	// CEN bit 0. Counter Enable. 0 disable; 1 enable.
// 	TIM4->CR1 &= ~(1<<0);

// 	// CKD[1:0] bits 8:9. Clock division: (1x) 00: t_dts = t_ck_int.
// 	TIM4->CR1 &= ~(3<<8);

// 	// ARPE bit 7. Auto-reload preload enable. 1: ARR register is buffered.
// 	TIM4->CR1 |= (1<<7);

// 	// CMS[1:0] bits 6:5. Center-aligned mode selection. 00 Depends direction bit;
// 	TIM4->CR1 &= ~(3<<5);

// 	// DIR bit 4. 0 upcounter; 1 downcounter.
// 	TIM4->CR1 &= ~(1<<4);

// 	// OPM bit 3. One-pulse mode. 0 disable; 1 enable.
// 	TIM4->CR1 &= ~(1<<3);

// 	// URS bit 2. Update request source. 0 any event generate interrupt; 1 only ovr generate interrupt.
// 	TIM4->CR1 &= ~(1<<2);

// 	// UDIS bit 1. Update disable. 0 update enable; 1 update disable;
// 	TIM4->CR1 &= ~(1<<1);


// 	/* ----- CR2 register ----- */
// 	// TIM4->CR2 

// 	/* ----- Slave Mode Control Register (SMRC) ----- */
	
// 	/* ----- Event Generation Register (EGR) ----- */
// 	// UG bit 0. Update generation. 0 no action; 1 Re-initialize the couter.
// 	TIM4->EGR |= (1<<0);

// 	/* ----- Capture/compare mode register (CCMR1) ----- */
// 	// OC1M bits [6:4]. Output compare 1 mode. 110: PWM mode 1 - in upcounting.
// 	TIM4->CCMR1 &= ~(7<<4);
// 	TIM4->CCMR1 |=  (6<<4);

// 	// OC1PE bit 3. Output compare 1 preload enable. 0 disable; 1 Enable R/W operations to CCR1.
// 	TIM4->CCMR1 |=  (1<<3);

// 	// OC1FE bit 2. Output compare 1 fast eanble. 0 fast disable.
// 	TIM4->CCMR1 &= ~(1<<2);

// 	// CC1S[1:0] bit [1:0]. Capture compare. 00 CC1 channel is configured as output
// 	TIM4->CCMR1 &= ~(3<<0);


// 	/* ----- Capture/compare mode register (CCMR2) ----- */
// 	TIM4->CCMR2 = 0;

// 	/* ----- Capture/compare enable register (CCER) ----- */
// 	// CC1P bit 1. Capture/compare 1 output polarity. 0 active high; 1 active low
// 	TIM4->CCER &= ~(1<<1);

// 	// CC1E bit 0. Campture/Compare 1 output enable. 0 disable; 1 enable.
// 	TIM4->CCER |= (1<<0);

// 	/* ----- Counter register (CNT) ----- */
// 	// CNT. Reset counter.
// 	TIM4->CNT = 0;

// 	/* ----- Prescale (PSC) ----- */
// 	// 16 bit value
// 	TIM4->PSC = 8000-1;

// 	/* ----- Auto-Reload Register (ARR) ----- */
// 	// 16 bit. Determines de PWM frequency;
// 	TIM4->ARR = 1000-1;

// 	/* ----- Capture/Compare register 1 (CCR1) ----- */
// 	// 16 bit for duty cycle;
// 	TIM4->CCR1 = 1000;


// 	// CEN bit 0. Counter Enable. 0 disable; 1 enable.
// 	TIM4->CR1 |= (1<<0);
// }
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
#endif /* __TIM_DRIVER_HPP__ */
