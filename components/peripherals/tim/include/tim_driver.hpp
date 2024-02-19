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

#define DEBUG_TIM_DRIVER 1

enum class timer_mode {
    timer_default,
	timer_interrupt,
	timer_counter,
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
				- PA0 (4) - pin 10;
				- PA15 pin 38 (REMAP);
			TIM2_CH2:
				- PA1 (5) - pin 11;
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

	TIM_DRIVER(int timer_num, double f, timer_mode mode, int channel = 1) : timer_num_(timer_num), f_usr_(f), mode_(mode), channel_(channel) {

		switch(timer_num) {
			case 1: {
				TIMX_ = TIM1;
				htimX_ = &htim1_;
				tim_isr_flag_ = &tim1_flag_;
				tim_cnt_ = &tim1_cnt_;
				// it uses APB2 prescaler
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
		init();
	}
	~TIM_DRIVER(void) {}

	void init(void) {

		#ifdef DEBUG_TIM_DRIVER
		printf("TIM%d initializing...\n", timer_num_);
		#endif
		// calculate PSC_ and ARR_ registers given a frequency
		timer_calculation_();

		htimX_->Instance = TIMX_;
		switch(CKD_) {
			case 1:
				htimX_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;	// TIMx_CR1. Set CKD[1:0] bits
				break;
			case 2:
				htimX_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;	// TIMx_CR1. Set CKD[1:0] bits
				break;
			case 4:
				htimX_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;	// TIMx_CR1. Set CKD[1:0] bits
				break;
			default:
				htimX_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;	// TIMx_CR1. Set CKD[1:0] bits
				break;
		}
		htimX_->Init.Prescaler = PSC_;									// TIMx_PSC 16 bit register (clk division)
		htimX_->Init.Period = static_cast<uint32_t>(ARR_);										// TIMx_ARR register (count up to this register value)
		htimX_->Init.CounterMode = TIM_COUNTERMODE_UP;					// TIMx_CR1 set DIR bit
		htimX_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;	// TIMx_CR1 set ARPE bit 7

		if (HAL_TIM_Base_Init(htimX_) != HAL_OK) {
			printf("TIM%d: base init error\n", timer_num_);
			Error_Handler();
		} else {
			printf("TIM%d: base init\n", timer_num_);
		}

		TIM_ClockConfigTypeDef sClockSourceConfig;
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(htimX_, &sClockSourceConfig) != HAL_OK) {
			printf("TIM%d: clk config error\n", timer_num_);
			Error_Handler();
		} else {
			printf("TIM%d: clk config\n", timer_num_);
		}

		switch(mode_) {
			case timer_mode::timer_interrupt: {
				if(HAL_TIM_Base_Start_IT(htimX_) != HAL_OK) {	// update interrupt enable;
					printf("TIM%d: interrupt error\n", timer_num_);
					Error_Handler();
				} else
					printf("TIM%d: interrupt init\n", timer_num_);
				break;
			}
			case timer_mode::timer_counter: {
				// printf("TIM%d: interrupt without interrupt!\n", timer_num_);
				HAL_TIM_Base_Start(htimX_);	// Enable the counter. Or use enable_cnt();
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
						printf("TIM ch Error\n");
						Error_Handler();
						break;
				}

				if (HAL_TIM_PWM_Init(htimX_) != HAL_OK) {
					printf("TIM%d: PWM init error\n", timer_num_);					
					Error_Handler();
				} else {
					printf("TIM%d PWM init\n", timer_num_);
				}

				TIM_MasterConfigTypeDef sMasterConfig;
				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(htimX_, &sMasterConfig) != HAL_OK) {
					printf("TIM%d: PWM sync error\n", timer_num_);										
					Error_Handler();
				} else {
					printf("TIME master sync done\n");
				}

				TIM_OC_InitTypeDef sConfigOC;
				sConfigOC.OCMode = TIM_OCMODE_PWM1;
				sConfigOC.Pulse = static_cast<uint16_t>(ARR_/2.0);
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

				sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
				sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
				sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
				if (HAL_TIM_PWM_ConfigChannel(htimX_, &sConfigOC, channel_addr_) != HAL_OK) {
					printf("TIM%d: PWM channel config error\n", timer_num_);					
					Error_Handler();
				} else {
					printf("TIM%d: PWM channel configured\n", timer_num_);
				}

				HAL_TIM_MspPostInit_(htimX_, channel_);

				if(HAL_TIM_PWM_Start(htimX_, channel_addr_) != HAL_OK) {
					printf("TIM%d: PWM init error\n", timer_num_);					
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
		// TIMX_->CNT = 0;
		__HAL_TIM_SET_COUNTER(htimX_, 0);
	}
	void set_cnt(uint32_t value) {
		__HAL_TIM_SET_COUNTER(htimX_, value);	// or TIMX_->CNT = value;
	}
	uint32_t get_cnt(void) {
		return __HAL_TIM_GET_COUNTER(htimX_); // or
		// return TIMX_->CNT;
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
		__HAL_TIM_ENABLE(htimX_); // or TIMX_->CR1 |= TIM_CR1_CEN;
	}
	void disable_cnt(void) {
		__HAL_TIM_DISABLE(htimX_);	// or TIMX_CR1 &= ~(TIM_CR1_CEN);
	}
	// This attribute force compile include this functions avoid optimization
	int __attribute__((__used__)) isr_flag(void) {
		if(*tim_isr_flag_) {
			*tim_isr_flag_ = 0;
			return 1;
		} else
			return 0;
	}
	// void reset_isr_flag(void) {
	// 	*tim_isr_flag_ = 1;
	// }

	uint16_t get_ARR(void) {
		return ARR_;
	}

private:
	// General timer parameters
	int timer_num_;					// timer number;
	double f_usr_;					// user frequency requested [Hz];
	timer_mode mode_;				// working mode
	int channel_;					// user channel selected;
	uint32_t duty_max_;				// max duty cycle [%];
    uint32_t f_sys_;				// system clock frequency [Hz];
	double f_bus_;					// bus frequency link to tim [Hz];
	double f_cnt_;					// The frequency of update the cnt register;
	double f_tim_;					// The frequency of timer. 1/T. T is the time to full cycle from 0 to max value;
	double T_tim_;					// TIM time period [s];
	uint32_t* tim_cnt_;				// The instant counter ((TIMX_CNT) register value;
	uint8_t* tim_isr_flag_;			// Handle flag to connect with ISR. Become 1 when interrupt occurs;
	// uint32_t duty_cycle_;
	// uint32_t cnt_clk_;		
	// uint32_t timer_clk_;	

	/* ----- STM32F103 specifics ----- */
	TIM_HandleTypeDef *htimX_;		// TIM structure handle;
    TIM_TypeDef *TIMX_;				// TIM module number;
	uint32_t channel_addr_;			// TIM addr number;

	// These prescalers are configured on RCC clock tree. MUST BE EQUAL ON REGISTER TREE!
	uint16_t AHB_div_ = 1;			// RCC bus prescaler (1, 2, 4, 8, 16, 32, 64, 256 and 512)
	uint8_t APB1_div_ = 2;			// RCC prescaler for TIM2, 3, 4 (divs: 1, 2, 4);
	uint8_t APB2_div_ = 2;			// RCC prescaler for TIM1 (divs: 1, 2, 4);
	uint8_t CKD_v_[3] = {1, 2, 4};	// TIM clock division (1, 2 or 4) on CR1. CKD[1:0] bits 9 and 8;
	uint8_t CKD_ = 1;				// TIM counter divider
	uint16_t PSC_ = 1;				// TIM Prescaler register 16-bit value;
	uint16_t ARR_ = 1;				// TIM Auto reload Register 16-bit value;

	// STM32F103 16-bit timer module registers (stm32f101x6.h file)
	uint16_t TIMX_CR1_, TIMX_CR2_;
	uint16_t TIMX_DIER_, TIMX_EGR_, TIMX_SR_;
	uint16_t TIMX_CNT_, TIMX_ARR_, TIMX_PSC_, TIMX_CKD_;

	uint8_t TIMX_EGR_UG_bit = 0;

	// STM32F103 16-bit Reset and Clock Control 16-bit registers
	uint16_t RCC_CR_;			// RCC Control Register
	uint16_t RCC_AHBENR_;
	uint16_t RCC_APB2ENR_;
	uint32_t RCC_APB1RSTR_, RCC_APB1ENR_, RCC_CFGR_;
	// RCC_TypeDef RCCx;
	/* ----- STM32F103 specifics ----- */

	void update_registers_(void) {
		TIMX_CR1_ = static_cast<uint16_t>(TIMX_->CR1);
		TIMX_CR2_ = static_cast<uint16_t>(TIMX_->CR2);
		TIMX_SR_ = static_cast<uint16_t>(TIMX_->SR);
		TIMX_PSC_ = static_cast<uint16_t>(TIMX_->PSC);
		TIMX_ARR_ = static_cast<uint16_t>(TIMX_->ARR);
		TIMX_EGR_ = static_cast<uint16_t>(TIMX_->EGR);
		TIMX_DIER_ = static_cast<uint16_t>(TIMX_->DIER);
		RCC_APB1RSTR_ = RCC->APB1RSTR;
		RCC_APB1ENR_ = RCC->APB1ENR;
	}
	/*
	* @param _f desired frequency
	*/
	void timer_calculation_(void) {

		// Desired time period [s];
		double _t = 1/f_usr_;

		// System clock frequency [Hz]
		f_sys_ = HAL_RCC_GetHCLKFreq();

		// Bus frequency - update AHB, APB1 and APB2 prescalers from RCC Register
		get_AHB_APBx_div_();

		// According with RCC clock tree into datasheet on page 93;
		uint8_t APBx_div = 0;
		if(timer_num_ > 1) {
			// TIM2, 3, 4 uses APB1 low speed
			APBx_div = APB1_div_;
		} else
			// TIM1 uses APB2 high speed
			APBx_div = APB2_div_;

		if(APBx_div == 1) {
			f_bus_ =  1.0*static_cast<double>(f_sys_)/(static_cast<double>(AHB_div_)*static_cast<double>(APBx_div));
			printf("1x freq\n");
		}
		else {
			f_bus_ = 2.0*static_cast<double>(f_sys_)/(static_cast<double>(AHB_div_)*static_cast<double>(APBx_div));	// suppose to 2x
			printf("2x freq\n");
		}	

		// double kt1 = AHB_div_*APB1_div_/f_sys_; // Time constant 1;
		// double T_cnt_ = kt1*CKD_*PSC_/f_sys_;   // Counter period [s];
		// double T_tim_ = (AHB_div_*APB1_div_/f_sys_)*CKD_*PSC_*ARR_/f_sys_;
	
		double T_tim_max = static_cast<double>(CKD_v_[2])*65535.0*65535.0/f_bus_;
		if(_t > T_tim_max) {
			printf("Max tim period: %.1f s\n", T_tim_max);
			return;
		}

		// printf("Start The Timer finder\n");
		int a = 0;
		int ARR_temp_ = 0;
		for(uint8_t k=0; k<3; k++) {
			if(a == 1) {
				// printf("a = 1 break!\n");
				break;
			}

			CKD_ = CKD_v_[k];
			// printf("CKD_:%u\n", CKD_);
			for(int j=1; j<65536; j++) {
				PSC_ = static_cast<uint16_t>(j);
				// printf("PSC_:%u\n", PSC_);
				
				// counter frequency [Hz];
				f_cnt_ = f_bus_/(static_cast<double>(CKD_)*static_cast<double>(PSC_+1));

				ARR_temp_ = f_cnt_*_t;
				if(ARR_temp_ < 65536.0) {
					// printf("Found!\n");
					a = 1;
					ARR_ = static_cast<uint16_t>(ARR_temp_);
					break;
				}

				// for(uint16_t i=1; i<65535; i++) {
				// 	ARR_ = i;
				// 	T_tim_ = static_cast<double>(ARR_)/f_cnt_;
				// 	if(T_tim_ >= _t) {
				// 		printf("Found!\n");
				// 		a = 1;
				// 		break;
				// 	}
				// }
			}
		}

		// timer frequency [Hz];
		f_tim_ = f_cnt_/static_cast<double>(ARR_);

		// Timer period [s];
		T_tim_ = (static_cast<double>(ARR_))/f_cnt_;

		printf("f_sys_: %.0lu MHz\n", f_sys_/1000000);
		printf("f_bus_: %.0f kHz\n", f_bus_/1000);
		printf("f_cnt_: %.0f Hz\n", f_cnt_);
		printf("f_tim_: %.3f Hz\n", f_tim_);
		printf("T_tim_: %.3f ms\n", T_tim_*1000);
		printf("CKD_:%u, PSC_:%u, ARR_:%u\n", CKD_, PSC_, ARR_);
	}

	void set_TIM_EGR_UG_bit(void) {
		TIMX_EGR_ |= (1<<0);
	}

	void get_TIM_CNT_(void) {
		TIMX_CNT_ = static_cast<uint16_t>(TIMX_->CNT);

		printf("TIMX_CNT_:%u\n", TIMX_CNT_);
	}
	void get_TIM_ARR_(void) {
		TIMX_ARR_ = static_cast<uint16_t>(TIMX_->ARR);
		printf("TIMX_ARR_:%u\n", TIMX_ARR_);
	}
	void get_TIM_CKD_(void) {
		TIMX_CKD_ = static_cast<uint16_t>((TIMX_->CR1 >> 8) & 0x00000003);
		printf("TIMX_CKD_:%u\n", TIMX_CKD_);
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
	void get_RCC_CFGR_(void) {
		RCC_CFGR_ = RCC->CFGR;
	}
	void get_AHB_APBx_div_(void) {
		get_RCC_CFGR_();
		// Find AHB prescale - HPRE[3:0] bits - 7 to 4 bits of CFGR

		uint32_t AHB_div_t_ = (RCC_CFGR_ >> 4) & 0x0000000F;
		switch (AHB_div_t_) {
			case 8:
				AHB_div_ = 2;
				break;
			case 9:
				AHB_div_ = 4;
				break;
			case 10:
				AHB_div_ = 8;
				break;
			case 11:
				AHB_div_ = 16;
				break;
			case 12:
				AHB_div_ = 64;
				break;
			case 13:
				AHB_div_ = 128;
				break;
			case 14:
				AHB_div_ = 256;
				break;
			case 15:
				AHB_div_ = 512;
				break;
			default:
				AHB_div_ = 1;
				break;
		}

		// Find APB1 prescale PPRE1[2:0] starting at bit 8 to bit 10 on RCC CFGR register
		uint32_t APB1_div_t_ = (RCC_CFGR_ >> 8) & 0x00000007;
		switch (APB1_div_t_) {
			case 4:
				APB1_div_ = 2;
				break;
			case 5: 
				APB1_div_ = 4;
				break;
			case 6:
				APB1_div_ = 8;
				break;
			case 7:
				APB1_div_ = 16;
				break;
			default:
				APB1_div_ = 1; // not divided
				break;
		}

		// Find APB2 prescale
		uint32_t APB2_div_t_ = (RCC_CFGR_ >> 11) & 0x00000007;
		switch (APB2_div_t_) {
			case 4:
				APB2_div_ = 2;
				break;
			case 5: 
				APB2_div_ = 4;
				break;
			case 6:
				APB2_div_ = 8;
				break;
			case 7:
				APB2_div_ = 16;
				break;
			default:
				APB2_div_ = 1; // not divided
				break;
		}
		printf("AHB_div_:%u, APB1_div_:%u, APB2_div_:%u\n", AHB_div_, APB1_div_, APB2_div_);
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
