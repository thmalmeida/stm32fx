/*
 * gpio.h
 *
 *  Created on: 22 de fev de 2017
 *      Author: titi
 * modified on: 2023-03-01
 */

#ifndef GPIO_HPP__
#define GPIO_HPP__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "delay.h"

// STM32----------------------
#include "stm32_log.h"
#include "stm32f1xx_hal.h"
// -----------------------


/**
 * TODO fazer melhor controle da variavel driver_instaled, que contém quantos dispositivos
 * 	estao se utilizando do driver de interrucao
 */

class GPIO_driver{
	public:
		GPIO_driver(int pin_number, int direction);

		void mode(int direction);
		int read(void);
		void write(int level);
		void toggle(void);
		void reset(void) noexcept;

		// void register_interrupt(gpio_isr_t handler, void* isr_args);
		// void unregister_interrupt();

		// void enable_interrupt(gpio_int_type_t type);
		// void disable_interrupt();

		// void pull(gpio_pull_mode_t mode);

		// void strength(gpio_drive_cap_t cap);
		// void hold(bool hold);
		// static void deep_sleep_hold(bool hold);

		// void* get_isr_args();
		~GPIO_driver();

	// protected:
		// int level = 0;
		// gpio_num_t num;
		// void* isr_args = NULL;
		// static unsigned int driver_instaled;
	private:
		uint16_t gpio_pin_mask_;
		int level_ = 0;
		int direction_ = 0;

		GPIO_TypeDef *port_;
};

// typedef struct {
// 	GPIO_driver* esse;
// 	void* args;
// }gpio_isr_args_t;

#endif /* GPIO_HPP__ */



















// OLD things bellow

// class GPIO_dri {
// public:

// 	void gateConfig(uint8_t pin, uint8_t dir);
// 	void gateSet(uint8_t pin, uint8_t status);
// 	void gateToggle(uint8_t pin);
// 	uint8_t gateRead(uint8_t pin, uint8_t reg);
// };

// void GPIO::gateConfig(uint8_t pin, uint8_t dir)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;

// 	if(dir)
// 	{
// 		// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// Push pull
// //		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	// open drain
// 		// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

// 		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
// 		GPIO_InitStruct.Pull = GPIO_NOPULL;
// 		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //GPIO_SPEED_FREQ_LOW;
// 	}
// 	else
// 	{
// 		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
// 		GPIO_InitStruct.Pull = GPIO_PULLUP;
// 		// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	// Pull-Up mode
// //		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	// Pull-Down mode
// //		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

// 	}

// 	switch (pin)
// 	{
// 		case 2:
// 			GPIO_InitStruct.Pin = GPIO_PIN_0;		
// 			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		
// 			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
// 			GPIO_Init(GPIOC, &GPIO_InitStructure);
// 			break;

// 		case 3:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
// 			GPIO_Init(GPIOC, &GPIO_InitStructure);
// 			break;

// 		case 4:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
// 			GPIO_Init(GPIOC, &GPIO_InitStructure);
// 			break;

// 		case 5:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 6:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 7:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 8:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 9:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 10:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 11:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 12:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 13:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 14:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 15:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 16:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 21:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 22:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 23:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 24:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 25:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 26:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 27:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 28:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 29:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 30:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
// 			GPIO_Init(GPIOA, &GPIO_InitStructure);
// 			break;

// 		case 31:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 32:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 33:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 34:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 35:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 36:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;

// 		case 37:
// 			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
// 			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
// 			GPIO_Init(GPIOB, &GPIO_InitStructure);
// 			break;
// 	}

// 	HAL_GPIO_Init(port_, &GPIO_InitStruct);
// }
// void GPIO::write(uint8_t status)
// {
// 	HAL_GPIO_WritePin(GPIOB, gpio_, GPIO_PIN_RESET);

// 	switch (pin)
// 	{
// 		case 2:
// 			if(status)
// 			{
// //				GPIOC -> BSRR = (1<<(13+16)); // 16 bit shift
// 				GPIOC -> BSRR = (1<<13);
// //				GPIO_SetBits(GPIOC, 13);
// 			}
// 			else
// 			{
// 				GPIOC -> BRR  = (1<<13);
// 			}
// 			break;

// 		case 3:
// 			if(status)
// 			{
// 				GPIOC -> BSRR = (1<<14);
// 			}
// 			else
// 			{
// 				GPIOC -> BRR  = (1<<14);
// 			}
// 			break;

// 		case 4:
// 			if(status)
// 			{
// 				GPIOC -> BSRR = (1<<15);
// 			}
// 			else
// 			{
// 				GPIOC -> BRR  = (1<<15);
// 			}
// 			break;

// 		case 5:
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<0);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<0);
// 			}
// 			break;

// 		case 6:
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<1);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<1);
// 			}
// 			break;

// 		case 7:
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<2);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<2);
// 			}
// 			break;

// 		case 8:
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<3);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<3);
// 			}
// 			break;

// 		case 9:
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<4);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<4);
// 			}
// 			break;

// 		case 10:		// PA5 (SPI1_SCK)
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<5);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<5);
// 			}
// 			break;

// 		case 11:	// PA6
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<6);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<6);
// 			}
// 			break;

// 		case 12:	// PA7
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<7);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<7);
// 			}
// 			break;

// 		case 13:	// PB0
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<0);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<0);
// 			}
// 			break;

// 		case 14:	// PB1
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<1);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<1);
// 			}
// 			break;

// 		case 15:	// PB10
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<10);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<10);
// 			}
// 			break;

// 		case 16:	// PB11
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<11);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<11);
// 			}
// 			break;

// 		case 21:	// PB12
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<12);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<12);
// 			}
// 			break;

// 		case 22:	// PB13
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<13);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<13);
// 			}
// 			break;

// 		case 23:	// PB14
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<14);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<14);
// 			}
// 			break;

// 		case 24:	// PB15
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<15);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<15);
// 			}
// 			break;

// 		case 25:	// PA8
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<8);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<8);
// 			}
// 			break;

// 		case 26:	// PA9
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<9);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<9);
// 			}
// 			break;

// 		case 27:	// PA10
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<10);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<10);
// 			}
// 			break;

// 		case 28:	// PA11 (USB-)
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<11);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<11);
// 			}
// 			break;

// 		case 29:	// PA12 (USB+)
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<12);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<12);
// 			}
// 			break;

// 		case 30:	// PA15
// 			if(status)
// 			{
// 				GPIOA -> BSRR = (1<<15);
// 			}
// 			else
// 			{
// 				GPIOA -> BRR  = (1<<15);
// 			}
// 			break;

// 		case 31:	// PB3
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<3);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<3);
// 			}
// 			break;

// 		case 32:	// PB4
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<4);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<4);
// 			}
// 			break;

// 		case 33:	// PB5
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<5);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<5);
// 			}
// 			break;

// 		case 34:	// PB6
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<6);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<6);
// 			}
// 			break;

// 		case 35:	// PB7
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<7);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<7);
// 			}
// 			break;

// 		case 36:	// PB8
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<8);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<8);
// 			}
// 			break;

// 		case 37:	// PB9
// 			if(status)
// 			{
// 				GPIOB -> BSRR = (1<<9);
// 			}
// 			else
// 			{
// 				GPIOB -> BRR  = (1<<9);
// 			}
// 			break;
// 	}
// }
// void GPIO::gateToggle(uint8_t pin)
// {
// 	switch (pin)
// 	{
// 		case 2:
// 			GPIOC -> ODR ^= (1<<13);
// 			break;

// 		case 3:
// 			GPIOC -> ODR ^= (1<<14);
// 			break;

// 		case 4:
// 			GPIOC -> ODR ^= (1<<15);
// 			break;

// 		case 5:
// 			GPIOA -> ODR ^= (1<<0);
// 			break;

// 		case 6:
// 			GPIOA -> ODR ^= (1<<1);
// 			break;

// 		case 7:
// 			GPIOA -> ODR ^= (1<<2);
// 			break;

// 		case 8:
// 			GPIOA -> ODR ^= (1<<3);
// 			break;

// 		case 9:
// 			GPIOA -> ODR ^= (1<<4);
// 			break;

// 		case 10:
// 			GPIOA -> ODR ^= (1<<5);
// 			break;

// 		case 11:
// 			GPIOA -> ODR ^= (1<<6);
// 			break;

// 		case 12:
// 			GPIOA -> ODR ^= (1<<7);
// 			break;

// 		case 13:
// 			GPIOB -> ODR ^= (1<<0);
// 			break;

// 		case 14:
// 			GPIOB -> ODR ^= (1<<1);
// 			break;

// 		case 15:
// 			GPIOB -> ODR ^= (1<<10);
// 			break;

// 		case 16:
// 			GPIOB -> ODR ^= (1<<11);
// 			break;

// 		case 21:
// 			GPIOB -> ODR ^= (1<<12);
// 			break;

// 		case 22:
// 			GPIOB -> ODR ^= (1<<13);
// 			break;

// 		case 23:
// 			GPIOB -> ODR ^= (1<<14);
// 			break;

// 		case 24:
// 			GPIOB -> ODR ^= (1<<15);
// 			break;

// 		case 25:
// 			GPIOA -> ODR ^= (1<<8);
// 			break;

// 		case 26:
// 			GPIOA -> ODR ^= (1<<9);
// 			break;

// 		case 27:
// 			GPIOA -> ODR ^= (1<<10);
// 			break;

// 		case 28:
// 			GPIOA -> ODR ^= (1<<11);
// 			break;

// 		case 29:
// 			GPIOA -> ODR ^= (1<<12);
// 			break;

// 		case 30:
// 			GPIOA -> ODR ^= (1<<15);
// 			break;

// 		case 31:
// 			GPIOB -> ODR ^= (1<<3);
// 			break;

// 		case 32:
// 			GPIOB -> ODR ^= (1<<4);
// 			break;

// 		case 33:
// 			GPIOB -> ODR ^= (1<<5);
// 			break;

// 		case 34:
// 			GPIOB -> ODR ^= (1<<6);
// 			break;

// 		case 35:
// 			GPIOB -> ODR ^= (1<<7);
// 			break;

// 		case 36:
// 			GPIOB -> ODR ^= (1<<8);
// 			break;

// 		case 37:
// 			GPIOB -> ODR ^= (1<<9);
// 			break;
// 	}

// }
// uint8_t GPIO::gateRead(uint8_t pin, uint8_t reg)	// reg: read register input IDR (0) or output ODR (1)
// {
// 	uint8_t status = 0;

// 	switch (pin)
// 	{
// 		case 2:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);

// //			status =  (uint8_t) (((GPIOC -> ODR) & (1 << 13)) >> 13);
// 			break;

// 		case 3:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_14);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
// 			break;

// 		case 4:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);
// 			break;

// 		case 5:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
// 			break;

// 		case 6:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
// 			break;

// 		case 7:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
// 			break;

// 		case 8:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
// 			break;

// 		case 9:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
// 			break;

// 		case 10:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
// 			break;

// 		case 11:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
// 			break;

// 		case 12:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
// 			break;

// 		case 13:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
// 			break;

// 		case 14:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
// 			break;

// 		case 15:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_10);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10);
// 			break;

// 		case 16:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
// 			break;

// 		case 21:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
// 			break;

// 		case 22:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_13);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
// 			break;

// 		case 23:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_14);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
// 			break;

// 		case 24:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);
// 			break;

// 		case 25:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);
// 			break;

// 		case 26:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_9);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);
// 			break;

// 		case 27:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_10);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);
// 			break;

// 		case 28:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_11);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
// 			break;

// 		case 29:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
// 			break;

// 		case 30:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);
// 			break;

// 		case 31:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
// 			break;

// 		case 32:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
// 			break;

// 		case 33:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
// 			break;

// 		case 34:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_6);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
// 			break;

// 		case 35:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
// 			break;

// 		case 36:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_8);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);
// 			break;

// 		case 37:
// 			if(reg)
// 				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9);
// 			else
// 				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);
// 			break;
// 	}

// 	return status;
// }

// #endif /* __GPIO_HPP__ */
