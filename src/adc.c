/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint32_t ADC_SR_reg = 0;
uint8_t STRT_bit = 0;		// bit 4
uint8_t JSTRT_bit = 0;		// bit 3
uint8_t JEOC_bit = 0;		// bit 2
uint8_t EOC_bit = 0;		// bit 1
uint8_t AWD_bit = 0;		// bit 0

// Each SQR slot has 5 bits with 32 channels possibilities.
uint32_t ADC_SQR1_reg = 0;	// ADC regular sequence register 1 (ADC_SQR1)
uint8_t L_bits = 0;			// bit[23:20]. Regular channel sequence length. 0000 is one conversion.
uint32_t ADC_SQR2_reg = 0;	// ADC regular sequence register 2 (ADC_SQR2)
uint32_t ADC_SQR3_reg = 0;	// ADC regular sequence register 3 (ADC_SQR3)

// It has 3 bits into each slot
uint32_t ADC_SMPR1_reg = 0;	// ADC Sample Time Register 1
uint32_t ADC_SMPR2_reg = 0;	// ADC Sample Time Register 2

uint32_t ADC_CR1_reg = 0;
uint8_t AWDE_bit = 0;		// bit 23 - Analog watchdog enable on regular channels
uint8_t JAWDE_bit = 0;		// bit 22 - Analog watchdog enable on injected channels
uint8_t DUALMOD_bits = 0;	// bit[19-16] - Dual mode selection
uint8_t DISCNUM_bits = 0;	// bit[15-13] - Discontinuous mode channel count
uint8_t DISCEN_bit = 0;		// bit[11] - Discontinuous mode on regular channels
uint8_t SCAN_bit = 0;		// bit 8 - Scan mode
uint8_t EOCIE_bit = 0;		// bit 5 - Interrupt enable for EOC

uint32_t ADC_CR2_reg = 0;
uint8_t TSVREFE_bit = 0;	// bit 23 - Temp sensor and Vref enable
uint8_t SWSTART_bit = 0;	// bit 22 - Start conversion of regular channels
uint8_t EXTTRIG_bit = 0;	// bit 20 - External trigger conversion mode for regula channel
uint8_t EXTSEL_bits = 0;	// bits[19:17] - External event select for regular group
uint8_t ALIGN_bit = 0;		// bit 11 - Data alignment
uint8_t DMA_bit = 0;		// bit 8 - Direct Memory acess mode
uint8_t RSTCAL_bit = 0;		// bit 3 - Reset calibration
uint8_t CAL_bit = 0;		// bit 2 - A/D auto calibration
uint8_t CONT_bit = 0;		// bit 1 - Continuous conversion
uint8_t ADON_bit = 0;		// bit 0 - A/D converter ON/OFF

uint32_t DMA1_ISR_reg = 0;
uint8_t TEIF1_bit = 0;
uint8_t HTIF1_bit = 0;
uint8_t TCIF1_bit = 0;
uint8_t GIF1_bit = 0;

uint16_t DMA1_CNDTR_reg = 0;
uint32_t DMA1_CPAR_reg = 0;
uint32_t DMA1_CMAR_reg = 0;

uint16_t adc_buffer[ADC_BUFLEN];   //For ADC samples.
uint16_t RxData[3];
float Temperature;
uint8_t adc_dma_flag = 0;		// dma1 IRQ flag
uint8_t adc_dma_te_flag = 0;	// transfer error
uint8_t adc_dma_ht_flag = 0;	// half transfer
uint8_t adc_dma_tc_flag = 0;	// transfer complete
uint8_t adc_dma_gi_flag = 0;	// global interruption

void adc_read_SR_reg(void) {
	// read SR reg
	ADC_SR_reg = ADC1->SR;

	STRT_bit = (ADC_SR_reg >> 4) & 1;
	JSTRT_bit = (ADC_SR_reg >> 3) & 1;
	JEOC_bit = (ADC_SR_reg >> 2) & 1;
	EOC_bit = (ADC_SR_reg >> 1) & 1;
	AWD_bit = (ADC_SR_reg >> 0) & 1;
}
void adc_print_SR_reg(void) {
	printf("ADC_SR1- 0x%08x STRT:%u JSTRT:%u JEOC:%u EOC:%u AWD:%u\n",
		ADC_SR_reg, STRT_bit, JSTRT_bit, JEOC_bit, EOC_bit, AWD_bit);
}
void adc_read_CR1_reg(void) {
	// read CR1 reg bits
	ADC_CR1_reg = ADC1->CR1;

	AWDE_bit = (ADC_CR1_reg >> 23) & 1;
	JAWDE_bit = (ADC_CR1_reg >> 22) & 1;
	DUALMOD_bits = (ADC_CR1_reg >> 16) & 0x0F;
	DISCNUM_bits = (ADC_CR1_reg >> 13) & 0x07;
	DISCEN_bit = (ADC_CR1_reg >> 11) & 1;
	SCAN_bit = 	(ADC_CR1_reg >> 8) & 1;
	EOCIE_bit = (ADC_CR1_reg >> 5) & 1;
}
void adc_print_CR1_reg(void) {
	printf("ADC_CR1-0x%08x AWDE:%u JAWDE:%u DUALMOD:0x%02x DISCNUM:0x%02x DISCEN:%u SCAN:%u EOCIE:%u\n",
		ADC_CR1_reg, AWDE_bit, JAWDE_bit, DUALMOD_bits, DISCNUM_bits, DISCEN_bit, SCAN_bit, EOCIE_bit);
}
void adc_read_CR2_reg(void) {
	// read CR2 reg bits
	ADC_CR2_reg = ADC1->CR2;

	TSVREFE_bit = (ADC_CR2_reg >> 23) & 1; // ADC_CR2_TSVREFE_Pos
	SWSTART_bit = (ADC_CR2_reg >> 22) & 1;
	EXTTRIG_bit = (ADC_CR2_reg >> 20) & 1;
	EXTSEL_bits = (ADC_CR2_reg >> 17) & 0x07;
	ALIGN_bit = (ADC_CR2_reg >> 11) & 1;
	DMA_bit = (ADC_CR2_reg >> 8) & 1;
	RSTCAL_bit = (ADC_CR2_reg >> 3) & 1;
	CAL_bit = (ADC_CR2_reg >> 2) & 1;
	CONT_bit = (ADC_CR2_reg >> 1) & 1;
	ADON_bit = (ADC_CR2_reg >> 0) & 1;
}
void adc_print_CR2_reg(void) {
	printf("ADC_CR2-0x%08x TSVREFE:%u SWSTART:%u EXTTRIG:%u EXTSEL:0x%02x ALIGN:%u DMA:%u RSTCAL:%u CAL:%u CONT:%u ADON:%u\n",
		ADC_CR1_reg, TSVREFE_bit, SWSTART_bit, EXTTRIG_bit, EXTSEL_bits, ALIGN_bit, DMA_bit, RSTCAL_bit, CAL_bit, CONT_bit, ADON_bit);
}
void adc_read_SQR1_reg(void) {
	ADC_SQR1_reg = ADC1->SQR1;
	// SQ16_bits = (ADC_SQR1_reg >> 15) & 0x1F;
	// SQ15_bits = (ADC_SQR1_reg >> 10) & 0x1F;
	// SQ14_bits = (ADC_SQR1_reg >> 5) & 0x1F;
	// SQ13_bits = (ADC_SQR1_reg >> 0) & 0x1F;
}
void adc_read_SQR2_reg(void) {
	ADC_SQR2_reg = ADC1->SQR2;
}
void adc_read_SQR3_reg(void) {
	// Regular sequence register.
	ADC_SQR3_reg = ADC1->SQR3;
}

void adc_print_rank(int n) {

	int ch = 0;
	int slot = (n-1)*5;
	if(n<7) {
		adc_read_SQR3_reg();
		ch = (ADC_SQR3_reg >> slot) & 0x1F;
	} else if(n>=7 && n<=12) {
		adc_read_SQR2_reg();
		ch = (ADC_SQR2_reg >> slot) & 0x1F;
	} else if (n>=13 && n<=16) {
		adc_read_SQR1_reg();
		ch = (ADC_SQR1_reg >> slot) & 0x1F;
	} else {
		printf("out rank%d has ch:%d\n", n, ch);
		return;
	}

	printf("rank%d has ch:%d\n", n, ch);
}
void adc_print_DISCNUM(void) {
	adc_read_CR1_reg();
	printf("disc num channels: %d\n", DISCNUM_bits);
}
void adc_print_L_regular(void) {
	// this prints the regular channel sequence length
	adc_read_SQR1_reg();
	L_bits = (ADC_SQR1_reg >> 20) & 0x0F;
	printf("reg ch Length:%d\n", (int)L_bits);
}
void adc_set_CR1_discnum(uint8_t value) {
	ADC1->CR1 &= ~(0x07 << 13);
	ADC1->CR1 |= (value << 13);
	// DISCNUM_bits = (ADC_CR1_reg >> 13) & 0x07;
}

void dma1_read_ISR_reg(void) {
	// Read ISR reg from DMA1 peripheral
	DMA1_ISR_reg = DMA1->ISR;
	TEIF1_bit = (DMA1_ISR_reg >> 3) & 1;
	HTIF1_bit = (DMA1_ISR_reg >> 2) & 1;
	TCIF1_bit = (DMA1_ISR_reg >> 1) & 1;
	GIF1_bit = (DMA1_ISR_reg >> 0) & 1;
}
void dma1_print_ISR_reg(void) {
	printf("DMA1_ISR- TEIF1:%u HTIF1:%u TCIF1:%u GIF1:%u\n", 
					TEIF1_bit, HTIF1_bit, TCIF1_bit, GIF1_bit);
}
void dma1_read_CNDTR_reg(void) {
	// Read memory size
	DMA1_CNDTR_reg = (uint16_t) DMA1_Channel1->CNDTR;
}
void dma1_print_CNDTR_reg(void) {
	printf("DMA1_CNDTR: %u\n", DMA1_CNDTR_reg);
}
void dma1_read_CPAR_reg(void) {
	DMA1_CPAR_reg = DMA1_Channel1->CPAR;

	// Set peripheral source address
	// DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
}
void dma1_print_CPAR_reg(void) {
	printf("CPAR peripheral addr: 0x%08x\n", DMA1_CPAR_reg);
}
void dma1_read_CMAR_reg(void) {
	DMA1_CMAR_reg = DMA1_Channel1->CMAR;
}
void dma1_print_CMAR_reg(void) {
	printf("CMAR memory addr: 0x%08x\n", DMA1_CMAR_reg);
}

void adc_set_CR2_EXTSEL_bits(uint8_t value) {
	ADC1->CR2 &= ~(0x07 << 17);
	ADC1->CR2 |= (value << 17);
}
void adc_set_CR2_DMA_bit(void) {
	ADC1->CR2 &= ~(0x01 << 8);
	ADC1->CR2 |= (1 << 8);
}
uint8_t adc_read_SR_EOC_bit(void) {
	return EOC_bit;
}

void adc_init(void) {
	/* Single conversion mode. Page 219 of RM0008 for STM32F103

	Steps to follow

	1. Enable ADC and GPIO clock
	2. Set the prescalar in the Clock configuration register (RCC_CFGR)
	3. Disable scan mode in control Register 1 (CR1)
	4. Enable/Disable Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	5. Set the Sampling Time for the channels in ADC_SMPRx
	6. Set the Regular channel sequence length in ADC_SQR1
	7. Set the Respective GPIO PINs in the Analog Mode
	************************************************/

	/*
		Single conversion mode
		CONT bit should be 0
		If a regular channel was converted:
			–The converted data is stored in the 16-bit ADC_DR register
			–The EOC (End Of Conversion) flag is set
			–and an interrupt is generated if the EOCIE is set.
	*/
	// 1- Enable ADC and GPIO clock

	// ADC1 clock enable (ADC1 EN: bit 9)
	RCC->APB2ENR |= (1<<9);	//RCC_APB2ENR_ADC1EN;

	// 2- Set the prescalar in the Clock configuration register (RCC_CFGR)

	// ADC1 prescale to maximum div by 8. Value: 11
	RCC->CFGR |= (3 << 14);	// 48 MHz/8 = 6 MHz


	// --- Control Register 1 (CR1) configuration ---

	// Disable Dual mode. Enable independent mode: 0000;
	ADC1->CR1 &= ~(0x0F << 16);

	// Discontinous number DISCNUM[2:0] at bit 13: 000: 1 channel only
	ADC1->CR1 &= ~(7<<13);
	// ADC1->CR1 |= (1<<13);

	// Discontinous mode on regular channels enable at bit 11. (Enable = 1/ Disable = 0)
	ADC1->CR1 &= ~(1<<11);	// disable discontinous mode;

	// 3- Disable the Scan Mode in the Control Register 1 (CR1)
	ADC1->CR1 &= ~(1<<8);

	// Disable End Of Conversion Interrupt bit (EOCIE)
	ADC1->CR1 &= ~(1<<5);

	// Disable AD module
	ADC1->CR2 &= ~(1<<0);

	// Enable External trigger interrupt
	ADC1->CR2 |= (1<<20);

	// Enable Temperature and Vrefint with TSVREFE bit
	// ADC1->CR2 |= (1<<23);

	// External trigger conversion mode for regular channels using:
		// 011: Timer 2 CC2 event
		// 100: Timer 3 TRGO event (SELECT)
	// ADC1->CR2 &= ~(7<<17);
	// ADC1->CR2 |= (4<<17);
	
	// External trigger with Software control bit.
		// 111: SWSTART
	ADC1->CR2 |= (7<<17);

	// Data alignment to rigth
	ADC1->CR2 &= ~(1<<11);
		
	// Enable DMA for ADC (DMA)
	ADC1->CR2 |= (1<<8);	//(1 << ADC_CR2_DMA_Pos);
	// LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	// 4- CR2: Enable/Disable Continuous Conversion, EOC and Alignment in Control Register 2 (CR2)
	// Continuous conversion mode (Enable = 1\ Disable = 0)
	// ADC1->CR2 &= ~(1<<1);	// Disable
	ADC1->CR2 |=  (1<<1);	// Enable

	// 5. Set the Sampling Time for the channels in ADC_SMPRx

	// Set 239.5 cycles sampling time to rank 1 - set 111 to SMP0[2:0] at bit 0 (Channel AN2).
	// AN8 - PB0 bit 6
	// AN4 - PA4 bit 12
	// ADC1->SMPR2 |= (7<<0);

	// Set regular channel sequence length (number of channels)
	// ADC1->SQR1 &= ~(0x0F<<20);	// 0000: length is 1 conversion

	// Set channel 3 to rank 1
	// ADC1->SQR3 |= (3<<0);

	// Set PB0 to analog input;
	// GPIOA->CRLMODER |= (3 << 4);

	// Enable ADC1
	// ADC1->CR2 |= (1 << 0);

	// Wait for ADC to stabilize (approx 10us)
	// HAL_Delay(1);

	// Ref.: https://controllerstech.com/dma-with-adc-using-registers-in-stm32/
}
void adc_set_num_reg_channels(int length) {
	// Number of regular channels to be readed. It must use SCAN mode?

	// Set regular channel sequence length (number of channels)
	ADC1->SQR1 &= ~(0x0F<<20);	// 0000: length is 1 conversion (clear all bits)

	if((length > 1) && (length < 33)) {
		uint8_t num = length - 1;
		num &= 0x0F;
		ADC1->SQR1 |= (num<<20);
	}
}
void adc_set_channel_to_rank(int channel, int rank) {
	// Up to 16 ADC channels to read;
	// 5. Set channel on specific rank. Set channel x to rank n
	uint8_t rank_slot = 0;
	uint8_t ch = (uint8_t) channel;

	// Find rank slot first bit to configure
	uint8_t rank_first_bit_config;

	if((rank>=1) && (rank<=6)) {
		rank_slot = (uint8_t) (rank - 1);
		rank_first_bit_config = rank_slot*5;
		ADC1->SQR3 |= (ch << rank_first_bit_config);
	}
	else if((rank>=7)&&(rank<=12)) {
		rank_slot = (uint8_t) (rank - 7);
		rank_first_bit_config = rank_slot*5;
		ADC1->SQR2 |= (ch << rank_first_bit_config);
	}
	else if((rank>=13)&&(rank<=16)) {
		rank_slot = (uint8_t) (rank - 13);
		rank_first_bit_config = rank_slot*5;
		ADC1->SQR1 |= (ch << rank_first_bit_config);
	}

	// Set PB0 to analog input;
	// GPIOA->CRLMODER |= (3 << 4);
}
void adc_set_rank_sampling_time(int rank, int st) {
	// 6. Set the Sampling Time for the channels in ADC_SMPRx
	// Set 239.5 cycles sampling time to rank 1 - set 111 to SMP0[2:0] at bit 0 (Channel AN2).
	// AN8 - PB0 bit 6
	// AN4 - PA4 bit 12

	uint8_t _st = (uint8_t) st;
	uint8_t smpr_slot = 0;

	if((rank>=1) && (rank<=10)) {
		smpr_slot = ((uint8_t) rank - 1)*3;
		ADC1->SMPR2 |= (st<<smpr_slot);
	}
	else if((rank>10) && (rank<=17)) {
		smpr_slot = ((uint8_t) rank - 10)*3;
		ADC1->SMPR1 |= (st<<smpr_slot);
	}
}
void adc_module_enable(void) {
	// Enable ADC1
	ADC1->CR2 |= (1 << 0);

	// Wait for ADC to stabilize (approx 10us)
	HAL_Delay(1);

	// Ref.: https://controllerstech.com/dma-with-adc-using-registers-in-stm32/
}
void adc_module_disable(void) {
	// Disable AD module
	ADC1->CR2 &= ~(1<<0);
}
void adc_start_conversion(void) {
	// Only if SWSTART bit in CR2 is linked toward EXTTRIG 111

	// Clear Status register
	ADC1->SR = 0;

	// Conversion on external event enabled EXTTRIG
	// ADC1->CR2 |= (1<<20);

	// Software start conversion (SWSTART)
	// DMA1_Channel1->CCR &= ~(1<<0);
	// DMA1_Channel1->CNDTR = ADC_BUFLEN;
	// DMA1_Channel1->CCR |= (1<<0);

	ADC1->CR2 |= (1<<22);
}
void adc_stop_conversion(void) {

	// Disable ADC1
	ADC1->CR2 &= ~(1<<0);

	// Continuous conversion mode (Enable = 1\ Disable = 0)
	ADC1->CR2 &= ~(1<<1);
	
	// ADC1->CR2 &= ~(1<<22);

}
void adc_prescale(uint8_t div) {

	// Clear ADCPRE[1:0] 15 and 15 bits;
	RCC->CFGR &= ~(3 << 14);
	
	switch (div) {
		case 2: {
			// 00: PCLK2 divided by 2
			RCC->CFGR &= ~(3 << 14);
			break;
		}
		case 4: {
			// 01: PCLK2 divided by 4
			RCC->CFGR |= (1 << 14);
			break;
		}
		case 6: {
			// 10: PCLK2 divided by 6
			RCC->CFGR |= (2 << 14);
			break;
		}
		case 8: {
			// 11: PCLK2 divided by 8
			RCC->CFGR |= (3 << 14);
			break;
		}
		default: {
			// 00: PCLK2 divided by 2
			RCC->CFGR &= ~(3 << 14);
			break;
		}
	}
}
void adc_gpioa_config(void) {
	// Configure IN2 in GPIOA. 
	
	// Enable GPIOA clock
	RCC->APB2ENR |= (1<<2);
	
	// Mode to input analog mode to MODE2[1:0] = 00 at bit 8. (Page 171 - rev 21.)
	// AN2 at bit 8
	// AN3 at bit 12
	// AN4 at bit 16
	GPIOA->CRL &= ~(3<<16);

	// Config to Analog Input. CNF2[1:0] = 00 at bit 18.
	// AN4 at bit 18
	GPIOA->CRL &= ~(3<<18);	
}

uint16_t adc_read_oneshot(void) {

	int i = 0;
	do {
		adc_read_SR_reg();
		i++;
	} while(!EOC_bit && (i<100000));

	return (uint16_t)ADC1->DR;
}

void adc_dma_begin(uint16_t* dest_addr, uint16_t size) {
	// adc_gpioa_config();						// Configure AN2 to read analog values;
	adc_prescale(8);						// Configure RCC ADC prescale;
	adc_init();								// Configure ADC control registers;
	adc_dma_init();							// Configure DMA, link with ADC peripheral and enable;
	adc_dma_config_addr(dest_addr, size);	// Configure DMA array address to write ADC values;
	adc_dma_config_it();					// Configure DMA half and complete transfer interruption;
	}
void adc_dma_init(void) {
	/* Initialize the DMA
	Steps to follow:
		1- Enable DMA clock;
		2- Set data direction from peripheral to memory
		3- Configure Circular/Normal mode;
		4- Enable/Disable Memory Increment and Peripheral Increment
		5- Set the Data buffer size
		6- Select the channel for the stream
	*/

	// Enable DMA1 clock
	RCC->AHBENR |= (1<<0);

	// Disable DMA channel before edit;
	DMA1_Channel1->CCR &= ~(1<<0);
	
	// Disable Memory to memory mode
	DMA1_Channel1->CCR &= ~(1<<14);

	// Channel priority level to low: 00 (PL[1:0])
	DMA1_Channel1->CCR &= ~(3<<12);

	// Memory size to 16 bits: 01 (MSIZE)
	DMA1_Channel1->CCR &= ~(3<<10);	// Clear MSIZE[1:0] at bit 10
	DMA1_Channel1->CCR |= (1<<10);	// Set 01: 16 bits

	// Peripheral size to 16 bits: 01 (PSIZE)
	DMA1_Channel1->CCR &= ~(3<<8);	// Clear PSIZE[1:0] at bit 8
	DMA1_Channel1->CCR |= (1<<8);	// Set 01: 16 bits

	// Memory increment mode enable (MINC)
	DMA1_Channel1->CCR |= (1<<7);

	// Peripheral increment mode disable (PINC)
	DMA1_Channel1->CCR &= ~(1<<6);

	// Circular mode:1, Normal mode: 0 (CIRC)
	DMA1_Channel1->CCR &= ~(1<<5);	// Normal mode

	// Data transfer direction from peripheral to memory (DIR)
	DMA1_Channel1->CCR &= ~(1<<4);	// 0: read from peripheral

	// Enable interrupts Trasfer error (TEIE), Half transfer (HTIE) and Transfer complete (TCIE)
	DMA1_Channel1->CCR |= (1<<3) | (1<<2) | (1<<1);

	// Enable DMA channel after edit;
	DMA1_Channel1->CCR |= (1<<0);
}
void adc_dma_config_addr(uint16_t* dest_addr, int size) {
	// void DMA_Config(uint32_t srcAdd, uint32_t destAdd, uint16_t size) {
	// 
	/* Configure the DMA pointer to memory
	1- Set the data size in the CNDTR register
	2- Set the peripheral address
	3- Set the memory address
	3- Enable the DMA channel stream
	*/

	// Disable DMA channel before edit;
	DMA1_Channel1->CCR &= ~(1<<0);

	// Set buffer size
	DMA1_Channel1->CNDTR = (uint32_t)(size);

	// Set peripheral source address
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));

	printf("CPAR addr: 0x%08x\n", DMA1_Channel1->CPAR);

	// Set buffer memory address
	DMA1_Channel1->CMAR = (uint32_t) dest_addr;

	// Enable DMA Channel
	DMA1_Channel1->CCR |= (1<<0);
}
void adc_dma_config_it(void) {
	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	// HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
void adc_dma_reset_cnt(uint32_t value) {
	DMA1_Channel1->CCR &= ~(1<<0);
	DMA1_Channel1->CNDTR = value;
	DMA1_Channel1->CCR |= (1<<0);
}

void adc_power_on(void) {
	ADC1->CR2 |= (0<<1);
}
void adc_power_off(void) {
	ADC1->CR2 &= ~(0<<1);
}

void ADC_Init(void) {
	/************** STEPS TO FOLLOW *****************
	1. Enable ADC and GPIO clock
	2. Set the prescalar in the Clock configuration register (RCC_CFGR)
	3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
	4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	5. Set the Sampling Time for the channels in ADC_SMPRx
	6. Set the Regular channel sequence length in ADC_SQR1
	7. Set the Respective GPIO PINs in the Analog Mode
	************************************************/

//1. Enable ADC and GPIO clock
	RCC->APB2ENR |= 1<<9;  // enable ADC1 clock
	RCC->APB2ENR |= (1<<2);  // enable GPIOA clock
	
//2. Set the prescalar in the Clock configuration register (RCC_CFGR)	
	RCC->CFGR |= (2<<14);  // Prescaler 6, ADC Clock = 72/6 = 12 MHz  		 
	
//3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)	
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	// Resolution is 12 bit in F103
	
//4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	ADC1->CR2 &= ~(1<<1);     // enable continuous conversion mode
	// EOC after each conversion by default
	ADC1->CR2 |= (7<<17);  // External Event selection pointed to SWSTART bit
	ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT
	
//5. Set the Sampling Time for the channels	
	ADC1->SMPR2 &= ~((7<<3) | (7<<12));  // Sampling time of 1.5 cycles for channel 1 and channel 4

//6. Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR1 |= (2<<20);   // SQR1_L =2 for 3 conversions
	
//7. Set the Respective GPIO PINs in the Analog Mode	
	GPIOA->CRL &= ~(0xf<<4);  // analog mode for PA 1
	GPIOA->CRL &= ~(0xf<<16); // analog mode for PA 4
	
	
	/**************************************************************************************************/
	
	
	// Sampling Freq for Temp Sensor 
	ADC1->SMPR1 |= (7<<18);  // Sampling time (71.5 cycles) of 7 us for channel 16.. It should be <17.1 us
	
	// Set the TSVREFE Bit to wake the sensor
	ADC1->CR2 |= (1<<23);
	
	// Enable DMA for ADC
	ADC1->CR2 |= (1<<8);
	
//	// Enable Continuous Request
//	ADC1->CR2 |= (1<<9);
	
	// Channel Sequence
	ADC1->SQR3 |= (1<<0);  // SEQ1 for Channel 1
	ADC1->SQR3 |= (4<<5);  // SEQ2 for CHannel 4
	ADC1->SQR3 |= (16<<10);  // SEQ3 for CHannel 16
}
void ADC_Enable2(void) {
	/************** STEPS TO FOLLOW *****************
	1. Enable the ADC by setting ADON bit in CR2
	2. Wait for ADC to stabilize (approx 10us) 
	************************************************/
	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1
	
	uint32_t delay = 10000;
	while (delay--);
}
void ADC_Start(void) {	
	/************** STEPS TO FOLLOW *****************
	1. Clear the Status register
	2. Start the Conversion by Setting the SWSTART bit in CR2
	*************************************************/
	ADC1->SR = 0;                      // Clear Status register
	ADC1->CR2 |= (1<<20);              // Conversion on external event enabled
	ADC1->CR2 |= 1<<22;                // Start conversion
}
void DMA_Init(void) {
	/************** STEPS TO FOLLOW *****************
	1. Enable DMA clock
	2. Set the DATA Direction
	3. Enable/Disable the Circular Mode
	4. Enable/Disable the Memory Increment and Peripheral Increment
	5. Set the Data Size
	6. Select the channel for the Stream
	************************************************/
	
	// 1. Enable DMA1 Clock
	RCC->AHBENR |= 1<<0;
	
	// 2. Set the Data Direction
//	DMA1_Channel7->CCR |= (1<<4);   // Read From Memory
	DMA1_Channel1->CCR &= ~(1<<4);   // Read From Peripheral

	// 2. Enable the circular mode (CIRC)
	DMA1_Channel1->CCR |= 1<<5;
	
	// 3. Enable the Memory Increment (MINC)
	DMA1_Channel1->CCR |= 1<<7;
	
	// 4. Set the Peripheral data size (PSIZE)
	DMA1_Channel1->CCR |= (1<<8);  // 01 : 8 Bit Data
	
	// 5. Set the Memory data size (MSIZE)
	DMA1_Channel1->CCR |= (1<<10);  // 01 : 8 Bit Data
}
void DMA_Config(uint32_t srcAdd, uint32_t destAdd, uint16_t size) {
	
	/************** STEPS TO FOLLOW *****************
	1. Set the Data Size in the CNDTR Register
	2. Set the Peripheral Address and the Memory Address
	3. Enable the DMA Stream
		 
		 Some peripherals don't need a start condition, like UART, So as soon as you enable the DMA, the transfer will begin
		 While Peripherals like ADC needs the Start condition, so Start the ADC later in the program, to enable the transfer
	************************************************/
	
	DMA1_Channel1->CNDTR = size;   // Set the size of the transfer
	
	DMA1_Channel1->CPAR = srcAdd;  // Source address is peripheral address
	
	DMA1_Channel1->CMAR = destAdd;  // Destination Address is memory address
	
	// Enable the DMA Stream
	DMA1_Channel1->CCR |= (1<<0);  // EN =1
}
void adc_test_2(void) {
	SystemInit();
	tim2_config_2();

	ADC_Init ();
	ADC_Enable2();
	DMA_Init ();

	DMA_Config ((uint32_t ) &ADC1->DR, (uint32_t) RxData, 3);

	ADC_Start ();

	while (1)
	{

		Temperature = ((1.43 - ((float)(3.3*RxData[2]/(float)4095))) / 0.0043) + 25;
		printf("Temp: %f\n", Temperature);
		Delay_ms(10);
	}
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(adcHandle->Instance == ADC1)
	{
		printf("ADC DMA register!\n");
		/* USER CODE BEGIN ADC1_MspInit 0 */

		/* USER CODE END ADC1_MspInit 0 */
		/* ADC1 clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();

		// __HAL_RCC_GPIOA_CLK_ENABLE();
		// /**ADC1 GPIO Configuration
		// PA0-WKUP     ------> ADC1_IN0
		// */
		
		// // GPIO_InitStruct.Pin = GPIO_PIN_3;
		// // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		// // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// /* ADC1 DMA Init */
		// hdma_adc1.Instance = DMA1_Channel1;
		// hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		// hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		// hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;

		// hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		// // hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		// hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		// // hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;

		// hdma_adc1.Init.Mode = DMA_NORMAL; //DMA_CIRCULAR;
		// hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
		// if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
		// 	Error_Handler();
		// }

		// __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);
		
		// /* Enable DMA controller clock*/
		// /* DMA controller clock enable */
		// __HAL_RCC_DMA1_CLK_ENABLE();

		// /* DMA interrupt init */
		// /* DMA1_Channel1_IRQn interrupt configuration */
		// HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
		// HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	}
}
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {
	if(adcHandle->Instance==ADC1) {
		/* USER CODE BEGIN ADC1_MspDeInit 0 */

		/* USER CODE END ADC1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_ADC1_CLK_DISABLE();

		/**ADC1 GPIO Configuration
		PA0-WKUP     ------> ADC1_IN0
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

		/* ADC1 DMA DeInit */
		HAL_DMA_DeInit(adcHandle->DMA_Handle);
		/* USER CODE BEGIN ADC1_MspDeInit 1 */
		/* USER CODE END ADC1_MspDeInit 1 */
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

	/* USER CODE END DMA1_Channel1_IRQn 0 */
	// HAL_DMA_IRQHandler(&hdma_adc1);/
	
	// printf("DMA1: IRQ\n");
	dma1_read_ISR_reg();
	adc_dma_flag = 1;

	if((TEIF1_bit)) {
		DMA1->IFCR |= (1<<3);
		// printf("DMA1: TEIF\n");
		adc_dma_te_flag = 1;
	}

	// Half Transfer HTIF
	if(HTIF1_bit) {
		DMA1->IFCR |= (1<<2);
		adc_dma_ht_flag = 1;
		// printf("DMA1: HTIF\n");
	}

	// Transfer Complete TCIF
	if(TCIF1_bit) {
		DMA1->IFCR |= (1<<1);
    	adc_dma_tc_flag = 1;
		// printf("DMA1: TCIF\n");
		
	}

	// Global interrupt (TEIF | HTIF | TCIF)
	if(GIF1_bit) {
		DMA1->IFCR |= (1<<0);
		adc_dma_gi_flag = 1;
		// printf("DMA1: GIF\n");
	}

	// Transfer error TEIF
	// if((DMA1->ISR) & (1<<3)) {
	// 	DMA1->IFCR |= (1<<3);
	// 	printf("DMA1: TEIF\n");
	// }

	// // Half Transfer HTIF
	// if((DMA1->ISR) & (1<<2)) {
	// 	DMA1->IFCR |= (1<<2);
	// 	printf("DMA1: HTIF\n");
	// }

	// // Transfer Complete TCIF
	// if((DMA1->ISR) & (1<<1)) {
	// 	DMA1->IFCR |= (1<<1);
    // 	adc_dma_tc_flag = 1;
	// 	printf("DMA1: TCIF\n");
	// }

	// // Global interrupt (TEIF | HTIF | TCIF)
	// if((DMA1->ISR) & (1<<0)) {
	// 	DMA1->IFCR |= (1<<0);
	// 	printf("DMA1: GIF\n");
	// }

	// adc_dma_flag = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	printf("ADC cplt!\n");
	adc_dma_flag = 1;
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	printf("ADC half!\n");
	adc_dma_flag = 1;
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
	printf("ADC: callback dma error\n");
}