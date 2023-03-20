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
	printf("ADC_SR1- STRT:%u JSTRT:%u JEOC:%u EOC:%u AWD:%u\n",
		STRT_bit, JSTRT_bit, JEOC_bit, EOC_bit, AWD_bit);
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
	printf("ADC_CR1- AWDE:%u JAWDE:%u DUALMOD:0x%02x DISCNUM:0x%02x DISCEN:%u SCAN:%u EOCIE:%u\n",
		AWDE_bit, JAWDE_bit, DUALMOD_bits, DISCNUM_bits, DISCEN_bit, SCAN_bit, EOCIE_bit);
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
		printf("ADC_CR2- TSVREFE:%u SWSTART:%u EXTTRIG:%u EXTSEL:0x%02x ALIGN:%u DMA:%u RSTCAL:%u CAL:%u CONT:%u ADON:%u\n",
			TSVREFE_bit, SWSTART_bit, EXTTRIG_bit, EXTSEL_bits, ALIGN_bit, DMA_bit, RSTCAL_bit, CAL_bit, CONT_bit, ADON_bit);
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
void adc_dma_init(void) {
	// Dual mode: independent mode;
	ADC1->CR1 &= ~(0x0F << 16);

	// External trigger conversion mode for regular channels
	ADC1->CR2 |= (1 << 20);

	// Enable Continuous conversion mode
	ADC1->CR2 |= (1 << 1);

	// Data rigth data align
	ADC1->CR2 &= ~(1 << 11); 

	// Enable DMA for ADC
	ADC1->CR2 |= (1 << 8);

	// Set 1.5 cycles sampling time to IN2 - set 000 to SMP2[2:0]
	ADC1->SMPR2 &= ~(7 << 6);

	// Set regular channel sequence length
	ADC1->SQR1 |= (1 << 20);

	// Set to the first rank the channel 2
	ADC1->SQR3 |= (2 << 0);

	// Set PA2 to analog input;
	// GPIOA->CRLMODER |= (3 << 4);

	// Enable ADC1
	ADC1->CR2 |= (1 << 0);

	// Ref.: https://controllerstech.com/dma-with-adc-using-registers-in-stm32/
	// Initialize the DMA


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

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC1 GPIO Configuration
		PA0-WKUP     ------> ADC1_IN0
		*/
		
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* ADC1 DMA Init */
		hdma_adc1.Instance = DMA1_Channel1;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;

		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		// hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		// hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;

		hdma_adc1.Init.Mode = DMA_NORMAL; //DMA_CIRCULAR;
		hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;
		if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);
		
		/* Enable DMA controller clock*/
		/* DMA controller clock enable */
		__HAL_RCC_DMA1_CLK_ENABLE();

		/* DMA interrupt init */
		/* DMA1_Channel1_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
