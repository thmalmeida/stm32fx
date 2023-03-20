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
uint8_t DUAMOD_bits = 0;	// bit[19-16] - Dual mode selection
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

}
void adc_print_SR_reg(void) {
	
}
void adc_read_CR1_reg(void) {
	// read CR1 reg bits
	ADC_CR1_reg = ADC1->CR1;

	AWDE_bit = (ADC_CR1_reg >> 23) & 1;
	JAWDE_bit = (ADC_CR1_reg >> 22) & 1;
	DUAMOD_bits = (ADC_CR1_reg >> 16) & 0x0F;
	DISCNUM_bits = (ADC_CR1_reg >> 13) & 0x07;
	DISCEN_bit = (ADC_CR1_reg >> 11) & 1;
	SCAN_bit = 	(ADC_CR1_reg >> 8) & 1;
	EOCIE_bit = (ADC_CR1_reg >> 5) & 1;
}
void adc_print_CR1_reg(void) {
	printf("ADC_CR1- AWDE:%u JAWDE:%u DUAMOD:0x%02x DISCNUM:0x%02x DISCEN:%u SCAN:%u EOCIE:%u\n",
		AWDE_bit, JAWDE_bit, DUAMOD_bits, DISCNUM_bits, DISCEN_bit, SCAN_bit, EOCIE_bit);
}
void adc_read_CR2_reg(void) {
	// read CR2 reg bits
	ADC_CR2_reg = ADC1->CR2;

	TSVREFE_bit = (ADC_CR2_reg >> 23) & 1; // ADC_CR2_TSVREFE_Pos
	SWSTART_bit = (ADC_CR2_reg >> 22) & 1;
	EXTTRIG_bit = (ADC_CR2_reg >> 20) & 1;
	EXTSEL_bits = (ADC_CR2_reg >> 17) & 0x03;
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


void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(adcHandle->Instance == ADC1)
	{
		/* USER CODE BEGIN ADC1_MspInit 0 */

		/* USER CODE END ADC1_MspInit 0 */
		/* ADC1 clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC1 GPIO Configuration
		PA0-WKUP     ------> ADC1_IN0
		*/
		
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* ADC1 DMA Init */
		hdma_adc1.Instance = DMA1_Channel1;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;

		// hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		// hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;

		hdma_adc1.Init.Mode = DMA_CIRCULAR; //DMA_NORMAL;
		hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
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
