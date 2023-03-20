/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ADC_H__
#define ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "system_main.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void adc_read_SR_reg(void);
void adc_print_SR_reg(void);
void adc_read_CR1_reg(void);
void adc_print_CR1_reg(void);
void adc_read_CR2_reg(void);
void adc_print_CR2_reg(void);
uint8_t adc_read_SR_EOC_bit(void);
void adc_set_CR2_EXTSEL_bits(uint8_t value);
void adc_set_CR2_DMA_bit(void);
void adc_dma_init(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H__ */

