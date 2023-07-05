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

#define ADC_BUFLEN 200

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern uint16_t adc_buffer[ADC_BUFLEN];
extern uint16_t RxData[3];
extern float Temperature;
extern uint8_t adc_dma_flag;
extern uint8_t adc_dma_te_flag;
extern uint8_t adc_dma_ht_flag;
extern uint8_t adc_dma_tc_flag;
extern uint8_t adc_dma_gi_flag;

void adc_read_SR_reg(void);
void adc_print_SR_reg(void);
void adc_read_CR1_reg(void);
void adc_print_CR1_reg(void);
void adc_read_CR2_reg(void);
void adc_print_CR2_reg(void);
void adc_read_SQR1_reg(void);
void adc_print_L_regular(void);
void dma1_read_ISR_reg(void);
void dma1_print_ISR_reg(void);
void dma1_print_ISR_reg(void);
void dma1_read_CNDTR_reg(void);
void dma1_print_CNDTR_reg(void);
void dma1_read_CPAR_reg(void);
void dma1_print_CPAR_reg(void);
void dma1_read_CMAR_reg(void);
void dma1_print_CMAR_reg(void);

void adc_print_rank(int n);
void adc_print_DISCNUM(void);
void adc_set_CR1_discnum(uint8_t value);

void adc_set_CR2_EXTSEL_bits(uint8_t value);
void adc_set_CR2_DMA_bit(void);
uint8_t adc_read_SR_EOC_bit(void);
void adc_init(void);
void adc_set_num_reg_channels(int length);
void adc_set_channel_to_rank(int channel, int rank);
void adc_set_rank_sampling_time(int rank, int st);
void adc_module_enable(void);
void adc_module_disable(void);
void adc_start_conversion(void);
void adc_stop_conversion(void);

void adc_prescale(uint8_t div);

void adc_gpioa_config(void);

uint16_t adc_read_oneshot(void);

void adc_dma_init(void);
void adc_dma_config_addr(uint16_t* dest_addr, int size);
void adc_dma_config_it(void);
void adc_dma_reset_cnt(uint32_t value);

void adc_power_on(void);
void adc_power_off(void);

void adc_dma_begin(uint16_t* dest_addr, uint16_t size);

void ADC_Init(void);
void ADC_Enable2(void);
void ADC_Start(void);
void DMA_Init(void);
void DMA_Config(uint32_t srcAdd, uint32_t destAdd, uint16_t size);
void adc_test_2(void);



#ifdef __cplusplus
}
#endif

#endif /* ADC_H__ */

