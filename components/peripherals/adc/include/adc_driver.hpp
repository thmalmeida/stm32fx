/*
 * adc.h
 *
 *  Created on: 18 de fev de 2017
 *      Author: titi
 */

#ifndef ADC_DRIVER_HPP__
#define ADC_DRIVER_HPP__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// STM32----------------------
#include "adc.h"
#include "stm32_log.h"
// ---------------------------

class ADC_driver {
public:
	uint8_t channel;

	// ADC_driver(void) {}
	ADC_driver(void) {
		hadc1_ = &hadc1;
	}
	~ADC_driver(void) {}

	void init(void) {

		hadc1_->Instance = ADC1;
		hadc1_->Init.ScanConvMode = ADC_SCAN_DISABLE;// ADC_SCAN_ENABLE;			// use SCAN when read 2 or more outputs simultaneusly
		hadc1_->Init.ContinuousConvMode = DISABLE;
		hadc1_->Init.DiscontinuousConvMode = DISABLE;
		hadc1_->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; //ADC_SOFTWARE_START;
		hadc1_->Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc1_->Init.NbrOfConversion = 1;						// will convert 2 channels
		
		if (HAL_ADC_Init(hadc1_) != HAL_OK) {
			printf("ADC init error!\n");
			Error_Handler();
		} else {
			printf("ADC: initialized!\n");
		}

		ADC_ChannelConfTypeDef sConfig = {0};

		/** Configure Regular Channel */
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		// sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		if (HAL_ADC_ConfigChannel(hadc1_, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}

		// /** Configure Regular Channel */
		// sConfig.Channel = ADC_CHANNEL_VREFINT;
		// sConfig.Rank = ADC_REGULAR_RANK_2;
		// // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		// if (HAL_ADC_ConfigChannel(hadc1_, &sConfig) != HAL_OK)
		// {
		// 	printf("ADC: rank 2 error!\n");
		// 	Error_Handler();
		// }

		// /** Configure Regular Channel */
		// sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
		// sConfig.Rank = ADC_REGULAR_RANK_3;
		// // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		// if (HAL_ADC_ConfigChannel(hadc1_, &sConfig) != HAL_OK)
		// {
		// 	printf("ADC: rank 3 error!\n");
		// 	Error_Handler();
		// }
	}
	void calibrate(void) {
		HAL_ADCEx_Calibration_Start(hadc1_);
	}
	void select_channel(int channel) {

	}
	uint16_t read(int channel) {

		uint32_t adc_raw = 0;
		HAL_ADC_Start(hadc1_);
		HAL_ADC_PollForConversion(hadc1_, 500);
		adc_raw = HAL_ADC_GetValue(hadc1_);
		HAL_ADC_Stop(hadc1_);

		return static_cast<uint16_t>(adc_raw);
	}
	void read_stream(uint32_t *adc_data_raw, int length) {
		HAL_ADC_Start_DMA(hadc1_, adc_data_raw, static_cast<uint32_t>(length));
		// HAL_ADC_Stop_DMA(hadc1_);
	}


private:
	ADC_HandleTypeDef *hadc1_;
	DMA_HandleTypeDef *hdma_adc1_;
};

#endif /* __ADC_DRIVER_HPP__ */
