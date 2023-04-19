/*
 * adc.h
 *
 *  Created on: 18 de fev de 2017
 *      Author: titi
 */


/* Briefly description of ADC behavior's

	1
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

enum class adc_read_mode {
	single_read = 0,
	stream_dma_read
};

class ADC_driver {
public:
	uint8_t channel;

	// ADC_driver(void) {}
	ADC_driver(uint32_t channel, adc_read_mode mode) {
		hadc1_ = &hadc1;

		init(channel, mode);
	}
	~ADC_driver(void) {}

	void init(uint32_t channel, adc_read_mode mode) {

		if(mode == adc_read_mode::single_read) {
			hadc1_->Instance = ADC1;
			hadc1_->Init.ScanConvMode = ADC_SCAN_DISABLE;		// use SCAN when read 2 or more outputs simultaneusly
			hadc1_->Init.ContinuousConvMode = DISABLE;
			hadc1_->Init.DiscontinuousConvMode = DISABLE;
			hadc1_->Init.ExternalTrigConv = ADC_SOFTWARE_START; // ADC_EXTERNALTRIGCONV_T3_TRGO;
			hadc1_->Init.DataAlign = ADC_DATAALIGN_RIGHT;
			hadc1_->Init.NbrOfConversion = 1;					// will convert 2 channels
			
			if (HAL_ADC_Init(hadc1_) != HAL_OK) {
				printf("ADC init error!\n");
				Error_Handler();
			} else {
				printf("ADC single read init\n");
			}

			channel_config(channel);

		} else if(mode == adc_read_mode::stream_dma_read) {
			// adc with dma init
		}
	}
	void channel_config(uint8_t channel) {

		/** Configure Regular Channel */
		ADC_ChannelConfTypeDef sConfig = {0};
		switch (channel) {
			case 0:
				channel_ = ADC_CHANNEL_0;
			break;

			case 1:
				channel_ = ADC_CHANNEL_1;
				break;
			
			case 2:
				channel_ = ADC_CHANNEL_2;
				break;

			case 3:
				channel_ = ADC_CHANNEL_3;
				break;

			case 4:
				channel_ = ADC_CHANNEL_4;
				break;

			case 5:
				channel_ = ADC_CHANNEL_5;
				break;

			case 6:
				channel_ = ADC_CHANNEL_6;
				break;

			case 7:
				channel_ = ADC_CHANNEL_7;
				break;

			case 8:
				channel_ = ADC_CHANNEL_8;
				break;

			case 9:
				channel_ = ADC_CHANNEL_9;
				break;
		}
		sConfig.Channel = channel_;
		printf("ADC: config channel: %d\n", static_cast<int>(channel_));
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
	void channel_select(int channel) {

	}
	void calibrate(void) {
		HAL_ADCEx_Calibration_Start(hadc1_);
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
	uint32_t channel_;

	// STM32F specifics
	ADC_HandleTypeDef *hadc1_;
	DMA_HandleTypeDef *hdma_adc1_;
};

#endif /* __ADC_DRIVER_HPP__ */
