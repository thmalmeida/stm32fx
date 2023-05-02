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

enum class adc_mode {
	noption = 0,
	oneshot,
	stream
};

enum class adc_stream_states {
	stopped = 0,
	running
};

struct pattern_s {
	uint32_t channel;
	uint32_t attenuation = 0;
	uint32_t bitwidth = 12;
	uint32_t rank_position;
};

class ADC_driver {
public:
	ADC_driver(adc_mode mode = adc_mode::oneshot) {
		hadc1_ = &hadc1;

		switch (mode) {
			case adc_mode::oneshot: {
				oneshot_init();
				break;
			}
			case adc_mode::stream: {
				stream_init();
				break;
			}
			default: {
				break;
			}
		}
	}
	~ADC_driver(void) {}
	// Oneshot functions - ADC single mode
	void oneshot_init(void) {
		// resource allocation
		hadc1_->Instance = ADC1;
		hadc1_->Init.ScanConvMode = ADC_SCAN_ENABLE;// ADC_SCAN_DISABLE;		// use SCAN when read 2 or more outputs simultaneusly
		hadc1_->Init.ContinuousConvMode = ENABLE;
		hadc1_->Init.DiscontinuousConvMode = DISABLE;
		hadc1_->Init.ExternalTrigConv = ADC_SOFTWARE_START; // ADC_EXTERNALTRIGCONV_T3_TRGO;
		hadc1_->Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc1_->Init.NbrOfConversion = 1;					// will convert 1 channels
		
		if (HAL_ADC_Init(hadc1_) != HAL_OK) {
			printf("ADC init error!\n");
			Error_Handler();
		} else {
			// printf("ADC single read init\n");
		}

		// next step is configure the channels
	}
	void oneshot_deinit(void) {
		if(HAL_ADC_DeInit(hadc1_) != HAL_OK) {
			printf("ADC deinit error!\n");
		} else {
			printf("ADC deinit\n");
		}
	}
	void oneshot_channel_config(int channel) {
		/** Configure Regular Channel */
		GPIO_InitTypeDef GPIO_InitStruct;	// = {0};
			
		switch (channel) {
			case 0:
				channel_addr_ = ADC_CHANNEL_0;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_0;			// ADC1_IN0 at PA0
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				break;

			case 1:
				channel_addr_ = ADC_CHANNEL_1;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_1;			// ADC1_IN1 at PA1
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				break;
			
			case 2:
				channel_addr_ = ADC_CHANNEL_2;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_2;			// ADC1_IN2 at PA2
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				break;

			case 3:
				channel_addr_ = ADC_CHANNEL_3;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_3;			// ADC1_IN3 at PA3
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				break;

			case 4:
				channel_addr_ = ADC_CHANNEL_4;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_4;			// ADC1_IN4 at PA4
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				break;

			case 5:
				channel_addr_ = ADC_CHANNEL_5;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_5;			// ADC1_IN5 at PA5
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				break;

			case 6:
				channel_addr_ = ADC_CHANNEL_6;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_6;			// ADC1_IN6 at PA6
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				break;

			case 7:
				channel_addr_ = ADC_CHANNEL_7;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_7;			// ADC1_IN7 at PA7
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				break;

			case 8:
				channel_addr_ = ADC_CHANNEL_8;
				__HAL_RCC_GPIOB_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_0;			// ADC1_IN8 at PB0
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				break;

			case 9:
				channel_addr_ = ADC_CHANNEL_9;
				__HAL_RCC_GPIOB_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_1;			// ADC1_IN9 at PB1
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				break;

			case 17:
				channel_addr_ = ADC_CHANNEL_VREFINT;
				break;

			case 16:
				channel_addr_ = ADC_CHANNEL_TEMPSENSOR;
				break;
			
			default:
				return;
				break;
		}

		cirp_++;									// increment to next pattern table position
		ptable_[cirp_].channel = channel_addr_;
		ptable_[cirp_].rank_position = cirp_+1;		// rank position starts at 1

		ADC_ChannelConfTypeDef sConfig;
		sConfig.Channel = ptable_[cirp_].channel;
		sConfig.Rank = ptable_[cirp_].rank_position;	//ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		if (HAL_ADC_ConfigChannel(hadc1_, &sConfig) != HAL_OK) {
			printf("ADC channel config error\n");
			Error_Handler();
		} else {
			printf("ADC channel %lu configured\n", channel_addr_);
		}

		printf("index:%d ch:%lu, rank: %lu\n", cirp_, ptable_[cirp_].channel, ptable_[cirp_].rank_position);
		// /** Configure Regular Channel */
		// sConfig.Channel = ADC_CHANNEL_VREFINT;
		// sConfig.Rank = ADC_REGULAR_RANK_2;
		// sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		// if (HAL_ADC_ConfigChannel(hadc1_, &sConfig) != HAL_OK)
		// {
		// 	printf("ADC: rank 2 error!\n");
		// 	Error_Handler();
		// }
	}
	void channel_select(int channel) {
		channel_ = channel;
		uint8_t index = find_index_(channel);
		printf("index:%d ch:%lu, rank: %lu\n", index, ptable_[index].channel, ptable_[index].rank_position);

		ADC_ChannelConfTypeDef sConfig;
		sConfig.Channel = ptable_[index].channel;					//static_cast<uint32_t>(pattern_table[index][0]);
		sConfig.Rank = ptable_[index].rank_position;				//static_cast<uint32_t>(pattern_table[index][1]);
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

		if (HAL_ADC_ConfigChannel(hadc1_, &sConfig) != HAL_OK) {
			printf("ADC channel config error\n");
			Error_Handler();
		} else {
			// printf("ADC channel %lu selected\n", channel_addr_);
		}

	}
	uint8_t find_index_(int channel) {
		// this will seek for channels corresponding position
		// printf("find index- \n");
		for(int i=0; i<=cirp_; i++) {
			// printf("\tch:%lu ptable_ch:%lu\n", static_cast<uint32_t>(channel), ptable_[i].channel);
			if(static_cast<uint32_t>(channel) == ptable_[i].channel) {
				return i;
			}
		}
		// if no channel found, return 0;
		return 0;
	}
	int read(void) {
		int adc_raw = 0;
		HAL_ADC_Start(hadc1_);
		HAL_ADC_PollForConversion(hadc1_, 10);
		adc_raw = static_cast<int>(HAL_ADC_GetValue(hadc1_));
		HAL_ADC_Stop(hadc1_);

		return adc_raw;
	}
	int read(int channel) {
		channel_select(channel);

		return read();
	}
	// Stream functions - ADC DMA Continuous mode
	void stream_init(void) {
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

		// adc_dma_begin();
	}
	void stream_deinit(void) {
	}
	void stream_read(uint16_t *buffer, int length) {

		uint32_t buffer_temp[length];
		HAL_ADC_Start_DMA(hadc1_, &buffer_temp[0], static_cast<uint32_t>(length));
		
		for(int i=0; i<length; i++) {
			buffer[i] = buffer_temp[i];
		}
		// HAL_ADC_Stop_DMA(hadc1_);
	}
	void stream_read(int channel, uint16_t* buffer, int length) {
		channel_select(channel);
		stream_read(buffer, length);
	}
	void stream_start(void) {
	}
	void calibrate(void) {
		HAL_ADCEx_Calibration_Start(hadc1_);
	}

	void ptable_print_(void) {
		printf("ptable_- \n");
		for(int i=0; i<=cirp_; i++) {
			printf("\ti:%d ch:%lu, rank: %lu\n", i, ptable_[i].channel, ptable_[i].rank_position);
		}
	}

private:
	int channel_;
	uint32_t channel_addr_;	// channel type converted
	int cirp_ = -1;			// channel index rank position
	pattern_s ptable_[17];	// pattern table
	


/*
			Pattern Table [12][2]
		 _________________________
  index	|_Channel_|_Rank_position_|
	0	|    2    |      1        |
	1	|    3    |      2        |
	2	|    1    |      3        |
	3	|    4    |      4        |
	4	|  Vref   |      5        |
	5	|__Temp___|______6________|

*/


	// STM32F specifics
	ADC_HandleTypeDef *hadc1_;
	DMA_HandleTypeDef *hdma_adc1_;
};

#endif /* __ADC_DRIVER_HPP__ */
