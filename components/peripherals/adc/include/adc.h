/*
 * adc.h
 *
 *  Created on: 18 de fev de 2017
 *      Author: titi
 */

#ifndef HARDWARE_ADC_H_
#define HARDWARE_ADC_H_

#include <stdio.h>
#include <stm32f10x.h>

class ADC {
public:
	uint8_t _channel;

	void adc_begin();

	int adc_readChannel(uint8_t channel);
	void adc_selectChannel(uint8_t channel);
};

#endif /* HARDWARE_ADC_H_ */
