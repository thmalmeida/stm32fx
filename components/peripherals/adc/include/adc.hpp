/*
 * adc.h
 *
 *  Created on: 18 de fev de 2017
 *      Author: titi
 */

#ifndef __ADC_HPP_
#define __ADC_HPP_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "system_main.h"

class ADC_driver {
public:
	uint8_t _channel;

	adc(void) {

	}
	~adc(void) {}

	void init();
	uint16_t read(int channel);
	void select_channel(int channel);
};

#endif /* __ADC_HPP_ */
