#ifndef __HX711_H__
#define __HX711_H__

#include "gpio.hpp"

/* This files is used to control the HX711 module
* 
* The HX711 digital communication is made in a serial way using two pins.
* We need one pin for data and another for the clock.
* 
*/

#define HX711_time_protocol_us		1     // time delay protocol of HX711 interface [us];
#define HX711_pin_sck 				5
#define HX711_pin_data				6

class HX711 {
	public:

	void init(void);

	uint32_t read(void) {
		int cycles = 24;				// 24 bit read size
		uint32_t value = 0;

		for(int i=0; i<cycles; i++) {
			pin_sck(1);
			_delay_us(HX711_time_protocol_us);

			value = value << 1;

			write_pin_sck(0);
			_delay_ms(HX711_time_protocol_us);



		}

		return value;
	}

	private:

	GPIO_driver pin_[2];

	void write_pin_sck_(int status) {

	}

	uint32_t read_pin_data_(void) {

		return value;
	}
}

#endif