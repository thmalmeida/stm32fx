#ifndef __HX711_H__
#define __HX711_H__

#include "gpio_driver.h"

/* This files is used to control the HX711 module
* 
* The HX711 digital communication is made in a serial way using two pins.
* We need one pin for data and another for the clock.
* 
*/

#define HX711_time_protocol_us		1     // time delay protocol of HX711 interface [us];

class HX711 {
public:

	HX711(int pin_dout, int pin_pd_sck) : pin_{{pin_dout, 0},{pin_pd_sck, 1}} {
		pin_[0].mode(0);	// set pin_dout as input
		pin_[1].mode(1);	// set pin sck as output

		
		// gateConfig(pin_data, 0);		// data pin
		// gateConfig(pin_sck, 1);			// sck pin
		// gateConfig(pin_tare_HX711, 0);	// data pin
		// gateConfig(pin_led, 1);			// led
		// gateConfig(pin_beep, 1);		// beep
		// pin_data_HX711 = pin_data;
		// pin_sck_HX711 = pin_sck;
	}
	uint32_t read(void) {
		uint32_t value = 0;

		// wait pin data go low
		// while(read_pin_data_());

		// 24 bit read size. Should run 24 cycles
		for(int i=0; i<24; i++) {

			// start new wave cycle
			write_pin_sck_(1);
			delay_us(HX711_time_protocol_us);

			write_pin_sck_(0);
			delay_ms(HX711_time_protocol_us);

			// shift buffer left before add the new bit
			value <<= 1;

			// OR operation between new pin read and existing buffer value
			value |= read_pin_data_();
		}

		// XOR operatinon on 24 th bit recommended by datasheet
		// value ^= 0x800000;

		write_pin_sck_(1);
		delay_us(HX711_time_protocol_us);

		write_pin_sck_(0);
		delay_us(HX711_time_protocol_us);

		return value;
	}
	void power_down(void) {
		write_pin_sck_(1);
		delay_us(65);
	}
	void reset(void) {
		power_down();
		write_pin_sck_(0);
	}

private:

	GPIO_DRIVER pin_[2];

	void write_pin_sck_(int status) {
		if(status) {
			pin_[1].write(1);
		}
	}
	uint32_t read_pin_data_(void) {
		return static_cast<uint32_t>(pin_[0].read());
	}
};

#endif