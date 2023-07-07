#ifndef _PCY8575_HPP__
#define _PCY8575_HPP__

#include "i2c.h"
// #include "tim.h"						
#include "gpio.hpp"
#include "stm32_log.h"
#include "backup.hpp"
#include "reset_reason.hpp"
#include "tim_driver.hpp"	// needs for module uptime. TIM3 update every 1 second.
#include "iwdg.h"
#include "adc_driver.hpp"

#include "delay.hpp"
#include "dsp.hpp"

/* list of I2C addresses */
#define PCY8575_ADDR			0x53	// device address: 0b0010 0011 >> 0b0001 0001 = 0x11
#define PCY8575_NORMAL_SPEED	100000  // i2c normal speed [Hz]
#define PCY8575_FAST_SPEED		400000  // i2c fast speed [Hz]


/* list of command registers - opcodes */
#define PCY8575_REG_PROBE		0x00
#define PCY8575_REG_SOFT_RESET	0x01
#define PCY8575_REG_CONFIG		0x02
#define PCY8575_REG_PUT			0x03
#define PCY8575_REG_GET			0x04
#define PCY8575_REG_TEMPERATURE	0x05
#define PCY8575_REG_UPTIME		0x06
#define PCY8575_REG_RST_REASON	0x07
#define PCY8575_REG_I_PROCESS	0x08
#define PCY8575_REG_IRMS		0x09
#define PCY8575_REG_I_DATA		0x0A
#define PCY8575_REG_TEST		0x0B
#define PCY8575_REG_I_SET_NP	0x0C
#define PCY8575_REG_I_GET_NP	0x0D



#define PCY8575_DEBUG_PRINT		1

/* PCY8575 protocol will works always in a slave mode. 
The master uC has the control over the SCL clock.

There are 16 controlled pins
____________.________.________.
slave_addr|0| opcode | byte 1

opcode:
	- PROBE:		0x00
	- RESET:		0x01	// soft reset
	- CONFIG:		0x02	// configure ports
	- PUT:	 		0x03
	- GET:			0x04
	- TEMP:			0x05
	- UPTIME:		0x06
	- RST_REASON:	0x07
	- IRMS:			0x08
	- I_DATA:		0x09
	- DATA_TEST:	

protocol example

PROBE:		write													read
Start | ADDR - R/W = 0 | PROBE	| Stop | ... delay ... | Start | ADDR - R/W = 1 | byte 0 | Stop |

SOFT RESET:	write
Start | ADDR - R/W = 0 | RESET	| Stop |

CONFIG:		write				  P07-P00   P15-P00
Start | ADDR - R/W = 0 | CONFIG	| byte 0  | byte 1  | Stop |

PUT:		write
Start | ADDR - R/W = 0 | PUT    | byte 0  | byte 1  | Stop |

GET:		write												 Read			  P07-P00   P15-P00
Start | ADDR - R/W = 0 | GET    | Stop | ... delay ... | Start | ADDR - R/W = 1 | byte 0  | byte 1 | Stop |

TEMP:		write												 Read					16 bits
Start | ADDR - R/W = 0 | TEMP   | Stop | ... delay ... | Start | ADDR - R/W = 1 | byte L  | byte H | Stop |

UPTIME:		write												 Read			     	32 bits
Start | ADDR - R/W = 0 | UPTIME | Stop | ... delay ... | Start | ADDR - R/W = 1 | byte L  | byte L | byte H | byte H | Stop |

IRMS:		write																		16 bits
Start | ADDR - R/W = 0 | IRMS	| Stop | ... delay ... | Start | ADDR - R/W = 1 | byte L  | byte H | Stop |

RST_REASON:	write												 Read				8 bits
Start | ADDR - R/W = 0 | REASON | Stop | ... delay ... | Start | ADDR - R/W = 1 | byte L  | Stop |

I_DATA:		write				  number to be readed max of 2^16  				   Read. Total of 2*n_samples bytes to transmit.
Start | ADDR - R/W = 0 | I_DATA | byte L | byte H | Stop | ... delay ... | Start | ADDR - R/W = 1 | byte H0 | byte L0 | ... | byte H(n-1) | byte L(n-1) | Stop |

I_CONFIG?

*/

extern uint16_t adc_array_data_raw[4];

class pcy8575 {

public:

	static const int num_pins = 16;

	// (Pin_number, mode)
	pcy8575(void) : pin_{
							{31, 1}, {25, 1}, {22, 1}, {21, 1},
							{20, 1}, {19, 1}, {18, 1}, {17, 1},
							{16, 1}, {28, 1}, {27, 1}, {1 , 1},
							{10, 1}, {11, 1}, {12, 1}, {13, 1}},
					timer_(3, 1, timer_mode::timer_interrupt),
					adc0(adc_mode::stream) {}

	~pcy8575(void) {}

	// pcy8575 standard functions
	void init(void) {
		// I2C slave initialization
		i2c_init(PCY8575_ADDR, PCY8575_NORMAL_SPEED);

		// Restore last output pins before reset
		put(backup_DR1_get());

		// WDT init
		iwdg_init();

		// Verify output pins
		output_ = get();

		#ifdef PCY8575_DEBUG_PRINT
		printf("PCY8575 init with output:0x%04x\n",output_);
		#endif
	}
	void handle_message(void) {
		// master read - slave transmitter
		if(i2c_has_data_tx) {
			i2c_has_data_tx = 0;

			if(i2c_data_tx_size) {
				#ifdef PCY8575_DEBUG_PRINT
				// reg_addr_ = i2c_data_rx[0];
				// printf("opcode:0x%02x, output:0x%04x, i2c_data_tx_size: %d\n", reg_addr_, output_, i2c_data_tx_size);
				// for(int i=0; i<i2c_data_tx_size; i++) {
				// 	printf("data_tx_[%d]: 0x%02x\n", i, data_tx_[i]);
				// }
				// printf("output_: 0x%04x\n", output_);
				#endif
			}
		}
		// master write - slave receiver
		if(i2c_has_data_rx) {
			i2c_has_data_rx = 0;
			reg_addr_ = i2c_data_rx[0];

			switch (reg_addr_) {
				case PCY8575_REG_PROBE: { // PROBE ok
					#ifdef PCY8575_DEBUG_PRINT
					printf("opcode:0x%02x probe\n", reg_addr_);
					for(int i=0; i<i2c_data_rx_size; i++) {
						printf("i2c_data_rx[%d]: 0x%02x\n", i, i2c_data_rx[i]);
					}
					#endif
					data_tx_[0] = 0xAA;
					i2c_data_tx = &data_tx_[0];
					break;
				}
				case PCY8575_REG_SOFT_RESET: { // Soft RESET
					#ifdef PCY8575_DEBUG_PRINT
					printf("opcode:0x%02x restarting...\n\n", reg_addr_);
					for(int i=0; i<i2c_data_rx_size; i++) {
						printf("i2c_data_rx[%d]: 0x%02x\n", i, i2c_data_rx[i]);
					}
					#endif
					HAL_Delay(5);
					soft_reset();
					break;
				}
				case PCY8575_REG_CONFIG: { // CONFIG
					#ifdef PCY8575_DEBUG_PRINT
					printf("opcode:0x%02x config\n", reg_addr_);
					for(int i=0; i<i2c_data_rx_size; i++) {
						printf("i2c_data_rx[%d]: 0x%02x\n", i, i2c_data_rx[i]);
					}
					#endif

					port_config_ = (i2c_data_rx[2] << 8) | i2c_data_rx[1];
					config(port_config_);		
					break;
				}
				case PCY8575_REG_PUT: { // PUT
					#ifdef PCY8575_DEBUG_PRINT
					printf("opcode:0x%02x put\n", reg_addr_);
					for(int i=0; i<i2c_data_rx_size; i++) {
						printf("i2c_data_rx[%d]: 0x%02x\n", i, i2c_data_rx[i]);
					}
					#endif
					output_ = (i2c_data_rx[2] << 8) | i2c_data_rx[1];
					put(output_);
					break;
				}
				case PCY8575_REG_GET: { // GET
					// Master read has write first. Print functions cause delay errors.
					output_ = get();
					data_tx_[0] = static_cast<uint8_t>(output_);
					data_tx_[1] = static_cast<uint8_t>(output_ >> 8);
					i2c_data_tx = &data_tx_[0];
					break;
				}
				case PCY8575_REG_TEMPERATURE: { // TEMP
					temp_ = temp();
					data_tx_[0] = static_cast<uint8_t>(temp_);
					data_tx_[1] = static_cast<uint8_t>(temp_ >> 8);
					i2c_data_tx = &data_tx_[0];
					break;
				}
				case PCY8575_REG_UPTIME: { // UPTIME
					uptime_temp_ = uptime();
					data_tx_[0] = static_cast<uint8_t>(uptime_temp_);
					data_tx_[1] = static_cast<uint8_t>(uptime_temp_ >> 8);
					data_tx_[2] = static_cast<uint8_t>(uptime_temp_ >> 16);
					data_tx_[3] = static_cast<uint8_t>(uptime_temp_ >> 24);
					i2c_data_tx = &data_tx_[0];
					break;
				}
				case PCY8575_REG_RST_REASON: {
					data_tx_[0] = static_cast<uint8_t>(reset_reason());
					i2c_data_tx = &data_tx_[0];
					break;
				}
				case PCY8575_REG_I_PROCESS: {
					process();
				}
					break;
				case PCY8575_REG_IRMS: {
					data_tx_[0] = static_cast<uint8_t>(irms_);
					data_tx_[1] = static_cast<uint8_t>(irms_ >> 8);
					i2c_data_tx = &data_tx_[0];
					break;
				}
				case PCY8575_REG_I_DATA: {
					i2c_data_tx = (uint8_t*) adc_array16_raw_;
					break;
				}
				case PCY8575_REG_I_SET_NP: {
					int num = (i2c_data_rx[2] << 8) | i2c_data_rx[1];
					adc_malloc(num);
					adc0.stream_addr_config(adc_array16_raw_);
					adc0.stream_length_config(num);
					break;
				}
				case PCY8575_REG_I_GET_NP: {
					data_tx_[0] = static_cast<uint8_t>(adc0.stream_length());
					data_tx_[1] = static_cast<uint8_t>(adc0.stream_length() >> 8);
					i2c_data_tx = &data_tx_[0];
					break;
				}
				case PCY8575_REG_TEST: {
					i2c_data_tx = (uint8_t*)&adc_array_data_raw[0];
					break;
				}
				default:
					#ifdef PCY8575_DEBUG_PRINT
					printf("default handle msg\n");
					#endif
					break;
			}

			// if(i2c_data_rx_size > 1) {
			// 	printf("\ni2c_data_rx_size: %d\n", i2c_data_rx_size);
			// 	for(int i=0; i<i2c_data_rx_size-1; i++) {
			// 		printf("i2c_data_rx[%d]: 0x%02x\n", i, i2c_data_rx[i]);
			// 	}
			// }
			// else if(i2c_data_rx_size) {
			// 	// this takes the reg_add sent by master into write before read.
			// 	reg_addr_ = i2c_data_rx[0];
			// }
		}
	}
	void soft_reset(void) {
		HAL_NVIC_SystemReset();
	}
	void config(uint16_t port_config) {
		for(int i=0; i<num_pins; i++) {
			pin_[i].mode((port_config >> i) & 0x01);
		}
	}
	void put(uint16_t output) {
		for(int i=0; i<16; i++) {
			pin_[i].write((output >> i) & 0x01);
		}
	}
	uint16_t get(void) {

		uint16_t output = 0;
		for(int i=0; i<16; i++) {
			if(pin_[i].read())
				output |= (1 << i);
			else
				output &= ~(1 << i);

		}
		// return output;
		return output;
	}

	// must called function to run FSM machine
	void run(void) {
		handle_message();			// handle message fsm

		// print_samples();

		// 1 second flag
		if(timer_.get_isr_flag()) {
			// process();				// start adc dma stream each one second;
			iwdg_refresh();			// clear wdt to prevent reset
		}
	}

	// other functions
	uint16_t temp(void) {
		uint16_t temp = static_cast<uint16_t>(0x1243);

		return temp;
	}
	uint32_t uptime(void) {
		return timer_.get_uptime();
	}

	// configure functions for extra features
	void adc_init(void) {
		// ---- ADC signal parameters
		// 1- just for find the number of points and it's sampling rate;
		// 2- another reason to use adc0 class outside pcy8575 is to keep adc_array_raw vector allocated.
		// 
		// Sampling parameters for STM32F103C8T6 device
		float F_clk = 48000000;							// Main clock system [Hz]
		float div_1 = 1;								// Advanced High-performance Bus (AHB) prescale;
		float div_2 = 2;								// Advanced Peripheral Bus (APB2) prescale;
		float div_3 = 8;								// ADC prescale
		float adc_clk = F_clk/div_1/div_2/div_3;		// ADC clock after all prescalers
		float T_conv = 12.5 + 239.5;					// Number of clock cycles to make one conversion
		float Fs_adc = adc_clk/T_conv;					// Sample rate calculation [Samples/s];

		float f_signal = 60.0;							// signal frequency [Hz]
		float n_points_cycle = Fs_adc/f_signal;			// number of points per cycle or period time

		int n_cycles = 2;								// Number of cycles desired to analyze
		int n_points = ceil(n_cycles*n_points_cycle);	// The number of points is calculated based on the ADC sample rate.
		// uint16_t adc_array_raw[n_points];				// Array allocation to receive converted points
		// uint16_t *adc_array_raw = new uint16_t[n_points];
		// uint16_t *adc_array_raw;

		printf("Sample rate: %f\n", Fs_adc);
		printf("points/cycle: %f\n", n_points_cycle);
		printf("n cycles: %d\n", n_cycles);
		printf("total points: %d\n", n_points);

		// ADC_driver adc0(adc_mode::stream);
		adc0.stream_init();
		adc0.channel_config(3);

		adc_malloc(n_points);
		adc0.stream_addr_config(adc_array16_raw_);
		adc0.stream_length_config(n_points);
		// adc0.stream_addr_config(n_points);
		// memset(adc_array_raw, 0, sizeof(adc_array_raw));
		// ---- end	
	}
	// void adc_config(ADC_driver* adc) {
		// adc0_ = adc;
	// }
	void adc_malloc(int length) {
		if(!adc_alloc_flag_) {
			adc_alloc_flag_ = 1;
			adc_array16_raw_ = (uint16_t*) malloc(length*sizeof(uint16_t));
		} else {
			adc_array16_raw_ = (uint16_t*) realloc(adc_array16_raw_, length*sizeof(uint16_t));
		}
	}
	void adc_buffer_clear(void) {
		// delete[] adc_array16_raw_;
		free(adc_array16_raw_);
	}

	// dsp functions
	uint16_t irms(void) {

		int n_samples = adc0.stream_length();
		// time domain load current;
		double iL_t[n_samples];

		// rms load current;
		double iL_rms;

		// Read stream array from ADC using DMA and fill stream_data_p memory pointer
		adc0.stream_read();
		// delay_ms(100);

		uint32_t count = 0;
		while(!adc0.stream_ready()) {
			count++;
			delay_ms(1);
		}

		printf("Count: %lu\n", count);

		// Convert digital ADC raw array to iL(t) signal;
		s0_.calc_iL_t(adc_array16_raw_, &iL_t[0], n_samples);

		// Remove DC offset
		s0_.dc_remove(&iL_t[0], n_samples);

		// Find the RMS value from iL(t) signal
		iL_rms = s0_.calc_rms(&iL_t[0], n_samples);

		// print adc raw values for debug purposes
		// printf("adc_buffer: ");
		// for(int i=0; i<n_samples; i++) {
		// 	printf("%u, ", adc_array16_raw_[i]);
		// }
		// printf("\n");

		// print rms load current in Amperes
		// printf("iL_rms:%.2lf A\n", iL_rms);
		// ESP_LOGI(TAG_SETUP, "iL_rms:%.2lf A\n", iL_rms);

		return static_cast<uint16_t>(iL_rms*1000);
	}
	void process(void) {
		irms_ = irms();
		print_samples();
		printf("irms: %u\n\n", irms_);
	}
	// void convert_8_to_16(uint8_t* src, uint16_t* dest, int src_len) {

	// }
	// void convert_16_to_8(uint16_t* src, uint8_t* dest, int src_len) {

	// }
	void print_samples(void) {

		// if(adc0.stream_ready()) {
			printf("stream_data_p: ");
			for(auto i=0; i<adc0.stream_length(); i++) {
				printf("%u, ", adc_array16_raw_[i]);
			}
			printf("\n");
		// }

				// for(int i=0; i<adc0.stream_length()*2; i+=2) {
		// 	adc_array8_raw_[i] = 0x00FF & adc_array16_raw_[i];	// low byte first
		// 	adc_array8_raw_[i+1] = adc_array16_raw_[i] >> 8;				// high byte
		// }

		// printf("adc_data_raw_: ");
		// for(auto i=0; i<adc0.stream_length(); i++) {
		// 	printf("%u, ", adc_array8_raw_[i]);
		// }
		// printf("\n");

	}

	// Test functions
	void test_up(void) {
		// uint16_t value = (1 << (pino - 1));
		// put(1<<11);
		pin_[11].write(1);
	}
	void test_down(void) {
		// uint16_t value = ~(1 << (pino - 1));
		// put(value);
		pin_[11].write(0);
	}

private:

	uint8_t reg_addr_ = 0;			// i2c protocol: register to read
	GPIO_driver pin_[num_pins];
	uint16_t output_ = 0;
	uint16_t port_config_ = 0xFFFF;
	uint16_t temp_ = 0;
	uint16_t uptime_temp_ = 0;
	uint16_t irms_ = 0;
	uint8_t data_tx_[10];
	// const std::size_t pin_count_ = sizeof(pin_) / sizeof(pin_[0]);

	// for uptime and 1 second flag
	TIM_driver timer_;

	// DSP functions;
	DSP s0_;

	// ADC sensors
	ADC_driver adc0;
	uint16_t *adc_array16_raw_;
	uint8_t adc_alloc_flag_ = 0;
};

#endif // _PCY8575_HPP__
