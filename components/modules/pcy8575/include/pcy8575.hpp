#ifndef _PCY8575_HPP__
#define _PCY8575_HPP__

#include "i2c.h"
#include "tim.h"						// needs for module uptime. TIM3 update every 1 second.
#include "gpio.hpp"
#include "stm32_log.h"
#include "backup.hpp"
#include "reset_reason.hpp"

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
#define PCY8575_REG_IRMS		0x08
#define PCY8575_REG_I_DATA		0x09


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

I_DATA:		write				  number to be readed mas 2^16   				   Read. Total of 2*n_samples bytes to transmit.
Start | ADDR - R/W = 0 | I_DATA | byte H | byte L | Stop | ... delay ... | Start | ADDR - R/W = 1 | byte H0 | byte L0 | ... | byte H(n-1) | byte L(n-1) | Stop |
*/

class pcy8575 {

public:

	static const int num_pins = 16;

	// (Pin_number, mode)
	pcy8575(void) : pin_{
							{31, 1}, {25, 1}, {22, 1}, {21, 1},
							{20, 1}, {19, 1}, {18, 1}, {17, 1},
							{16, 1}, {28, 1}, {27, 1}, {1 , 1},
							{10, 1}, {11, 1}, {12, 1}, {13, 1}} {}

	~pcy8575(void) {}

	void init(void) {
		// I2C slave initialization
		i2c_init(PCY8575_ADDR, PCY8575_NORMAL_SPEED);

		// Restore last output pins before reset
		put(backup_DR1_get());

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
				// 	printf("i2c_data_tx[%d]: 0x%02x\n", i, i2c_data_tx[i]);
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
					i2c_data_tx[0] = 0xAA;
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
					i2c_data_tx[0] = static_cast<uint8_t>(output_);
					i2c_data_tx[1] = static_cast<uint8_t>(output_ >> 8);
					break;
				}
				case PCY8575_REG_TEMPERATURE: { // TEMP
					temp_ = temp();
					i2c_data_tx[0] = static_cast<uint8_t>(temp_);
					i2c_data_tx[1] = static_cast<uint8_t>(temp_ >> 8);
					break;
				}
				case PCY8575_REG_UPTIME: { // UPTIME
					uptime_temp_ = uptime();
					i2c_data_tx[0] = static_cast<uint8_t>(uptime_temp_);
					i2c_data_tx[1] = static_cast<uint8_t>(uptime_temp_ >> 8);
					i2c_data_tx[2] = static_cast<uint8_t>(uptime_temp_ >> 16);
					i2c_data_tx[3] = static_cast<uint8_t>(uptime_temp_ >> 24);
					break;
				}
				case PCY8575_REG_RST_REASON: {
					i2c_data_tx[0] = static_cast<uint8_t>(reset_reason());
					break;
				}
				case PCY8575_REG_IRMS: {
					irms_ = irms();	// this conversion located here will depends the time calculation because the i2c transfer time
					i2c_data_tx[0] = static_cast<uint8_t>(irms_);
					i2c_data_tx[1] = static_cast<uint8_t>(irms_ >> 8);
					break;
				}
				case PCY8575_REG_I_DATA: {

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
	uint16_t temp(void) {
		uint16_t temp = static_cast<uint16_t>(0x1243);

		return temp;
	}
	uint32_t uptime(void) {
		return tim3_cnt;
	}
	uint16_t irms(void) {
		return 0x2468;
	}
	void soft_reset(void) {
		HAL_NVIC_SystemReset();
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

	uint8_t reg_addr_ = 0;							// register to read
	GPIO_driver pin_[num_pins];
	uint16_t output_ = 0;
	uint16_t port_config_ = 0xFFFF;
	uint16_t temp_ = 0;
	uint16_t uptime_temp_ = 0;
	uint16_t irms_ = 0;
	// const std::size_t pin_count_ = sizeof(pin_) / sizeof(pin_[0]);
};

#endif // _PCY8575_HPP__