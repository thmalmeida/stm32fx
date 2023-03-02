#ifndef _PCY8575_HPP__
#define _PCY8575_HPP__

#include "i2c.h"
#include "gpio.hpp"
#include "stm32_log.h"

/* list of I2C addresses */
#define PCY8575_ADDR					0x53	// device address: 0b0010 0011 >> 0b0001 0001 = 0x11
#define PCY8575_NORMAL_SPEED			100000  // i2c normal speed [Hz]


/* list of command registers */
#define PCY8575_REG_PROBE				0x00
#define PCY8575_REG_SOFT_RESET			0x01
#define PCY8575_REG_CONFIG				0x02
#define PCY8575_REG_PUT					0x03
#define PCY8575_REG_GET					0x04
#define PCY8575_REG_TEMPERATURE			0x05

/* PCY8575 protocol will works always in a slave mode. 
The master uC has the control over the SCL clock.

When master write

when master read


write mode

There are 16 controlled pins
____________.________.________.
slave_addr|0| opcode | byte 1

opcode:
	- PROBE ok:		0x00
	- Soft RESET:	0x01
	- CONFIG ports:	0x02
	- PUT:	 		0x03
	- GET:			0x04
	- TEMP:			0x05

protocol example

PROBE:		write													read
Start | ADDR - R/W = 0 | PROBE | Stop | ... delay ... | Start | ADDR - R/W = 1 | read byte 1 |

CONFIG:		write				  P07-P00   P15-P00
Start | ADDR - R/W = 0 | CONFIG	| byte 0  | byte 1  | Stop |

PUT:		write
Start | ADDR - R/W = 0 | CONFIG | Stop | ... delay ... | Start | ADDR - R/W = 1 | read byte 1 |

GET:		write												 Read			  P07-P00   P15-P00
Start | ADDR - R/W = 0 | GET    | Stop | ... delay ... | Start | ADDR - R/W = 1 | byte 0  | byte 1 |

GET TEMP:	write												 Read			      16 bits
Start | ADDR - R/W = 0 | TEMP   | Stop | ... delay ... | Start | ADDR - R/W = 1 | byte L  | byte H |

SOFT RESET:	write
Start | ADDR - R/W = 0 | RESET | Stop |

*/

class pcy8575 {

	public:

	pcy8575(void) : pin_{
							{16, 1}, {17, 1}, {18, 1}, {19, 1}, {20, 1}, {21, 1}, {22, 1}, {25 , 1},
							{26, 1}, {27, 1}, {28, 1}, {1 , 1}, {31, 1}, {32, 1}, {12, 1}, {13, 1}} {}
	// pcy8575::pcy8575(void) : pin_{
	// 								{1 , 1}, {2 , 1}} {}

	~pcy8575(void) {}

	void init(void) {
		i2c_init(PCY8575_ADDR, PCY8575_NORMAL_SPEED);
		i2c_enable_interrupt();
		i2c_set_ack();
		// i2c_set_nostretch();

		i2c_read_CR1_reg();
		i2c_read_CR2_reg();
		i2c_read_OAR1_reg();

		i2c_print_CR2_reg();
		i2c_print_CR1_reg();
		i2c_print_addr1();
		// printf("I2C State: 0x%02x\n",HAL_I2C_GetState(&hi2c2));

		// I2C to GPIO system
		// declare pin status registers
	}
	void handle_message(void) {
		
		// master read - slave transmitter
		if(i2c_has_data_tx) {
			i2c_has_data_tx = 0;

			if(i2c_data_tx_size) {
				reg_addr_ = i2c_data_rx[0];

				// printf("\ni2c_data_tx_size: %d, reg_addr:0x%02x\n", i2c_data_tx_size, reg_addr_);
				// for(int i=0; i<i2c_data_tx_size; i++) {
				// 	printf("i2c_data_tx[%d]: 0x%02x\n", i, i2c_data_tx[i]);
				// }
				// printf("output_: 0x%04x\n", output_);
			}
		}

		// master write - slave receiver
		if(i2c_has_data_rx) {
			i2c_has_data_rx = 0;

			switch (i2c_data_rx[0]) {
				case 0x00: 	// PROBE ok
					printf("opcode: probe!: 0x%04x\n", output_);
				break;

				case 0x01:	// Soft RESET
					printf("opcode: restarting...\n\n");
					HAL_Delay(5);
					soft_reset();
				break;

				case 0x02:	// CONFIG
					// printf("opcode: port config\n");
					port_config_ = (i2c_data_rx[2] << 8) | i2c_data_rx[1];
					config(port_config_);		
				break;

				case 0x03:	// PUT
					output_ = (i2c_data_rx[2] << 8) | i2c_data_rx[1];
					put(output_);
					// printf("opcode: put received 0x%04x, 0x%02x, 0x%02x\n", output_, i2c_data_rx[2], i2c_data_rx[1]);
				break;

				case 0x04:	// GET
					// Master read has write first. Print functions cause delay errors.
					output_ = get();
					i2c_data_tx[0] = output_ & 0xFF;
					i2c_data_tx[1] = (output_ >> 8) & 0xFF;
					// printf("opcode: get received, output:0x%04x\n", output_);
				break;

				case 0x05:	// TEMP
					// printf("get temp\n");
					temp_ = get_temp();
					i2c_data_tx[0] = static_cast<uint8_t>(temp_);
					i2c_data_tx[1] = static_cast<uint8_t>(temp_ >> 8);
				break;

				default:
					printf("default handle msg\n");
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
		for(int i=0; i<16; i++) {
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
	uint16_t get_temp(void) {
		uint16_t temp = static_cast<uint16_t>(0x1243);

		return temp;
	}
	void soft_reset(void) {
		HAL_NVIC_SystemReset();
	}
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
	GPIO_driver pin_[16];
	uint16_t output_ = 0;
	uint16_t port_config_ = 0xFFFF;
	uint16_t temp_ = 0;

	// const std::size_t pin_count_ = sizeof(pin_) / sizeof(pin_[0]);

};

#endif //#ifndef _PCY8575_H__