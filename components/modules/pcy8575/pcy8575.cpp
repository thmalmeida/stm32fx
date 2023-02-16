#include "pcy8575.hpp"

const char* TAG_PCY8575 = "PCY8575";

pcy8575::pcy8575(void) {

}
void pcy8575::init(void) {

	i2c_init(PCY8575_ADDR, 100000);
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
void pcy8575::handle_message(void) {

}