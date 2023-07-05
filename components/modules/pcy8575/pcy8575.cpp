#include "pcy8575.hpp"

const char* TAG_PCY8575 = "PCY8575";

uint16_t adc_array_data_raw[4] = {0x0A03, 0x0405, 0x0607, 0x0809};

// void pcy8575::init(void) {

// }
// void pcy8575::handle_message(void) {

// 	if(i2c_has_data_tx) {
// 		i2c_has_data_tx = 0;

// 		if(i2c_data_tx_size) {
// 			printf("\ni2c_data_tx_size: %d, reg_addr:0x%02x\n", i2c_data_tx_size, reg_addr_);
// 			reg_addr_ = 0x00;
// 			for(int i=0; i<i2c_data_tx_size-1; i++) {
// 				printf("data_tx[%d]: 0x%02x\n", i, data_tx[i]);
// 				data_tx[i]++;
// 			}
// 		}
// 	}

// 	if(i2c_has_data_rx) {
// 		i2c_has_data_rx = 0;

// 		if(i2c_data_rx_size > 1) {
// 			printf("\ni2c_data_rx_size: %d\n", i2c_data_rx_size);
// 			for(int i=0; i<i2c_data_rx_size-1; i++) {
// 				printf("data_rx[%d]: 0x%02x\n", i, data_rx[i]);
// 			}
// 		}
// 		else if(i2c_data_rx_size) {
// 			// this takes the reg_add sent by master into write before read.
// 			reg_addr_ = data_rx[0];
// 		}
// 	}
// }
// void pcy8575::test(void) {
// 	pin_[15].toggle();
// 	delay_ms(500);
// }