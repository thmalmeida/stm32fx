/*
 * spi.h
 *
 *  Created on: 8 de fev de 2017
 *      Author: titi
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <stm32f10x.h>
#include <stm32f10x_spi.h>
#include "../stm32f10x_it.h"

// SPI Defines
/*
	SPI1: default
	SPI1_MOSI - PA7
	SPI1_MISO - PA6
	SPI1_SCK  - PA5
	SPI1_NSS  - PA4

	SPI1: remap
	SPI1_MOSI - PB5
	SPI1_MISO - PB4
	SPI1_SCK  - PB3
	SPI1_NSS  - PA15
*/

class SPI {
public:
	volatile int SPI1_cnt;
	volatile unsigned int SPI1_flag_rx;
	volatile unsigned int SPI1_flag_err;
	volatile unsigned char SPI1_rxd;
	volatile unsigned char SPI1_txd;

	unsigned char SPI_received_string[50]; // this will hold the recieved string

	void set_SPI_to_Master(uint8_t mode, uint8_t spi_port);//, uint8_t spi_remap);
	void set_SPI_to_Slave();
	void print(char const *s);
	unsigned char transfer_Byte(uint16_t Data);
	void writeByte(uint16_t Data);
	unsigned char readByte();
	int available();
//	void master_demo01();
//	void slave_demo01();

private:
};

#endif /* SPI_H_ */
