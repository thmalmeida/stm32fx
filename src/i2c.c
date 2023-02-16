/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

I2C_HandleTypeDef hi2c2;
volatile int i2c_it_flag = 0;
uint8_t data_rx[10] = {0};
uint8_t data_tx[10] = {0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e};
volatile uint8_t transferDirection, transferRequested;

// CR2 reg
uint16_t CR2_reg;
uint8_t ITBUFEN_bit;	// bit 10
uint8_t ITEVTEN_bit;	// bit 9
uint8_t ITERREN_bit;	// bit 8
uint8_t FREQ;

// OAR1 reg
uint16_t OAR1_reg;
uint8_t addr1, addr2;
uint8_t OAR1_14;

// OAR2 reg
uint16_t OAR2_reg;
uint8_t ENDUAL_bit;

//CR1 reg
uint16_t CR1_reg;
uint8_t PEC_bit;			// bit 12
uint8_t POS_bit;			// bit 11
uint8_t ACK_bit;			// bit 10
uint8_t STOP_bit;			// bit 9
uint8_t NOSTRETCH_bit;		// bit 7
uint8_t ENGC_bit;			// bit 6

// SR1 reg
uint16_t SR1_reg = 0;
uint8_t SMB_bit = 0;		// bit 15
uint8_t TIMEOUT_bit = 0;	// bit 14
uint8_t PEC_bit = 0;		// bit 12
uint8_t OVR_bit = 0;		// bit 11
uint8_t AF_bit = 0;			// bit 10
uint8_t ARLO_bit = 0;		// bit 9
uint8_t BERR_bit = 0;		// bit 8
uint8_t TxE_bit = 0;		// bit 7
uint8_t RxNE_bit = 0;		// bit 6
uint8_t STOPF_bit = 0;		// bit 4
uint8_t ADDR10_bit = 0;		// bit 3
uint8_t BTF_bit = 0;		// bit 2
uint8_t ADDR_bit = 0;		// bit 1
uint8_t SB_bit = 0;			// bit 0

// SR2 reg (read only)
uint16_t SR2_reg = 0;
uint8_t TRA_bit;			// bit 2 - TRA:0 write, TRA:1 read request
uint8_t BUSY_bit;			// bit 1
uint8_t MSL_bit;			// bit 0

// state machine variables


void i2c_init(uint32_t addr1, uint32_t freq) {
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = freq;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = addr1;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		printf("Error I2C init!\n");
		Error_Handler();
	}

	// // Enable I2C peripheral
	// I2C2->CR1 |= (1 << 0);
}
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    // /* I2C2 interrupt Init */
    // HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

    // HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
	}
}
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
	if(i2cHandle->Instance==I2C2)
	{
		/* USER CODE BEGIN I2C2_MspDeInit 0 */

		/* USER CODE END I2C2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C2_CLK_DISABLE();

		/**I2C2 GPIO Configuration
		PB10     ------> I2C2_SCL
		PB11     ------> I2C2_SDA
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

		/* I2C2 interrupt Deinit */
		HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
		HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
		/* USER CODE BEGIN I2C2_MspDeInit 1 */

		/* USER CODE END I2C2_MspDeInit 1 */
	}
}
void i2c_read_SR1_reg(void) {
	// Read SR1 reg bits
	SR1_reg = I2C2->SR1;

	SMB_bit	= (SR1_reg >> 15) & 0x01;		// int err
	TIMEOUT_bit = (SR1_reg >> 14) & 0x01;	// int err
	PEC_bit = (SR1_reg >> 12) & 0x01;		// int err
	OVR_bit = (SR1_reg >> 11) & 0x01;		// int err
	AF_bit = (SR1_reg >> 10) & 0x01;		// int err
	ARLO_bit = (SR1_reg >> 9) & 0x01;		// int err
	BERR_bit = (SR1_reg >> 8) & 0x01;		// int err
	TxE_bit = (SR1_reg >> 7) & 0x01;		// int ev
	RxNE_bit = (SR1_reg >> 6) & 0x01;		// int ev
	STOPF_bit = (SR1_reg >> 4) & 0x01;		// int ev
	ADDR10_bit = (SR1_reg >> 3) & 0x01;		// int ev
	BTF_bit = (SR1_reg >> 2) & 0x01;		// int ev: byte transfer finish
	ADDR_bit = (SR1_reg >> 1) & 0x01;		// int ev: address match
	SB_bit = (SR1_reg >> 0) & 0x01;			// int ev: start bit (master only?)
}
void i2c_print_SR1_reg(void) {
	printf("SR1- TIMEOUT:%d OVR:%d AF:%d ARLO:%d BERR:%d TxE:%d RxNE:%d STOPF:%d BTF:%d ADDR10:%d ADDR:%d SB:%d\n", TIMEOUT_bit, OVR_bit, AF_bit, ARLO_bit, BERR_bit, TxE_bit, RxNE_bit, STOPF_bit, BTF_bit, ADDR10_bit, ADDR_bit, SB_bit);
}
void i2c_read_SR2_reg(void) {
	SR2_reg = I2C2->SR2;
	TRA_bit = (SR2_reg >> 2) & 0x01;
	BUSY_bit = (SR2_reg >> 1) & 0x01;
	MSL_bit = (SR2_reg >> 0) & 0x01;
}
void i2c_print_SR2_reg(void) {
	printf("SR2- I2C2->SR2: 0x%04x, TRA:%d, BUSY:%d, MSL:%d\n", SR2_reg, TRA_bit, BUSY_bit, MSL_bit);
}
void i2c_read_CR1_reg(void) {
	// CR1 reg
	CR1_reg = I2C2->CR1;

	PEC_bit = (CR1_reg >> 12) & 0x01;
	POS_bit = (CR1_reg >> 11) & 0x01;
	ACK_bit = (CR1_reg >> 10) & 0x01;
	STOP_bit = (CR1_reg >> 9) & 0x01;
	NOSTRETCH_bit = (CR1_reg >> 7) & 0x01;
	ENGC_bit = (CR1_reg >> 6) & 0x01;
}
void i2c_print_CR1_reg(void) {
	printf("CR1- PEC: %d, POS: %d, ACK: %d, STOPF:%d, NOSTRETCH: %d, ENGC: %d\n", PEC_bit, POS_bit, ACK_bit, STOPF_bit, NOSTRETCH_bit, ENGC_bit);
}
void i2c_read_CR2_reg(void) {
	// CR2 reg
	CR2_reg = I2C2->CR2;

	ITBUFEN_bit = (CR2_reg >> 10) & 0x01;	// bit 10
	ITEVTEN_bit = (CR2_reg >> 9) & 0x01;	// bit 9
	ITERREN_bit = (CR2_reg >> 8) & 0x01;	// bit 8
	FREQ = I2C2->CR2 & 0x003F;				// bit [5:0]
}
void i2c_print_CR2_reg(void) {
	printf("CR2- ITBUFEN:%d, ITEVTEN:%d, ITERREN:%d, FREQ:%d\n", ITBUFEN_bit, ITEVTEN_bit, ITERREN_bit, FREQ);
}
void i2c_read_OAR1_reg(void) {
	OAR1_reg = I2C2->OAR1;
	OAR1_14 = (OAR1_reg >> 14) & 0x01;
	addr1 = (OAR1_reg >> 1) & 0x7F;
}
void i2c_print_addr1(void) {
	printf("addr1: 0x%02x\n", addr1);
}
void i2c_read_OAR2(void) {
	OAR2_reg = I2C2->OAR2;

	addr2 = (OAR2_reg >> 1) & 0x7F;
	ENDUAL_bit = (OAR2_reg >> 0) & 0x01;
}
void i2c_set_ack(void) {
	// Set ACK bit
	I2C2->CR1 |= (1<<10);
}
void i2c_reset_ack(void) {
	// Reset ACK bit
	I2C2->CR1 &= ~(1<<10);
}
void i2c_set_engc(void) {
	// Enable general call bit into CR1;
	I2C2->CR1 |= (1<<6);
}
void i2c_set_nostretch(void) {
	// Set nostretch bit
	I2C2->CR1 |= (1 << 7);
}
void i2c_enable_interrupt(void) {

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

	// enable listen
	HAL_I2C_EnableListen_IT(&hi2c2);
	I2C2->CR2 |= (1 << 10);	// ITBUFEN
	// I2C2->CR2 |= (1 << 9);	// ITEVTEN
	// I2C2->CR2 |= (1 << 8);	// ITERREN
}
void i2c_interrupt_handle(void) {
	// Slave transmitter
	// EV1: ADDR=1, cleared by reading SR1 followed by reading SR2
	// EV3-1: TxE=1, shift register empty, data register empty, write Data1 in DR.
	// EV3: TxE=1, shift register not empty, data register empty, cleared by writing DR
	// EV3-2: AF=1; AF is cleared by writing ‘0’ in AF bit of SR1 register.

	// Slave receiver
	// EV1: ADDR=1, cleared by reading SR1 followed by reading SR2
	// EV2: RxNE=1 cleared by reading DR register.
	// EV4: STOPF=1, cleared by reading SR1 register followed by writing to the CR1 register

	i2c_read_SR1_reg();

	int n = 0, timeout = 100000;
	if(ADDR_bit || RxNE_bit || TxE_bit) {

		// EV1: ADDR=1, cleared by reading SR1 followed by reading SR2 (Slave Transmitter/Receiver)
		i2c_read_SR2_reg();

		if(TRA_bit) {	// Slave transmit - Master read
		// EV3-1: TxE=1, shift register empty, data register empty, write Data1 in DR.
		// EV3: TxE=1, shift register not empty, data register empty, cleared by writing DR
		// EV3-2: AF=1; AF is cleared by writing ‘0’ in AF bit of SR1 register.
			do {
				do {
					i2c_read_SR1_reg();

					timeout--;
					if(!timeout) {
						printf("Error timeout!\n");
						i2c_print_SR1_reg();
						i2c_read_SR2_reg();
						i2c_print_SR2_reg();
						i2c_read_CR1_reg();
						i2c_print_CR1_reg();
						i2c_read_CR2_reg();
						i2c_print_CR2_reg();
						return;
					}
				} while(!TxE_bit && !AF_bit);

				if(!AF_bit) {
					I2C2->DR = data_tx[n];
					n++;
				}

			} while(!AF_bit);

			// It will clean on ER handle
			if(AF_bit) {
			// Clear AF bit into SR1
				I2C2->SR1 &= ~(1<<10);
			}

			printf("n: %d\n", n);
			for(int i=0; i<n-1; i++) {
				// printf("data_tx[%d]: 0x%02x\n", i, data_tx[i]);
				data_tx[i]++;
			}
		}
		else {			// Slave receive - Master transmit
			// Must implement flag timeout!
			do { // EV4: STOPF=1, cleared by reading SR1 register followed by writing to the CR1 register
				do { // EV2: RxNE=1 cleared by reading DR register.
					i2c_read_SR1_reg();

					// timeout--;
					// if(!timeout) {
					// 	return;
					// 	printf("Error timeout!\n");
					// }
				} while(!RxNE_bit && !STOPF_bit);

				if(!STOPF_bit) {
					data_rx[n] = I2C2->DR;
					//Read data from DR: *data++ = I2C2->DR;  defined as char* data;
					n++;
				}
			} while(!STOPF_bit);

			// Clear STOPF bit: read SR1 following write CR1
			// Set ACK bit
			I2C2->CR1 |= (1<<10);

			// for(int i=0; i<n-1; i++) {
			// 	printf("data_rx[%d]: 0x%02x\n", i, data_rx[i]);
			// }
		}
	}
	/* Debug prints */
	// printf("n: %d\n", n);
	// i2c_print_SR1_reg();

	// i2c_print_SR2_reg();	
	// i2c_read_SR2_reg();
	// i2c_print_SR2_reg();

	// i2c_read_SR1_reg();
	// i2c_print_SR1_reg();
}