/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "system_main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c2;
extern volatile int i2c_transfer_direction;
extern volatile uint8_t transferDirection, transferRequested;
extern uint8_t i2c_data_rx[10];
// extern uint8_t i2c_data_tx[10];
extern uint8_t* i2c_data_tx;

// CR2 reg
extern uint16_t CR2_reg;
extern uint8_t ITBUFEN_bit;	// bit 10
extern uint8_t ITEVTEN_bit;	// bit 9
extern uint8_t ITERREN_bit;	// bit 8
extern uint8_t FREQ;

// OAR1 reg
extern uint16_t OAR1_reg;
extern uint8_t addr1, addr2;
extern uint8_t OAR1_14;

// OAR2 reg
extern uint16_t OAR2_reg;
extern uint8_t ENDUAL_bit;

//CR1 reg
extern uint16_t CR1_reg;
extern uint8_t PEC_bit;		  	// bit 12
extern uint8_t POS_bit;		  	// bit 11
extern uint8_t ACK_bit;		  	// bit 10
extern uint8_t STOP_bit;			// bit 9
extern uint8_t NOSTRETCH_bit;	// bit 7
extern uint8_t ENGC_bit;			// bit 6

// SR1 reg
extern uint16_t SR1_reg;
extern uint8_t SMB_bit;     // bit 15
extern uint8_t TIMEOUT_bit;	// bit 14
extern uint8_t PEC_bit;     // bit 12
extern uint8_t OVR_bit;	  	// bit 11
extern uint8_t AF_bit;			// bit 10
extern uint8_t ARLO_bit;		// bit 9
extern uint8_t BERR_bit;		// bit 8
extern uint8_t TxE_bit;	  	// bit 7
extern uint8_t RxNE_bit;		// bit 6
extern uint8_t STOPF_bit;		// bit 4
extern uint8_t ADDR10_bit;  // bit 3
extern uint8_t BTF_bit;	  	// bit 2
extern uint8_t ADDR_bit;		// bit 1
extern uint8_t SB_bit;      // bit 0

// SR2 reg (read only)
extern uint16_t SR2_reg;
extern uint8_t TRA_bit;		// bit 2 - TRA:0 write, TRA:1 read request
extern uint8_t BUSY_bit;	// bit 1
extern uint8_t MSL_bit;		// bit 0

// state machine variables
extern volatile uint8_t i2c_has_data_rx;
extern volatile uint8_t i2c_has_data_tx;
extern volatile uint16_t i2c_data_rx_size;
extern volatile uint16_t i2c_data_tx_size;

void i2c_init(uint32_t addr1, uint32_t freq);
void i2c_read_SR1_reg(void);
void i2c_print_SR1_reg(void);
void i2c_read_SR2_reg(void);
void i2c_print_SR2_reg(void);
void i2c_read_CR1_reg(void);
void i2c_print_CR1_reg(void);
void i2c_read_CR2_reg(void);
void i2c_print_CR2_reg(void);
void i2c_read_OAR1_reg(void);
void i2c_print_addr1(void);
void i2c_read_OAR2_reg(void);
void i2c_set_ack(void);
void i2c_reset_ack(void);
void i2c_set_nostretch(void);
void i2c_set_engc(void);
void i2c_enable_interrupt(void);
void i2c_interrupt_ev_handle(void);
void i2c_interrupt_er_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

