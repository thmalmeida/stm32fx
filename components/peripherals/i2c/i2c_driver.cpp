#include "i2c_driver.hpp"

I2C_Driver::I2C_Driver(int port, uint32_t freq) : i2c_master_port_(port), freq_(freq) {
	init();
}
I2C_Driver::~I2C_Driver(void) {
	deinit();
}

void I2C_Driver::init(void) {
	// Initialize the I2C low level resources by implementing the @ref HAL_I2C_MspInit()

	// I2C_HandleTypeDef* i2cHandle;
	// i2cHandle = &hi2c_;
	if(i2c_master_port_ == 1) {
		#ifdef I2C_DEBUG
		printf("I2C: select I2C1, fbus: %lu Hz\n", freq_);
		#endif
		hi2c_.Instance = I2C1;
	} else {
		#ifdef I2C_DEBUG
		printf("I2C: select I2C2, fbus: %lu Hz\n", freq_);
		#endif
		hi2c_.Instance = I2C2;
	}

	hi2c_.Init.ClockSpeed = freq_;
	hi2c_.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c_.Init.OwnAddress1 = 0x23;
	// hi2c_.Init.OwnAddress1 = 0;
	hi2c_.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	// hi2c_.Init.OwnAddress2 = 0;
	hi2c_.Init.OwnAddress2 = 0x24;
	hi2c_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c_.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	GPIO_InitTypeDef GPIO_InitStruct;
	if(hi2c_.Instance == I2C1) {
		#ifdef I2C_DEBUG
		printf("I2C: enable GPIOB clk PB6 and PB7\n");
		#endif
		// GPIOB clock enable	
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C1 GPIO Configuration - without remap
		PB6     ------> I2C1_SCL
		PB7     ------> I2C1_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* I2C1 clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();

	} else if(hi2c_.Instance == I2C2) {
		#ifdef I2C_DEBUG
		printf("I2C: enable GPIOB clk PB10 and PB11\n");
		#endif
		// GPIOB clock enable	
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/* I2C2 GPIO Configuration - without remap
		PB10     ------> I2C1_SCL
		PB11     ------> I2C1_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* I2C2 clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();
		// reset();

		// This is also required before configuring the I2C peripheral in STM32F1xx devices

		/* I2C1 interrupt Init */
		// HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
		// HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

		// HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
		// HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
	}

	if (HAL_I2C_Init(&hi2c_) != HAL_OK) {
		printf("I2C init error\n");
		Error_Handler();
	}
	else {
		printf("I2C init done\n");
	}
	// I2C1->CR1 |=(1<<15);
}
void I2C_Driver::deinit(void) {

	if(hi2c_.Instance == I2C1) {
		__HAL_RCC_I2C1_CLK_DISABLE();
		/**I2C1 GPIO Configuration
		PB6     ------> I2C2_SCL
		PB7     ------> I2C2_SDA
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

	} else if(hi2c_.Instance == I2C2) {
		__HAL_RCC_I2C2_CLK_DISABLE();
		/**I2C2 GPIO Configuration
		PB10     ------> I2C2_SCL
		PB11     ------> I2C2_SDA
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
	}
}

i2c_ans I2C_Driver::write(uint8_t slave_addr, uint8_t reg) {
	uint8_t data = 0x00;
	return write(slave_addr, reg, &data, 0);
}
i2c_ans I2C_Driver::write(uint8_t slave_addr, uint8_t reg, uint8_t data) {
	return write(slave_addr, reg, &data, 1);
}
i2c_ans I2C_Driver::write(uint8_t slave_addr, uint8_t reg, uint8_t *data, size_t len) {
	if(HAL_I2C_Mem_Write(&hi2c_, static_cast<uint16_t>(slave_addr << 1) | I2C_MASTER_WRITE, reg, 1, data, len, I2C_COMMAND_WAIT_MS) != HAL_OK) {
		#ifdef I2C_DEBUG
		printf("Err i2c write Mem\n");
		#endif
		return i2c_ans::error_write;
	} else {
		return i2c_ans::ok;
	}
}
i2c_ans I2C_Driver::write(uint8_t slave_addr, uint8_t *data, size_t len) {
	
	// if(HAL_I2C_Mem_Write(&hi2c_, (slave_addr << 1) | I2C_MASTER_WRITE, reg, 1, data_tx, len - 1, I2C_COMMAND_WAIT_MS) == HAL_OK) {

	// HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c_, (slave_addr << 1) | I2C_MASTER_WRITE, data, len, I2C_COMMAND_WAIT_MS);
	if(HAL_I2C_Master_Transmit(&hi2c_, (slave_addr << 1) | I2C_MASTER_WRITE, data, len, I2C_COMMAND_WAIT_MS) != HAL_OK) {
		#ifdef I2C_DEBUG
		printf("Err i2c write Trans\n");
		#endif
		return i2c_ans::error_write;
	} else {
		return i2c_ans::ok;
	}
}

i2c_ans I2C_Driver::read(uint8_t slave_addr, uint8_t *data) {
	return read(slave_addr, data, 1);
}
i2c_ans I2C_Driver::read(uint8_t slave_addr, uint8_t *data, size_t len) {
	if(HAL_I2C_Master_Receive(&hi2c_, (slave_addr << 1) | I2C_MASTER_READ, data, len, I2C_COMMAND_WAIT_MS) != HAL_OK) {
		#ifdef I2C_DEBUG
		printf("Err i2c read only\n");
		#endif
		return i2c_ans::error_read;
	}
	else {
		return i2c_ans::ok;
	}	
}
i2c_ans I2C_Driver::read(uint8_t slave_addr, uint8_t reg, uint8_t *data) {
	return read(slave_addr, reg, data, 1);
}
i2c_ans I2C_Driver::read(uint8_t slave_addr, uint8_t reg, uint8_t *data, size_t len) {
	return read(slave_addr, &reg, 1, data, len);
}
i2c_ans I2C_Driver::read(uint8_t slave_addr, const uint8_t *write_buffer, size_t write_buffer_len, uint8_t *read_buffer, size_t read_buffer_len) {
	if(write_buffer_len == 1) {
		// receive read
		uint8_t reg = write_buffer[0];
		if(HAL_I2C_Mem_Read(&hi2c_, (slave_addr << 1) | I2C_MASTER_WRITE, reg, 1, read_buffer, read_buffer_len, I2C_COMMAND_WAIT_MS) != HAL_OK) {
			#ifdef I2C_DEBUG
			printf("Err i2c read receive 1\n");
			#endif
			return i2c_ans::error_read;
		}
	} else {
		// transmit write first
		if(HAL_I2C_Master_Transmit(&hi2c_, static_cast<uint16_t>(slave_addr << 1) | I2C_MASTER_WRITE, (uint8_t*) write_buffer, write_buffer_len, I2C_COMMAND_WAIT_MS) != HAL_OK) {
		// if(HAL_I2C_Mem_Write(&hi2c_, (slave_addr << 1) | I2C_MASTER_WRITE, reg, 1, write_buffer, write_buffer_len, I2C_COMMAND_WAIT_MS) != HAL_OK) {
			#ifdef I2C_DEBUG
			printf("Err i2c read transmit 2\n");
			#endif
			return i2c_ans::error_write;
		}
		// receive read
		if(HAL_I2C_Master_Receive(&hi2c_, (slave_addr << 1) | I2C_MASTER_READ, read_buffer, read_buffer_len, I2C_COMMAND_WAIT_MS) != HAL_OK) {
		// if(HAL_I2C_Mem_Read(&hi2c_, (slave_addr << 1) | I2C_MASTER_WRITE, reg, 1, read_buffer, read_buffer_len, I2C_COMMAND_WAIT_MS) != HAL_OK) {
			#ifdef I2C_DEBUG
			printf("Err i2c read receive 2\n");
			#endif
			return i2c_ans::error_read;
		}
	}

	return i2c_ans::ok;
}

bool I2C_Driver::probe(uint8_t addr) {
	if(HAL_I2C_IsDeviceReady(&hi2c_, addr << 1, 2, I2C_COMMAND_WAIT_MS) == HAL_OK)
		return true;
	else
		return false;

	// // old way
	// uint8_t data;
	// if(read(addr, 0x00, &data) == i2c_ans::ok)
	// 	return true;
	// return false;
}
uint8_t I2C_Driver::probe_find(uint8_t addr_init /* = 0 */) {

	for(uint8_t i = addr_init; i < 128; i++) {
		if(HAL_I2C_IsDeviceReady(&hi2c_, i << 1, 1, I2C_COMMAND_WAIT_MS) == HAL_OK) {
			// printf("addr: 0x%02x\n", i);
			return i;
		}
	}

	// Old way
	// uint8_t data;
	// for(uint8_t i = addr_init; i < 128; i++) {
	// 	if(read(i, 0x00, &data) == i2c_ans::ok) {
	// 		printf("addr: 0x%02x\n", i);
	// 		return i;
	// 	}
	// }

	return 0xFF;
}
void I2C_Driver::probe_list(void) {
	for(uint8_t i = 0; i < 128; i++)
		if(HAL_I2C_IsDeviceReady(&hi2c_, i << 1, 1, I2C_COMMAND_WAIT_MS) == HAL_OK)
			printf("addr: 0x%02x\n", i);
}

// STM32 debug purpose
void I2C_Driver::log_show(void) {
	state_ = HAL_I2C_GetState(&hi2c_);
	error_ = HAL_I2C_GetError(&hi2c_);
	mode_ = HAL_I2C_GetMode(&hi2c_);

	printf("i2c-- state: 0x%02x, mode: 0x%02x, error: 0x%04lx\n", static_cast<uint8_t>(state_), static_cast<uint8_t>(mode_), error_);
}

/*
 *	Slave member functions
 */
void I2C_Driver::listen_enable(void) {
	printf("I2C: enabling listen...   ");
	if(HAL_I2C_EnableListen_IT(&hi2c_) != HAL_OK)
	{
		/* Transfer error in reception process */
		// Error_Handler();
		printf("Error!\n");
 	}
	else {
		printf("Done!\n");
	}
}
void I2C_Driver::listen(void) {

	// if (Xfer_Complete ==1)
	// {
	// 	HAL_Delay(1);
		/*##- Put I2C peripheral in listen mode process ###########################*/
		// Xfer_Complete =0;
	// }


	if(__HAL_I2C_GET_FLAG(&hi2c_, I2C_FLAG_ADDR) == FlagStatus::SET) {

		// printf("Direction: %d\n", __HAL_I2C_GET_FLAG(&hi2c_, I2C_FLAG_TRA));

		// uint8_t data_rx[2];
		// data_rx[0] = 0x00;
		// data_rx[1] = 0x03;
		// HAL_I2C_Slave_Receive(&hi2c_, &data_rx[0], 2, 1000);
		// printf("Send back\n");
		// (HAL_I2C_Slave_Seq_Transmit_IT(&i2c.hi2c_, (uint8_t *)aTxBuffer, TXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
		// HAL_I2C_Slave_Transmit(&hi2c_, &data_rx[0], 1, 1000);
		// __HAL_I2C_CLEAR_ADDRFLAG(&hi2c_);
		// listen_enable();
		// listen_enable();
		// HAL_Delay(2000);
	}
	// listen_enable();
}

// ?
// i2c_ans I2C_Driver::set_mask(uint8_t slave_addr, uint8_t reg, uint8_t data, uint8_t mask, bool ack_check /* = true */) {
// 	uint8_t content;

// 	int ret = read(slave_addr, reg, &content, ack_check);
// 	if(ret < 0){
// 		return ret;
// 	}

// 	content &= ~mask;
// 	content |= data & mask;

// 	return write(slave_addr, reg, content, ack_check);
// }
	// HHAL_I2C_AddrCallback
	// HAL_I2C_AddrCallback()
	// uint8_t data = 0x00;
	// HAL_I2C_Slave_Transmit(&hi2c_, &data, 1, 1000);
		// HAL_I2C_Slave_Receive(&hi2c_, (uint8_t *) data_rx, 12, 10000);
	// while(__HAL_I2C_GET_FLAG(&hi2c_, I2C_FLAG_ADDR) == FlagStatus::RESET)
	// {
		
	// }
	// else
	// {
	// 	printf("nothing yet\n");
	// }

	// char data_rx[20];