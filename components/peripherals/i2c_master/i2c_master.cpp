#include "i2c_master.hpp"

//#define I2C_WRITE_FLAG	0
//#define I2C_READ_FLAG		1

// static const char *TAG_I2C = "I2C";

// I2C_Master::I2C_Master(int port, int scl, int sda, uint32_t freq, bool pull_up /* = false */) : i2c_master_port_(port) {
	// // Configuration
	// i2c_config_t conf = {};
	// conf.mode = I2C_MODE_MASTER;
	// conf.sda_io_num = sda;
	// conf.sda_pullup_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	// conf.scl_io_num = scl;
	// conf.scl_pullup_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	// conf.master.clk_speed = freq; // I2C_FAST_SPEED_HZ or I2C_NORMAL_SPEED_HZ;

	// esp_err_t err = i2c_param_config(i2c_master_port_, &conf);
	// if(err != ESP_OK) {
	// 	printf("I2C config error [%d/%s]\n", err, esp_err_to_name(err));
	// }

	// // Install driver
	// int intr_alloc_flags = 0;
	// err = i2c_driver_install(i2c_master_port_, conf.mode, 0, 0, intr_alloc_flags);

	// if(err != ESP_OK) { 
	// 	ESP_LOGE(TAG_I2C, "I2C install error [%d/%s]\n", err, esp_err_to_name(err));
	// }
// }
void I2C_Master::init(uint32_t freq) {
	freq_ = freq;
	i2c_master_port_ = 1;
	// Initialize the I2C low level resources by implementing the @ref HAL_I2C_MspInit()

	// I2C_HandleTypeDef* i2cHandle;
	// i2cHandle = &hi2c2_;

	// init()
	hi2c2_.Instance = I2C2;
	hi2c2_.Init.ClockSpeed = freq_;
	hi2c2_.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2_.Init.OwnAddress1 = 0x23;
	hi2c2_.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2_.Init.OwnAddress2 = 0x24;
	hi2c2_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2_.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	GPIO_InitTypeDef GPIO_InitStruct;
	// if(i2cHandle->Instance==I2C2)
	// {
		/* I2C1 GPIO Configuration
		PB10     ------> I2C1_SCL
		PB11     ------> I2C1_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
		// __HAL_RCC_GPIOB_CLK_ENABLE();
		/* I2C2 clock enable */

		__HAL_RCC_I2C2_CLK_ENABLE();
		reset();

		// This is also required before configuring the I2C peripheral in STM32F1xx devices

		/* I2C1 interrupt Init */
		// HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
		// HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

		// HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
		// HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
	// }

	printf("I2C: initializing...   ");
	if (HAL_I2C_Init(&hi2c2_) != HAL_OK)
	{
		printf("Error!\n");
		// Error_Handler();
	}
	else
	{
		printf("Done!\n");
	}
	// I2C1->CR1 |=(1<<15);
}
void I2C_Master::deinit(void) {

	// if(hi2c2_.Instance == I2C2)
	{
		__HAL_RCC_I2C2_CLK_DISABLE();
		/**I2C2 GPIO Configuration
		PB10     ------> I2C2_SCL
		PB11     ------> I2C2_SDA
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
	}
}
void I2C_Master::log_show(void) {
	state_ = HAL_I2C_GetState(&hi2c2_);
	error_ = HAL_I2C_GetError(&hi2c2_);
	mode_ = HAL_I2C_GetMode(&hi2c2_);

	printf("i2c-- state: 0x%02x, mode: 0x%02x, error: 0x%04lx\n", static_cast<uint8_t>(state_), static_cast<uint8_t>(mode_), error_);
}
int I2C_Master::write(uint8_t slave_addr, uint8_t reg, uint8_t data, bool ack_check /* = true */){
	return write(slave_addr, reg, &data, 1, ack_check);
}
int I2C_Master::write(uint8_t slave_addr, uint8_t reg, bool ack_check /* = true */){
	uint8_t data = 0x00;
	return write(slave_addr, reg, &data, 0, ack_check);
}
int I2C_Master::write(uint8_t slave_addr, uint8_t reg, uint8_t* data, size_t len, bool ack_check = true) {
	// ESP32
	// esp_err_t ret;
	// i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
	// i2c_master_start(cmd_handle);
	// i2c_master_write_byte(cmd_handle, slave_addr << 1 | I2C_MASTER_WRITE, ack_check);
	// i2c_master_write_byte(cmd_handle, reg, ack_check);
	// i2c_master_write(cmd_handle, data, len, ack_check);
	// i2c_master_stop(cmd_handle);
	// ret = i2c_master_cmd_begin(i2c_master_port_, cmd_handle, I2C_COMMAND_WAIT_MS / portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd_handle);
	// if (ret != ESP_OK) {
	// 	return I2C_ERR_WRITE;
	// }


	// STM32
	HAL_StatusTypeDef ret;

	// ret = HAL_I2C_Master_Transmit(&hi2c2_, static_cast<uint16_t>(slave_addr << 1 | I2C_MASTER_WRITE), data, len, I2C_COMMAND_WAIT_MS);
	ret = HAL_I2C_Mem_Write(&hi2c2_, (slave_addr << 1) | I2C_MASTER_WRITE, reg, 1, data, len, I2C_COMMAND_WAIT_MS);

	if(ret != HAL_OK) {
		printf("Err i2c write[%d]\n", static_cast<int>(ret));
		return I2C_ERR_OK;
	}

	return I2C_ERR_OK;
}
int I2C_Master::write(uint8_t slave_addr, uint8_t *data, uint8_t data_len) {
	
	// esp_err_t ret = i2c_master_write_to_device(i2c_master_port_, slave_addr, data, data_len, I2C_COMMAND_WAIT_MS / portTICK_PERIOD_MS);

	// if (ret != ESP_OK) {
	// 	printf("Error on write2");
	// 	return I2C_ERR_WRITE;
	// }

	// STM32
	printf("not implemented!\n");

	return I2C_ERR_OK;
}
int I2C_Master::set_mask(uint8_t slave_addr, uint8_t reg, uint8_t data, uint8_t mask, bool ack_check /* = true */) {
	uint8_t content;

	int ret = read(slave_addr, reg, &content, ack_check);
	if(ret < 0){
		return ret;
	}

	content &= ~mask;
	content |= data & mask;

	return write(slave_addr, reg, content, ack_check);
}
int I2C_Master::read(uint8_t slave_addr, uint8_t reg, uint8_t* data, size_t len, bool ack_check) {

	// ESP32
	// esp_err_t ret;
	// // Read: write to register first
	// i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
	// i2c_master_start(cmd_handle);
	// i2c_master_write_byte(cmd_handle, slave_addr << 1 | I2C_MASTER_WRITE, ack_check);
	// i2c_master_write_byte(cmd_handle, reg, ack_check);
	// i2c_master_stop(cmd_handle);
	// ret = i2c_master_cmd_begin(i2c_master_port_, cmd_handle, I2C_COMMAND_WAIT_MS / portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd_handle);
	// if (ret != ESP_OK) {
	// 	// printf("read: error on write: %s", esp_err_to_name(ret));
	// 	return I2C_ERR_WRITE;
	// }
	//
	// // Read: read start
	// cmd_handle = i2c_cmd_link_create();
	// i2c_master_start(cmd_handle);
	// i2c_master_write_byte(cmd_handle, slave_addr << 1 | I2C_MASTER_READ, ack_check);
	// if(len > 1) {
	// 	i2c_master_read(cmd_handle, data, len - 1, I2C_MASTER_ACK);
	// }
	// i2c_master_read_byte(cmd_handle, data + len - 1, I2C_MASTER_NACK);
	// i2c_master_stop(cmd_handle);
	// ret = i2c_master_cmd_begin(i2c_master_port_, cmd_handle, I2C_COMMAND_WAIT_MS / portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd_handle);
	// 
	// if(ret != ESP_OK) {
	// 	printf("read: error on read: %s", esp_err_to_name(ret));
	// 	return I2C_ERR_READ;
	// }

	// STM32
	HAL_StatusTypeDef ret;
	// transmit write first
	ret = HAL_I2C_Master_Transmit(&hi2c2_, (slave_addr << 1) | I2C_MASTER_WRITE, &reg, 1, I2C_COMMAND_WAIT_MS);
	// if(HAL_I2C_Mem_Write(&hi2c2_, (slave_addr << 1) | I2C_MASTER_WRITE, reg, 1, data, len, I2C_COMMAND_WAIT_MS) != HAL_OK)
	if(ret != HAL_OK) {
		printf("Err i2c read transmit[%d]\n", static_cast<int>(ret));
		return I2C_ERR_WRITE;
	}
	// receive read
	ret = HAL_I2C_Master_Receive(&hi2c2_, (slave_addr << 1) | I2C_MASTER_READ, data, len, I2C_COMMAND_WAIT_MS);
	if(ret != HAL_OK) {
		printf("Err i2c read receive[%d]\n", static_cast<int>(ret));
		return I2C_ERR_WRITE;
	}

	return I2C_ERR_OK;
}
int I2C_Master::read(uint8_t slave_addr, uint8_t reg, uint8_t* data, bool ack_check /* = true */) {
	return read(slave_addr, reg, data, 1, ack_check);
}
int I2C_Master::read_only(uint8_t slave_addr, uint8_t* data, size_t len, bool ack_check) {
	// ESP32
	// // Read only: read start	
	// i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
	// i2c_master_start(cmd_handle);
	// i2c_master_write_byte(cmd_handle, slave_addr << 1 | I2C_MASTER_READ, ack_check);
	// if(data_len > 1) {
	// 	i2c_master_read(cmd_handle, data, data_len - 1, I2C_MASTER_ACK);
	// }
	// i2c_master_read_byte(cmd_handle, data + data_len - 1, I2C_MASTER_NACK);
	// i2c_master_stop(cmd_handle);
	// esp_err_t ret = i2c_master_cmd_begin(i2c_master_port_, cmd_handle, I2C_COMMAND_WAIT_MS / portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd_handle);

	// if(ret != ESP_OK) {
	// 	printf("read only: error on read: %s", esp_err_to_name(ret));
	// 	return I2C_ERR_READ;
	// }

	// STM32
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Receive(&hi2c2_, (slave_addr << 1) | I2C_MASTER_READ, data, len, I2C_COMMAND_WAIT_MS);
	if(ret != HAL_OK) {
		printf("Err i2c read only[%d]\n", static_cast<int>(ret));
		return I2C_ERR_READ;
	}

	return I2C_ERR_OK;
}
int I2C_Master::read(uint8_t slave_address, const uint8_t *write_buffer, size_t write_buffer_len, uint8_t *read_buffer, size_t read_buffer_len) {

	// esp_err_t ret = i2c_master_write_read_device(i2c_master_port_, slave_address, write_buffer, write_buffer_len, read_buffer, read_buffer_len, I2C_COMMAND_WAIT_MS / portTICK_PERIOD_MS);

	// if(ret != ESP_OK) {
	// 	printf("read2: error on read: %s", esp_err_to_name(ret));
	// 	return I2C_ERR_READ;
	// }

	// STM32
	printf("not implemented!\n");

	return I2C_ERR_OK;
}
bool I2C_Master::probe(uint8_t addr) noexcept
{
	// HAL_I2C_IsDeviceReady
	uint8_t data;
	if(read(addr, 0x00, &data) > 0)
		return true;
	return false;
}
uint8_t I2C_Master::probe_addr(uint8_t addr_init /* = 0 */){
	uint8_t data;
	for(uint8_t i = addr_init; i < 127; i++){
		if(read(i, 0x00, &data) > 0){
			// return i;
			printf("addr: 0x%02x\n", i);
		}
		delay_ms(10);
	}
	return 0xFF;
}
bool I2C_Master::probe2(uint8_t addr) {

	if(HAL_I2C_IsDeviceReady(&hi2c2_, addr, 2, 1000) != HAL_OK) {
		printf("not found!\n");
		return false;
	}
	else {
		printf("Device ok!\n");
		return true;
	}
}

/*
 *	Slave member functions
 */
void I2C_Master::listen_enable(void) {
	printf("I2C: enabling listen...   ");
	if(HAL_I2C_EnableListen_IT(&hi2c2_) != HAL_OK)
	{
		/* Transfer error in reception process */
		// Error_Handler();
		printf("Error!\n");
 	}
	else {
		printf("Done!\n");
	}
}
void I2C_Master::listen(void) {

	// if (Xfer_Complete ==1)
	// {
	// 	HAL_Delay(1);
		/*##- Put I2C peripheral in listen mode process ###########################*/
		// Xfer_Complete =0;
	// }


	if(__HAL_I2C_GET_FLAG(&hi2c2_, I2C_FLAG_ADDR) == FlagStatus::SET) {

		printf("Direction: %d\n", __HAL_I2C_GET_FLAG(&hi2c2_, I2C_FLAG_TRA));

		uint8_t data_rx[2];
		data_rx[0] = 0x00;
		data_rx[1] = 0x03;
		// HAL_I2C_Slave_Receive(&hi2c2_, &data_rx[0], 2, 1000);
		printf("Send back\n");
		// (HAL_I2C_Slave_Seq_Transmit_IT(&i2c.hi2c2_, (uint8_t *)aTxBuffer, TXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK)
		// HAL_I2C_Slave_Transmit(&hi2c2_, &data_rx[0], 1, 1000);
		// __HAL_I2C_CLEAR_ADDRFLAG(&hi2c2_);
		// listen_enable();
		// listen_enable();
		// HAL_Delay(2000);
	}
	// listen_enable();
}
	// HHAL_I2C_AddrCallback
	// HAL_I2C_AddrCallback()
	// uint8_t data = 0x00;
	// HAL_I2C_Slave_Transmit(&hi2c2_, &data, 1, 1000);
		// HAL_I2C_Slave_Receive(&hi2c2_, (uint8_t *) data_rx, 12, 10000);
	// while(__HAL_I2C_GET_FLAG(&hi2c2_, I2C_FLAG_ADDR) == FlagStatus::RESET)
	// {
		
	// }
	// else
	// {
	// 	printf("nothing yet\n");
	// }

	// char data_rx[20];
;