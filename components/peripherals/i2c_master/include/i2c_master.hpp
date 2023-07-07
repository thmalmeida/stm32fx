#ifndef I2C_MASTER_HPP__
#define I2C_MASTER_HPP__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "delay.hpp"


// STM32----------------------
#include "stm32_log.h"
#include "system_main.h"
// ---------------------------

// ESP32 ---------------------
// #include "esp_log.h"
// #include "driver/i2c.h"
// #include "driver/gpio.h"
// ---------------------------

#define I2C_ERR_OK			1
#define I2C_ERR_WRITE		-1
#define I2C_ERR_READ		-2
#define I2C_ERR_ACK			-3

#define I2C_MASTER_WRITE	0
#define I2C_MASTER_READ 	1

#define I2C_FAST_SPEED_HZ	400000
#define I2C_NORMAL_SPEED_HZ	100000

#ifdef CONFIG_I2C_COMMAND_WAIT
#define I2C_COMMAND_WAIT_MS	CONFIG_I2C_COMMAND_WAIT
#else
#define I2C_COMMAND_WAIT_MS	500
#endif /* CONFIG_I2C_COMMAND_WAIT */

class I2C_Master{
	public:
		// I2C_Master(int port, int scl, int sda, uint32_t freq, bool pull_up = false);
		I2C_Master(int port, uint32_t freq) : i2c_master_port_(port), freq_(freq)  {};
		I2C_Master(void) {};
		~I2C_Master() {};

		// void init(int intr_alloc_flags = 0);
		// void deinit();

		void init(uint32_t freq);
		void deinit(void);

		int write(uint8_t slave_addr, uint8_t reg, uint8_t* data, size_t len, bool ack_check = true);
		int write(uint8_t slave_addr, uint8_t reg, uint8_t data, bool ack_check = true);
		int write(uint8_t slave_addr, uint8_t reg, bool ack_check = true);

		int write(uint8_t slave_addr, uint8_t *data, uint8_t data_len);

		int set_mask(uint8_t slave_addr, uint8_t reg, uint8_t data, uint8_t mask, bool ack_check = true);

		int read(uint8_t slave_addr, uint8_t reg, uint8_t* data, size_t len, bool ack_check = true);
		int read(uint8_t slave_addr, uint8_t reg, uint8_t* data, bool ack_check = true);
		int read(uint8_t slave_address, const uint8_t *write_buffer, size_t write_buffer_len, uint8_t *read_buffer, size_t read_buffer_len);
		int read_only(uint8_t slave_addr, uint8_t* data, size_t data_len, bool ack_check = true);

		bool probe2(uint8_t addr);

		bool probe(uint8_t addr) noexcept;
		uint8_t probe_addr(uint8_t addr_init = 0);

		// Slave functions
		void listen_enable(void);
		void listen(void);
		void log_show(void);

		void reset(void) {
			// declared into stm32f101xb.h
			I2C1 -> CR1 |= I2C_CR1_SWRST;//(1<<15);
			I2C1 -> CR1 &= ~I2C_CR1_SWRST;//(1<<15);
			// or
			// __HAL_RCC_I2C2_FORCE_RESET();
			// __HAL_RCC_I2C2_RELEASE_RESET();
		}

		I2C_HandleTypeDef hi2c2_;

	protected:
		int	i2c_master_port_;

		HAL_I2C_StateTypeDef state_;
		HAL_I2C_ModeTypeDef mode_;
		uint32_t error_;
		uint32_t freq_;
};

#endif /* I2C_MASTER_HPP__ */
