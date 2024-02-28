#ifndef __TEST_FUNCTIONS_HPP__
#define __TEST_FUNCTIONS_HPP__

#include "system_main.h"		// stm32f103 includes
// includes test functions (aht10 sensor..)
// #include "i2c_master.hpp"
#include "aht10.hpp"
// #include "gpio.hpp"
#include "tim.h"
#include "tim_driver.hpp"

// includes for pcy8575
#include "pcy8575.hpp"

// For test purpose functions
#include "adc_driver.hpp"
#include "iwdg.h"
#include "backup.hpp"
#include "reset_reason.hpp"

#include "delay.hpp"

// lcd character
#include "lcd.h"

// For weighing scale system only
#include "weighing_scale.h"

// time test
#include <time.h>
#include <chrono>

void i2c_slave_pcy8575(void);
void weighing_scale(void);

void test_aht10(void);

void i2c_slave_pcy8575_adc(void);
void i2c_slave_pcy8575_orig(void);
void test_adc_class(void);
void test_adc_dma(void);
void test_adc_dma_tim_class(void);
void test_adc_oneshot(void);
void test_adc_stream(void);
void test_adc_stream_reg(void);
void test_gpio(void);
void test_bkp(void);
void test_lcd(void);
void test_pjb20(void);
void test_pwm(void);
void test_time(void);
void test_timer_counter(void);
void test_timer_interrupt(void);
void test_timer_pwm(void);

void test_lcd_aht10_pwm(void);

#endif