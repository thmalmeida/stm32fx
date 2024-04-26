#ifndef __TEST_FUNCTIONS_HPP__
#define __TEST_FUNCTIONS_HPP__

#include "system_main.h"		// stm32f103 includes


// sensors
#include "aht10.hpp"
// #include "ahtx0.hpp"
// #include "bmp180.hpp"
// #include "bmp280.hpp"

#include "tim.h"
#include "tim_driver.hpp"

// displays
#include "lcd.hpp"              // lcd character
#include "ssd1306.hpp"          // OLED display

// includes for pcy8575
#include "pcy8575.hpp"

// For test purpose functions
#include "adc_driver.hpp"
#include "iwdg.h"
#include "backup.hpp"
#include "reset_reason.hpp"

#include "delay.hpp"

// For weighing scale system only
#include "weighing_scale.hpp"

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
void test_i2c(void);
void test_lcd(void);
void test_pjb20(void);
void test_pwm(void);
void test_ssd1306(void);
void test_time(void);
void test_timer_counter(void);
void test_timer_interrupt(void);
void test_timer_pwm(void);

void test_lcd_aht10_pwm(void);

#endif