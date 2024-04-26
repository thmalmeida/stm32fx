#include "system_main.h"		// stm32f103 includes
#include "setup.hpp"

int main(void)
{
	/* MCU system configuration - Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	SystemClock_Config_72MHz_HSE_LSI_ADC();
	// SystemClock_Config_48MHz_HSE_ADC();

	/* delay_us() and delay_ms() init using TIM4 */
	delay_init();


	/* 1- I2C to gpio main code. It works with Acionna0 */
	// i2c_slave_pcy8575();


	/* 2- Weighing Scale System */
	// weighing_scale();


	/* 3- Test functions */
	// test_adc_dma();
	// test_adc_dma_tim_class();
	// test_adc_oneshot();
	// test_adc_stream();
	// test_adc_stream_reg();
	// test_gpio();
	// test_i2c();
	// test_lcd();
	// test_timer_interrupt();
	// test_timer_counter();
	// test_timer_pwm();
	// test_pjb20();
	// test_aht10();
	test_ssd1306();
	// test_i2c();
	// test_bkp();
	// test_pwm();
	// test_time();
	// test_lcd_aht10_pwm();
	
    return 0;
}
