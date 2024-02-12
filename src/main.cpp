#include "system_main.h"		// stm32f103 includes
#include "test_functions.hpp"

int main(void)
{
	/* System must initializes */
	HAL_Init();								// Reset of all peripherals, Initializes the Flash interface and the Systick. */
	SystemClock_Config_48MHz_HSE_ADC();		// Configure the system clock
	HAL_Delay(100);							// little delay after peripheral reset;
	// SystemClock_Config_8MHz_HSI();
	// printf("\nSystem reset...\n");		// Beautiful welcome message;

	/* I2C to gpio main code. It works with Acionna0 */
	// i2c_slave_pcy8575();

	/* Weighing Scale System */
	// weighing_scale();

	/* Test functions */
	// test_adc_dma();
	// test_adc_dma_tim_class();
	// test_adc_oneshot();
	// test_adc_stream();
	// test_adc_stream_re	g();
	// test_gpio();
	// printf("TESTE00!\n");
	// test_timer_counter();
	// test_timer_pwm();
	// test_pjb20();
	test_aht10();
	// test_bkp();
	// test_pwm();
	// test_time();
	// timer_calculation(10);
    return 0;
}
