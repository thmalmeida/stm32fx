#include "system_main.h"		// stm32f103 includes
#include "test_functions.hpp"

int main(void)
{
	/* System must initializes */
	HAL_Init();								// Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Delay(10);							// little delay after peripheral reset;
	SystemClock_Config_48MHz_HSE_ADC();		// Configure the system clock
	// SystemClock_Config_8MHZ_HSI();
	// printf("\nSystem reset...\n");			// Beautiful welcome message;

	/* your sample code */
	// i2c_slave_pcy8575();
	// test_adc_dma();
	// test_adc_dma_tim_class();
	// test_adc_oneshot();
	// test_adc_stream();
	// test_adc_stream_re	g();
	// test_timer_pwm();
	// test_pjb20();
	test_aht10();
	// test_bkp();
	// test_pwm();
	// test_time();

    return 0;
}
