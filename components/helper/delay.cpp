#include "delay.hpp"

static uint8_t tim1_state = 0;

// static TIM_DRIVER timer_1us(1, 500000, timer_mode::timer_counter);

void delay_ms(uint32_t milliseconds)
{
	HAL_Delay(milliseconds);
	// sys_delay_ms(milliseconds);
}
void delay_us(uint32_t microseconds) {
	// if(!tim1_state) {
	// 	tim1_state = 1;

	// 	// Timer 1, fclk = 1 MHz, non interrupt mode;

	// 	printf("DELAY TIM1 timer\n");
	// }

	// https://controllerstech.com/create-1-microsecond-delay-stm32/
	// TIM1->CNT = 0;
	// timer_1us.reset_cnt();

	// while(timer_1us.get_cnt() < microseconds);
	// 	printf("v:%lu\n", timer_1us.get_cnt());
	// }

	// __HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	// while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter


	// Old way;
	uint32_t a;
	for(uint32_t i=0; i<microseconds	; i++) {
		a++;
	}
	// ets_delay_us(microseconds);
}
