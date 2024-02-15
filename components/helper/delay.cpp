#include "delay.hpp"

uint8_t tim1_state = 0;

// double T_us = 0.000001;
// TIM_DRIVER timer_us(4, 0.001, timer_mode::timer_counter);


// #define SYSTICK_LOAD (SystemCoreClock/1000000U)
// #define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
 
// #define DELAY_US(us) \
//     do { \
//          uint32_t start = SysTick->VAL; \
//          uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
//          while((start - SysTick->VAL) < ticks); \
//     } while (0)
// #define DELAY_MS(ms) \
//     do { \
//         for (uint32_t i = 0; i < ms; ++i) { \
//             DELAY_US(1000); \
//         } \
//     } while (0)

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


	// timer_us.enable_cnt();
	// timer_us.reset_cnt();
	uint32_t count = 0;

	// while(count < microseconds) {
	// 	if(timer_us.isr_flag()){
	// 		count++;
	// 	}
	// }

	// while(timer_us.get_cnt() < microseconds);
		// printf("v:%lu\n", timer_1us.get_cnt());
	// }
	// timer_us.disable_cnt();

	// __HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	// while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter


	// Old way;
	uint32_t a;
	for(uint32_t i=0; i<microseconds	; i++) {
		a++;
	}
	// ets_delay_us(microseconds);
}
