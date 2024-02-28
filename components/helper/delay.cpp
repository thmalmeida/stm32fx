#include "delay.hpp"

/*
* by thmalmeida;
* last updated: 20240219
*/

// static TIM_DRIVER timer_us(4, 0.001, timer_mode::timer_counter);
// #define SYSTICK_LOAD (SystemCoreClock/1000000U)
// #define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
// void delay_us(uint32_t us) {
//     do {
//          uint32_t start = SysTick->VAL; 
//          uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  
//          while((start - SysTick->VAL) < ticks); 
//     } while (0)
// }

// static TIM_DRIVER timer_us(3, 0.001, timer_mode::timer_counter);
// static bool tim1_state = false;

// static TIM_DRIVER *timer_us;

// static TIM_DRIVER timer_us(4, 1/0.002, timer_mode::timer_counter);

void delay_init(void) {
	TIM_DRIVER timer0(4, 1/0.002, timer_mode::timer_counter);
	// timer_us = &timer0;

	printf("delay timer init\n");
}
void delay_ms(uint32_t milliseconds) {
	HAL_Delay(milliseconds);		// stm32
	// sys_delay_ms(milliseconds);	// esp32
}
void delay_us(uint32_t microseconds) {
	// timer_us.reset_cnt();
	TIM4->CNT = 0;
	// printf("V:%lu, cnt:%lu\n", 24*microseconds, timer_us.get_cnt());
	// while(timer_us.get_cnt() < 24*microseconds) {
	while(TIM4->CNT <24*microseconds);
		// printf("cnt:%lu\n",TIM4->CNT);
		// if(c++ > 1000000) {
		// 	printf("FOUND!\n");
		// 	break;
		// }
	// }
	// another way
	// if(tim1_state == false) {
	// 	tim1_state = true;
	// 	TIM_DRIVER timer_us(1, 1/0.002, timer_mode::timer_counter);
	// 	timer1 = &timer_us;
	// }

	// // dummy way;
	// uint32_t a=0;
	// for(uint32_t i=0; i<10*microseconds; i++) {
	// 	a++;
	// }
}
