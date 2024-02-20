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

static TIM_DRIVER *timer_us;

void delay_init(void) {
	TIM_DRIVER timer0(4, 1/0.002, timer_mode::timer_counter);
	timer_us = &timer0;
}
void delay_ms(uint32_t milliseconds) {
	HAL_Delay(milliseconds);		// stm32
	// sys_delay_ms(milliseconds);	// esp32
}
void delay_us(uint32_t microseconds) {
	timer_us->reset_cnt();
	while(timer_us->get_cnt() < 24*microseconds);

	// another way
	// if(tim1_state == false) {
	// 	tim1_state = true;
	// 	TIM_DRIVER timer_us(1, 1/0.002, timer_mode::timer_counter);
	// 	timer1 = &timer_us;
	// }

	// // dummy way;
	// uint32_t a;
	// for(uint32_t i=0; i<microseconds	; i++) {
	// 	a++;
	// }
}
