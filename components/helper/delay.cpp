#include "delay.hpp"

// static TIM_DRIVER timer_us(4, 0.001, timer_mode::timer_counter);
// #define SYSTICK_LOAD (SystemCoreClock/1000000U)
// #define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
 
// #define DELAY_US(us) 
//     do {
//          uint32_t start = SysTick->VAL; 
//          uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  
//          while((start - SysTick->VAL) < ticks); 
//     } while (0)
// #define DELAY_MS(ms) 
//     do { 
//         for (uint32_t i = 0; i < ms; ++i) { 
//             DELAY_US(1000); 
//         } 
//     } while (0)
// static TIM_DRIVER timer_us(3, 0.001, timer_mode::timer_counter);

static TIM_DRIVER *timer_us;
// static bool tim1_state = false;

void delay_init(void) {
	TIM_DRIVER timer0(4, 1/0.002, timer_mode::timer_counter);
	timer_us = &timer0;
}

// TIM_DRIVER timer_us(1, 1/0.002, timer_mode::timer_counter);

void delay_ms(uint32_t milliseconds)
{
	HAL_Delay(milliseconds);
	// sys_delay_ms(milliseconds);
}
void delay_us(uint32_t microseconds) {
	
	// if(tim1_state == false) {
	// 	tim1_state = true;
	// 	TIM_DRIVER timer_us(1, 1/0.002, timer_mode::timer_counter);
	// 	timer1 = &timer_us;
	// }

	// uint32_t value = static_cast<uint32_t>(18.0*microseconds);
	timer_us->reset_cnt();
	while(timer_us->get_cnt() < 24*microseconds);

	// // Old way;
	// uint32_t a;
	// for(uint32_t i=0; i<microseconds	; i++) {
	// 	a++;
	// }
}
