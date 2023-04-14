#include "delay.h"

void delay_ms(uint32_t milliseconds)
{
	HAL_Delay(milliseconds);
	// sys_delay_ms(milliseconds);
}
void delay_us(uint32_t microseconds)
{
	uint32_t a;
	for(int i=0; i<microseconds	; i++) {
		a++;
	}
	// https://controllerstech.com/create-1-microsecond-delay-stm32/
	// ets_delay_us(microseconds);
}
