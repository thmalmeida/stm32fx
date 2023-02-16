// #ifndef __USART_HPP_
// #define __USART_HPP__

// /* Includes ------------------------------------------------------------------*/
// #include "stm32f1xx_hal.h"
// // #include "stm32f1xx_it.h"

// // #include <iostream>
// // #include <string>	
// // #include <stdio.h>
// // #include <stdlib.h>

// // #include <stdio.h>
// // #include "stm32f1xx_hal.h"

// // extern "C" {
// // extern UART_HandleTypeDef huart1;
// // }

// class USART_driver {
// public:

// 	static const int bufferSize = 60;
// 	char buffer[bufferSize];
// 	volatile int _USART1_cnt;
// 	unsigned char _received_string[bufferSize];
// //	extern __IO int USART1_cnt;
// //	extern __IO char received_string[MAX_STRLEN+1]; // this will hold the recieved string


// 	USART_driver(uint32_t baud_rate) {
// 		begin(baud_rate);
// 	};

// 	void begin(uint32_t baud_rate);

// 	void print(char const *s);
// 	void println(char const *s);
// 	void println(int value);
// 	void write(char c);
// 	int available(void);
// 	char read(void);
// 	void end(void);

// 	UART_HandleTypeDef huart2_;

// private:
// 	// void USART1_IRQHandler(void);
// };

// #endif /* __USART_H__ */

