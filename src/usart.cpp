// #include "usart.hpp"

// // extern UART_HandleTypeDef huart1;

// void USART_driver::begin(uint32_t baudrate)
// {
// 	/* This is a concept that has to do with the libraries provided by ST
// 	* to make development easier the have made up something similar to
// 	* classes, called TypeDefs, which actually just define the common
// 	* parameters that every peripheral needs to work correctly
// 	*
// 	* They make our life easier because we don't have to mess around with
// 	* the low level stuff of setting bits in the correct registers
// 	*/

// 	// USART_InitTypeDef USART_InitStruct; 			// this is for the USART1 initilization
// 	// NVIC_InitTypeDef NVIC_InitStructure; 			// this is used to configure the NVIC (nested vector interrupt controller)

// 	GPIO_InitTypeDef GPIO_InitStruct = {0};			// this is for the GPIO pins used as TX and RX
// 	/* enable APB2 peripheral clock for USART1
// 	* note that only USART1 and USART6 are connected to APB2
// 	* the other USARTs are connected to APB1
// 	*/
// 	__HAL_RCC_USART2_CLK_ENABLE();					// USART1 clock enable
// 	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

//     __HAL_RCC_GPIOA_CLK_ENABLE();
// 	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// used for PA9 and PA10

// 	/* This sequence sets up the TX and RX pins
// 	* so they work correctly with the USART2 peripheral
// 	*/
//     /**USART2 GPIO Configuration
//     PA2     ------> USART2_TX
//     PA3     ------> USART2_RX
//     */
// 	GPIO_InitStruct.Pin = GPIO_PIN_2; 				// Pin 6 (TX)
// 	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 		// the pins are configured as alternate function so the USART peripheral has access to them, Push pull output
// 	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	// this defines the IO speed and has nothing to do with the baudrate!
// 	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);			// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

// 	GPIO_InitStruct.Pin = GPIO_PIN_3;				// Pin 7 (RX)
// 	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
// 	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
// 	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//     /* USART2 interrupt Init */
//     HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
//     HAL_NVIC_EnableIRQ(USART2_IRQn);

// 	// remap pins to usart2
// 	// __HAL_AFIO_REMAP_USART2_ENABLE();

// 	// Interrupt register
// 	// HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
// 	// HAL_NVIC_EnableIRQ(USART1_IRQn);

// 	/* Now the USART_InitStruct is used to define the
// 	* properties of USART1
// 	*/
// 	huart2_.Instance = USART2;
// 	huart2_.Init.BaudRate = baudrate;
// 	huart2_.Init.WordLength = UART_WORDLENGTH_8B;
// 	huart2_.Init.StopBits = UART_STOPBITS_1;
// 	huart2_.Init.Parity = UART_PARITY_NONE;
// 	huart2_.Init.Mode = UART_MODE_TX_RX;
// 	huart2_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
// 	huart2_.Init.OverSampling = UART_OVERSAMPLING_16;
// 	if (HAL_UART_Init(&huart2_) != HAL_OK)
// 	{

// 	}

// 	// old configuration
// 	// USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
// 	// USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
// 	// USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
// 	// USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
// 	// USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
// 	// USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
// 	// USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


// 	/* Here the USART1 receive interrupt is enabled
// 	* and the interrupt controller is configured
// 	* to jump to the USART1_IRQHandler() function
// 	* if the USART1 receive interrupt occurs
// 	*/
// 	// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

// 	// NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		// we want to configure the USART1 interrupts
// 	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
// 	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;	// this sets the subpriority inside the group
// 	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// the USART1 interrupts are globally enabled
// 	// NVIC_Init(&NVIC_InitStructure);							// the properties are passed to the NVIC_Init function which takes care of the low level stuff

// 	// // finally this enables the complete USART1 peripheral
// 	// USART_Cmd(USART1, ENABLE);
// }
// void USART_driver::print(char const *s)
// {
// 	// uint8_t data[] = "HELLO WORLD \r\n";

// 	// HAL_UART_Transmit(&huart2_, static_cast<uint8_t*>(s), sizeof (*s), 1000);

// 	// while(*s)
// 	// {
// 	// 	// wait until data register is empty. Transmission complete (TC) bit
// 	// 	while(!(USART1->SR & USART_SR_TC));
// 	// 	USART1 -> DR = (*s & (uint16_t)0x00FF);
// 	// 	s++;
// 	// }
// }
// void USART_driver::println(char const *s)
// {
// 	// while(*s)
// 	// {
// 	// 	// wait until data register is empty. Transmission complete (TC) bit
// 	// 	while(!(USART1->SR & USART_SR_TC));
// 	// 	USART1 -> DR = (*s & (uint16_t)0x00FF);
// 	// 	s++;
// 	// }

// 	// while(!(USART1->SR & USART_SR_TC));
// 	// USART1 -> DR = (0x0d & (uint16_t)0x00FF);

// 	// while(!(USART1->SR & USART_SR_TC));
// 	// USART1 -> DR = (0x0a & (uint16_t)0x00FF);
// }
// void USART_driver::println(int value)
// {
// 	// char *s = (char *)malloc(8 * sizeof(char));
// 	// sprintf(s,"%d", value);

// 	// while(*s)
// 	// {
// 	// 	// wait until data register is empty. Transmission complete (TC) bit
// 	// 	while(!(USART1->SR & USART_SR_TC));
// 	// 	USART1 -> DR = (*s & (uint16_t)0x00FF);
// 	// 	s++;
// 	// }

// 	// while(!(USART1->SR & USART_SR_TC));
// 	// USART1 -> DR = (0x0d & (uint16_t)0x00FF);

// 	// while(!(USART1->SR & USART_SR_TC));
// 	// USART1 -> DR = (0x0a & (uint16_t)0x00FF);
// }
// void USART_driver::write(char c)
// {
// 	uint8_t data[] = "HELLO WORLD \r\n";

// 	HAL_UART_Transmit(&huart2_, data, sizeof (data), 1000);

// 	// // toggle LED 
// 	// HAL_Delay (250);  // 250 ms delay

// 	// Old way using registers
// 	// // wait until transmittion complete
// 	// while( !(USART1-> SR & USART_SR_TC));
// 	// USART1 -> DR = (c & (uint16_t)0x00FF);

// 	// while( !(USART1-> SR & USART_SR_TC));
// 	// USART1 -> DR = (0x0d & (uint16_t)0x00FF);

// 	// while( !(USART1-> SR & USART_SR_TC));
// 	// USART1 -> DR = (0x0a & (uint16_t)0x00FF);
// /*
// common ascii codes to know
// Char  Dec  Oct  Hex   WhatAreThey
// ---------------------------------------
// (nul)   0 0000 0x00   Null
// (ht)    9 0011 0x09   Horizontal Tab
// (nl)   10 0012 0x0a   New Line
// (vt)   11 0013 0x0b   Vertical Tab
// (cr)   13 0015 0x0d   Carriage Return
// (sp)   32 0040 0x20   Space
// 0      48 0060 0x30   zero
// A      65 0101 0x41   capital A
// a      97 0141 0x61   lowercase a


// ascii codes and their escape sequences
// ASCII Name   Description     C Escape Sequence
// ----------------------------------------------
// nul          null byte       \0 (zero)
// bel          bel character   \a
// bs           backspace       \b
// ht           horizontal tab  \t
// np           formfeed        \f
// nl           newline         \n
// cr           carriage return \r

// end of line sequences
// Windows end of line sequence:  \r\n
// Unix end of line sequence: \n
// Mac end of line sequence: \r

// ascii table
// Char  Dec  Oct  Hex | Char  Dec  Oct  Hex | Char  Dec  Oct  Hex | Char Dec  Oct   Hex
// -------------------------------------------------------------------------------------
// (nul)   0 0000 0x00 | (sp)   32 0040 0x20 | @      64 0100 0x40 | `      96 0140 0x60
// (soh)   1 0001 0x01 | !      33 0041 0x21 | A      65 0101 0x41 | a      97 0141 0x61
// (stx)   2 0002 0x02 | "      34 0042 0x22 | B      66 0102 0x42 | b      98 0142 0x62
// (etx)   3 0003 0x03 | #      35 0043 0x23 | C      67 0103 0x43 | c      99 0143 0x63
// (eot)   4 0004 0x04 | $      36 0044 0x24 | D      68 0104 0x44 | d     100 0144 0x64
// (enq)   5 0005 0x05 | %      37 0045 0x25 | E      69 0105 0x45 | e     101 0145 0x65
// (ack)   6 0006 0x06 | &      38 0046 0x26 | F      70 0106 0x46 | f     102 0146 0x66
// (bel)   7 0007 0x07 | '      39 0047 0x27 | G      71 0107 0x47 | g     103 0147 0x67
// (bs)    8 0010 0x08 | (      40 0050 0x28 | H      72 0110 0x48 | h     104 0150 0x68
// (ht)    9 0011 0x09 | )      41 0051 0x29 | I      73 0111 0x49 | i     105 0151 0x69
// (nl)   10 0012 0x0a | *      42 0052 0x2a | J      74 0112 0x4a | j     106 0152 0x6a
// (vt)   11 0013 0x0b | +      43 0053 0x2b | K      75 0113 0x4b | k     107 0153 0x6b
// (np)   12 0014 0x0c | ,      44 0054 0x2c | L      76 0114 0x4c | l     108 0154 0x6c
// (cr)   13 0015 0x0d | -      45 0055 0x2d | M      77 0115 0x4d | m     109 0155 0x6d
// (so)   14 0016 0x0e | .      46 0056 0x2e | N      78 0116 0x4e | n     110 0156 0x6e
// (si)   15 0017 0x0f | /      47 0057 0x2f | O      79 0117 0x4f | o     111 0157 0x6f
// (dle)  16 0020 0x10 | 0      48 0060 0x30 | P      80 0120 0x50 | p     112 0160 0x70
// (dc1)  17 0021 0x11 | 1      49 0061 0x31 | Q      81 0121 0x51 | q     113 0161 0x71
// (dc2)  18 0022 0x12 | 2      50 0062 0x32 | R      82 0122 0x52 | r     114 0162 0x72
// (dc3)  19 0023 0x13 | 3      51 0063 0x33 | S      83 0123 0x53 | s     115 0163 0x73
// (dc4)  20 0024 0x14 | 4      52 0064 0x34 | T      84 0124 0x54 | t     116 0164 0x74
// (nak)  21 0025 0x15 | 5      53 0065 0x35 | U      85 0125 0x55 | u     117 0165 0x75
// (syn)  22 0026 0x16 | 6      54 0066 0x36 | V      86 0126 0x56 | v     118 0166 0x76
// (etb)  23 0027 0x17 | 7      55 0067 0x37 | W      87 0127 0x57 | w     119 0167 0x77
// (can)  24 0030 0x18 | 8      56 0070 0x38 | X      88 0130 0x58 | x     120 0170 0x78
// (em)   25 0031 0x19 | 9      57 0071 0x39 | Y      89 0131 0x59 | y     121 0171 0x79
// (sub)  26 0032 0x1a | :      58 0072 0x3a | Z      90 0132 0x5a | z     122 0172 0x7a
// (esc)  27 0033 0x1b | ;      59 0073 0x3b | [      91 0133 0x5b | {     123 0173 0x7b
// (fs)   28 0034 0x1c | <      60 0074 0x3c | \      92 0134 0x5c | |     124 0174 0x7c
// (gs)   29 0035 0x1d | =      61 0075 0x3d | ]      93 0135 0x5d | }     125 0175 0x7d
// (rs)   30 0036 0x1e | >      62 0076 0x3e | ^      94 0136 0x5e | ~     126 0176 0x7e
// (us)   31 0037 0x1f | ?      63 0077 0x3f | _      95 0137 0x5f | (del) 127 0177 0x7f
// */
// }
// int USART_driver::available(void)
// {
// 	// return _USART1_cnt;
// }
// char USART_driver::read(void)
// {
// 	// int i;
// 	// char data = 0;

// 	// if(_USART1_cnt)
// 	// {
// 	// 	data = _received_string[0];

// 	// 	// Fila
// 	// 	for(i=0;i<_USART1_cnt;i++)
// 	// 	{
// 	// 		_received_string[i] = _received_string[i+1];
// 	// 	}
// 	// 	_USART1_cnt--;
// 	// }

// 	// return data;
// }
// void USART_driver::end(void) {
// 	// if(huart1_.Instance == USART1) {
// 	// 	/* Peripheral clock disable */
// 	// 	__HAL_RCC_USART1_CLK_DISABLE();

// 	// 	/**USART1 GPIO Configuration
// 	// 	PB6     ------> USART1_TX
// 	// 	PB7     ------> USART1_RX
// 	// 	*/
// 	// 	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

// 	// 	/* USART1 interrupt Deinit */
// 	// 	HAL_NVIC_DisableIRQ(USART1_IRQn);
// 	// }
// }
// // void USART_driver::USART1_IRQHandler(void)
// // {
// // 	// HAL_UART_IRQHandler
// // 	// check if the USART1 receive interrupt flag was set

// // // 	while(USART_GetITStatus(USART1, USART_IT_RXNE))
// // // 	{
// // // //		LED_green_toogle();
// // // 		char t = USART1->DR; // the character from the USART1 data register is saved in t
// // // 		if(USART1_cnt < MAX_STRLEN)
// // // 		{
// // // 			received_string[USART1_cnt] = t;
// // // 			USART1_cnt++;
// // // 		}
// // // 		else
// // // 		{
// // // 			memset(received_string,0,sizeof(received_string));
// // // 			USART1_cnt = 0;
// // // 		}

// // //		USART1_putc(t);

// // 		// check if the received character is not the LF character (used to determine end of string)
// // 		// or the if the maximum string length has been been reached
// // //		if( (t != '\n') && (cnt < MAX_STRLEN) )
// // //		{
// // //			received_string[cnt] = t;
// // //			cnt++;
// // //		}
// // //		else{ // otherwise reset the character counter and print the received string
// // //			cnt = 0;
// // //			USART1_puts(received_string);
// // //			memset(received_string,0,sizeof(received_string));
// // //		}
// // 	// }
// // }
// // void USART_driver::USART1_IRQHandler(void) {
// // 	// check if the USART1 receive interrupt flag was set
// // 	while(USART_GetITStatus(USART1, USART_IT_RXNE))
// // 	{
// // 		GPIOC -> ODR ^= (1<<13);
// // 		char t = (USART1->DR); // the character from the USART1 data register is saved in t

// // //		if(Serial._USART1_cnt < MAX_STRLEN)
// // //		{
// // //			Serial._received_string[Serial._USART1_cnt] = t;
// // //			Serial._USART1_cnt++;
// // //		}
// // //		else
// // //		{
// // ////			memset(Serial._received_string,0,sizeof(Serial._received_string));
// // //			Serial._USART1_cnt = 0;
// // //		}

// // 		// wait until transmittion complete
// // 		while( !(USART1-> SR & USART_SR_TC));
// // 		USART1 -> DR = (t & (uint16_t)0x00FF);

// // 		while( !(USART1-> SR & USART_SR_TC));
// // 		USART1 -> DR = (0x0d & (uint16_t)0x00FF);

// // 		while( !(USART1-> SR & USART_SR_TC));
// // 		USART1 -> DR = (0x0a & (uint16_t)0x00FF);
// // 	}
// // }
// // void USART_driver::USART1_IRQHandler(void)
// // {
// // 	// check if the USART1 receive interrupt flag was set
// // 	while(USART_GetITStatus(USART1, USART_IT_RXNE))
// // 	{
// // 		GPIOC -> ODR ^= (1<<13);
// // 		char t = (USART1->DR); // the character from the USART1 data register is saved in t

// // //		if(Serial._USART1_cnt < MAX_STRLEN)
// // //		{
// // //			Serial._received_string[Serial._USART1_cnt] = t;
// // //			Serial._USART1_cnt++;
// // //		}
// // //		else
// // //		{
// // ////			memset(Serial._received_string,0,sizeof(Serial._received_string));
// // //			Serial._USART1_cnt = 0;
// // //		}

// // 		// wait until transmittion complete
// // 		while( !(USART1-> SR & USART_SR_TC));
// // 		USART1 -> DR = (t & (uint16_t)0x00FF);

// // 		while( !(USART1-> SR & USART_SR_TC));
// // 		USART1 -> DR = (0x0d & (uint16_t)0x00FF);

// // 		while( !(USART1-> SR & USART_SR_TC));
// // 		USART1 -> DR = (0x0a & (uint16_t)0x00FF);
// // 	}
// // }
