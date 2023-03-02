/*
 * spi.cpp
 *
 *  Created on: 8 de fev de 2017
 *      Author: titi
 */
#include "spi.h"

/*
 *	// mode=1; used on nokia5110 GLCD
 *	// for SPI2 does not have remap option;
 */

#define SPIy			SPI1

#define SPI1_PORT		GPIOA
#define SPI1_PIN_MOSI	GPIO_Pin_7
#define SPI1_PIN_MISO	GPIO_Pin_6
#define SPI1_PIN_SCK 	GPIO_Pin_5
#define SPI1_PIN_NSS 	GPIO_Pin_4

#define SPI2_PORT		GPIOB
#define SPI2_PIN_MOSI	GPIO_Pin_15
#define SPI2_PIN_MISO	GPIO_Pin_14
#define SPI2_PIN_SCK 	GPIO_Pin_13
#define SPI2_PIN_NSS 	GPIO_Pin_12

void SPI::set_SPI_to_Master(uint8_t mode, uint8_t spi_port)//, uint8_t spi_remap)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	switch (spi_port)
	{
		case 1:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MISO | SPI1_PIN_SCK | SPI1_PIN_MOSI;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(SPI1_PORT, &GPIO_InitStructure);
			break;

		case 2:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = SPI2_PIN_MISO | SPI2_PIN_SCK | SPI2_PIN_MOSI;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(SPI2_PORT, &GPIO_InitStructure);
			break;
	}


	/* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
//	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_NSS;
//	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(SPI_PORT, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MISO;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(SPI_PORT, &GPIO_InitStructure);
//	SPI1_REMAP
//	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
// or
//	AFIO -> EVCR |= AFIO_EVCR_PORT_PA | AFIO_EVCR_PIN_PX5;
//	AFIO -> EVCR |= AFIO_EVCR_EVOE;
//	AFIO -> MAPR |= AFIO_MAPR_SPI1_REMAP;

	SPI_InitTypeDef   SPI_InitStructure;

	/* SPIy Config -------------------------------------------------------------*/
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		// Slave mode selected
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	// byte size
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;// | SPI_NSSInternalSoft_Set;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	switch (mode)
	{
		case 1:		// used on nokia5110 GLCD
		{
			SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
			break;
		}

		default:	// default mode full duplex with miso/mosi elements
		{
			SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;			// clock is low when idle
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;		// data sampled at first edge
			break;
		}
	}

	switch (spi_port)
	{
		case 1:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

			SPI_Init(SPI1, &SPI_InitStructure);
			SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
			SPI_Cmd(SPI1, ENABLE);

			break;

		case 2:
			RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

			SPI_Init(SPI2, &SPI_InitStructure);
			SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);
			SPI_Cmd(SPI2, ENABLE);
			break;
	}

//	SPI_SSOutputCmd(SPI1, ENABLE);
//	Interrupt ------
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_OVR, ENABLE);
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);
//	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
//	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;				// we want to configure the USART1 interrupts
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		// this sets the subpriority inside the group
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// the USART1 interrupts are globally enabled
//	NVIC_Init(&NVIC_InitStructure);								// the properties are passed to the NVIC_Init function which takes care of the low level stuff
//	Interrupt end ---------

}
void SPI::set_SPI_to_Slave()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MOSI | SPI1_PIN_MISO;
	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_PIN_SCK;
	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);

//	SPI1_REMAP
//	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
	// or
//	AFIO -> EVCR |= AFIO_EVCR_PORT_PA | AFIO_EVCR_PIN_PX5;
//	AFIO -> EVCR |= AFIO_EVCR_EVOE;
//	AFIO -> MAPR |= AFIO_MAPR_SPI1_REMAP;

	SPI_InitTypeDef   SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* SPIy Config -------------------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;		// Slave mode selected
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	// byte size
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;			// clock is low when idle
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;		// data sampled at second edge
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Reset);

//	Interrupt ------
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_OVR, ENABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;				// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);								// the properties are passed to the NVIC_Init function which takes care of the low level stuff
//	Interrupt end ---------

	/* Add IRQ vector to NVIC */
	/* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
//	NVIC_InitTypeDef NVIC_InitStruct;
//
//	NVIC_InitStruct.NVIC_IRQChannel = SPI1_IRQn; //EXTI15_10_IRQn;
//	/* Set priority */
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
//	/* Set sub priority */
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
//	/* Enable interrupt */
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	/* Add to NVIC */
//	NVIC_Init(&NVIC_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}
void SPI::print(char const *s)
{
	while(*s)
	{
		// wait until data register is empty. Transmission complete (TC) bit
		while( !(SPI1-> SR & 0x00000002) );
			SPI1 -> DR = (*s & (uint16_t)0x01FF);
		s++;
	}
}
unsigned char SPI::transfer_Byte(uint16_t Data)
{
	unsigned char Data2 = 0;
	// Verify if transmit buffer is empty TXE bit
	while(!(SPI1-> SR & SPI_I2S_FLAG_TXE) );
	SPI1 -> DR = Data;
//	SPI1 -> DR = (Data & (uint16_t)0x00FF);
	while(!(SPI1-> SR & SPI_I2S_FLAG_RXNE) );
	Data2 = SPI1 -> DR;

	return Data2;
}
void SPI::writeByte(uint16_t Data)
{
	while(!(SPI1-> SR & SPI_I2S_FLAG_TXE) );
	SPI1 -> DR = Data;
}
unsigned char SPI::readByte()
{
	int i;
	char data = 0;

	if(SPI1_cnt)
	{
		data = SPI_received_string[0];

		for(i=0;i<SPI1_cnt;i++)
		{
			SPI_received_string[i] = SPI_received_string[i+1];
		}
		SPI1_cnt--;
	}

	return data;
}
int SPI::available()
{
	return SPI1_cnt;
}
//void SPI::master_demo01()
//{
//	// Slave/Master with interruption
//	if(SPI1_flag_rx)
//	{
//		SPI1_flag_rx = 0;
////		USART1_print("Received: ");
////		USART1_putc(SPI1_rxd);
////		USART1_print("Transmitted: ");
////		USART1_putc(SPI1_txd);
////		LED_green_toogle();
//	}
//	if(SPI1_flag_err)
//	{
//		SPI1_flag_err = 0;
////		USART1_println("Err");
//	}
//
//
////		// Master
////		if(SPI1_txd > 0x5a)
////		{
////			SPI1_txd = 0x41;	// A
////		}
////
////		if(SPI1_flag_rx)
////		{
////			SPI1_flag_rx = 0;
////			USART1_print("Receiving: ");
////			USART1_putc(SPI1_rxd);
////		}
////		USART1_print("Transmitting: ");
////		USART1_putc(SPI1_txd);
////		SPI1_writeByte(SPI1_txd);
//
//		// All data transmitted/received but SPI may be busy so wait until done.
//		while(SPI1->SR & SPI_I2S_FLAG_BSY);
//
//		// TX: 1- empty
//		if(SPI1 -> SR & (uint16_t) SPI_SR_TXE)
//		{
////			USART1_print("Transmitting: ");
//			SPI1 -> DR = txcm;
//			//    Wait until the data has been transmitted.
//			while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
////			USART1_putc(txcm);
//		}
//
//		// Wait for any data on MISO pin to be received.
//		while (!(SPI1->SR & SPI_I2S_FLAG_RXNE));
//		// RX: 1- not empty
//		if(SPI1 -> SR & (uint16_t) SPI_SR_RXNE)
//		{
////			LED_green_toogle();
//			rxc = SPI1 -> DR;
////			USART1_print("Receiving: ");
////			USART1_putc(rxc);
//		//			rxc++;
//		//			txc = rxc;
//		}
//		delay_ms(500);
//}
//void SPI::slave_demo01()
//{
//	// Slave/Master with interruption
//	if(SPI1_flag_rx)
//	{
//		SPI1_flag_rx = 0;
////		USART1_print("Received: ");
////		USART1_putc(SPI1_rxd);
////		USART1_print("Transmitted: ");
////		USART1_putc(SPI1_txd);
////		LED_green_toogle();
//	}
//	if(SPI1_flag_err)
//	{
//		SPI1_flag_err = 0;
////		USART1_println("Err");
//	}
//
//
//	// Slave Polling
//	//	TX: 1- empty
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
//	{
//		rxc = SPI1 -> DR;
////			while (!(SPI1->SR & SPI_I2S_FLAG_RXNE));
////		USART1_print("Receiving: ");
////		USART1_putc(rxc);
////			rxc++;
////			txcs = rxc;
//	}
//
//	// RX: 1- not empty
//	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
//	{
//		SPI1 -> DR = txcs;
////			while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
////		USART1_print("Transmitting: ");
////		USART1_putc(txcs);
////		LED_green_toogle();
//	}
////		if(err_cnt > 10)
////		{
////			err_cnt=0;
////			SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
////			delay_ms(100);
////			SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Reset);
////		}
//}
//
//
