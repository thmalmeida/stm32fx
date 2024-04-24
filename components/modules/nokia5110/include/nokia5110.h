/*
 * nokia5110.h
 *
 *  Created on: 25 de mai de 2017
 *      Author: titi
 */

#ifndef NOKIA5110_NOKIA5110_H_
#define NOKIA5110_NOKIA5110_H_

#include "../Hardware/spi.h"
#include "../Hardware/gpio.h"
#include "glcd.h"
#include "fonts/fonts.h"

#define LCD_PORT 						GPIOA

#define	LCD_PIN_LED						13	// PB0
#define LCD_PIN_SCE						16	// PB11
#define LCD_PIN_RESET					15	// PB10
#define LCD_PIN_COMMAND					14	// PB1
#define LCD_PIN_CLOCK					10	// PA5
#define LCD_PIN_DATA					12	// PA7

//#define SPIy                   SPI1
//#define SPIy_CLK               RCC_APB2Periph_SPI1
//#define SPIy_GPIO              GPIOA
////#define SPIy_GPIO_CLK          RCC_APB2Periph_GPIOA
//#define SPIy_PIN_SCK           GPIO_Pin_5
//#define SPIy_PIN_MISO          GPIO_Pin_6
//#define SPIy_PIN_MOSI          GPIO_Pin_7

#define PCD8544_TIME_DELAY				20

#define BLACK 							1
#define WHITE 							0

#define LCDWIDTH						84	// Pixels
#define LCDHEIGHT						48

#define PCD8544_POWERDOWN				0x04	// or
#define PCD8544_ENTRYMODE				0x02
#define PCD8544_EXTENDEDINSTRUCTION		0x01

#define PCD8544_DISPLAYBLANK			0x0
#define PCD8544_DISPLAYNORMAL			0x4
#define PCD8544_DISPLAYALLON			0x1
#define PCD8544_DISPLAYINVERTED			0x5

// H = 0
#define PCD8544_FUNCTION_SET			0x20
#define PCD8544_POWER_DOWN				(1<<2)
//#define PCD8544_FUNCTION_SET			(1<<5)
#define PCD8544_DISPLAYCONTROL			0x08
#define PCD8544_SET_YADDR				0x40
#define PCD8544_SET_XADDR				0x80

// H = 1
#define PCD8544_SET_TEMP				0x04
#define PCD8544_SET_BIAS				0x10
#define PCD8544_SET_VOP					0x80

#define PCD8544_NOP						0

#define PCD8544_HORIZONTAL_ADDRESSING	0
#define PCD8544_VERTICAL_ADDRESSING  	(1<<1)
#define PCD8544_EXTENDED_INSTRUCTION	(1<<0)

#define PCD8544_SET_Y_ADDRESS			0x40
#define PCD8544_SET_X_ADDRESS			0x80

#define PCD8544_MAX_BANKS				6
#define PCD8544_MAX_COLS				84

// name Basic instruction set (H=0)
#define PCD8544_DISPLAY_CONTROL			(1<<3)
#define PCD8544_DISPLAY_BLANK			0x0
#define PCD8544_DISPLAY_NORMAL			(1<<2)
#define PCD8544_DISPLAY_ALL_ON			(1<<0)
#define PCD8544_DISPLAY_INVERTED		(1<<2|1<<0)
// name Extended instruction set (H=1)

//#define PCD8544_SET_TEMP (1<<2)
#define PCD8544_TEMPCO_0				0b00
#define PCD8544_TEMPCO_1				0b01
#define PCD8544_TEMPCO_2				0b10
#define PCD8544_TEMPCO_3				0b11

//#define PCD8544_SET_BIAS (1<<4)
//#define PCD8544_SET_VOP  (1<<7)

/* Private define ------------------------------------------------------------*/
#define BufferSize 32
#define SEND_LIMIT 3

class NOKIA5110 : SPI, public GPIO {
public:

	static const int bufferSize = 20;
	char buffer[bufferSize];

	void RCC_Configuration();
	void GPIO_Configuration();
	void SPI1_Configuration();

	void glcd_init(uint8_t spi_port);//, uint8_t spi_remap);
	void glcd_init_pins();
	void glcd_data(uint8_t c);
	void glcd_write();
	void glcd_clear2();
	void glcd_set_x_address(uint8_t x);
	void glcd_set_y_address(uint8_t y);

	void glcd_command(uint8_t c);

	void glcd_put_char(char c);
	void glcd_put_string(uint8_t x, uint8_t y, char const *str);
	void glcd_reset();
	void glcd_print(void);
	void glcd_set_contrast(uint8_t val);
	void glcd_power_down(void);
	void glcd_power_up(void);
	void glcd_PCD8544_init(void);
	void glcd_example();
	void glcd_dot_print(uint8_t x, uint8_t y, uint8_t size, uint8_t value);
	void glcd_space_print(uint8_t x, uint8_t y);
	void glcd_hash_print(uint8_t x, uint8_t y);
	void glcd_Arial16x24_str(uint8_t x, uint8_t y, char const *str);
	void glcd_big_str(uint8_t x, uint8_t y, char const *str);

	void glcd_SPI1_debug();

	void glcd_backlight(uint8_t status);

protected:
	void LCD_ENABLE();
	void LCD_DISABLE();
	void LCD_RESET_ON();
	void LCD_RESET_OFF();
	void LCD_DC_COMMAND();
	void LCD_DC_DATA();
};

void NOKIA5110::LCD_ENABLE()
{
//	GPIO_ResetBits(LCD_PORT, LCD_PIN_SCE);
	gateSet(LCD_PIN_SCE, 0);
}
void NOKIA5110::LCD_DISABLE()
{
//	GPIO_SetBits(LCD_PORT, LCD_PIN_SCE);
	gateSet(LCD_PIN_SCE, 1);
}
void NOKIA5110::LCD_RESET_ON()
{
//	GPIO_ResetBits(LCD_PORT, LCD_PIN_RESET);
	gateSet(LCD_PIN_RESET, 0);
}
void NOKIA5110::LCD_RESET_OFF()
{
//	GPIO_SetBits(LCD_PORT, LCD_PIN_RESET);
	gateSet(LCD_PIN_RESET, 1);
}
void NOKIA5110::LCD_DC_COMMAND()
{
//	GPIO_ResetBits(LCD_PORT, LCD_PIN_COMMAND);
	gateSet(LCD_PIN_COMMAND, 0);
}
void NOKIA5110::LCD_DC_DATA()
{
//	GPIO_SetBits(LCD_PORT, LCD_PIN_COMMAND);
	gateSet(LCD_PIN_COMMAND, 1);
}
void NOKIA5110::glcd_backlight(uint8_t status)
{
	if(status)
		gateSet(LCD_PIN_LED, 1);
	else
		gateSet(LCD_PIN_LED, 0);
}
void NOKIA5110::RCC_Configuration(void)
{
	/* PCLK2 = HCLK/2 */
	RCC_PCLK2Config(RCC_HCLK_Div2);
}
//void GPIO_Configuration()
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//
//	/* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
//	GPIO_InitStructure.GPIO_Pin = SPIy_PIN_SCK | SPIy_PIN_MOSI | SPIy_PIN_MISO;
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
//	/* Configure SCK and MOSI pins as Alternate Function Push Pull */
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(LCD_PORT, &GPIO_InitStructure);
//
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5); //SCK
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5); //MISO
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5); //MOSI
//
//	/*Configure GPIO pins*/
//	GPIO_InitStructure.GPIO_Pin = LCD_PIN_SCE | LCD_PIN_RESET | LCD_PIN_COMMAND | LCD_PIN_LED;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(LCD_PORT, &GPIO_InitStructure);
//
//
////	GPIOA ->
//}
//void writeByte(uint8_t B)
//{
//	SPI_SendData8(SPI1, B);
//}
void NOKIA5110::glcd_reset()
{
//	LCD_DISABLE();
	LCD_RESET_ON();
	_delay_us(PCD8544_TIME_DELAY);
	LCD_RESET_OFF();
	_delay_us(PCD8544_TIME_DELAY);
	LCD_ENABLE();
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_command(uint8_t c)
{
	LCD_RESET_OFF();
	LCD_ENABLE();
	LCD_DC_COMMAND();
	writeByte(c);
//	SPI_write(c);
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_data(uint8_t c)
{
	LCD_RESET_OFF();
	LCD_ENABLE();
	LCD_DC_DATA();
	_delay_us(PCD8544_TIME_DELAY);
	writeByte(c);
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_SPI1_debug()
{
	char c = 'A';
	while(1)
	{
		writeByte(c);
	}
}
void NOKIA5110::glcd_init(uint8_t spi_port)//, uint8_t spi_remap)	// spi_port=1 (SPI1), spi_port=2 (SPI2) spi_remap=0 (default), spi_remap=1 (remap port)
{
//	SPI1_Configuration();
//	GPIO_Configuration();
	gateConfig(LCD_PIN_SCE, 1);
	gateConfig(LCD_PIN_RESET, 1);
	gateConfig(LCD_PIN_COMMAND, 1);
	gateConfig(LCD_PIN_DATA, 1);
	gateConfig(LCD_PIN_CLOCK, 1);
	gateConfig(LCD_PIN_LED, 1);

	set_SPI_to_Master(1, spi_port);//, spi_remap);

	// --- Reset and Enable
	glcd_reset();
	glcd_backlight(1);

	// --- glcd intialization
	// get into the EXTENDED mode!
	glcd_command(PCD8544_FUNCTION_SET | PCD8544_EXTENDEDINSTRUCTION);

	// LCD bias select (4 is optimal?)
	glcd_command(PCD8544_SET_BIAS | 0x04);

	// set VOP
//	if (contrast > 0x7f)
//	contrast = 0x7f;

	glcd_command(PCD8544_SET_VOP | 40); // Experimentally determined

	// normal mode
	glcd_command(PCD8544_FUNCTION_SET);

	// Set display to Normal
	glcd_command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);

	glcd_clear2();
//	/* Initialization sequence of controller */
//	glcd_reset();
//
//	/* Get into the EXTENDED mode! */
//	glcd_command(PCD8544_FUNCTION_SET | PCD8544_EXTENDED_INSTRUCTION);
//
//	/* LCD bias select (4 is optimal?) */
//	glcd_command(PCD8544_SET_BIAS | 0x2);
//
//	/* Set VOP */
//	glcd_command(PCD8544_SET_VOP | 50); // Experimentally determined
//
//	/* Back to standard instructions */
//	glcd_command(PCD8544_FUNCTION_SET);
//
//	/* Normal mode */
//	glcd_command(PCD8544_DISPLAY_CONTROL | PCD8544_DISPLAY_NORMAL);
//
////	glcd_select_screen((uint8_t *)&glcd_buffer,&glcd_bbox);
//
////	glcd_set_contrast(50);
//
////	glcd_clear();
}
void NOKIA5110::glcd_print(void)
{
	glcd_data(0x5A);
	glcd_data(0x5A);
	glcd_data(0x5A);
	glcd_data(0x5A);
	glcd_data(0x5A);
}
void NOKIA5110::glcd_set_contrast(uint8_t val)
{
	glcd_command(PCD8544_FUNCTION_SET | PCD8544_EXTENDED_INSTRUCTION);
	glcd_command(PCD8544_SET_VOP | (val&0x7f));
	glcd_command(PCD8544_FUNCTION_SET);
	glcd_command(PCD8544_DISPLAY_CONTROL | PCD8544_DISPLAY_NORMAL);
}
void NOKIA5110::glcd_power_down(void)
{
	/* First, fill RAM with zeroes to ensure minimum specified current consumption */
	glcd_clear2();

	/* Power down */
	glcd_command(PCD8544_FUNCTION_SET|PCD8544_POWER_DOWN);
}
void NOKIA5110::glcd_power_up(void)
{
	glcd_command(PCD8544_FUNCTION_SET);
}
void NOKIA5110::glcd_set_y_address(uint8_t y)
{
	glcd_command(PCD8544_SET_Y_ADDRESS | (y > 5 ? 5 : y));
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_set_x_address(uint8_t x)
{
//	glcd_command(PCD8544_SET_X_ADDRESS | (x & 0x7f));
	glcd_command(0b10000000 | (x & 0b01111111));
	_delay_us(PCD8544_TIME_DELAY);
}
void NOKIA5110::glcd_clear2()
{
	int i;
	for(i=0; i<PCD8544_MAX_BANKS*PCD8544_MAX_COLS; i++)
	{
//		glcd_set_x_address(i);
//		delay_ms(1);
//		for(j=0; i<PCD8544_MAX_COLS; j++)
		{
//			glcd_set_y_address(j);
			glcd_data(0);
		}
	}
//	for (bank = 0; bank < PCD8544_MAX_BANKS; bank++) {
//		/* Each bank is a single row 8 bits tall */
//		uint8_t column;
//
//		if (glcd_bbox_selected->y_min >= (bank+1)*8) {
//			continue; /* Skip the entire bank */
//		}
//
//		if (glcd_bbox_selected->y_max < bank*8) {
//			break;    /* No more banks need updating */
//		}
//
//		glcd_command(PCD8544_SET_Y_ADDRESS | bank);
//		delay_ms(10);
//		glcd_command(PCD8544_SET_X_ADDRESS | glcd_bbox_selected->x_min);
//		delay_ms(10);
//		for (column = glcd_bbox_selected->x_min; column <= glcd_bbox_selected->x_max; column++)
//		{
//			glcd_data( glcd_buffer_selected[PCD8544_MAX_COLS * bank + column] );
//			delay_ms(10);
//		}
//	}
}
void NOKIA5110::glcd_write()
{
	uint8_t bank;

	for (bank = 0; bank < PCD8544_MAX_BANKS; bank++) {
		/* Each bank is a single row 8 bits tall */
		uint8_t column;

		if (glcd_bbox_selected->y_min >= (bank+1)*8) {
			continue; /* Skip the entire bank */
		}

		if (glcd_bbox_selected->y_max < bank*8) {
			break;    /* No more banks need updating */
		}

		glcd_command(PCD8544_SET_Y_ADDRESS | bank);
		glcd_command(PCD8544_SET_X_ADDRESS | glcd_bbox_selected->x_min);
		for (column = glcd_bbox_selected->x_min; column <= glcd_bbox_selected->x_max; column++)
		{
			glcd_data( glcd_buffer_selected[PCD8544_MAX_COLS * bank + column] );
			_delay_ms(10);
		}
	}

	glcd_reset_bbox();

}
void NOKIA5110::glcd_PCD8544_init(void) {

	glcd_reset();

	/* Get into the EXTENDED mode! */
	glcd_command(PCD8544_FUNCTION_SET | PCD8544_EXTENDED_INSTRUCTION);

	/* LCD bias select (4 is optimal?) */
	glcd_command(PCD8544_SET_BIAS | 0x2);

	/* Set VOP (affects contrast) */
	glcd_command(PCD8544_SET_VOP | 80); /* Experimentally determined, play with this figure until contrast looks nice */

	/* Back to standard instructions */
	glcd_command(PCD8544_FUNCTION_SET);

	/* Normal mode */
	glcd_command(PCD8544_DISPLAY_CONTROL | PCD8544_DISPLAY_NORMAL);
}
void NOKIA5110::glcd_put_char(char c)
{
	int i;
	glcd_data(0x00);					// print some 1x8 vertical blank space;
	for(i=0;i<5;i++)					// this font has 5 cols length of 1 byte each;
	{
		glcd_data(FontNew[(c-32)*5+i]);
	}

//	for(i=0;i<23;i++)
//	{
//		glcd_data(Numbers[(c-(32+17))*23+i]);
//	}
}
void NOKIA5110::glcd_put_string(uint8_t x, uint8_t y, char const *str)
{
	glcd_set_x_address(x);
	glcd_set_y_address(y);
	int i, length = strlen(str);

	for(i=0;i<length;i++)
		glcd_put_char(str[i]);
}
void NOKIA5110::glcd_example()
{
//	char *str1 = "EXAMPLE";
	char *str1 = {};
	strcpy(str1, "EXAMPLE");
//	char str2[9];
//	sprintf(str2,"EXAMPLE%d",0);
	glcd_command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
	glcd_put_string(13, 3, str1);
}
void NOKIA5110::glcd_dot_print(uint8_t x, uint8_t y, uint8_t size, uint8_t value)
{
	int i;

	glcd_set_x_address(x);	// set x,y dot position;
	glcd_set_y_address(y);

	glcd_data(0x00);		// little space before
	for(i=0;i<size;i++)
	{
		glcd_data(value);	// square dot
	}
	glcd_data(0x00);		// little space after dot
}
void NOKIA5110::glcd_space_print(uint8_t x, uint8_t y)
{
	glcd_set_x_address(x);
	glcd_set_y_address(y);

	glcd_data(0x00);					// print some 1x8 vertical blank space;
	for(int i=0;i<5;i++)					// this font has 5 cols length of 1 byte each;
	{
		glcd_data(0x00);
	}
}
void NOKIA5110::glcd_hash_print(uint8_t x, uint8_t y)
{
	glcd_set_x_address(x);
	glcd_set_y_address(y);

	glcd_data(0x00);					// print some 1x8 vertical blank space;
	for(int i=0;i<5;i++)					// this font has 5 cols length of 1 byte each;
	{
		glcd_data(0xFF);
	}
}
void NOKIA5110::glcd_Arial16x24_str(uint8_t x, uint8_t y, char const *str)
{
	int nb = 3; 				// number of bytes;
	int vl_len = 49;			// vector line lenght			22+1 = 23 the first is the character size

	int i, k, length = strlen(str);
	uint8_t x1, y1;
	x1 = x;
	y1 = y;
//	int j = 0;					// this variable multiplex the lines o bytes;
	int vc_len = (vl_len-1)/nb;	// graph character line height length 24 or nb bytes;

	for(i=0;i<length;i++)
	{
		glcd_set_x_address(x1+vc_len*i);	// 0, 11, 23, 35
		glcd_set_y_address(y1);
		for(k=0;k<vc_len;k++)
		{
			glcd_data(Arial16x24[vl_len*(str[i]-32)+nb*k+1]);
		}

		glcd_set_x_address(x1+vc_len*i);
		glcd_set_y_address(y1+1);
		for(k=0;k<vc_len;k++)
		{
			glcd_data(Arial16x24[vl_len*(str[i]-32)+nb*k+2]);
		}

		glcd_set_x_address(x1+vc_len*i);
		glcd_set_y_address(y1+2);
		for(k=0;k<vc_len;k++)
		{
			glcd_data(Arial16x24[vl_len*(str[i]-32)+nb*k+3]);
		}

		//		k=0;
		//		glcd_set_x_address(x1+vc_len*i);
		//		glcd_set_y_address(y1+j);
		//		do
		//		{
		//			for(j=0;j<nb;j++)
		//			{
		//				glcd_data(Arial16x24[vl_len*(str[i]-48)+k+1]);
		//				glcd_set_x_address(x1+vc_len*i+k%nb);
		//				glcd_set_y_address(y1+j);
		//				k++;
		//			}
		//		}while(k < vl_len - 1);
	}
}
void NOKIA5110::glcd_big_str(uint8_t x, uint8_t y, char const *str)
{
	int i, k, length = strlen(str);
	uint8_t x1, y1;

	x1 = x;
	y1 = y;

	int vl_len = 23;			// vector line lenght			22+1 = 23 the first is the character size
	int vc_len = (vl_len-1)/2;	// graph character line length	11;

	for(i=0;i<length;i++)
	{
		glcd_set_x_address(x1+vc_len*i);	// 0, 11, 23, 35
		glcd_set_y_address(y1);
		for(k=0;k<vc_len;k++)
		{
			glcd_data(Arial2[vl_len*(str[i]-32)+2*k+1]);
		}

		glcd_set_x_address(x1+vc_len*i);
		glcd_set_y_address(y1+1);
		for(k=0;k<vc_len;k++)
		{
			glcd_data(Arial2[vl_len*(str[i]-32)+2*k+2]);
		}

	}
//	glcd_data(0x00);					// print some 1x8 vertical blank space;
//	for(i=0;i<5;i++)					// this font has 5 cols length of 1 byte each;
//	{
//		glcd_data(FontNew[(c-32)*5+i]);
//	}



//	glcd.glcd_set_x_address(0);
//	glcd.glcd_set_y_address(0);
//	glcd.glcd_data(0x00);
//	for(i=0;i<11;i++)
//	{
//		glcd.glcd_data(Arial[23*3+2*i+1]);
//	}
//
//	glcd.glcd_set_x_address(1);
//	glcd.glcd_set_y_address(1);
//	for(i=0;i<11;i++)
//	{
//		glcd.glcd_data(Arial[23*3+2*i+2]);
//	}
//
//	glcd.glcd_set_x_address(12);
//	glcd.glcd_set_y_address(0);
//	glcd.glcd_data(0x00);
//	for(i=0;i<11;i++)
//	{
//		glcd.glcd_data(Arial[23*2+2*i+1]);
//	}
//
//	glcd.glcd_set_x_address(13);
//	glcd.glcd_set_y_address(1);
//	for(i=0;i<11;i++)
//	{
//		glcd.glcd_data(Arial[23*2+2*i+2]);
//	}
//
//	glcd.glcd_set_x_address(26);
//	glcd.glcd_set_y_address(0);
//	glcd.glcd_data(0x00);
//	for(i=0;i<11;i++)
//	{
//		glcd.glcd_data(Arial[23*1+2*i+1]);
//	}
//
//	glcd.glcd_set_x_address(27);
//	glcd.glcd_set_y_address(1);
//	for(i=0;i<11;i++)
//	{
//		glcd.glcd_data(Arial[23*1+2*i+2]);
//	}
}



#endif /* NOKIA5110_NOKIA5110_H_ */
