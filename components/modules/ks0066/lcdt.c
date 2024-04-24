/*
 * lcdt.c
 *
 *  Created on: 5 de out de 2016
 *      Author: titi
 */
#include "lcdt.h"

void lcdt_rs_conf(uint8_t dir)
{
	if(dir)
	{
		rs_ddr |=  (1 << rs_pin);
	}
	else
	{
		rs_ddr &= ~(1 << rs_pin);
	}
}
void lcdt_en_conf(uint8_t dir)
{
	if(dir)
	{
		en_ddr |=  (1 << en_pin);
	}
	else
	{
		en_ddr &= ~(1 << en_pin);
	}
}
void lcdt_d4_conf(uint8_t dir)
{
	if(dir)
	{
		d4_ddr |=  (1 << d4_pin);
	}
	else
	{
		d4_ddr &= ~(1 << d4_pin);
	}
}
void lcdt_d5_conf(uint8_t dir)
{
	if(dir)
	{
		d5_ddr |=  (1 << d5_pin);
	}
	else
	{
		d5_ddr &= ~(1 << d5_pin);
	}
}
void lcdt_d6_conf(uint8_t dir)
{
	if(dir)
	{
		d6_ddr |=  (1 << d6_pin);
	}
	else
	{
		d6_ddr &= ~(1 << d6_pin);
	}
}
void lcdt_d7_conf(uint8_t dir)
{
	if(dir)
	{
		d7_ddr |=  (1 << d7_pin);
	}
	else
	{
		d7_ddr &= ~(1 << d7_pin);
	}
}
void lcdt_rs(uint8_t status)
{
	if(status)
	{
		rs_on();
	}
	else
	{
		rs_off();
	}
}
void lcdt_en(uint8_t status)
{
	if(status)
	{
		en_on();
	}
	else
	{
		en_off();
	}
}
void lcdt_d4(uint8_t status)
{
	if(status)
	{
		d4_on();
	}
	else
	{
		d4_off();
	}
}
void lcdt_d5(uint8_t status)
{
	if(status)
	{
		d5_on();
	}
	else
	{
		d5_off();
	}
}
void lcdt_d6(uint8_t status)
{
	if(status)
	{
		d6_on();
	}
	else
	{
		d6_off();
	}
}
void lcdt_d7(uint8_t status)
{
	if(status)
	{
		d7_on();
	}
	else
	{
		d7_off();
	}
}

void lcdt_pulseEnable(void)
{
	lcdt_en(0);
	_delay_us(1);
	lcdt_en(1);
	_delay_us(1);		// enable pulse must be >450ns
	lcdt_en(0);
	_delay_us(100);		// commands need > 37us to settle
}
void lcdt_write4bits(uint8_t value)
{
	lcdt_d4((value >> 0) & 0x01);
	lcdt_d5((value >> 1) & 0x01);
	lcdt_d6((value >> 2) & 0x01);
	lcdt_d7((value >> 3) & 0x01);

	lcdt_pulseEnable();
}
void lcdt_send(uint8_t value, uint8_t mode)
{
	lcdt_rs(mode);

	lcdt_write4bits(value>>4);
	lcdt_write4bits(value);
}
void lcdt_write(uint8_t value)
{
	lcdt_send(value, 1);
}
void lcdt_command(uint8_t value)
{
	lcdt_send(value, 0);
}
void lcdt_setRowOffsets(int row0, int row1, int row2, int row3)
{
	_row_offsets[0] = row0;
	_row_offsets[1] = row1;
	_row_offsets[2] = row2;
	_row_offsets[3] = row3;
}
void lcdt_display()
{
	_displaycontrol |= lcdt_DISPLAYON;
	lcdt_command(lcdt_DISPLAYCONTROL | _displaycontrol);
}
void lcdt_begin(uint8_t cols, uint8_t lines)//, uint8_t dotsize)
{
    _displayfunction = lcdt_4BITMODE | lcdt_2LINE | lcdt_5x8DOTS;

    _numlines = lines;

	lcdt_setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);

	lcdt_rs_conf(1);
	lcdt_en_conf(1);

	lcdt_d4_conf(1);
	lcdt_d5_conf(1);
	lcdt_d6_conf(1);
	lcdt_d7_conf(1);

	// this is according to the hitachi HD44780 datasheet
	// page 45 figure 23

	// we start in 8bit mode, try to set 4 bit mode
	lcdt_write4bits(0x03);
	_delay_us(4500); // wait min 4.1ms

	// second try
	lcdt_write4bits(0x03);
	_delay_us(4500); // wait min 4.1ms

	// third go!
	lcdt_write4bits(0x03);
	_delay_us(150);

	// finally, set to 4-bit interface
	lcdt_write4bits(0x02);

	// done!
	// finally, set # lines, font size, etc.
	lcdt_command(lcdt_FUNCTIONSET | _displayfunction);

	// turn the display on with no cursor or blinking default
	_displaycontrol = lcdt_DISPLAYON | lcdt_CURSOROFF | lcdt_BLINKOFF;
	lcdt_display();

	// clear it off
	lcdt_clear();

	// Initialize to default text direction (for romance languages)
	_displaymode = lcdt_ENTRYLEFT | lcdt_ENTRYSHIFTDECREMENT;
	// set the entry mode
	lcdt_command(lcdt_ENTRYMODESET | _displaymode);
}
void lcdt_clear()
{
	lcdt_command(lcdt_CLEARDISPLAY);  // clear display, set cursor position to zero
	_delay_ms(2);  // this command takes a long time!
}
void lcdt_home()
{
	lcdt_command(lcdt_RETURNHOME);  // set cursor position to zero
	_delay_ms(2);  // this command takes a long time!
}
void lcdt_setCursor(uint8_t col, uint8_t row)
{
	const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
	if ( row >= max_lines )
	{
		row = max_lines - 1;    // we count rows starting w/0
	}

	if ( row >= _numlines )
	{
		row = _numlines - 1;    // we count rows starting w/0
	}
	lcdt_command(lcdt_SETDDRAMADDR | (col + _row_offsets[row]));
}
void lcdt_print(uint8_t col, uint8_t row, const char* s)
{
	lcdt_setCursor(col, row);

	while(*s)
	{
		lcdt_write(*s);
		s++;
	}
//	for(uint8_t i = 0; string[i] != limit; i++)
//		lcdt_write(string[i]);
}





