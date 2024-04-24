/*
 * lcdt.h
 *
 *  Created on: 5 de out de 2016
 *      Author: titi
 */

#ifndef LCDT_H_
#define LCDT_H_

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif

 // LCD CLASS
 // commands
 #define lcdt_CLEARDISPLAY 0x01
 #define lcdt_RETURNHOME 0x02
 #define lcdt_ENTRYMODESET 0x04
 #define lcdt_DISPLAYCONTROL 0x08
 #define lcdt_CURSORSHIFT 0x10
 #define lcdt_FUNCTIONSET 0x20
 #define lcdt_SETCGRAMADDR 0x40
 #define lcdt_SETDDRAMADDR 0x80

 // flags for display entry mode
 #define lcdt_ENTRYRIGHT 0x00
 #define lcdt_ENTRYLEFT 0x02
 #define lcdt_ENTRYSHIFTINCREMENT 0x01
 #define lcdt_ENTRYSHIFTDECREMENT 0x00

 // flags for display on/off control
 #define lcdt_DISPLAYON 0x04
 #define lcdt_DISPLAYOFF 0x00
 #define lcdt_CURSORON 0x02
 #define lcdt_CURSOROFF 0x00
 #define lcdt_BLINKON 0x01
 #define lcdt_BLINKOFF 0x00

 // flags for display/cursor shift
 #define lcdt_DISPLAYMOVE 0x08
 #define lcdt_CURSORMOVE 0x00
 #define lcdt_MOVERIGHT 0x04
 #define lcdt_MOVELEFT 0x00

 // flags for function set
 #define lcdt_8BITMODE 0x10
 #define lcdt_4BITMODE 0x00
 #define lcdt_2LINE 0x08
 #define lcdt_1LINE 0x00
 #define lcdt_5x10DOTS 0x04
 #define lcdt_5x8DOTS 0x00

 uint8_t _row_offsets[4];
 uint8_t _displayfunction;
 uint8_t _displaycontrol;
 uint8_t _displaymode;
 uint8_t _initialized;
 uint8_t _numlines;

 #define rs_ddr		DDRC
 #define rs_port	PORTC
 #define rs_pin		5

 #define en_ddr		DDRC
 #define en_port	PORTC
 #define en_pin		4

 #define d4_ddr		DDRC
 #define d4_port	PORTC
 #define d4_pin		3

 #define d5_ddr		DDRC
 #define d5_port	PORTC
 #define d5_pin		2

 #define d6_ddr		DDRC
 #define d6_port	PORTC
 #define d6_pin		1

 #define d7_ddr		DDRC
 #define d7_port	PORTC
 #define d7_pin		0

 #define rs_on()	rs_port |=  (1 << rs_pin);
 #define rs_off()	rs_port &= ~(1 << rs_pin);
 #define en_on()	en_port |=  (1 << en_pin);
 #define en_off()	en_port &= ~(1 << en_pin);
 #define d4_on()	d4_port |=  (1 << d4_pin);
 #define d4_off()	d4_port &= ~(1 << d4_pin);
 #define d5_on()	d5_port |=  (1 << d5_pin);
 #define d5_off()	d5_port &= ~(1 << d5_pin);
 #define d6_on()	d6_port |=  (1 << d6_pin);
 #define d6_off()	d6_port &= ~(1 << d6_pin);
 #define d7_on()	d7_port |=  (1 << d7_pin);
 #define d7_off()	d7_port &= ~(1 << d7_pin);

 void lcdt_rs_conf(uint8_t dir);
 void lcdt_en_conf(uint8_t dir);
 void lcdt_d4_conf(uint8_t dir);
 void lcdt_d5_conf(uint8_t dir);
 void lcdt_d6_conf(uint8_t dir);
 void lcdt_d7_conf(uint8_t dir);
 void lcdt_rs(uint8_t status);
 void lcdt_en(uint8_t status);
 void lcdt_d4(uint8_t status);
 void lcdt_d5(uint8_t status);
 void lcdt_d6(uint8_t status);
 void lcdt_d7(uint8_t status);
 void lcdt_pulseEnable(void);
 void lcdt_write4bits(uint8_t value);
 void lcdt_send(uint8_t value, uint8_t mode);
 inline void lcdt_command(uint8_t value);
 void lcdt_write(uint8_t value);
 void lcdt_begin(uint8_t cols, uint8_t lines);
 void lcdt_print(uint8_t col, uint8_t row, const char* s);
 void lcdt_clear();
 void lcdt_home();
 void lcdt_setCursor(uint8_t col, uint8_t row);

#ifdef __cplusplus
}
#endif


#endif /* LCDT_H_ */
