#ifndef KS0066_H__
#define KS0066_H__

#include "gpio_driver.h"

/* KS0066/ST7066U(5C) controller for LCD. 
* It uses 4-bit bus and 8-bit bus are selected by the DL bit in the instruction register.
* DR - Data Register (8-bit)
* IL - Instruction Register (8-bit)
*
* Class implemented for 4-bit interface mode
*
*	RS = 0 to ir,	RS = 1 to DDRAM or CGRAM
*	RW = 0 (write), RW = 1 (read);
*
* 	RS	RW		Operation Description
*	0	0		ir write as an internal operation (display clear, etc.)
*	0	1		Read busy flag (DB7) and address counter (DB0 to DB7)
*	1	0		Write data to DDRAM or CGRAM (DR to DDRAM or CGRAM)
*	1	1		Read data from DDRAM or CGRAM (DDRAM or CGRAM to DR)
*/

/*	Register Select (RS)
*	0 - Instruction Register (ir)
*	1 - Data Register (DR) */
enum class reg_t {
	ir = 0,
	dr
};
/*	Operation Type (RW)
*	0 - Write
*	1 - Read */
enum class op_t {
	write = 0,
	read
};

// Instruction for KS0066/ST0065U/ST0066C - Function Set (FS)
#define KS0066_FS_CLEAR_DISPLAY		0x01
#define KS0066_FS_RETURN_HOME		0x02
#define KS0066_FS_ENTRY_MODE_SET	0x08	// Bits [1:0] - Inc: I/D=0, Dec: I/D=1, Shift S=1 or don't Shif S=0;
#define KS0066_FS_01 				0x03	// First init functions set (FS)
#define KS0066_FS_02 				0x02	// 
#define KS0066_FS_DISPLAY_ON		0x0D	// Turn on display and cursor;
#define KS0066_FS_DISPLAY_CONTROL	0x08	// 0b000001DCB, D=0, display off, D=1, display on; C=0, cursor off, C=1, cursor on;
#define KS0066_EN_PULSE_WIDTH		2		// enable pulse width [us]. Must be more than 450 ns;
#define KS0066_EN_RISE_FALL_TIME	1		// time to enable pulse rise/fall [us];
#define KS0066_DATA_HOLD_TIME		1		// data hold time after pulse enable go low [us];

/*
* 0b00011NFX		N = 0, 1 line mode, N = 1, 2;
* 0b000001DC		
* 0b00000001		
*/

class KS0066 {
public:
	KS0066(int pin_en, int pin_rs, int pin_rw, int pin_d7, int pin_d6, int pin_d5, int pin_d4) : pin_{{pin_en, 1}, {pin_rs, 1}, {pin_rw, 1}, {pin_d7, 1}, {pin_d6, 1}, {pin_d5, 1}, {pin_d4, 1}} {
		pin_en_ = pin_en;
		pin_rs_ = pin_rs;
		pin_rw_ = pin_rw;
		pin_d7_ = pin_d7;
		pin_d6_ = pin_d6;
		pin_d5_ = pin_d5;
		pin_d4_ = pin_d4;
	}
	~KS0066(void) {}

	void init(void) {
		// wait 15 ms or more after Vdd reaches 4.5 V
		delay_ms(50);

		// Function set
		rs_(reg_t::ir);
		rw_(op_t::write);
		d7_(0);
		d6_(0);
		d5_(1);
		d4_(1);
		delay_us(50);

		// Function set
		d4_(0);
		delay_us(40);

		// Function set
		d4_(0);
		delay_us(40);

		clear_display();
		delay_ms(2);
	}
	/*
	* @brief Display control command.
	* @param d display on/off;
	* @param c cursor on/off;
	* @param b blink on/off;
	*/	
	void control_display(int d, int c, int b) {
		uint8_t fs = 0x00;

		fs |= ((d << 2) | (c<<1) | (b<<0));
		rs_(reg_t::ir);
		rw_(op_t::write);
		write_byte(fs);
	}
	void clear_display(void) {
		rs_(reg_t::ir);
		rw_(op_t::write);
		write_nibble(KS0066_FS_CLEAR_DISPLAY);
		en_pulse_();
	}
	void return_home(void) {
		rs_(reg_t::ir);
		rw_(op_t::write);
		write_byte(KS0066_FS_RETURN_HOME);
	}
	void entry_mode_set(int d, int s) {
		rs_(reg_t::ir);
		rw_(op_t::write);
		write_byte(KS0066_FS_ENTRY_MODE_SET | (static_cast<uint8_t>(d) << 1) | (static_cast<uint8_t>(s) << 0));	// 0b0000 01ds
	}
	/*	@brief Will set nibble from D[7:4]
	* 	@param cmd nibble to be set.
	*	@return no return
	*/
	void write_nibble(uint8_t cmd) {		
		d7_((cmd >> 3) & 0x01);
		d6_((cmd >> 2) & 0x01);
		d5_((cmd >> 1) & 0x01);
		d4_((cmd >> 0) & 0x01);

		en_pulse_();
	}
	void write_byte(uint8_t cmd) {
		write_nibble(cmd>>4);
		write_nibble(cmd);
	}
	int busy_flag(void) {
		rs_(reg_t::ir);
		rw_(op_t::read);
		return d7_();
	}

private:
	void en_pulse_(void) {
		en_(0);
		delay_us(KS0066_EN_RISE_FALL_TIME);
		en_(1);
		delay_us(KS0066_EN_PULSE_WIDTH);
		en_(0);
		delay_us(KS0066_DATA_HOLD_TIME);
	}
	void en_(int state) {
		pin_[0].write(state);
	}
	void rs_(reg_t reg) {
		pin_[1].write(static_cast<int>(reg));
	}
	void rw_(op_t op) {
		pin_[2].write(static_cast<int>(op));
	}
	void d7_(int state) {
		pin_[3].write(state);
	}
	void d6_(int state) {
		pin_[4].write(state);
	}
	void d5_(int state) {
		pin_[5].write(state);
	}
	void d4_(int state) {
		pin_[6].write(state);
	}
	int d7_(void) {
		pin_[3].mode(0);
		int value = pin_[6].read();
		pin_[3].mode(1);
		return value;
	}

	int pin_en_, pin_rs_, pin_rw_;			// control pins
	int pin_d4_, pin_d5_, pin_d6_, pin_d7_;	// data pins

	GPIO_DRIVER pin_[7];					// en, rs, rw, d7, d6, d5 and d4
};

#endif
