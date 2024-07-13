#ifndef KS0066_HPP__
#define KS0066_HPP__

#include "gpio_driver.hpp"
#include "delay.hpp"

/* by thmalmeida@gmail.com on 20240315
* last modified:
* 20240501
*
*/

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
/* Read/Write pin 
*	0 - Disable pin_rw and is write to lcd only
*	1 - Enable pin_rw and is able to read from or write to lcd
*/
enum class rw_pin_t {
	disable = 0,
	enable
};


// Instruction for KS0066/ST0065U/ST0066C - Function Set (FS)
#define KS0066_CLEAR_DISPLAY		0x01
#define KS0066_RETURN_HOME			0x02
#define KS0066_ENTRY_MODE_SET		0x04		// Bits [1:0] - Inc: I/D=0, Dec: I/D=1, Shift S=1 or don't Shif S=0;
#define KS0066_FUNCTION_SET			0x20		// 0b001 DL - N F X X. DL (8-bit/4-bit); N=0, 1 line, N=1, 2 line mode; F font type (5x11/5x8dots) or F=0 display off, F=1 display on;
#define KS0066_DISPLAY_CONTROL		0x08		// 0b000001DCB, D=0, display off, D=1, display on; C=0, cursor off, C=1, cursor on;
#define KS0066_CGRAM_ADDR_SET		0x40		// 0  1 A5 A4 A3 A2 A1 A0 (6-bit address)
#define KS0066_DDRAM_ADDR_SET		0x80		// 1 A6 A5 A4 A3 A2 A1 A0 (7-bits address)
#define KS0066_FIRST_LINE_ADDR		0x00
#define KS0066_SECOND_LINE_ADDR		0x40

#define KS0066_EN_PULSE_WIDTH		2			// enable pulse width [us]. Must be more than 450 ns;
#define KS0066_EN_RISE_FALL_TIME	2			// time to enable pulse rise/fall [us];
#define KS0066_DATA_HOLD_TIME		40			// data hold time after pulse enable go low [us];
#define KS0066_DDRAM_SET_TIME		40

// #define KS0066_PIN_RW			1			// comment this line if PIN_RW is not used and is always grounded

#define KS0066_DEBUG				1

class KS0066 {
public:

	#ifdef KS0066_PIN_RW
	KS0066(int pin_rs, int pin_rw, int pin_en, int pin_d4, int pin_d5, int pin_d6, int pin_d7) : pin_{{pin_rs, 1}, {pin_rw, 1}, {pin_en, 1}, {pin_d4, 1}, {pin_d5, 1}, {pin_d6, 1}, {pin_d7, 1}} {
		pin_en_ = pin_en;
		pin_rs_ = pin_rs;
		pin_rw_ = pin_rw;
		pin_d7_ = pin_d7;
		pin_d6_ = pin_d6;
		pin_d5_ = pin_d5;
		pin_d4_ = pin_d4;
	}
	#else
	KS0066(int pin_rs, int pin_en, int pin_d4, int pin_d5, int pin_d6, int pin_d7) : pin_{{pin_rs, 1}, {pin_en, 1}, {pin_d4, 1}, {pin_d5, 1}, {pin_d6, 1}, {pin_d7, 1}} {
		pin_en_ = pin_en;
		pin_rs_ = pin_rs;
		pin_d7_ = pin_d7;
		pin_d6_ = pin_d6;
		pin_d5_ = pin_d5;
		pin_d4_ = pin_d4;
	}	
	#endif
	~KS0066(void) {}

	void init(uint8_t columns, uint8_t lines) {
		columns_ = columns;
		lines_ = lines;

		// After powr on, wait 15 ms or more until Vdd reaches 4.5 V
		delay_ms(50);			// wait for more than 30 ms;
		#ifdef KS0066_DEBUG
		printf("KS0066 delay after power on\n");
		#endif

		// Function 4 bit set with N and F
		set_4bit(1, 0);			// N=0 is 1-line, N=1 is 2-line mode;
		delay_us(50);			// wait for more than 39 us;
		#ifdef KS0066_DEBUG
		printf("KS0066 set 4-bit mode\n");
		#endif

		// Function set - display on/off control
		control_display(1, 0, 0);		
		delay_us(40);			// wait for more than 39 us
		#ifdef KS0066_DEBUG
		printf("KS0066 control display\n");
		#endif

		// display clear
		clear_display();
		delay_ms(2);			// wait for more than 1.53 ms
		#ifdef KS0066_DEBUG
		printf("KS0066 clear display\n");
		#endif		

		// entry mode set
		entry_mode_set(1, 0);	// I/D = 1, increment mode. SH=0 shift off;
		#ifdef KS0066_DEBUG
		printf("KS0066 entry mode set\n");
		#endif		
	}
	// void test_pins(void) {
	// 	while(1) {
	// 		rs_(reg_t::dr);
	// 		rw_(op_t::write);
	// 		printf("write\n");
	// 		d4_(1);
	// 		d5_(1);
	// 		d6_(1);
	// 		d7_(1);
	// 		printf("1\n");
	// 		delay_ms(2000);
	// 		rs_(reg_t::ir);
	// 		rw_(op_t::read);
	// 		printf("read\n");
	// 		en_(0);
	// 		d4_(0);
	// 		d5_(0);
	// 		d6_(0);
	// 		d7_(0);
	// 		printf("0\n");
	// 		delay_ms(2000);			
	// 	}
	// }
	/* @brief 4-bit mode
	* @param n N=0 is 1-line, N=1 is 2-line mode;
	* @param f F=0 5×8; F=1 5x10 character font. 
	*/
	void set_4bit(uint8_t n, uint8_t f) {
		rs_(reg_t::ir);
		#ifdef KS0066_PIN_RW
		rw_(op_t::write);
		#endif

		uint8_t fs = KS0066_FUNCTION_SET;
		fs |= (n<<3) | (f<<1);
		// write_nibble_(0x02);		// write 0010
		write_byte_(fs);			// write 0b0010-NFXX, where N=0 is 1-line mode, N=1 is 2-line mode. F=0, display off. F=1 display on;
	}
	/* @brief Display control command.
	* 0b00001DCB
	* @param d display on/off;
	* @param c cursor on/off;
	* @param b blink on/off;
	*/	
	void control_display(uint8_t d, uint8_t c, uint8_t b) {
		uint8_t fs = KS0066_DISPLAY_CONTROL;
		fs |= ((d << 2) | (c<<1) | (b<<0));

		rs_(reg_t::ir);
		#ifdef KS0066_PIN_RW
		rw_(op_t::write);
		#endif
		write_byte_(fs);
	}
	void clear_display(void) {
		rs_(reg_t::ir);
		#ifdef KS0066_PIN_RW
		rw_(op_t::write);
		#endif
		write_byte_(KS0066_CLEAR_DISPLAY);
		en_pulse_();
	}
	void return_home(void) {
		rs_(reg_t::ir);
		#ifdef KS0066_PIN_RW
		rw_(op_t::write);
		#endif
		write_byte_(KS0066_RETURN_HOME);
	}
	#ifdef KS0066_PIN_RW
	// not implemented yet
	char read(int line, int column) {
		return 'A';
	}
	#endif
	void write(char c) {
		rs_(reg_t::dr);
		#ifdef KS0066_PIN_RW
		rw_(op_t::write);
		#endif
		write_byte_(c);
	}
	void print(char *str) {
		rs_(reg_t::dr);
		#ifdef KS0066_PIN_RW
		rw_(op_t::write);
		#endif

		while(*str) {
			write_byte_(*str);
			str++;
		}
	}
	/* @brief entry mode set
	* 0b0000-01 I/D SH
	* @param d 0-decrement, 1-increment
	* @param s SH- 0 shift off, 1 shift on
	*/
	void entry_mode_set(uint8_t d, uint8_t s) {
		uint8_t fs = KS0066_ENTRY_MODE_SET;
		fs |= ((d << 1) | (s<<0));		// 0b0000-01 I/D S

		rs_(reg_t::ir);
		#ifdef KS0066_PIN_RW
		rw_(op_t::write);
		#endif
		write_byte_(fs);
	}
	/* @brief set cursor position
	* @param l line position
	* @param c column position
	*/
	void position_set(uint8_t l, uint8_t c) {
		uint8_t line_addr = l*KS0066_SECOND_LINE_ADDR;

		rs_(reg_t::ir);
		#ifdef KS0066_PIN_RW
		rw_(op_t::write);
		#endif
		write_byte_(KS0066_DDRAM_ADDR_SET | line_addr | c);
		delay_us(KS0066_DDRAM_SET_TIME);
	}
private:
	/* @brief Will set nibble from D[7:4]
	* 	@param cmd nibble to be set.
	*	@return no return
	*/
	void write_nibble_(uint8_t cmd) {		
		d7_((cmd >> 3) & 0x01);
		d6_((cmd >> 2) & 0x01);
		d5_((cmd >> 1) & 0x01);
		d4_((cmd >> 0) & 0x01);

		en_pulse_();
	}
	void write_byte_(uint8_t cmd) {
		write_nibble_(cmd>>4);
		write_nibble_(cmd);
	}
	#ifdef KS0066_PIN_RW
	int busy_flag_(void) {
		rs_(reg_t::ir);
		rw_(op_t::read);
		return d7_();
	}
	#endif
	void en_pulse_(void) {
		en_(0);
		delay_us(KS0066_EN_RISE_FALL_TIME);
		en_(1);
		delay_us(KS0066_EN_PULSE_WIDTH);
		en_(0);
		delay_us(KS0066_DATA_HOLD_TIME);
	}
	void rs_(reg_t reg) {
		pin_[0].write(static_cast<int>(reg));
	}
	#ifdef KS0066_PIN_RW
	void rw_(op_t op) {
		pin_[1].write(static_cast<int>(op));
	}
	#endif
	void en_(int state) {
		#ifdef KS0066_PIN_RW
		pin_[2].write(state);
		#else
		pin_[1].write(state);
		#endif
	}
	void d4_(int state) {
		#ifdef KS0066_PIN_RW
		pin_[3].write(state);
		#else
		pin_[2].write(state);
		#endif
	}
	void d5_(int state) {
		#ifdef KS0066_PIN_RW
		pin_[4].write(state);
		#else
		pin_[3].write(state);
		#endif
	}
	void d6_(int state) {
		#ifdef KS0066_PIN_RW
		pin_[5].write(state);
		#else
		pin_[4].write(state);
		#endif
	}
	void d7_(int state) {
		#ifdef KS0066_PIN_RW
		pin_[6].write(state);
		#else
		pin_[5].write(state);
		#endif
	}
	#ifdef KS0066_PIN_RW
	int d7_(void) {
		pin_[6].mode(0);
		int value = pin_[6].read();
		pin_[6].mode(1);
		
		return value;
	}
	#endif

	uint8_t columns_, lines_;

	int pin_d4_, pin_d5_, pin_d6_, pin_d7_;	// data pins
	int pin_en_, pin_rs_;					// control pins
	#ifdef KS0066_PIN_RW
	int pin_rw_;							// rw control pin

	GPIO_Driver pin_[7];					// rs, rw, en, d4, d5, d6 and d7
	#else

	GPIO_Driver pin_[6];					// rs, en, d4, d5, d6 and d7
	#endif
};

#endif




	// void init2(void) {

	// 	rs_(reg_t::ir);
	// 		#ifdef KS0066_PIN_RW
		// rw_(op_t::write);
		// #endif

	// 	write_nibble_(0x03);
	// 	delay_ms(5);

	// 	write_nibble_(0x03);
	// 	delay_ms(5);

	// 	write_nibble_(0x03);
	// 	delay_us(150);

	// 	write_nibble_(0x02);

	// 	write_byte_(0x20 | 0x00 | 0x08);	// 0b0010-1000

	// 	write_byte_(0x08 | 0x04);			// 0b0000-1100

	// 	write_byte_(0x01);					// 0b0000-0001

	// 	write_byte_(0x04 | 0x02);			// 0b0000-0110

	// }