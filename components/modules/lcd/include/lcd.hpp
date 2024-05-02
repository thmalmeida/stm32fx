#ifndef LCD_HPP__
#define LCD_HPP__

#include "gpio_driver.hpp"
#include "ks0066.hpp"

#define PIN_D7			32
#define PIN_D6			31
#define PIN_D5			30
#define PIN_D4			29
#define PIN_EN			28
// #define PIN_RW			28
#define PIN_RS			27



#define NUMBER_OF_LINES	2

#define LCD_DEBUG 1

class LCD_Driver {
public:
	#ifdef KS0066_PIN_RW
	LCD_Driver(void) : controller_(PIN_RS, PIN_RW, PIN_EN, PIN_D4, PIN_D5, PIN_D6, PIN_D7) {
		init();
		// controller_.test_pins();
	}
	LCD_Driver(int pin_rs, int pin_rw, int pin_en, int pin_d4, int pin_d5, int pin_d6, int pin_d7) : controller_(pin_rs, pin_rw, pin_en, pin_d4, pin_d5, pin_d6, pin_d7) {
		init();

	char read(int line, int column) {
		return controller_.read(line, column);
	}
	}
	#else
	LCD_Driver(void) : controller_(PIN_RS, PIN_EN, PIN_D4, PIN_D5, PIN_D6, PIN_D7) {
		init();
	}
	// leak pin rw and ground it
	LCD_Driver(int pin_rs, int pin_en, int pin_d4, int pin_d5, int pin_d6, int pin_d7) : controller_(pin_rs, pin_en, pin_d4, pin_d5, pin_d6, pin_d7) {
		init();
	}
	#endif
	~LCD_Driver(void) {}

	void write(char c) {
		controller_.write(c);
	}
	void write(char c, int line, int column) {
		controller_.position_set(line, column);
		write(c);
	}
	void print(char *str) {
		controller_.print(str);
	}
	void print(char *str, int line, int column) {
		controller_.position_set(line, column);
		print(str);
	}
	void clear(void) {
		controller_.clear_display();
	}
	void home(void) {
		controller_.return_home();
	}
	void position(uint8_t x, uint8_t y) {
		controller_.position_set(x, y);
	}
	void init(void) {
		controller_.init(16, 2);
		printf("LCD init\n");
	}

private:
	KS0066 controller_;
};

#endif
