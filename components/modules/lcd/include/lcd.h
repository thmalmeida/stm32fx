#ifndef LCD_H__
#define LCD_H__

#include "gpio_driver.h"
#include "ks0066.h"

#define PIN_RS			27
#define PIN_RW			28
#define PIN_EN			29
#define PIN_D4			30
#define PIN_D5			31
#define PIN_D6			32
#define PIN_D7			13

#define NUMBER_OF_LINES	2

#define LCD_DEBUG 1

class LCD_DRIVER {
public:
	LCD_DRIVER(void) : controller_(PIN_RS, PIN_RW, PIN_EN, PIN_D4, PIN_D5, PIN_D6, PIN_D7) {
		init();
		// controller_.test_pins();
	}
	~LCD_DRIVER(void) {}

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
