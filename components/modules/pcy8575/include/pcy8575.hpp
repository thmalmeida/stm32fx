#ifndef _PCY8575_HPP__
#define _PCY8575_HPP__

#include "i2c.h"
#include "stm32_log.h"

/* list of I2C addresses */
#define PCY8575_ADDR						0x23	// device address

class pcy8575 {

	public:
	
	pcy8575(void);
    ~pcy8575(void) {};

    void init(void);
    void handle_message(void);

    private:

};

#endif //#ifndef _PCY8575_H__