#ifndef __RESET_REASON_HPP__
#define __RESET_REASON_HPP__

#include "stdint.h"
#include "system_main.h"

// these bits is located into RCC->CSR register
enum class reset_reason_t {
	UNKNOW = 0,
	LOW_POWER_RESET,				// Low-power management reset (LPWRRSTF bit 31)
	WINDOW_WATCHDOG_RESET,			// Window watchdog reset flag (WWDGRSTF bit 30)
	INDEPENDENT_WATCHDOG_RESET,		// Independent watchdog reset flag (IWDGRSTF bit 29)
	SOFTWARE_RESET,					// Software reset flag (SFTRST bit 28)
	POWER_ON_POWER_DOWN_RESET,		// POR/PDR reset flag (PORRSTF bit 27)
	EXTERNAL_RESET_PIN_RESET		// External Reset from NRST pin (PINRSTF bit 26)
};

reset_reason_t reset_reason(void);
void reset_clear_flags(void);

#endif