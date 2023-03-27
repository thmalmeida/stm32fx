#include "reset_reason.hpp"

reset_reason_t reset_reason(void) {
	reset_reason_t r;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
		r = reset_reason_t::LOW_POWER_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
		r = reset_reason_t::WINDOW_WATCHDOG_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		r = reset_reason_t::INDEPENDENT_WATCHDOG_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
		r = reset_reason_t::SOFTWARE_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
		r = reset_reason_t::POWER_ON_POWER_DOWN_RESET;
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		r = reset_reason_t::EXTERNAL_RESET_PIN_RESET;
	}
	else {
		r = reset_reason_t::UNKNOW;
	}

	return r; 
}

void reset_clear_flags(void) {
	// set RMVF bit 24;
	RCC->CSR |= (1<<24);

	// or
	// Clear all the reset flags or else they will remain set during future
	// resets until system power is fully removed.
	__HAL_RCC_CLEAR_RESET_FLAGS();
}
