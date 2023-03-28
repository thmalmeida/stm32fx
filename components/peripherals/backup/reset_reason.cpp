#include "reset_reason.hpp"

static uint8_t reset_reason_read_flag = 0;
static reset_reason_t reset_reason_last = reset_reason_t::UNKNOW;

static void reset_reason_clear_flags(void) {
	// set RMVF bit 24;
	// RCC->CSR |= (1<<24);
	// or
	// Clear all the reset flags or else they will remain set during future
	// resets until system power is fully removed.
	__HAL_RCC_CLEAR_RESET_FLAGS();
}
reset_reason_t reset_reason(void) {
	// All theses flags is located at RCC->CSR register
	if(!reset_reason_read_flag) {
		if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
			reset_reason_last = reset_reason_t::LOW_POWER_RESET;
		}
		else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
			reset_reason_last = reset_reason_t::WINDOW_WATCHDOG_RESET;
		}
		else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
			reset_reason_last = reset_reason_t::INDEPENDENT_WATCHDOG_RESET;
		}
		else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
			reset_reason_last = reset_reason_t::SOFTWARE_RESET;
		}
		else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
			reset_reason_last = reset_reason_t::POWER_ON_POWER_DOWN_RESET;
		}
		else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
			reset_reason_last = reset_reason_t::EXTERNAL_RESET_PIN_RESET;
		}
		else {
			reset_reason_last = reset_reason_t::UNKNOW;
		}

		reset_reason_read_flag = 1;
		reset_reason_clear_flags();
	}

	return reset_reason_last;
}
