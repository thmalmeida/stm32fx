#include "backup.hpp"

static uint8_t bkp_en = 0;

static void backup_enable(void) {
	// Enable power and bkp interface clock setting PWREN and BKPEN
	RCC->APB1ENR |= (1<<28) | (1<<27);

	// set the DBP bit in the Power control register (PWR_CR) to enable access to the Backup registers and RTC.
	// Enable access to BKP register setting DBP bit
	PWR->CR |= (1<<8);
}
static void backup_disable(void) {
	if(bkp_en) {
		PWR->CR &= ~(1<<8);
		bkp_en = 0;
	}
}
void backup_DR1_set(uint16_t value) {
	if(!bkp_en) {
		backup_enable();
		bkp_en = 1;
	}
	BKP->DR1 = value;
	backup_disable();
}
uint16_t backup_DR1_get(void) {
	return BKP->DR1;
}
