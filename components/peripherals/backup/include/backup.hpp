#ifndef __BACKUP_HPP__
#define __BACKUP_HPP__

#include "stdint.h"
#include "system_main.h"

void backup_DR1_set(uint16_t value);
uint16_t backup_DR1_get(void);

#endif