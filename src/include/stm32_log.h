// All credit to Carmine Noviello for this code
// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f030R8/system/include/retarget/retarget.h

#ifndef _RETARGET_H__
#define _RETARGET_H__

#include <stdio.h>
#include <sys/stat.h>

#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>


#include "stm32f1xx_hal.h"
// #include <errno.h>

// void RetargetInit(UART_HandleTypeDef *huart);
int _close(int file);
void _exit (int status);
int _fstat(int file, struct stat* st);
int _getpid(void);
int _isatty(int file);
int _kill(int pid, int sig);
int _lseek(int file, int ptr, int dir);
int _open(char *path, int flags, ...);
int _read(int file, char* ptr, int len);
int _times(struct tms *buf);
int _unlink(char *name);
int _write(int file, char* ptr, int len);

#endif //#ifndef _RETARGET_H__