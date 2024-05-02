#include "stm32_log.h"

/* System calls from syscalls.c file. Modified by thmalmeida on 20240426 */

/* Variables */
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

// Needs segger host link to read works 20240502
// volatile int32_t ITM_RxBuffer=ITM_RXBUFFER_EMPTY; // Initialize as EMPTY

char *__env[1] = { 0 };
char **environ = __env;

/* Functions */
void initialise_monitor_handles() {
}
int _close(int file) {
	// if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
	// return 0;

	// errno = EBADF;
	(void)file;
	return -1;
}
void _exit (int status) {
	_kill(status, -1);
	while (1) {}    /* Make sure we hang here */
}
int _fstat(int file, struct stat* st) {
	// if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
	// 	st->st_mode = S_IFCHR;
	// 	return 0;
	// }

	// errno = EBADF;
	(void)file;
	st->st_mode = S_IFCHR;

	return 0;
}
int _getpid(void) {
	return 1;
}
int _isatty(int file) {
	//   if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
	//     return 1;

	//   errno = EBADF;
	(void)file;
	return 1;
}
int _kill(int pid, int sig) {
	(void)pid;
	(void)sig;
	errno = EINVAL;
	return -1;
}
int _lseek(int file, int ptr, int dir) {
	(void)file;
	(void)ptr;
	(void)dir;

	return -1;
}
int _open(char *path, int flags, ...) {
	(void)path;
	(void)flags;
	/* Pretend like we always fail */
	return -1;
}
int _read(int file, char* ptr, int len) {
    // for (int DataIdx = 0; DataIdx < len; DataIdx++) {
    //     *ptr++ = ITM_ReceiveChar();
    // }
	return len;

	// using UART??
	// HAL_StatusTypeDef hstatus;
	// if (fd == STDIN_FILENO) {
	// hstatus = HAL_UART_Receive(gHuart, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
	// if (hstatus == HAL_OK)
	// return 1;
	// else
	// return EIO;
	// }
	// errno = EBADF;
}
int _times(struct tms *buf) {
	(void)buf;
	return -1;
}
int _unlink(char *name) {
  (void)name;
  errno = ENOENT;
  return -1;
}
int _write(int file, char *ptr, int len) {
	for (int i = 0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return (len);
}
int _gettimeofday(struct timeval *__restrict__ tp, void *__restrict__ tzp) {
	return 0;
}


// int _write(int32_t file, uint8_t *ptr, int32_t len)
// int _write(int fd, char* ptr, int len) {
//   HAL_StatusTypeDef hstatus;

//   if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
//     hstatus = HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
//     if (hstatus == HAL_OK)
//       return len;
//     else
//       return EIO;
//   }
//   errno = EBADF;
//   return -1;
// }