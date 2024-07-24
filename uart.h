#pragma once

#include <stdint.h>
#include <stddef.h>
#include <driverlib.h>

size_t uart_write(int handle, const unsigned char *buf, size_t bufSize);
void uart_printf(char* format, ...);