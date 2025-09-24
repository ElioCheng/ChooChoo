#ifndef PRINTF_H
#define PRINTF_H

#include <stdarg.h>
#include <stddef.h>

// ascii digit to integer
int a2d(const char ch);

// ascii string to unsigned int, with base
char a2ui(char ch, char **src, unsigned int base, unsigned int *nump);

// unsigned int to ascii string, with base
void ui2a(unsigned int num, unsigned int base, char *buf, int flags, int precision);

// signed int to ascii string
void i2a(int num, char *buf, int flags, int precision);

// format string to buffer with a size limit, return the number of characters written
int __raw_vsnprintf(char *buf, size_t size, const char *fmt, va_list *p_args);

// format string to buffer, return the number of characters written
int sprintf(char *buf, const char *fmt, ...);

// format string to buffer with a size limit, return the number of characters written
int snprintf(char *buf, size_t size, const char *fmt, ...);

#endif /* printf.h */
