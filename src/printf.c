#include "printf.h"
#include "compiler.h"
#include "arch/rpi.h"
#include "types.h"
#include "stdarg.h"
#include <stdint.h>

#define HEX_BASE 16
#define OCT_BASE 8
#define DEC_BASE 10
#define FLAGS_ZERO 1u << 0
#define FLAGS_LONG 1u << 1
#define FLAGS_LONG_LONG 1u << 2
#define FLAGS_PREFIX_0x 1u << 3
#define FLAGS_LEFT 1u << 4

#define isdigit(c) ((c) >= '0' && (c) <= '9')

// ascii digit to integer
int a2d(const char ch)
{
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	if (ch >= 'a' && ch <= 'f')
		return ch - 'a' + 10;
	if (ch >= 'A' && ch <= 'F')
		return ch - 'A' + 10;
	return -1;
}

/**
 * ascii string to unsigned int, with base
 * @param ch - current character,
 * @param src - pointer to the current position in the string
 * @param base - base to convert to
 * @param nump - pointer to the number to convert
 * @return - next character after the number
 */
char a2ui(char ch, char **src, unsigned int base, unsigned int *nump)
{
	unsigned int num;
	int digit;
	char *p;

	p = *src;
	num = 0;
	while ((digit = a2d(ch)) >= 0) {
		if ((unsigned int)digit > base)
			break;
		// Check for overflow
		if (num > (UINT_MAX - digit) / base) {
			return -1; // Overflow occurred
		}
		num = num * base + digit;
		ch = *p++;
	}
	*src = p;
	*nump = num;
	return ch;
}

// unsigned int to ascii string, with base
void ui2a(unsigned int num, unsigned int base, char *buf, int flags, int precision)
{
	unsigned int n = 0;
	unsigned int d = 1;
	char prefix[3] = { 0 };
	int prefix_len = 0;
	char temp_buf[32]; // Temporary buffer for number conversion
	int temp_len = 0;

	// Handle prefix for hex
	if ((flags & FLAGS_PREFIX_0x) && base == HEX_BASE) {
		prefix[0] = '0';
		prefix[1] = 'x';
		prefix_len = 2;
	}

	// Calculate divisor
	while ((num / d) >= base) {
		if (d > UINT_MAX / base) {
			break; // Prevent overflow
		}
		d *= base;
	}

	// Convert number to temporary buffer
	while (d != 0) {
		unsigned int dgt = num / d;
		num %= d;
		d /= base;
		if (n || dgt > 0 || d == 0) {
			temp_buf[temp_len++] = dgt + (dgt < 10 ? '0' : 'a' - 10);
			n++;
		}
	}

	// Add prefix if needed
	if (prefix_len > 0) {
		for (int i = 0; i < prefix_len; i++) {
			*buf++ = prefix[i];
		}
	}

	// Handle precision padding
	if (precision > 0) {
		int padding = precision - temp_len;
		if (padding > 0) {
			for (int i = 0; i < padding; i++) {
				*buf++ = '0';
			}
		}
	}

	// Copy the number
	for (int i = 0; i < temp_len; i++) {
		*buf++ = temp_buf[i];
	}
	*buf = 0;
}

// signed int to ascii string
void i2a(int num, char *buf, int flags, int precision)
{
	char sign = 0;
	if (num < 0) {
		sign = '-';
		num = -num;
	}

	// Convert the number to a temporary buffer
	char temp[32];
	ui2a(num, 10, temp, flags, precision);

	// Copy the number to the output buffer
	if (sign) {
		*buf++ = sign;
	}

	// Copy the number
	char *p = temp;
	while (*p) {
		*buf++ = *p++;
	}
	*buf = '\0';
}

static void __buf_putc(char **p, const char *end, char c, int *idx)
{
	if (*p < end) {
		*(*p)++ = c;
	}
	if (idx) {
		(*idx)++;
	}
}

static void __buf_puts(char **p, const char *end, const char *buf, int *idx, int precision, int width, int flags)
{
	int count = 0;
	int len = 0;
	const char *tmp = buf;
	char sign = 0;

	// Check for sign character
	if (*tmp == '-' || *tmp == '+' || *tmp == ' ') {
		sign = *tmp;
		tmp++;
	}

	// Calculate length
	while (*tmp != '\0' && (precision == -1 || len < precision)) {
		len++;
		tmp++;
	}

	// Add sign to total length
	if (sign) {
		len++;
	}

	// Handle right padding (default)
	if (!(flags & FLAGS_LEFT) && width > len) {
		int padding = width - len;
		char pad_char = (flags & FLAGS_ZERO) ? '0' : ' ';

		// If zero padding and we have a sign, output sign first
		if (pad_char == '0' && sign) {
			__buf_putc(p, end, sign, idx);
			sign = 0; // Clear sign as we've output it
		}

		while (padding-- > 0) {
			__buf_putc(p, end, pad_char, idx);
		}
	}

	// Output sign if not already output
	if (sign) {
		__buf_putc(p, end, sign, idx);
	}

	// Output the string
	while (*buf != '\0' && (precision == -1 || count < precision)) {
		__buf_putc(p, end, *buf, idx);
		buf++;
		count++;
	}

	// Handle left padding
	if ((flags & FLAGS_LEFT) && width > len) {
		int padding = width - len;
		while (padding-- > 0) {
			__buf_putc(p, end, ' ', idx);
		}
	}
}

static int __handle_fmt(char **pstr, const char *end, size_t buf_size, int index, const char **pfmt, va_list *p_args)
{
	char buf[buf_size];
	const char *fmt = *pfmt;
	int idx = index;
	char ch;
	int flags = 0;
	int width = 0;
	int precision = -1;

	// Parse flags
	while (1) {
		ch = *(fmt++);
		switch (ch) {
		case '0':
			flags |= FLAGS_ZERO;
			continue;
		case '-':
			flags |= FLAGS_LEFT;
			continue;
		case '#':
			flags |= FLAGS_PREFIX_0x;
			continue;
		default:
			break;
		}
		break;
	}

	// Parse width
	if (ch == '*') {
		width = va_arg(*p_args, int);
		if (width < 0) {
			flags |= FLAGS_LEFT;
			width = -width;
		}
		ch = *(fmt++);
	} else if (isdigit(ch)) {
		width = ch - '0';
		ch = *(fmt++);
		while (isdigit(ch)) {
			width = width * 10 + (ch - '0');
			ch = *(fmt++);
		}
	}

	// Parse precision
	if (ch == '.') {
		ch = *(fmt++);
		if (ch == '*') {
			precision = va_arg(*p_args, int);
			if (precision < 0) {
				precision = -1; // Negative precision is ignored
			}
			ch = *(fmt++);
		} else {
			precision = 0;
			if (isdigit(ch)) {
				precision = ch - '0';
				ch = *(fmt++);
				while (isdigit(ch)) {
					precision = precision * 10 + (ch - '0');
					ch = *(fmt++);
				}
			}
		}
	}

	// Parse length modifier
	if (ch == 'l') {
		ch = *(fmt++);
		if (ch == 'l') {
			flags |= FLAGS_LONG_LONG;
			ch = *(fmt++);
		} else {
			flags |= FLAGS_LONG;
		}
	}

	// Handle format specifier
	switch (ch) {
	case 'u':
		if (flags & FLAGS_LONG_LONG) {
			ui2a(va_arg(*p_args, unsigned long long), 10, buf, flags, precision);
		} else if (flags & FLAGS_LONG) {
			ui2a(va_arg(*p_args, unsigned long), 10, buf, flags, precision);
		} else {
			ui2a(va_arg(*p_args, unsigned int), 10, buf, flags, precision);
		}
		__buf_puts(pstr, end, buf, &idx, -1, width, flags);
		break;
	case 'd':
	case 'i':
		if (flags & FLAGS_LONG_LONG) {
			i2a(va_arg(*p_args, long long), buf, flags, precision);
		} else if (flags & FLAGS_LONG) {
			i2a(va_arg(*p_args, long), buf, flags, precision);
		} else {
			i2a(va_arg(*p_args, int), buf, flags, precision);
		}
		__buf_puts(pstr, end, buf, &idx, -1, width, flags);
		break;
	case 'x':
	case 'X':
		if (flags & FLAGS_LONG_LONG) {
			ui2a(va_arg(*p_args, unsigned long long), 16, buf, flags, precision);
		} else if (flags & FLAGS_LONG) {
			ui2a(va_arg(*p_args, unsigned long), 16, buf, flags, precision);
		} else {
			ui2a(va_arg(*p_args, unsigned int), 16, buf, flags, precision);
		}
		__buf_puts(pstr, end, buf, &idx, -1, width, flags);
		break;
	case 'o':
		if (flags & FLAGS_LONG_LONG) {
			ui2a(va_arg(*p_args, unsigned long long), 8, buf, flags, precision);
		} else if (flags & FLAGS_LONG) {
			ui2a(va_arg(*p_args, unsigned long), 8, buf, flags, precision);
		} else {
			ui2a(va_arg(*p_args, unsigned int), 8, buf, flags, precision);
		}
		__buf_puts(pstr, end, buf, &idx, -1, width, flags);
		break;
	case 'b':
		ui2a(va_arg(*p_args, unsigned int), 2, buf, flags, precision);
		__buf_puts(pstr, end, buf, &idx, -1, width, flags);
		break;
	case 'c':
		__buf_putc(pstr, end, (char)va_arg(*p_args, int), &idx);
		break;
	case 's':
		__buf_puts(pstr, end, va_arg(*p_args, char *), &idx, precision, width, flags);
		break;
	case 'p':
		flags |= FLAGS_PREFIX_0x;
		precision = 8;
		ui2a((unsigned long)va_arg(*p_args, void *), 16, buf, flags, precision);
		__buf_puts(pstr, end, buf, &idx, -1, width, flags);
		break;
	case '%':
		__buf_putc(pstr, end, '%', &idx);
		break;
	default:
		break;
	}

	*pfmt = fmt;
	return idx;
}

// format string to buffer with a size limit, return the number of characters written
int __raw_vsnprintf(char *buf, size_t size, const char *fmt, va_list *p_args)
{
	if (!buf || !fmt || size == 0) {
		return -1;
	}

	char *str, *end;
	const char *f = fmt;
	int idx = 0;

	str = buf;
	end = buf + size - 1; // Leave room for null terminator

	while (*f != '\0') {
		if (*f != '%') {
			__buf_putc(&str, end, *f, &idx);
			f++;
		} else {
			f++;
			idx = __handle_fmt(&str, end, size, idx, &f, p_args);
		}
	}

	*str = '\0';
	return idx;
}

int __attribute__((format(printf, 2, 3))) sprintf(char *buf, const char *fmt, ...)
{
	va_list args;
	int r;

	va_start(args, fmt);
	r = __raw_vsnprintf(buf, SIZE_MAX, fmt, &args);
	va_end(args);

	return r;
}

int __attribute__((format(printf, 3, 4))) snprintf(char *buf, size_t size, const char *fmt, ...)
{
	if (size == 0) {
		return -1;
	}

	va_list args;
	int r;

	va_start(args, fmt);
	r = __raw_vsnprintf(buf, size, fmt, &args);
	va_end(args);

	return r;
}

// void __maybe_unused format_reference()
// {
// 	uart_printf(CONSOLE, 1024, "[42][%d]\r\n", 42); // "42"
// 	uart_printf(CONSOLE, 1024, "[   42][%5d]\r\n", 42); // "   42"
// 	uart_printf(CONSOLE, 1024, "[42   ][%-5d]\r\n", 42); // "42   "
// 	uart_printf(CONSOLE, 1024, "[00042][%05d]\r\n", 42); // "00042"

// 	// Hexadecimal formatting
// 	uart_printf(CONSOLE, 1024, "[2a][%x]\r\n", 42); // "2a"
// 	uart_printf(CONSOLE, 1024, "[0x2a][%#x]\r\n", 42); // "0x2a"
// 	uart_printf(CONSOLE, 1024, "[2a][%X]\r\n", 42); // "2a"

// 	// Octal formatting
// 	uart_printf(CONSOLE, 1024, "[52][%o]\r\n", 42); // "52"

// 	// Long integer formatting
// 	uart_printf(CONSOLE, 1024, "[42][%ld]\r\n", 42L); // "42"
// 	uart_printf(CONSOLE, 1024, "[42][%lld]\r\n", 42LL); // "42"

// 	// String and character
// 	uart_printf(CONSOLE, 1024, "[hello][%s]\r\n", "hello"); // "hello"
// 	uart_printf(CONSOLE, 1024, "[A][%c]\r\n", 'A'); // "A"
// 	// Pointer
// 	void *var = (void *)0x12345678;
// 	void *var2 = (void *)0x1234;
// 	uart_printf(CONSOLE, 1024, "[0x12345678][%p]\r\n", var); // "0x12345678"
// 	uart_printf(CONSOLE, 1024, "[0x00001234][%p]\r\n", var2); // "0x00001234"
// 	// String precision examples
// 	uart_printf(CONSOLE, 1024, "[hel][%.3s]\r\n", "hello"); // "hel"
// 	uart_printf(CONSOLE, 1024, "[hi][%.5s]\r\n", "hi"); // "hi"
// 	uart_printf(CONSOLE, 1024, "[][%.0s]\r\n", "test"); // ""

// 	// Integer precision examples
// 	uart_printf(CONSOLE, 1024, "[00042][%.5d]\r\n", 42); // "00042"
// 	uart_printf(CONSOLE, 1024, "[12345][%.3d]\r\n", 12345); // "12345"
// 	uart_printf(CONSOLE, 1024, "[0][%.0d]\r\n", 0); // "0"

// 	// Hexadecimal precision examples
// 	uart_printf(CONSOLE, 1024, "[001a][%.4x]\r\n", 0x1A); // "001a"
// 	uart_printf(CONSOLE, 1024, "[1a][%.2x]\r\n", 0x1A); // "1a"
// 	uart_printf(CONSOLE, 1024, "[0x001a][%#.4x]\r\n", 0x1A); // "0x001a"

// 	// Octal precision examples
// 	uart_printf(CONSOLE, 1024, "[0052][%.4o]\r\n", 42); // "0052"
// 	uart_printf(CONSOLE, 1024, "[52][%.2o]\r\n", 42); // "52"

// 	// Long integer precision examples
// 	uart_printf(CONSOLE, 1024, "[00042][%.5ld]\r\n", 42L); // "00042"
// 	uart_printf(CONSOLE, 1024, "[12345][%.3ld]\r\n", 12345L); // "12345"
// }
