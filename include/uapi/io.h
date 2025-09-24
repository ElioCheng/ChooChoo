#ifndef __UAPI_IO_H__
#define __UAPI_IO_H__

#include "types.h"
#include "string.h"
#include "printf.h"

#define IO_CHANNEL_CONSOLE 1
#define IO_CHANNEL_MARKLIN 2
#define IO_SERVER_NAME "io_server"

#define IO_REQ_PUTN_MAX_LEN 1024 * 1024 // 1MB

typedef enum {
	IO_REQ_GETC,
	IO_REQ_PUTC,
	IO_REQ_PUTN,
	IO_REQ_TRYGETC,
	IO_REQ_RX_NOTIFY,
	IO_REQ_TX_NOTIFY,
	IO_REQ_CTS_NOTIFY
} io_request_type_t;

#define IO_SUCCESS 0
#define IO_ERROR -1
#define IO_BLOCKED -2
#define IO_NO_DATA -3

typedef struct {
	io_request_type_t type;
	int channel;
	union {
		struct {
		} getc;
		struct {
			unsigned char ch;
		} putc;
		struct {
			const char str[IO_REQ_PUTN_MAX_LEN];
			size_t len;
		} putn;
		struct {
		} trygetc;
		struct {
			int channel;
		} notify;
	};
} io_request_t;

typedef struct {
	int result;
} io_reply_t;

int Getc(int tid, int channel);
int TryGetc(int tid, int channel);
int Putc(int tid, int channel, unsigned char ch);
int Putn(int tid, int channel, const char *str, size_t len);

static inline int uart_putc(int line, char c)
{
	return Putc(-1, line, c);
}

static inline int uart_getc(int line)
{
	return Getc(-1, line);
}

static inline int console_putc(char c)
{
	return uart_putc(IO_CHANNEL_CONSOLE, c);
}

static inline int console_getc(void)
{
	return uart_getc(IO_CHANNEL_CONSOLE);
}

static inline int console_trygetc(void)
{
	return TryGetc(-1, IO_CHANNEL_CONSOLE);
}

static inline int console_puts(const char *str)
{
	return Putn(-1, IO_CHANNEL_CONSOLE, str, strlen(str));
}

static inline int console_printf(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char buffer[500 * 1024];
	__raw_vsnprintf(buffer, sizeof(buffer), fmt, &args);

	int ret = Putn(-1, IO_CHANNEL_CONSOLE, buffer, strlen(buffer));

	va_end(args);

	return ret;
}

static inline int marklin_putc(char c)
{
	return uart_putc(IO_CHANNEL_MARKLIN, c);
}

static inline int marklin_getc(void)
{
	return uart_getc(IO_CHANNEL_MARKLIN);
}

static inline int marklin_trygetc(void)
{
	return TryGetc(-1, IO_CHANNEL_MARKLIN);
}

#ifdef ENABLE_BUSY_WAIT_DEBUG
int busy_wait_console_putc(char c);
int busy_wait_console_puts(const char *str);
int busy_wait_marklin_putc(char c);
#endif

#endif
