#ifndef __UAPI_LOG_H__
#define __UAPI_LOG_H__

#include "io.h"
#include "types.h"
#include "string.h"
#include <stdarg.h>

#ifndef LOG_MODULE
#error "LOG_MODULE is not defined"
#endif

typedef enum {
	LOG_LEVEL_DEBUG = 0,
	LOG_LEVEL_INFO = 1,
	LOG_LEVEL_WARN = 2,
	LOG_LEVEL_ERROR = 3,
	LOG_LEVEL_FATAL = 4,
	LOG_LEVEL_NONE = 5
} log_level_t;

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

static const char *log_level_strings[] = { "DEBUG", "INFO ", "WARN ", "ERROR", "FATAL" };

#define log_should_print(level) (level >= LOG_LEVEL)

static inline void _log_write(log_level_t level, const char *module, const char *func, int line, const char *fmt, ...)
{
	if (!log_should_print(level)) {
		return;
	}

	va_list args;
	va_start(args, fmt);

	char message_buffer[1024];
	__raw_vsnprintf(message_buffer, sizeof(message_buffer), fmt, &args);
	va_end(args);

#ifdef ENABLE_BUSY_WAIT_DEBUG
	char log_buffer[1200];
	sprintf(log_buffer, "[%s][%s][%s:%d] %s\r\n", log_level_strings[level], module, func, line, message_buffer);
	busy_wait_console_puts(log_buffer);
#else
	console_printf("[%s][%s][%s:%d] %s\r\n", log_level_strings[level], module, func, line, message_buffer);
#endif
}

#define LOG_DEBUG(fmt, ...)                                                                              \
	do {                                                                                             \
		if (log_should_print(LOG_LEVEL_DEBUG)) {                                                 \
			_log_write(LOG_LEVEL_DEBUG, LOG_MODULE, __func__, __LINE__, fmt, ##__VA_ARGS__); \
		}                                                                                        \
	} while (0)

#define LOG_INFO(fmt, ...)                                                                              \
	do {                                                                                            \
		if (log_should_print(LOG_LEVEL_INFO)) {                                                 \
			_log_write(LOG_LEVEL_INFO, LOG_MODULE, __func__, __LINE__, fmt, ##__VA_ARGS__); \
		}                                                                                       \
	} while (0)

#define LOG_WARN(fmt, ...)                                                                              \
	do {                                                                                            \
		if (log_should_print(LOG_LEVEL_WARN)) {                                                 \
			_log_write(LOG_LEVEL_WARN, LOG_MODULE, __func__, __LINE__, fmt, ##__VA_ARGS__); \
		}                                                                                       \
	} while (0)

#define LOG_ERROR(fmt, ...)                                                                              \
	do {                                                                                             \
		if (log_should_print(LOG_LEVEL_ERROR)) {                                                 \
			_log_write(LOG_LEVEL_ERROR, LOG_MODULE, __func__, __LINE__, fmt, ##__VA_ARGS__); \
		}                                                                                        \
	} while (0)

#define LOG_FATAL(fmt, ...)                                                                              \
	do {                                                                                             \
		if (log_should_print(LOG_LEVEL_FATAL)) {                                                 \
			_log_write(LOG_LEVEL_FATAL, LOG_MODULE, __func__, __LINE__, fmt, ##__VA_ARGS__); \
		}                                                                                        \
	} while (0)

#define log_debug(fmt, ...) LOG_DEBUG(fmt, ##__VA_ARGS__)
#define log_info(fmt, ...) LOG_INFO(fmt, ##__VA_ARGS__)
#define log_warn(fmt, ...) LOG_WARN(fmt, ##__VA_ARGS__)
#define log_error(fmt, ...) LOG_ERROR(fmt, ##__VA_ARGS__)
#define log_fatal(fmt, ...) LOG_FATAL(fmt, ##__VA_ARGS__)

#endif // __UAPI_LOG_H__
