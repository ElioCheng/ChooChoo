#ifndef UAPI_KLOG_H
#define UAPI_KLOG_H

#include "types.h"
#include "syscall.h"
#include "printf.h"

#define KLOG_NONE 0
#define KLOG_PANIC 1
#define KLOG_ERROR 2
#define KLOG_WARNING 3
#define KLOG_INFO 4
#define KLOG_DEBUG 5

static inline int klog(u8 level, const char *fmt, ...)
{
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	int ret = __raw_vsnprintf(buf, sizeof(buf), fmt, &args);
	va_end(args);
	KLog(level, buf);
	return ret;
}

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define klog_panic(fmt, ...) klog(KLOG_PANIC, fmt, ##__VA_ARGS__)
#define klog_error(fmt, ...) klog(KLOG_ERROR, fmt, ##__VA_ARGS__)
#define klog_warning(fmt, ...) klog(KLOG_WARNING, fmt, ##__VA_ARGS__)
#define klog_warn(fmt, ...) klog(KLOG_WARNING, fmt, ##__VA_ARGS__)
#define klog_info(fmt, ...) klog(KLOG_INFO, fmt, ##__VA_ARGS__)
#define klog_debug(fmt, ...) klog(KLOG_DEBUG, fmt, ##__VA_ARGS__)

#endif /* UAPI_KLOG_H */
