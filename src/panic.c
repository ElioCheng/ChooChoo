#include "exception.h"
#include "klog.h"
#include "printf.h"
#include "symbol.h"
#include "uart.h"
#include "timer/time.h"

void do_panic(const char *func, const char *line, const char *fmt, ...)
{
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	__raw_vsnprintf(buf, sizeof(buf), fmt, &args);
	va_end(args);

	uart_process_tx_buffers_blocking();
	klog_set_destinations(KLOG_DEST_CONSOLE);
	klog_force(KLOG_PANIC, func, line, "%s", buf);
	uart_process_tx_buffers_blocking();

	dump_current_context(1);
	uart_process_tx_buffers_blocking();

	asm volatile("b .");
}
