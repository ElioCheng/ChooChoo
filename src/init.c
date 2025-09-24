#include "arch/cpu.h"
#include "klog.h"
#include "arch/rpi.h"
#include "timer/timer.h"
#include "compiler.h"
#include "uart.h"
#include "timer/time.h"
#include "symbol.h"
#include "boot_test.h"
#include "arch/exception.h"
#include "sched.h"
#include "interrupt.h"

#define KLOG_DEFAULT_DESTINATIONS (KLOG_DEST_CONSOLE | KLOG_DEST_MEMORY)

extern void setup_mmu();

extern char __user_task_start[];

extern char __bss_start[];
extern char __bss_end[];

int __noreturn kmain(void)
{
#if defined(MMU)
	setup_mmu();
#endif

	memset(__bss_start, 0, __bss_end - __bss_start);

	time_init();

	gpio_init();

	uart_config_and_enable(CONSOLE);
	uart_config_and_enable(MARKLIN);

	klog_init(KLOG_DEFAULT_DESTINATIONS);
	uart_puts(CONSOLE, "\033[2J\033[H");
	klog_info("Kernel started");

	symbol_init();

	exception_init();

	timer_subsystem_init();

	task_init();

	sched_init();

	boot_test();

	interrupt_init();

	uart_init_interrupts();

	time_setup_timer_tick();

	task_t *test_task = task_create((void *)__user_task_start, 0);

	if (test_task) {
		sched_add_task(test_task);
	}

	klog_info("Init task created, switching to user space");
	uart_process_tx_buffers_blocking();

	klog_set_destinations(KLOG_DEST_MEMORY);

	sched_schedule();

	panic("kmain: should not be reached");
	UNREACHABLE();
}
