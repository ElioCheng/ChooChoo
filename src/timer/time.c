#include "timer/time.h"
#include "arch/rpi.h"
#include "printf.h"
#include "interrupt.h"
#include "arch/interrupts.h"
#include "arch/gic.h"
#include "event.h"
#include "klog.h"
#include "uart.h"

static u64 time_last_tick = 0;
static u64 time_boot_tick = 0;
static u32 timer_tick_count = 0;

static void timer_tick_handler(u32 irq, void *data)
{
	(void)data;
	timer_tick_count++;

	klog_debug("Timer tick interrupt %u (count: %u)", irq, timer_tick_count);

	SYSTEM_TIMER_REG(CS) = (1 << 1);

	u32 current_time = SYSTEM_TIMER_REG(CLO);
	SYSTEM_TIMER_REG(C1) = current_time + 10000;

	event_unblock_waiting_tasks(EVENT_TIMER_TICK, timer_tick_count);
}

void time_setup_timer_tick(void)
{
	u32 current_time = SYSTEM_TIMER_REG(CLO);

	if (interrupt_register_handler(IRQ_SYSTEM_TIMER_1, timer_tick_handler, NULL) == 0) {
		interrupt_set_type(IRQ_SYSTEM_TIMER_1, IRQ_TYPE_LEVEL_HIGH);

		interrupt_enable(IRQ_SYSTEM_TIMER_1);

		SYSTEM_TIMER_REG(CS) = (1 << 1);

		current_time = SYSTEM_TIMER_REG(CLO);
		SYSTEM_TIMER_REG(C1) = current_time + 10000;

		klog_info("Timer C1 interrupt configured for 10ms intervals (IRQ %u)", IRQ_SYSTEM_TIMER_1);
	} else {
		klog_error("Failed to register timer tick interrupt handler");
	}
}

void time_test(void)
{
	if (TICK_TO_MS(time_get_boot_time_tick() - time_last_tick) >= 2) {
		time_last_tick = time_get_boot_time_tick();
		char buf[TIME_STYLE_HHMMSSMS_BUF_SIZE];
		time_format_time(buf, time_last_tick, TIME_STYLE_HHMMSSMS);
		uart_printf(CONSOLE, 1024, "Time test: %s\r", buf);
	}
}

void time_init(void)
{
	time_last_tick = TIME_GET_TICK_64();
	time_boot_tick = TIME_GET_TICK_64();
}

u64 time_get_boot_time_tick(void)
{
	return TIME_GET_TICK_64() - time_boot_tick;
}

// format time to string, the buffer must be large enough to hold the string
int time_format_time(char *buf, u64 tick, u32 style)
{
	u64 seconds = TICK_TO_S(tick);
	u64 milliseconds = TICK_TO_MS(tick);
	u64 minutes = seconds / 60;
	u64 hours = minutes / 60;

	int ret = 0;

	switch (style) {
	case TIME_STYLE_HHMMSSMS:
		seconds = seconds % 60;
		minutes = minutes % 60;
		milliseconds = milliseconds % 1000;
		ret = snprintf(buf, 20, "%02d:%02d:%02d.%03d", (int)hours, (int)minutes, (int)seconds,
			       (int)milliseconds);
		break;
	case TIME_STYLE_SSMS:
	default:
		milliseconds = milliseconds % 1000;
		ret = snprintf(buf, 20, "%5d.%03d", (int)seconds, (int)milliseconds);
		break;
	}

	return ret;
}

// TODO: REMOVE ALL THE CALLS TO SLEEP
void time_sleep_ms(u64 ms)
{
	u64 start = TIME_GET_TICK_64();
	while (TICK_TO_MS(TIME_GET_TICK_64() - start) < ms) {
		asm volatile("nop");
	}
}

void time_sleep_us(u64 us)
{
	u64 start = TIME_GET_TICK_64();
	while (TICK_TO_US(TIME_GET_TICK_64() - start) < us) {
		asm volatile("nop");
	}
}
