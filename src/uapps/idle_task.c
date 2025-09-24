#include "idle.h"
#include "idle_task.h"
#include "printf.h"
#include "syscall.h"
#include "io.h"

static idle_stats_t idle_stats = { 0 };

static u64 last_report_time_ms = 0;
static const u64 REPORT_INTERVAL_MS = 500;
static char idle_report_buf[1024];

static u64 report_count = 0;

// Walkaround for the lack of a time server in the kernel
#define MMIO_BASE 0xFE000000
#define SYSTEM_TIMER_BASE (MMIO_BASE + 0x3000)

#define CS 0x00
#define CLO 0x04
#define CHI 0x08

#define SYSTEM_TIMER_REG(reg) (*(volatile u32 *)(SYSTEM_TIMER_BASE + reg))

#define TIME_FREQ 1000000 // 1MHz

static inline u64 time_get_tick_64(void)
{
	u32 hi = SYSTEM_TIMER_REG(CHI);
	u32 lo = SYSTEM_TIMER_REG(CLO);

	if (SYSTEM_TIMER_REG(CLO) < lo) {
		// CLO wrapped around between reading CHI and CLO
		hi = SYSTEM_TIMER_REG(CHI);
	}
	return ((u64)hi << 32) | lo;
}

#define TIME_GET_TICK_64() (time_get_tick_64())
#define TIME_GET_TICK_US() (TIME_GET_TICK_64())
#define TIME_GET_TICK_MS() (TIME_GET_TICK_64() / (TIME_FREQ / 1000))
#define TIME_GET_TICK_S() (TIME_GET_TICK_64() / TIME_FREQ)

#define TICK_TO_US(t) ((t) * 1000000 / TIME_FREQ)
#define TICK_TO_MS(t) ((t) * 1000 / TIME_FREQ)
#define TICK_TO_S(t) ((t) / TIME_FREQ)

static inline void report_stats(void)
{
	if (!idle_stats.display_enabled) {
		return;
	}

	snprintf(idle_report_buf, sizeof(idle_report_buf),
		 "\033[?25l" // Hide cursor
		 "\033[s" // Save cursor position
		 "\033[H" // Move to top-left
		 "\033[K" // Clear line
		 "CPU Usage: %llu %% [window: %llu ms] [count: %llu]"
		 "\033[u" // Restore cursor position
		 "\033[?25h", // Show cursor
		 100 - idle_stats.idle_percentage, IDLE_STATS_WINDOW_MS, report_count);

	console_puts(idle_report_buf);
	report_count++;
}

void __noreturn idle_task_main(void)
{
	report_count = 0;
	SetupIdleTask(&idle_stats);
	last_report_time_ms = TIME_GET_TICK_MS();

	while (1) {
		u64 current_time = TIME_GET_TICK_MS();
		if (current_time - last_report_time_ms >= REPORT_INTERVAL_MS) {
			report_stats();
			last_report_time_ms = current_time;
		}
		asm volatile("wfi" ::: "memory");
	}
}
