#ifndef TIME_H
#define TIME_H

#include "types.h"

#define MMIO_BASE 0xFE000000
#define SYSTEM_TIMER_BASE (MMIO_BASE + 0x3000)

#define CS 0x00
#define CLO 0x04
#define CHI 0x08
#define C0 0x0C
#define C1 0x10
#define C2 0x14
#define C3 0x18

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

void time_test(void);
void time_init(void);
u64 time_get_boot_time_tick(void);

void time_setup_timer_tick(void);

#define TIME_STYLE_HHMMSSMS 0
#define TIME_STYLE_SSMS 1

#define TIME_STYLE_HHMMSSMS_BUF_SIZE 10
#define TIME_STYLE_SSMS_BUF_SIZE 10

int time_format_time(char *buf, u64 time, u32 style);

void time_sleep_ms(u64 ms);
void time_sleep_us(u64 us);

#endif /* TIME_H */
