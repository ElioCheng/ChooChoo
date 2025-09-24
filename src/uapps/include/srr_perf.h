#ifndef __UAPPS_SRR_PERF_H__
#define __UAPPS_SRR_PERF_H__

#include "types.h"

// Walkaround for the lack of a time server in the kernel
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

void srr_perf_main(void);

#endif /* __UAPPS_SRR_PERF_H__ */
