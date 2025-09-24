#ifndef CPU_H
#define CPU_H

#include "types.h"

static inline u64 get_cpu_id(void)
{
	u64 cpu_id;
	asm volatile("mrs %0, mpidr_el1" : "=r" (cpu_id));
	cpu_id = (cpu_id & 0xFF) >> 4;
	return cpu_id;
}

static inline u64 get_current_el()
{
	u64 el;
	asm volatile("mrs %0, CurrentEL" : "=r"(el));
	el = (el >> 2) & 0x3;
	return el;
}

static inline u64 get_sp()
{
	u64 sp;
	asm volatile("mov %0, sp" : "=r"(sp));
	return sp;
}

#endif
