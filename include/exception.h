#ifndef EXCEPTION_H
#define EXCEPTION_H

#include "context.h"
#include "types.h"

void dump_data(unsigned long addr_start, unsigned long addr_end, int force);
void dump_around_address(u64 address, size_t size, int force);
void dump_around_pc(int force);
void dump_backtrace(int force);
void dump_registers(int force);
void dump_current_context(int force);
void dump_context(context_t *context, int force);

#endif
