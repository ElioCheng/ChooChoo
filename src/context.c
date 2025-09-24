#include "context.h"

void context_init(context_t *context, void *stack_top, void *entry_point)
{
	memset(context, 0, sizeof(context_t));
	REG_SP(context->regs) = (u64)stack_top;
	REG_PC(context->regs) = (u64)entry_point;
	REG_SPSR(context->regs) = 0x0;
	REG_ELR(context->regs) = (u64)entry_point;
}
