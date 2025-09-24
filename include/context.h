#ifndef __CONTEXT_H__
#define __CONTEXT_H__

#include "arch/registers.h"
#include "string.h"

typedef struct context {
	struct arch_regs regs;
} context_t;

void context_init(context_t *context, void *stack_top, void *entry_point);
#endif /* __CONTEXT_H__ */
