#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "types.h"
#include "arch/gic.h"

void interrupt_init(void);
void handle_irq(void);

int interrupt_register_handler(u32 irq, irq_handler_t handler, void *data);
void interrupt_unregister_handler(u32 irq);
void interrupt_enable(u32 irq);
void interrupt_disable(u32 irq);
void interrupt_set_type(u32 irq, irq_type_t type);

#endif // INTERRUPT_H
