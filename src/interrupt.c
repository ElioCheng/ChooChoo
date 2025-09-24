#include "interrupt.h"
#include "arch/gic.h"
#include "klog.h"
#include "context.h"
#include "task.h"
#include "sched.h"

extern u8 from_exception;

void interrupt_init(void)
{
	gic_init();
	klog_info("Interrupt subsystem initialized");
}

void handle_irq(void)
{
	u32 irq = gic_get_interrupt();

	if (irq == GIC_SPURIOUS_INTID) {
		klog_debug("Spurious interrupt received");
		return;
	}

	klog_debug("Handling IRQ %u", irq);

	gic_handle_interrupt(irq);

	gic_end_interrupt(irq);

	klog_debug("IRQ %u handling complete", irq);
}

int interrupt_register_handler(u32 irq, irq_handler_t handler, void *data)
{
	return gic_register_handler(irq, handler, data);
}

void interrupt_unregister_handler(u32 irq)
{
	gic_unregister_handler(irq);
}

void interrupt_enable(u32 irq)
{
	gic_enable_interrupt(irq);
}

void interrupt_disable(u32 irq)
{
	gic_disable_interrupt(irq);
}

void interrupt_set_type(u32 irq, irq_type_t type)
{
	gic_set_type(irq, type);
}
