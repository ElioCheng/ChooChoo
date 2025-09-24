#include "arch/gic.h"
#include "klog.h"
#include "string.h"
#include "compiler.h"

typedef struct {
	irq_handler_t handler;
	void *data;
} irq_entry_t;

static irq_entry_t irq_handlers[GIC_MAX_INTERRUPTS];

void gic_init(void)
{
	u32 typer, num_interrupts;

	klog_info("Initializing GIC");

	memset(irq_handlers, 0, sizeof(irq_handlers));

	typer = gicd_read(GICD_TYPER);
	num_interrupts = ((typer & 0x1F) + 1) * 32;

	gicd_write(GICD_CTLR, 0);
	gicc_write(GICC_CTLR, 0);

	gicd_write(GICD_ICENABLER, 0xFFFFFFFF);  /* Disable all SGIs/PPIs */
	gicd_write(GICD_ICPENDR, 0xFFFFFFFF);    /* Clear pending */
	gicd_write(GICD_ICACTIVER, 0xFFFFFFFF);  /* Clear active */
	gicd_write(GICD_IGROUPR, 0xFFFFFFFF);    /* All to Group 1 (non-secure) */

	for (u32 i = 0; i < 32; i += 4) {
		gicd_write(GICD_IPRIORITYR + i, 0x80808080);
	}

	for (u32 i = 32; i < num_interrupts; i += 32) {
		gicd_write(GICD_ICENABLER + (i / 32) * 4, 0xFFFFFFFF); // Disable interrupt
		gicd_write(GICD_ICPENDR + (i / 32) * 4, 0xFFFFFFFF); // Clear pending state
		gicd_write(GICD_ICACTIVER + (i / 32) * 4, 0xFFFFFFFF); // Clear active state

		/* Route all SPIs to CPU 0 */
		for (u32 j = 0; j < 32; j += 4) {
			gicd_write(GICD_ITARGETSR + ((i + j) & ~0x3), 0x01010101);
		}

		gicd_write(GICD_IGROUPR + (i / 32) * 4, 0xFFFFFFFF); // Set group 1 (non-secure)

		for (u32 j = 0; j < 32; j += 4) {
			gicd_write(GICD_IPRIORITYR + i + j, 0x80808080);
		}
	}

	dsb();
	isb();

	/* Enable forwarding of both Group 0 and Group 1 interrupts */
	gicd_write(GICD_CTLR, GICD_CTLR_ENABLE | GICD_CTLR_ENABLEGRP1);

	gicc_write(GICC_PMR, 0xF0); // Set priority mask to allow priorities 0x00-0xEF
	gicc_write(GICC_BPR, 7); // No sub-priority grouping

	gicc_write(GICC_CTLR, GICC_CTLR_ENABLE | GICC_CTLR_ENABLEGRP1);

	klog_info("GIC initialized");
}

void gic_enable_interrupt(u32 irq)
{
	if (irq >= GIC_MAX_INTERRUPTS) {
		klog_error("Invalid IRQ number: %u", irq);
		return;
	}

	u32 reg = GICD_ISENABLER + (irq / 32) * 4;
	u32 bit = 1u << (irq % 32);
	gicd_write(reg, bit);

	klog_debug("Enabled IRQ %u", irq);
}

void gic_disable_interrupt(u32 irq)
{
	if (irq >= GIC_MAX_INTERRUPTS) {
		klog_error("Invalid IRQ number: %u", irq);
		return;
	}

	u32 reg = GICD_ICENABLER + (irq / 32) * 4;
	u32 bit = 1u << (irq % 32);
	gicd_write(reg, bit);

	klog_debug("Disabled IRQ %u", irq);
}

void gic_set_type(u32 irq, irq_type_t type)
{
	if (irq >= GIC_MAX_INTERRUPTS || irq < 16) {
		klog_error("Invalid IRQ number for type configuration: %u", irq);
		return;
	}

	u32 reg = GICD_ICFGR + (irq / 16) * 4;
	u32 shift = ((irq % 16) * 2) + 1; /* bit 0 is reserved        */
	u32 mask = 1u << shift;
	u32 val = gicd_read(reg);

	if (type == IRQ_TYPE_EDGE_RISING || type == IRQ_TYPE_EDGE_FALLING)
		val |= mask; /* edge-triggered */
	else
		val &= ~mask; /* level-sensitive */

	gicd_write(reg, val);
}

u32 gic_get_interrupt(void)
{
	u32 iar = gicc_read(GICC_IAR);
	u32 irq = iar & 0x3FF;

	if (irq == GIC_SPURIOUS_INTID) {
		klog_debug("Spurious interrupt");
		return GIC_SPURIOUS_INTID;
	}
	return irq;
}

void gic_end_interrupt(u32 irq)
{
	if (irq >= GIC_MAX_INTERRUPTS && irq != GIC_SPURIOUS_INTID) {
		klog_error("Invalid IRQ number for EOI: %u", irq);
		return;
	}
	gicc_write(GICC_EOIR, irq);
}

int gic_register_handler(u32 irq, irq_handler_t handler, void *data)
{
	if (irq >= GIC_MAX_INTERRUPTS) {
		klog_error("Invalid IRQ number: %u", irq);
		return -1;
	}
	if (irq_handlers[irq].handler) {
		klog_error("IRQ %u already has a handler", irq);
		return -1;
	}
	irq_handlers[irq].handler = handler;
	irq_handlers[irq].data = data;
	klog_debug("Registered handler for IRQ %u", irq);
	return 0;
}

void gic_unregister_handler(u32 irq)
{
	if (irq >= GIC_MAX_INTERRUPTS) {
		klog_error("Invalid IRQ number: %u", irq);
		return;
	}
	irq_handlers[irq].handler = NULL;
	irq_handlers[irq].data = NULL;
	klog_debug("Unregistered handler for IRQ %u", irq);
}

void gic_handle_interrupt(u32 irq)
{
	if (irq >= GIC_MAX_INTERRUPTS) {
		klog_error("Invalid IRQ number in handler: %u", irq);
		return;
	}
	if (irq_handlers[irq].handler)
		irq_handlers[irq].handler(irq, irq_handlers[irq].data);
	else
		klog_warning("Unhandled interrupt: %u", irq);
}
