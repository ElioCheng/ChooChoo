#ifndef ARCH_GIC_H
#define ARCH_GIC_H

#include "types.h"

#define GIC_BASE 0xFF840000UL
#define GICD_BASE (GIC_BASE + 0x1000UL) // Distributor
#define GICC_BASE (GIC_BASE + 0x2000UL) // CPU Interface

// GICD Register Offsets
#define GICD_CTLR 0x000 // Distributor Control Register
#define GICD_TYPER 0x004 // Interrupt Controller Type Register
#define GICD_IIDR 0x008 // Distributor Implementer Identification Register
#define GICD_IGROUPR 0x080 // Interrupt Group Registers
#define GICD_ISENABLER 0x100 // Interrupt Set-Enable Registers
#define GICD_ICENABLER 0x180 // Interrupt Clear-Enable Registers
#define GICD_ISPENDR 0x200 // Interrupt Set-Pending Registers
#define GICD_ICPENDR 0x280 // Interrupt Clear-Pending Registers
#define GICD_ISACTIVER 0x300 // Interrupt Set-Active Registers
#define GICD_ICACTIVER 0x380 // Interrupt Clear-Active Registers
#define GICD_IPRIORITYR 0x400 // Interrupt Priority Registers
#define GICD_ITARGETSR 0x800 // Interrupt Processor Targets Registers
#define GICD_ICFGR 0xC00 // Interrupt Configuration Registers
#define GICD_SGIR 0xF00 // Software Generated Interrupt Register

// GICC Register Offsets
#define GICC_CTLR 0x000 // CPU Interface Control Register
#define GICC_PMR 0x004 // Interrupt Priority Mask Register
#define GICC_BPR 0x008 // Binary Point Register
#define GICC_IAR 0x00C // Interrupt Acknowledge Register
#define GICC_EOIR 0x010 // End of Interrupt Register
#define GICC_RPR 0x014 // Running Priority Register
#define GICC_HPPIR 0x018 // Highest Priority Pending Interrupt Register
#define GICC_ABPR 0x01C // Aliased Binary Point Register
#define GICC_AIAR 0x020 // Aliased Interrupt Acknowledge Register
#define GICC_AEOIR 0x024 // Aliased End of Interrupt Register
#define GICC_AHPPIR 0x028 // Aliased Highest Priority Pending Interrupt Register
#define GICC_IIDR 0x0FC // CPU Interface Implementer Identification Register

#define GICD_CTLR_ENABLE (1 << 0)
#define GICD_CTLR_ENABLEGRP0 (1 << 0)
#define GICD_CTLR_ENABLEGRP1 (1 << 1)
#define GICC_CTLR_ENABLE (1 << 0)
#define GICC_CTLR_ENABLEGRP0 (1 << 0)
#define GICC_CTLR_ENABLEGRP1 (1 << 1)
#define GICC_CTLR_FIQEN (1 << 3)

#define GICD_ICFGR_LEVEL 0x0
#define GICD_ICFGR_EDGE 0x2

#define GIC_SPURIOUS_INTID 1023

#define GIC_MAX_INTERRUPTS 1024

typedef enum {
	IRQ_TYPE_LEVEL_HIGH = 0,
	IRQ_TYPE_EDGE_RISING = 1,
	IRQ_TYPE_LEVEL_LOW = 2,
	IRQ_TYPE_EDGE_FALLING = 3,
} irq_type_t;

typedef void (*irq_handler_t)(u32 irq, void *data);

void gic_init(void);
void gic_enable_interrupt(u32 irq);
void gic_disable_interrupt(u32 irq);
void gic_set_type(u32 irq, irq_type_t type);
u32 gic_get_interrupt(void);
void gic_end_interrupt(u32 irq);
int gic_register_handler(u32 irq, irq_handler_t handler, void *data);
void gic_unregister_handler(u32 irq);
void gic_handle_interrupt(u32 irq);

static inline u32 gicd_read(u32 offset)
{
	return *(volatile u32 *)(GICD_BASE + offset);
}

static inline void gicd_write(u32 offset, u32 value)
{
	*(volatile u32 *)(GICD_BASE + offset) = value;
}

static inline u32 gicc_read(u32 offset)
{
	return *(volatile u32 *)(GICC_BASE + offset);
}

static inline void gicc_write(u32 offset, u32 value)
{
	*(volatile u32 *)(GICC_BASE + offset) = value;
}

#endif // ARCH_GIC_H
