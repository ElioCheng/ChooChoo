#include "exception.h"
#include "arch/registers.h"
#include "context.h"
#include "klog.h"
#include "types.h"
#include "symbol.h"
#include "string.h"

#define LOG(force, fmt, ...)                                 \
	do {                                                 \
		if (force) {                                 \
			klog_force_info(fmt, ##__VA_ARGS__); \
		} else {                                     \
			klog_info(fmt, ##__VA_ARGS__);       \
		}                                            \
	} while (0)

/* dump 8 words per line, caller should give valid address range which is 32N */
void dump_data(unsigned long addr_start, unsigned long addr_end, int force)
{
	u32 words[8UL];
	unsigned long _addr_start = addr_start;
	for (; _addr_start < addr_end; _addr_start += 8UL * sizeof(u32)) {
		words[0UL] = *(u32 *)(_addr_start);
		words[1UL] = *(u32 *)(_addr_start + 4UL * sizeof(u32));
		words[2UL] = *(u32 *)(_addr_start + 8UL * sizeof(u32));
		words[3UL] = *(u32 *)(_addr_start + 12UL * sizeof(u32));
		words[4UL] = *(u32 *)(_addr_start + 16UL * sizeof(u32));
		words[5UL] = *(u32 *)(_addr_start + 20UL * sizeof(u32));
		words[6UL] = *(u32 *)(_addr_start + 24UL * sizeof(u32));
		words[7UL] = *(u32 *)(_addr_start + 28UL * sizeof(u32));

		LOG(force, "%#.8x: %.8x %.8x %.8x %.8x %.8x %.8x %.8x %.8x", _addr_start, words[0UL], words[1UL],
		    words[2UL], words[3UL], words[4UL], words[5UL], words[6UL], words[7UL]);
	}
}

void dump_around_address(u64 address, size_t size, int force)
{
	unsigned char *p = (unsigned char *)address;

	dump_data(ptr_to_ulong(p) - size / 2, ptr_to_ulong(p) + size / 2, force);
}

static inline u64 get_pc()
{
	u64 pc;
	asm volatile("adr %0, ." : "=r"(pc));
	return pc;
}

void dump_around_pc(int force)
{
	u64 pc = get_pc();
	LOG(force, "Dumping memory around PC: %p", pc);
	dump_around_address(pc, 256, force);
}

static inline u64 get_elr()
{
	u64 elr;
	asm volatile("mrs %0, elr_el1" : "=r"(elr));
	return elr;
}

static inline u64 get_sp()
{
	u64 sp;
	asm volatile("mov %0, sp" : "=r"(sp));
	return sp;
}

void dump_around_sp(int force)
{
	u64 sp = get_sp();
	LOG(force, "Dumping memory around SP: %p", sp);
	dump_around_address(sp, 256, force);
}

void dump_around_elr(int force)
{
	u64 elr = get_elr();
	LOG(force, "Dumping memory around ELR: %p", elr);
	dump_around_address(elr, 256, force);
}

extern uint64_t __text_start[], __text_end[];

void dump_backtrace(int force)
{
	u64 fp;
	u64 lr;
	int depth = 0;
	const int MAX_DEPTH = 16; // Limit the depth to avoid infinite loops
	LOG(force, "Dumping backtrace at PC: %p", get_pc());

	// Get the current frame pointer
	asm volatile("mov %0, x29" : "=r"(fp));

	while (fp && depth < MAX_DEPTH) {
		// The return address (LR) is stored at fp + 8
		lr = *(u64 *)(fp + 8);
		// if (lr < (u64)__text_start || lr > (u64)__text_end) {
		// 	klog_info("Invalid return address %p, stopping backtrace", lr);
		// 	break;
		// }

		const char *symbol_info = symbol_lookup(lr);
		if (strcmp(symbol_info, "_reboot") == 0) {
			break;
		}
		LOG(force, "#%d: %p in %s", depth, lr, symbol_info);
		fp = *(u64 *)fp;
		depth++;
	}
}

#define DUMP_REG(x)                                    \
	do {                                           \
		u64 x;                                 \
		asm volatile("mov %0, " #x : "=r"(x)); \
		LOG(force, #x " = %#.8lx", x);         \
	} while (0)

#define DUMP_REGS(x, y)                                            \
	do {                                                       \
		u64 x, y;                                          \
		asm volatile("mov %0, " #x : "=r"(x));             \
		asm volatile("mov %0, " #y : "=r"(y));             \
		LOG(force, #x " = %#.8lx, " #y " = %#.8lx", x, y); \
	} while (0)

#define DUMP_SYS_REG(x)                                \
	do {                                           \
		u64 x;                                 \
		asm volatile("mrs %0, " #x : "=r"(x)); \
		LOG(force, #x " = %#.8lx", x);         \
	} while (0)

void dump_registers(int force)
{
	LOG(force, "Dumping registers");
	DUMP_REGS(x0, x1);
	DUMP_REGS(x2, x3);
	DUMP_REGS(x4, x5);
	DUMP_REGS(x6, x7);
	DUMP_REGS(x8, x9);
	DUMP_REGS(x10, x11);
	DUMP_REGS(x12, x13);
	DUMP_REGS(x14, x15);
	DUMP_REGS(x16, x17);
	DUMP_REGS(x18, x19);
	DUMP_REGS(x20, x21);
	DUMP_REGS(x22, x23);
	DUMP_REGS(x24, x25);
	DUMP_REGS(x26, x27);
	DUMP_REGS(x28, x29);
	DUMP_REG(lr);
	DUMP_REG(sp);
	DUMP_REG(fp);
	DUMP_SYS_REG(esr_el1);
	DUMP_SYS_REG(elr_el1);
	LOG(force, "pc = %#.8lx", get_pc());
}

void dump_current_context(int force)
{
	LOG(force, "Dumping current context");
	dump_backtrace(force);
	dump_registers(force);
	dump_around_pc(force);
	dump_around_sp(force);
	dump_around_elr(force);
}

#define DUMP_CONTEXT_REG(context, x)            \
	do {                                    \
		u64 x = REG_##x(context->regs); \
		LOG(force, #x " = %#.8lx", x);  \
	} while (0)

#define DUMP_CONTEXT_REGS(context, x, y)                           \
	do {                                                       \
		u64 x = REG_##x(context->regs);                    \
		u64 y = REG_##y(context->regs);                    \
		LOG(force, #x " = %#.8lx, " #y " = %#.8lx", x, y); \
	} while (0)

void dump_context_regs(context_t *context, int force)
{
	LOG(force, "Dumping context registers");
	DUMP_CONTEXT_REGS(context, X0, X1);
	DUMP_CONTEXT_REGS(context, X2, X3);
	DUMP_CONTEXT_REGS(context, X4, X5);
	DUMP_CONTEXT_REGS(context, X6, X7);
	DUMP_CONTEXT_REGS(context, X8, X9);
	DUMP_CONTEXT_REGS(context, X10, X11);
	DUMP_CONTEXT_REGS(context, X12, X13);
	DUMP_CONTEXT_REGS(context, X14, X15);
	DUMP_CONTEXT_REGS(context, X16, X17);
	DUMP_CONTEXT_REGS(context, X18, X19);
	DUMP_CONTEXT_REGS(context, X20, X21);
	DUMP_CONTEXT_REGS(context, X22, X23);
	DUMP_CONTEXT_REGS(context, X24, X25);
	DUMP_CONTEXT_REGS(context, X26, X27);
	DUMP_CONTEXT_REGS(context, X28, X29);
	DUMP_CONTEXT_REG(context, LR);
	DUMP_CONTEXT_REG(context, SP);
	DUMP_CONTEXT_REG(context, SPSR);
	DUMP_CONTEXT_REG(context, ELR);
	DUMP_CONTEXT_REG(context, TPIDR);
}

void dump_context(context_t *context, int force)
{
	LOG(force, "Dumping context");
	dump_context_regs(context, force);
	u64 pc = REG_PC(context->regs);
	LOG(force, "Dumping memory around PC: %p", pc);
	dump_around_address(pc, 256, force);

	u64 sp = REG_SP(context->regs);
	LOG(force, "Dumping memory around SP: %p", sp);
	dump_around_address(sp, 256, force);
}