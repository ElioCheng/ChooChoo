#ifndef __REGISTERS_H__
#define __REGISTERS_H__

#include "types.h"

// 304 bytes
struct __attribute__((packed)) arch_regs {
	u64 x[32];  // 0-255   (256 bytes)
	u64 sp;      // 256-263 (8 bytes)
	u64 pc;      // 264-271 (8 bytes)
	u64 spsr;    // 272-279 (8 bytes)
	u64 elr;     // 280-287 (8 bytes)
	u64 tpidr;   // 288-295 (8 bytes)
	u8 padding[8]; // 296-303 (8 bytes)
};

// X0-x7 are callee-saved
// X8-x15 are caller-saved
// X16-x29 are callee-saved
// X30 is the link register
// SP is the stack pointer
// PC is the program counter
// SPSR is the saved program status register
#define REG_X0(regs) (regs.x[0])
#define REG_X1(regs) (regs.x[1])
#define REG_X2(regs) (regs.x[2])
#define REG_X3(regs) (regs.x[3])
#define REG_X4(regs) (regs.x[4])
#define REG_X5(regs) (regs.x[5])
#define REG_X6(regs) (regs.x[6])
#define REG_X7(regs) (regs.x[7])
#define REG_X8(regs) (regs.x[8])
#define REG_X9(regs) (regs.x[9])
#define REG_X10(regs) (regs.x[10])
#define REG_X11(regs) (regs.x[11])
#define REG_X12(regs) (regs.x[12])
#define REG_X13(regs) (regs.x[13])
#define REG_X14(regs) (regs.x[14])
#define REG_X15(regs) (regs.x[15])
#define REG_X16(regs) (regs.x[16])
#define REG_X17(regs) (regs.x[17])
#define REG_X18(regs) (regs.x[18])
#define REG_X19(regs) (regs.x[19])
#define REG_X20(regs) (regs.x[20])
#define REG_X21(regs) (regs.x[21])
#define REG_X22(regs) (regs.x[22])
#define REG_X23(regs) (regs.x[23])
#define REG_X24(regs) (regs.x[24])
#define REG_X25(regs) (regs.x[25])
#define REG_X26(regs) (regs.x[26])
#define REG_X27(regs) (regs.x[27])
#define REG_X28(regs) (regs.x[28])
#define REG_X29(regs) (regs.x[29])
#define REG_X30(regs) (regs.x[30])
#define REG_LR(regs) (REG_X30(regs))
#define REG_SP(regs) (regs.sp)
#define REG_PC(regs) (regs.pc)
#define REG_SPSR(regs) (regs.spsr)
#define REG_ELR(regs) (regs.elr)
#define REG_TPIDR(regs) (regs.tpidr)

#define read_sysreg(reg) ({ \
	long __res; \
	asm volatile("mrs %0, " reg "\n" \
		: "=r" (__res) \
		: \
		: "memory"); \
	__res; \
})

#define write_sysreg(reg, val) ({ \
	asm volatile("msr " reg ", %0\n" \
		: \
		: "r" (val) \
		: "memory"); \
})

#define read_reg(reg) ({ \
	u64 __res; \
	asm volatile("mov %0, " reg "\n" \
		: "=r" (__res) \
		: \
		: "memory"); \
	__res; \
})

#define write_reg(reg, val) ({ \
	asm volatile("mov " reg ", %0\n" \
		: \
		: "r" (val) \
		: "memory"); \
})

#endif
