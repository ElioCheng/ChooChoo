#include "arch/registers.h"
#include "compiler.h"
#include "exception.h"
#include "klog.h"
#include "panic.h"
#include "syscall.h"
#include "uart.h"
#include "context.h"
#include "task.h"
#include "sched.h"
#include "interrupt.h"
#include "klog.h"

u8 from_exception = 0;

void handle_sync_exception(context_t *context)
{
	klog_set_destinations(KLOG_DEST_CONSOLE);
	klog_print_all_unread();
	klog_info("handle_sync_exception");
	u64 esr = read_sysreg("esr_el1");
	u64 ec = (esr >> 26) & 0x3f;
	u64 far = read_sysreg("far_el1");
	klog_info("esr = %#lx, ec = %#lx, far = %#lx", esr, ec, far);

	dump_current_context(0);
	dump_context(context, 0);
	uart_process_tx_buffers_blocking();
	asm volatile("b .");
}

void sync_el0_handler(context_t *context)
{
	from_exception = 1;
	u64 esr = read_sysreg("esr_el1");
	u64 ec = (esr >> 26) & 0x3f;
	u64 far = read_sysreg("far_el1");

	klog_debug("esr = %#lx, ec = %#lx, far = %#lx", esr, ec, far);
	uart_process_tx_buffers_blocking();

	// Save current task context if we have a current task
	if (current_task) {
		// Copy the saved context from the stack to the task structure
		memcpy(&current_task->context, context, sizeof(context_t));
	}

	if (ec == 0x15) {
		// System call
		handle_syscall(current_task);
	} else if (ec == 0x24) {
		u64 elr = read_sysreg("elr_el1");
		klog_set_destinations(KLOG_DEST_CONSOLE);
		klog_print_all_unread();
		klog_panic("Data abort at %#lx", elr);
		handle_sync_exception(context);
	} else if (ec == 0x20) {
		u64 elr = read_sysreg("elr_el1");
		klog_set_destinations(KLOG_DEST_CONSOLE);
		klog_print_all_unread();
		klog_panic("Instruction abort at %#lx", elr);
		handle_sync_exception(context);
	} else {
		// Other synchronous exception
		handle_sync_exception(context);
	}

	sched_schedule();

	panic("sync_el0_handler: should not be reached");
	UNREACHABLE();
}

void irq_el0_handler(context_t *context)
{
	from_exception = 1;

	if (current_task) {
		memcpy(&current_task->context, context, sizeof(context_t));
	}
	uart_process_tx_buffers_blocking();

	handle_irq();

	sched_schedule();

	panic("irq_el0_handler: should not be reached");
	UNREACHABLE();
}

void other_handler()
{
	klog_set_destinations(KLOG_DEST_CONSOLE);
	klog_print_all_unread();

	u64 esr = read_sysreg("esr_el1");
	u64 ec = (esr >> 26) & 0x3f;
	u64 far = read_sysreg("far_el1");
	u64 elr = read_sysreg("elr_el1");
	u64 spsr = read_sysreg("spsr_el1");
	u64 currentel = read_sysreg("currentel");

	klog_error("=== OTHER HANDLER CALLED ===");
	klog_error("Current EL: %#lx", (currentel >> 2) & 0x3);
	klog_error("ELR_EL1 (exception return addr): %#lx", elr);
	klog_error("SPSR_EL1 (saved program state): %#lx", spsr);
	klog_error("ESR_EL1 (exception syndrome): %#lx", esr);
	klog_error("EC (exception class): %#lx", ec);
	klog_error("FAR_EL1 (fault address): %#lx", far);

	const char *ec_name = "UNKNOWN";
	switch (ec) {
	case 0x00:
		ec_name = "UNKNOWN";
		break;
	case 0x01:
		ec_name = "WFI/WFE";
		break;
	case 0x03:
		ec_name = "MCR/MRC (CP15)";
		break;
	case 0x04:
		ec_name = "MCRR/MRRC (CP15)";
		break;
	case 0x05:
		ec_name = "MCR/MRC (CP14)";
		break;
	case 0x06:
		ec_name = "LDC/STC (CP14)";
		break;
	case 0x07:
		ec_name = "FP/SIMD";
		break;
	case 0x0C:
		ec_name = "MRRC (CP14)";
		break;
	case 0x0E:
		ec_name = "ILLEGAL EXECUTION";
		break;
	case 0x11:
		ec_name = "SVC (A32)";
		break;
	case 0x12:
		ec_name = "HVC (A32)";
		break;
	case 0x13:
		ec_name = "SMC (A32)";
		break;
	case 0x15:
		ec_name = "SVC (A64)";
		break;
	case 0x16:
		ec_name = "HVC (A64)";
		break;
	case 0x17:
		ec_name = "SMC (A64)";
		break;
	case 0x18:
		ec_name = "MSR/MRS/SYS";
		break;
	case 0x20:
		ec_name = "INSTRUCTION ABORT (lower EL)";
		break;
	case 0x21:
		ec_name = "INSTRUCTION ABORT (same EL)";
		break;
	case 0x22:
		ec_name = "PC ALIGNMENT";
		break;
	case 0x24:
		ec_name = "DATA ABORT (lower EL)";
		break;
	case 0x25:
		ec_name = "DATA ABORT (same EL)";
		break;
	case 0x26:
		ec_name = "SP ALIGNMENT";
		break;
	case 0x28:
		ec_name = "FP EXCEPTION (A32)";
		break;
	case 0x2C:
		ec_name = "FP EXCEPTION (A64)";
		break;
	case 0x2F:
		ec_name = "SERROR";
		break;
	case 0x30:
		ec_name = "BREAKPOINT (lower EL)";
		break;
	case 0x31:
		ec_name = "BREAKPOINT (same EL)";
		break;
	case 0x32:
		ec_name = "STEP (lower EL)";
		break;
	case 0x33:
		ec_name = "STEP (same EL)";
		break;
	case 0x34:
		ec_name = "WATCHPOINT (lower EL)";
		break;
	case 0x35:
		ec_name = "WATCHPOINT (same EL)";
		break;
	case 0x38:
		ec_name = "BKPT (A32)";
		break;
	case 0x3A:
		ec_name = "VECTOR CATCH (A32)";
		break;
	case 0x3C:
		ec_name = "BRK (A64)";
		break;
	}
	klog_error("Exception class: %s", ec_name);

	// Additional information for data/instruction aborts
	if (ec == 0x24 || ec == 0x25 || ec == 0x20 || ec == 0x21) {
		// Data/Instruction abort - decode more information
		u64 dfsc = esr & 0x3f; // Data Fault Status Code
		klog_error("Fault Status Code: %#lx", dfsc);

		const char *fault_type = "UNKNOWN";
		switch (dfsc) {
		case 0x04:
			fault_type = "Translation fault (level 0)";
			break;
		case 0x05:
			fault_type = "Translation fault (level 1)";
			break;
		case 0x06:
			fault_type = "Translation fault (level 2)";
			break;
		case 0x07:
			fault_type = "Translation fault (level 3)";
			break;
		case 0x09:
			fault_type = "Access flag fault (level 1)";
			break;
		case 0x0A:
			fault_type = "Access flag fault (level 2)";
			break;
		case 0x0B:
			fault_type = "Access flag fault (level 3)";
			break;
		case 0x0D:
			fault_type = "Permission fault (level 1)";
			break;
		case 0x0E:
			fault_type = "Permission fault (level 2)";
			break;
		case 0x0F:
			fault_type = "Permission fault (level 3)";
			break;
		}
		klog_error("Fault type: %s", fault_type);

		// Show the faulting instruction
		klog_error("Faulting instruction at %#lx:", elr);
		u32 *instr = (u32 *)elr;
		if (elr >= 0x80000 && elr < 0x200000) { // Valid kernel address
			klog_error("Instruction: %#x", *instr);
		} else {
			klog_error("Cannot read instruction - invalid address");
		}
	}

	uart_process_tx_buffers_blocking();

	panic("other_handler: Unhandled exception - system halted");
}
