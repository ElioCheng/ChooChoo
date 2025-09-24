#include "symbol.h"
#include "printf.h"
#include "klog.h"
#include "arch/cpu.h"

// This will be filled by the linker script with symbols
extern kernel_symbol_t __symbols_start[];
extern kernel_symbol_t __symbols_end[];
extern uint64_t __text_start[], __text_end[];
extern uint64_t __rodata_start[], __rodata_end[];
extern uint64_t __data_start[], __data_end[];
extern uint64_t __bss_start[], __bss_end[];
extern uint64_t __user_task_start[], __user_task_end[];
extern uint64_t __user_stacks_start[], __user_stacks_end[];

// Symbol table - will be populated at build time
static kernel_symbol_t *symbol_table = NULL;
static int symbol_count = 0;

static void test_uapp_symbol_resolution(void)
{
	klog_info("Testing user app symbol resolution...");

	uint64_t uapp_start = (uint64_t)__user_task_start;
	uint64_t uapp_end = (uint64_t)__user_task_end;

	klog_info("User app loaded at: %p-%p", uapp_start, uapp_end);

	const char *start_symbol = symbol_lookup(uapp_start);
	klog_info("Symbol at user app start (%p): %s", uapp_start, start_symbol);

	if (uapp_end > uapp_start) {
		uint64_t mid_addr = uapp_start + 0x100;
		const char *mid_symbol = symbol_lookup(mid_addr);
		klog_info("Symbol at user app +0x100 (%p): %s", mid_addr, mid_symbol);
	}
}

void symbol_init(void)
{
	symbol_table = __symbols_start;
	symbol_count = __symbols_end - __symbols_start;

	klog_info("Symbol table initialized with %d symbols (kernel + user apps)", symbol_count);
	klog_info("Current SP: %p", get_sp());
	klog_info("Text: %p-%p", (uint64_t)__text_start, (uint64_t)__text_end);
	klog_info("Rodata: %p-%p", (uint64_t)__rodata_start, (uint64_t)__rodata_end);
	klog_info("Data: %p-%p", (uint64_t)__data_start, (uint64_t)__data_end);
	klog_info("BSS: %p-%p", (uint64_t)__bss_start, (uint64_t)__bss_end);
	klog_info("Symbol table: %p-%p", (uint64_t)__symbols_start, (uint64_t)__symbols_end);
	klog_info("User task stack: %p-%p", (uint64_t)__user_stacks_start, (uint64_t)__user_stacks_end);
	klog_info("User task: %p-%p", (uint64_t)__user_task_start, (uint64_t)__user_task_end);

	test_uapp_symbol_resolution();
}

const char *symbol_lookup(uint64_t addr)
{
	if (!symbol_table || symbol_count == 0) {
		klog_error("Symbol table not initialized");
		return "unknown";
	}

	// Binary search for the closest symbol (symbols are sorted by address)
	int left = 0;
	int right = symbol_count - 1;
	int closest = -1;
	uint64_t closest_addr = 0;

	while (left <= right) {
		int mid = left + (right - left) / 2;

		if (symbol_table[mid].addr <= addr) {
			if (closest == -1 || symbol_table[mid].addr > closest_addr) {
				closest = mid;
				closest_addr = symbol_table[mid].addr;
			}
			left = mid + 1;
		} else {
			right = mid - 1;
		}
	}

	if (closest != -1) {
		uint64_t offset = addr - symbol_table[closest].addr;
		if (offset == 0) {
			return symbol_table[closest].name;
		} else {
			static char buffer[128];
			snprintf(buffer, sizeof(buffer), "%s+%#lx", symbol_table[closest].name, offset);
			return buffer;
		}
	}

	return "unknown";
}
