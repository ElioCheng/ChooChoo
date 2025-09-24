#include "compiler.h"
#include "syscall.h"
#include "name.h"
#include "io.h"
#include "string.h"
#include "timer/time.h"

#define printf console_printf

static __maybe_unused void test_console_output(void)
{
	int io_tid = WhoIs(IO_SERVER_NAME);

	printf("=== Testing Console Output ===\r\n");

	printf("Testing Putc to console: ");
	for (char c = 'A'; c <= 'E'; c++) {
		int result = Putc(io_tid, IO_CHANNEL_CONSOLE, c);
		if (result != 0) {
			printf("Error: Putc failed with result %d\r\n", result);
			return;
		}
	}
	printf(" [Done]\r\n");

	printf("Testing Putn to console: ");
	Putn(io_tid, IO_CHANNEL_CONSOLE, "Hello, World!", 13);
	printf(" [Done]\r\n");
}

static __maybe_unused void test_marklin_output(void)
{
	int io_tid = WhoIs(IO_SERVER_NAME);

	printf("=== Testing Marklin Output ===\r\n");

	printf("Testing Putc to Marklin: ");
	for (char c = '1'; c <= '5'; c++) {
		int result = Putc(io_tid, IO_CHANNEL_MARKLIN, c);
		if (result != 0) {
			printf("Error: Marklin Putc failed with result %d\r\n", result);
			return;
		}
	}
	printf(" [Done]\r\n");
}

static __maybe_unused void test_input_operations(void)
{
	int io_tid = WhoIs(IO_SERVER_NAME);

	printf("=== Testing Input Operations ===\r\n");

	// Test console input
	printf("Testing console Getc: ");
	int result = Getc(io_tid, IO_CHANNEL_CONSOLE);
	if (result >= 0) {
		printf("Got char: '%c' (0x%02x)\r\n", (char)result, result);
	} else {
		printf("No input available (expected)\r\n");
	}
}

void io_test_task(void)
{
	int my_tid = MyTid();
	printf("IO Test Task (TID %d) starting...\r\n", my_tid);

	int io_tid = WhoIs(IO_SERVER_NAME);
	if (io_tid < 0) {
		printf("Error: IO Server not found!\r\n");
		Exit();
	}

	printf("Found IO Server at TID %d\r\n", io_tid);

	printf("\r\n======================================\r\n");
	printf("       IO SERVER TEST SUITE\r\n");
	printf("======================================\r\n");

	test_console_output();
	printf("\r\n");

	test_input_operations();
	printf("\r\n");

	test_marklin_output();
	printf("\r\n");

	printf("======================================\r\n");
	printf("       IO TEST SUITE COMPLETE\r\n");
	printf("======================================\r\n");

	printf("IO Test Task (TID %d) completed successfully!\r\n", my_tid);
	Exit();
}
