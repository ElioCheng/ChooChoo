#include "idle_task.h"
#include "io.h"
#include "name.h"
#include "task.h"
#include "random.h"
#include "syscall.h"
#include "name_server.h"
#include "compiler.h"
#include "clock_server.h"
#include "marklin/controller/marklin.h"
#include "io_server.h"
#include "io_test.h"
#include "klog.h"

#define LOG_MODULE "INIT"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

extern char __bss_start[];
extern char __bss_end[];

void init_bss(void)
{
	memset(__bss_start, 0, __bss_end - __bss_start);
}

int __noreturn __attribute__((section(".text.entry"))) main()
{
	init_bss();

	Create(MAX_PRIORITIES - 1, idle_task_main);

	Create(NAME_SERVER_PRIORITY, name_task);

	Create(IO_SERVER_PRIORITY, io_server_task);

	Create(CLOCK_SERVER_PRIORITY, clock_server_main);

	Create(MARKLIN_CONTROLLER_PRIORITY, marklin_controller_task);

	// Create(10, io_test_task);

	log_info("Init done");

	Exit();
	UNREACHABLE();
}
