#include "syscall.h"
#include "printf.h"
#include "compiler.h"
#include "idle.h"
#include <stdarg.h>

inline int syscall(syscall_num_t num, long args[6])
{
	if (num >= SYSCALL_NUM) {
		return -1; // Invalid syscall number
	}

	int result = 0;

	register long x0 asm("x0") = args[0];
	register long x1 asm("x1") = args[1];
	register long x2 asm("x2") = args[2];
	register long x3 asm("x3") = args[3];
	register long x4 asm("x4") = args[4];
	register long x5 asm("x5") = args[5];
	register long x8 asm("x8") = num;

	asm volatile("svc #0\n"
		     : "=r"(result)
		     : "r"(x8), "r"(x0), "r"(x1), "r"(x2), "r"(x3), "r"(x4), "r"(x5)
		     : "memory");

	return result;
}

int Create(int priority, void (*function)())
{
	long args[6] = { (long)priority, (long)function, 0, 0, 0, 0 };
	return syscall(SYS_CREATE, args);
}

int MyTid()
{
	long args[6] = { 0, 0, 0, 0, 0, 0 };
	return syscall(SYS_MYTID, args);
}

int MyParentTid()
{
	long args[6] = { 0, 0, 0, 0, 0, 0 };
	return syscall(SYS_MYPARENTTID, args);
}

void Yield()
{
	long args[6] = { 0, 0, 0, 0, 0, 0 };
	syscall(SYS_YIELD, args);
}

void Exit()
{
	long args[6] = { 0, 0, 0, 0, 0, 0 };
	syscall(SYS_EXIT, args);
}

int Send(int tid, const char *msg, int msglen, char *reply, int rplen)
{
	long args[6] = { (long)tid, (long)msg, (long)msglen, (long)reply, (long)rplen, 0 };
	return syscall(SYS_SEND, args);
}

int Receive(int *tid, char *msg, int msglen)
{
	long args[6] = { (long)tid, (long)msg, (long)msglen, 0, 0, 0 };
	return syscall(SYS_RECEIVE, args);
}

int ReceiveNonBlock(int *tid, char *msg, int msglen)
{
	long args[6] = { (long)tid, (long)msg, (long)msglen, 0, 0, 0 };
	return syscall(SYS_RECEIVE_NONBLOCK, args);
}

int Reply(int tid, const char *reply, int rplen)
{
	long args[6] = { (long)tid, (long)reply, (long)rplen, 0, 0, 0 };
	return syscall(SYS_REPLY, args);
}

int KLog(u8 level, const char *msg)
{
	long args[6] = { (long)level, (long)msg, 0, 0, 0, 0 };
	return syscall(SYS_KLOG, args);
}

int WaitTid(int tid)
{
	long args[6] = { (long)tid, 0, 0, 0, 0, 0 };
	return syscall(SYS_WAIT_TID, args);
}

int AwaitEvent(int event_id)
{
	long args[6] = { (long)event_id, 0, 0, 0, 0, 0 };
	return syscall(SYS_AWAIT_EVENT, args);
}

int SetupIdleTask(idle_stats_t *idle_stats)
{
	long args[6] = { (long)idle_stats, 0, 0, 0, 0, 0 };
	return syscall(SYS_SETUP_IDLE_TASK, args);
}

void __noreturn __attribute__((format(printf, 1, 2))) Panic(const char *fmt, ...)
{
	va_list va_args;
	char buf[1024];
	va_start(va_args, fmt);
	__raw_vsnprintf(buf, sizeof(buf), fmt, &va_args);
	va_end(va_args);
	long args[6] = { (long)buf, 0, 0, 0, 0, 0 };
	syscall(SYS_PANIC, args);

	UNREACHABLE();
}

int GetUnreadKlogs(char *buffer, int buffer_size, int *num_entries)
{
	long args[6] = { (long)buffer, (long)buffer_size, (long)num_entries, 0, 0, 0 };
	return syscall(SYS_GET_UNREAD_KLOGS, args);
}

int GetTaskInfo(char *buffer, int buffer_size)
{
	long args[6] = { (long)buffer, (long)buffer_size, 0, 0, 0, 0 };
	return syscall(SYS_GET_TASK_INFO, args);
}

void __noreturn Reboot()
{
	long args[6] = { 0, 0, 0, 0, 0, 0 };
	syscall(SYS_REBOOT, args);
	UNREACHABLE(); // Should never reach here
}

int Kill(int tid, int kill_children)
{
	long args[6] = { (long)tid, (long)kill_children, 0, 0, 0, 0 };
	return syscall(SYS_KILL, args);
}

int ToggleIdleDisplay(void)
{
	long args[6] = { 0, 0, 0, 0, 0, 0 };
	return syscall(SYS_TOGGLE_IDLE_DISPLAY, args);
}
