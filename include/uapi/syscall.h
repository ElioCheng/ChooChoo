#ifndef __UAPI_SYSCALL_H__
#define __UAPI_SYSCALL_H__

#include "types.h"
#include "idle.h"

#undef SYSCALL
#undef __SYSCALL_LIST_H__
#define SYSCALL(name, num) name = num,
typedef enum {
#include "syscall_list.h"
	SYSCALL_NUM
} syscall_num_t;
#undef SYSCALL

int Create(int priority, void (*function)());

int MyTid();

int MyParentTid();

void Yield();

void Exit();

int Send(int tid, const char *msg, int msglen, char *reply, int rplen);

int Receive(int *tid, char *msg, int msglen);

int ReceiveNonBlock(int *tid, char *msg, int msglen);

int Reply(int tid, const char *reply, int rplen);

int KLog(u8 level, const char *msg);

int WaitTid(int tid);

int AwaitEvent(int event_id);

int SetupIdleTask(idle_stats_t *idle_stats);

void __attribute__((noreturn)) Panic(const char *fmt, ...);

int GetUnreadKlogs(char *buffer, int buffer_size, int *num_entries);

int GetTaskInfo(char *buffer, int buffer_size);

void __attribute__((noreturn)) Reboot();

int Kill(int tid, int kill_children);

int ToggleIdleDisplay(void);

int syscall(syscall_num_t num, long args[6]);

#endif
