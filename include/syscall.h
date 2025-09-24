#ifndef __SYSCALL_H__
#define __SYSCALL_H__

#include "types.h"
#include "context.h"
#include "task.h"
#include "idle.h"

#define SYSCALL_NAME_LEN 32

#undef SYSCALL
#undef __SYSCALL_LIST_H__
#define SYSCALL(name, num) name = num,
typedef enum {
#include "syscall_list.h"
	SYSCALL_NUM
} syscall_num_t;
#undef SYSCALL

#define SYSCALL_SET_RESULT(task, value) REG_X0(task->context.regs) = value

void handle_syscall(task_t *current_task);

i64 syscall_create(task_t *current_task, int priority, void (*function)());

i64 syscall_mytid(task_t *current_task);

i64 syscall_myparenttid(task_t *current_task);

void syscall_yield(task_t *current_task);

void syscall_exit(task_t *current_task);

i64 syscall_send(task_t *current_task, int tid, const char *msg, int msglen, char *reply, int rplen);

i64 syscall_receive(task_t *current_task, int *tid, char *msg, int msglen);

i64 syscall_receive_nonblock(task_t *current_task, int *tid, char *msg, int msglen);

i64 syscall_reply(task_t *current_task, int tid, const char *reply, int rplen);

i64 syscall_klog(task_t *current_task, u8 level, const char *msg);

i64 syscall_wait_tid(task_t *current_task, int tid);

i64 syscall_await_event(task_t *current_task, int event_id);

i64 syscall_panic(task_t *current_task, const char *msg);

i64 syscall_setup_idle_task(task_t *current_task, idle_stats_t *idle_stats);

i64 syscall_get_unread_klogs(task_t *current_task, char *buffer, int buffer_size, int *num_entries);

i64 syscall_get_task_info(task_t *current_task, char *buffer, int buffer_size);

void __noreturn syscall_reboot(task_t *current_task);

i64 syscall_kill(task_t *current_task, int tid, int kill_children);

i64 syscall_toggle_idle_display(task_t *current_task);

#endif
