#include "arch/registers.h"
#include "syscall.h"
#include "dlist.h"
#include "klog.h"
#include "panic.h"
#include "task.h"
#include "sched.h"
#include "uart.h"
#include "printf.h"
#include "string.h"
#include "event.h"
#include <stdarg.h>

extern void _reboot(void);

char *syscall_name[SYSCALL_NUM] = {
#undef SYSCALL
#undef __SYSCALL_LIST_H__
#define SYSCALL(name, num) #name,
#include "syscall_list.h"
#undef SYSCALL
#undef __SYSCALL_LIST_H__
};
#define SYSCALL_NAME(num) syscall_name[num - 1]

#define min(a, b) ((a) < (b) ? (a) : (b))

void handle_syscall(task_t *current_task)
{
	BUG_ON(current_task == NULL);
	context_t *context = &current_task->context;
	u64 syscall_num = REG_X8(context->regs);

	klog_debug("tid: %d, syscall_num = %#lx, syscall_name = %s", current_task->tid, syscall_num,
		   SYSCALL_NAME(syscall_num));

	switch (syscall_num) {
	case SYS_CREATE: {
		u64 priority = REG_X0(context->regs);
		u64 function_ptr = REG_X1(context->regs);
		i64 result = (u64)syscall_create(current_task, (int)priority, (void (*)())function_ptr);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_MYTID: {
		u64 result = (u64)syscall_mytid(current_task);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_MYPARENTTID: {
		u64 result = (u64)syscall_myparenttid(current_task);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_YIELD:
		syscall_yield(current_task);
		break;
	case SYS_EXIT:
		syscall_exit(current_task);
		break;
	case SYS_SEND: {
		u64 tid = REG_X0(context->regs);
		u64 msg_ptr = REG_X1(context->regs);
		u64 msglen = REG_X2(context->regs);
		u64 reply_ptr = REG_X3(context->regs);
		u64 rplen = REG_X4(context->regs);
		i64 result = syscall_send(current_task, (int)tid, (const char *)msg_ptr, (int)msglen, (char *)reply_ptr,
					  (int)rplen);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_RECEIVE: {
		u64 tid_ptr = REG_X0(context->regs);
		u64 msg_ptr = REG_X1(context->regs);
		u64 msglen = REG_X2(context->regs);
		i64 result = syscall_receive(current_task, (int *)tid_ptr, (char *)msg_ptr, (int)msglen);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_REPLY: {
		u64 tid = REG_X0(context->regs);
		u64 reply_ptr = REG_X1(context->regs);
		u64 rplen = REG_X2(context->regs);
		i64 result = syscall_reply(current_task, (int)tid, (const char *)reply_ptr, (int)rplen);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_KLOG: {
		u64 level = REG_X0(context->regs);
		u64 msg_ptr = REG_X1(context->regs);
		u64 result = syscall_klog(current_task, (u8)level, (const char *)msg_ptr);
		SYSCALL_SET_RESULT(current_task, result);
	} break;

	case SYS_PANIC: {
		u64 msg_ptr = REG_X0(context->regs);
		syscall_panic(current_task, (const char *)msg_ptr);
		// This syscall should never return
	} break;
	case SYS_WAIT_TID: {
		u64 tid = REG_X0(context->regs);
		i64 result = syscall_wait_tid(current_task, (int)tid);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_AWAIT_EVENT: {
		u64 event_id = REG_X0(context->regs);
		i64 result = syscall_await_event(current_task, (int)event_id);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_SETUP_IDLE_TASK: {
		u64 idle_stats_ptr = REG_X0(context->regs);
		i64 result = syscall_setup_idle_task(current_task, (idle_stats_t *)idle_stats_ptr);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_GET_UNREAD_KLOGS: {
		u64 buffer_ptr = REG_X0(context->regs);
		u64 buffer_size = REG_X1(context->regs);
		u64 num_entries_ptr = REG_X2(context->regs);
		i64 result = syscall_get_unread_klogs(current_task, (char *)buffer_ptr, (int)buffer_size,
						      (int *)num_entries_ptr);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_GET_TASK_INFO: {
		u64 buffer_ptr = REG_X0(context->regs);
		u64 buffer_size = REG_X1(context->regs);
		i64 result = syscall_get_task_info(current_task, (char *)buffer_ptr, (int)buffer_size);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_RECEIVE_NONBLOCK: {
		u64 tid_ptr = REG_X0(context->regs);
		u64 msg_ptr = REG_X1(context->regs);
		u64 msglen = REG_X2(context->regs);
		i64 result = syscall_receive_nonblock(current_task, (int *)tid_ptr, (char *)msg_ptr, (int)msglen);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_REBOOT:
		syscall_reboot(current_task);
		// This syscall should never return
		break;
	case SYS_KILL: {
		u64 tid = REG_X0(context->regs);
		u64 kill_children = REG_X1(context->regs);
		i64 result = syscall_kill(current_task, (int)tid, (int)kill_children);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	case SYS_TOGGLE_IDLE_DISPLAY: {
		i64 result = syscall_toggle_idle_display(current_task);
		SYSCALL_SET_RESULT(current_task, result);
	} break;
	default:
		klog_error("Unknown syscall number: %#lx", syscall_num);
		break;
	}

	// klog_debug("handle_syscall end");
}

i64 syscall_create(task_t *current_task, int priority, void (*function)())
{
	i64 ret = 0;
	if (!is_valid_priority(priority)) {
		klog_error("[t:%d p:%d] syscall_create: invalid priority = %d", current_task->tid,
			   current_task->priority, priority);
		ret = -1;
	}

	if (ret == 0) {
		task_t *new_task = task_create(function, priority);
		if (new_task) {
			sched_add_task(new_task);
			ret = new_task->tid;
		}
	}
	klog_debug("[t:%d p:%d] syscall_create: priority = %d, function = %p -> %d", current_task->tid,
		   current_task->priority, priority, function, ret);
	return ret;
}

i64 syscall_mytid(task_t *current_task)
{
	int tid = current_task ? current_task->tid : 0;
	klog_debug("[t:%d p:%d] syscall_mytid: %d", current_task->tid, current_task->priority, tid);
	return tid;
}

i64 syscall_myparenttid(task_t *current_task)
{
	int parent_tid = current_task ? current_task->parent_tid : 0;
	klog_debug("[t:%d p:%d] syscall_myparenttid: %d", current_task->tid, current_task->priority, parent_tid);
	return parent_tid;
}

void syscall_yield(task_t *current_task)
{
	klog_debug("[t:%d p:%d] syscall_yield", current_task->tid, current_task->priority);
	sched_yield();
}

static void __syscall_unblock_waiting_tasks(task_t *unblock_task)
{
	SYSCALL_SET_RESULT(unblock_task, 0);
}

void syscall_exit(task_t *current_task)
{
	klog_debug("[t:%d p:%d] syscall_exit", current_task->tid, current_task->priority);
	if (current_task) {
		int exiting_tid = current_task->tid;

		sched_unblock_waiting_tasks(exiting_tid, __syscall_unblock_waiting_tasks);

		task_destroy(current_task);
		current_task = NULL;
		sched_schedule();
	}
}

i64 syscall_klog(task_t *current_task, u8 level, const char *msg)
{
	klog_internal(level, __func__, TOSTRING(__LINE__), "[t:%d p:%d] %s", current_task->tid, current_task->priority,
		      msg);
	return 0;
}

static i64 __syscall_send_finish(task_t *sender, int rplen)
{
	sender->ipc_reply_ptr = NULL;
	sender->ipc_reply_max_len = 0;
	sender->ipc_send_ptr = NULL;
	sender->ipc_send_len = 0;

	SYSCALL_SET_RESULT(sender, rplen);
	sched_unblock_task(sender);
	return 0;
}

static i64 __syscall_receive_finish(task_t *receiver, int msglen)
{
	receiver->ipc_receive_ptr = NULL;
	receiver->ipc_receive_max_len = 0;
	receiver->ipc_receive_tid = NULL;
	sched_unblock_task(receiver);
	SYSCALL_SET_RESULT(receiver, msglen);
	return 0;
}

i64 syscall_send(task_t *current_task, int tid, const char *msg, int msglen, char *reply, int rplen)
{
	klog_debug("[t:%d p:%d] syscall_send: tid=%d, msglen=%d, rplen=%d", current_task->tid, current_task->priority,
		   tid, msglen, rplen);
	task_t *receiver = task_get_by_id(tid);
	if (!receiver) {
		klog_error("[t:%d p:%d] syscall_send: invalid TID %d", current_task->tid, current_task->priority, tid);
		return -1; // Invalid TID
	}

	// Store message data in sender task for later retrieval
	current_task->ipc_send_ptr = (char *)msg;
	current_task->ipc_send_len = msglen;
	current_task->ipc_reply_ptr = reply;
	current_task->ipc_reply_max_len = rplen;

	if (receiver->state == TASK_STATE_BLOCKED && receiver->block_reason == TASK_BLOCK_IPC_RECEIVE) {
		klog_debug("[t:%d p:%d] syscall_send: receiver found, delivering message directly", current_task->tid,
			   current_task->priority);

		// Copy message with size check to prevent overflow
		int copy_len = min(msglen, (int)receiver->ipc_receive_max_len);
		memcpy(receiver->ipc_receive_ptr, msg, copy_len);
		*receiver->ipc_receive_tid = current_task->tid;

		__syscall_receive_finish(receiver, msglen); // Return actual message size, not truncated
	} else {
		klog_debug("[t:%d p:%d] syscall_send: receiver not ready, queuing sender", current_task->tid,
			   current_task->priority);
		dlist_insert_tail(&receiver->ipc_sender_queue, &current_task->ipc_sender_node);
	}

	// Block sender waiting for reply
	sched_block_task(current_task, TASK_BLOCK_IPC_REPLY);
	sched_schedule();

	// This should never be reached as the task is blocked
	panic("syscall_send: task resumed unexpectedly");
	return -2;
}

i64 syscall_receive(task_t *current_task, int *tid, char *msg, int msglen)
{
	if (dlist_is_empty(&current_task->ipc_sender_queue)) {
		klog_debug("[t:%d p:%d] syscall_receive: no sender, blocking task", current_task->tid,
			   current_task->priority);
		current_task->ipc_receive_ptr = msg;
		current_task->ipc_receive_max_len = msglen;
		current_task->ipc_receive_tid = tid;

		sched_block_task(current_task, TASK_BLOCK_IPC_RECEIVE);
		sched_schedule();

		// This should never be reached as the task is blocked
		panic("syscall_receive: task resumed unexpectedly");
		return -1;
	} else {
		klog_debug("[t:%d p:%d] syscall_receive: sender found, processing message", current_task->tid,
			   current_task->priority);
		struct dlist_node *next_sender_node = dlist_first(&current_task->ipc_sender_queue);
		task_t *next_sender = dlist_entry(next_sender_node, task_t, ipc_sender_node);
		dlist_del(next_sender_node);

		*tid = next_sender->tid;

		int copy_len = min((int)next_sender->ipc_send_len, msglen);
		memcpy(msg, next_sender->ipc_send_ptr, copy_len);

		return (i64)next_sender->ipc_send_len;
	}
}

i64 syscall_receive_nonblock(task_t *current_task, int *tid, char *msg, int msglen)
{
	if (dlist_is_empty(&current_task->ipc_sender_queue)) {
		klog_debug("[t:%d p:%d] syscall_receive_nonblock: no sender available", current_task->tid,
			   current_task->priority);
		return -1; // No sender available, return immediately
	} else {
		klog_debug("[t:%d p:%d] syscall_receive_nonblock: sender found, processing message", current_task->tid,
			   current_task->priority);
		struct dlist_node *next_sender_node = dlist_first(&current_task->ipc_sender_queue);
		task_t *next_sender = dlist_entry(next_sender_node, task_t, ipc_sender_node);
		dlist_del(next_sender_node);

		*tid = next_sender->tid;

		int copy_len = min((int)next_sender->ipc_send_len, msglen);
		memcpy(msg, next_sender->ipc_send_ptr, copy_len);

		return (i64)next_sender->ipc_send_len;
	}
}

i64 syscall_reply(task_t *current_task, int tid, const char *reply, int rplen)
{
	task_t *sender = task_get_by_id(tid);
	if (!sender) {
		// klog_error("[t:%d p:%d] syscall_reply: invalid TID %d", current_task->tid, current_task->priority, tid);
		return -1; // Invalid TID
	}

	if (sender->state != TASK_STATE_BLOCKED || sender->block_reason != TASK_BLOCK_IPC_REPLY) {
		klog_error("[t:%d p:%d] syscall_reply: task %d is not blocked on IPC", current_task->tid,
			   current_task->priority, tid);
		return -2; // Not blocked on IPC
	}

	// Copy reply with size check to prevent overflow
	int copy_len = min(rplen, (int)sender->ipc_reply_max_len);
	klog_debug("[t:%d p:%d] syscall_reply: copying %d bytes (requested %d) to task %d", current_task->tid,
		   current_task->priority, copy_len, rplen, tid);
	memcpy(sender->ipc_reply_ptr, reply, copy_len);

	__syscall_send_finish(sender, rplen); // Pass original size for truncation detection
	return copy_len; // Return actual bytes copied
}

i64 __noreturn syscall_panic(task_t *current_task, const char *msg)
{
	if (!msg) {
		klog_error("[t:%d p:%d] syscall_panic: null message pointer", current_task->tid,
			   current_task->priority);
		panic("User-triggered panic: <null message>");
	} else {
		klog_error("[t:%d p:%d] syscall_panic: user panic triggered: %s", current_task->tid,
			   current_task->priority, msg);
		panic("User-triggered panic: %s", msg);
	}
	UNREACHABLE();
}

i64 syscall_wait_tid(task_t *current_task, int tid)
{
	klog_debug("[t:%d p:%d] syscall_wait_tid: waiting for tid=%d", current_task->tid, current_task->priority, tid);

	task_t *target_task = task_get_by_id(tid);
	if (!target_task) {
		klog_error("[t:%d p:%d] syscall_wait_tid: invalid TID %d", current_task->tid, current_task->priority,
			   tid);
		return -1;
	}

	if (target_task->state == TASK_STATE_TERMINATED) {
		klog_debug("[t:%d p:%d] syscall_wait_tid: task %d already terminated", current_task->tid,
			   current_task->priority, tid);
		return 0;
	}

	// Don't allow a task to wait for itself
	if (tid == current_task->tid) {
		klog_error("[t:%d p:%d] syscall_wait_tid: task cannot wait for itself", current_task->tid,
			   current_task->priority);
		return -2;
	}

	current_task->wait_tid = tid;
	sched_block_task(current_task, TASK_BLOCK_WAIT_TID);
	sched_schedule();

	panic("syscall_wait_tid: task resumed unexpectedly");
	return -3;
}

i64 syscall_await_event(task_t *current_task, int event_id)
{
	klog_debug("[t:%d p:%d] syscall_await_event: event_id=%d", current_task->tid, current_task->priority, event_id);

	if (!is_valid_event_id(event_id)) {
		klog_error("[t:%d p:%d] syscall_await_event: invalid event ID %d", current_task->tid,
			   current_task->priority, event_id);
		return EVENT_ERROR;
	}

	current_task->event_id = event_id;

	sched_block_task(current_task, TASK_BLOCK_AWAIT_EVENT);
	sched_schedule();

	panic("syscall_await_event: task resumed unexpectedly");
	return -2;
}

i64 syscall_setup_idle_task(task_t *current_task, idle_stats_t *idle_stats)
{
	klog_debug("[t:%d p:%d] syscall_setup_idle_task: idle_stats=%p", current_task->tid, current_task->priority,
		   idle_stats);
	setup_idle_task(current_task, idle_stats);
	return 0;
}

i64 syscall_get_unread_klogs(task_t *current_task, char *buffer, int buffer_size, int *num_entries)
{
	klog_debug("[t:%d p:%d] syscall_get_unread_klogs: buffer=%p, buffer_size=%d", current_task->tid,
		   current_task->priority, buffer, buffer_size);

	if (!buffer || buffer_size <= 0 || !num_entries) {
		klog_error("[t:%d p:%d] syscall_get_unread_klogs: invalid parameters", current_task->tid,
			   current_task->priority);
		return -1;
	}

	int result = klog_read_all_unread_formatted(buffer, buffer_size, (size_t *)num_entries);

	klog_debug("[t:%d p:%d] syscall_get_unread_klogs: returning %d entries, %d bytes", current_task->tid,
		   current_task->priority, *num_entries, result);
	return result; // Return bytes written
}

i64 syscall_get_task_info(task_t *current_task, char *buffer, int buffer_size)
{
	klog_debug("[t:%d p:%d] syscall_get_task_info: buffer=%p, buffer_size=%d", current_task->tid,
		   current_task->priority, buffer, buffer_size);

	if (!buffer || buffer_size <= 0) {
		klog_error("[t:%d p:%d] syscall_get_task_info: invalid parameters", current_task->tid,
			   current_task->priority);
		return -1;
	}

	int result = task_format_info(buffer, buffer_size);
	if (result < 0) {
		klog_error("[t:%d p:%d] syscall_get_task_info: failed to format task info", current_task->tid,
			   current_task->priority);
		return -2;
	}

	klog_debug("[t:%d p:%d] syscall_get_task_info: returning %d bytes", current_task->tid, current_task->priority,
		   result);
	return result; // Return bytes written
}

void __noreturn syscall_reboot(task_t *current_task)
{
	klog_info("[t:%d p:%d] syscall_reboot: system reboot requested", current_task->tid, current_task->priority);
	_reboot();
	UNREACHABLE();
}

static void __syscall_kill_children(task_t *current_task, int parent_tid)
{
	// We need to iterate through all tasks to find children
	for (int i = 1; i < MAX_TASKS; i++) {
		task_t *task = task_get_by_id(i);
		if (task && task->parent_tid == parent_tid && task->state != TASK_STATE_TERMINATED) {
			klog_debug("[t:%d p:%d] syscall_kill: killing child task %d (parent %d)", current_task->tid,
				   current_task->priority, i, parent_tid);

			__syscall_kill_children(current_task, i);

			sched_unblock_waiting_tasks(i, __syscall_unblock_waiting_tasks);
			task_destroy(task);
		}
	}
}

i64 syscall_kill(task_t *current_task, int tid, int kill_children)
{
	klog_debug("[t:%d p:%d] syscall_kill: killing tid=%d, kill_children=%d", current_task->tid,
		   current_task->priority, tid, kill_children);

	task_t *target_task = task_get_by_id(tid);
	if (!target_task) {
		klog_error("[t:%d p:%d] syscall_kill: invalid TID %d", current_task->tid, current_task->priority, tid);
		return -1;
	}

	if (target_task->state == TASK_STATE_TERMINATED) {
		klog_debug("[t:%d p:%d] syscall_kill: task %d already terminated", current_task->tid,
			   current_task->priority, tid);
		return 0;
	}

	// Don't allow a task to kill itself
	if (tid == current_task->tid) {
		klog_error("[t:%d p:%d] syscall_kill: task cannot kill itself", current_task->tid,
			   current_task->priority);
		return -2;
	}

	// Kill children first if requested
	if (kill_children) {
		klog_debug("[t:%d p:%d] syscall_kill: killing children of task %d", current_task->tid,
			   current_task->priority, tid);
		__syscall_kill_children(current_task, tid);
	}

	// Unblock any tasks waiting for the target task
	sched_unblock_waiting_tasks(tid, __syscall_unblock_waiting_tasks);

	// Terminate the target task
	task_destroy(target_task);

	klog_debug("[t:%d p:%d] syscall_kill: successfully killed task %d (children: %s)", current_task->tid,
		   current_task->priority, tid, kill_children ? "yes" : "no");
	return 0;
}

i64 syscall_toggle_idle_display(task_t *current_task)
{
	klog_debug("[t:%d p:%d] syscall_toggle_idle_display", current_task->tid, current_task->priority);
	
	extern idle_stats_t *idle_stats;
	if (idle_stats == NULL) {
		klog_error("[t:%d p:%d] syscall_toggle_idle_display: idle_stats not initialized", current_task->tid,
			   current_task->priority);
		return -1;
	}
	
	idle_stats->display_enabled = !idle_stats->display_enabled;
	
	klog_debug("[t:%d p:%d] syscall_toggle_idle_display: display_enabled = %s", current_task->tid,
		   current_task->priority, idle_stats->display_enabled ? "true" : "false");
	
	return idle_stats->display_enabled ? 1 : 0;
}
