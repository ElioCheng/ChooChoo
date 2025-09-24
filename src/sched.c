#include "sched.h"
#include "arch/cpu.h"
#include "arch/rpi.h"
#include "dlist.h"
#include "symbol.h"
#include "syscall.h"
#include "task.h"
#include "klog.h"
#include "arch/registers.h"
#include "context.h"
#include "timer/timer.h"
#include "uart.h"
#include "idle.h"
#include "panic.h"
#include <stddef.h>

struct scheduler kernel_scheduler;

task_t *current_task = NULL;

void sched_init(void)
{
	int i;
	current_task = NULL;
	for (i = 0; i < MAX_PRIORITIES; i++) {
		dlist_init(&kernel_scheduler.ready_queues[i]);
	}

	dlist_init(&kernel_scheduler.blocked_queue);

	for (i = 0; i < PRIORITY_BITMAP_SIZE; i++) {
		kernel_scheduler.priority_bitmap[i] = 0;
	}

	kernel_scheduler.total_tasks = 0;
	kernel_scheduler.active_tasks = 0;

	klog_info("Scheduler initialized");
}

void sched_add_task(task_t *task)
{
	if (!task)
		return;

	dlist_init_node(&task->ready_queue_node);
	dlist_init_node(&task->blocked_queue_node);
	dlist_init(&task->ipc_sender_queue);

	task->state = TASK_STATE_READY;
	sched_enqueue_ready(task);

	// DLIST_PRINT(&kernel_scheduler.ready_queues[task->priority], task_t, ready_queue, "tid: %d", tid, 5);

	kernel_scheduler.total_tasks++;
	kernel_scheduler.active_tasks++;

	klog_debug("Added task %d (priority %d) to scheduler (total tasks %d)", task->tid, task->priority,
		   kernel_scheduler.total_tasks);
}

void sched_remove_task(task_t *task)
{
	if (!task)
		return;

	if (task->state == TASK_STATE_READY) {
		dlist_del(&task->ready_queue_node);
		if (dlist_is_empty(&kernel_scheduler.ready_queues[task->priority])) {
			sched_clear_priority_bit(task->priority);
		}
	} else if (task->state == TASK_STATE_BLOCKED) {
		dlist_del(&task->blocked_queue_node);
	}

	// DLIST_PRINT(&kernel_scheduler.ready_queues[task->priority], task_t, ready_queue, "tid: %d", tid, 5);

	task->state = TASK_STATE_TERMINATED;
	kernel_scheduler.active_tasks--;

	if (task == current_task) {
		current_task = NULL;
	}

	klog_debug("Removed task %d from scheduler", task->tid);
}

void sched_block_current(int block_reason)
{
	sched_block_task(current_task, block_reason);
	current_task = NULL;
}

void sched_block_task(task_t *task, int block_reason)
{
	if (!task)
		return;

	if (task->state == TASK_STATE_READY) {
		dlist_del(&task->ready_queue_node);
		if (dlist_is_empty(&kernel_scheduler.ready_queues[task->priority])) {
			sched_clear_priority_bit(task->priority);
		}
	}

	task->state = TASK_STATE_BLOCKED;
	task->block_reason = block_reason;
	dlist_insert_tail(&kernel_scheduler.blocked_queue, &task->blocked_queue_node);

	klog_debug("Blocked task %d", task->tid);
}

void sched_unblock_task(task_t *task)
{
	if (!task || task->state != TASK_STATE_BLOCKED)
		return;

	dlist_del(&task->blocked_queue_node);

	task->state = TASK_STATE_READY;
	task->block_reason = TASK_BLOCK_NONE;
	task->wait_tid = -1;
	sched_enqueue_ready(task);

	klog_debug("Unblocked task %d", task->tid);
}

void sched_unblock_waiting_tasks(int exited_tid, void (*callback)(task_t *task))
{
	klog_debug("Unblocking tasks waiting for TID %d to exit", exited_tid);

	task_t *task;
	struct dlist_node *n;

	dlist_for_each_entry_safe(task, n, &kernel_scheduler.blocked_queue, task_t, blocked_queue_node)
	{
		if (task->state != TASK_STATE_BLOCKED) {
			continue;
		}

		if (task->block_reason == TASK_BLOCK_WAIT_TID && task->wait_tid == exited_tid) {
			klog_debug("Unblocking task %d that was waiting for TID %d", task->tid, exited_tid);
			sched_unblock_task(task);
			callback(task);
		}
	}
}

void sched_unblock_event_tasks(int event_id, int event_data)
{
	klog_debug("Unblocking tasks waiting for event %d", event_id);

	task_t *task;
	struct dlist_node *n;

	dlist_for_each_entry_safe(task, n, &kernel_scheduler.blocked_queue, task_t, blocked_queue_node)
	{
		if (task->state != TASK_STATE_BLOCKED) {
			continue;
		}

		if (task->block_reason == TASK_BLOCK_AWAIT_EVENT && task->event_id == event_id) {
			klog_debug("Unblocking task %d that was waiting for event %d", task->tid, event_id);
			REG_X0(task->context.regs) = event_data; // SYSCALL_AWAIT_EVENT return value set here
			sched_unblock_task(task);
		}
	}
}

void sched_enqueue_ready(task_t *task)
{
	if (!task || task->priority >= MAX_PRIORITIES)
		return;

	// If task is already in READY state and in a ready queue, don't re-enqueue
	if (task->state == TASK_STATE_READY) {
		// Check if the node is already linked (not pointing to itself)
		if (task->ready_queue_node.next != &task->ready_queue_node &&
		    task->ready_queue_node.prev != &task->ready_queue_node) {
			klog_debug("Task %d (priority %d) already in ready queue, skipping enqueue", task->tid,
				   task->priority);
			return;
		}
	}

	// If the task was in a ready queue before, remove it first
	if (task->ready_queue_node.next != &task->ready_queue_node &&
	    task->ready_queue_node.prev != &task->ready_queue_node) {
		dlist_del(&task->ready_queue_node);
		// Check if the old priority queue is now empty
		if (dlist_is_empty(&kernel_scheduler.ready_queues[task->priority])) {
			sched_clear_priority_bit(task->priority);
		}
	}

	// klog_debug("Enqueuing task %d (priority %d)", task->tid, task->priority);
	dlist_insert_tail(&kernel_scheduler.ready_queues[task->priority], &task->ready_queue_node);
	// DLIST_PRINT(&kernel_scheduler.ready_queues[task->priority], task_t, ready_queue, "tid: %d", tid, 5);
	// uart_process_tx_buffers_blocking();
	int len = dlist_len(&kernel_scheduler.ready_queues[task->priority]);
	klog_debug("Enqueued task %d (priority %d), ready queue length %d", task->tid, task->priority, len);

	sched_set_priority_bit(task->priority);

	task->state = TASK_STATE_READY;
}

task_t *sched_dequeue_ready(int priority)
{
	if (priority >= MAX_PRIORITIES)
		return NULL;

	struct dlist_node *ready_queue = &kernel_scheduler.ready_queues[priority];

	if (dlist_is_empty(ready_queue)) {
		return NULL;
	}

	struct dlist_node *first = dlist_first(ready_queue);
	task_t *task = dlist_entry(first, task_t, ready_queue_node);

	dlist_del(&task->ready_queue_node);

	// DLIST_PRINT(&kernel_scheduler.ready_queues[task->priority], task_t, ready_queue, "tid: %d", tid, 5);

	if (dlist_is_empty(ready_queue)) {
		sched_clear_priority_bit(priority);
	}

	int len = dlist_len(ready_queue);

	klog_debug("Dequeued task %d (priority %d), ready queue length %d", task->tid, task->priority, len);

	return task;
}

void sched_set_priority_bit(int priority)
{
	if (priority >= MAX_PRIORITIES)
		return;
	set_bit(kernel_scheduler.priority_bitmap, priority);
}

void sched_clear_priority_bit(int priority)
{
	if (priority >= MAX_PRIORITIES)
		return;
	clear_bit(kernel_scheduler.priority_bitmap, priority);
}

int sched_find_highest_priority(void)
{
	for (int word = 0; word < PRIORITY_BITMAP_SIZE; word++) {
		if (kernel_scheduler.priority_bitmap[word] != 0) {
			int bit_in_word = ffs_u32(kernel_scheduler.priority_bitmap[word]) - 1;
			return word * 32 + bit_in_word;
		}
	}
	return -1;
}

task_t *sched_pick_next(void)
{
	int highest_priority = sched_find_highest_priority();
	if (highest_priority < 0) {
		return NULL;
	}
	klog_debug("Picking next task with highest priority %d", highest_priority);

	// Get the first task at this priority
	task_t *next_task = sched_dequeue_ready(highest_priority);
	if (!next_task) {
		return NULL;
	}

	return next_task;
}

void sched_yield(void)
{
	if (current_task && current_task->state == TASK_STATE_ACTIVE) {
		klog_debug("Yielded by task %d (priority %d)", current_task->tid, current_task->priority);
	}

	sched_schedule();
}

void sched_schedule()
{
	task_dump();
	if (current_task) {
		task_t *last_scheduled_task = current_task;

		if (task_is_idle_task((struct task *)last_scheduled_task)) {
			idle_stop_accounting();
		}

		if (last_scheduled_task->state == TASK_STATE_READY || last_scheduled_task->state == TASK_STATE_ACTIVE) {
			sched_enqueue_ready(last_scheduled_task);
		}
		current_task = NULL;
	}

	task_t *next_task = sched_pick_next();

	if (!next_task) {
		klog_error("No ready tasks to schedule (this should not happen with idle task)");
		panic("Scheduler found no ready tasks");
	}
	// klog_debug("Picked next task %d (priority %d)", next_task->tid, next_task->priority);

	if (current_task && (current_task->state == TASK_STATE_ACTIVE)) {
		current_task->state = TASK_STATE_READY;
		sched_enqueue_ready(current_task);
	}

	if (task_is_idle_task((struct task *)next_task)) {
		idle_start_accounting();
	}

	next_task->state = TASK_STATE_ACTIVE;
	current_task = next_task;

	klog_debug("Scheduling task %d (priority %d)", next_task->tid, next_task->priority);

	context_switch_to(next_task);
}

extern void switch_to_user_mode(context_t *context);

void context_switch_to(task_t *next_task)
{
	klog_debug("Switching to task %d (%s) (priority %d) (@%p in %s) (Kernel SP: %p, User SP: %p)", next_task->tid,
		   symbol_lookup((uint64_t)next_task->entry_point), next_task->priority,
		   REG_PC(next_task->context.regs), symbol_lookup((uint64_t)REG_PC(next_task->context.regs)), get_sp(),
		   REG_SP(next_task->context.regs));
	update_gpio_indicator(next_task->tid);

	if (!next_task)
		return;

	switch_to_user_mode(&next_task->context);
}
