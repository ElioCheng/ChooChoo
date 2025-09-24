#include "task.h"
#include "event.h"
#include "sched.h"
#include "klog.h"
#include "arch/registers.h"
#include "string.h"
#include "context.h"
#include "symbol.h"
#include "params.h"
#include "printf.h"
#include "types.h"
#include <stddef.h>

// External stack area symbols
extern char __user_stacks_start[];
char *__user_stacks_end = (char *)__user_stacks_start + (MAX_TASKS * TASK_STACK_SIZE);

// Task table
static task_t task_table[MAX_TASKS];
static bool task_id_used[MAX_TASKS];

// Stack allocation bitmap
static bool stack_allocated[MAX_TASKS];

void task_init(void)
{
	int i;

	for (i = 0; i < MAX_TASKS; i++) {
		task_table[i].tid = -1;
		task_table[i].state = TASK_STATE_TERMINATED;
		task_table[i].block_reason = TASK_BLOCK_NONE;
		task_table[i].wait_tid = -1;
		task_id_used[i] = false;
		stack_allocated[i] = false;

		dlist_init_node(&task_table[i].ready_queue_node);
		dlist_init_node(&task_table[i].blocked_queue_node);
		dlist_init(&task_table[i].ipc_sender_queue);
	}

	klog_info("Task system initialized");
}

int task_alloc_tid(void)
{
	for (int i = 1; i < MAX_TASKS; i++) {
		if (!task_id_used[i]) {
			task_id_used[i] = true;
			return i;
		}
	}
	return -1;
}

void task_free_tid(int tid)
{
	if (tid > 0 && tid < MAX_TASKS) {
		task_id_used[tid] = false;
	}
}

task_t *task_get_by_id(int tid)
{
	if (tid <= 0 || tid >= MAX_TASKS || !task_id_used[tid]) {
		return NULL;
	}
	return &task_table[tid];
}

void *task_alloc_stack(int task_id)
{
	if (task_id < 0 || task_id >= MAX_TASKS) {
		return NULL;
	}

	if (stack_allocated[task_id]) {
		klog_error("Stack already allocated for task %d", task_id);
		return NULL;
	}

	void *stack_base = __user_stacks_start + (task_id * TASK_STACK_SIZE);

	if (stack_base + TASK_STACK_SIZE > (void *)__user_stacks_end) {
		klog_error("Stack allocation out of bounds for task %d", task_id);
		return NULL;
	}

	stack_allocated[task_id] = true;

	klog_debug("Allocated stack for task %d at %p (size: %d bytes)", task_id, stack_base, TASK_STACK_SIZE);

	return stack_base;
}

void task_free_stack(void *stack_base)
{
	if (!stack_base)
		return;

	ptrdiff_t offset = (char *)stack_base - __user_stacks_start;
	int task_id = offset / TASK_STACK_SIZE;

	if (task_id >= 0 && task_id < MAX_TASKS) {
		stack_allocated[task_id] = false;
		klog_debug("Freed stack for task %d", task_id);
	}
}

void task_setup_stack(task_t *task, void (*entry_point)(void))
{
	if (!task || !task->stack_base)
		return;

	// Stack grows downward
	task->stack_top = (char *)task->stack_base + task->stack_size;

	memset(&task->context, 0, sizeof(context_t));

	klog_debug("Set up stack for task %d: base=%p, top=%p, entry=%p", task->tid, task->stack_base, task->stack_top,
		   entry_point);
}

task_t *task_create(void (*entry_point)(void), int priority)
{
	if (!entry_point || priority < 0 || priority >= MAX_PRIORITIES) {
		klog_error("Invalid task parameters");
		return NULL;
	}

	int tid = task_alloc_tid();
	if (tid < 0) {
		klog_error("No available task IDs");
		return NULL;
	}

	task_t *task = &task_table[tid];

	void *stack_base = task_alloc_stack(tid);
	if (!stack_base) {
		task_free_tid(tid);
		klog_error("Failed to allocate stack for task %d", tid);
		return NULL;
	}

	memset(stack_base, 0, TASK_STACK_SIZE);

	task->tid = tid;
	task->parent_tid = current_task ? current_task->tid : 0;
	task->priority = priority;
	task->state = TASK_STATE_READY;
	task->block_reason = TASK_BLOCK_NONE;
	task->wait_tid = -1;
	task->entry_point = entry_point;
	task->stack_base = stack_base;
	task->stack_size = TASK_STACK_SIZE;

	task_setup_stack(task, entry_point);

	dlist_init_node(&task->ready_queue_node);
	dlist_init_node(&task->blocked_queue_node);
	dlist_init(&task->ipc_sender_queue);

	context_init(&task->context, task->stack_top, entry_point);

	klog_debug("Created task %d (priority %d) with entry point %p", tid, priority, entry_point);

	return task;
}

void task_destroy(task_t *task)
{
	if (!task)
		return;

	klog_debug("Destroying task %d", task->tid);

	sched_remove_task(task);

	task_free_stack(task->stack_base);
	task_free_tid(task->tid);

	task->tid = -1;
	task->state = TASK_STATE_TERMINATED;
}

void task_set_state(task_t *task, task_state_t state)
{
	if (!task)
		return;

	klog_debug("Task %d state change: %s -> %s", task->tid, task_state_to_string(task->state),
		   task_state_to_string(state));

	task->state = state;
}

const char *task_state_to_string(task_state_t state)
{
	switch (state) {
	case TASK_STATE_ACTIVE:
		return "ACTIVE";
	case TASK_STATE_READY:
		return "READY";
	case TASK_STATE_BLOCKED:
		return "BLOCKED";
	case TASK_STATE_TERMINATED:
		return "TERMINATED";
	}
	return "UNKNOWN";
}

const char *task_block_reason_to_string(task_block_reason_t reason)
{
	switch (reason) {
	case TASK_BLOCK_NONE:
		return "NONE";
	case TASK_BLOCK_TIMER:
		return "TIMER";
	case TASK_BLOCK_IPC_RECEIVE:
		return "IPC_RECEIVE";
	case TASK_BLOCK_IPC_REPLY:
		return "IPC_REPLY";
	case TASK_BLOCK_WAIT_TID:
		return "WAIT_TID";
	case TASK_BLOCK_AWAIT_EVENT:
		return "AWAIT_EVENT";
	}
	return "UNKNOWN";
}

void task_dump(void)
{
	klog_debug("Dumping task table");
	klog_debug("Current Task: %d", current_task ? current_task->tid : -1);
	// klog_debug("Last Scheduled Task: %d", kernel_scheduler.last_scheduled_task ? kernel_scheduler.last_scheduled_task->tid : -1);
	for (int i = 0; i < MAX_TASKS; i++) {
		if (task_id_used[i]) {
			task_t *task = &task_table[i];
			if (task->state == TASK_STATE_BLOCKED) {
				switch (task->block_reason) {
				case TASK_BLOCK_AWAIT_EVENT:
					klog_debug(
						"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d, block_reason=%s, awaiting_event=%s",
						i, task_state_to_string(task_table[i].state), task_table[i].priority,
						task_table[i].entry_point,
						symbol_lookup((uint64_t)task_table[i].entry_point), task_table[i].tid,
						task_table[i].parent_tid,
						task_block_reason_to_string(task_table[i].block_reason),
						event_id_to_string(task_table[i].event_id));
					break;
				case TASK_BLOCK_WAIT_TID:
					klog_debug(
						"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d, block_reason=%s, wait_tid=%d",
						i, task_state_to_string(task_table[i].state), task_table[i].priority,
						task_table[i].entry_point,
						symbol_lookup((uint64_t)task_table[i].entry_point), task_table[i].tid,
						task_table[i].parent_tid,
						task_block_reason_to_string(task_table[i].block_reason),
						task_table[i].wait_tid);
					break;
				case TASK_BLOCK_IPC_RECEIVE:
				case TASK_BLOCK_IPC_REPLY:
					klog_debug(
						"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d, block_reason=%s",
						i, task_state_to_string(task_table[i].state), task_table[i].priority,
						task_table[i].entry_point,
						symbol_lookup((uint64_t)task_table[i].entry_point), task_table[i].tid,
						task_table[i].parent_tid,
						task_block_reason_to_string(task_table[i].block_reason));
					break;
				case TASK_BLOCK_TIMER:
				default:
					klog_debug(
						"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d, block_reason=%s",
						i, task_state_to_string(task_table[i].state), task_table[i].priority,
						task_table[i].entry_point,
						symbol_lookup((uint64_t)task_table[i].entry_point), task_table[i].tid,
						task_table[i].parent_tid,
						task_block_reason_to_string(task_table[i].block_reason));
					break;
				}
			} else {
				klog_debug(
					"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d",
					i, task_state_to_string(task_table[i].state), task_table[i].priority,
					task_table[i].entry_point, symbol_lookup((uint64_t)task_table[i].entry_point),
					task_table[i].tid, task_table[i].parent_tid);
			}
		}
	}
}

int task_format_info(char *buffer, int buffer_size)
{
	if (!buffer || buffer_size <= 0) {
		return -1;
	}

	int offset = 0;
	int remaining = buffer_size;
	int written;

	// Add header information
	written = snprintf(buffer + offset, remaining, "=== TASK TABLE DUMP ===\n");
	if (written >= remaining)
		return -1;
	offset += written;
	remaining -= written;

	written = snprintf(buffer + offset, remaining, "Current Task: %d\n", current_task ? current_task->tid : -1);
	if (written >= remaining)
		return -1;
	offset += written;
	remaining -= written;

	// Iterate through tasks
	for (int i = 0; i < MAX_TASKS; i++) {
		if (task_id_used[i]) {
			task_t *task = &task_table[i];
			if (task->state == TASK_STATE_BLOCKED) {
				switch (task->block_reason) {
				case TASK_BLOCK_AWAIT_EVENT:
					written = snprintf(
						buffer + offset, remaining,
						"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d, block_reason=%s, awaiting_event=%s\n",
						i, task_state_to_string(task_table[i].state), task_table[i].priority,
						task_table[i].entry_point,
						symbol_lookup((uint64_t)task_table[i].entry_point), task_table[i].tid,
						task_table[i].parent_tid,
						task_block_reason_to_string(task_table[i].block_reason),
						event_id_to_string(task_table[i].event_id));
					break;
				case TASK_BLOCK_WAIT_TID:
					written = snprintf(
						buffer + offset, remaining,
						"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d, block_reason=%s, wait_tid=%d\n",
						i, task_state_to_string(task_table[i].state), task_table[i].priority,
						task_table[i].entry_point,
						symbol_lookup((uint64_t)task_table[i].entry_point), task_table[i].tid,
						task_table[i].parent_tid,
						task_block_reason_to_string(task_table[i].block_reason),
						task_table[i].wait_tid);
					break;
				case TASK_BLOCK_IPC_RECEIVE:
				case TASK_BLOCK_IPC_REPLY:
				case TASK_BLOCK_TIMER:
				default:
					written = snprintf(
						buffer + offset, remaining,
						"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d, block_reason=%s\n",
						i, task_state_to_string(task_table[i].state), task_table[i].priority,
						task_table[i].entry_point,
						symbol_lookup((uint64_t)task_table[i].entry_point), task_table[i].tid,
						task_table[i].parent_tid,
						task_block_reason_to_string(task_table[i].block_reason));
					break;
				}
			} else {
				written = snprintf(
					buffer + offset, remaining,
					"Task %d: state=%s, priority=%d, entry_point=%p in %s, tid=%d, parent_tid=%d\n",
					i, task_state_to_string(task_table[i].state), task_table[i].priority,
					task_table[i].entry_point, symbol_lookup((uint64_t)task_table[i].entry_point),
					task_table[i].tid, task_table[i].parent_tid);
			}

			if (written >= remaining)
				return -1;
			offset += written;
			remaining -= written;
		}
	}

	return offset; // Return total bytes written
}
