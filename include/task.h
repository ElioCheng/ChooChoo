#ifndef __TASK_H__
#define __TASK_H__

#include "params.h"

#ifdef __KERNEL__
#include "types.h"
#include "dlist.h"
#include "context.h"

typedef enum task_state {
	TASK_STATE_ACTIVE, // Currently running
	TASK_STATE_READY, // Ready to run
	TASK_STATE_BLOCKED, // Blocked waiting for something
	TASK_STATE_TERMINATED, // Task has finished
} task_state_t;

typedef enum task_block_reason {
	TASK_BLOCK_NONE, // Not blocked
	TASK_BLOCK_TIMER, // Blocked waiting for timer
	TASK_BLOCK_IPC_RECEIVE, // Blocked waiting for a message
	TASK_BLOCK_IPC_REPLY, // Blocked waiting for a reply in IPC
	TASK_BLOCK_WAIT_TID, // Blocked waiting for another task to exit
	TASK_BLOCK_AWAIT_EVENT, // Blocked waiting for an event
} task_block_reason_t;

typedef struct task {
	int tid; // Task ID
	int parent_tid; // Parent task ID

	int priority; // Task priority (0 = highest)
	task_state_t state; // Current task state

	// Blocking information
	task_block_reason_t block_reason; // Why the task is blocked
	int block_ipc_tid; // TID of the task this task is waiting for in IPC
	int wait_tid; // TID of the task this task is waiting to exit
	int event_id; // Event ID the task is waiting for (if blocked on event)
	char *ipc_send_ptr; // Pointer to IPC send buffer
	size_t ipc_send_len; // Length of IPC send buffer
	char *ipc_receive_ptr; // Pointer to IPC receive buffer
	int *ipc_receive_tid; // Pointer to TID that sent the message
	size_t ipc_receive_max_len; // Maximum length of IPC receive buffer
	char *ipc_reply_ptr; // Pointer to IPC reply buffer
	size_t ipc_reply_max_len; // Maximum length of IPC reply buffer

	// Context and stack management
	context_t context; // Saved registers when not running
	void *stack_base; // Bottom of stack (lowest address)
	void *stack_top; // Top of stack (highest address, initial SP)
	size_t stack_size; // Stack size in bytes

	// Entry point for new tasks
	void (*entry_point)(void);

	// Scheduling queues - intrusive linkage
	struct dlist_node ready_queue_node; // For ready queue
	struct dlist_node blocked_queue_node; // For blocked queue

	// For message passing
	struct dlist_node ipc_sender_queue; // For IPC
	struct dlist_node ipc_sender_node; // Node in sender queue
} task_t;

// Current running task
extern task_t *current_task;

// Task management functions
void task_init(void);
task_t *task_create(void (*entry_point)(void), int priority);
void task_destroy(task_t *task);

// Stack management
void *task_alloc_stack(int task_id);
void task_free_stack(void *stack_base);
void task_setup_stack(task_t *task, void (*entry_point)(void));

// Task state management
void task_set_state(task_t *task, task_state_t state);
const char *task_state_to_string(task_state_t state);

// Task table management
task_t *task_get_by_id(int tid);
int task_alloc_tid(void);
void task_free_tid(int tid);

void task_dump(void);
int task_format_info(char *buffer, int buffer_size);
#endif /* __KERNEL__ */

#endif /* __TASK_H__ */
