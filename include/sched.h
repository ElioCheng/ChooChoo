#ifndef __SCHED_H__
#define __SCHED_H__

#include "types.h"
#include "dlist.h"
#include "task.h"
#include "params.h"

#define PRIORITY_BITMAP_SIZE ((MAX_PRIORITIES + 31) / 32)

// Priority-based scheduling system
struct scheduler {
	// Array of FIFO ready queues, one per priority (0 = highest priority)
	struct dlist_node ready_queues[MAX_PRIORITIES];

	// Bitmap to track non-empty priority queues
	// Bit i is set if priority i queue is non-empty
	u32 priority_bitmap[PRIORITY_BITMAP_SIZE];

	// Blocked tasks queue
	struct dlist_node blocked_queue;

	// Total number of tasks
	int total_tasks;
	int active_tasks;
};

// Global scheduler instance
extern struct scheduler kernel_scheduler;

// Scheduling functions
void sched_init(void);
void sched_add_task(task_t *task);
void sched_remove_task(task_t *task);
void sched_block_current(int block_reason);
void sched_block_task(task_t *task, int block_reason);
void sched_unblock_task(task_t *task);
void sched_unblock_waiting_tasks(int exited_tid, void (*callback)(task_t *task));
void sched_unblock_event_tasks(int event_id, int event_data);
task_t *sched_pick_next(void);
void sched_yield(void);
void sched_schedule();

// Priority queue management
void sched_enqueue_ready(task_t *task);
task_t *sched_dequeue_ready(int priority);
void sched_set_priority_bit(int priority);
void sched_clear_priority_bit(int priority);
int sched_find_highest_priority(void);

// Context switching
void context_switch_to(task_t *next_task);

// Helper macros for bit manipulation
static inline int ffs_u32(u32 word)
{
	if (word == 0)
		return 0;
	return __builtin_ffs(word);
}

static inline void set_bit(u32 *bitmap, int bit)
{
	int word = bit / 32;
	int offset = bit % 32;
	bitmap[word] |= (1U << offset);
}

static inline void clear_bit(u32 *bitmap, int bit)
{
	int word = bit / 32;
	int offset = bit % 32;
	bitmap[word] &= ~(1U << offset);
}

static inline bool test_bit(const u32 *bitmap, int bit)
{
	int word = bit / 32;
	int offset = bit % 32;
	return (bitmap[word] & (1U << offset)) != 0;
}

static inline bool is_valid_priority(int priority)
{
	return priority >= 0 && priority < MAX_PRIORITIES;
}

#endif /* __SCHED_H__ */
