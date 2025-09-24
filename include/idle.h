#ifndef __IDLE_H__
#define __IDLE_H__

#include "task.h"
#include "types.h"
#include "compiler.h"
#include "uapi/idle.h"

extern task_t *idle_task;
extern idle_stats_t *idle_stats;
extern bool is_idle_running;

void idle_start_accounting(void);
void idle_stop_accounting(void);

static inline bool task_is_idle_task(task_t *task)
{
	return task == idle_task;
}

void setup_idle_task(task_t *task, idle_stats_t *idle_stats);

void idle_init_stats(idle_stats_t *stats, u64 window_ms);

#endif /* __IDLE_H__ */
