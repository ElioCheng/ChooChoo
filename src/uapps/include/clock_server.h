#ifndef __CLOCK_SERVER_H__
#define __CLOCK_SERVER_H__

#include "clock.h"
#include "types.h"
#include "dlist.h"
#include "params.h"

#define MAX_DELAYED_TASKS MAX_TASKS
#define CLOCK_SERVER_PRIORITY 3

typedef struct delayed_task {
	int tid;
	int wake_time_tick;
	struct dlist_node node;
} delayed_task_t;

typedef struct {
	int current_time_tick;
	struct dlist_node delay_list;
	delayed_task_t task_pool[MAX_DELAYED_TASKS];
	int free_tasks[MAX_DELAYED_TASKS];
	int free_tasks_head;
	int free_tasks_tail;
	int tasks_count;
} clock_server_state_t;

void clock_server_main(void);

void clock_notifier_main(void);

#endif /* __CLOCK_SERVER_H__ */
