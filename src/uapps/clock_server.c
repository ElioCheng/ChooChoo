#include "clock_server.h"
#include "compiler.h"
#include "dlist.h"
#include "name.h"
#include "syscall.h"
#include "task.h"
#include "event.h"

static void init_task_pool(clock_server_state_t *state)
{
	state->tasks_count = 0;
	state->free_tasks_head = 0;
	state->free_tasks_tail = MAX_DELAYED_TASKS - 1; // Start with tail at end of array
	for (int i = 0; i < MAX_DELAYED_TASKS; i++) {
		state->free_tasks[i] = i;
		state->task_pool[i].tid = -1; // Mark as unused
		dlist_init_node(&state->task_pool[i].node);
	}
}

static int add_delayed_task(clock_server_state_t *state, int tid, int wake_time)
{
	if (state->tasks_count >= MAX_DELAYED_TASKS) {
		return CLOCK_ERR_INVALID_TID; // No free task slots
	}

	int next_free_index = state->free_tasks[state->free_tasks_head];
	state->task_pool[next_free_index] = (delayed_task_t){ tid, wake_time, { 0 } };
	dlist_init_node(&state->task_pool[next_free_index].node);
	state->free_tasks_head = (state->free_tasks_head + 1) % MAX_DELAYED_TASKS;
	state->tasks_count++;
	struct dlist_node *prev = &state->delay_list;

	delayed_task_t *pos;
	struct dlist_node *n;
	dlist_for_each_entry_safe(pos, n, &state->delay_list, delayed_task_t, node)
	{
		if (pos->wake_time_tick > wake_time) {
			break; // Found the right position to insert
		}
		prev = &pos->node;
	}
	dlist_insert(prev, &state->task_pool[next_free_index].node);
	return CLOCK_SUCCESS;
}

static void wake_expired_tasks(clock_server_state_t *state)
{
	struct dlist_node *pos, *n;
	dlist_for_each_safe(pos, n, &state->delay_list)
	{
		delayed_task_t *task = dlist_entry(pos, delayed_task_t, node);
		if (task->wake_time_tick <= state->current_time_tick) {
			// Wake up the task with current time
			clock_reply_t reply;
			reply.time_tick = state->current_time_tick;
			Reply(task->tid, (const char *)&reply, sizeof(reply));
			dlist_del(&task->node);

			// Mark the task as free
			int index = task - state->task_pool;
			state->task_pool[index].tid = -1; // Mark as unused
			state->free_tasks_tail = (state->free_tasks_tail + 1) % MAX_DELAYED_TASKS;
			state->free_tasks[state->free_tasks_tail] = index; // Get index in pool
			state->tasks_count--;
		} else {
			break; // No more expired tasks
		}
	}
}

static void process_request(clock_server_state_t *state, int sender_tid, clock_request_t *request)
{
	clock_reply_t reply;

	switch (request->type) {
	case CLOCK_TIME:
		reply.time_tick = state->current_time_tick;
		Reply(sender_tid, (const char *)&reply, sizeof(reply));
		break;

	case CLOCK_DELAY:
		if (request->ticks < 0) {
			reply.time_tick = CLOCK_ERR_NEGATIVE_DELAY;
			Reply(sender_tid, (const char *)&reply, sizeof(reply));
			break;
		} else {
			int wake_time = state->current_time_tick + request->ticks;
			int result = add_delayed_task(state, sender_tid, wake_time);
			if (result < 0) {
				reply.time_tick = result; // Error code
				Reply(sender_tid, (const char *)&reply, sizeof(reply));
			}
		}
		break;

	case CLOCK_DELAY_UNTIL:
		if (request->ticks < 0) {
			reply.time_tick = CLOCK_ERR_NEGATIVE_DELAY;
			Reply(sender_tid, (const char *)&reply, sizeof(reply));
			break;
		} else {
			int wake_time = request->ticks;
			int result = add_delayed_task(state, sender_tid, wake_time);
			if (result < 0) {
				reply.time_tick = result; // Error code
				Reply(sender_tid, (const char *)&reply, sizeof(reply));
			}
		}
		break;

	case CLOCK_TICK_NOTIFY:
		state->current_time_tick++;
		wake_expired_tasks(state);
		Reply(sender_tid, (const char *)&reply, sizeof(reply));
		break;

	default:
		reply.time_tick = CLOCK_ERR_INVALID_TID;
		Reply(sender_tid, (const char *)&reply, sizeof(reply));
		break;
	}
}

void clock_server_main(void)
{
	clock_server_state_t state;
	clock_request_t request;
	int sender_tid;

	state.current_time_tick = 0;
	dlist_init(&state.delay_list);
	init_task_pool(&state);

	RegisterAs(CLOCK_SERVER_NAME);

	Create(CLOCK_SERVER_PRIORITY - 1, clock_notifier_main);

	for (;;) {
		int result = Receive(&sender_tid, (char *)&request, sizeof(request));

		if (result < 0) {
			continue;
		}

		process_request(&state, sender_tid, &request);
	}
}

void clock_notifier_main(void)
{
	clock_request_t notify_msg;
	clock_reply_t reply;

	notify_msg.type = CLOCK_TICK_NOTIFY;
	notify_msg.ticks = 0;

	int clock_server_tid = WhoIs(CLOCK_SERVER_NAME);

	for (;;) {
		int result = AwaitEvent(EVENT_TIMER_TICK);

		if (result < 0) {
			continue;
		}

		Send(clock_server_tid, (const char *)&notify_msg, sizeof(notify_msg), (char *)&reply, sizeof(reply));
	}
}
