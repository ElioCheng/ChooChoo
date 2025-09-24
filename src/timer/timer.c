#include "panic.h"
#include "timer/timer.h"
#include "timer/time.h"
#include "printf.h"
#include "klog.h"
// TODO: REPLACE THE LIST WITH A BST
// The list is sorted by the expiration time
static DLIST_HEAD(active_timers);

void timer_subsystem_init(void)
{
	dlist_init(&active_timers);
}

void timer_init(timer_t *timer, const char *name, timer_callback_fn callback, void *arg)
{
	dlist_init_node(&timer->node);
	timer->callback = callback;
	timer->arg = arg;
	timer->expires = 0;
	timer->period = 0;
	timer->active = false;
	snprintf(timer->name, sizeof(timer->name), "%s", name);
}

void timer_start_once(timer_t *timer, u64 ms)
{
	// Remove from list if already active
	if (timer->active) {
		dlist_del(&timer->node);
	}

	timer->expires = TIME_GET_TICK_MS() + ms;
	timer->period = 0;
	timer->active = true;

	// Insert into list in order of expiration
	struct dlist_node *pos;
	dlist_for_each(pos, &active_timers)
	{
		timer_t *t = dlist_entry(pos, timer_t, node);
		if (timer->expires < t->expires) {
			dlist_insert(pos->prev, &timer->node);
			return;
		}
	}
	dlist_insert_tail(&active_timers, &timer->node);
}

void timer_start_periodic(timer_t *timer, u64 ms)
{
	// Remove from list if already active
	if (timer->active) {
		dlist_del(&timer->node);
	}

	// Set up timer
	timer->expires = TIME_GET_TICK_MS() + ms;
	timer->period = ms;
	timer->active = true;

	// Insert into list in order of expiration
	struct dlist_node *pos;
	dlist_for_each(pos, &active_timers)
	{
		timer_t *t = dlist_entry(pos, timer_t, node);
		if (timer->expires < t->expires) {
			dlist_insert(pos->prev, &timer->node);
			return;
		}
	}
	dlist_insert_tail(&active_timers, &timer->node);
}

void timer_stop(timer_t *timer)
{
	if (timer->active) {
		dlist_del(&timer->node);
		timer->active = false;
	}
}

bool timer_is_active(timer_t *timer)
{
	return timer->active;
}

#ifdef DEBUG_BUILD
static void _timer_debug_check_list(void)
{
	struct dlist_node *current, *next;
	// Walk through the list and check if it is sorted by the expiration time
	for (current = active_timers.next; current != &active_timers; current = next) {
		next = current->next;
		if (next == &active_timers) {
			break;
		}
		timer_t *current_timer = dlist_entry(current, timer_t, node);
		timer_t *next_timer = dlist_entry(next, timer_t, node);
		if (current_timer->expires > next_timer->expires) {
			DLIST_PRINT(&active_timers, timer_t, node, "active_timers: %d", expires, 10);
			panic("Timer list is not sorted: %s[%d] > %s[%d]", current_timer->name, current_timer->expires,
			      next_timer->name, next_timer->expires);
		}
	}
}
#else
void _timer_debug_check_list(void)
{
}
#endif

void timer_process(void)
{
	u64 current_time = TIME_GET_TICK_MS();
	struct dlist_node *pos, *n;

	_timer_debug_check_list();

	dlist_for_each_safe(pos, n, &active_timers)
	{
		timer_t *timer = dlist_entry(pos, timer_t, node);

		if (current_time >= timer->expires) {
			// Execute callback
			if (timer->callback) {
				timer->callback(timer->arg);
			} else {
				klog_warning("Timer %s has no callback", timer->name);
			}

			if (timer->period > 0) {
				// Reschedule periodic timer
				timer->expires = current_time + timer->period;

				// Remove and reinsert to maintain sorted order
				dlist_del(&timer->node);
				struct dlist_node *new_pos;
				bool inserted = false;
				dlist_for_each(new_pos, &active_timers)
				{
					timer_t *t = dlist_entry(new_pos, timer_t, node);
					if (timer->expires < t->expires) {
						dlist_insert(new_pos->prev, &timer->node);
						inserted = true;
						break;
					}
				}
				if (!inserted) {
					dlist_insert_tail(&active_timers, &timer->node);
				}
			} else {
				// One-shot timer, remove it
				dlist_del(&timer->node);
				timer->active = false;
			}
		} else {
			// Stop processing timers
			break;
		}
	}
}
