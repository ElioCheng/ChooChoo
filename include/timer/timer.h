#ifndef TIMER_H
#define TIMER_H

#include "types.h"
#include "dlist.h"

// Timer callback function type
typedef void (*timer_callback_fn)(void *arg);

#define TIMER_NAME_MAX_LEN 32
// Timer structure
typedef struct timer {
	struct dlist_node node; // List node for timer management
	timer_callback_fn callback; // Callback function to execute
	void *arg; // Argument to pass to callback
	u64 expires; // Expiration time in ticks
	u64 period; // Period for periodic timers (0 for one-shot)
	bool active; // Whether the timer is active
	char name[TIMER_NAME_MAX_LEN]; // Name of the timer
} timer_t;

// Initialize a timer
void timer_init(timer_t *timer, const char *name, timer_callback_fn callback, void *arg);

// Start a one-shot timer that expires after 'ms' milliseconds
void timer_start_once(timer_t *timer, u64 ms);

// Start a periodic timer that expires every 'ms' milliseconds
void timer_start_periodic(timer_t *timer, u64 ms);

// Stop a timer
void timer_stop(timer_t *timer);

// Check if a timer is active
bool timer_is_active(timer_t *timer);

// Process expired timers
void timer_process(void);

// Initialize the timer subsystem
void timer_subsystem_init(void);

#endif /* TIMER_H */
