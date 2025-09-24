#include "idle.h"
#include "timer/time.h"
#include "klog.h"

task_t *idle_task = NULL;
idle_stats_t *idle_stats = NULL;
bool is_idle_running = false;

static void idle_update_percentage(void);

inline void idle_start_accounting(void)
{
	if (!is_idle_running && likely(idle_stats != NULL)) {
		u64 current_time = TIME_GET_TICK_US();
		idle_stats->last_idle_start_time = current_time;
		is_idle_running = true;
	}
}

inline void idle_stop_accounting(void)
{
	if (is_idle_running && likely(idle_stats != NULL)) {
		u64 current_time = TIME_GET_TICK_US();
		u64 idle_duration = current_time - idle_stats->last_idle_start_time;

		idle_stats->idle_time_in_window += idle_duration;

		is_idle_running = false;

		idle_update_percentage();
	}
}

static void idle_update_percentage(void)
{
	if (unlikely(idle_stats == NULL))
		return;

	u64 current_time = TIME_GET_TICK_US();

	if (!idle_stats->is_measuring) {
		idle_stats->last_measurement_time = current_time;
		idle_stats->idle_time_in_window = 0;
		idle_stats->is_measuring = true;
		return;
	}

	u64 elapsed_time = current_time - idle_stats->last_measurement_time;

	if (elapsed_time >= idle_stats->measurement_window_us) {
		if (elapsed_time > 0) {
			idle_stats->idle_percentage = (u32)((idle_stats->idle_time_in_window * 100) / elapsed_time);
		} else {
			idle_stats->idle_percentage = 0;
		}

		idle_stats->last_measurement_time = current_time;
		idle_stats->idle_time_in_window = 0;
	}
}

void idle_init_stats(idle_stats_t *stats, u64 window_ms)
{
	if (unlikely(stats == NULL))
		return;

	stats->last_idle_start_time = 0;
	stats->last_measurement_time = 0;
	stats->idle_time_in_window = 0;
	stats->measurement_window_us = window_ms * 1000;
	stats->idle_percentage = 0;
	stats->is_measuring = false;
	stats->display_enabled = true;
}

inline void setup_idle_task(task_t *task, idle_stats_t *stats)
{
	idle_task = task;
	idle_stats = stats;

	if (stats != NULL) {
		idle_init_stats(stats, IDLE_STATS_WINDOW_MS);
	}
}
