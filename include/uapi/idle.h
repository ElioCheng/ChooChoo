#ifndef __UAPI_IDLE_H__
#define __UAPI_IDLE_H__

#include "types.h"
#include "compiler.h"

#define IDLE_STATS_WINDOW_MS 1000  // 1 second

typedef struct {
	u64 last_idle_start_time;
	u64 last_measurement_time;
	u64 idle_time_in_window;
	u64 measurement_window_us;
	u32 idle_percentage;
	bool is_measuring;
	bool display_enabled;
} idle_stats_t;

#endif /* __UAPI_IDLE_H__ */
