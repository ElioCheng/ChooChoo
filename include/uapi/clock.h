#ifndef __UAPI_CLOCK_H__
#define __UAPI_CLOCK_H__

#include "types.h"
#define CLOCK_SERVER_NAME "clock_server"

typedef enum {
	CLOCK_TIME = 1, // Request current time
	CLOCK_DELAY, // Request relative delay
	CLOCK_DELAY_UNTIL, // Request absolute delay
	CLOCK_TICK_NOTIFY // Notifier tick message
} clock_msg_type_t;

typedef struct {
	clock_msg_type_t type;
	int ticks;
} clock_request_t;

typedef struct {
	int time_tick;
} clock_reply_t;

#define CLOCK_SUCCESS 0
#define CLOCK_ERR_INVALID_TID -1
#define CLOCK_ERR_NEGATIVE_DELAY -2

#define MS_PER_TICK 10
#define TICK_PER_S 100
#define MS_TO_TICK(ms) ((ms) / MS_PER_TICK)
#define TICK_TO_MS(tick) ((tick) * MS_PER_TICK)
#define TICK_TO_S(tick) ((tick) / TICK_PER_S)

int Time(int tid);

int Delay(int tid, int ticks);

int DelayUntil(int tid, int ticks);

#define TIME_STYLE_HHMMSSMS 0
#define TIME_STYLE_SSMS 1

int time_format_time(char *buf, u64 tick, u32 style);

#endif /* __UAPI_CLOCK_H__ */
