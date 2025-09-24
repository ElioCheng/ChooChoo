#include "clock.h"
#include "syscall.h"
#include "printf.h"

int Time(int tid)
{
	clock_request_t request;
	clock_reply_t reply;

	request.type = CLOCK_TIME;
	request.ticks = 0; // Not used for TIME requests

	int result = Send(tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return CLOCK_ERR_INVALID_TID;
	}

	return reply.time_tick;
}

int Delay(int tid, int ticks)
{
	if (ticks < 0) {
		return CLOCK_ERR_NEGATIVE_DELAY;
	}

	clock_request_t request;
	clock_reply_t reply;

	request.type = CLOCK_DELAY;
	request.ticks = ticks;

	int result = Send(tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return CLOCK_ERR_INVALID_TID;
	}

	return reply.time_tick;
}

int DelayUntil(int tid, int ticks)
{
	if (ticks < 0) {
		return CLOCK_ERR_NEGATIVE_DELAY;
	}

	clock_request_t request;
	clock_reply_t reply;

	request.type = CLOCK_DELAY_UNTIL;
	request.ticks = ticks;

	int result = Send(tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return CLOCK_ERR_INVALID_TID;
	}

	return reply.time_tick;
}

int time_format_time(char *buf, u64 tick, u32 style)
{
	u64 seconds = TICK_TO_S(tick);
	u64 milliseconds = TICK_TO_MS(tick);
	u64 minutes = seconds / 60;
	u64 hours = minutes / 60;

	int ret = 0;
	switch (style) {
	case TIME_STYLE_HHMMSSMS:
		seconds = seconds % 60;
		minutes = minutes % 60;
		milliseconds = milliseconds % 1000;
		ret = snprintf(buf, 20, "%02d:%02d:%02d.%03d", (int)hours, (int)minutes, (int)seconds,
			       (int)milliseconds);
		break;
	case TIME_STYLE_SSMS:
	default:
		milliseconds = milliseconds % 1000;
		ret = snprintf(buf, 20, "%5d.%03d", (int)seconds, (int)milliseconds);
		break;
	}

	return ret;
}
