#include "event.h"
#include "sched.h"
#include "klog.h"

void event_unblock_waiting_tasks(int event_id, int event_data)
{
	klog_debug("Event %d occurred with data %d", event_id, event_data);

	if (!is_valid_event_id(event_id)) {
		klog_error("Invalid event ID %d", event_id);
		return;
	}

	sched_unblock_event_tasks(event_id, event_data);
}

char *event_id_to_string(int event_id)
{
	switch (event_id) {
	case EVENT_TIMER_TICK:
		return "TIMER_TICK";
	case EVENT_UART_RX:
		return "UART_RX";
	case EVENT_UART_TX:
		return "UART_TX";
	case EVENT_UART_MS:
		return "UART_MS";
	}
	return "UNKNOWN";
}
