#ifndef __EVENT_H__
#define __EVENT_H__

#include "types.h"

// Event IDs for AwaitEvent syscall
#define EVENT_TIMER_TICK 1 // Timer interrupt event
#define EVENT_UART_RX 2 // UART receive interrupt
#define EVENT_UART_TX 3 // UART transmit interrupt
#define EVENT_UART_MS 4 // UART modem status interrupt (CTS changes)
#define EVENT_MAX 4 // Maximum valid event ID

// Return values
#define EVENT_DATA_NONE 0
#define EVENT_ERROR -1

#ifdef __KERNEL__
static inline bool is_valid_event_id(int event_id)
{
	return event_id >= 1 && event_id <= EVENT_MAX;
}

void event_unblock_waiting_tasks(int event_id, int event_data);

char *event_id_to_string(int event_id);
#endif // __KERNEL__

#endif // __EVENT_H__
