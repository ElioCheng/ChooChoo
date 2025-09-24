#ifndef MARKLIN_MSGQUEUE_H
#define MARKLIN_MSGQUEUE_H

#include "types.h"
#include "marklin/error.h"
#include "marklin/msgqueue/api.h"

#define MARKLIN_MSGQUEUE_SERVER_TASK_PRIORITY 4
#define MARKLIN_MSGQUEUE_MAX_MESSAGES_PER_SUBSCRIBER 128

typedef enum {
	MARKLIN_MSGQUEUE_REQ_PUBLISH,
	MARKLIN_MSGQUEUE_REQ_SUBSCRIBE,
	MARKLIN_MSGQUEUE_REQ_UNSUBSCRIBE,
	MARKLIN_MSGQUEUE_REQ_RECEIVE,
	MARKLIN_MSGQUEUE_REQ_RECEIVE_NONBLOCK,
	MARKLIN_MSGQUEUE_REQ_GET_PENDING_COUNT,
} marklin_msgqueue_request_type_t;

typedef struct {
	marklin_msgqueue_message_t message;
	u32 sequence_number;
} marklin_msgqueue_internal_message_t;

typedef struct {
	int tid;
	marklin_msgqueue_event_type_t event_type;
	u32 subscription_id;
	int is_active;
	int pending_messages;
	marklin_msgqueue_internal_message_t message_queue[MARKLIN_MSGQUEUE_MAX_MESSAGES_PER_SUBSCRIBER];
	int queue_head;
	int queue_tail;
} marklin_msgqueue_subscriber_info_t;

typedef struct {
	marklin_msgqueue_request_type_t type;
	union {
		struct {
			marklin_msgqueue_event_type_t event_type;
			u32 data_size;
			u8 data[MARKLIN_MSGQUEUE_MAX_DATA_SIZE];
		} publish;
		struct {
			marklin_msgqueue_event_type_t event_type;
		} subscribe;
		struct {
			marklin_msgqueue_event_type_t event_type;
			u32 subscription_id;
		} unsubscribe;
		struct {
			u32 timeout_ticks;
		} receive;
	};
} marklin_msgqueue_request_t;

typedef struct {
	marklin_error_t error;
	union {
		struct {
			u32 subscription_id;
		} subscribe;
		struct {
			marklin_msgqueue_message_t message;
		} receive;
		struct {
			int pending_count;
		} pending_count;
	};
} marklin_msgqueue_reply_t;

void marklin_msgqueue_server_task(void);

#endif /* MARKLIN_MSGQUEUE_H */
