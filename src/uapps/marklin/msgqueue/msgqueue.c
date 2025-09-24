#include "marklin/msgqueue/msgqueue.h"
#include "marklin/msgqueue/api.h"
#include "name.h"
#include "syscall.h"
#include "string.h"
#include "compiler.h"
#include "clock.h"

#define LOG_MODULE "msgqueue"
#define LOG_LEVEL LOG_LEVEL_ERROR
#include "log.h"

// #########################################################
// # Message Queue Client API Implementation
// #########################################################

int msgqueue_server_tid = -1;

static inline int get_msgqueue_server_tid(void)
{
	if (msgqueue_server_tid < 0) {
		msgqueue_server_tid = WhoIs(MARKLIN_MSGQUEUE_SERVER_NAME);
	}
	return msgqueue_server_tid;
}

marklin_error_t Marklin_MsgQueue_Publish(marklin_msgqueue_event_type_t event_type, const void *data, u32 data_size)
{
	int server_tid = get_msgqueue_server_tid();
	if (server_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	if (!data || data_size == 0 || data_size > MARKLIN_MSGQUEUE_MAX_DATA_SIZE) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_msgqueue_request_t request;
	marklin_msgqueue_reply_t reply;

	request.type = MARKLIN_MSGQUEUE_REQ_PUBLISH;
	request.publish.event_type = event_type;
	request.publish.data_size = data_size;

	memcpy(request.publish.data, data, data_size);

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_MsgQueue_Subscribe(marklin_msgqueue_event_type_t event_type,
					   marklin_msgqueue_subscription_t *subscription)
{
	int server_tid = get_msgqueue_server_tid();
	if (server_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	if (!subscription) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_msgqueue_request_t request;
	marklin_msgqueue_reply_t reply;

	request.type = MARKLIN_MSGQUEUE_REQ_SUBSCRIBE;
	request.subscribe.event_type = event_type;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	if (reply.error == MARKLIN_ERROR_OK) {
		subscription->event_type = event_type;
		subscription->subscriber_tid = MyTid();
		subscription->subscription_id = reply.subscribe.subscription_id;
	}

	return reply.error;
}

marklin_error_t Marklin_MsgQueue_Unsubscribe(const marklin_msgqueue_subscription_t *subscription)
{
	int server_tid = get_msgqueue_server_tid();
	if (server_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	if (!subscription) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_msgqueue_request_t request;
	marklin_msgqueue_reply_t reply;

	request.type = MARKLIN_MSGQUEUE_REQ_UNSUBSCRIBE;
	request.unsubscribe.event_type = subscription->event_type;
	request.unsubscribe.subscription_id = subscription->subscription_id;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_MsgQueue_Receive(marklin_msgqueue_message_t *message, u32 timeout_ticks)
{
	int server_tid = get_msgqueue_server_tid();
	if (server_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	if (!message) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_msgqueue_request_t request;
	marklin_msgqueue_reply_t reply;

	request.type = MARKLIN_MSGQUEUE_REQ_RECEIVE;
	request.receive.timeout_ticks = timeout_ticks;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	if (reply.error == MARKLIN_ERROR_OK) {
		*message = reply.receive.message;
	}

	return reply.error;
}

marklin_error_t Marklin_MsgQueue_ReceiveNonBlock(marklin_msgqueue_message_t *message)
{
	int server_tid = get_msgqueue_server_tid();
	if (server_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	if (!message) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_msgqueue_request_t request;
	marklin_msgqueue_reply_t reply;

	request.type = MARKLIN_MSGQUEUE_REQ_RECEIVE_NONBLOCK;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	if (reply.error == MARKLIN_ERROR_OK) {
		*message = reply.receive.message;
	}

	return reply.error;
}

int Marklin_MsgQueue_GetPendingCount(void)
{
	int server_tid = get_msgqueue_server_tid();
	if (server_tid < 0) {
		return -1;
	}

	marklin_msgqueue_request_t request;
	marklin_msgqueue_reply_t reply;

	request.type = MARKLIN_MSGQUEUE_REQ_GET_PENDING_COUNT;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0 || reply.error != MARKLIN_ERROR_OK) {
		return -1;
	}

	return reply.pending_count.pending_count;
}

// #########################################################
// # Message Queue Server Implementation
// #########################################################

typedef struct {
	int tid;
	u32 timeout_ticks;
	u64 request_time;
} marklin_msgqueue_pending_receive_t;

typedef struct {
	marklin_msgqueue_subscriber_info_t subscribers[MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS];
	marklin_msgqueue_pending_receive_t pending_receives[MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS];
	int pending_receive_count;
	u32 next_subscription_id;
	u32 next_sequence_number;
} marklin_msgqueue_server_state_t;

static marklin_msgqueue_server_state_t *server_state_g = NULL;

static void server_state_init(marklin_msgqueue_server_state_t *server_state)
{
	memset(server_state, 0, sizeof(*server_state));

	server_state->next_subscription_id = 1;
	server_state->next_sequence_number = 1;
	server_state->pending_receive_count = 0;

	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		server_state->subscribers[i].is_active = 0;
		server_state->subscribers[i].pending_messages = 0;
		server_state->subscribers[i].queue_head = 0;
		server_state->subscribers[i].queue_tail = 0;
		server_state->subscribers[i].tid = -1;
		server_state->subscribers[i].event_type = 0xFF; // Invalid event type as canary
		server_state->subscribers[i].subscription_id = 0;

		server_state->pending_receives[i].tid = -1;
		server_state->pending_receives[i].timeout_ticks = 0;
		server_state->pending_receives[i].request_time = 0;
	}
}

static marklin_msgqueue_subscriber_info_t *find_subscriber_by_tid_and_event(int tid,
									    marklin_msgqueue_event_type_t event_type)
{
	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		if (server_state_g->subscribers[i].is_active && server_state_g->subscribers[i].tid == tid &&
		    server_state_g->subscribers[i].event_type == event_type) {
			return &server_state_g->subscribers[i];
		}
	}
	return NULL;
}

static marklin_msgqueue_subscriber_info_t *
find_subscriber_by_subscription_id(int tid, marklin_msgqueue_event_type_t event_type, u32 subscription_id)
{
	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		if (server_state_g->subscribers[i].is_active && server_state_g->subscribers[i].tid == tid &&
		    server_state_g->subscribers[i].event_type == event_type &&
		    server_state_g->subscribers[i].subscription_id == subscription_id) {
			return &server_state_g->subscribers[i];
		}
	}
	return NULL;
}

static marklin_msgqueue_subscriber_info_t *find_free_subscriber_slot(void)
{
	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		if (!server_state_g->subscribers[i].is_active) {
			return &server_state_g->subscribers[i];
		}
	}
	return NULL;
}

static int subscriber_queue_is_full(marklin_msgqueue_subscriber_info_t *subscriber)
{
	return subscriber->pending_messages >= MARKLIN_MSGQUEUE_MAX_MESSAGES_PER_SUBSCRIBER;
}

static marklin_error_t subscriber_queue_enqueue(marklin_msgqueue_subscriber_info_t *subscriber,
						const marklin_msgqueue_message_t *message)
{
	if (subscriber_queue_is_full(subscriber)) {
		return MARKLIN_ERROR_QUEUE_FULL;
	}

	marklin_msgqueue_internal_message_t *internal_msg = &subscriber->message_queue[subscriber->queue_tail];

	internal_msg->message = *message;
	internal_msg->sequence_number = server_state_g->next_sequence_number++;

	subscriber->queue_tail = (subscriber->queue_tail + 1) % MARKLIN_MSGQUEUE_MAX_MESSAGES_PER_SUBSCRIBER;
	subscriber->pending_messages++;

	return MARKLIN_ERROR_OK;
}

static marklin_error_t subscriber_queue_dequeue(marklin_msgqueue_subscriber_info_t *subscriber,
						marklin_msgqueue_message_t *message)
{
	if (subscriber->pending_messages == 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_msgqueue_internal_message_t *internal_msg = &subscriber->message_queue[subscriber->queue_head];

	*message = internal_msg->message;

	subscriber->queue_head = (subscriber->queue_head + 1) % MARKLIN_MSGQUEUE_MAX_MESSAGES_PER_SUBSCRIBER;
	subscriber->pending_messages--;

	return MARKLIN_ERROR_OK;
}

static marklin_error_t add_pending_receive(int tid, u32 timeout_ticks, u64 request_time)
{
	if (server_state_g->pending_receive_count >= MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS) {
		return MARKLIN_ERROR_QUEUE_FULL;
	}

	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		if (server_state_g->pending_receives[i].tid == -1) {
			server_state_g->pending_receives[i].tid = tid;
			server_state_g->pending_receives[i].timeout_ticks = timeout_ticks;
			server_state_g->pending_receives[i].request_time = request_time;
			server_state_g->pending_receive_count++;
			return MARKLIN_ERROR_OK;
		}
	}

	return MARKLIN_ERROR_QUEUE_FULL;
}

static void remove_pending_receive(int tid)
{
	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		if (server_state_g->pending_receives[i].tid == tid) {
			server_state_g->pending_receives[i].tid = -1;
			server_state_g->pending_receives[i].timeout_ticks = 0;
			server_state_g->pending_receives[i].request_time = 0;
			server_state_g->pending_receive_count--;
			break;
		}
	}
}

static marklin_error_t check_and_reply_pending_receives(void)
{
	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		if (server_state_g->pending_receives[i].tid == -1) {
			continue;
		}

		int tid = server_state_g->pending_receives[i].tid;

		marklin_msgqueue_subscriber_info_t *subscriber_with_messages = NULL;
		for (int j = 0; j < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; j++) {
			marklin_msgqueue_subscriber_info_t *subscriber = &server_state_g->subscribers[j];
			if (subscriber->is_active && subscriber->tid == tid && subscriber->pending_messages > 0) {
				subscriber_with_messages = subscriber;
				break;
			}
		}

		if (subscriber_with_messages) {
			marklin_msgqueue_reply_t reply;
			reply.error = subscriber_queue_dequeue(subscriber_with_messages, &reply.receive.message);

			Reply(tid, (const char *)&reply, sizeof(reply));
			remove_pending_receive(tid);
		}
	}

	return MARKLIN_ERROR_OK;
}

static marklin_error_t handle_publish_request(const marklin_msgqueue_request_t *request)
{
	marklin_msgqueue_message_t message;
	message.event_type = request->publish.event_type;
	message.data_size = request->publish.data_size;

	memcpy(message.data, request->publish.data, request->publish.data_size);

	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		marklin_msgqueue_subscriber_info_t *subscriber = &server_state_g->subscribers[i];

		if (subscriber->is_active) {
			if (subscriber->event_type == request->publish.event_type) {
				subscriber_queue_enqueue(subscriber, &message);
			}
		}
	}

	check_and_reply_pending_receives();

	return MARKLIN_ERROR_OK;
}

static marklin_error_t handle_subscribe_request(const marklin_msgqueue_request_t *request, int subscriber_tid,
						marklin_msgqueue_reply_t *reply)
{
	marklin_msgqueue_subscriber_info_t *existing =
		find_subscriber_by_tid_and_event(subscriber_tid, request->subscribe.event_type);

	if (existing) {
		reply->subscribe.subscription_id = existing->subscription_id;
		return MARKLIN_ERROR_OK;
	}

	marklin_msgqueue_subscriber_info_t *subscriber = find_free_subscriber_slot();
	if (!subscriber) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	subscriber->tid = subscriber_tid;
	subscriber->event_type = request->subscribe.event_type;
	subscriber->subscription_id = server_state_g->next_subscription_id++;
	subscriber->is_active = 1;
	subscriber->pending_messages = 0;
	subscriber->queue_head = 0;
	subscriber->queue_tail = 0;

	reply->subscribe.subscription_id = subscriber->subscription_id;

	return MARKLIN_ERROR_OK;
}

static marklin_error_t handle_unsubscribe_request(const marklin_msgqueue_request_t *request, int subscriber_tid)
{
	marklin_msgqueue_subscriber_info_t *subscriber = find_subscriber_by_subscription_id(
		subscriber_tid, request->unsubscribe.event_type, request->unsubscribe.subscription_id);

	if (!subscriber) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	subscriber->is_active = 0;
	return MARKLIN_ERROR_OK;
}

static marklin_error_t handle_receive_request(const marklin_msgqueue_request_t *request, int subscriber_tid,
					      marklin_msgqueue_reply_t *reply)
{
	marklin_msgqueue_subscriber_info_t *subscriber_with_messages = NULL;

	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		marklin_msgqueue_subscriber_info_t *subscriber = &server_state_g->subscribers[i];

		if (subscriber->is_active && subscriber->tid == subscriber_tid && subscriber->pending_messages > 0) {
			subscriber_with_messages = subscriber;
			break;
		}
	}

	if (subscriber_with_messages) {
		return subscriber_queue_dequeue(subscriber_with_messages, &reply->receive.message);
	}

	// No messages available - add to pending receive list for blocking behavior
	u64 current_time = Time(0);
	marklin_error_t result = add_pending_receive(subscriber_tid, request->receive.timeout_ticks, current_time);

	if (result != MARKLIN_ERROR_OK) {
		return result;
	}

	return MARKLIN_ERROR_PENDING;
}

static marklin_error_t handle_receive_nonblock_request(const marklin_msgqueue_request_t *request
						       __attribute__((unused)),
						       int subscriber_tid, marklin_msgqueue_reply_t *reply)
{
	marklin_msgqueue_subscriber_info_t *subscriber_with_messages = NULL;

	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		marklin_msgqueue_subscriber_info_t *subscriber = &server_state_g->subscribers[i];

		if (subscriber->is_active && subscriber->tid == subscriber_tid && subscriber->pending_messages > 0) {
			subscriber_with_messages = subscriber;
			break;
		}
	}

	// klog_info("MsgQueue: Checking subscriber %d for messages", subscriber_tid);

	if (!subscriber_with_messages) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	return subscriber_queue_dequeue(subscriber_with_messages, &reply->receive.message);
}

static marklin_error_t handle_get_pending_count_request(int subscriber_tid, marklin_msgqueue_reply_t *reply)
{
	int total_pending = 0;

	for (int i = 0; i < MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS; i++) {
		marklin_msgqueue_subscriber_info_t *subscriber = &server_state_g->subscribers[i];

		if (subscriber->is_active && subscriber->tid == subscriber_tid) {
			total_pending += subscriber->pending_messages;
		}
	}

	reply->pending_count.pending_count = total_pending;
	return MARKLIN_ERROR_OK;
}

void __noreturn marklin_msgqueue_server_task(void)
{
	int sender_tid;
	marklin_msgqueue_request_t request;
	marklin_msgqueue_reply_t reply;
	marklin_msgqueue_server_state_t server_state;

	server_state_init(&server_state);
	server_state_g = &server_state;

	if (RegisterAs(MARKLIN_MSGQUEUE_SERVER_NAME) < 0) {
		log_error("MsgQueue: Failed to register server name");
		Exit();
	}

	log_info("MsgQueue: Server started");

	for (;;) {
		int result = Receive(&sender_tid, (char *)&request, sizeof(request));

		if (result < 0) {
			continue;
		}

		reply.error = MARKLIN_ERROR_OK;

		switch (request.type) {
		case MARKLIN_MSGQUEUE_REQ_PUBLISH:
			reply.error = handle_publish_request(&request);
			break;

		case MARKLIN_MSGQUEUE_REQ_SUBSCRIBE:
			reply.error = handle_subscribe_request(&request, sender_tid, &reply);
			break;

		case MARKLIN_MSGQUEUE_REQ_UNSUBSCRIBE:
			reply.error = handle_unsubscribe_request(&request, sender_tid);
			break;

		case MARKLIN_MSGQUEUE_REQ_RECEIVE:
			reply.error = handle_receive_request(&request, sender_tid, &reply);
			break;

		case MARKLIN_MSGQUEUE_REQ_RECEIVE_NONBLOCK:
			reply.error = handle_receive_nonblock_request(&request, sender_tid, &reply);
			break;

		case MARKLIN_MSGQUEUE_REQ_GET_PENDING_COUNT:
			reply.error = handle_get_pending_count_request(sender_tid, &reply);
			break;

		default:
			reply.error = MARKLIN_ERROR_INVALID_ARGUMENT;
			break;
		}

		// Only reply if the request is not pending (blocking)
		if (reply.error != MARKLIN_ERROR_PENDING) {
			Reply(sender_tid, (const char *)&reply, sizeof(reply));
		}
	}

	UNREACHABLE();
}
