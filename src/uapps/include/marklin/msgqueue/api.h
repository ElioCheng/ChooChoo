#ifndef MARKLIN_MSGQUEUE_API_H
#define MARKLIN_MSGQUEUE_API_H

#include "marklin/error.h"
#include "types.h"

#define MARKLIN_MSGQUEUE_SERVER_NAME "marklin_msgqueue_server"
#define MARKLIN_MSGQUEUE_MAX_SUBSCRIBERS 32
#define MARKLIN_MSGQUEUE_MAX_DATA_SIZE (4096 - sizeof(marklin_msgqueue_event_type_t) - sizeof(u32))

typedef enum {
	MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE,
	MARKLIN_MSGQUEUE_EVENT_TYPE_TRAIN_POSITION,
	MARKLIN_MSGQUEUE_EVENT_TYPE_SWITCH_STATE,
	MARKLIN_MSGQUEUE_EVENT_TYPE_BLOCK_RESERVATION,
} marklin_msgqueue_event_type_t;

typedef struct {
	marklin_msgqueue_event_type_t event_type;
	u32 data_size;
	u8 data[MARKLIN_MSGQUEUE_MAX_DATA_SIZE];
} marklin_msgqueue_message_t;

typedef struct {
	marklin_msgqueue_event_type_t event_type;
	int subscriber_tid;
	u32 subscription_id;
} marklin_msgqueue_subscription_t;

#define MARKLIN_MSGQUEUE_CAST_TO(type, msg) (((msg)->data_size == sizeof(type)) ? (type *)(msg)->data : NULL)

#define MARKLIN_MSGQUEUE_CAST_FROM(type, msg, data_ptr)                   \
	do {                                                              \
		(msg)->data_size = sizeof(type);                          \
		if ((msg)->data_size <= MARKLIN_MSGQUEUE_MAX_DATA_SIZE) { \
			*((type *)(msg)->data) = *(data_ptr);             \
		}                                                         \
	} while (0)

// Publisher API

/**
 * Publish a message to all subscribers of the given event type
 * @param event_type The type of event to publish
 * @param data Pointer to the data to publish
 * @param data_size Size of the data in bytes
 * @return MARKLIN_ERROR_OK on success, error code otherwise
 */
marklin_error_t Marklin_MsgQueue_Publish(marklin_msgqueue_event_type_t event_type, const void *data, u32 data_size);

/**
 * Publish a typed message (convenience macro)
 * @param event_type The type of event to publish
 * @param data_ptr Pointer to the typed data
 */
#define Marklin_MsgQueue_PublishTyped(event_type, data_ptr) \
	Marklin_MsgQueue_Publish(event_type, data_ptr, sizeof(*(data_ptr)))

// Subscriber API

/**
 * Subscribe to messages of a specific event type
 * @param event_type The type of event to subscribe to
 * @param subscription Output parameter for the subscription handle
 * @return MARKLIN_ERROR_OK on success, error code otherwise
 */
marklin_error_t Marklin_MsgQueue_Subscribe(marklin_msgqueue_event_type_t event_type,
					   marklin_msgqueue_subscription_t *subscription);

/**
 * Unsubscribe from messages of a specific event type
 * @param subscription The subscription handle to cancel
 * @return MARKLIN_ERROR_OK on success, error code otherwise
 */
marklin_error_t Marklin_MsgQueue_Unsubscribe(const marklin_msgqueue_subscription_t *subscription);

/**
 * Wait for and receive a message for subscribed events
 * @param message Output buffer for the received message
 * @param timeout_ticks Maximum time to wait for a message (0 for infinite)
 * @return MARKLIN_ERROR_OK on success, error code otherwise
 */
marklin_error_t Marklin_MsgQueue_Receive(marklin_msgqueue_message_t *message, u32 timeout_ticks);

/**
 * Check for messages without blocking
 * @param message Output buffer for the received message
 * @return MARKLIN_ERROR_OK if message received, MARKLIN_ERROR_NOT_FOUND if no message available
 */
marklin_error_t Marklin_MsgQueue_ReceiveNonBlock(marklin_msgqueue_message_t *message);

/**
 * Get the number of messages waiting in the queue for this subscriber
 * @return Number of pending messages, or -1 on error
 */
int Marklin_MsgQueue_GetPendingCount(void);

#endif /* MARKLIN_MSGQUEUE_API_H */
