#include "marklin/command/command.h"
#include "clock_server.h"
#include "marklin/error.h"
#include "name.h"
#include "syscall.h"
#include "io.h"
#include "klog.h"
#include "priority_queue.h"
#include "marklin/conductor/switch.h"

#define LOG_MODULE "MARKLIN_CMD"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

// #########################################################
// # Command Server Public API
// #########################################################
int cmd_server_tid = -1;

marklin_error_t Marklin_ScheduleCommand(marklin_cmd_type_t type, u8 cmd, u8 param, i32 gap_ticks)
{
	if (cmd_server_tid < 0) {
		cmd_server_tid = WhoIs(MARKLIN_CMD_SERVER_NAME);
		if (cmd_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_cmd_request_t request;
	marklin_cmd_reply_t reply;

	request.type = MARKLIN_CMD_REQ_SCHEDULE;
	request.schedule_cmd.cmd_type = type;
	request.schedule_cmd.cmd = cmd;
	request.schedule_cmd.param = param;
	request.schedule_cmd.gap_ticks = gap_ticks;
	request.schedule_cmd.priority = MARKLIN_CMD_PRIORITY_MEDIUM;
	request.schedule_cmd.train_id = 0;
	request.schedule_cmd.is_blocking = 0;

	int result = Send(cmd_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	return reply.error;
}

marklin_error_t Marklin_ScheduleCommandBlocking(marklin_cmd_type_t type, u8 cmd, u8 param, i32 gap_ticks)
{
	if (cmd == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (cmd_server_tid < 0) {
		cmd_server_tid = WhoIs(MARKLIN_CMD_SERVER_NAME);
		if (cmd_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_cmd_request_t request;
	marklin_cmd_reply_t reply;

	request.type = MARKLIN_CMD_REQ_SCHEDULE_BLOCKING;
	request.schedule_cmd.cmd_type = type;
	request.schedule_cmd.cmd = cmd;
	request.schedule_cmd.param = param;
	request.schedule_cmd.gap_ticks = gap_ticks;
	request.schedule_cmd.priority = MARKLIN_CMD_PRIORITY_MEDIUM; // Default priority
	request.schedule_cmd.train_id = 0; // Unknown train
	request.schedule_cmd.is_blocking = 1;

	int result = Send(cmd_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	return reply.error;
}

marklin_error_t Marklin_ScheduleCommandWithPriority(marklin_cmd_type_t type, u8 cmd, u8 param, i32 gap_ticks,
						    marklin_cmd_priority_t priority, u8 train_id)
{
	if (cmd_server_tid < 0) {
		cmd_server_tid = WhoIs(MARKLIN_CMD_SERVER_NAME);
		if (cmd_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_cmd_request_t request;
	marklin_cmd_reply_t reply;

	request.type = MARKLIN_CMD_REQ_SCHEDULE;
	request.schedule_cmd.cmd_type = type;
	request.schedule_cmd.cmd = cmd;
	request.schedule_cmd.param = param;
	request.schedule_cmd.gap_ticks = gap_ticks;
	request.schedule_cmd.priority = priority;
	request.schedule_cmd.train_id = train_id;
	request.schedule_cmd.is_blocking = 0;

	int result = Send(cmd_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	return reply.error;
}

marklin_error_t Marklin_ScheduleCommandBlockingWithPriority(marklin_cmd_type_t type, u8 cmd, u8 param, i32 gap_ticks,
							    marklin_cmd_priority_t priority, u8 train_id)
{
	if (cmd == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (cmd_server_tid < 0) {
		cmd_server_tid = WhoIs(MARKLIN_CMD_SERVER_NAME);
		if (cmd_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_cmd_request_t request;
	marklin_cmd_reply_t reply;

	request.type = MARKLIN_CMD_REQ_SCHEDULE_BLOCKING;
	request.schedule_cmd.cmd_type = type;
	request.schedule_cmd.cmd = cmd;
	request.schedule_cmd.param = param;
	request.schedule_cmd.gap_ticks = gap_ticks;
	request.schedule_cmd.priority = priority;
	request.schedule_cmd.train_id = train_id;
	request.schedule_cmd.is_blocking = 1;

	int result = Send(cmd_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	return reply.error;
}

marklin_error_t Marklin_ScheduleEmergencyStop(u8 train_id)
{
	// Emergency stop command: speed 0 for the specified train
	return Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, 0, train_id, 0,
						   MARKLIN_CMD_PRIORITY_CRITICAL, train_id);
}

// #########################################################
// # Command Server Task
// #########################################################
#define LOG_MODULE "MARKLIN_CMD"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

#define MAX_QUEUED_COMMANDS 128
#define DEFAULT_GAP_TICKS 1 // Default gap between commands 20ms

// Command comparison function for priority queue (min-heap)
// Returns: <0 if a has higher priority than b, >0 if b has higher priority than a, 0 if equal
static int cmd_compare(const marklin_cmd_t *a, const marklin_cmd_t *b)
{
	// First compare by priority (lower number = higher priority)
	if (a->priority != b->priority) {
		return (a->priority < b->priority) ? -1 : 1;
	}

	// Within same priority, use timestamp for FIFO ordering (earlier timestamp = higher priority)
	if (a->timestamp != b->timestamp) {
		return (a->timestamp < b->timestamp) ? -1 : 1;
	}

	return 0;
}

// Priority queue for commands
PRIORITY_QUEUE_DECLARE(marklin_cmd_pqueue, marklin_cmd_t, MAX_QUEUED_COMMANDS);
static struct marklin_cmd_pqueue cmd_priority_queue;

// Global timestamp counter
static u64 global_timestamp = 0;

static int timer_task_tid = -1;

static void cmd_queue_init(void)
{
	pq_init(&cmd_priority_queue, cmd_compare);
	global_timestamp = 0;
}

static int cmd_queue_is_empty(void)
{
	return pq_is_empty(&cmd_priority_queue);
}

static int cmd_queue_is_full(void)
{
	return pq_is_full(&cmd_priority_queue);
}

static marklin_error_t cmd_queue_enqueue(const marklin_cmd_t *cmd)
{
	if (cmd_queue_is_full()) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	// Check for MARKLIN_CMD_SOLENOID_OFF deduplication
	if (cmd->cmd_type == MARKLIN_CMD_TYPE_SINGLE && cmd->cmd == MARKLIN_CMD_SOLENOID_OFF) {
		for (int i = 0; i < cmd_priority_queue.size; i++) {
			marklin_cmd_t *queued_cmd = cmd_priority_queue.items[i];
			if (queued_cmd->cmd_type == MARKLIN_CMD_TYPE_SINGLE &&
			    queued_cmd->cmd == MARKLIN_CMD_SOLENOID_OFF) {
				return MARKLIN_ERROR_OK;
			}
		}
	}

	// Create a copy of the command with timestamp
	marklin_cmd_t cmd_copy = *cmd;
	cmd_copy.timestamp = ++global_timestamp;

	// Allocate space for the command (priority queue expects pointers)
	static marklin_cmd_t cmd_storage[MAX_QUEUED_COMMANDS];
	static int storage_index = 0;

	// Simple circular allocation for command storage
	marklin_cmd_t *stored_cmd = &cmd_storage[storage_index];
	storage_index = (storage_index + 1) % MAX_QUEUED_COMMANDS;

	*stored_cmd = cmd_copy;

	if (!pq_push(&cmd_priority_queue, stored_cmd)) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	return MARKLIN_ERROR_OK;
}

static marklin_error_t cmd_queue_dequeue(marklin_cmd_t *cmd)
{
	if (cmd_queue_is_empty()) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_cmd_t *queued_cmd = pq_pop(&cmd_priority_queue);
	if (!queued_cmd) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	*cmd = *queued_cmd;
	return MARKLIN_ERROR_OK;
}

static void marklin_send_command_to_uart(const marklin_cmd_t *cmd)
{
	marklin_putc(cmd->cmd);
	if (cmd->cmd_type == MARKLIN_CMD_TYPE_WITH_PARAM) {
		marklin_putc(cmd->param);
	}

	// If this was a blocking command, reply to the original sender
	if (cmd->is_blocking && cmd->sender_tid >= 0) {
		marklin_cmd_reply_t reply;
		reply.error = MARKLIN_ERROR_OK;
		Reply(cmd->sender_tid, (const char *)&reply, sizeof(reply));
	}
}

void __noreturn marklin_cmd_server_task(void)
{
	int sender_tid;
	marklin_cmd_request_t request;
	marklin_cmd_reply_t reply;
	marklin_cmd_t current_cmd;

	cmd_queue_init();

	RegisterAs(MARKLIN_CMD_SERVER_NAME);

	cmd_server_tid = MyTid();
	timer_task_tid = Create(MARKLIN_CMD_TIMER_TASK_PRIORITY, marklin_cmd_timer_task);

	klog_info("Command server task started");

	for (;;) {
		int result = Receive(&sender_tid, (char *)&request, sizeof(request));

		if (result < 0) {
			continue;
		}

		reply.error = MARKLIN_ERROR_OK;

		switch (request.type) {
		case MARKLIN_CMD_REQ_SCHEDULE:
			reply.error = cmd_queue_enqueue(&request.schedule_cmd);
			Reply(sender_tid, (const char *)&reply, sizeof(reply));
			break;

		case MARKLIN_CMD_REQ_SCHEDULE_BLOCKING:
			request.schedule_cmd.sender_tid = sender_tid;
			reply.error = cmd_queue_enqueue(&request.schedule_cmd);

			if (reply.error != MARKLIN_ERROR_OK) {
				Reply(sender_tid, (const char *)&reply, sizeof(reply));
			}
			break;

		case MARKLIN_CMD_REQ_TIMER_READY:
			if (!cmd_queue_is_empty()) {
				marklin_error_t queue_result = cmd_queue_dequeue(&current_cmd);

				if (queue_result == MARKLIN_ERROR_OK) {
					marklin_send_command_to_uart(&current_cmd);
					if (current_cmd.gap_ticks <= 0) {
						reply.timer.next_delay_ticks = DEFAULT_GAP_TICKS;
					} else {
						reply.timer.next_delay_ticks = current_cmd.gap_ticks;
					}
				} else {
					reply.timer.next_delay_ticks = DEFAULT_GAP_TICKS;
				}
			} else {
				reply.timer.next_delay_ticks = DEFAULT_GAP_TICKS;
			}

			Reply(sender_tid, (const char *)&reply, sizeof(reply));
			break;

		default:
			reply.error = MARKLIN_ERROR_INVALID_ARGUMENT;
			Reply(sender_tid, (const char *)&reply, sizeof(reply));
			break;
		}
	}

	UNREACHABLE();
}

void __noreturn marklin_cmd_timer_task(void)
{
	marklin_cmd_request_t request;
	marklin_cmd_reply_t reply = { 0 };
	i32 next_delay = DEFAULT_GAP_TICKS;
	int clock_tid;

	RegisterAs(MARKLIN_CMD_TIMER_NAME);

	cmd_server_tid = WhoIs(MARKLIN_CMD_SERVER_NAME);

	clock_tid = WhoIs(CLOCK_SERVER_NAME);

	request.type = MARKLIN_CMD_REQ_TIMER_READY;

	for (;;) {
		int result =
			Send(cmd_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));
		if (result >= 0 && reply.error == MARKLIN_ERROR_OK) {
			next_delay = reply.timer.next_delay_ticks;
		} else {
			next_delay = DEFAULT_GAP_TICKS;
		}
		Delay(clock_tid, next_delay);
	}

	UNREACHABLE();
}
