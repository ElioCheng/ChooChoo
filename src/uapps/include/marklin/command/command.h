#ifndef MARKLIN_COMMAND_H
#define MARKLIN_COMMAND_H

#include "types.h"
#include "marklin/error.h"
#include "marklin/command/api.h"

// Command priority levels
typedef enum marklin_cmd_priority_enum {
	MARKLIN_CMD_PRIORITY_CRITICAL = 0, // Emergency stops, immediate safety commands
	MARKLIN_CMD_PRIORITY_HIGH = 1, // Train movement commands
	MARKLIN_CMD_PRIORITY_MEDIUM = 2, // Switch operations, system control
	MARKLIN_CMD_PRIORITY_LOW = 3 // Sensor polling
} marklin_cmd_priority_t;

#define MARKLIN_CMD_TIMER_NAME "marklin_cmd_timer"

#define MARKLIN_CMD_SERVER_TASK_PRIORITY 4
#define MARKLIN_CMD_TIMER_TASK_PRIORITY (MARKLIN_CMD_SERVER_TASK_PRIORITY - 1)

typedef struct {
	marklin_cmd_type_t cmd_type;
	u8 cmd;
	u8 param;
	i32 gap_ticks;
	marklin_cmd_priority_t priority;
	u8 train_id;
	u64 timestamp;
	int is_blocking;
	int sender_tid;
} marklin_cmd_t;

typedef enum {
	MARKLIN_CMD_REQ_SCHEDULE,
	MARKLIN_CMD_REQ_SCHEDULE_BLOCKING,
	MARKLIN_CMD_REQ_TIMER_READY,
} marklin_cmd_request_type_t;

typedef struct {
	marklin_cmd_request_type_t type;
	union {
		marklin_cmd_t schedule_cmd;
	};
} marklin_cmd_request_t;

typedef struct {
	marklin_error_t error;
	union {
		struct {
			i32 next_delay_ticks;
		} timer;
	};
} marklin_cmd_reply_t;

void marklin_cmd_server_task(void);
void marklin_cmd_timer_task(void);

#endif /* MARKLIN_COMMAND_H */
