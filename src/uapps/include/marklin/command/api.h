#ifndef MARKLIN_COMMAND_API_H
#define MARKLIN_COMMAND_API_H

#include "marklin/error.h"
#define MARKLIN_CMD_SERVER_NAME "marklin_cmd_server"
typedef enum {
	MARKLIN_CMD_TYPE_SINGLE,
	MARKLIN_CMD_TYPE_WITH_PARAM,
} marklin_cmd_type_t;

typedef enum marklin_cmd_priority_enum marklin_cmd_priority_t;

marklin_error_t Marklin_ScheduleCommand(marklin_cmd_type_t type, u8 cmd, u8 param, i32 gap_ticks);
marklin_error_t Marklin_ScheduleCommandBlocking(marklin_cmd_type_t type, u8 cmd, u8 param, i32 gap_ticks);

// Priority-aware command scheduling
marklin_error_t Marklin_ScheduleCommandWithPriority(marklin_cmd_type_t type, u8 cmd, u8 param, i32 gap_ticks,
						    marklin_cmd_priority_t priority, u8 train_id);
marklin_error_t Marklin_ScheduleCommandBlockingWithPriority(marklin_cmd_type_t type, u8 cmd, u8 param, i32 gap_ticks,
							    marklin_cmd_priority_t priority, u8 train_id);

// Convenience function for emergency stops
marklin_error_t Marklin_ScheduleEmergencyStop(u8 train_id);

#endif /* MARKLIN_COMMAND_API_H */
