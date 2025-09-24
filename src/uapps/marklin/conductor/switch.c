#include "marklin/conductor/switch.h"
#include "marklin/conductor/conductor.h"
#include "marklin/error.h"
#include "marklin/msgqueue/api.h"
#include "marklin/command/api.h"
#include "marklin/command/command.h"
#include "clock.h"
#include "name.h"
#include "compiler.h"
#include "syscall.h"

#define LOG_MODULE "switch"
#define LOG_LEVEL LOG_LEVEL_ERROR
#include "log.h"
#include "klog.h"

// ############################################################################
// # Global Data
// ############################################################################

extern conductor_task_data_t *g_conductor_data;

// ############################################################################
// # Private Helper Functions
// ############################################################################
static void switch_publish_update(switch_lookup_entry_t *entry)
{
	marklin_switch_state_t update_data = { .switch_id = entry->state.switch_id,
					       .direction = entry->state.direction,
					       .last_changed_tick = entry->state.last_changed_tick };

	Marklin_MsgQueue_PublishTyped(MARKLIN_MSGQUEUE_EVENT_TYPE_SWITCH_STATE, &update_data);
}

// ############################################################################
// # Public API Implementation
// ############################################################################

void conductor_consume_switch_update(u8 switch_id, track_direction direction, u32 tick)
{
	if (!g_conductor_data) {
		return;
	}

	switch_lookup_entry_t *entry = conductor_get_switch_lookup_entry(switch_id);
	if (!entry) {
		Panic("Switch: Get Empty lookup entry for switch %d", switch_id);
		return;
	}

	entry->state.direction = direction;
	entry->state.last_changed_tick = tick;
	switch_publish_update(entry);
}

marklin_error_t switch_set_direction(u8 switch_id, track_direction direction, u8 disengage_solenoid, bool force_update)
{
	if (direction != DIR_STRAIGHT && direction != DIR_CURVED) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	switch_lookup_entry_t *switch_entry = conductor_get_switch_lookup_entry(switch_id);

	if (!force_update && (direction == switch_entry->state.direction)) {
		return MARKLIN_ERROR_OK;
	}

	u8 cmd;
	if (direction == DIR_STRAIGHT) {
		cmd = MARKLIN_CMD_SWITCH_STRAIGHT;
	} else {
		cmd = MARKLIN_CMD_SWITCH_CURVE;
	}

	// Schedule the switch command
	marklin_error_t result = Marklin_ScheduleCommand(MARKLIN_CMD_TYPE_WITH_PARAM, cmd, switch_id,
							 disengage_solenoid == 1 ?
								 MS_TO_TICK(MARKLIN_SOLENOID_DEACTIVATE_MS) :
								 MS_TO_TICK(MARKLIN_SWITCH_CMD_DELAY_MS));
	if (result != MARKLIN_ERROR_OK) {
		Panic("Switch: Failed to schedule switch command for switch %d", switch_id);
		return result;
	}

	// Update local switch state
	u32 current_tick = 0;
	if (g_conductor_data && g_conductor_data->clock_server_tid >= 0) {
		current_tick = Time(g_conductor_data->clock_server_tid);
	}

	conductor_consume_switch_update(switch_id, direction, current_tick);

	// Disengage solenoid if requested
	if (disengage_solenoid == 1) {
		return Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_SINGLE, MARKLIN_CMD_SOLENOID_OFF, 0, 0,
							   MARKLIN_CMD_PRIORITY_LOW, 0);
	}

	return MARKLIN_ERROR_OK;
}
