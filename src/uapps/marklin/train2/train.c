#include "marklin/train2/train.h"
#include "marklin/command/command.h"
#include "marklin/common/track_node.h"
#include "marklin/conductor/path.h"
#include "marklin/controller/api.h"
#include "marklin/conductor/api.h"
#include "marklin/error.h"
#include "marklin/msgqueue/api.h"
#include "marklin/topology/track.h"
#include "marklin/topology/api.h"
#include "marklin/train2/api.h"
#include "marklin/train/kinematics.h"
#include "marklin/train2/model.h"
#include "compiler.h"
#include "name.h"
#include "stdbool.h"
#include "syscall.h"
#include "clock.h"
#include "clock_server.h"
#include "string.h"
#include "random.h"
#include "types.h"

#define LOG_MODULE "TRAIN2"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"
#include "klog.h"

// Block ownership validation is now handled through conductor APIs
// No direct access to conductor data needed

// ############################################################################
// # State Machine Framework Implementation
// ############################################################################

// State machine state table
static const train_state_descriptor_t train_state_table[] = {
	{ TRAIN_STATE_IDLE, "IDLE", train_state_idle_handler, train_state_idle_entry, train_state_idle_exit },
	{ TRAIN_STATE_MOVING, "MOVING", train_state_moving_handler, train_state_moving_entry, train_state_moving_exit },
	{ TRAIN_STATE_STOPPING, "STOPPING", train_state_stopping_handler, train_state_stopping_entry,
	  train_state_stopping_exit },
	{ TRAIN_STATE_REVERSING, "REVERSING", train_state_reversing_handler, train_state_reversing_entry,
	  train_state_reversing_exit },
	{ TRAIN_STATE_ERROR, "ERROR", train_state_error_handler, train_state_error_entry, train_state_error_exit }
};

static const u32 TRAIN_STATE_TABLE_SIZE = sizeof(train_state_table) / sizeof(train_state_descriptor_t);

// ############################################################################
// # Configuration Constants
// ############################################################################

#define TRAIN_PATH_REQUEST_INTERVAL_MS 2000
#define TRAIN_PATH_CONTINUATION_INTERVAL_MS 500

// Kinematic estimation configuration constants
#define TRAIN_STOPPING_SAFETY_MARGIN_PERCENT 0
#define TRAIN_DEFAULT_SEGMENT_LENGTH_MM 400
#define TRAIN_DEFAULT_BLOCK_LENGTH_MM 800
#define TRAIN_MAX_RESERVATION_MULTIPLIER 2
#define TRAIN_FALLBACK_STOP_TIME_MS 5000
#define TRAIN_EMERGENCY_STOP_THRESHOLD_MM 50
#define TRAIN_LOW_SPEED_THRESHOLD_MM 500 // Distance below which to use low speed
#define TRAIN_LOW_SPEED_LEVEL 5 // Speed level to use for short distances

// Collision avoidance safety constants
#define TRAIN_BLOCK_SAFETY_MARGIN_MM 100 // Additional safety margin for block boundary checks
#define TRAIN_MAX_LOOKAHEAD_BLOCKS 3 // Maximum number of blocks to check ahead

#define GLOBAL_ALLOW_REVERSAL true
#define GLOBAL_USE_BLOCK_EXIT_AS_START false
// ############################################################################
// # Global State
// ############################################################################

const track_node *track_nodes;
int track_nodes_size;
marklin_track_type_t track_type;

// ############################################################################
// # Forward Declarations
// ############################################################################

// Initialization and main loop
static void train_init_task_data(train_task_data_t *data);
static void train_autonomous_loop(train_task_data_t *data);
static void train_position_report(train_task_data_t *data);

// Sensor tracking functions
static void train_process_sensor_update(train_task_data_t *data, const marklin_msgqueue_message_t *message);
static void train_calculate_next_sensors(train_task_data_t *data);
static bool train_is_sensor_expected(train_task_data_t *data, const track_node *sensor_node);
static void train_update_position_from_sensor(train_task_data_t *data, const track_node *sensor_node,
					      const marklin_sensor_state_t *sensor_update);
static void train_check_sensor_timeouts(train_task_data_t *data);

// Operating mode specific functions
static void train_waypoint_mode_update(train_task_data_t *data);

// Random destination helper
static const track_node *train_select_random_destination(train_task_data_t *data);

// Kinematic stop completion handler
static void train_handle_kinematic_stop_completion(train_task_data_t *data, train_stop_action_t stop_reason);

// Speed control helpers
static u8 train_calculate_effective_speed(train_task_data_t *data);
static marklin_error_t train_apply_speed_change(train_task_data_t *data, u8 new_effective_speed);

// Helper function to find minimum speed with velocity data
// static u8 train_find_minimum_speed_with_velocity_data(train_task_data_t *data);

// Block management helpers for safe stopping
static u32 train_calculate_blocks_needed_for_stopping(train_task_data_t *data);
static bool train_try_reserve_stopping_path(train_task_data_t *data, u32 blocks_needed);

// Collision avoidance safety check
static void train_check_block_safety_conditions(train_task_data_t *data);

// Emergency stop functions
marklin_error_t train_force_stop(train_task_data_t *data);
static marklin_error_t train_safe_release_with_lookahead(train_task_data_t *data, const track_node *sensor_node);
static void train_ensure_current_block_reserved(train_task_data_t *data);

// Mode-specific command handlers
static marklin_error_t train_handle_mode_command(train_task_data_t *data, const marklin_train_command_t *command);
static marklin_error_t train_handle_manual_command(train_task_data_t *data, const marklin_train_command_t *command);
static marklin_error_t train_handle_waypoint_command(train_task_data_t *data, const marklin_train_command_t *command);
static marklin_error_t train_handle_emergency_command(train_task_data_t *data, const marklin_train_command_t *command);
static marklin_error_t train_handle_debug_command(train_task_data_t *data, const marklin_train_command_t *command);

// Mode validation helpers
static bool train_is_command_valid_for_mode(train_operating_mode_t mode, marklin_train_command_type_t cmd_type);

// Blacklist checking helper

static void train_init_blacklist_cache(train_task_data_t *data);
static void train_check_blacklisted_sensor_arrival(train_task_data_t *data);

// Centralized signal monitoring functions
// Removed signal-related function declarations - using block-based safety only

// Distance-based reservation calculation
static kinematic_distance_t train_calculate_distance_needed_for_speed(train_task_data_t *data, u8 speed,
								      bool from_higher);

// ############################################################################
// # State Machine Core Functions
// ############################################################################

void train_state_machine_init(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	// Initialize state machine context
	data->state_machine.current_state = TRAIN_STATE_IDLE;
	data->state_machine.previous_state = TRAIN_STATE_IDLE;
	data->state_machine.movement_state = MOVEMENT_STATE_STATIONARY;
	data->state_machine.path_state = PATH_STATE_NONE;
	data->state_machine.reversal_state = REV_STATE_NONE;
	data->state_machine.pending_event = TRAIN_EVENT_NONE;
	data->state_machine.event_pending = false;
	data->state_machine.state_entry_time_tick = Time(data->clock_server_tid);
	data->state_machine.transition_count = 0;

	// Initialize kinematic stopping tracking
	data->state_machine.kinematic_stopping_active = false;
	data->state_machine.kinematic_stop_start_time_tick = 0;
	data->state_machine.kinematic_stop_duration_ms = 0;

	// Initialize state transition tracking
	data->state_machine.last_moving_exit_time_tick = 0;

	// Execute entry action for initial state
	for (u32 i = 0; i < TRAIN_STATE_TABLE_SIZE; i++) {
		if (train_state_table[i].state == TRAIN_STATE_IDLE) {
			if (train_state_table[i].entry_action) {
				train_state_table[i].entry_action(data);
			}
			break;
		}
	}

	log_info("Train %d: State machine initialized in state %s", data->train_id, train_state_name(TRAIN_STATE_IDLE));
}

static train_transition_result_t train_state_machine_transition(train_task_data_t *data, train_state_t new_state)
{
	if (!data || new_state >= TRAIN_STATE_ERROR + 1) {
		log_error("Train %d: Invalid transition to state %d", data ? data->train_id : 0, new_state);
		return TRANSITION_IGNORED;
	}

	if (data->state_machine.current_state == new_state) {
		log_debug("Train %d: Already in state %s, no transition needed", data->train_id,
			  train_state_name(new_state));
		return TRANSITION_HANDLED; // No transition needed
	}

	train_state_t old_state = data->state_machine.current_state;
	u64 transition_time = Time(data->clock_server_tid);

	// Record timestamp when exiting MOVING state for sensor validation
	if (old_state == TRAIN_STATE_MOVING && new_state != TRAIN_STATE_MOVING) {
		data->state_machine.last_moving_exit_time_tick = transition_time;
		log_debug("Train %d: Recorded MOVING exit time: %llu", data->train_id, transition_time);
	}

	log_info("Train %d: State transition %s -> %s (transition #%u) at tick %llu", data->train_id,
		 train_state_name(old_state), train_state_name(new_state), data->state_machine.transition_count + 1,
		 transition_time);

	// Find exit action for current state
	for (u32 i = 0; i < TRAIN_STATE_TABLE_SIZE; i++) {
		if (train_state_table[i].state == old_state) {
			if (train_state_table[i].exit_action) {
				train_state_table[i].exit_action(data);
			}
			break;
		}
	}

	// Update state
	data->state_machine.previous_state = old_state;
	data->state_machine.current_state = new_state;
	data->state_machine.state_entry_time_tick = Time(data->clock_server_tid);
	data->state_machine.transition_count++;

	// Find entry action for new state
	for (u32 i = 0; i < TRAIN_STATE_TABLE_SIZE; i++) {
		if (train_state_table[i].state == new_state) {
			if (train_state_table[i].entry_action) {
				train_state_table[i].entry_action(data);
			}
			break;
		}
	}

	log_info("Train %d: State transition %s -> %s (count: %d) @ %d", data->train_id, train_state_name(old_state),
		 train_state_name(new_state), data->state_machine.transition_count, Time(data->clock_server_tid));

	return TRANSITION_HANDLED;
}

train_transition_result_t train_state_machine_process_event(train_task_data_t *data, train_event_t event)
{
	if (!data || event == TRAIN_EVENT_NONE) {
		return TRANSITION_IGNORED;
	}

	// Find handler for current state
	train_state_handler_t handler = NULL;
	for (u32 i = 0; i < TRAIN_STATE_TABLE_SIZE; i++) {
		if (train_state_table[i].state == data->state_machine.current_state) {
			handler = train_state_table[i].handler;
			break;
		}
	}

	if (!handler) {
		log_error("Train %d: No handler found for state %s", data->train_id,
			  train_state_name(data->state_machine.current_state));
		return TRANSITION_IGNORED;
	}

	log_debug("Train %d: Processing event %s in state %s", data->train_id, train_event_name(event),
		  train_state_name(data->state_machine.current_state));

	train_transition_result_t result = handler(data, event);

	log_debug("Train %d: Event %s result: %s", data->train_id, train_event_name(event),
		  train_transition_result_name(result));

	return result;
}

void train_state_machine_update(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	// Process any pending events
	if (data->state_machine.event_pending) {
		train_state_machine_process_event(data, data->state_machine.pending_event);
		data->state_machine.event_pending = false;
		data->state_machine.pending_event = TRAIN_EVENT_NONE;
	}

	// Check for kinematic stop completion in MOVING state
	if (data->state_machine.current_state == TRAIN_STATE_MOVING && data->state_machine.kinematic_stopping_active) {
		if (train_check_kinematic_stop_complete(data)) {
			// Handle stop completion based on reason
			train_handle_kinematic_stop_completion(data, data->state_machine.kinematic_stop_reason);

			// Clear kinematic stopping state
			data->state_machine.kinematic_stopping_active = false;
			data->state_machine.movement_state = MOVEMENT_STATE_STATIONARY;

			// Transition to IDLE
			train_state_machine_transition(data, TRAIN_STATE_IDLE);
			log_info("Train %d: Kinematic stop complete, transitioned to IDLE @%d", data->train_id,
				 Time(data->clock_server_tid));
		}
	}

	// Check for emergency stop completion in STOPPING state
	if (data->state_machine.current_state == TRAIN_STATE_STOPPING && data->motion.commanded_speed == 0) {
		// Emergency stop complete, transition to IDLE
		train_state_machine_transition(data, TRAIN_STATE_IDLE);
		log_info("Train %d: Emergency stop complete, transitioned to IDLE", data->train_id);
	}

	// Update movement sub-state for acceleration/deceleration completion in MOVING state
	if (data->state_machine.current_state == TRAIN_STATE_MOVING && !data->state_machine.kinematic_stopping_active) {
		// Check if acceleration/deceleration has completed (reached target speed)
		if (data->state_machine.movement_state == MOVEMENT_STATE_ACCELERATING ||
		    data->state_machine.movement_state == MOVEMENT_STATE_DECELERATING) {
			// In this implementation, we assume commanded_speed represents target speed
			// A more sophisticated implementation would track actual vs commanded speed
			u8 target_speed = data->motion.requested_speed;
			u8 current_speed = data->motion.commanded_speed;

			if (current_speed == target_speed && target_speed > 0) {
				data->state_machine.movement_state = MOVEMENT_STATE_CRUISING;
				log_debug("Train %d: Reached target speed %d, now CRUISING", data->train_id,
					  target_speed);
			} else if (current_speed == 0) {
				data->state_machine.movement_state = MOVEMENT_STATE_STATIONARY;
				log_debug("Train %d: Speed reached 0, now STATIONARY", data->train_id);
			}
		}
	}
}

const char *train_state_name(train_state_t state)
{
	for (u32 i = 0; i < TRAIN_STATE_TABLE_SIZE; i++) {
		if (train_state_table[i].state == state) {
			return train_state_table[i].name;
		}
	}
	return "UNKNOWN";
}

const char *train_event_name(train_event_t event)
{
	switch (event) {
	case TRAIN_EVENT_NONE:
		return "NONE";
	case TRAIN_EVENT_START_MOVING:
		return "START_MOVING";
	case TRAIN_EVENT_STOP_REQUESTED:
		return "STOP_REQUESTED";
	case TRAIN_EVENT_EMERGENCY_STOP:
		return "EMERGENCY_STOP";
	case TRAIN_EVENT_SENSOR_TRIGGERED:
		return "SENSOR_TRIGGERED";
	case TRAIN_EVENT_DESTINATION_REACHED:
		return "DESTINATION_REACHED";
	case TRAIN_EVENT_PATH_END_REACHED:
		return "PATH_END_REACHED";
	case TRAIN_EVENT_REVERSAL_NEEDED:
		return "REVERSAL_NEEDED";
	case TRAIN_EVENT_REVERSAL_COMPLETE:
		return "REVERSAL_COMPLETE";
	case TRAIN_EVENT_ERROR_DETECTED:
		return "ERROR_DETECTED";
	case TRAIN_EVENT_SPEED_CHANGED:
		return "SPEED_CHANGED";
	case TRAIN_EVENT_PATH_CONTINUATION_NEEDED:
		return "PATH_CONTINUATION_NEEDED";
	default:
		return "UNKNOWN";
	}
}

const char *train_movement_state_name(train_movement_state_t state)
{
	switch (state) {
	case MOVEMENT_STATE_STATIONARY:
		return "STATIONARY";
	case MOVEMENT_STATE_ACCELERATING:
		return "ACCELERATING";
	case MOVEMENT_STATE_CRUISING:
		return "CRUISING";
	case MOVEMENT_STATE_DECELERATING:
		return "DECELERATING";
	default:
		return "UNKNOWN";
	}
}

const char *train_path_state_name(train_path_state_t state)
{
	switch (state) {
	case PATH_STATE_NONE:
		return "NONE";
	case PATH_STATE_REQUESTING:
		return "REQUESTING";
	case PATH_STATE_ACTIVE:
		return "ACTIVE";
	case PATH_STATE_REACHED:
		return "REACHED";
	case PATH_STATE_CONTINUATION_NEEDED:
		return "CONTINUATION_NEEDED";
	case PATH_STATE_AT_REVERSAL:
		return "AT_REVERSAL";
	case PATH_STATE_REVERSING:
		return "REVERSING";
	default:
		return "UNKNOWN";
	}
}

const char *train_reversal_state_name(train_reversal_state_t state)
{
	switch (state) {
	case REV_STATE_NONE:
		return "NONE";
	case REV_STATE_STOPPING:
		return "STOPPING";
	case REV_STATE_COMMAND:
		return "COMMAND";
	case REV_STATE_CLEARING:
		return "CLEARING";
	case REV_STATE_RESUMING:
		return "RESUMING";
	default:
		return "UNKNOWN";
	}
}

const char *train_transition_result_name(train_transition_result_t result)
{
	switch (result) {
	case TRANSITION_HANDLED:
		return "HANDLED";
	case TRANSITION_IGNORED:
		return "IGNORED";
	case TRANSITION_DEFERRED:
		return "DEFERRED";
	default:
		return "UNKNOWN";
	}
}

void train_state_machine_debug_print_status(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	log_debug("Train %d State Machine Status:", data->train_id);
	log_debug("  Primary State: %s", train_state_name(data->state_machine.current_state));
	log_debug("  Previous State: %s", train_state_name(data->state_machine.previous_state));
	log_debug("  Movement State: %s", train_movement_state_name(data->state_machine.movement_state));
	log_debug("  Path State: %s", train_path_state_name(data->state_machine.path_state));
	log_debug("  Reversal State: %s", train_reversal_state_name(data->state_machine.reversal_state));
	log_debug("  Pending Event: %s", train_event_name(data->state_machine.pending_event));
	log_debug("  Event Pending: %s", data->state_machine.event_pending ? "YES" : "NO");
	log_debug("  State Entry Time: %u", data->state_machine.state_entry_time_tick);
	log_debug("  Transition Count: %u", data->state_machine.transition_count);
}

train_status_t train_get_external_status(const train_task_data_t *data)
{
	if (!data) {
		return TRAIN_STATUS_IDLE;
	}

	// Map state machine states to external API status
	switch (data->state_machine.current_state) {
	case TRAIN_STATE_IDLE:
		// Check path sub-state for more specific status
		if (data->state_machine.path_state == PATH_STATE_REQUESTING) {
			return TRAIN_STATUS_REQUESTING_PATH;
		}
		return TRAIN_STATUS_IDLE;

	case TRAIN_STATE_MOVING:
		// If kinematic stopping is active, report as stopping to external API
		if (data->state_machine.kinematic_stopping_active) {
			return TRAIN_STATUS_STOPPING;
		}
		return TRAIN_STATUS_MOVING;

	case TRAIN_STATE_REVERSING:
		return TRAIN_STATUS_MOVING;

	case TRAIN_STATE_STOPPING:
		return TRAIN_STATUS_STOPPING;

	case TRAIN_STATE_ERROR:
	default:
		// Error state maps to IDLE for external interface
		return TRAIN_STATUS_IDLE;
	}
}

bool train_check_kinematic_stop_complete(train_task_data_t *data)
{
	if (!data || !data->state_machine.kinematic_stopping_active) {
		return false;
	}

	u32 current_time_tick = Time(data->clock_server_tid);
	u32 elapsed_time_tick = current_time_tick - data->state_machine.kinematic_stop_start_time_tick;

	// Check if enough time has elapsed based on kinematic calculations
	if (elapsed_time_tick >=
	    MS_TO_TICK(data->state_machine.kinematic_stop_duration_ms) + MS_TO_TICK(4000)) { // 4 second buffer
		log_debug("Train %d: Kinematic stop complete after %u ticks (predicted %u ms)", data->train_id,
			  elapsed_time_tick, data->state_machine.kinematic_stop_duration_ms);
		return true;
	}

	// // Also check if train speed is actually 0 (safety check)
	// if (data->motion.commanded_speed == 0) {
	// 	log_debug("Train %d: Kinematic stop complete - speed reached 0 after %u tick", data->train_id,
	// 		  elapsed_time_tick);
	// 	return true;
	// }

	return false;
}

static void train_handle_kinematic_stop_completion(train_task_data_t *data, train_stop_action_t stop_reason)
{
	if (!data) {
		return;
	}

	log_info("Train %d: Handling kinematic stop completion (reason: %d)", data->train_id, stop_reason);

	switch (stop_reason) {
	case TRAIN_STOP_DESTINATION:
		// Clear destination and set path state to NONE (as requested by user)
		if (data->destination) {
			// Record arrival time for random destination pause
			if (data->random_destination_enabled) {
				data->destination_arrival_time = Time(data->clock_server_tid);
				log_info("Train %d: Arrived at random destination, starting pause timer",
					 data->train_id);
			}

			log_info("Train %d: Reached destination %s, clearing destination", data->train_id,
				 data->destination_name);
			data->destination = NULL;
			data->destination_name[0] = '\0';
			data->destination_offset_mm = 0;
		}
		data->state_machine.path_state = PATH_STATE_NONE;
		break;

	case TRAIN_STOP_PATH_END:
		// Keep destination but mark path for continuation
		log_info("Train %d: Stopped at end of activated path segment", data->train_id);
		data->state_machine.path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = true;
		break;

	case TRAIN_STOP_REVERSAL:
		// This case should no longer occur with simplified logic
		log_warn("Train %d: Unexpected stop for reversal - treating as path end", data->train_id);
		data->state_machine.path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = true;
		break;

	case TRAIN_CONTINUE:
	default:
		// Manual stop or force stop - check if path continuation is needed
		if (data->needs_path_continuation && data->destination != NULL) {
			log_info("Train %d: Force stop completed - destination %s preserved for continuation",
				 data->train_id, data->destination_name);
			data->state_machine.path_state = PATH_STATE_CONTINUATION_NEEDED;
		} else {
			log_info("Train %d: Manual stop completed", data->train_id);
		}
		break;
	}
}

static const char *train_external_status_name(train_status_t status)
{
	switch (status) {
	case TRAIN_STATUS_IDLE:
		return "IDLE";
	case TRAIN_STATUS_REQUESTING_PATH:
		return "REQUESTING_PATH";
	case TRAIN_STATUS_MOVING:
		return "MOVING";
	case TRAIN_STATUS_STOPPING:
		return "STOPPING";
	default:
		return "UNKNOWN";
	}
}

// Convert stop conditions to state machine events
static void train_check_and_generate_events(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	// Only check conditions if in MOVING state and actually moving
	if (data->state_machine.current_state != TRAIN_STATE_MOVING || data->motion.commanded_speed == 0) {
		return;
	}

	// Use existing unified stop condition logic but convert to events
	train_stop_action_t stop_action = train_check_unified_stop_conditions(data);

	switch (stop_action) {
	case TRAIN_CONTINUE:
		// No event needed
		break;

	case TRAIN_STOP_DESTINATION:
		train_state_machine_process_event(data, TRAIN_EVENT_DESTINATION_REACHED);
		break;

	case TRAIN_STOP_PATH_END:
		train_state_machine_process_event(data, TRAIN_EVENT_PATH_END_REACHED);
		break;

	case TRAIN_STOP_REVERSAL:
		train_state_machine_process_event(data, TRAIN_EVENT_REVERSAL_NEEDED);
		break;

	case TRAIN_STOP_LOW_SPEED_TIMER:
		// For low speed timer expiration, treat as destination reached if at destination,
		// otherwise treat as path end for continuation
		if (data->destination && data->motion.current_position.sensor == data->destination) {
			train_state_machine_process_event(data, TRAIN_EVENT_DESTINATION_REACHED);
		} else {
			train_state_machine_process_event(data, TRAIN_EVENT_PATH_END_REACHED);
		}
		break;

	case TRAIN_EMERGENCY_STOP:
		train_state_machine_process_event(data, TRAIN_EVENT_EMERGENCY_STOP);
		break;
	}
}

// ############################################################################
// # Speed Control Helper Functions
// ############################################################################

void train_report(train_task_data_t *data)
{
	UNUSED(data);
	// log_info("Train %d: Current stop distance: %d", data->train_id, data->motion.current_stop_distance);
}

// Helper function to calculate distance to destination for low speed mode detection
static kinematic_distance_t train_calculate_distance_to_destination(train_task_data_t *data)
{
	if (!data || !data->destination) {
		return 0;
	}

	// Use path total distance if available (most accurate)
	if (data->has_active_path && data->current_path.total_distance > 0) {
		return data->current_path.total_distance;
	}

	// Fall back to position-based calculation
	train_position_t current_pos = data->motion.current_position;
	if (!current_pos.sensor) {
		return 0;
	}

	kinematic_distance_t compensated_offset =
		train_calculate_stopping_offset(data, data->destination_offset_mm, data->motion.direction);

	train_position_t target_pos = { .sensor = data->destination, .offset_mm = compensated_offset };
	return train_position_distance_between(&current_pos, &target_pos, true);
}

static u8 train_calculate_effective_speed(train_task_data_t *data)
{
	if (!data) {
		return 0;
	}

	// Check if kinematic stopping is active - maintain speed 0 during deceleration
	if (data->state_machine.kinematic_stopping_active) {
		return 0;
	}

	// In waypoint mode without a destination, effective speed should be 0
	if (data->operating_mode == TRAIN_MODE_WAYPOINT) {
		if (data->destination == NULL) {
			return 0;
		}
		switch (data->state_machine.path_state) {
		case PATH_STATE_ACTIVE:
			break;
		case PATH_STATE_NONE:
		case PATH_STATE_REQUESTING:
		case PATH_STATE_REACHED:
		case PATH_STATE_AT_REVERSAL:
		case PATH_STATE_CONTINUATION_NEEDED:
			return 0;
		case PATH_STATE_REVERSING:
			return data->motion.commanded_speed;
		}
	}

	// Check for low speed mode for short distance navigation
	if (data->operating_mode == TRAIN_MODE_WAYPOINT && data->destination != NULL &&
	    data->motion.requested_speed > 0 &&
	    (data->state_machine.current_state == TRAIN_STATE_IDLE ||
	     data->state_machine.current_state == TRAIN_STATE_MOVING)) {
		// Calculate distance to check for low speed mode
		kinematic_distance_t distance_to_check = 0;

		if (data->state_machine.path_state == PATH_STATE_ACTIVE && data->activation_end_point) {
			// During active navigation, check distance to activation end point
			train_position_t current_pos = data->motion.current_position;
			train_position_t end_pos = { .sensor = data->activation_end_point, .offset_mm = 0 };
			kinematic_distance_t distance_to_activation =
				train_position_distance_between(&current_pos, &end_pos, true);

			// If we're very close to activation end point, use distance to final destination instead
			// This prevents tiny distances that cause immediate timer expiration
			if (distance_to_activation < 200 && data->destination) {
				distance_to_check = train_calculate_distance_to_destination(data);
				log_debug(
					"Train %d: Close to activation end (%lldmm), using destination distance (%lldmm)",
					data->train_id, distance_to_activation, distance_to_check);
			} else {
				distance_to_check = distance_to_activation;
			}
		} else {
			// When starting or no active path, check distance to final destination
			distance_to_check = train_calculate_distance_to_destination(data);
		}

		if (train_should_use_low_speed_mode(data, distance_to_check)) {
			// Start low speed mode if not already active
			if (!data->low_speed_mode_active) {
				train_start_low_speed_mode(data, distance_to_check);
			}
			log_debug("Train %d: Using low speed mode for short distance (%lldmm)", data->train_id,
				  distance_to_check);
			return TRAIN_LOW_SPEED_LEVEL;
		}
	}

	// Calculate the effective speed (requested speed with block-based constraints)
	u8 normal_effective_speed = data->motion.requested_speed;

	// Return the normal effective speed
	return normal_effective_speed;
}

static marklin_error_t train_apply_speed_change(train_task_data_t *data, u8 new_effective_speed)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (data->motion.commanded_speed != new_effective_speed) {
		u8 old_speed = data->motion.commanded_speed;
		marklin_error_t result =
			train_set_speed_and_headlight(data, new_effective_speed, MARKLIN_TRAIN_HEADLIGHT_AUTO);

		if (result == MARKLIN_ERROR_OK) {
			data->motion.commanded_speed = new_effective_speed;

			// Update movement state based on speed change (only if in MOVING state)
			if (data->state_machine.current_state == TRAIN_STATE_MOVING &&
			    !data->state_machine.kinematic_stopping_active) {
				if (new_effective_speed == 0) {
					data->state_machine.movement_state = MOVEMENT_STATE_STATIONARY;
				} else if (new_effective_speed > old_speed) {
					data->state_machine.movement_state = MOVEMENT_STATE_ACCELERATING;
				} else if (new_effective_speed < old_speed) {
					data->state_machine.movement_state = MOVEMENT_STATE_DECELERATING;
				}
				// If speed unchanged, keep current movement state

				log_debug("Train %d: Speed change %d->%d, movement state: %s", data->train_id,
					  old_speed, new_effective_speed,
					  train_movement_state_name(data->state_machine.movement_state));
			}

			// Generate appropriate state machine events based on speed transitions
			if (old_speed == 0 && new_effective_speed > 0) {
				// Starting movement - transition from idle to moving
				train_state_machine_process_event(data, TRAIN_EVENT_START_MOVING);
			} else if (old_speed > 0 && new_effective_speed == 0) {
				// Stopping movement - transition from moving to idle
				train_state_machine_process_event(data, TRAIN_EVENT_STOP_REQUESTED);
			} else {
				// Speed change within same movement state
				train_state_machine_process_event(data, TRAIN_EVENT_SPEED_CHANGED);
			}
		}
		return result;
	}

	return MARKLIN_ERROR_OK;
}

static marklin_error_t train_update_effective_speed(train_task_data_t *data)
{
	u8 new_effective_speed = train_calculate_effective_speed(data);
	return train_apply_speed_change(data, new_effective_speed);
}

// ############################################################################
// # Mode Management Functions
// ############################################################################

static bool train_is_command_valid_for_mode(train_operating_mode_t mode, marklin_train_command_type_t cmd_type)
{
	switch (cmd_type) {
	// Mode management commands
	case MARKLIN_TRAIN_CMD_SET_MODE:
		return true;

	// Manual mode commands
	case MARKLIN_TRAIN_CMD_MANUAL_SET_EFFECTIVE_SPEED:
	case MARKLIN_TRAIN_CMD_MANUAL_TOGGLE_HEADLIGHT:
	case MARKLIN_TRAIN_CMD_MANUAL_STOP:
		return (mode == TRAIN_MODE_MANUAL);

	case MARKLIN_TRAIN_CMD_MANUAL_REVERSE:
		// Allow reverse in manual mode, or in waypoint mode when stopped
		return (mode == TRAIN_MODE_MANUAL || mode == TRAIN_MODE_WAYPOINT);

	// Waypoint mode commands
	case MARKLIN_TRAIN_CMD_SET_REQUESTED_SPEED:
		return (mode == TRAIN_MODE_WAYPOINT);

	case MARKLIN_TRAIN_CMD_SET_DESTINATION:
		return (mode == TRAIN_MODE_WAYPOINT);

	// Emergency commands
	case MARKLIN_TRAIN_CMD_EMERGENCY_STOP:
		return true;

	// Navigation commands - allowed in all modes (switches to waypoint mode)
	case MARKLIN_TRAIN_CMD_NAVIGATE_TO_DESTINATION:
		return true;

	// Model query - allowed in all modes
	case MARKLIN_TRAIN_CMD_GET_KINEMATIC_MODEL:
		return true;

	// Debug command - allowed in all modes
	case MARKLIN_TRAIN_CMD_DEBUG_INFO:
		return true;

	// Clear destination command - allowed in waypoint mode
	case MARKLIN_TRAIN_CMD_CLEAR_DESTINATION:
		return (mode == TRAIN_MODE_WAYPOINT);

	// Random destination mode - only allowed in waypoint mode
	case MARKLIN_TRAIN_CMD_SET_RANDOM_DESTINATION_MODE:
		log_info("Train %d: Random destination mode command received", mode);
		return (mode == TRAIN_MODE_WAYPOINT);

	default:
		return false;
	}
}

// Switch train to a new operating mode with proper cleanup
void train_switch_to_mode(train_task_data_t *data, train_operating_mode_t new_mode)
{
	if (data->operating_mode == new_mode) {
		return;
	}

	switch (data->operating_mode) {
	case TRAIN_MODE_WAYPOINT:
		if (data->has_active_path) {
			data->has_active_path = false;
		}
		train_release_all_blocks(data, true);
		break;
	case TRAIN_MODE_MANUAL:
		break;
	}

	data->operating_mode = new_mode;

	switch (new_mode) {
	case TRAIN_MODE_MANUAL:
		// Manual mode - no signal constraints
		break;
	case TRAIN_MODE_WAYPOINT:
		data->has_active_path = false;
		break;
	}
}

// ############################################################################
// # Mode-Specific Command Handlers
// ############################################################################

static marklin_error_t train_handle_mode_command(train_task_data_t *data, const marklin_train_command_t *command)
{
	switch (command->command_type) {
	case MARKLIN_TRAIN_CMD_SET_MODE:
		train_switch_to_mode(data, command->set_mode.mode);
		return MARKLIN_ERROR_OK;
	default:
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}
}

static marklin_error_t train_handle_manual_command(train_task_data_t *data, const marklin_train_command_t *command)
{
	switch (command->command_type) {
	case MARKLIN_TRAIN_CMD_MANUAL_SET_EFFECTIVE_SPEED:
		// In manual mode, directly set the effective speed (ignoring signal limits)
		data->motion.commanded_speed = command->manual_set_effective_speed.effective_speed;
		if (command->manual_set_effective_speed.headlight != MARKLIN_TRAIN_HEADLIGHT_AUTO) {
			data->headlight = command->manual_set_effective_speed.headlight;
		}

		// Generate state machine events based on speed
		if (command->manual_set_effective_speed.effective_speed > 0) {
			train_state_machine_process_event(data, TRAIN_EVENT_START_MOVING);
		} else {
			train_state_machine_process_event(data, TRAIN_EVENT_STOP_REQUESTED);
		}

		return train_set_speed_and_headlight(data, data->motion.commanded_speed, data->headlight);

	case MARKLIN_TRAIN_CMD_MANUAL_REVERSE:
		// Generate reversal event if not already reversing
		if (data->state_machine.current_state != TRAIN_STATE_REVERSING) {
			train_state_machine_process_event(data, TRAIN_EVENT_REVERSAL_NEEDED);
		}
		return MARKLIN_ERROR_OK;

	case MARKLIN_TRAIN_CMD_MANUAL_TOGGLE_HEADLIGHT:
		return train_toggle_headlight(data);

	case MARKLIN_TRAIN_CMD_MANUAL_STOP:
		train_state_machine_process_event(data, TRAIN_EVENT_STOP_REQUESTED);
		return MARKLIN_ERROR_OK;

	default:
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}
}

static marklin_error_t train_handle_waypoint_command(train_task_data_t *data, const marklin_train_command_t *command)
{
	switch (command->command_type) {
	case MARKLIN_TRAIN_CMD_SET_REQUESTED_SPEED:
		data->motion.requested_speed = command->set_requested_speed.requested_speed;
		return train_update_effective_speed(data);

	case MARKLIN_TRAIN_CMD_SET_DESTINATION:
		if (data->operating_mode != TRAIN_MODE_WAYPOINT) {
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}
		return train_set_destination(data, (const struct marklin_destination_cmd *)&command->set_destination);

	case MARKLIN_TRAIN_CMD_MANUAL_REVERSE:
		// Allow reverse in waypoint mode only when stopped
		if (data->operating_mode == TRAIN_MODE_WAYPOINT && data->motion.commanded_speed == 0) {
			return train_reverse_and_continue(data);
		}
		return MARKLIN_ERROR_INVALID_ARGUMENT;

	case MARKLIN_TRAIN_CMD_SET_RANDOM_DESTINATION_MODE:
		data->random_destination_enabled = command->set_random_destination_mode.enabled;
		data->last_random_destination_time = 0; // Reset timer when toggling
		data->destination_arrival_time = 0; // Reset arrival timer when toggling
		log_info("Train %d: Random destination mode %s", data->train_id,
			 data->random_destination_enabled ? "enabled" : "disabled");
		return MARKLIN_ERROR_OK;

	default:
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}
}

static marklin_error_t train_handle_emergency_command(train_task_data_t *data, const marklin_train_command_t *command)
{
	switch (command->command_type) {
	case MARKLIN_TRAIN_CMD_EMERGENCY_STOP:
		train_state_machine_process_event(data, TRAIN_EVENT_EMERGENCY_STOP);
		return MARKLIN_ERROR_OK;

	default:
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}
}

static marklin_error_t train_handle_debug_command(train_task_data_t *data, const marklin_train_command_t *command)
{
	UNUSED(command);

	log_warn("==== TRAIN %d DEBUG INFO ====", data->train_id);

	// Identity and basic info
	log_warn("Identity: ID=%d, Length=%lldmm", data->train_id, data->train_length_mm);

	// Motion state
	const char *direction_str = (data->motion.direction == TRAIN_DIRECTION_FORWARD) ? "FORWARD" : "REVERSE";
	log_warn("Motion: Commanded=%d, Direction=%s", data->motion.commanded_speed, direction_str);
	log_warn("        Stop Distance=%lldmm, Requested Speed=%d", data->motion.current_stop_distance,
		 data->motion.requested_speed);

	// Position and navigation
	const char *current_sensor = data->motion.current_position.sensor ? data->motion.current_position.sensor->name :
									    "UNKNOWN";
	log_warn("Position: Sensor=%s, Offset=%lldmm", current_sensor, data->motion.current_position.offset_mm);

	const char *dest_sensor = data->destination ? data->destination->name : "NONE";
	log_warn("Destination: %s (offset=%lldmm)", dest_sensor, data->destination_offset_mm);

	// Expected sensors and timing
	log_warn("Expected Sensors: Count=%d", data->motion.expected_sensor_count);
	for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
		const char *sensor_name = data->motion.expected_sensors[i] ? data->motion.expected_sensors[i]->name :
									     "NULL";
		kinematic_time_t current_time = Time(data->clock_server_tid);
		i32 time_to_deadline = kinematic_ticks_to_ms(data->motion.sensor_timeout_deadlines[i] - current_time);
		log_warn("  [%d]: %s, Distance=%lldmm, Timeout in %dms", i, sensor_name,
			 data->motion.expected_distances[i], time_to_deadline);
	}

	// Status and mode
	const char *status_str = train_external_status_name(train_get_external_status(data));
	const char *mode_str = (data->operating_mode == TRAIN_MODE_MANUAL) ? "MANUAL" : "WAYPOINT";
	log_warn("Status: %s (State: %s), Mode: %s", status_str, train_state_name(data->state_machine.current_state),
		 mode_str);

	// Path and reservations
	if (data->has_active_path) {
		log_warn("Path: ACTIVE, State=%d, Ends at reversal=%s, End of activation=%s",
			 data->state_machine.path_state, data->path_ends_at_reversal ? "YES" : "NO",
			 data->activation_end_point ? data->activation_end_point->name : "None");

		// Print the path diagram
		path_print(&data->current_path);
	} else {
		log_warn("Path: NONE");
	}
	log_warn("Reservations: %d blocks", data->reserved_block_count);

	// Kinematic model status
	log_warn("Kinematic: Model enabled=%s", data->kinematic_model_enabled ? "YES" : "NO");

	// Random destination status
	if (data->random_destination_enabled) {
		if (data->destination_arrival_time > 0) {
			kinematic_time_t current_time = Time(data->clock_server_tid);
			u32 time_since_arrival = kinematic_ticks_to_ms(current_time - data->destination_arrival_time);
			log_warn("Random Destination: ENABLED, Time since arrival: %ums", time_since_arrival);
		} else {
			log_warn("Random Destination: ENABLED, No recent arrival");
		}
	} else {
		log_warn("Random Destination: DISABLED");
	}

	log_warn("==== END DEBUG INFO ====");

	return MARKLIN_ERROR_OK;
}

// ############################################################################
// # Initialization Functions
// ############################################################################

static void train_init_task_data(train_task_data_t *data)
{
	memset(data, 0, sizeof(train_task_data_t));
	data->train_id = 0;
	data->destination = NULL;
	data->destination_name[0] = '\0';
	// Status now handled by state machine
	data->operating_mode = TRAIN_MODE_MANUAL; // Default to manual mode

	data->clock_server_tid = -1;
	data->controller_tid = -2;
	data->conductor_tid = -3;
	data->command_server_tid = -4;

	// Initialize unified motion state
	memset(&data->motion, 0, sizeof(train_motion_state_t));
	data->motion.commanded_speed = 0;
	data->motion.requested_speed = 0;
	data->motion.direction = TRAIN_DIRECTION_FORWARD;
	data->motion.current_position.sensor = NULL;
	data->motion.commanded_speed_from_higher = false;
	data->motion.current_stop_distance = 0;
	data->motion.last_stop_distance_update = 0;
	data->motion.expected_sensors[0] = NULL;
	data->motion.expected_sensors[1] = NULL;
	data->motion.expected_distances[0] = 0;
	data->motion.expected_distances[1] = 0;
	data->motion.expected_arrival_times[0] = 0;
	data->motion.expected_arrival_times[1] = 0;
	data->motion.sensor_timeout_deadlines[0] = 0;
	data->motion.sensor_timeout_deadlines[1] = 0;
	data->motion.sensor_timeout_logged[0] = false;
	data->motion.sensor_timeout_logged[1] = false;
	data->motion.expected_sensor_count = 0;

	data->headlight = MARKLIN_TRAIN_HEADLIGHT_ON;

	data->last_path_request_tick = 0;
	data->last_position_report_tick = 0;
	data->last_path_continuation_tick = 0;

	// Sensor tracking initialization
	data->sensor_subscription_active = false;
	data->last_sensor_trigger_tick = 0;

	// Operating mode specific initialization - Waypoint mode
	memset(&data->current_path, 0, sizeof(path_result_t));
	data->has_active_path = false;
	data->path_ends_at_reversal = false;

	// Path management state machine initialization
	data->state_machine.path_state = PATH_STATE_NONE;

	// Block-based safety only - no signal system

	// Kinematic model initialization
	data->kinematic_model_enabled = true; // Enable by default

	data->kinematic_model = NULL;

	// Random destination initialization
	data->random_destination_enabled = false; // Disabled by default
	data->last_random_destination_time = 0;
	data->destination_arrival_time = 0;

	// Low speed mode initialization
	data->low_speed_mode_active = false;
	data->low_speed_start_time = 0;
	data->low_speed_expected_duration_ms = 0;
	data->low_speed_target_distance = 0;

	// NOTE: State machine will be initialized later when clock_server_tid is available
}

static void train_init_blacklist_cache(train_task_data_t *data)
{
	memset(data->sensor_blacklist_cache, false, sizeof(data->sensor_blacklist_cache));

	sensor_blacklist_t blacklist;
	marklin_error_t result = Marklin_GetSensorBlacklist(&blacklist);

	if (result != MARKLIN_ERROR_OK) {
		return;
	}

	for (u8 i = 0; i < blacklist.count; i++) {
		u8 bank = blacklist.sensors[i].bank;
		u8 sensor_id = blacklist.sensors[i].sensor_id;

		if (bank < 5 && sensor_id >= 1 && sensor_id <= 16) {
			data->sensor_blacklist_cache[bank][sensor_id - 1] = true;
		}
	}
}

// ############################################################################
// # Main Train Task Entry Point
// ############################################################################

void __noreturn marklin_train_task(void)
{
	train_task_data_t train_data;

	train_init_task_data(&train_data);

	// Initialize kinematic model system
	marklin_error_t model_init_result = kinematic_model_init();
	if (model_init_result != MARKLIN_ERROR_OK) {
		log_error("Train task: Failed to initialize kinematic model system: %d", model_init_result);
		Exit();
	}

	train_data.clock_server_tid = WhoIs(CLOCK_SERVER_NAME);
	if (train_data.clock_server_tid <= 0) {
		Exit();
	}

	// Initialize state machine now that we have clock server
	train_state_machine_init(&train_data);

	train_data.conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (train_data.conductor_tid <= 0) {
		Exit();
	}

	train_data.command_server_tid = WhoIs(MARKLIN_CMD_SERVER_NAME);
	if (train_data.command_server_tid <= 0) {
		Exit();
	}

	train_data.controller_tid = WhoIs(MARKLIN_CONTROLLER_SERVER_NAME);
	if (train_data.controller_tid <= 0) {
		Exit();
	}

	track_nodes_size = Marklin_GetTrackNodes(&track_nodes, &track_type);
	if (track_nodes_size < 0) {
		Exit();
	}

	// Initialize sensor blacklist cache
	train_init_blacklist_cache(&train_data);

	marklin_train_spawn_info_t train_info;
	if (Marklin_ControllerGetSelfTrainInfo(&train_info) != MARKLIN_ERROR_OK) {
		Exit();
	}

	train_data.train_id = train_info.train_id;

	// Initialize motion state with starting location
	// Initialize motion state with starting location
	train_data.motion.current_position.sensor = train_info.init_location;
	train_data.motion.current_position.sensor = train_info.init_location;
	train_data.motion.current_position.offset_mm = 0;
	train_data.motion.last_position_update = Time(train_data.clock_server_tid);

	// // Initialize kinematic model for this specific train
	marklin_error_t model_create_result = kinematic_model_create_default(&train_data);
	if (model_create_result != MARKLIN_ERROR_OK) {
		log_error("Train %d: Failed to initialize kinematic model: %d", train_data.train_id,
			  model_create_result);
		// Continue execution - kinematic model is optional for basic train operation
	} else {
		log_info("Train %d: Kinematic model initialized successfully", train_data.train_id);
	}

	train_set_speed_and_headlight(&train_data, 0, MARKLIN_TRAIN_HEADLIGHT_ON);

	// Subscribe to sensor updates for position tracking

	train_switch_to_mode(&train_data, TRAIN_MODE_WAYPOINT);

	marklin_error_t sub_result =
		Marklin_MsgQueue_Subscribe(MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE, &train_data.sensor_subscription);
	if (sub_result == MARKLIN_ERROR_OK) {
		train_data.sensor_subscription_active = true;
		log_info("Train %d: Subscribed to sensor updates", train_data.train_id);
	} else {
		log_error("Train %d: Failed to subscribe to sensor updates: %d", train_data.train_id, sub_result);
	}

	// Calculate initial expected sensors
	train_calculate_next_sensors(&train_data);

	train_autonomous_loop(&train_data);

	Exit();
	UNREACHABLE();
}

// ############################################################################
// # Position Reporting Functions
// ############################################################################

static void train_position_report(train_task_data_t *data)
{
	marklin_train_position_data_t position_data = {
		.train_id = data->train_id,
		.current_location = data->motion.current_position.sensor,
		.direction = data->motion.direction,
		.headlight = data->headlight,
		.current_speed = data->motion.commanded_speed,
		.destination = data->destination,
		.mode = data->operating_mode,
		.location_offset_mm = data->motion.current_position.offset_mm,
		.destination_offset_mm = data->destination_offset_mm,
		.status = train_get_external_status(data),
		.next_sensor_1 = data->motion.expected_sensor_count > 0 ? data->motion.expected_sensors[0] : NULL,
		.next_sensor_2 = data->motion.expected_sensor_count > 1 ? data->motion.expected_sensors[1] : NULL,
	};
	strncpy(position_data.destination_name, data->destination_name, 15);
	position_data.destination_name[15] = '\0';
	Marklin_MsgQueue_PublishTyped(MARKLIN_MSGQUEUE_EVENT_TYPE_TRAIN_POSITION, &position_data);
}

// ############################################################################
// # Main Autonomous Loop
// ############################################################################

static void train_autonomous_loop(train_task_data_t *data)
{
	for (;;) {
		// 1. Update state machine - process any pending events first
		train_state_machine_update(data);

		// 2. Update continuous position tracking
		train_update_current_position(data);

		// 2a. Ensure we always own the block we're currently in
		train_ensure_current_block_reserved(data);

		// 3. Poll sensor updates (non-blocking) and generate events
		if (data->sensor_subscription_active) {
			marklin_msgqueue_message_t message;
			marklin_error_t msg_result = Marklin_MsgQueue_ReceiveNonBlock(&message);

			if (msg_result == MARKLIN_ERROR_OK &&
			    message.event_type == MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE) {
				train_process_sensor_update(data, &message);
				// Generate sensor trigger event for state machine
				train_state_machine_process_event(data, TRAIN_EVENT_SENSOR_TRIGGERED);
			}
		}

		// 4. Update stopping distance
		train_update_stop_distance(data);

		// 5. Check stop conditions and generate state machine events
		train_check_and_generate_events(data);

		// 5.5. Check sensor timeout conditions
		train_check_sensor_timeouts(data);

		// 5.6. Check for blacklisted sensor arrivals and simulate if needed
		if (data->kinematic_model_enabled) {
			train_check_blacklisted_sensor_arrival(data);
		}

		// 6. Operating mode specific updates
		switch (data->operating_mode) {
		case TRAIN_MODE_WAYPOINT:
			train_waypoint_mode_update(data);
			break;
		case TRAIN_MODE_MANUAL:
		default:

			// Basic reservation for manual mode: reserve 1-2 blocks ahead for safety
			if (data->motion.current_position.sensor &&
			    data->motion.current_position.sensor->type == NODE_SENSOR) {
				// Calculate basic segments needed (1 for stopped, 2 for moving)
				u32 basic_segments = data->motion.commanded_speed > 0 ? 2 : 1;
				data->segments_needed_to_stop = basic_segments;

				log_debug("Train %d: Manual mode needs %d segments for safety", data->train_id,
					  basic_segments);
			}
			break;
		}

		// 7. Broadcast position
		train_position_report(data);

		// 8. Handle interactive commands (non-blocking receive)
		marklin_train_command_t command;
		int sender_tid;
		int result = ReceiveNonBlock(&sender_tid, (char *)&command, sizeof(command));
		if (result == sizeof(command)) {
			marklin_error_t cmd_result = MARKLIN_ERROR_OK;

			if (!train_is_command_valid_for_mode(data->operating_mode, command.command_type)) {
				log_info("Train %d: Invalid command %d for mode %d", data->train_id,
					 command.command_type, data->operating_mode);
				cmd_result = MARKLIN_ERROR_INVALID_ARGUMENT;
			} else {
				log_info("Train %d: Received command %d from %d", data->train_id, command.command_type,
					 sender_tid);
				switch (command.command_type) {
				case MARKLIN_TRAIN_CMD_SET_MODE:
					cmd_result = train_handle_mode_command(data, &command);
					break;

				case MARKLIN_TRAIN_CMD_MANUAL_SET_EFFECTIVE_SPEED:
				case MARKLIN_TRAIN_CMD_MANUAL_TOGGLE_HEADLIGHT:
				case MARKLIN_TRAIN_CMD_MANUAL_STOP:
					cmd_result = train_handle_manual_command(data, &command);
					break;

				case MARKLIN_TRAIN_CMD_MANUAL_REVERSE:
					// Route to appropriate handler based on mode
					if (data->operating_mode == TRAIN_MODE_MANUAL) {
						cmd_result = train_handle_manual_command(data, &command);
					} else if (data->operating_mode == TRAIN_MODE_WAYPOINT) {
						cmd_result = train_handle_waypoint_command(data, &command);
					}
					break;

				case MARKLIN_TRAIN_CMD_SET_REQUESTED_SPEED:
				case MARKLIN_TRAIN_CMD_SET_DESTINATION:
					cmd_result = train_handle_waypoint_command(data, &command);
					break;

				case MARKLIN_TRAIN_CMD_EMERGENCY_STOP:
					cmd_result = train_handle_emergency_command(data, &command);
					break;

				case MARKLIN_TRAIN_CMD_NAVIGATE_TO_DESTINATION:
					cmd_result = train_navigate_to_destination(
						data, command.navigate_to_destination.destination_name,
						command.navigate_to_destination.requested_speed);
					break;

				// Random destination mode command
				case MARKLIN_TRAIN_CMD_SET_RANDOM_DESTINATION_MODE:
					cmd_result = train_handle_waypoint_command(data, &command);
					break;

				// Debug command
				case MARKLIN_TRAIN_CMD_DEBUG_INFO:
					cmd_result = train_handle_debug_command(data, &command);
					break;

				// Clear destination command
				case MARKLIN_TRAIN_CMD_CLEAR_DESTINATION:
					cmd_result = train_clear_destination(data);
					break;

				default:
					cmd_result = MARKLIN_ERROR_INVALID_ARGUMENT;
					break;
				}
			}

			Reply(sender_tid, (const char *)&cmd_result, sizeof(cmd_result));
		}

		train_check_block_safety_conditions(data);

		Delay(data->clock_server_tid, 1); // 20ms loop interval
	}
}

// ############################################################################
// # Unified Kinematic Stopping System
// ############################################################################

// Update the current stopping distance (called once per loop)
void train_update_stop_distance(train_task_data_t *data)
{
	if (!data || !data->kinematic_model_enabled) {
		data->motion.current_stop_distance = 0;
		return;
	}

	kinematic_time_t current_time = Time(data->clock_server_tid);

	// Only update if enough time has passed or speed changed
	if (current_time - data->motion.last_stop_distance_update < 2) { // 20ms update interval
		return;
	}

	data->motion.current_stop_distance = kinematic_model_get_stop_distance(
		data, data->motion.commanded_speed, data->motion.commanded_speed_from_higher);

	data->motion.last_stop_distance_update = current_time;
}

// Check for collision risks by examining block ownership ahead of train
static void train_check_block_safety_conditions(train_task_data_t *data)
{
	if (!data || !data->motion.current_position.sensor) {
		return;
	}

	// kinematic_distance_t safety_distance = TRAIN_BLOCK_SAFETY_MARGIN_MM;

	// Check ownership of next expected sensors within safety distance
	for (int i = 0; i < 1 && data->motion.expected_sensors[i]; i++) {
		const track_node *next_sensor = data->motion.expected_sensors[i];
		// kinematic_distance_t distance_to_sensor = data->motion.expected_distances[i];

		// If we're close enough to this sensor to be concerned about collision
		// if (distance_to_sensor > 0 && distance_to_sensor <= safety_distance) {
		bool owns_block = false;
		u8 owner_train_id = 0;

		marklin_error_t result =
			Marklin_CheckBlockOwnership(data->train_id, next_sensor, &owns_block, &owner_train_id);
		// log_error("Checking block ownership for train %d at sensor %s: owns_block=%d, owner_train_id=%d",
		//   data->train_id, next_sensor->name, owns_block, owner_train_id);

		if (result == MARKLIN_ERROR_OK && !owns_block &&
		    data->state_machine.movement_state != MOVEMENT_STATE_STATIONARY) {
			// Block ahead is not owned by us - potential collision!
			// log_error("COLLISION AVOIDANCE: Train %d approaching unreserved block at sensor %s "
			// 	  "(distance: %lldmm, safety_distance: %lldmm, owned by train %d) - FORCE STOP",
			// 	  data->train_id, next_sensor->name, distance_to_sensor, safety_distance,
			// 	  owner_train_id);
			// train_force_stop(data);
			return;
			// }
		}
	}
}

// Unified function to check all stop conditions
train_stop_action_t train_check_unified_stop_conditions(train_task_data_t *data)
{
	static int count = 0;
	count++;

	if (!data || data->motion.commanded_speed == 0) {
		return TRAIN_CONTINUE; // Already stopped
	}

	// 0. Check collision avoidance conditions first (highest priority for safety)
	train_check_block_safety_conditions(data);

	// 1. Check low speed mode timer expiration (high priority for precision navigation)
	if (data->low_speed_mode_active && train_check_low_speed_timer(data)) {
		log_info("Train %d: Low speed mode timer expired - stopping with force stop", data->train_id);
		return TRAIN_STOP_LOW_SPEED_TIMER;
	}

	// 2. Check collision avoidance conditions (high priority)

	// 3. Check path activation end conditions (for partial path activation)
	if (data->state_machine.path_state == PATH_STATE_ACTIVE && data->activation_end_point) {
		// Check if we are approaching the end of the activated path segment
		if (data->motion.current_position.sensor == data->activation_end_point) {
			log_info("Train %d: Reached end of activated path segment at %s", data->train_id,
				 data->activation_end_point->name);
			if (data->activation_end_point == data->destination) {
				return TRAIN_STOP_DESTINATION;
			} else {
				return TRAIN_STOP_PATH_END;
			}
		}

		// Check if we are close to the end point based on expected distance
		if (data->motion.expected_sensors[0] == data->activation_end_point &&
		    data->motion.expected_distances[0] > 0 && data->motion.current_stop_distance > 0) {
			if (data->motion.expected_distances[0] <= data->motion.current_stop_distance) {
				log_info("Train %d: Approaching end of activated path segment at %s (distance: %lldmm)",
					 data->train_id, data->activation_end_point->name,
					 data->motion.expected_distances[0]);
				return TRAIN_STOP_PATH_END;
			}
		}
	}

	// 1.7. Check minimum speed mode conditions (for short paths and activated segments)
	// If we're in waypoint mode and using minimum speed due to short path or activated segment,
	// trigger emergency stop when approaching destination or activation end point
	// if (data->operating_mode == TRAIN_MODE_WAYPOINT && data->state_machine.path_state == PATH_STATE_ACTIVE &&
	//     data->has_active_path && data->current_path.total_distance > 0) {
	// 	// PRIORITY A: Check for minimum speed mode due to short activated path segment
	// 	if (data->activation_end_point && data->activation_end_point != data->destination) {
	// 		train_position_t current_pos = data->motion.current_position;
	// 		train_position_t activation_end_pos = { .sensor = data->activation_end_point, .offset_mm = 0 };
	// 		kinematic_distance_t distance_to_activation_end =
	// 			train_position_distance_between(&current_pos, &activation_end_pos, true);

	// 		// Check if we're in minimum speed mode for activated segment
	// 		if (data->motion.current_stop_distance > 0 && distance_to_activation_end > 0 &&
	// 		    data->motion.current_stop_distance > distance_to_activation_end) {
	// 			// Treat very close approach to activation end as path end reached
	// 			if (distance_to_activation_end <= TRAIN_EMERGENCY_STOP_THRESHOLD_MM) {
	// 				log_info(
	// 					"Train %d: Very close to activation end %s (%lldmm) - treating as path end reached",
	// 					data->train_id, data->activation_end_point->name,
	// 					distance_to_activation_end);
	// 				return TRAIN_STOP_PATH_END;
	// 			}
	// 		}
	// 	}

	// 	// PRIORITY B: Check for minimum speed mode due to short full path (existing logic)
	// 	if (data->destination) {
	// 		// Check if we're likely in minimum speed mode (stop distance > full path distance)
	// 		if (data->motion.current_stop_distance > 0 &&
	// 		    data->motion.current_stop_distance > data->current_path.total_distance) {
	// 			// Calculate distance to destination
	// 			train_position_t current_pos = data->motion.current_position;
	// 			kinematic_distance_t compensated_offset = train_calculate_stopping_offset(
	// 				data, data->destination_offset_mm, data->motion.direction);
	// 			train_position_t target_pos = { .sensor = data->destination,
	// 							.offset_mm = compensated_offset };
	// 			kinematic_distance_t distance_to_target =
	// 				train_position_distance_between(&current_pos, &target_pos, true);

	// 			// Treat very close approach to destination as destination reached
	// 			if (distance_to_target > 0 && distance_to_target <= TRAIN_EMERGENCY_STOP_THRESHOLD_MM) {
	// 				log_info(
	// 					"Train %d: Very close to destination %s (%lldmm) - treating as destination reached",
	// 					data->train_id,
	// 					data->destination->name ? data->destination->name : "unknown",
	// 					distance_to_target);
	// 				return TRAIN_STOP_DESTINATION;
	// 			}
	// 		}
	// 	}
	// }

	// 2. Check destination stopping conditions
	if (data->destination && data->motion.current_position.sensor) {
		// Calculate distance to destination target
		train_position_t current_pos = data->motion.current_position;
		kinematic_distance_t compensated_offset =
			train_calculate_stopping_offset(data, data->destination_offset_mm, data->motion.direction);
		train_position_t target_pos = { .sensor = data->destination, .offset_mm = compensated_offset };

		kinematic_distance_t distance_to_target =
			train_position_distance_between(&current_pos, &target_pos, true);
		if (count % 10 == 0) {
			train_report(data);
			// log_info("Train %d: Distance to target: %d", data->train_id, distance_to_target);
			// log_info("Train %d: target: %s:%d", data->train_id, target_pos.sensor->name,
			//  target_pos.offset_mm);
		}

		if (data->motion.current_stop_distance > 0) {
			if (distance_to_target > 0 && distance_to_target <= data->motion.current_stop_distance) {
				log_info(
					"Train %d: Very close to destination %s (%lldmm) - treating as destination reached",
					data->train_id, data->destination->name ? data->destination->name : "unknown",
					distance_to_target);
				return TRAIN_STOP_DESTINATION;
			}
		} else {
			if (distance_to_target > 0 && distance_to_target <= TRAIN_EMERGENCY_STOP_THRESHOLD_MM &&
			    data->state_machine.movement_state != MOVEMENT_STATE_STATIONARY) {
				log_info(
					"Train %d: Very close to destination %s (%lldmm) without stop distance data - treating as destination reached",
					data->train_id, data->destination->name ? data->destination->name : "unknown",
					distance_to_target);
				train_force_stop(data);
				return TRAIN_STOP_DESTINATION;
			}
		}

		// Check if we've arrived at destination (for both cases)
		if (train_position_is_at_destination(&current_pos, &target_pos, 100)) {
			log_info("Train %d: Arrived at destination %s", data->train_id,
				 data->destination->name ? data->destination->name : "unknown");
			return TRAIN_STOP_DESTINATION;
		}
	}

	// 3. Check signal-based speed reduction (lower priority)
	// if (data->signal_context.has_signal && data->signal_context.distance_valid) {
	// 	signal_state_t signal_state = data->signal_context.current_signal.state;
	// 	if (signal_state == SIGNAL_YELLOW && data->motion.current_stop_distance > 0) {
	// 		kinematic_distance_t approach_distance =
	// 			data->motion.current_stop_distance * TRAIN_SIGNAL_APPROACH_THRESHOLD_PERCENT / 100;

	// 		if (approach_distance >= data->signal_context.distance_to_signal) {
	// 			return TRAIN_STOP_SIGNAL;
	// 		}
	// 	}
	// }

	return TRAIN_CONTINUE;
}

// Execute the stop action determined by unified check
void train_execute_stop_action(train_task_data_t *data, train_stop_action_t action)
{
	switch (action) {
	case TRAIN_CONTINUE:
		return; // No action needed

	case TRAIN_STOP_DESTINATION:
		log_info("Train %d: Stopping for destination %s", data->train_id,
			 data->destination ? data->destination->name : "unknown");
		train_stop(data);
		// Destination clearing and path state reset will be handled by kinematic stop completion
		break;

	case TRAIN_STOP_PATH_END:
		log_info("Train %d: Stopping at end of activated path segment", data->train_id);
		train_stop(data);
		// Set state so the state machine will trigger path continuation
		data->state_machine.path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = true;
		break;

	case TRAIN_STOP_REVERSAL:
		// This case should no longer occur with simplified logic
		log_warn("Train %d: Unexpected TRAIN_STOP_REVERSAL - treating as path end", data->train_id);
		train_stop(data);
		data->state_machine.path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = true;
		break;

	case TRAIN_STOP_LOW_SPEED_TIMER:
		log_info("Train %d: Low speed mode timer expired - executing force stop", data->train_id);
		train_force_stop(data); // Use force stop for instant deceleration
		train_stop_low_speed_mode(data); // Clear low speed mode state
		// Check if we've reached destination
		if (data->destination && data->motion.current_position.sensor == data->destination) {
			// We've reached the destination - clear it
			data->state_machine.path_state = PATH_STATE_REACHED;
		} else {
			// Partial progress - may need path continuation
			data->state_machine.path_state = PATH_STATE_ACTIVE;
			data->needs_path_continuation = true;
		}
		break;

	case TRAIN_EMERGENCY_STOP:
		log_warn("Train %d: Emergency stop triggered - safety violation", data->train_id);
		train_emergency_stop(data);
		// Status now handled by state machine
		break;
	}
}

// ############################################################################
// # Train Movement Control Functions
// ############################################################################

marklin_error_t train_set_speed_and_headlight(train_task_data_t *data, u8 speed, marklin_train_headlight_t headlight)
{
	u8 cmd = 0;
	u8 param = data->train_id;

	if (speed > MARKLIN_TRAIN_MAX_SPEED) {
		speed = MARKLIN_TRAIN_MAX_SPEED;
	}

	if (data->motion.commanded_speed == speed && data->headlight == headlight) {
		return MARKLIN_ERROR_OK;
	}

	cmd = speed;

	if (headlight == MARKLIN_TRAIN_HEADLIGHT_AUTO) {
		headlight = data->headlight;
	}

	if (headlight == MARKLIN_TRAIN_HEADLIGHT_ON) {
		cmd += MARKLIN_HEADLIGHT_ON_CMD;
	}

	marklin_error_t result = Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, cmd, param,
								     MARKLIN_TRAIN_CMD_DELAY_TICKS,
								     MARKLIN_CMD_PRIORITY_HIGH, data->train_id);
	log_info("Train %d: Set speed to %d with headlight %d", data->train_id, speed, headlight);
	if (result == MARKLIN_ERROR_OK) {
		data->motion.commanded_speed = speed;
		data->headlight = headlight;

		kinematic_time_t current_time = Time(data->clock_server_tid);

		data->motion.speed_change_time = current_time;
	}

	return result;
}

marklin_error_t train_set_speed(train_task_data_t *data, u8 speed)
{
	return train_set_speed_and_headlight(data, speed, data->headlight);
}

marklin_error_t train_set_headlight(train_task_data_t *data, marklin_train_headlight_t headlight)
{
	return train_set_speed_and_headlight(data, data->motion.commanded_speed, headlight);
}

marklin_error_t train_toggle_headlight(train_task_data_t *data)
{
	return train_set_speed_and_headlight(data, data->motion.commanded_speed, !data->headlight);
}

marklin_error_t train_reverse(train_task_data_t *data)
{
	log_info("Train %d: Executing reversal at position %s (offset: %d mm)", data->train_id,
		 data->motion.current_position.sensor ? data->motion.current_position.sensor->name : "unknown",
		 data->motion.current_position.offset_mm);

	marklin_error_t result = Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM,
								     MARKLIN_REVERSE_CMD + 16, data->train_id,
								     MARKLIN_TRAIN_CMD_DELAY_TICKS,
								     MARKLIN_CMD_PRIORITY_HIGH, data->train_id);
	if (result == MARKLIN_ERROR_OK) {
		data->motion.commanded_speed = 0;
		data->motion.direction = TRAIN_DIRECTION_REVERSE;

		// Update position to reverse node if it exists
		if (data->motion.current_position.sensor && data->motion.current_position.sensor->reverse) {
			data->motion.current_position.sensor = data->motion.current_position.sensor->reverse;
			data->motion.current_position.offset_mm = 0;
			log_info("Train %d: Updated position to reverse node %s after reversing", data->train_id,
				 data->motion.current_position.sensor->name);
		} else {
			log_warn("Train %d: No reverse node available for current position", data->train_id);
		}

		train_calculate_next_sensors(data);

		train_release_all_blocks(data, true);

		log_info("Train %d: Reversal completed, direction now: %s", data->train_id,
			 data->motion.direction == TRAIN_DIRECTION_FORWARD ? "FORWARD" : "REVERSE");
	} else {
		log_error("Train %d: Failed to execute reversal command: %d", data->train_id, result);
	}

	return result;
}

marklin_error_t train_stop(train_task_data_t *data)
{
	u8 current_speed = data->motion.commanded_speed;
	if (current_speed == 0) {
		return MARKLIN_ERROR_OK;
	}
	data->motion.commanded_speed_from_higher = false;
	marklin_error_t result = train_set_speed(data, 0);
	if (result != MARKLIN_ERROR_OK) {
		return result;
	}

	// Use kinematic model for stopping time if available
	kinematic_time_t kinematic_stop_time = 0;
	if (data->kinematic_model_enabled && current_speed > 0) {
		kinematic_stop_time =
			kinematic_model_get_stop_time(data, current_speed, data->motion.commanded_speed_from_higher);
	}

	// Convert kinematic time to milliseconds for delay
	u32 stop_time_ms;
	if (kinematic_stop_time > 0) {
		stop_time_ms = kinematic_ticks_to_ms(kinematic_stop_time);
		log_debug("Train %d: Using kinematic stopping time: %u ms for speed %d", data->train_id, stop_time_ms,
			  current_speed);
	} else {
		// Fallback with speed-based calculation for safety
		stop_time_ms = TRAIN_FALLBACK_STOP_TIME_MS;
		log_debug("Train %d: Using fallback stopping time: %u ms for speed %d (kinematic model unavailable)",
			  data->train_id, stop_time_ms, current_speed);
	}

	// Before stopping, try to reserve the stopping path for safety
	if (data->motion.current_stop_distance > 0) {
		u32 blocks_needed = train_calculate_blocks_needed_for_stopping(data);
		bool reservation_success = train_try_reserve_stopping_path(data, blocks_needed);

		if (reservation_success) {
			log_info("Train %d: Secured %d blocks for stopping path", data->train_id, blocks_needed);
		} else {
			log_warn("Train %d: Could not fully secure stopping path, proceeding with caution",
				 data->train_id);
		}
	}

	// Block release will be handled by unified cleanup when train transitions to IDLE
	Delay(data->clock_server_tid, MS_TO_TICK(stop_time_ms));

	return MARKLIN_ERROR_OK;
}

marklin_error_t train_force_stop(train_task_data_t *data)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_error_t result = Marklin_ScheduleCommandBlockingWithPriority(
		MARKLIN_CMD_TYPE_WITH_PARAM, MARKLIN_REVERSE_CMD + 16, data->train_id, MARKLIN_TRAIN_CMD_DELAY_TICKS,
		MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);
	result = Marklin_ScheduleCommandBlockingWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, 1 + 16, data->train_id,
							     MARKLIN_TRAIN_CMD_DELAY_TICKS,
							     MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);
	result = Marklin_ScheduleCommandBlockingWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, 0 + 16, data->train_id,
							     MARKLIN_TRAIN_CMD_DELAY_TICKS,
							     MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);
	result = Marklin_ScheduleCommandBlockingWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, MARKLIN_REVERSE_CMD + 16,
							     data->train_id, MARKLIN_TRAIN_CMD_DELAY_TICKS,
							     MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);

	// Set immediate stop flags
	data->motion.commanded_speed = 0;
	data->motion.commanded_speed_from_higher = false;
	data->state_machine.movement_state = MOVEMENT_STATE_STATIONARY;

	// Block release will be handled by unified cleanup when transitioning to IDLE

	return result;
}

marklin_error_t train_emergency_stop(train_task_data_t *data)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	log_warn("Train %d: EMERGENCY STOP activated at position %s (speed: %d)", data->train_id,
		 data->motion.current_position.sensor ? data->motion.current_position.sensor->name : "unknown",
		 data->motion.commanded_speed);

	// Use reverse command with CRITICAL priority for immediate hard stop
	marklin_error_t result = Marklin_ScheduleCommandBlockingWithPriority(
		MARKLIN_CMD_TYPE_WITH_PARAM, MARKLIN_REVERSE_CMD + 16, data->train_id, MARKLIN_TRAIN_CMD_DELAY_TICKS,
		MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);
	result = Marklin_ScheduleCommandBlockingWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, 1 + 16, data->train_id,
							     MARKLIN_TRAIN_CMD_DELAY_TICKS,
							     MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);
	result = Marklin_ScheduleCommandBlockingWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, 0 + 16, data->train_id,
							     MARKLIN_TRAIN_CMD_DELAY_TICKS,
							     MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);
	result = Marklin_ScheduleCommandBlockingWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, MARKLIN_REVERSE_CMD + 16,
							     data->train_id, 0, MARKLIN_CMD_PRIORITY_CRITICAL,
							     data->train_id);

	// Set immediate stop flags
	data->motion.commanded_speed = 0;
	data->motion.requested_speed = 0;
	data->motion.commanded_speed_from_higher = false;

	// Clear destination for emergency stops (not force stops)
	if (data->destination) {
		log_info("Train %d: Emergency stop - clearing destination %s", data->train_id, data->destination_name);
		data->destination = NULL;
		data->destination_name[0] = '\0';
		data->destination_offset_mm = 0;
	}

	// Block release and path cleanup will be handled by unified cleanup in train_state_idle_entry

	if (result == MARKLIN_ERROR_OK) {
		log_info("Train %d: Emergency stop command issued with CRITICAL priority", data->train_id);
	} else {
		log_error("Train %d: Failed to issue emergency stop command: %d", data->train_id, result);
	}

	return result;
}

marklin_error_t train_reverse_and_continue(train_task_data_t *data)
{
	marklin_error_t result = MARKLIN_ERROR_OK;
	int current_speed = data->motion.commanded_speed;

	if (current_speed == 0) {
		return train_reverse(data);
	}

	result = train_stop(data);
	if (result != MARKLIN_ERROR_OK) {
		return result;
	}

	result = train_reverse(data);
	if (result != MARKLIN_ERROR_OK) {
		return result;
	}

	result = train_set_speed(data, current_speed);
	return result;
}

marklin_error_t train_set_destination(train_task_data_t *data, const struct marklin_destination_cmd *dest_cmd)
{
	if (!data || !dest_cmd) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	const track_node *destination_node =
		marklin_find_node_by_name(track_nodes, track_nodes_size, dest_cmd->destination_name);

	if (!destination_node) {
		log_error("Train %d: Destination sensor '%s' not found", data->train_id, dest_cmd->destination_name);
		return MARKLIN_ERROR_NOT_FOUND;
	}

	// Store destination in train data
	data->destination = destination_node;
	strncpy(data->destination_name, dest_cmd->destination_name, 15);
	data->destination_name[15] = '\0';
	data->destination_offset_mm = dest_cmd->offset_mm;
	data->state_machine.path_state = PATH_STATE_NONE;

	// Use centralized path management
	return MARKLIN_ERROR_OK;
}

marklin_error_t train_clear_destination(train_task_data_t *data)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Clear destination if one exists
	if (data->destination) {
		log_info("Train %d: Clearing destination %s", data->train_id, data->destination_name);
		data->destination = NULL;
		data->destination_name[0] = '\0';
		data->destination_offset_mm = 0;
	} else {
		log_info("Train %d: No destination to clear", data->train_id);
	}

	// Reset path state to NONE
	data->state_machine.path_state = PATH_STATE_NONE;

	// Clear any active path
	if (data->has_active_path) {
		data->has_active_path = false;
		data->path_ends_at_reversal = false;
		data->needs_path_continuation = false;
		data->at_reversal_point = false;
		Marklin_FreePath(&data->current_path);
		log_debug("Train %d: Cleared active path", data->train_id);
	}

	// If train is currently moving to a destination, stop it and transition to IDLE
	if (data->state_machine.current_state == TRAIN_STATE_MOVING) {
		log_info("Train %d: Stopping train due to destination clear", data->train_id);
		train_stop(data);
		train_state_machine_transition(data, TRAIN_STATE_IDLE);
	}

	return MARKLIN_ERROR_OK;
}

marklin_error_t train_navigate_to_destination(train_task_data_t *data, const char *destination_name, u8 requested_speed)
{
	if (!data || !destination_name) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Validate requested speed
	if (requested_speed > MARKLIN_TRAIN_MAX_SPEED) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Find destination node by name
	const track_node *destination_node = marklin_find_node_by_name(track_nodes, track_nodes_size, destination_name);

	if (!destination_node) {
		log_error("Train %d: Destination '%s' not found", data->train_id, destination_name);
		return MARKLIN_ERROR_NOT_FOUND;
	}

	// Switch to waypoint mode if not already in it
	if (data->operating_mode != TRAIN_MODE_WAYPOINT) {
		data->operating_mode = TRAIN_MODE_WAYPOINT;
		log_info("Train %d: Switched to waypoint mode for navigation", data->train_id);
	}

	// Set requested speed
	data->motion.requested_speed = requested_speed;
	log_info("Train %d: Set requested speed to %d", data->train_id, requested_speed);

	// Set destination in train data
	data->destination = destination_node;
	strncpy(data->destination_name, destination_name, 15);
	data->destination_name[15] = '\0';

	data->state_machine.path_state = PATH_STATE_NONE;

	// Trigger effective speed update to generate appropriate state machine events
	// This ensures the train transitions to MOVING state if requested_speed > 0
	train_update_effective_speed(data);

	return MARKLIN_ERROR_OK;
}

// ############################################################################
// # Sensor Tracking Functions
// ############################################################################

static void train_calculate_next_sensors(train_task_data_t *data)
{
	if (!data->motion.current_position.sensor) {
		data->motion.expected_sensors[0] = NULL;
		data->motion.expected_sensors[1] = NULL;
		data->motion.expected_distances[0] = 0;
		data->motion.expected_distances[1] = 0;
		data->motion.expected_arrival_times[0] = 0;
		data->motion.expected_arrival_times[1] = 0;
		data->motion.sensor_timeout_deadlines[0] = 0;
		data->motion.sensor_timeout_deadlines[1] = 0;
		data->motion.sensor_timeout_logged[0] = false;
		data->motion.sensor_timeout_logged[1] = false;
		data->motion.expected_sensor_count = 0;
		return;
	}

	// Query the conductor for next two sensors (it knows switch states)
	marklin_error_t result = Marklin_GetNextTwoSensors(data->motion.current_position.sensor,
							   TRAIN_DIRECTION_FORWARD, data->motion.expected_sensors,
							   data->motion.expected_distances,
							   &data->motion.expected_sensor_count);

	if (result != MARKLIN_ERROR_OK) {
		log_error("Train %d: Failed to get next sensors from conductor: %d", data->train_id, result);
		data->motion.expected_sensors[0] = NULL;
		data->motion.expected_sensors[1] = NULL;
		data->motion.expected_distances[0] = 0;
		data->motion.expected_distances[1] = 0;
		data->motion.expected_arrival_times[0] = 0;
		data->motion.expected_arrival_times[1] = 0;
		data->motion.sensor_timeout_deadlines[0] = 0;
		data->motion.sensor_timeout_deadlines[1] = 0;
		data->motion.sensor_timeout_logged[0] = false;
		data->motion.sensor_timeout_logged[1] = false;
		data->motion.expected_sensor_count = 0;
	} else {
		// Calculate expected arrival times and timeout deadlines for each sensor
		kinematic_time_t current_time = Time(data->clock_server_tid);
		kinematic_velocity_t current_velocity = kinematic_model_get_velocity(
			data, data->motion.commanded_speed, data->motion.commanded_speed_from_higher);

		for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
			if (data->motion.commanded_speed == 0) {
				data->motion.expected_arrival_times[i] = 0;
				data->motion.sensor_timeout_deadlines[i] = 0;
				data->motion.sensor_timeout_logged[i] = false;
				continue;
			}

			if (data->motion.expected_distances[i] > 0 && current_velocity > 0) {
				kinematic_time_t travel_time = kinematic_time_for_distance(
					data->motion.expected_distances[i] - data->motion.current_position.offset_mm,
					current_velocity);

				data->motion.expected_arrival_times[i] = current_time + travel_time;

				// Set timeout deadline with 150% grace period, minimum 2 seconds
				kinematic_time_t grace_period = travel_time / 2; // 50% additional time
				kinematic_time_t min_timeout = kinematic_ms_to_ticks(4000); // 2 seconds minimum
				if (grace_period < min_timeout) {
					grace_period = min_timeout;
				}

				data->motion.sensor_timeout_deadlines[i] =
					data->motion.expected_arrival_times[i] + grace_period;
				data->motion.sensor_timeout_logged[i] = false;
			} else {
				data->motion.expected_arrival_times[i] = 0;
				data->motion.sensor_timeout_deadlines[i] = 0;
				data->motion.sensor_timeout_logged[i] = false;
			}
		}

		log_debug("Train %d: Expected sensors (%d): %s, %s", data->train_id, data->motion.expected_sensor_count,
			  data->motion.expected_sensors[0] ? data->motion.expected_sensors[0]->name : "none",
			  data->motion.expected_sensors[1] ? data->motion.expected_sensors[1]->name : "none");

		// Log if any expected sensors are blacklisted
		for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
			if (data->motion.expected_sensors[i] &&
			    train_is_sensor_blacklisted(data, data->motion.expected_sensors[i])) {
				log_info(
					"Train %d: Expected sensor %s is blacklisted - relying on kinematic positioning",
					data->train_id, data->motion.expected_sensors[i]->name);
			}
		}
	}
}

bool train_is_sensor_blacklisted(train_task_data_t *data, const track_node *sensor_node)
{
	if (!data || !sensor_node) {
		return false;
	}

	if (!sensor_node->name || strlen(sensor_node->name) < 2) {
		return false;
	}

	u8 bank = marklin_parse_sensor_bank_from_name(sensor_node->name);
	u8 sensor_id = marklin_parse_sensor_id_from_name(sensor_node->name);

	if (bank == 0xff || sensor_id == 0xff) {
		return false;
	}

	return data->sensor_blacklist_cache[bank][sensor_id - 1];
}

static void train_check_blacklisted_sensor_arrival(train_task_data_t *data)
{
	if (!data || data->motion.expected_sensor_count == 0 || !data->motion.expected_sensors[0]) {
		return;
	}

	if (!train_is_sensor_blacklisted(data, data->motion.expected_sensors[0])) {
		return;
	}

	if (data->motion.current_position.offset_mm < data->motion.expected_distances[0]) {
		return;
	}

	// we have passed the last sensor, so we can simulate a sensor trigger
	kinematic_time_t current_time = Time(data->clock_server_tid);

	log_info("Train %d: Simulating sensor trigger for blacklisted sensor %s", data->train_id,
		 data->motion.expected_sensors[0]->name);

	// Create a mock sensor update for the simulated trigger
	marklin_sensor_state_t mock_sensor_update = { .bank = 0, // Not used for position update
						      .sensor_id = 0, // Not used for position update
						      .triggered = 1,
						      .last_triggered_tick = current_time };
	train_update_position_from_sensor(data, data->motion.expected_sensors[0], &mock_sensor_update);

	if (data->kinematic_model_enabled) {
		data->last_sensor_trigger_tick = current_time;
	}
}

static bool train_is_sensor_expected(train_task_data_t *data, const track_node *sensor_node)
{
	if (!sensor_node) {
		return false;
	}

	// Check if sensor matches any of the expected sensors
	for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
		if (sensor_node == data->motion.expected_sensors[i]) {
			return true;
		}
	}

	return false;
}

/**
 * Enhanced sensor validation that checks state machine, expected sensors, and block ownership.
 * Only processes sensors when train is in MOVING state (including kinematic stopping).
 * This prevents trains from processing sensor updates when stopped or in emergency states.
 *
 * @param data Train task data
 * @param sensor_node The sensor node to validate
 * @param sensor_update The sensor update containing hardware trigger timestamp
 * @return true if train should process this sensor update, false otherwise
 */
static bool train_should_process_sensor_update(train_task_data_t *data, const track_node *sensor_node,
					       const marklin_sensor_state_t *sensor_update)
{
	if (!data || !sensor_node) {
		return false;
	}

	if (sensor_node != data->motion.expected_sensors[0] && sensor_node != data->motion.expected_sensors[1]) {
		return false;
	}

	// Check if train is currently in MOVING state OR if sensor was triggered while in MOVING state
	bool currently_moving = (data->state_machine.current_state == TRAIN_STATE_MOVING);
	bool triggered_while_moving = false;

	if (!currently_moving && data->state_machine.last_moving_exit_time_tick > 0) {
		// Check if sensor was triggered before we exited MOVING state
		u64 sensor_trigger_time_tick = sensor_update->last_triggered_tick;
		triggered_while_moving = (sensor_trigger_time_tick <= data->state_machine.last_moving_exit_time_tick);

		if (triggered_while_moving) {
			log_info(
				"Train %d: Processing delayed sensor %s triggered @%llu tick while in MOVING state (exited @%llu tick, current: %s)",
				data->train_id, sensor_node->name, sensor_trigger_time_tick,
				data->state_machine.last_moving_exit_time_tick,
				train_state_name(data->state_machine.current_state));
		}
	}

	if (!currently_moving && !triggered_while_moving) {
		log_error(
			"Train %d: Ignoring sensor %s triggered @%llu - not in MOVING state and not triggered while moving (current: %s, last exit: %llu)",
			data->train_id, sensor_node->name, sensor_update->last_triggered_tick,
			train_state_name(data->state_machine.current_state),
			data->state_machine.last_moving_exit_time_tick);
		return false;
	}

	// First check if sensor is expected (existing validation)
	bool is_expected = false;
	for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
		if (sensor_node == data->motion.expected_sensors[i]) {
			is_expected = true;
			break;
		}
	}

	if (!is_expected) {
		return false;
	}

	// Now check block ownership validation using dedicated API
	bool owns_sensor_block = false;
	u8 owner_train_id = 0;
	marklin_error_t ownership_result =
		Marklin_CheckBlockOwnership(data->train_id, sensor_node, &owns_sensor_block, &owner_train_id);

	if (ownership_result != MARKLIN_ERROR_OK) {
		// If ownership check fails, allow the sensor update to maintain compatibility
		// This handles cases where sensors might not be properly assigned to blocks
		log_info("Train %d: Block ownership check failed for sensor %s (error: %d), allowing update",
			 data->train_id, sensor_node->name, ownership_result);
		return true;
	}

	// Check if this train owns the block containing the sensor
	if (owns_sensor_block) {
		// Block ownership validated, now check timing to detect early triggers
		// This prevents processing sensors triggered by other trains before block transfer

		// Find which expected sensor this corresponds to for timing validation
		int sensor_index = -1;
		for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
			if (data->motion.expected_sensors[i] == sensor_node) {
				sensor_index = i;
				break;
			}
		}

		if (sensor_index >= 0) {
			kinematic_time_t trigger_time = sensor_update->last_triggered_tick;
			kinematic_time_t expected_time = data->motion.expected_arrival_times[sensor_index];

			// Define timing tolerance - sensors can trigger up to 2 seconds early due to various factors
			// but significantly earlier triggers are likely from another train
			const kinematic_time_t EARLY_TRIGGER_TOLERANCE_TICK = 500;

			// Check if sensor triggered too early (beyond reasonable tolerance)
			if (expected_time > 0 && trigger_time > 0) {
				log_info("Train %d: Sensor %s expected at %lldtick, trigger time %lldticj",
					 data->train_id, sensor_node->name, expected_time, trigger_time);
				if (trigger_time < (expected_time - EARLY_TRIGGER_TOLERANCE_TICK)) {
					kinematic_time_t early_by =
						(expected_time - EARLY_TRIGGER_TOLERANCE_TICK) - trigger_time;
					log_info(
						"Train %d: Ignoring early sensor %s trigger (trigger: %lldms, expected: %lldms, early by: %lldms)",
						data->train_id, sensor_node->name, trigger_time, expected_time,
						early_by);
					return false;
				}

				// Additional physical possibility check
				if (data->motion.last_position_update > 0 &&
				    trigger_time > data->motion.last_position_update) {
					kinematic_time_t time_since_last_update =
						trigger_time - data->motion.last_position_update;
					kinematic_distance_t expected_distance =
						data->motion.expected_distances[sensor_index];

					// Calculate maximum possible distance train could have traveled since last update
					// Assume maximum speed of 14 with generous acceleration allowance
					const kinematic_velocity_t MAX_POSSIBLE_VELOCITY =
						150000; // ~15 mm/tick in kinematic units
					kinematic_distance_t max_possible_distance =
						(MAX_POSSIBLE_VELOCITY * time_since_last_update) / 1000;

					// If expected distance is much larger than physically possible, reject
					if (expected_distance > 0 && expected_distance > (max_possible_distance * 2)) {
						log_error(
							"Train %d: Ignoring physically impossible sensor %s trigger (expected distance: %lldmm, max possible: %lldmm, time: %lldms)",
							data->train_id, sensor_node->name, expected_distance,
							max_possible_distance, time_since_last_update);
						return false;
					}
				}
			}
		}

		// Timing and physical validation passed or not applicable, allow processing
		return true;
	}

	// Special case: Allow processing if train has any block reservations
	// This handles transitions between blocks where sensors might trigger slightly early/late
	if (data->reserved_block_count > 0) {
		// Apply timing validation for boundary sensors
		int sensor_index = -1;
		for (u8 j = 0; j < data->motion.expected_sensor_count; j++) {
			if (data->motion.expected_sensors[j] == sensor_node) {
				sensor_index = j;
				break;
			}
		}

		if (sensor_index >= 0) {
			kinematic_time_t trigger_time = sensor_update->last_triggered_tick;
			kinematic_time_t expected_time = data->motion.expected_arrival_times[sensor_index];

			// Use slightly more lenient tolerance for boundary sensors (3 seconds)
			// since they can have more timing variation during block transitions
			const kinematic_time_t BOUNDARY_EARLY_TOLERANCE_MS = 3000;

			if (expected_time > 0 && trigger_time > 0) {
				if (trigger_time < (expected_time - BOUNDARY_EARLY_TOLERANCE_MS)) {
					kinematic_time_t early_by =
						(expected_time - BOUNDARY_EARLY_TOLERANCE_MS) - trigger_time;
					log_info(
						"Train %d: Ignoring early boundary sensor %s trigger (trigger: %lldms, expected: %lldms, early by: %lldms)",
						data->train_id, sensor_node->name, trigger_time, expected_time,
						early_by);
					return false;
				}
			}
		}

		log_info("Train %d: Allowing boundary sensor %s (train has block reservations)", data->train_id,
			 sensor_node->name);
		return true;
	}

	// Block is not owned by this train - reject the sensor update
	if (owner_train_id == 0) {
		log_info("Train %d: Ignoring sensor %s from unowned block", data->train_id,
			 sensor_node->name ? sensor_node->name : "unnamed");
	} else {
		log_info("Train %d: Ignoring sensor %s from block owned by train %d", data->train_id,
			 sensor_node->name ? sensor_node->name : "unnamed", owner_train_id);
	}
	return false;
}

static void train_update_position_from_sensor(train_task_data_t *data, const track_node *sensor_node,
					      const marklin_sensor_state_t *sensor_update)
{
	if (!sensor_node) {
		return;
	}

	// Check if this is one of our expected sensors
	log_info("Train %d: Sensor %s triggered at position %s (offset: %lldmm)", data->train_id,
		 sensor_node->name ? sensor_node->name : "unnamed",
		 data->motion.current_position.sensor ? data->motion.current_position.sensor->name : "unknown",
		 data->motion.current_position.offset_mm);

	if (!train_is_sensor_expected(data, sensor_node)) {
		log_warn("Train %d: Sensor %s triggered but not expected - ignoring", data->train_id,
			 sensor_node->name);
		return;
	}

	// Calculate timing difference between expected and actual arrival
	kinematic_time_t actual_arrival_time = Time(data->clock_server_tid);
	kinematic_time_t expected_arrival_time = 0;

	// Find which expected sensor this corresponds to
	for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
		if (data->motion.expected_sensors[i] == sensor_node) {
			expected_arrival_time = data->motion.expected_arrival_times[i];
			break;
		}
	}

	if (expected_arrival_time > 0) {
		kinematic_time_t timing_difference = actual_arrival_time - expected_arrival_time;
		i32 timing_difference_ms = kinematic_ticks_to_ms(timing_difference);

		if (timing_difference_ms > 0) {
			log_info("Train %d: Sensor %s arrived %dms LATE (expected vs actual)", data->train_id,
				 sensor_node->name, timing_difference_ms);
		} else if (timing_difference_ms < 0) {
			log_info("Train %d: Sensor %s arrived %dms EARLY (expected vs actual)", data->train_id,
				 sensor_node->name, -timing_difference_ms);
		} else {
			log_info("Train %d: Sensor %s arrived ON TIME", data->train_id,
				 sensor_node->name ? sensor_node->name : "unnamed");
		}
	}

	// Clear expected sensors since we've arrived at one of them
	data->motion.expected_sensors[0] = NULL;
	data->motion.expected_sensors[1] = NULL;
	data->motion.expected_distances[0] = 0;
	data->motion.expected_distances[1] = 0;
	data->motion.expected_arrival_times[0] = 0;
	data->motion.expected_arrival_times[1] = 0;
	data->motion.sensor_timeout_deadlines[0] = 0;
	data->motion.sensor_timeout_deadlines[1] = 0;
	data->motion.sensor_timeout_logged[0] = false;
	data->motion.sensor_timeout_logged[1] = false;
	data->motion.expected_sensor_count = 0;

	data->motion.current_position.sensor = sensor_node;
	data->motion.current_position.offset_mm = 0; // Reset offset to sensor position
	data->last_sensor_trigger_tick = sensor_update->last_triggered_tick;
	data->motion.last_position_update = data->last_sensor_trigger_tick;

	// Log destination sensor reached
	if (data->destination && sensor_node == data->destination) {
		log_info("Train %d: Destination sensor %s triggered (offset target: %lldmm)", data->train_id,
			 data->destination_name, data->destination_offset_mm);
	}

	// Recalculate next expected sensors
	train_calculate_next_sensors(data);
}

static void train_process_sensor_update(train_task_data_t *data, const marklin_msgqueue_message_t *message)
{
	if (!data || !message) {
		return;
	}

	marklin_sensor_state_t *sensor_update = MARKLIN_MSGQUEUE_CAST_TO(marklin_sensor_state_t, message);
	if (!sensor_update) {
		return;
	}

	if (!sensor_update->triggered) {
		return;
	}

	const track_node *sensor_node = marklin_find_sensor_node_by_bank_id(
		track_nodes, track_nodes_size, sensor_update->bank, sensor_update->sensor_id);
	if (!sensor_node) {
		log_error("Train %d: Sensor update for unknown sensor bank=%d, id=%d", data->train_id,
			  sensor_update->bank, sensor_update->sensor_id);
		return;
	}

	log_error("Train %d: Sensor %s triggered, expected: %s, %s", data->train_id,
		  sensor_node->name ? sensor_node->name : "unnamed",
		  data->motion.expected_sensors[0] ? data->motion.expected_sensors[0]->name : "none",
		  data->motion.expected_sensors[1] ? data->motion.expected_sensors[1]->name : "none");

	if (train_should_process_sensor_update(data, sensor_node, sensor_update)) {
		log_error("Train %d: Expected sensor %s triggered, updating position", data->train_id,
			  sensor_node->name);

		const track_node *first_expected_sensor = data->motion.expected_sensors[0];
		const track_node *second_expected_sensor = data->motion.expected_sensors[1];
		train_update_position_from_sensor(data, sensor_node, sensor_update);

		log_debug("Train %d: Expected next sensors: %s, %s", data->train_id,
			  data->motion.expected_sensors[0] ? data->motion.expected_sensors[0]->name : "none",
			  data->motion.expected_sensors[1] ? data->motion.expected_sensors[1]->name : "none");

		// Kinematic model integration
		if (data->kinematic_model_enabled) {
			kinematic_time_t trigger_time = sensor_update->last_triggered_tick;

			data->last_sensor_trigger_tick = trigger_time;
		}

		if (sensor_node == second_expected_sensor) {
			train_safe_release_with_lookahead(data, first_expected_sensor);
		}

		train_safe_release_with_lookahead(data, sensor_node);
	} else {
		return;
	}
}

static void train_check_sensor_timeouts(train_task_data_t *data)
{
	if (!data || data->motion.expected_sensor_count == 0) {
		return;
	}

	// Only check timeouts if train is supposed to be moving
	if (data->motion.commanded_speed == 0 || data->state_machine.current_state == TRAIN_STATE_IDLE) {
		return;
	}

	kinematic_time_t current_time = Time(data->clock_server_tid);
	u8 timed_out_sensors = 0;

	for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
		if (data->motion.sensor_timeout_deadlines[i] == 0) {
			continue; // No timeout set for this sensor
		}

		if (data->motion.sensor_timeout_deadlines[i] != 0 &&
		    current_time > data->motion.sensor_timeout_deadlines[i]) {
			// Only log the timeout message once per sensor
			if (!data->motion.sensor_timeout_logged[i]) {
				log_warn("Train %d: Sensor %s timed out (deadline exceeded)", data->train_id,
					 data->motion.expected_sensors[i] ? data->motion.expected_sensors[i]->name :
									    "unknown");
				data->motion.sensor_timeout_logged[i] = true;
			}
			timed_out_sensors++;
		}
	}

	// Only trigger timeout state if ALL expected sensors have timed out
	if (timed_out_sensors > 0 && timed_out_sensors == data->motion.expected_sensor_count) {
		log_error("Train %d: ALL EXPECTED SENSORS TIMED OUT - Initiating emergency stop", data->train_id);

		// Emergency stop the train
		train_emergency_stop(data);

		// Update train status - train is stopping due to sensor timeout
		// Status now handled by state machine

		// Clear all expected sensors and timeouts
		data->motion.expected_sensors[0] = NULL;
		data->motion.expected_sensors[1] = NULL;
		data->motion.expected_distances[0] = 0;
		data->motion.expected_distances[1] = 0;
		data->motion.expected_arrival_times[0] = 0;
		data->motion.expected_arrival_times[1] = 0;
		data->motion.sensor_timeout_deadlines[0] = 0;
		data->motion.sensor_timeout_deadlines[1] = 0;
		data->motion.sensor_timeout_logged[0] = false;
		data->motion.sensor_timeout_logged[1] = false;
		data->motion.expected_sensor_count = 0;
	}
}

// ############################################################################
// # Random Destination Selection
// ############################################################################

#define RANDOM_DESTINATION_DELAY_MS 100
#define RANDOM_DESTINATION_ARRIVAL_PAUSE_MS 100 // Pause after arriving at destination
#define RANDOM_DESTINATION_MIN_DISTANCE_MM 700

static const track_node *train_select_random_destination(train_task_data_t *data)
{
	if (!data || !data->motion.current_position.sensor) {
		return NULL;
	}

	// Predefined list of destination sensors (well-distributed across the track)
	// static const char *destination_names[] = { "E8", "C11", "E5", "B2", "C16", "E12", "D6", "D7" };
	static const char *destination_names[] = {
		"C13", "C14", "C11", "C12", "B1", "B2", "A3", "A4", "E1", "E2", "D1", "D2", "E7", "E8",
	};
	static const u32 destination_count = sizeof(destination_names) / sizeof(destination_names[0]);

	u32 random_index = (u32)random_range(0, destination_count - 1);

	const track_node *fallback_candidate = NULL;
	kinematic_distance_t fallback_distance = 0;

	for (u32 attempts = 0; attempts < destination_count; attempts++) {
		const char *candidate_name = destination_names[(random_index + attempts) % destination_count];
		const track_node *candidate = marklin_find_node_by_name(track_nodes, track_nodes_size, candidate_name);

		if (candidate && candidate != data->motion.current_position.sensor) {
			path_result_t path_result;
			marklin_error_t path_result_err = Marklin_FindPath(
				data->motion.current_position.sensor, candidate, data->train_id, GLOBAL_ALLOW_REVERSAL,
				GLOBAL_USE_BLOCK_EXIT_AS_START, NULL, 0, &path_result);

			log_info("Train %d: Evaluating random destination %s (path result: %d)", data->train_id,
				 candidate_name, path_result_err);
			if (path_result_err == MARKLIN_ERROR_OK) {
				kinematic_distance_t path_distance = (kinematic_distance_t)path_result.total_distance;

				if (path_distance >= RANDOM_DESTINATION_MIN_DISTANCE_MM) {
					log_info(
						"Train %d: Selected ideal random destination %s (path distance: %lldmm)",
						data->train_id, candidate_name, path_distance);
					Marklin_FreePath(&path_result);
					return candidate;
				}

				if (path_distance > fallback_distance) {
					fallback_distance = path_distance;
					fallback_candidate = candidate;
				}
			}

			Marklin_FreePath(&path_result);
		}
	}

	return fallback_candidate;
}

static void train_waypoint_mode_update(train_task_data_t *data)
{
	train_update_effective_speed(data);

	// Use centralized path state machine for waypoint mode
	train_path_update_state_machine(data);

	// Random destination logic: if enabled, idle, and enough time has passed, select a random destination
	if (data->random_destination_enabled && data->state_machine.current_state == TRAIN_STATE_IDLE &&
	    data->destination == NULL) {
		u64 current_time = Time(data->clock_server_tid);

		// Check both regular delay and arrival pause
		bool regular_delay_satisfied =
			(current_time - data->last_random_destination_time >= MS_TO_TICK(RANDOM_DESTINATION_DELAY_MS));
		bool arrival_pause_satisfied = (data->destination_arrival_time == 0) ||
					       (current_time - data->destination_arrival_time >=
						MS_TO_TICK(RANDOM_DESTINATION_ARRIVAL_PAUSE_MS));

		if (regular_delay_satisfied && arrival_pause_satisfied) {
			const track_node *random_destination = train_select_random_destination(data);
			if (random_destination) {
				data->last_random_destination_time = current_time;
				data->destination_arrival_time = 0; // Reset arrival timer
				log_info("Train %d: Setting random destination %s (after pause)", data->train_id,
					 random_destination->name);

				marklin_error_t nav_result =
					train_navigate_to_destination(data, random_destination->name, 10);
				if (nav_result != MARKLIN_ERROR_OK) {
					log_warn("Train %d: Failed to navigate to random destination %s: error %d",
						 data->train_id, random_destination->name, nav_result);
				}
			}
		}
	}
}

// ############################################################################
// # Reservation Management Functions
// ############################################################################

// Calculate the distance needed for safe stopping based on current speed and kinematics
static kinematic_distance_t train_calculate_distance_needed_for_speed(train_task_data_t *data, u8 speed,
								      bool from_higher)
{
	if (!data) {
		Panic("train_calculate_distance_needed_for_speed: data is NULL");
		return 0;
	}

	if (speed == 0) {
		return 700;
	}

	kinematic_distance_t stopping_distance_mm = 0;

	// Use cached stopping distance from unified kinematic system if available
	if (data->motion.current_stop_distance > 0) {
		stopping_distance_mm = kinematic_model_get_stop_distance(data, speed, from_higher) * 2;
		if (stopping_distance_mm < 500) {
			stopping_distance_mm = 500;
		}
	} else {
		stopping_distance_mm = 500;
	}

	log_debug("Train %d: Calculated %lldmm distance needed for speed %d", data->train_id, stopping_distance_mm,
		  speed);

	return stopping_distance_mm;
}

// ############################################################################
// # Public Train API Implementation
// ############################################################################

marklin_error_t Marklin_TrainNavigateToDestination(u8 train_id, const char *destination_name, bool allow_reverse,
						   u8 requested_speed)
{
	if (!destination_name) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Validate requested speed
	if (requested_speed > MARKLIN_TRAIN_MAX_SPEED) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Create navigation command
	marklin_train_command_t command = { .command_type = MARKLIN_TRAIN_CMD_NAVIGATE_TO_DESTINATION,
					    .navigate_to_destination = { .allow_reverse = allow_reverse,
									 .requested_speed = requested_speed } };

	// Copy destination name safely
	strncpy(command.navigate_to_destination.destination_name, destination_name, 15);
	command.navigate_to_destination.destination_name[15] = '\0';

	// Send command to train controller
	return Marklin_ControllerTrainCommand(train_id, &command);
}

// ############################################################################
// # Centralized Path Management Functions
// ############################################################################

marklin_error_t train_path_request_to_destination(train_task_data_t *data, const track_node *destination,
						  bool allow_reverse)
{
	if (!data || !destination) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Check if we're already at the destination
	if (data->motion.current_position.sensor == destination) {
		data->state_machine.path_state = PATH_STATE_REACHED;
		return MARKLIN_ERROR_OK;
	}

	// Set state to requesting
	data->state_machine.path_state = PATH_STATE_REQUESTING;
	// Status now handled by state machine

	// Find path using block-aware pathfinding in conductor
	path_result_t path_result;
	marklin_error_t path_error = Marklin_FindPath(data->motion.current_position.sensor, destination, data->train_id,
						      allow_reverse, GLOBAL_USE_BLOCK_EXIT_AS_START, NULL, 0,
						      &path_result);

	if (path_error != MARKLIN_ERROR_OK) {
		log_error("Train %d: Failed to find path from %s to %s: error %d", data->train_id,
			  data->motion.current_position.sensor->name, destination->name, path_error);
		data->state_machine.path_state = PATH_STATE_NONE;
		// Status now handled by state machine
		data->path_ends_at_reversal = false;
		return path_error;
	}

	// Activate the path
	log_info("Train %d: Activating path with %p", data->train_id, &path_result);
	return train_path_activate_result(data, &path_result);
}

marklin_error_t train_path_activate_result(train_task_data_t *data, path_result_t *path_result)
{
	if (!data || !path_result) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Check if path starts with an immediate reversal
	bool starts_with_reversal = false;
	if (!dlist_is_empty(&path_result->nodes)) {
		path_node_t *first_node = dlist_entry(dlist_last(&path_result->nodes), path_node_t, list);
		if (first_node && first_node->reverse_here) {
			starts_with_reversal = true;
			log_info("Train %d: Path starts with immediate reversal at %s", data->train_id,
				 first_node->node ? first_node->node->name : "unknown");
		}
	}

	// If path starts with reversal, execute it immediately before activating
	if (starts_with_reversal) {
		log_info("Train %d: Executing immediate reversal before path activation", data->train_id);
		marklin_error_t reverse_result = train_reverse(data);
		if (reverse_result != MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to execute immediate reversal: %d", data->train_id, reverse_result);
			Marklin_FreePath(path_result);
			data->state_machine.path_state = PATH_STATE_NONE;
			return reverse_result;
		}
		// Clear the reverse_here flag since we've already reversed
		path_node_t *first_node = dlist_entry(dlist_first(&path_result->nodes), path_node_t, list);
		if (first_node) {
			first_node->reverse_here = false;
		}
	}

	marklin_path_activation_result_t activation_result;
	kinematic_distance_t distance_needed = train_calculate_distance_needed_for_speed(
		data, data->motion.requested_speed, data->motion.commanded_speed > data->motion.requested_speed);

	marklin_error_t activation_error = Marklin_ActivatePath(path_result, data->train_id, distance_needed * 1.2,
								data->motion.current_position.sensor,
								data->motion.current_position.offset_mm,
								&activation_result);

	if (activation_error != MARKLIN_ERROR_OK && activation_error != MARKLIN_ERROR_ALREADY_RESERVED) {
		const char *dest_name = data->destination ? data->destination->name : "unknown";
		log_error("Train %d: Failed to activate path to %s: error %d", data->train_id, dest_name,
			  activation_error);

		Marklin_FreePath(path_result);
		data->state_machine.path_state = PATH_STATE_NONE;
		// Status now handled by state machine
		data->path_ends_at_reversal = false;
		return activation_error;
	}

	// Update tracking
	data->motion.expected_sensors[0] = activation_result.next_expected_sensor;
	data->motion.expected_distances[0] = activation_result.next_expected_distance;
	data->motion.expected_sensor_count = activation_result.next_expected_sensor ? 1 : 0;
	train_calculate_next_sensors(data);

	if (data->has_active_path) {
		Marklin_FreePath(&data->current_path);
	}

	// Transfer ownership of the path (move linked list and metadata)
	// Copy metadata
	data->current_path.total_distance = path_result->total_distance;
	data->current_path.num_reversals = path_result->num_reversals;
	data->current_path.pool = path_result->pool;

	// Transfer the linked list using proper dlist operations
	if (!dlist_is_empty(&path_result->nodes)) {
		// Replace our list head with the source list head (transfers all nodes)
		dlist_replace(&path_result->nodes, &data->current_path.nodes);
	} else {
		// Empty list - just initialize our head
		dlist_init(&data->current_path.nodes);
	}

	// Clear the source path to prevent double-free
	dlist_init(&path_result->nodes);
	path_result->pool = NULL;
	data->has_active_path = true;
	data->state_machine.path_state = PATH_STATE_ACTIVE;
	// Status now handled by state machine
	data->path_ends_at_reversal = false; // No longer tracking mid-path reversals

	// Initialize activation distance tracking for re-activation
	data->last_activation_distance = distance_needed;

	// Store activation result metadata for progressive path handling
	data->last_activation_result = activation_result;
	data->activation_stop_reason = activation_result.stop_reason;
	data->activation_end_point = activation_result.furthest_activated_node;
	// Simplified - no reversal tracking needed
	data->at_reversal_point = false;
	data->reversal_node = NULL;
	data->reversal_next_node = NULL;

	// Add reserved blocks to train's tracking state
	for (u32 i = 0; i < activation_result.blocks_reserved && i < MAX_TRACK_BLOCKS; i++) {
		if (activation_result.reserved_block_nodes[i] != NULL) {
			train_add_reserved_block(data, activation_result.reserved_block_nodes[i]);
			log_debug("Train %d: Added reserved block %s to local tracking state", data->train_id,
				  activation_result.reserved_block_nodes[i]->name);
		}
	}

	// log_info("Train %d: Added %u reserved blocks to local tracking state", data->train_id,
	//  activation_result.blocks_reserved);

	// Determine if we need path continuation based on stop reason
	data->needs_path_continuation = (activation_result.stop_reason != PATH_ACTIVATION_STOP_END_OF_PATH);

	// Set appropriate path state based on activation result
	// Always start with PATH_STATE_ACTIVE - train must travel to reversal point first
	data->state_machine.path_state = PATH_STATE_ACTIVE;

	// Trigger effective speed update to generate appropriate state machine events
	// This ensures the train transitions to MOVING state when path becomes active with requested_speed > 0
	train_update_effective_speed(data);

	return MARKLIN_ERROR_OK;
}

void train_path_update_state_machine(train_task_data_t *data)
{
	if (!data) {
		log_error("train_path_update_state_machine: Invalid data", data->train_id);
		return;
	}

	switch (data->state_machine.path_state) {
	case PATH_STATE_NONE:
		// Check if we have a destination and should request a path
		if (data->destination && data->operating_mode == TRAIN_MODE_WAYPOINT) {
			u64 current_tick = Time(data->clock_server_tid);
			if (current_tick - data->last_path_request_tick >= MS_TO_TICK(TRAIN_PATH_REQUEST_INTERVAL_MS)) {
				data->last_path_request_tick = current_tick;
				train_path_request_to_destination(data, data->destination, GLOBAL_ALLOW_REVERSAL);
			}
		}
		break;

	case PATH_STATE_REQUESTING:
		// Already in progress, do nothing
		break;

	case PATH_STATE_ACTIVE:
		// NOTE: Destination arrival checking is now handled by unified stop system
		// The unified system will clear destination and set status to IDLE when arrived
		// Check if destination was cleared (meaning we arrived)
		if (!data->destination) {
			data->state_machine.path_state = PATH_STATE_REACHED;
		}

		// Handle path continuation if needed (for partially activated paths)
		if (data->needs_path_continuation) {
			u64 current_tick = Time(data->clock_server_tid);
			if (current_tick - data->last_path_continuation_tick >=
			    MS_TO_TICK(TRAIN_PATH_CONTINUATION_INTERVAL_MS)) {
				data->last_path_continuation_tick = current_tick;
				// Train has stopped at end of partial path, continue activation
				train_handle_path_continuation(data);
			}
		}
		break;

	case PATH_STATE_REACHED:
		// Destination reached, clear path data
		if (data->has_active_path) {
			Marklin_FreePath(&data->current_path);
			data->has_active_path = false;
			data->path_ends_at_reversal = false;
		}
		data->state_machine.path_state = PATH_STATE_NONE;
		break;

	case PATH_STATE_AT_REVERSAL:
		// This state should no longer be reached with simplified reversal logic
		log_warn("Train %d: Unexpected PATH_STATE_AT_REVERSAL - transitioning to ACTIVE", data->train_id);
		data->state_machine.path_state = PATH_STATE_ACTIVE;
		break;

	case PATH_STATE_REVERSING:
		// This state should no longer be reached with simplified reversal logic
		log_warn("Train %d: Unexpected PATH_STATE_REVERSING - transitioning to ACTIVE", data->train_id);
		data->state_machine.path_state = PATH_STATE_ACTIVE;
		break;

	case PATH_STATE_CONTINUATION_NEEDED:
		// Path needs continuation - similar logic to PATH_STATE_ACTIVE
		if (data->needs_path_continuation) {
			u64 current_tick = Time(data->clock_server_tid);
			if (current_tick - data->last_path_continuation_tick >=
			    MS_TO_TICK(TRAIN_PATH_CONTINUATION_INTERVAL_MS)) {
				data->last_path_continuation_tick = current_tick;
				train_handle_path_continuation(data);
			}
		}
		break;
	}
}

// ############################################################################
// # Position Utilities
// ############################################################################

marklin_error_t train_position_validate(const train_position_t *position)
{
	if (!position || !position->sensor) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Validate offset range
	if (position->offset_mm < -1000 || position->offset_mm > 1000) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	return MARKLIN_ERROR_OK;
}

kinematic_distance_t train_position_distance_between(const train_position_t *from, const train_position_t *to,
						     bool use_effective_distance)
{
	if (!from || !to || !from->sensor || !to->sensor) {
		return -1;
	}

	// Get the distance between sensors using conductor API for switch-aware calculation
	kinematic_distance_t raw_distance, effective_distance;
	marklin_error_t result =
		Marklin_CalculateTrackDistance(from->sensor, to->sensor, 0, &raw_distance, &effective_distance);
	if (result != MARKLIN_ERROR_OK) {
		return -1; // Path not found
	}

	// Add the offset difference - use effective distance for physics calculations
	kinematic_distance_t total_distance =
		(use_effective_distance ? effective_distance : raw_distance) + (to->offset_mm - from->offset_mm);

	return total_distance;
}

bool train_position_is_at_destination(const train_position_t *current, const train_position_t *destination,
				      kinematic_distance_t tolerance_mm)
{
	if (!current || !destination) {
		return false;
	}

	// If same sensor, just check offset difference
	if (current->sensor == destination->sensor) {
		kinematic_distance_t offset_diff = current->offset_mm - destination->offset_mm;
		if (offset_diff < 0)
			offset_diff = -offset_diff; // abs()
		return offset_diff <= tolerance_mm;
	}

	// Different sensors - calculate distance
	kinematic_distance_t distance = train_position_distance_between(current, destination, false);
	if (distance < 0) {
		return false; // Distance calculation failed
	}

	return distance <= tolerance_mm;
}

// Compensation for the train length
kinematic_distance_t train_calculate_stopping_offset(const train_task_data_t *data, kinematic_distance_t target_offset,
						     train_direction_t direction)
{
	if (!data) {
		return target_offset;
	}

	kinematic_distance_t train_length = 200;
	kinematic_distance_t base_offset = target_offset;

	// Adjust offset based on direction
	if (direction == TRAIN_DIRECTION_FORWARD) {
		return base_offset - 50;
	} else {
		return base_offset - train_length + 50;
	}
}

// ############################################################################
// # Continuous Position Tracking
// ############################################################################

void train_update_current_position(train_task_data_t *data)
{
	if (!data || !data->kinematic_model_enabled) {
		return; // Can't update position without kinematic model
	}

	if (!data->motion.current_position.sensor) {
		return; // No known position to update from
	}

	kinematic_time_t current_time = Time(data->clock_server_tid);
	kinematic_time_t time_elapsed = current_time - data->motion.last_position_update;

	// Only update if enough time has elapsed (avoid excessive updates)
	if (time_elapsed < 1) { // 10ms minimum update interval
		return;
	}

	// Get current velocity from kinematic model
	kinematic_velocity_t current_velocity = kinematic_model_get_velocity(data, data->motion.commanded_speed,
									     data->motion.commanded_speed_from_higher);
	if (current_velocity <= 0) {
		// Train is stopped or no velocity data available
		data->motion.last_position_update = current_time;
		return;
	}

	// Calculate distance traveled since last update
	// distance = velocity * time
	kinematic_distance_t distance_traveled = kinematic_distance_from_velocity(current_velocity, time_elapsed);

	// Update position based on direction
	data->motion.current_position.offset_mm += distance_traveled;
	kinematic_distance_t raw_distance, effective_distance;
	if (data->motion.expected_sensor_count > 0 && data->motion.expected_sensors[0]) {
		Marklin_CalculateTrackDistance(data->motion.current_position.sensor, data->motion.expected_sensors[0],
					       data->train_id, &raw_distance, &effective_distance);
	} else {
		raw_distance = 0;
		effective_distance = 0;
	}
	if (data->motion.current_position.offset_mm > raw_distance) {
		data->motion.current_position.offset_mm = raw_distance;
	}

	// Update stored velocity and timestamp
	data->motion.last_position_update = current_time;

	// log_debug("Train %d: Position updated - sensor: %s, offset: %lldmm, velocity: %lld", data->train_id,
	//   data->motion.current_position.sensor ? data->motion.current_position.sensor->name : "none",
	//   data->motion.current_position.offset_mm, current_velocity);
}

// ############################################################################
// # Block Management Functions
// ############################################################################

/**
 * Calculate how many blocks ahead the train needs for safe stopping.
 * Uses current stopping distance and estimates blocks along the path.
 */
static u32 train_calculate_blocks_needed_for_stopping(train_task_data_t *data)
{
	if (!data || data->motion.current_stop_distance <= 0) {
		return 1; // At least keep current block
	}

	// Estimate blocks needed based on stop distance and typical block size
	// Add safety margin by using smaller block size estimate
	const kinematic_distance_t CONSERVATIVE_BLOCK_SIZE = TRAIN_DEFAULT_SEGMENT_LENGTH_MM; // 400mm
	u32 blocks_needed = (data->motion.current_stop_distance / CONSERVATIVE_BLOCK_SIZE) + 2; // +2 for safety

	// Cap at reasonable maximum to prevent excessive reservations
	const u32 MAX_STOPPING_BLOCKS = 5;
	if (blocks_needed > MAX_STOPPING_BLOCKS) {
		blocks_needed = MAX_STOPPING_BLOCKS;
	}

	log_debug("Train %d: Calculated %d blocks needed for stopping (stop distance: %lldmm)", data->train_id,
		  blocks_needed, data->motion.current_stop_distance);

	return blocks_needed;
}

/**
 * Try to reserve blocks along the train's forward path for safe stopping.
 * Returns true if successful, false if any reservation fails.
 */
static bool train_try_reserve_stopping_path(train_task_data_t *data, u32 blocks_needed)
{
	if (!data || !data->motion.current_position.sensor || blocks_needed == 0) {
		return true; // Nothing to reserve
	}

	// Try to reserve blocks containing the expected sensors
	u32 reserved_count = 0;
	for (u8 i = 0; i < data->motion.expected_sensor_count && reserved_count < blocks_needed; i++) {
		if (data->motion.expected_sensors[i]) {
			// Try to reserve the block containing this expected sensor
			marklin_error_t result =
				Marklin_ReserveSpecificBlock(data->train_id, data->motion.expected_sensors[i]);
			if (result == MARKLIN_ERROR_OK) {
				train_add_reserved_block(data, data->motion.expected_sensors[i]);
				reserved_count++;
				log_info("Train %d: Reserved stopping block containing sensor %s", data->train_id,
					 data->motion.expected_sensors[i]->name);
			} else {
				log_warn("Train %d: Failed to reserve stopping block containing sensor %s (error: %d)",
					 data->train_id, data->motion.expected_sensors[i]->name, result);
				// Continue trying other blocks rather than failing completely
			}
		}
	}

	return reserved_count > 0; // Success if we reserved at least one block
}

/**
 * Safely release a block only if we can secure the next blocks for stopping.
 * This prevents trains from sliding into unreserved blocks during deceleration.
 */
static marklin_error_t train_safe_release_with_lookahead(train_task_data_t *data, const track_node *sensor_node)
{
	if (!data || !sensor_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// If train is moving, try to secure stopping path before releasing this block
	if (data->motion.commanded_speed > 0) {
		u32 blocks_needed = train_calculate_blocks_needed_for_stopping(data);

		if (blocks_needed > 1) { // If we need more than just current block
			bool reservation_success = train_try_reserve_stopping_path(data, blocks_needed - 1);

			if (!reservation_success) {
				log_warn(
					"Train %d: Cannot secure stopping path, keeping block containing %s for safety",
					data->train_id, sensor_node->name);
				return MARKLIN_ERROR_OK; // Don't release if we can't secure ahead
			}
		}
	}

	// Safe to release this block now
	return train_release_exited_block(data, sensor_node);
}

void train_add_reserved_block(train_task_data_t *data, const track_node *block_node)
{
	if (!data || !block_node || data->reserved_block_count >= MAX_TRACK_BLOCKS) {
		return;
	}

	for (u32 i = 0; i < data->reserved_block_count; i++) {
		if (data->reserved_block_nodes[i] == block_node) {
			return;
		}
	}

	data->reserved_block_nodes[data->reserved_block_count] = block_node;
	data->reserved_block_count++;
	log_debug("Train %d: Added block tracking for node %s (total: %d)", data->train_id, block_node->name,
		  data->reserved_block_count);
}

void train_remove_reserved_block(train_task_data_t *data, const track_node *block_node)
{
	if (!data || !block_node) {
		return;
	}

	for (u32 i = 0; i < data->reserved_block_count; i++) {
		if (data->reserved_block_nodes[i] == block_node) {
			for (u32 j = i; j < data->reserved_block_count - 1; j++) {
				data->reserved_block_nodes[j] = data->reserved_block_nodes[j + 1];
			}
			data->reserved_block_count--;
			log_debug("Train %d: Removed block tracking for node %s (total: %d)", data->train_id,
				  block_node->name, data->reserved_block_count);
			return;
		}
	}
}

void train_clear_all_reserved_blocks(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	if (data->reserved_block_count > 0) {
		log_debug("Train %d: Cleared all %d tracked blocks", data->train_id, data->reserved_block_count);
		data->reserved_block_count = 0;
	}
}

marklin_error_t train_release_all_blocks(train_task_data_t *data, bool keep_current_block)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_error_t result;

	const track_node *keep_block_node = (keep_current_block && data->motion.current_position.sensor) ?
						    data->motion.current_position.sensor :
						    NULL;

	log_info("Train %d: Releasing all blocks (keep current: %s, keep_block_node: %s)", data->train_id,
		 keep_current_block ? "yes" : "no", keep_block_node ? keep_block_node->name : "none");
	result = Marklin_ReleaseTrainBlocks(data->train_id, keep_block_node);
	if (result == MARKLIN_ERROR_OK || result == MARKLIN_ERROR_NOT_FOUND) {
		if (keep_block_node) {
			train_clear_all_reserved_blocks(data);
			train_add_reserved_block(data, keep_block_node);
		} else {
			train_clear_all_reserved_blocks(data);
		}
	}

	return result;
}

marklin_error_t train_release_specific_block(train_task_data_t *data, const track_node *block_node)
{
	if (!data || !block_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Pass current position sensor to ensure atomic operation
	const track_node *current_block_node = data->motion.current_position.sensor;
	marklin_error_t result = Marklin_ReleaseSpecificBlock(data->train_id, block_node, current_block_node);
	if (result == MARKLIN_ERROR_OK) {
		train_remove_reserved_block(data, block_node);
	}

	return result;
}

marklin_error_t train_release_exited_block(train_task_data_t *data, const track_node *sensor_node)
{
	if (!data || !sensor_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Pass current position sensor to ensure atomic operation
	const track_node *current_block_node = data->motion.current_position.sensor;
	marklin_error_t result = Marklin_ReleaseSpecificBlock(data->train_id, sensor_node, current_block_node);
	if (result == MARKLIN_ERROR_OK) {
		train_remove_reserved_block(data, sensor_node);
		log_info("Train %d: Released block containing sensor %s", data->train_id,
			 sensor_node->name ? sensor_node->name : "unnamed");
	} else if (result == MARKLIN_ERROR_NOT_OWNER || result == MARKLIN_ERROR_NOT_FOUND) {
		log_info("Train %d: No block to release for sensor %s (not owned or not found)", data->train_id,
			 sensor_node->name ? sensor_node->name : "unnamed");
		result = MARKLIN_ERROR_OK;
	}

	return result;
}

static void train_ensure_current_block_reserved(train_task_data_t *data)
{
	// Only proceed if we have a valid current position
	if (!data || !data->motion.current_position.sensor) {
		return;
	}

	// Try to reserve the block containing our current position
	marklin_error_t result = Marklin_ReserveSpecificBlock(data->train_id, data->motion.current_position.sensor);

	if (result == MARKLIN_ERROR_OK) {
		// Successfully reserved - add to our tracking if not already there
		bool already_tracked = false;
		for (u32 i = 0; i < data->reserved_block_count; i++) {
			if (data->reserved_block_nodes[i] == data->motion.current_position.sensor) {
				already_tracked = true;
				break;
			}
		}

		if (!already_tracked) {
			train_add_reserved_block(data, data->motion.current_position.sensor);
			log_info("Train %d: Reserved current block containing sensor %s", data->train_id,
				 data->motion.current_position.sensor->name);
		}
	} else if (result == MARKLIN_ERROR_ALREADY_RESERVED) {
		// Block is reserved by someone else - this is a problem!
		// log_warn("Train %d: Current block containing sensor %s is reserved by another train!", data->train_id,
		//  data->motion.current_position.sensor->name);
	}
	// MARKLIN_ERROR_NOT_FOUND means the sensor isn't in a block - that's OK
}

// ############################################################################
// # Progressive Path Activation Functions
// ############################################################################

marklin_error_t train_handle_path_continuation(train_task_data_t *data)
{
	if (!data || !data->has_active_path) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// data->needs_path_continuation = false;
	// if (data->state_machine.path_state != PATH_STATE_NONE && data->state_machine.path_state != PATH_STATE_REACHED) {
	// 	data->state_machine.path_state = PATH_STATE_NONE;
	// }
	// return MARKLIN_ERROR_OK;
	// log_info("Train %d: Continuing path activation from current position", data->train_id);

	// Re-activate the current path with updated distance requirements
	marklin_path_activation_result_t continuation_result;
	kinematic_distance_t distance_needed =
		train_calculate_distance_needed_for_speed(data, data->motion.requested_speed,
							  data->motion.commanded_speed > data->motion.requested_speed) *
		1.2;

	marklin_error_t activation_error = Marklin_ActivatePath(&data->current_path, data->train_id, distance_needed,
								data->motion.current_position.sensor,
								data->motion.current_position.offset_mm,
								&continuation_result);

	// log_info("Train %d: Continuing path activation with distance needed %lldmm", data->train_id, distance_needed);
	if (activation_error != MARKLIN_ERROR_OK && activation_error != MARKLIN_ERROR_ALREADY_RESERVED) {
		log_error("Train %d: Failed to continue path activation: error %d", data->train_id, activation_error);
		return activation_error;
	}

	// Update tracking with continuation results
	data->last_activation_result = continuation_result;
	data->activation_stop_reason = continuation_result.stop_reason;
	data->activation_end_point = continuation_result.furthest_activated_node;
	data->motion.expected_sensors[0] = continuation_result.next_expected_sensor;
	data->motion.expected_distances[0] = continuation_result.next_expected_distance;
	data->motion.expected_sensor_count = continuation_result.next_expected_sensor ? 1 : 0;

	log_info(
		"Train %d: Path continuation activated with next sensor %s at distance %lldmm, activation_end_point %s",
		data->train_id,
		continuation_result.next_expected_sensor ? continuation_result.next_expected_sensor->name : "none",
		continuation_result.next_expected_distance,
		data->activation_end_point ? data->activation_end_point->name : "none");

	train_calculate_next_sensors(data);

	// Add newly reserved blocks to train's tracking state
	for (u32 i = 0; i < continuation_result.blocks_reserved && i < MAX_TRACK_BLOCKS; i++) {
		if (continuation_result.reserved_block_nodes[i] != NULL) {
			train_add_reserved_block(data, continuation_result.reserved_block_nodes[i]);
			log_debug("Train %d: Added reserved block node %s to tracking during continuation (index %u)",
				  data->train_id,
				  continuation_result.reserved_block_nodes[i]->name ?
					  continuation_result.reserved_block_nodes[i]->name :
					  "unnamed",
				  i);
		}
	}

	// log_info("Train %d: Added %u additional reserved blocks during continuation", data->train_id,
	// 	 continuation_result.blocks_reserved);

	// Determine next state based on continuation result
	if (continuation_result.stop_reason == PATH_ACTIVATION_STOP_MAX_BLOCKS_REACHED) {
		data->state_machine.path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = true;
		log_info("Train %d: Path continuation still partial, more needed", data->train_id);

		// Trigger effective speed update to ensure proper state transition after partial continuation
		train_update_effective_speed(data);
	} else {
		data->state_machine.path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = false;
		log_info("Train %d: Path continuation completed to destination", data->train_id);

		// Trigger effective speed update to ensure proper state transition after continuation completion
		train_update_effective_speed(data);
	}

	// Resume movement
	// Status now handled by state machine
	return MARKLIN_ERROR_OK;
}

// Removed complex reversal state machine functions - no longer needed with simplified logic
// Reversals are now handled immediately when a path starts with a reversal

// ############################################################################
// # State Machine State Handlers and Actions
// ############################################################################

// ############################################################################
// # Unified Stop Cleanup Functions
// ############################################################################

/**
 * Unified stop cleanup function that handles block release and state management
 * for all types of train stops (normal, emergency, force).
 */
static void train_perform_stop_cleanup(train_task_data_t *data, bool is_force_stop)
{
	if (!data) {
		return;
	}

	log_info("Train %d: Performing stop cleanup (force_stop: %s)", data->train_id, is_force_stop ? "yes" : "no");

	// Release all blocks except the current one
	marklin_error_t release_result = train_release_all_blocks(data, true);
	if (release_result != MARKLIN_ERROR_OK && release_result != MARKLIN_ERROR_NOT_FOUND) {
		log_warn("Train %d: Failed to release blocks during stop cleanup: %d", data->train_id, release_result);
	}

	// Set path continuation state based on stop type and current state
	if (is_force_stop) {
		// Force stops (user commands) - preserve continuation state if we have a destination
		if (data->destination != NULL) {
			// Check if we've reached destination or still need to continue
			if (data->destination == data->motion.current_position.sensor) {
				// At destination - no continuation needed
				data->needs_path_continuation = false;
				data->state_machine.path_state = PATH_STATE_REACHED;
				log_info("Train %d: Force stop at destination - no continuation needed",
					 data->train_id);
			} else {
				// Not at destination yet - preserve continuation
				data->needs_path_continuation = true;
				data->state_machine.path_state = PATH_STATE_CONTINUATION_NEEDED;
				log_info("Train %d: Force stop - path continuation preserved", data->train_id);
			}
		} else {
			data->needs_path_continuation = false;
			data->state_machine.path_state = PATH_STATE_NONE;
		}
	} else {
		// For non-force stops, check if this is a mid-path stop that needs continuation
		if (data->destination != NULL && data->destination != data->motion.current_position.sensor &&
		    data->has_active_path &&
		    (data->state_machine.path_state == PATH_STATE_ACTIVE ||
		     data->state_machine.path_state == PATH_STATE_CONTINUATION_NEEDED)) {
			// This is a mid-path stop (e.g., due to block reservation limits) - preserve continuation
			data->needs_path_continuation = true;
			data->state_machine.path_state = PATH_STATE_CONTINUATION_NEEDED;
			log_info("Train %d: Mid-path stop - preserving path continuation", data->train_id);
		} else {
			// Emergency stops, destination arrivals, or stops with no active path - clear continuation
			data->needs_path_continuation = false;
			if (data->destination == data->motion.current_position.sensor) {
				data->state_machine.path_state = PATH_STATE_REACHED;
			} else if (data->state_machine.path_state != PATH_STATE_NONE &&
				   data->state_machine.path_state != PATH_STATE_REACHED) {
				data->state_machine.path_state = PATH_STATE_NONE;
			}
		}
	}

	// Clear random destination mode on any stop
	// data->random_destination_enabled = false;

	// Clear path if it exists and we're not continuing
	if (data->has_active_path && !data->needs_path_continuation) {
		Marklin_FreePath(&data->current_path);
		data->has_active_path = false;
		data->path_ends_at_reversal = false;
		data->at_reversal_point = false;
		data->reversal_node = NULL;
		data->reversal_next_node = NULL;
		log_debug("Train %d: Cleared active path during stop cleanup", data->train_id);
	}

	log_debug("Train %d: Stop cleanup completed", data->train_id);
}

// ############################################################################
// # State Machine Handlers
// ############################################################################

// IDLE State - train is stopped and not active
train_transition_result_t train_state_idle_handler(train_task_data_t *data, train_event_t event)
{
	switch (event) {
	case TRAIN_EVENT_START_MOVING:
		return train_state_machine_transition(data, TRAIN_STATE_MOVING);

	case TRAIN_EVENT_EMERGENCY_STOP:
		// Already idle, just acknowledge
		return TRANSITION_HANDLED;

	case TRAIN_EVENT_ERROR_DETECTED:
		return train_state_machine_transition(data, TRAIN_STATE_ERROR);

	default:
		return TRANSITION_IGNORED;
	}
}

void train_state_idle_entry(train_task_data_t *data)
{
	// Ensure train is actually stopped
	data->motion.commanded_speed = 0;

	// Determine if this is a force stop (user command or clear destination)
	// Force stops are user-initiated stops (not automatic due to path logic)
	// We detect this by checking if we have a destination but the train was manually stopped
	// or if the current path state indicates user intervention is needed
	bool is_force_stop =
		(data->destination != NULL &&
		 (data->state_machine.path_state == PATH_STATE_CONTINUATION_NEEDED ||
		  (data->state_machine.path_state == PATH_STATE_ACTIVE && data->motion.requested_speed == 0)));

	// Perform unified stop cleanup for all stop types
	train_perform_stop_cleanup(data, is_force_stop);

	// Only clear requested_speed if we don't need path continuation
	// This preserves the speed for partial path stops so the train can resume
	if (!data->needs_path_continuation) {
		data->motion.requested_speed = 0;
	}

	data->state_machine.movement_state = MOVEMENT_STATE_STATIONARY;

	// Safety fallback: If train is at destination but path state wasn't reset, fix it
	if (data->destination && data->motion.current_position.sensor == data->destination &&
	    data->state_machine.path_state == PATH_STATE_ACTIVE) {
		log_info("Train %d: Safety fallback - clearing destination %s and resetting path state in IDLE entry",
			 data->train_id, data->destination_name);
		data->destination = NULL;
		data->destination_name[0] = '\0';
		data->destination_offset_mm = 0;
		data->state_machine.path_state = PATH_STATE_NONE;
	}

	log_debug("Train %d: Entered IDLE state (requested_speed preserved: %s, stop_type: %s)", data->train_id,
		  data->needs_path_continuation ? "yes" : "no", is_force_stop ? "force" : "normal");
}

void train_state_idle_exit(train_task_data_t *data)
{
	log_debug("Train %d: Exiting IDLE state", data->train_id);
}

// MOVING State - train is in motion
train_transition_result_t train_state_moving_handler(train_task_data_t *data, train_event_t event)
{
	switch (event) {
	case TRAIN_EVENT_STOP_REQUESTED:
	case TRAIN_EVENT_DESTINATION_REACHED:
	case TRAIN_EVENT_PATH_END_REACHED:
		// Instead of transitioning to STOPPING, initiate kinematic stopping within MOVING state
		if (!data->state_machine.kinematic_stopping_active) {
			// Determine stop reason based on event type
			train_stop_action_t stop_reason;
			switch (event) {
			case TRAIN_EVENT_DESTINATION_REACHED:
				stop_reason = TRAIN_STOP_DESTINATION;
				break;
			case TRAIN_EVENT_PATH_END_REACHED:
				stop_reason = TRAIN_STOP_PATH_END;
				break;
			case TRAIN_EVENT_STOP_REQUESTED:
			default:
				stop_reason = TRAIN_CONTINUE; // Use CONTINUE to indicate manual stop
				break;
			}

			// Calculate kinematic stopping time
			u8 current_speed = data->motion.commanded_speed;
			kinematic_time_t kinematic_stop_time = 0;

			if (data->kinematic_model_enabled && current_speed > 0) {
				kinematic_stop_time = kinematic_model_get_stop_time(
					data, current_speed, data->motion.commanded_speed_from_higher);
			}

			// Convert to milliseconds, with fallback
			u32 stop_time_ms;
			if (kinematic_stop_time > 0) {
				stop_time_ms = kinematic_ticks_to_ms(kinematic_stop_time);
			} else {
				stop_time_ms = TRAIN_FALLBACK_STOP_TIME_MS;
			}

			// Initialize kinematic stopping with reason tracking
			data->state_machine.kinematic_stopping_active = true;
			data->state_machine.kinematic_stop_start_time_tick = Time(data->clock_server_tid);
			data->state_machine.kinematic_stop_duration_ms = stop_time_ms;
			data->state_machine.kinematic_stop_reason = stop_reason;
			data->state_machine.movement_state = MOVEMENT_STATE_DECELERATING;

			// Execute the actual stop command
			train_stop(data);

			log_info("Train %d: Initiated kinematic stopping (duration: %u ms, reason: %d) @%d",
				 data->train_id, stop_time_ms, stop_reason, Time(data->clock_server_tid));
		}
		return TRANSITION_HANDLED;

	case TRAIN_EVENT_EMERGENCY_STOP:
		// Emergency stops still go to STOPPING state for immediate response
		return train_state_machine_transition(data, TRAIN_STATE_STOPPING);

	case TRAIN_EVENT_REVERSAL_NEEDED:
		return train_state_machine_transition(data, TRAIN_STATE_REVERSING);

	case TRAIN_EVENT_ERROR_DETECTED:
		return train_state_machine_transition(data, TRAIN_STATE_ERROR);

	case TRAIN_EVENT_SENSOR_TRIGGERED:
	case TRAIN_EVENT_SPEED_CHANGED:
		// Handle within current state - this is crucial for position tracking during deceleration
		return TRANSITION_HANDLED;

	default:
		return TRANSITION_IGNORED;
	}
}

void train_state_moving_entry(train_task_data_t *data)
{
	// Status now handled by state machine
	data->state_machine.movement_state = MOVEMENT_STATE_ACCELERATING;

	log_debug("Train %d: Entered MOVING state", data->train_id);
}

void train_state_moving_exit(train_task_data_t *data)
{
	// Clear kinematic stopping when exiting MOVING state
	if (data->state_machine.kinematic_stopping_active) {
		log_debug("Train %d: Clearing kinematic stopping on MOVING state exit", data->train_id);
		data->state_machine.kinematic_stopping_active = false;
	}

	log_debug("Train %d: Exiting MOVING state", data->train_id);
}

// STOPPING State - emergency stops only (normal stops use kinematic stopping in MOVING state)
train_transition_result_t train_state_stopping_handler(train_task_data_t *data, train_event_t event)
{
	switch (event) {
	case TRAIN_EVENT_EMERGENCY_STOP:
		// Already in emergency stop state, reinforce immediate stop
		train_emergency_stop(data);
		return TRANSITION_HANDLED;

	case TRAIN_EVENT_ERROR_DETECTED:
		return train_state_machine_transition(data, TRAIN_STATE_ERROR);

	default:
		// All other events ignored during emergency stop
		return TRANSITION_IGNORED;
	}
}

void train_state_stopping_entry(train_task_data_t *data)
{
	// Emergency stop: immediate halt regardless of kinematic model
	data->state_machine.movement_state = MOVEMENT_STATE_STATIONARY;

	// Clear path and reset state for safety
	if (data->destination) {
		log_warn("Train %d: Emergency stop - clearing destination %s", data->train_id, data->destination_name);
		data->destination = NULL;
		data->destination_name[0] = '\0';
		data->destination_offset_mm = 0;
	}

	// Reset path state to NONE
	data->state_machine.path_state = PATH_STATE_NONE;

	// Clear any active path
	if (data->has_active_path) {
		// Note: Path cleanup will be handled by the main loop
		data->has_active_path = false;
		data->path_ends_at_reversal = false;
		data->needs_path_continuation = false;
		data->at_reversal_point = false;
	}

	// Execute emergency stop with immediate speed=0
	train_emergency_stop(data);

	log_warn("Train %d: Entered STOPPING state - EMERGENCY STOP executed, path cleared", data->train_id);
}

void train_state_stopping_exit(train_task_data_t *data)
{
	// Emergency stop complete - transition to IDLE unless error occurred
	if (data->state_machine.current_state != TRAIN_STATE_ERROR) {
		train_state_machine_transition(data, TRAIN_STATE_IDLE);
	}

	log_info("Train %d: Exiting STOPPING state - emergency stop complete", data->train_id);
}

// REVERSING State - simplified to handle immediate reversals only
train_transition_result_t train_state_reversing_handler(train_task_data_t *data, train_event_t event)
{
	switch (event) {
	case TRAIN_EVENT_REVERSAL_COMPLETE:
		// Reversal complete, check what state to return to
		if (data->motion.requested_speed > 0) {
			return train_state_machine_transition(data, TRAIN_STATE_MOVING);
		} else {
			return train_state_machine_transition(data, TRAIN_STATE_IDLE);
		}

	case TRAIN_EVENT_EMERGENCY_STOP:
		return train_state_machine_transition(data, TRAIN_STATE_STOPPING);

	case TRAIN_EVENT_ERROR_DETECTED:
		return train_state_machine_transition(data, TRAIN_STATE_ERROR);

	default:
		return TRANSITION_IGNORED;
	}
}

void train_state_reversing_entry(train_task_data_t *data)
{
	// Simplified: just execute the reversal immediately
	log_debug("Train %d: Entered REVERSING state - executing immediate reversal", data->train_id);

	marklin_error_t result = train_reverse(data);
	if (result == MARKLIN_ERROR_OK) {
		// Reversal successful, generate completion event
		train_state_machine_process_event(data, TRAIN_EVENT_REVERSAL_COMPLETE);
	} else {
		// Reversal failed, generate error event
		log_error("Train %d: Failed to execute reversal: %d", data->train_id, result);
		train_state_machine_process_event(data, TRAIN_EVENT_ERROR_DETECTED);
	}
}

void train_state_reversing_exit(train_task_data_t *data)
{
	log_debug("Train %d: Exiting REVERSING state", data->train_id);
}

// ERROR State - train has encountered an error
train_transition_result_t train_state_error_handler(train_task_data_t *data, train_event_t event)
{
	switch (event) {
	case TRAIN_EVENT_EMERGENCY_STOP:
		// Force emergency stop
		train_emergency_stop(data);
		return TRANSITION_HANDLED;

	default:
		// In error state, most events are ignored until manual intervention
		return TRANSITION_IGNORED;
	}
}

void train_state_error_entry(train_task_data_t *data)
{
	// Clear path and reset state for safety (same as emergency stop)
	if (data->destination) {
		log_error("Train %d: Error state - clearing destination %s", data->train_id, data->destination_name);
		data->destination = NULL;
		data->destination_name[0] = '\0';
		data->destination_offset_mm = 0;
	}

	// Reset path state to NONE
	data->state_machine.path_state = PATH_STATE_NONE;

	// Clear any active path
	if (data->has_active_path) {
		data->has_active_path = false;
		data->path_ends_at_reversal = false;
		data->needs_path_continuation = false;
		data->at_reversal_point = false;
	}

	// Force stop the train
	train_emergency_stop(data);

	log_error("Train %d: Entered ERROR state - emergency stop activated, path cleared", data->train_id);
}

void train_state_error_exit(train_task_data_t *data)
{
	log_info("Train %d: Exiting ERROR state", data->train_id);
}

// ############################################################################
// # Low Speed Mode Implementation
// ############################################################################

void train_init_low_speed_mode(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	data->low_speed_mode_enabled = false; // Enable by default for backward compatibility
	data->low_speed_mode_active = false;
	data->low_speed_start_time = 0;
	data->low_speed_expected_duration_ms = 0;
	data->low_speed_target_distance = 0;
}

bool train_should_use_low_speed_mode(train_task_data_t *data, kinematic_distance_t distance)
{
	if (!data || data->operating_mode != TRAIN_MODE_WAYPOINT) {
		return false;
	}

	// Check if low speed mode is enabled
	if (!data->low_speed_mode_enabled) {
		return false;
	}

	// Use low speed mode for short distances, but only if distance is meaningful
	// Avoid activating for tiny distances that would cause immediate timer expiration
	return (distance >= 100 && distance < TRAIN_LOW_SPEED_THRESHOLD_MM);
}

void train_start_low_speed_mode(train_task_data_t *data, kinematic_distance_t distance)
{
	if (!data || data->low_speed_mode_active) {
		return;
	}

	// Get velocity for low speed level from kinematic model
	kinematic_velocity_t velocity = kinematic_model_get_velocity(data, TRAIN_LOW_SPEED_LEVEL, false);
	if (velocity <= 0) {
		// Fallback: assume 50mm/tick for speed level 5
		velocity = 50;
	}

	// Calculate expected travel time: time = distance / velocity (in ticks)
	// Add 20% safety margin for timing precision
	u64 expected_time_ticks = (distance * 1200) / (velocity * 1000); // 1200/1000 = 1.2x margin
	u64 calculated_duration_ms = expected_time_ticks * 10; // Convert ticks to ms

	// Enforce minimum duration to prevent immediate timer expiration
	const u64 MINIMUM_LOW_SPEED_DURATION_MS = 2000; // 2 seconds minimum
	data->low_speed_expected_duration_ms = (calculated_duration_ms > MINIMUM_LOW_SPEED_DURATION_MS) ?
						       calculated_duration_ms :
						       MINIMUM_LOW_SPEED_DURATION_MS;

	data->low_speed_mode_active = true;
	data->low_speed_start_time = Time(data->clock_server_tid);
	data->low_speed_target_distance = distance;

	log_info(
		"Train %d: Started low speed mode - distance: %lldmm, velocity: %lldmm/tick, calculated: %llums, duration: %llums",
		data->train_id, distance, velocity, calculated_duration_ms, data->low_speed_expected_duration_ms);
}

void train_stop_low_speed_mode(train_task_data_t *data)
{
	if (!data || !data->low_speed_mode_active) {
		return;
	}

	log_info("Train %d: Stopping low speed mode", data->train_id);
	data->low_speed_mode_active = false;
	data->low_speed_start_time = 0;
	data->low_speed_expected_duration_ms = 0;
	data->low_speed_target_distance = 0;
}

bool train_check_low_speed_timer(train_task_data_t *data)
{
	if (!data || !data->low_speed_mode_active) {
		return false;
	}

	u64 current_time = Time(data->clock_server_tid);
	u64 elapsed_ms = (current_time - data->low_speed_start_time) * 10; // Convert ticks to ms

	return (elapsed_ms >= data->low_speed_expected_duration_ms);
}

void train_enable_low_speed_mode(train_task_data_t *data, bool enable)
{
	if (!data) {
		return;
	}

	data->low_speed_mode_enabled = enable;

	// If disabling and currently active, stop the active low speed mode
	if (!enable && data->low_speed_mode_active) {
		train_stop_low_speed_mode(data);
	}

	log_info("Train %d: Low speed mode %s", data->train_id, enable ? "enabled" : "disabled");
}

// ############################################################################
// # Path Retry Management Implementation
// ############################################################################

void train_init_retry_state(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	data->last_path_failure_time = 0;
	data->consecutive_path_failures = 0;
	data->next_retry_time = 0;
	data->in_retry_backoff = false;
}

bool train_should_retry_path(train_task_data_t *data)
{
	if (!data) {
		return false;
	}

	// Don't retry if we haven't had failures or exceeded max attempts
	if (data->consecutive_path_failures == 0 || data->consecutive_path_failures >= TRAIN_PATH_RETRY_MAX_ATTEMPTS) {
		return false;
	}

	// Check if we're still in backoff period
	if (data->in_retry_backoff) {
		u64 current_time = (u64)Time(data->clock_server_tid);
		if (current_time < data->next_retry_time) {
			return false; // Still in backoff period
		}

		// Backoff period expired, allow retry
		data->in_retry_backoff = false;
		return true;
	}

	return true;
}

void train_record_path_failure(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	u64 current_time = (u64)Time(data->clock_server_tid);
	data->last_path_failure_time = current_time;
	data->consecutive_path_failures++;

	// Calculate next retry time with exponential backoff
	u64 delay_ms = train_calculate_retry_delay(data);
	data->next_retry_time = current_time + (delay_ms / 10); // Convert ms to ticks
	data->in_retry_backoff = true;

	log_info("Train %d: Path failure recorded (attempt %d/%d), next retry in %llu ms", data->train_id,
		 data->consecutive_path_failures, TRAIN_PATH_RETRY_MAX_ATTEMPTS, delay_ms);
}

void train_reset_retry_state(train_task_data_t *data)
{
	if (!data) {
		return;
	}

	if (data->consecutive_path_failures > 0) {
		log_info("Train %d: Path retry state reset after %d failures", data->train_id,
			 data->consecutive_path_failures);
	}

	data->consecutive_path_failures = 0;
	data->in_retry_backoff = false;
	data->next_retry_time = 0;
}

u64 train_calculate_retry_delay(train_task_data_t *data)
{
	if (!data || data->consecutive_path_failures == 0) {
		return TRAIN_PATH_RETRY_INITIAL_DELAY_MS;
	}

	// Exponential backoff: delay = initial * (multiplier ^ (failures - 1))
	u64 delay = TRAIN_PATH_RETRY_INITIAL_DELAY_MS;
	for (u32 i = 1; i < data->consecutive_path_failures; i++) {
		delay *= TRAIN_PATH_RETRY_BACKOFF_MULTIPLIER;
		if (delay > TRAIN_PATH_RETRY_MAX_DELAY_MS) {
			delay = TRAIN_PATH_RETRY_MAX_DELAY_MS;
			break;
		}
	}

	return delay;
}
