#include "marklin/train/train.h"
#include "marklin/command/command.h"
#include "marklin/common/track_node.h"
#include "marklin/conductor/path.h"
#include "marklin/controller/api.h"
#include "marklin/conductor/api.h"
#include "marklin/msgqueue/api.h"
#include "marklin/topology/track.h"
#include "marklin/topology/api.h"
#include "marklin/train/api.h"
#include "marklin/train/kinematics.h"
#include "marklin/train/model.h"
#include "marklin/train/calibration.h"
#include "marklin/train/offline.h"
#include "compiler.h"
#include "name.h"
#include "stdbool.h"
#include "syscall.h"
#include "clock.h"
#include "clock_server.h"
#include "string.h"
#include "random.h"
#include "types.h"

#define LOG_MODULE "MARKLIN_TRAIN"
#define LOG_LEVEL LOG_LEVEL_WARN
#include "log.h"
#include "klog.h"

// Block ownership validation is now handled through conductor APIs
// No direct access to conductor data needed

// ############################################################################
// # Configuration Constants
// ############################################################################

#define TRAIN_PATH_REQUEST_INTERVAL_MS 2000
#define TRAIN_PATH_CONTINUATION_INTERVAL_MS 1000

// Kinematic estimation configuration constants
#define TRAIN_STOPPING_SAFETY_MARGIN_PERCENT 0
#define TRAIN_DEFAULT_SEGMENT_LENGTH_MM 400
#define TRAIN_DEFAULT_BLOCK_LENGTH_MM 800
#define TRAIN_MAX_RESERVATION_MULTIPLIER 2
#define TRAIN_FALLBACK_STOP_TIME_MS 5000
#define TRAIN_EMERGENCY_STOP_THRESHOLD_MM 20

// Collision avoidance safety constants
#define TRAIN_BLOCK_SAFETY_MARGIN_MM 50 // Additional safety margin for block boundary checks
#define TRAIN_MAX_LOOKAHEAD_BLOCKS 3 // Maximum number of blocks to check ahead

#define GLOBAL_ALLOW_REVERSAL false
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

// Alignment and stopping distance helpers
static marklin_error_t train_align_to_sensor(train_task_data_t *data);

// Real-time speed tracking functions
static void train_update_current_speed_gradually(train_task_data_t *data);

// Operating mode specific functions
static void train_waypoint_mode_update(train_task_data_t *data);
static void train_common_mode_update(train_task_data_t *data);

// Random destination helper
static const track_node *train_select_random_destination(train_task_data_t *data);

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
static marklin_error_t train_handle_offline_experiment_command(train_task_data_t *data,
							       const marklin_train_command_t *command);
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
// # Speed Control Helper Functions
// ############################################################################

void train_report(train_task_data_t *data)
{
	UNUSED(data);
	// log_info("Train %d: Current stop distance: %d", data->train_id, data->motion.current_stop_distance);
}

static u8 train_calculate_effective_speed(train_task_data_t *data)
{
	if (!data) {
		return 0;
	}

	// In waypoint mode without a destination, effective speed should be 0
	if (data->operating_mode == TRAIN_MODE_WAYPOINT) {
		if (data->destination == NULL) {
			return 0;
		}
		switch (data->path_state) {
		case PATH_STATE_ACTIVE:
			break;
		case PATH_STATE_NONE:
		case PATH_STATE_REQUESTING:
		case PATH_STATE_REACHED:
		case PATH_STATE_AT_REVERSAL:
			return 0;
		case PATH_STATE_REVERSING:
			return data->motion.commanded_speed;
		}
	}

	// Calculate the effective speed (requested speed with block-based constraints)
	u8 normal_effective_speed = data->motion.requested_speed;

	// Check if we're in waypoint mode with an active path
	// if (data->operating_mode == TRAIN_MODE_WAYPOINT && data->path_state == PATH_STATE_ACTIVE &&
	//     data->has_active_path && data->current_path.total_distance > 0) {
	// 	// PRIORITY 1: Check if we have a partial activation (activation_end_point exists and differs from destination)
	// 	if (data->activation_end_point && data->activation_end_point != data->destination) {
	// 		// Calculate distance to activation end point
	// 		train_position_t current_pos = data->motion.current_position;
	// 		train_position_t activation_end_pos = {
	// 			.sensor = data->activation_end_point,
	// 			.offset_mm = 0 // Assume we need to stop at the sensor
	// 		};

	// kinematic_distance_t distance_to_activation_end =
	// 	train_position_distance_between(&current_pos, &activation_end_pos, true);

	// 	if (distance_to_activation_end > 0 && data->motion.current_stop_distance > 0) {
	// 		// Check if stopping distance exceeds available activated path distance
	// 		if (data->motion.current_stop_distance > distance_to_activation_end) {
	// 			u8 min_speed_with_data = train_find_minimum_speed_with_velocity_data(data);
	// 			if (min_speed_with_data > 0) {
	// 				log_info(
	// 					"Train %d: Activated path segment too short (%lldmm to %s) for stop distance (%lldmm), using minimum speed %d",
	// 					data->train_id, distance_to_activation_end,
	// 					data->activation_end_point->name,
	// 					data->motion.current_stop_distance, min_speed_with_data);
	// 				return min_speed_with_data;
	// 			} else {
	// 				log_error(
	// 					"Train %d: Activated path segment too short and no speed with velocity data available",
	// 					data->train_id);
	// 				return 0;
	// 			}
	// 		}
	// 	}
	// }

	// PRIORITY 2: Check if the stopping distance for the normal effective speed exceeds the full path distance
	// 	if (data->motion.current_stop_distance > 0 &&
	// 	    data->motion.current_stop_distance > data->current_path.total_distance) {
	// 		// Find the minimum speed with velocity data that can be used
	// 		u8 min_speed_with_data = train_find_minimum_speed_with_velocity_data(data);
	// 		if (min_speed_with_data > 0) {
	// 			log_info("Train %d: Path too short (%d mm) for stop distance (%d mm), using minimum speed %d",
	// 				 data->train_id, data->current_path.total_distance, data->motion.current_stop_distance,
	// 				 min_speed_with_data);
	// 			return min_speed_with_data;
	// 		} else {
	// 			log_error("Train %d: Path too short and no speed with velocity data available", data->train_id);
	// 			return 0;
	// 		}
	// 	}
	// }

	// Return the normal effective speed
	return normal_effective_speed;
}

/**
 * Find the minimum speed level that has velocity data available.
 * This is used when the requested speed is too high for a short path.
 *
 * @param data Train task data containing kinematic model
 * @return Minimum speed level (1-14) with velocity data, or 0 if none found
 */
// static u8 train_find_minimum_speed_with_velocity_data(train_task_data_t *data)
// {
// 	if (!data || !data->kinematic_model) {
// 		return 0;
// 	}

// 	// Check speeds from 1 to 14 to find the first one with velocity data
// 	for (u8 speed = 1; speed <= MARKLIN_TRAIN_MAX_SPEED; speed++) {
// 		// Check both directions (from higher and from lower speed)
// 		kinematic_velocity_t velocity_from_lower = kinematic_model_get_velocity(data, speed, false);
// 		kinematic_velocity_t velocity_from_higher = kinematic_model_get_velocity(data, speed, true);

// 		// If either direction has velocity data, this speed is usable
// 		if (velocity_from_lower > 0 || velocity_from_higher > 0) {
// 			return speed;
// 		}
// 	}

// 	// No speed has velocity data - return 0 (stop)
// 	return 0;
// }

static marklin_error_t train_apply_speed_change(train_task_data_t *data, u8 new_effective_speed)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (data->motion.commanded_speed != new_effective_speed) {
		marklin_error_t result =
			train_set_speed_and_headlight(data, new_effective_speed, MARKLIN_TRAIN_HEADLIGHT_AUTO);
		if (result == MARKLIN_ERROR_OK) {
			data->motion.commanded_speed = new_effective_speed;
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

	// Offline experiment commands - allowed in manual mode only
	case MARKLIN_TRAIN_CMD_START_OFFLINE_EXPERIMENT:
	case MARKLIN_TRAIN_CMD_STOP_OFFLINE_EXPERIMENT:
		return (mode == TRAIN_MODE_MANUAL);

	// Model query - allowed in all modes
	case MARKLIN_TRAIN_CMD_GET_KINEMATIC_MODEL:
		return true;

	// Debug command - allowed in all modes
	case MARKLIN_TRAIN_CMD_DEBUG_INFO:
		return true;

	// Random destination mode - only allowed in waypoint mode
	case MARKLIN_TRAIN_CMD_SET_RANDOM_DESTINATION_MODE:
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
		return train_set_speed_and_headlight(data, data->motion.commanded_speed, data->headlight);

	case MARKLIN_TRAIN_CMD_MANUAL_REVERSE:
		return train_reverse_and_continue(data);

	case MARKLIN_TRAIN_CMD_MANUAL_TOGGLE_HEADLIGHT:
		return train_toggle_headlight(data);

	case MARKLIN_TRAIN_CMD_MANUAL_STOP:
		return train_stop(data);

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
		return train_emergency_stop(data);

	default:
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}
}

static marklin_error_t train_handle_offline_experiment_command(train_task_data_t *data,
							       const marklin_train_command_t *command)
{
	log_info("Train %d: Handling offline experiment command", data->train_id);
	switch (command->command_type) {
	case MARKLIN_TRAIN_CMD_START_OFFLINE_EXPERIMENT:
		// Check if already running an experiment
		if (data->offline_experiment_active) {
			log_error("Train %d: Experiment already active", data->train_id);
			return MARKLIN_ERROR_UNKNOWN;
		}

		// Initialize experiment context based on type
		offline_create_default_config(command->start_offline_experiment.experiment_type,
					      &data->offline_experiment_context);

		// Copy speed levels from command
		u8 speed_count = command->start_offline_experiment.speed_count;
		if (speed_count > 28)
			speed_count = 28;

		if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
			for (u8 i = 0; i < speed_count; i++) {
				data->offline_experiment_context.velocity_loop.offline_speed_levels[i] =
					command->start_offline_experiment.speed_levels[i];
			}
			data->offline_experiment_context.velocity_loop.offline_speed_count = speed_count;
		} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_ACCELERATION) {
			// For acceleration, speeds come in pairs
			u8 pair_count = speed_count / 2;
			if (pair_count > MAX_ACCELERATION_SPEED_PAIRS) {
				pair_count = MAX_ACCELERATION_SPEED_PAIRS;
			}
			for (u8 i = 0; i < pair_count; i++) {
				data->offline_experiment_context.acceleration.from_speed_levels[i] =
					command->start_offline_experiment.speed_levels[i * 2];
				data->offline_experiment_context.acceleration.to_speed_levels[i] =
					command->start_offline_experiment.speed_levels[i * 2 + 1];
			}
			data->offline_experiment_context.acceleration.speed_pair_count = pair_count;
		} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_STOP_DISTANCE) {
			for (u8 i = 0; i < speed_count && i < MAX_STOP_DISTANCE_SPEEDS; i++) {
				data->offline_experiment_context.stop_distance.speed_levels[i] =
					command->start_offline_experiment.speed_levels[i];
			}
			data->offline_experiment_context.stop_distance.speed_count =
				(speed_count < MAX_STOP_DISTANCE_SPEEDS) ? speed_count : MAX_STOP_DISTANCE_SPEEDS;
		}

		// Start the experiment
		return train_start_offline_experiment(data);

	case MARKLIN_TRAIN_CMD_STOP_OFFLINE_EXPERIMENT:
		// Stop the experiment if active
		if (data->offline_experiment_active) {
			return train_offline_cleanup_experiment(data);
		}
		return MARKLIN_ERROR_OK;

	case MARKLIN_TRAIN_CMD_GET_KINEMATIC_MODEL:
		// Display the kinematic model
		kinematic_model_print_defaults(data);
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
	log_warn("Motion: Commanded=%d, Actual=%d, Direction=%s", data->motion.commanded_speed,
		 data->motion.actual_speed, direction_str);
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
	const char *status_str;
	switch (data->status) {
	case TRAIN_STATUS_IDLE:
		status_str = "IDLE";
		break;
	case TRAIN_STATUS_REQUESTING_PATH:
		status_str = "REQUESTING_PATH";
		break;
	case TRAIN_STATUS_MOVING:
		status_str = "MOVING";
		break;
	case TRAIN_STATUS_STOPPING:
		status_str = "STOPPING";
		break;
	default:
		status_str = "UNKNOWN";
		break;
	}
	const char *mode_str = (data->operating_mode == TRAIN_MODE_MANUAL) ? "MANUAL" : "WAYPOINT";
	log_warn("Status: %s, Mode: %s", status_str, mode_str);

	// Path and reservations
	if (data->has_active_path) {
		log_warn("Path: ACTIVE, State=%d, Ends at reversal=%s, End of activation=%s", data->path_state,
			 data->path_ends_at_reversal ? "YES" : "NO",
			 data->activation_end_point ? data->activation_end_point->name : "None");

		// Print the path diagram
		path_print(&data->current_path);
	} else {
		log_warn("Path: NONE");
	}
	log_warn("Reservations: %d blocks", data->reserved_block_count);

	// Kinematic model status
	log_warn("Kinematic: Model enabled=%s", data->kinematic_model_enabled ? "YES" : "NO");
	if (data->offline_experiment_active) {
		log_warn("Offline Experiment: ACTIVE (type=%d)", data->offline_experiment_context.type);
	}

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
// # Sensor Alignment Functions
// ############################################################################

static __maybe_unused marklin_error_t train_align_to_sensor(train_task_data_t *data)
{
	if (!data->motion.current_position.sensor || data->motion.current_position.sensor->type != NODE_SENSOR) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	u8 target_bank, target_sensor_id;
	marklin_get_sensor_info_from_node(data->motion.current_position.sensor, &target_bank, &target_sensor_id);

	if (target_bank == 0xFF || target_sensor_id == 0xFF) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Query the current sensor state directly
	marklin_sensor_state_t sensor_state;
	sensor_state.bank = target_bank;
	sensor_state.sensor_id = target_sensor_id;
	sensor_state.triggered = 0;
	marklin_error_t query_result = Marklin_GetSensorStates(&sensor_state, 1);

	if (query_result != MARKLIN_ERROR_OK) {
		log_error("Train %d: Failed to query sensor state for bank %d, sensor %d", data->train_id, target_bank,
			  target_sensor_id);
		return query_result;
	}

	if (!sensor_state.triggered) {
		log_info("Train %d: Sensor not triggered, aligning to sensor %s", data->train_id,
			 data->motion.current_position.sensor->name);

		// Subscribe to sensor updates for waiting
		marklin_msgqueue_subscription_t sensor_subscription;
		marklin_error_t sub_result =
			Marklin_MsgQueue_Subscribe(MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE, &sensor_subscription);
		if (sub_result != MARKLIN_ERROR_OK) {
			return sub_result;
		}

		// Start moving at lowest speed with headlight on
		marklin_error_t move_result = train_set_speed_and_headlight(data, 1, MARKLIN_TRAIN_HEADLIGHT_ON);
		if (move_result != MARKLIN_ERROR_OK) {
			Marklin_MsgQueue_Unsubscribe(&sensor_subscription);
			return move_result;
		}

		for (;;) {
			marklin_msgqueue_message_t message;
			marklin_error_t msg_result = Marklin_MsgQueue_Receive(&message, 0);

			if (msg_result == MARKLIN_ERROR_OK &&
			    message.event_type == MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE) {
				marklin_sensor_state_t *sensor_update =
					MARKLIN_MSGQUEUE_CAST_TO(marklin_sensor_state_t, &message);
				if (sensor_update && sensor_update->bank == target_bank &&
				    sensor_update->sensor_id == target_sensor_id && sensor_update->triggered) {
					// Sensor triggered! Stop the train
					log_info("Train %d: Sensor triggered, stopping", data->train_id);
					train_stop(data);
					break;
				}
			}
		}

		Marklin_MsgQueue_Unsubscribe(&sensor_subscription);
	}
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
	data->status = TRAIN_STATUS_IDLE;
	data->operating_mode = TRAIN_MODE_MANUAL; // Default to manual mode

	data->clock_server_tid = -1;
	data->controller_tid = -2;
	data->conductor_tid = -3;
	data->command_server_tid = -4;

	// Initialize unified motion state
	memset(&data->motion, 0, sizeof(train_motion_state_t));
	data->motion.commanded_speed = 0;
	data->motion.requested_speed = 0;
	data->motion.actual_speed = 0;
	data->motion.direction = TRAIN_DIRECTION_FORWARD;
	data->motion.current_position.sensor = NULL;
	data->motion.commanded_speed_from_higher = false;
	data->motion.actual_speed_from_higher = false;
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
	data->motion.expected_sensor_count = 0;

	data->headlight = MARKLIN_TRAIN_HEADLIGHT_OFF;

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
	data->path_state = PATH_STATE_NONE;

	// Block-based safety only - no signal system

	// Kinematic model initialization
	data->kinematic_model_enabled = true; // Enable by default

	// Offline experiment initialization
	data->offline_experiment_active = false;
	memset(&data->offline_experiment_context, 0, sizeof(offline_experiment_context_t));

	// Online calibration initialization
	memset(&data->online_calibration, 0, sizeof(online_calibration_context_t));

	data->kinematic_model = NULL;

	// Random destination initialization
	data->random_destination_enabled = false; // Disabled by default
	data->last_random_destination_time = 0;
	data->destination_arrival_time = 0;
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

	// Initialize online calibration system for this train
	// marklin_error_t calibration_init_result = train_online_calibration_init(&train_data);
	// if (calibration_init_result != MARKLIN_ERROR_OK) {
	// 	log_error("Train %d: Failed to initialize online calibration system: %d", train_data.train_id,
	// 		  calibration_init_result);
	// } else {
	// 	log_info("Train %d: Online calibration system initialized successfully", train_data.train_id);
	// }

	train_set_speed_and_headlight(&train_data, 0, MARKLIN_TRAIN_HEADLIGHT_ON);

	// marklin_error_t align_result = train_align_to_sensor(&train_data);
	// if (align_result != MARKLIN_ERROR_OK) {
	// 	log_error("Train %d: Failed to align to sensor", train_data.train_id);
	// }
	// log_info("Train %d: Aligned to sensor", train_data.train_id);

	// train_reverse_and_continue(&train_data);
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
		.status = data->status,
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

static __maybe_unused void train_autonomous_loop(train_task_data_t *data)
{
	offline_create_default_config(OFFLINE_EXPERIMENT_STOP_DISTANCE, &data->offline_experiment_context);
	// train_start_offline_experiment(data);

	// train_navigate_to_destination(data, "E7", false, 10);

	for (;;) {
		// 1. Update current_speed gradually based on acceleration/deceleration
		train_update_current_speed_gradually(data);

		// 2. Update continuous position tracking
		train_update_current_position(data);

		// 2a. Ensure we always own the block we're currently in
		train_ensure_current_block_reserved(data);

		// 5. Poll sensor updates (non-blocking)
		if (data->sensor_subscription_active) {
			marklin_msgqueue_message_t message;
			marklin_error_t msg_result = Marklin_MsgQueue_ReceiveNonBlock(&message);

			if (msg_result == MARKLIN_ERROR_OK &&
			    message.event_type == MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE) {
				train_process_sensor_update(data, &message);
			}
		}

		// 3. Update stopping distance
		train_update_stop_distance(data);

		// 4. Check all stop conditions unified
		train_stop_action_t stop_action = train_check_unified_stop_conditions(data);
		if (stop_action != TRAIN_CONTINUE) {
			train_execute_stop_action(data, stop_action);
		}

		// 4b. Update offline experiment if active
		if (data->offline_experiment_active) {
			train_update_offline_experiment(data);
		}

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
			// Manual mode: check signals for safety and do basic reservation
			train_common_mode_update(data);

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

		train_update_offline_experiment(data);

		// 8. Handle interactive commands (non-blocking receive)
		marklin_train_command_t command;
		int sender_tid;
		int result = ReceiveNonBlock(&sender_tid, (char *)&command, sizeof(command));
		if (result == sizeof(command)) {
			marklin_error_t cmd_result = MARKLIN_ERROR_OK;

			if (!train_is_command_valid_for_mode(data->operating_mode, command.command_type)) {
				cmd_result = MARKLIN_ERROR_INVALID_ARGUMENT;
			} else {
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

				// Offline experiment commands
				case MARKLIN_TRAIN_CMD_START_OFFLINE_EXPERIMENT:
				case MARKLIN_TRAIN_CMD_STOP_OFFLINE_EXPERIMENT:
				case MARKLIN_TRAIN_CMD_GET_KINEMATIC_MODEL:
					cmd_result = train_handle_offline_experiment_command(data, &command);
					break;

				// Random destination mode command
				case MARKLIN_TRAIN_CMD_SET_RANDOM_DESTINATION_MODE:
					cmd_result = train_handle_waypoint_command(data, &command);
					break;

				// Debug command
				case MARKLIN_TRAIN_CMD_DEBUG_INFO:
					cmd_result = train_handle_debug_command(data, &command);
					break;

				default:
					cmd_result = MARKLIN_ERROR_INVALID_ARGUMENT;
					break;
				}
			}

			Reply(sender_tid, (const char *)&cmd_result, sizeof(cmd_result));
		}

		Delay(data->clock_server_tid, 1); // 10ms loop interval
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

	if (data->motion.actual_speed == 0) {
		data->motion.current_stop_distance = 0;
	} else {
		data->motion.current_stop_distance = kinematic_model_get_stop_distance(
			data, data->motion.actual_speed, data->motion.actual_speed_from_higher);
	}

	data->motion.last_stop_distance_update = current_time;
}

// Check for collision risks by examining block ownership ahead of train
static void train_check_block_safety_conditions(train_task_data_t *data)
{
	if (!data || !data->motion.current_position.sensor) {
		return;
	}

	kinematic_distance_t safety_distance = TRAIN_BLOCK_SAFETY_MARGIN_MM;

	// Check ownership of next expected sensors within safety distance
	for (int i = 0; i < 2 && data->motion.expected_sensors[i]; i++) {
		const track_node *next_sensor = data->motion.expected_sensors[i];
		kinematic_distance_t distance_to_sensor = data->motion.expected_distances[i];

		// If we're close enough to this sensor to be concerned about collision
		if (distance_to_sensor > 0 && distance_to_sensor <= safety_distance) {
			bool owns_block = false;
			u8 owner_train_id = 0;

			marklin_error_t result =
				Marklin_CheckBlockOwnership(data->train_id, next_sensor, &owns_block, &owner_train_id);

			if (result == MARKLIN_ERROR_OK && !owns_block) {
				// Block ahead is not owned by us - potential collision!
				// log_error("COLLISION AVOIDANCE: Train %d approaching unreserved block at sensor %s "
				//   "(distance: %lldmm, safety_distance: %lldmm, owned by train %d) - FORCE STOP",
				//   data->train_id, next_sensor->name, distance_to_sensor, safety_distance,
				//   owner_train_id);
				// train_force_stop(data);
				return;
			}
		}
	}

	// Check if we're approaching the end of our reserved blocks
	// Look ahead through expected sensors to find the extent of our reservations
	kinematic_distance_t total_reserved_distance = 0;
	bool found_unreserved_block = false;

	for (int i = 0; i < 2 && data->motion.expected_sensors[i] && !found_unreserved_block; i++) {
		const track_node *sensor = data->motion.expected_sensors[i];
		kinematic_distance_t distance = data->motion.expected_distances[i];

		bool owns_block = false;
		u8 owner_train_id = 0;

		marklin_error_t result =
			Marklin_CheckBlockOwnership(data->train_id, sensor, &owns_block, &owner_train_id);

		if (result == MARKLIN_ERROR_OK) {
			if (owns_block) {
				total_reserved_distance = distance; // Update extent of reserved territory
			} else {
				found_unreserved_block = true;
				break;
			}
		}
	}

	// If we found the end of our reserved blocks and we're too close, force stop
	if (found_unreserved_block && total_reserved_distance > 0 && total_reserved_distance <= safety_distance) {
		// log_error("COLLISION AVOIDANCE: Train %d approaching end of reserved blocks "
		//   "(reserved_distance: %lldmm, safety_distance: %lldmm) - FORCE STOP",
		//   data->train_id, total_reserved_distance, safety_distance);
		// train_force_stop(data);
		return;
	}
}

// Unified function to check all stop conditions
train_stop_action_t train_check_unified_stop_conditions(train_task_data_t *data)
{
	static int count = 0;
	count++;

	if (!data || data->motion.actual_speed == 0) {
		return TRAIN_CONTINUE; // Already stopped
	}

	// 0. Check collision avoidance conditions first (highest priority for safety)
	train_check_block_safety_conditions(data);

	// 1. Check collision avoidance conditions (high priority)

	// 1.5. Check path activation end conditions (for partial path activation)
	if (data->path_state == PATH_STATE_ACTIVE && data->activation_end_point) {
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

	// 1.6. Check reversal point conditions
	if (data->path_state == PATH_STATE_AT_REVERSAL && data->reversal_node) {
		// Check if we are at or approaching the reversal point
		if (data->motion.current_position.sensor == data->reversal_node) {
			log_info("Train %d: Reached reversal point at %s", data->train_id, data->reversal_node->name);
			return TRAIN_STOP_REVERSAL;
		}
	}

	// 1.7. Check minimum speed mode conditions (for short paths and activated segments)
	// If we're in waypoint mode and using minimum speed due to short path or activated segment,
	// trigger emergency stop when approaching destination or activation end point
	if (data->operating_mode == TRAIN_MODE_WAYPOINT && data->path_state == PATH_STATE_ACTIVE &&
	    data->has_active_path && data->current_path.total_distance > 0) {
		// PRIORITY A: Check for minimum speed mode due to short activated path segment
		if (data->activation_end_point && data->activation_end_point != data->destination) {
			train_position_t current_pos = data->motion.current_position;
			train_position_t activation_end_pos = { .sensor = data->activation_end_point, .offset_mm = 0 };
			kinematic_distance_t distance_to_activation_end =
				train_position_distance_between(&current_pos, &activation_end_pos, true);

			// Check if we're in minimum speed mode for activated segment
			if (data->motion.current_stop_distance > 0 && distance_to_activation_end > 0 &&
			    data->motion.current_stop_distance > distance_to_activation_end) {
				// Trigger emergency stop when very close to activation end point in minimum speed mode
				if (distance_to_activation_end <= TRAIN_EMERGENCY_STOP_THRESHOLD_MM) {
					log_info(
						"Train %d: Emergency stop - minimum speed mode approaching activation end %s (distance: %lldmm)",
						data->train_id, data->activation_end_point->name,
						distance_to_activation_end);
					return TRAIN_EMERGENCY_STOP;
				}
			}
		}

		// PRIORITY B: Check for minimum speed mode due to short full path (existing logic)
		if (data->destination) {
			// Check if we're likely in minimum speed mode (stop distance > full path distance)
			if (data->motion.current_stop_distance > 0 &&
			    data->motion.current_stop_distance > data->current_path.total_distance) {
				// Calculate distance to destination
				train_position_t current_pos = data->motion.current_position;
				kinematic_distance_t compensated_offset = train_calculate_stopping_offset(
					data, data->destination_offset_mm, data->motion.direction);
				train_position_t target_pos = { .sensor = data->destination,
								.offset_mm = compensated_offset };
				kinematic_distance_t distance_to_target =
					train_position_distance_between(&current_pos, &target_pos, true);

				// Trigger emergency stop when we're very close to destination in minimum speed mode
				if (distance_to_target > 0 && distance_to_target <= TRAIN_EMERGENCY_STOP_THRESHOLD_MM) {
					log_info(
						"Train %d: Emergency stop - minimum speed mode approaching destination %s (distance: %lldmm)",
						data->train_id,
						data->destination->name ? data->destination->name : "unknown",
						distance_to_target);
					return TRAIN_EMERGENCY_STOP;
				}
			}
		}
	}

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
				return TRAIN_STOP_DESTINATION;
			}
		} else {
			if (distance_to_target > 0 && distance_to_target <= TRAIN_EMERGENCY_STOP_THRESHOLD_MM) {
				log_warn(
					"Train %d: Emergency stop - approaching destination %s without stop distance data (distance: %lldmm)",
					data->train_id, data->destination->name ? data->destination->name : "unknown",
					distance_to_target);
				return TRAIN_EMERGENCY_STOP;
			}
		}

		// Check if we've arrived at destination (for both cases)
		if (train_position_is_at_destination(&current_pos, &target_pos, 100)) {
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
		// Clear destination if arrived
		if (data->destination) {
			// Record arrival time for random destination pause
			if (data->random_destination_enabled) {
				data->destination_arrival_time = Time(data->clock_server_tid);
				log_info("Train %d: Arrived at random destination, starting pause timer",
					 data->train_id);
			}
			// train_position_t current_pos = data->motion.current_position;
			// kinematic_distance_t compensated_offset = train_calculate_stopping_offset(
			// data, data->destination_offset_mm, data->motion.direction);
			// train_position_t target_pos = { .sensor = data->destination, .offset_mm = compensated_offset };
			data->destination = NULL;
			data->destination_name[0] = '\0';
			data->destination_offset_mm = 0;
			data->status = TRAIN_STATUS_IDLE;
			// if (train_position_is_at_destination(&current_pos, &target_pos, 100)) {
			// 	log_info("Train %d: Arrived at destination %s (offset %lldmm)", data->train_id,
			// 		 data->destination_name, data->destination_offset_mm);
			// 	data->destination = NULL;
			// 	data->destination_name[0] = '\0';
			// 	data->destination_offset_mm = 0;
			// 	data->status = TRAIN_STATUS_IDLE;
			// }
		}
		break;

	case TRAIN_STOP_PATH_END:
		log_info("Train %d: Stopping at end of activated path segment", data->train_id);
		train_stop(data);
		// Set state so the state machine will trigger path continuation
		data->path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = true;
		break;

	case TRAIN_STOP_REVERSAL:
		log_info("Train %d: Stopping at reversal point", data->train_id);
		train_stop(data);
		// Set state so the state machine will handle reversal
		data->path_state = PATH_STATE_AT_REVERSAL;
		break;

	case TRAIN_EMERGENCY_STOP:
		log_warn("Train %d: Emergency stop triggered - safety violation", data->train_id);
		train_emergency_stop(data);
		data->status = TRAIN_STATUS_STOPPING;
		break;
	}
}

// ############################################################################
// # Real-time Speed Tracking Functions
// ############################################################################
//
// Speed Field Semantics:
// - effective_speed: The speed currently commanded to the train hardware (immediate)
// - requested_speed: The speed requested by user/autonomous systems
// - signal_speed: The maximum allowed speed due to signal constraints
// - current_speed: The real-time actual speed considering acceleration/deceleration physics
//
// Key Differences:
// - effective_speed changes immediately when a speed command is sent
// - current_speed tracks the gradual speed change during acceleration/deceleration
// - For kinematic calculations (stopping distance, predictions), use current_speed
// - For command/control logic, use effective_speed
//

static void train_update_current_speed_gradually(train_task_data_t *data)
{
	// int count = 0;

	if (!data->kinematic_model_enabled || data->motion.actual_speed == data->motion.commanded_speed) {
		return; // No gradual update needed
	}

	kinematic_time_t current_time = Time(data->clock_server_tid);
	kinematic_time_t time_since_change = current_time - data->motion.speed_change_time;

	// Only update if enough time has passed since the last speed change
	if (time_since_change < 5) { // 50ms minimum update interval
		return;
	}

	u8 from_speed = data->motion.actual_speed;
	u8 target_speed = data->motion.commanded_speed;

	if (from_speed == target_speed) {
		train_online_calibration_start_speed_measurement(data, target_speed, from_speed < target_speed);

		return; // Already at target speed
	}

	if (target_speed == 0) {
		// Stopping - skip acceleration calculation
		data->motion.actual_speed_from_higher = false;
		data->motion.actual_speed = 0;
		train_online_calibration_stop_measurement(data);
		return;
	}

	// Get acceleration/deceleration data from kinematic model
	kinematic_accel_t acceleration =
		kinematic_model_get_acceleration(data, from_speed, data->motion.actual_speed_from_higher, target_speed);
	if (acceleration == 0) {
		// No acceleration data available, use instantaneous update as fallback
		data->motion.actual_speed_from_higher = target_speed > from_speed;
		data->motion.actual_speed = target_speed;
		log_info("Train %d: No acceleration data available, using instantaneous update", data->train_id);
		train_online_calibration_stop_measurement(data);
		return;
	}
	// if (count % 10 == 0) {
	// 	count++;
	// 	log_info("Train %d: Acceleration: %d, from_speed: %d, target_speed: %d", data->train_id, acceleration,
	// 		 from_speed, target_speed);
	// }

	// Calculate expected speed after time_since_change using kinematic equations
	// v = v0 + a*t (where acceleration is already scaled)
	kinematic_velocity_t initial_velocity =
		kinematic_model_get_velocity(data, from_speed, target_speed < from_speed);
	if (initial_velocity == 0) {
		// No velocity data available, use instantaneous update as fallback
		data->motion.actual_speed_from_higher = target_speed > from_speed;
		data->motion.actual_speed = target_speed;
		train_online_calibration_stop_measurement(data);
		return;
	}

	// Calculate current velocity: v = v0 + a*t
	kinematic_velocity_t predicted_velocity =
		kinematic_distance_from_acceleration(initial_velocity, acceleration, time_since_change);

	// Convert predicted velocity back to speed level
	// Find the closest speed level that matches this velocity
	u8 predicted_speed = from_speed;
	kinematic_velocity_t min_velocity_diff =
		(kinematic_velocity_t)KINEMATIC_VELOCITY_SCALE_FACTOR * 1000; // Large initial value

	// Search in the direction of acceleration/deceleration
	bool accelerating = (target_speed > from_speed);
	u8 search_start = accelerating ? from_speed : target_speed;
	u8 search_end = accelerating ? target_speed : from_speed;

	for (u8 speed = search_start; speed <= search_end; speed++) {
		kinematic_velocity_t speed_velocity = kinematic_model_get_velocity(data, speed, !accelerating);
		if (speed_velocity == 0)
			continue;

		kinematic_velocity_t velocity_diff = (predicted_velocity > speed_velocity) ?
							     (predicted_velocity - speed_velocity) :
							     (speed_velocity - predicted_velocity);

		if (velocity_diff < min_velocity_diff) {
			min_velocity_diff = velocity_diff;
			predicted_speed = speed;
		}
	}

	if (predicted_speed != data->motion.actual_speed) {
		data->motion.actual_speed_from_higher = predicted_speed > from_speed;
		data->motion.actual_speed = predicted_speed;

		train_online_calibration_stop_measurement(data);

		// log_debug("Train %d: Gradual speed update - current_speed: %d, target: %d, time: %lld ticks",
		// 	  data->train_id, data->motion.actual_speed, target_speed, time_since_change);
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

		// Update kinematic model when speed changes
		if (data->kinematic_model_enabled && data->motion.actual_speed != speed) {
			kinematic_time_t current_time = Time(data->clock_server_tid);

			// Start gradual speed tracking - current_speed will be updated gradually
			// effective_speed represents the commanded speed, current_speed tracks actual motion
			data->motion.speed_change_time = current_time;

			// log_debug("Train %d: Speed command changed from %d to %d at time %lld", data->train_id,
			// 	  data->motion.actual_speed, speed, current_time);
		}
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
	data->motion.actual_speed_from_higher = false;
	marklin_error_t result = train_set_speed(data, 0);
	if (result != MARKLIN_ERROR_OK) {
		return result;
	}

	// Use kinematic model for stopping time if available
	kinematic_time_t kinematic_stop_time = 0;
	if (data->kinematic_model_enabled && current_speed > 0) {
		kinematic_stop_time =
			kinematic_model_get_stop_time(data, current_speed, data->motion.actual_speed_from_higher);
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

	// data->motion.requested_speed = 0;

	// Before releasing blocks, try to reserve the stopping path for safety
	if (data->motion.current_stop_distance > 0) {
		u32 blocks_needed = train_calculate_blocks_needed_for_stopping(data);
		bool reservation_success = train_try_reserve_stopping_path(data, blocks_needed);

		if (reservation_success) {
			log_info("Train %d: Secured %d blocks for stopping path before releasing others",
				 data->train_id, blocks_needed);
		} else {
			log_warn("Train %d: Could not fully secure stopping path, proceeding with caution",
				 data->train_id);
		}
	}

	// Release all blocks except the one containing the train when stopping
	train_release_all_blocks(data, true);
	Delay(data->clock_server_tid, MS_TO_TICK(stop_time_ms));

	return MARKLIN_ERROR_OK;
}

marklin_error_t train_force_stop(train_task_data_t *data)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_error_t result = Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM,
								     MARKLIN_REVERSE_CMD + 16, data->train_id,
								     MARKLIN_TRAIN_CMD_DELAY_TICKS,
								     MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);
	result = Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, MARKLIN_REVERSE_CMD + 16,
						     data->train_id, MARKLIN_TRAIN_CMD_DELAY_TICKS,
						     MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);

	// Set immediate stop flags
	data->motion.commanded_speed = 0;
	data->motion.actual_speed_from_higher = true; // Coming from higher speed

	// Release all blocks except current one for safety
	train_release_all_blocks(data, true);

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
	marklin_error_t result = Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM,
								     MARKLIN_REVERSE_CMD + 16, data->train_id, 0,
								     MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);
	result = Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, MARKLIN_REVERSE_CMD + 16,
						     data->train_id, 0, MARKLIN_CMD_PRIORITY_CRITICAL, data->train_id);

	// Set immediate stop flags
	data->motion.commanded_speed = 0;
	data->motion.requested_speed = 0;
	data->motion.actual_speed_from_higher = true; // Coming from higher speed

	// Before releasing blocks, try to reserve the stopping path for emergency stop safety
	if (data->motion.current_stop_distance > 0) {
		u32 blocks_needed = train_calculate_blocks_needed_for_stopping(data);
		bool reservation_success = train_try_reserve_stopping_path(data, blocks_needed);

		if (reservation_success) {
			log_warn("Train %d: EMERGENCY STOP - Secured %d blocks for stopping path", data->train_id,
				 blocks_needed);
		} else {
			log_warn("Train %d: EMERGENCY STOP - Could not secure stopping path, CRITICAL SAFETY ISSUE",
				 data->train_id);
		}
	}

	// Release all blocks except current one for safety
	train_release_all_blocks(data, true);

	// Update train status
	if (data->has_active_path) {
		Marklin_FreePath(&data->current_path);
		data->has_active_path = false;
		data->path_ends_at_reversal = false;
	}
	data->path_state = PATH_STATE_NONE;
	data->status = TRAIN_STATUS_IDLE;

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
	data->path_state = PATH_STATE_NONE;

	// Use centralized path management
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

	data->path_state = PATH_STATE_NONE;
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
		data->motion.expected_sensor_count = 0;
		return;
	}

	// Query the conductor for next two sensors (it knows switch states)
	marklin_error_t result = Marklin_GetNextTwoSensors(data->motion.current_position.sensor, data->motion.direction,
							   data->motion.expected_sensors,
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
		data->motion.expected_sensor_count = 0;
	} else {
		// Calculate expected arrival times and timeout deadlines for each sensor
		kinematic_time_t current_time = Time(data->clock_server_tid);
		kinematic_velocity_t current_velocity =
			kinematic_model_get_velocity(data, data->motion.actual_speed, false);

		for (u8 i = 0; i < data->motion.expected_sensor_count; i++) {
			if (data->motion.commanded_speed == 0) {
				data->motion.expected_arrival_times[i] = 0;
				data->motion.sensor_timeout_deadlines[i] = 0;
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
			} else {
				data->motion.expected_arrival_times[i] = 0;
				data->motion.sensor_timeout_deadlines[i] = 0;
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
		// Update online calibration system (though it won't use blacklisted sensor)
		train_online_calibration_update(data, data->motion.expected_sensors[0], current_time);

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
 * Enhanced sensor validation that checks both expected sensor list and block ownership.
 * This prevents trains from processing sensor updates from blocks they don't own.
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
			const kinematic_time_t EARLY_TRIGGER_TOLERANCE_TICK = 200;

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
						log_info(
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
		log_info("Train %d: Ignoring sensor %s from unowned block", data->train_id, sensor_node->name);
	} else {
		log_info("Train %d: Ignoring sensor %s from block owned by train %d", data->train_id, sensor_node->name,
			 owner_train_id);
	}
	return false;
}

/**
 * Find the path node corresponding to a given sensor in the current path.
 * Returns NULL if no matching path node is found.
 */
static __maybe_unused path_node_t *train_find_path_node_for_sensor(train_task_data_t *data,
								   const track_node *sensor_node)
{
	if (!data || !sensor_node || !data->has_active_path) {
		return NULL;
	}

	path_node_t *path_node;
	dlist_for_each_entry(path_node, &data->current_path.nodes, path_node_t, list)
	{
		if (path_node->node == sensor_node) {
			return path_node;
		}
	}

	return NULL;
}

static void train_update_position_from_sensor(train_task_data_t *data, const track_node *sensor_node,
					      const marklin_sensor_state_t *sensor_update)
{
	if (!sensor_node) {
		return;
	}

	// Check if this is one of our expected sensors
	log_info("Train %d: Sensor %s triggered at position %s (offset: %lldmm)", data->train_id, sensor_node->name,
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
			log_info("Train %d: Sensor %s arrived ON TIME", data->train_id, sensor_node->name);
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

	// Handle offline experiment sensor updates first
	if (data->offline_experiment_active) {
		train_process_offline_sensor_update(data, sensor_node);
	}

	log_info("Train %d: Sensor %s triggered, expected: %s, %s", data->train_id, sensor_node->name,
		 data->motion.expected_sensors[0] ? data->motion.expected_sensors[0]->name : "none",
		 data->motion.expected_sensors[1] ? data->motion.expected_sensors[1]->name : "none");

	if (train_should_process_sensor_update(data, sensor_node, sensor_update)) {
		log_debug("Train %d: Expected sensor %s triggered, updating position", data->train_id,
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

			// Update online calibration system with sensor trigger
			train_online_calibration_update(data, sensor_node, trigger_time);

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
	if (data->motion.commanded_speed == 0 || data->status == TRAIN_STATUS_IDLE) {
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
			log_warn("Train %d: Sensor %s timed out (deadline exceeded)", data->train_id,
				 data->motion.expected_sensors[i] ? data->motion.expected_sensors[i]->name : "unknown");
			timed_out_sensors++;
		}
	}

	// Only trigger timeout state if ALL expected sensors have timed out
	if (timed_out_sensors > 0 && timed_out_sensors == data->motion.expected_sensor_count) {
		log_error("Train %d: ALL EXPECTED SENSORS TIMED OUT - Initiating emergency stop", data->train_id);

		// Emergency stop the train
		train_emergency_stop(data);

		// Update train status - train is stopping due to sensor timeout
		data->status = TRAIN_STATUS_STOPPING;

		// Clear all expected sensors and timeouts
		data->motion.expected_sensors[0] = NULL;
		data->motion.expected_sensors[1] = NULL;
		data->motion.expected_distances[0] = 0;
		data->motion.expected_distances[1] = 0;
		data->motion.expected_arrival_times[0] = 0;
		data->motion.expected_arrival_times[1] = 0;
		data->motion.sensor_timeout_deadlines[0] = 0;
		data->motion.sensor_timeout_deadlines[1] = 0;
		data->motion.expected_sensor_count = 0;
	}
}

// ############################################################################
// # Operating Mode Specific Functions
// ############################################################################

// Common operations for all autonomous modes
static void train_common_mode_update(train_task_data_t *data)
{
	UNUSED(data);
}

// ############################################################################
// # Random Destination Selection
// ############################################################################

#define RANDOM_DESTINATION_DELAY_MS 5000
#define RANDOM_DESTINATION_ARRIVAL_PAUSE_MS 3000 // Pause after arriving at destination
#define RANDOM_DESTINATION_MIN_DISTANCE_MM 500

static const track_node *train_select_random_destination(train_task_data_t *data)
{
	if (!data || !data->motion.current_position.sensor) {
		return NULL;
	}

	// Predefined list of destination sensors (well-distributed across the track)
	static const char *destination_names[] = {
		"E5", "E6", "C13", "C14", "C11", "C12", "B1", "B2", "A3",
		"A4", "B5", "B6",  "E1",  "E2",	 "D1",	"D2", "E7", "E8",
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
			marklin_error_t path_result_err =
				Marklin_FindPath(data->motion.current_position.sensor, candidate, data->train_id,
						 GLOBAL_ALLOW_REVERSAL, GLOBAL_USE_BLOCK_EXIT_AS_START, &path_result);

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
	train_common_mode_update(data);

	train_update_effective_speed(data);

	// Use centralized path state machine for waypoint mode
	train_path_update_state_machine(data);

	// Random destination logic: if enabled, idle, and enough time has passed, select a random destination
	if (data->random_destination_enabled && data->status == TRAIN_STATUS_IDLE && data->destination == NULL) {
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
		return 300;
	}

	kinematic_distance_t stopping_distance_mm = 0;

	// Use cached stopping distance from unified kinematic system if available
	if (data->motion.current_stop_distance > 0) {
		stopping_distance_mm = kinematic_model_get_stop_distance(data, speed, from_higher) * 1.5;
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
		data->path_state = PATH_STATE_REACHED;
		return MARKLIN_ERROR_OK;
	}

	// Set state to requesting
	data->path_state = PATH_STATE_REQUESTING;
	data->status = TRAIN_STATUS_REQUESTING_PATH;

	// Find path using block-aware pathfinding in conductor
	path_result_t path_result;
	marklin_error_t path_error = Marklin_FindPath(data->motion.current_position.sensor, destination, data->train_id,
						      allow_reverse, GLOBAL_USE_BLOCK_EXIT_AS_START, &path_result);

	if (path_error != MARKLIN_ERROR_OK) {
		log_error("Train %d: Failed to find path from %s to %s: error %d", data->train_id,
			  data->motion.current_position.sensor->name, destination->name, path_error);
		data->path_state = PATH_STATE_NONE;
		data->status = TRAIN_STATUS_IDLE;
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

	marklin_path_activation_result_t activation_result;
	kinematic_distance_t distance_needed = train_calculate_distance_needed_for_speed(
		data, data->motion.requested_speed, data->motion.actual_speed > data->motion.requested_speed);
	kinematic_distance_t distance_past_reversal = distance_needed;

	marklin_error_t activation_error = Marklin_ActivatePath(
		path_result, data->train_id, distance_needed * 2, distance_past_reversal,
		data->motion.current_position.sensor, data->motion.current_position.offset_mm, &activation_result);

	if (activation_error != MARKLIN_ERROR_OK) {
		const char *dest_name = data->destination ? data->destination->name : "unknown";
		log_error("Train %d: Failed to activate path to %s: error %d", data->train_id, dest_name,
			  activation_error);

		Marklin_FreePath(path_result);
		data->path_state = PATH_STATE_NONE;
		data->status = TRAIN_STATUS_IDLE;
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
	data->path_state = PATH_STATE_ACTIVE;
	data->status = TRAIN_STATUS_MOVING;
	data->path_ends_at_reversal = activation_result.ends_at_reversal;

	// Initialize activation distance tracking for re-activation
	data->last_activation_distance = distance_needed;

	// Store reversal information for use in stopping calculations
	if (activation_result.ends_at_reversal) {
		log_debug("Train %d: Path ends at reversal point at node %s, will stop past reversal for clearance",
			  data->train_id,
			  activation_result.reversal_node ? activation_result.reversal_node->name : "unknown");
		if (activation_result.reversal_next_node) {
			log_debug("Train %d: Next node after reversal: %s", data->train_id,
				  activation_result.reversal_next_node->name);
		}
		if (activation_result.blocks_reserved_for_reversal > 0) {
			log_debug("Train %d: Reserved %d blocks (%lld mm) for reversal safety", data->train_id,
				  activation_result.blocks_reserved_for_reversal,
				  activation_result.reversal_safety_distance);
		}
	}

	// Store activation result metadata for progressive path handling
	data->last_activation_result = activation_result;
	data->activation_stop_reason = activation_result.stop_reason;
	data->activation_end_point = activation_result.furthest_activated_node;
	data->at_reversal_point = activation_result.ends_at_reversal;
	data->reversal_node = activation_result.reversal_node;
	data->reversal_next_node = activation_result.reversal_next_node;

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
	if (activation_result.ends_at_reversal) {
		data->path_state = PATH_STATE_AT_REVERSAL;
	} else {
		// Always use PATH_STATE_ACTIVE (partial vs full activation determined by needs_path_continuation flag)
		data->path_state = PATH_STATE_ACTIVE;
	}

	return MARKLIN_ERROR_OK;
}

void train_path_update_state_machine(train_task_data_t *data)
{
	if (!data) {
		log_error("train_path_update_state_machine: Invalid data", data->train_id);
		return;
	}

	switch (data->path_state) {
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
			data->path_state = PATH_STATE_REACHED;
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
		data->path_state = PATH_STATE_NONE;
		break;

	case PATH_STATE_AT_REVERSAL:
		// Train reached reversal point, need to handle reversal
		if (data->at_reversal_point) {
			// Train has stopped at reversal point, start reversal sequence
			train_handle_reversal_sequence(data);
		}
		break;

	case PATH_STATE_REVERSING:
		// Train is in process of reversing
		// Check if reversal is complete and continue with path
		if (data->status == TRAIN_STATUS_IDLE && data->needs_path_continuation) {
			u64 current_tick = Time(data->clock_server_tid);
			if (current_tick - data->last_path_continuation_tick >=
			    MS_TO_TICK(TRAIN_PATH_CONTINUATION_INTERVAL_MS)) {
				data->last_path_continuation_tick = current_tick;
				// Reversal complete, continue with remaining path
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

	kinematic_distance_t train_length = 2000;
	kinematic_distance_t base_offset = target_offset;

	// Add reversal clearance if path ends at reversal point
	if (data->path_ends_at_reversal && data->has_active_path) {
		base_offset += PATH_MIN_REVERSAL_LENGTH;
		// log_debug(
		// "Train %d: Added reversal clearance offset (%d mm) to stopping calculation for node clearance",
		// data->train_id, PATH_MIN_REVERSAL_LENGTH);
	}

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
	kinematic_velocity_t current_velocity = kinematic_model_get_velocity(data, data->motion.actual_speed, false);
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
	if (data->motion.actual_speed > 0 || data->motion.commanded_speed > 0) {
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
		log_info("Train %d: Released block containing sensor %s", data->train_id, sensor_node->name);
	} else if (result == MARKLIN_ERROR_NOT_OWNER || result == MARKLIN_ERROR_NOT_FOUND) {
		log_info("Train %d: No block to release for sensor %s (not owned or not found)", data->train_id,
			 sensor_node->name);
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

	// log_info("Train %d: Continuing path activation from current position", data->train_id);

	// Re-activate the current path with updated distance requirements
	marklin_path_activation_result_t continuation_result;
	kinematic_distance_t distance_needed =
		train_calculate_distance_needed_for_speed(data, data->motion.requested_speed,
							  data->motion.actual_speed > data->motion.requested_speed) *
		2;

	marklin_error_t activation_error = Marklin_ActivatePath(&data->current_path, data->train_id, distance_needed,
								distance_needed, data->motion.current_position.sensor,
								data->motion.current_position.offset_mm,
								&continuation_result);

	// log_info("Train %d: Continuing path activation with distance needed %lldmm", data->train_id, distance_needed);
	if (activation_error != MARKLIN_ERROR_OK) {
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
		continuation_result.next_expected_distance, data->activation_end_point->name);

	train_calculate_next_sensors(data);

	// Add newly reserved blocks to train's tracking state
	for (u32 i = 0; i < continuation_result.blocks_reserved && i < MAX_TRACK_BLOCKS; i++) {
		if (continuation_result.reserved_block_nodes[i] != NULL) {
			train_add_reserved_block(data, continuation_result.reserved_block_nodes[i]);
			log_debug("Train %d: Added reserved block node %s to tracking during continuation (index %u)",
				  data->train_id, continuation_result.reserved_block_nodes[i]->name, i);
		}
	}

	// log_info("Train %d: Added %u additional reserved blocks during continuation", data->train_id,
	// 	 continuation_result.blocks_reserved);

	// Determine next state based on continuation result
	if (continuation_result.ends_at_reversal) {
		if (data->motion.current_position.sensor == continuation_result.reversal_node) {
			data->path_state = PATH_STATE_AT_REVERSAL;
			data->at_reversal_point = true;
			data->reversal_node = continuation_result.reversal_node;
			data->reversal_next_node = continuation_result.reversal_next_node;
			log_info("Train %d: Path continuation reached reversal point", data->train_id);
		}
	} else if (continuation_result.stop_reason == PATH_ACTIVATION_STOP_MAX_BLOCKS_REACHED) {
		data->path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = true;
		log_info("Train %d: Path continuation still partial, more needed", data->train_id);
	} else {
		data->path_state = PATH_STATE_ACTIVE;
		data->needs_path_continuation = false;
		log_info("Train %d: Path continuation completed to destination", data->train_id);
	}

	// Resume movement
	data->status = TRAIN_STATUS_MOVING;
	return MARKLIN_ERROR_OK;
}

marklin_error_t train_handle_reversal_sequence(train_task_data_t *data)
{
	if (!data || !data->at_reversal_point || !data->reversal_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	log_info("Train %d: Starting reversal sequence at %s", data->train_id, data->reversal_node->name);

	// Step 1: Move past reversal point if we have a next node
	if (data->reversal_next_node) {
		log_info("Train %d: Moving past reversal point to %s for clearance", data->train_id,
			 data->reversal_next_node->name);
		train_set_speed(data, 4);
		Delay(data->clock_server_tid, MS_TO_TICK(300));
	}

	// Step 2: Set state to reversing
	data->path_state = PATH_STATE_REVERSING;

	// Step 3: Execute the actual reversal command
	marklin_error_t reverse_result = train_reverse(data);
	if (reverse_result != MARKLIN_ERROR_OK) {
		log_error("Train %d: Failed to execute reversal: error %d", data->train_id, reverse_result);
		return reverse_result;
	}

	log_info("Train %d: Reversal command executed, direction reversed", data->train_id);

	// Step 4: Continue with remaining path to final destination
	// After reversal, we need to continue the path activation
	data->at_reversal_point = false;
	data->needs_path_continuation = true;
	data->path_state = PATH_STATE_ACTIVE;

	return MARKLIN_ERROR_OK;
}
