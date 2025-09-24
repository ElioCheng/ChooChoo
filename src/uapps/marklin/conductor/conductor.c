#include "marklin/conductor/conductor.h"
#include "marklin/conductor/api.h"
#include "marklin/conductor/block.h"
#include "marklin/conductor/path.h"
#include "marklin/command/api.h"
#include "marklin/error.h"
#include "marklin/topology/api.h"
#include "compiler.h"
#include "marklin/train/kinematics.h"
#include "marklin/train2/train.h"
#include "name.h"
#include "syscall.h"
#include "clock.h"
#include "clock_server.h"
#include "string.h"
#include "marklin/conductor/sensor.h"
#include "marklin/conductor/switch.h"
#include "marklin/msgqueue/api.h"
#include "marklin/topology/track.h"
// #include "klog.h"

#define LOG_MODULE "conductor"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

// ############################################################################
// # Public API Implementation
// ############################################################################

marklin_error_t Marklin_SetSwitch(u8 switch_id, track_direction direction, u8 disengage_solenoid, bool force)
{
	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_SET_SWITCH;
	request.set_switch.switch_id = switch_id;
	request.set_switch.direction = direction;
	request.set_switch.disengage_solenoid = disengage_solenoid;
	request.set_switch.force = force;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_GetSensorStates(marklin_sensor_state_t *sensors, u32 count)
{
	if (!sensors || count == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_GET_SENSOR_STATES;
	request.get_sensor_states.sensors = sensors;
	request.get_sensor_states.count = count;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

// ############################################################################
// # Deadlock Detection Implementation
// ############################################################################

void conductor_init_deadlock_detection(conductor_task_data_t *data)
{
	if (!data) {
		return;
	}

	// Initialize failure tracking
	for (u32 i = 0; i < MAX_TRAINS; i++) {
		data->recent_failures[i].train_id = 0;
		data->recent_failures[i].requested_from = NULL;
		data->recent_failures[i].requested_to = NULL;
		data->recent_failures[i].blocking_block_count = 0;
		data->recent_failures[i].failure_time = 0;
		data->recent_failures[i].active = false;

		for (u32 j = 0; j < MAX_BLOCKING_BLOCKS; j++) {
			data->recent_failures[i].blocking_blocks[j] = NULL;
		}
	}
	data->failure_count = 0;

	// Initialize deadlock context
	for (u32 i = 0; i < MAX_TRAINS; i++) {
		data->deadlock_context.deadlocked_trains[i] = 0;
	}
	data->deadlock_context.deadlocked_count = 0;
	data->deadlock_context.detection_time = 0;
	data->deadlock_context.resolution_priority_train = 0;
	data->deadlock_context.resolution_in_progress = false;

	log_info("Deadlock detection system initialized");
}

void conductor_record_path_failure(conductor_task_data_t *data, u8 train_id, const track_node *from,
				   const track_node *to, track_block_t **blocking_blocks, u32 blocking_count)
{
	if (!data || train_id == 0 || !from || !to) {
		return;
	}

	// Clean up old failures first
	conductor_cleanup_old_failures(data);

	// Find an available slot (prefer reusing train's existing slot)
	int slot = -1;
	for (u32 i = 0; i < MAX_TRAINS; i++) {
		if (data->recent_failures[i].train_id == train_id) {
			slot = i;
			break;
		}
	}

	// If no existing slot, find a free one
	if (slot == -1) {
		for (u32 i = 0; i < MAX_TRAINS; i++) {
			if (!data->recent_failures[i].active) {
				slot = i;
				break;
			}
		}
	}

	if (slot == -1) {
		log_warn("No available slot to record path failure for train %d", train_id);
		return;
	}

	failed_path_request_t *failure = &data->recent_failures[slot];
	failure->train_id = train_id;
	failure->requested_from = from;
	failure->requested_to = to;
	failure->failure_time = Time(data->clock_server_tid);
	failure->active = true;

	// Copy blocking blocks (up to limit)
	failure->blocking_block_count = (blocking_count > MAX_BLOCKING_BLOCKS) ? MAX_BLOCKING_BLOCKS : blocking_count;
	for (u32 i = 0; i < failure->blocking_block_count; i++) {
		failure->blocking_blocks[i] = blocking_blocks[i];
	}

	// Clear unused slots
	for (u32 i = failure->blocking_block_count; i < MAX_BLOCKING_BLOCKS; i++) {
		failure->blocking_blocks[i] = NULL;
	}

	log_info("Recorded path failure for train %d: %s -> %s (%d blocking blocks)", train_id, from->name, to->name,
		 failure->blocking_block_count);
}

void conductor_cleanup_old_failures(conductor_task_data_t *data)
{
	if (!data) {
		return;
	}

	u64 current_time = Time(data->clock_server_tid);
	u32 cleaned = 0;

	for (u32 i = 0; i < MAX_TRAINS; i++) {
		if (data->recent_failures[i].active) {
			if (current_time - data->recent_failures[i].failure_time > DEADLOCK_DETECTION_WINDOW_MS) {
				data->recent_failures[i].active = false;
				data->recent_failures[i].train_id = 0;
				cleaned++;
			}
		}
	}

	if (cleaned > 0) {
		log_debug("Cleaned up %d old path failure records", cleaned);
	}
}

bool conductor_detect_deadlock(conductor_task_data_t *data)
{
	if (!data) {
		return false;
	}

	conductor_cleanup_old_failures(data);

	// Count active failures
	u32 active_failures = 0;
	for (u32 i = 0; i < MAX_TRAINS; i++) {
		if (data->recent_failures[i].active) {
			active_failures++;
		}
	}

	// Need at least 2 trains failing to have a deadlock
	if (active_failures < 2) {
		return false;
	}

	// Check for mutual blocking pattern
	for (u32 i = 0; i < MAX_TRAINS; i++) {
		if (!data->recent_failures[i].active)
			continue;

		for (u32 j = i + 1; j < MAX_TRAINS; j++) {
			if (!data->recent_failures[j].active)
				continue;

			// Check if train i's blocks are blocking train j, and vice versa
			bool i_blocks_j = false;
			bool j_blocks_i = false;

			// Check if train i's blocks are needed by train j
			for (u32 bi = 0; bi < data->recent_failures[i].blocking_block_count; bi++) {
				track_block_t *block_i = data->recent_failures[i].blocking_blocks[bi];
				if (block_i && block_i->owner_train_id == data->recent_failures[i].train_id) {
					for (u32 bj = 0; bj < data->recent_failures[j].blocking_block_count; bj++) {
						if (data->recent_failures[j].blocking_blocks[bj] == block_i) {
							i_blocks_j = true;
							break;
						}
					}
				}
			}

			// Check if train j's blocks are needed by train i
			for (u32 bj = 0; bj < data->recent_failures[j].blocking_block_count; bj++) {
				track_block_t *block_j = data->recent_failures[j].blocking_blocks[bj];
				if (block_j && block_j->owner_train_id == data->recent_failures[j].train_id) {
					for (u32 bi = 0; bi < data->recent_failures[i].blocking_block_count; bi++) {
						if (data->recent_failures[i].blocking_blocks[bi] == block_j) {
							j_blocks_i = true;
							break;
						}
					}
				}
			}

			// Mutual blocking detected
			if (i_blocks_j && j_blocks_i) {
				log_warn("Deadlock detected between train %d and train %d",
					 data->recent_failures[i].train_id, data->recent_failures[j].train_id);

				// Set up deadlock context
				data->deadlock_context.deadlocked_count = 2;
				data->deadlock_context.deadlocked_trains[0] = data->recent_failures[i].train_id;
				data->deadlock_context.deadlocked_trains[1] = data->recent_failures[j].train_id;
				data->deadlock_context.detection_time = Time(data->clock_server_tid);
				data->deadlock_context.resolution_in_progress = false;

				return true;
			}
		}
	}

	return false;
}

bool conductor_resolve_deadlock(conductor_task_data_t *data, u8 requesting_train_id,
				const track_block_t ***excluded_blocks, u32 *excluded_count)
{
	if (!data || !excluded_blocks || !excluded_count) {
		return false;
	}

	*excluded_blocks = NULL;
	*excluded_count = 0;

	if (!data->deadlock_context.resolution_in_progress) {
		return false;
	}

	// Find the other train in the deadlock
	u8 other_train_id = 0;
	for (u32 i = 0; i < data->deadlock_context.deadlocked_count; i++) {
		if (data->deadlock_context.deadlocked_trains[i] != requesting_train_id) {
			other_train_id = data->deadlock_context.deadlocked_trains[i];
			break;
		}
	}

	if (other_train_id == 0) {
		return false;
	}

	// Priority resolution: lower train ID gets higher priority (simple heuristic)
	u8 priority_train = (requesting_train_id < other_train_id) ? requesting_train_id : other_train_id;

	if (requesting_train_id != priority_train) {
		// This train should wait - exclude the other train's blocks
		for (u32 i = 0; i < MAX_TRAINS; i++) {
			if (data->recent_failures[i].active && data->recent_failures[i].train_id == other_train_id) {
				*excluded_blocks = (const track_block_t **)data->recent_failures[i].blocking_blocks;
				*excluded_count = data->recent_failures[i].blocking_block_count;

				log_info("Deadlock resolution: train %d excluded from %d blocks owned by train %d",
					 requesting_train_id, *excluded_count, other_train_id);
				return true;
			}
		}
	}

	return false;
}

marklin_error_t Marklin_FindPath(const track_node *from, const track_node *to, u8 train_id, bool allow_reversal,
				 bool use_block_exit_start, const track_block_t **excluded_blocks, u32 excluded_count,
				 path_result_t *result)
{
	if (!from || !to || !result) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_FIND_PATH;
	request.find_path.from = from;
	request.find_path.to = to;
	request.find_path.train_id = train_id;
	request.find_path.allow_reversal = allow_reversal;
	request.find_path.use_block_exit_start = use_block_exit_start;
	request.find_path.excluded_blocks = excluded_blocks;
	request.find_path.excluded_count = excluded_count;
	request.find_path.result = result;

	int comm_result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (comm_result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

void Marklin_FreePath(path_result_t *result)
{
	if (!result) {
		return;
	}

	// If no pool is assigned, path is already cleaned up
	if (!result->pool) {
		return;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		log_error("Path cleanup: conductor server not found");
		return;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_FREE_PATH;
	request.free_path.path = result;

	int send_result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));
	if (send_result < 0) {
		log_error("Path cleanup: failed to send cleanup request to conductor");
		return;
	}

	if (reply.error != MARKLIN_ERROR_OK) {
		log_error("Path cleanup: conductor returned error %d", reply.error);
	}
}

marklin_error_t Marklin_GetNextSensors(const track_node *current_location, train_direction_t direction,
				       const track_node **expected_sensor, kinematic_distance_t *expected_distance)
{
	if (!current_location || !expected_sensor) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_GET_NEXT_SENSORS;
	request.get_next_sensors.current_location = current_location;
	request.get_next_sensors.direction = direction;
	request.get_next_sensors.expected_sensor = expected_sensor;
	request.get_next_sensors.expected_distance = expected_distance;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_GetNextTwoSensors(const track_node *current_location, train_direction_t direction,
					  const track_node **sensors, kinematic_distance_t *distances, u8 *count)
{
	if (!current_location || !sensors || !distances || !count) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_GET_NEXT_TWO_SENSORS;
	request.get_next_two_sensors.current_location = current_location;
	request.get_next_two_sensors.direction = direction;
	request.get_next_two_sensors.sensors = sensors;
	request.get_next_two_sensors.distances = distances;
	request.get_next_two_sensors.count = count;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_CalculateTrackDistance(const track_node *from, const track_node *to, u8 train_id,
					       kinematic_distance_t *raw_distance,
					       kinematic_distance_t *effective_distance)
{
	if (!from || !to || !raw_distance || !effective_distance) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_CALCULATE_DISTANCE;
	request.calculate_distance.from = from;
	request.calculate_distance.to = to;
	request.calculate_distance.train_id = train_id;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	if (reply.error == MARKLIN_ERROR_OK) {
		*raw_distance = reply.calculate_distance.raw_distance;
		*effective_distance = reply.calculate_distance.effective_distance;
	}

	return reply.error;
}

marklin_error_t Marklin_ActivatePath(path_result_t *path, u8 train_id, kinematic_distance_t max_distance_to_reserve,
				     const track_node *current_sensor, kinematic_distance_t current_offset_mm,
				     marklin_path_activation_result_t *result)
{
	if (!path || train_id == 0 || !result) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_ACTIVATE_PATH;
	request.activate_path.path = path;
	request.activate_path.train_id = train_id;
	request.activate_path.max_distance_to_reserve = max_distance_to_reserve;
	request.activate_path.current_sensor = current_sensor;
	request.activate_path.current_offset_mm = current_offset_mm;
	request.activate_path.result = result;

	int comm_result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (comm_result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_ReleaseTrainBlocks(u8 train_id, const track_node *keep_block_node)
{
	if (train_id == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_RELEASE_TRAIN_BLOCKS;
	request.release_train_blocks.train_id = train_id;
	request.release_train_blocks.keep_block_node = keep_block_node;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));
	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_ReleaseSpecificBlock(u8 train_id, const track_node *block_node,
					     const track_node *current_block_node)
{
	if (train_id == 0 || !block_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_RELEASE_SPECIFIC_BLOCK;
	request.release_specific_block.train_id = train_id;
	request.release_specific_block.block_node = block_node;
	request.release_specific_block.current_block_node = current_block_node;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));
	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_ReserveSpecificBlock(u8 train_id, const track_node *block_node)
{
	if (train_id == 0 || !block_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_RESERVE_SPECIFIC_BLOCK;
	request.reserve_specific_block.train_id = train_id;
	request.reserve_specific_block.block_node = block_node;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));
	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_CheckBlockOwnership(u8 train_id, const track_node *block_node, bool *owns_block,
					    u8 *owner_train_id)
{
	if (train_id == 0 || !block_node || !owns_block || !owner_train_id) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_conductor_request_t request;
	marklin_conductor_reply_t reply;

	request.type = MARKLIN_CONDUCTOR_REQ_CHECK_BLOCK_OWNERSHIP;
	request.check_block_ownership.train_id = train_id;
	request.check_block_ownership.block_node = block_node;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));
	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	if (reply.error == MARKLIN_ERROR_OK) {
		*owns_block = reply.check_block_ownership.owns_block;
		*owner_train_id = reply.check_block_ownership.owner_train_id;
	}

	return reply.error;
}

// ############################################################################
// # Private Implementation
// ############################################################################

// Configuration constants
#define MARKLIN_SENSOR_POLL_INTERVAL_MS 100
#define MAX_ROUTE_RESERVATIONS 32
#define MAX_SIGNAL_HISTORY 80

// Global conductor data pointer for sensor processing
conductor_task_data_t *g_conductor_data = NULL;

// ############################################################################
// # Forward Declarations
// ############################################################################

// Initialization functions
static void conductor_init_task_data(conductor_task_data_t *data);

// Main loop functions
static void conductor_main_loop(conductor_task_data_t *data);
static void conductor_process_request(conductor_task_data_t *data, int sender_tid,
				      marklin_conductor_request_t *request);

// Next sensor calculation
static marklin_error_t conductor_calculate_next_sensors(const track_node *current_location, train_direction_t direction,
							const track_node **expected_sensor,
							kinematic_distance_t *expected_distance);

static marklin_error_t conductor_calculate_next_two_sensors(const track_node *current_location,
							    train_direction_t direction, const track_node **sensors,
							    kinematic_distance_t *distances, u8 *count);

static const track_node *conductor_find_next_sensor_on_edge(const track_node *start_node);

// Track distance calculation
static marklin_error_t conductor_calculate_track_distance(const track_node *from, const track_node *to, u8 train_id,
							  kinematic_distance_t *raw_distance,
							  kinematic_distance_t *effective_distance);

// Path activation
static marklin_error_t handle_activate_path_request(conductor_task_data_t *data,
						    const marklin_conductor_request_t *request);

// Block release handlers
static marklin_error_t handle_release_train_blocks_request(conductor_task_data_t *data,
							   const marklin_conductor_request_t *request);
static marklin_error_t handle_release_specific_block_request(conductor_task_data_t *data,
							     const marklin_conductor_request_t *request);
static marklin_error_t handle_reserve_specific_block_request(conductor_task_data_t *data,
							     const marklin_conductor_request_t *request);
// Block ownership check handler
static marklin_error_t handle_check_block_ownership_request(conductor_task_data_t *data,
							    const marklin_conductor_request_t *request,
							    marklin_conductor_reply_t *reply);
// Path cleanup handler
static marklin_error_t handle_free_path_request(conductor_task_data_t *data,
						const marklin_conductor_request_t *request);

// ############################################################################
// # Initialization Functions
// ############################################################################

void conductor_init_sensor_lookup(conductor_task_data_t *data)
{
	// Clear the lookup table
	memset(data->sensor_lookup, 0, sizeof(data->sensor_lookup));
	data->sensor_count = 0;

	// Iterate through all track nodes and collect sensors
	for (int i = 0; i < data->track_size; i++) {
		const track_node *node = &data->track_nodes[i];

		if (node->type == NODE_SENSOR && data->sensor_count < MARKLIN_SENSOR_BANK_COUNT * 16) {
			sensor_lookup_entry_t *entry = &data->sensor_lookup[data->sensor_count];

			entry->sensor_node = node;
			entry->state.bank = marklin_parse_sensor_bank_from_name(node->name);
			entry->state.sensor_id = marklin_parse_sensor_id_from_name(node->name);
			if (entry->state.bank == 0xff || entry->state.sensor_id == 0xff) {
				Panic("Invalid sensor name: %s", node->name);
			}

			entry->state.triggered = 0;
			entry->state.last_triggered_tick = 0;

			data->sensor_count++;
		}
	}
}

void conductor_init_switch_lookup(conductor_task_data_t *data)
{
	// Clear the lookup table
	memset(data->switch_lookup, 0, sizeof(data->switch_lookup));
	data->switch_count = 0;

	// Iterate through all track nodes and collect switches (NODE_BRANCH)
	for (int i = 0; i < data->track_size; i++) {
		const track_node *node = &data->track_nodes[i];

		if (node->type == NODE_BRANCH && data->switch_count < MARKLIN_SWITCH_MAX_COUNT) {
			switch_lookup_entry_t *entry = &data->switch_lookup[data->switch_count];

			entry->switch_node = node;

			entry->state.switch_id = node->num;

			entry->state.direction = DIR_STRAIGHT; // Initialize to straight
			entry->state.last_changed_tick = 0;

			data->switch_count++;
		}
	}
}

void conductor_init_blacklist_cache(conductor_task_data_t *data)
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

static void conductor_init_task_data(conductor_task_data_t *data)
{
	data->clock_server_tid = -1;
	data->command_server_tid = -1;
	data->track_nodes = NULL;
	data->track_size = 0;
	data->sensor_count = 0;
	data->switch_count = 0;
	data->track_type = 0;

	data->track_size = Marklin_GetTrackNodes(&data->track_nodes, &data->track_type);

	if (!data->track_nodes || data->track_size <= 0) {
		Panic("Track nodes not found @%p (size: %d)", data->track_nodes, data->track_size);
	}

	conductor_init_sensor_lookup(data);
	conductor_init_switch_lookup(data);

	conductor_init_blocks(data);

	conductor_init_blacklist_cache(data);

	// Initialize deadlock detection
	conductor_init_deadlock_detection(data);

	// Initialize path pools
	path_pools_init(data->path_pools, &data->free_path_pools);

	// Initialize reversal blacklist
	path_init_reversal_blacklist();
}

// ############################################################################
// # Next Sensor Calculation
// ############################################################################

static const track_node *conductor_find_next_sensor_on_edge(const track_node *start_node)
{
	if (!start_node) {
		return NULL;
	}

	const track_node *next_node = start_node;
	int max_hops = 50;

	while (next_node && max_hops-- > 0) {
		if (next_node->type == NODE_SENSOR && next_node != start_node) {
			return next_node;
		}

		switch (next_node->type) {
		case NODE_BRANCH: {
			switch_lookup_entry_t *switch_entry = conductor_get_switch_lookup_entry(next_node->num);
			if (!switch_entry) {
				Panic("Switch: Switch lookup entry returns NULL for switch %d", next_node->num);
			}
			track_direction switch_dir = switch_entry->state.direction;
			log_info("Find next sensor from %s: Next node: %s, following switch %d, direction: %d",
				 start_node->name, next_node->name, next_node->num, switch_dir);
			next_node = next_node->edge[switch_dir].dest;
			break;
		}
		case NODE_SENSOR:
		default:
			next_node = next_node->edge[DIR_AHEAD].dest;
			break;
		}
	}

	return NULL;
}

static marklin_error_t conductor_calculate_next_sensors(const track_node *current_location, train_direction_t direction,
							const track_node **expected_sensor,
							kinematic_distance_t *expected_distance)
{
	if (!current_location || !expected_sensor || !expected_distance) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	*expected_sensor = NULL;
	*expected_distance = 0;
	const track_node *current_node = current_location;
	if (direction == TRAIN_DIRECTION_REVERSE && current_node->reverse) {
		current_node = current_node->reverse;
	}

	if (!current_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	const track_node *next_sensor = conductor_find_next_sensor_on_edge(current_node);

	if (next_sensor) {
		*expected_sensor = next_sensor;
		kinematic_distance_t dummy_effective_distance;
		conductor_calculate_track_distance(current_node, next_sensor, 0, expected_distance,
						   &dummy_effective_distance);
	}

	return MARKLIN_ERROR_OK;
}

static marklin_error_t conductor_calculate_next_two_sensors(const track_node *current_location,
							    train_direction_t direction, const track_node **sensors,
							    kinematic_distance_t *distances, u8 *count)
{
	if (!current_location || !sensors || !distances || !count) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	*count = 0;
	sensors[0] = NULL;
	sensors[1] = NULL;
	distances[0] = 0;
	distances[1] = 0;

	const track_node *current_node = current_location;
	if (direction == TRAIN_DIRECTION_REVERSE && current_node->reverse) {
		current_node = current_node->reverse;
	}

	if (!current_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Find first sensor
	const track_node *first_sensor = conductor_find_next_sensor_on_edge(current_node);
	if (!first_sensor) {
		return MARKLIN_ERROR_OK; // No sensors found, count stays 0
	}

	sensors[0] = first_sensor;
	kinematic_distance_t dummy_effective_distance;
	conductor_calculate_track_distance(current_node, first_sensor, 0, &distances[0], &dummy_effective_distance);
	*count = 1;

	// Find second sensor by continuing from the first sensor
	const track_node *second_sensor = conductor_find_next_sensor_on_edge(first_sensor);
	if (second_sensor) {
		sensors[1] = second_sensor;
		conductor_calculate_track_distance(current_node, second_sensor, 0, &distances[1],
						   &dummy_effective_distance);
		*count = 2;
	}

	return MARKLIN_ERROR_OK;
}

// ############################################################################
// # Path Activation Implementation
// ############################################################################

static marklin_error_t conductor_set_switches_in_path(path_result_t *path, u8 train_id, conductor_task_data_t *data)
{
	if (!path || train_id == 0 || !data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// log_info("SWITCH_PASS: Starting switch setting pass for train %d", train_id);

	const u32 MAX_SAFE_PATH_NODES = 1000; // Reasonable upper bound for path length

	// Traverse path from start to end (forward direction) to set switches
	struct dlist_node *node = path->nodes.next;
	u32 switches_set = 0;
	u32 switches_failed = 0;
	u32 iterations = 0;

	while (node != &path->nodes && iterations < MAX_SAFE_PATH_NODES) {
		// Validate node pointer before dereferencing
		if (!node || (u64)node < 0x1000) {
			log_error("SWITCH_PASS: Corrupted node pointer at iteration %u", iterations);
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}

		path_node_t *current_path_node = dlist_entry(node, path_node_t, list);

		// Only process branch nodes (switches)
		if (current_path_node->node->type == NODE_BRANCH) {
			// log_info("SWITCH_PASS: Found branch node %s, searching for block owned by train %d",
			// 	 current_path_node->node->name ? current_path_node->node->name : "unnamed", train_id);
			// log_debug("SWITCH_PASS: Switch direction for %s: %d (%s)",
			// 	  current_path_node->node->name ? current_path_node->node->name : "unnamed",
			// 	  current_path_node->switch_dir,
			// 	  current_path_node->switch_dir == DIR_STRAIGHT ? "STRAIGHT" :
			// 	  current_path_node->switch_dir == DIR_CURVED	? "CURVED" :
			// 							  "UNKNOWN");

			// Find any block containing this turnout that is owned by this train
			track_block_t *turnout_block = conductor_find_block_containing_turnout_owned_by_train(
				current_path_node->node, train_id, data);

			if (turnout_block) {
				// log_info("SWITCH_PASS: Setting turnout %s to %s in block %d for train %d",
				// 	 current_path_node->node->name ? current_path_node->node->name : "unnamed",
				// 	 current_path_node->switch_dir == DIR_STRAIGHT ? "STRAIGHT" : "CURVED",
				// 	 turnout_block->block_id, train_id);

				marklin_error_t switch_result =
					conductor_set_turnout_in_block(turnout_block, current_path_node->node,
								       current_path_node->switch_dir, train_id);

				if (switch_result != MARKLIN_ERROR_OK) {
					log_warn("SWITCH_PASS: Failed to set turnout %s in block %d: error %d",
						 current_path_node->node->name ? current_path_node->node->name :
										 "unnamed",
						 turnout_block->block_id, switch_result);
					switches_failed++;
				} else {
					// log_info("SWITCH_PASS: Successfully set turnout %s in block %d",
					// 	 current_path_node->node->name ? current_path_node->node->name :
					// 					 "unnamed",
					// 	 turnout_block->block_id);
					switches_set++;
				}
			} else {
				// log_warn("SWITCH_PASS: Turnout %s not found in any block owned by train %d",
				//  current_path_node->node->name ? current_path_node->node->name : "unnamed",
				//  train_id);
				switches_failed++;
			}
		}

		// Safe iteration: validate next pointer and advance
		if (!node->next || (u64)node->next < 0x1000) {
			log_error("SWITCH_PASS: Corrupted next node pointer at iteration %u", iterations);
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}

		node = node->next;
		iterations++;
	}

	// Check for iteration limit
	if (iterations >= MAX_SAFE_PATH_NODES) {
		log_error("SWITCH_PASS: Hit iteration limit for train %d", train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Second pass: Handle merge nodes that require reverse turnout setting
	log_info("SWITCH_PASS: Starting merge node processing for train %d", train_id);
	
	node = path->nodes.next;
	iterations = 0;
	u32 merge_switches_set = 0;
	u32 merge_switches_failed = 0;

	while (node != &path->nodes && iterations < MAX_SAFE_PATH_NODES) {
		// Validate node pointer before dereferencing
		if (!node || (u64)node < 0x1000) {
			log_error("MERGE_PASS: Corrupted node pointer at iteration %u", iterations);
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}

		path_node_t *current_path_node = dlist_entry(node, path_node_t, list);

		// Only process merge nodes
		if (current_path_node->node->type == NODE_MERGE && current_path_node->node->reverse) {
			const track_node *reverse_node = current_path_node->node->reverse;
			
			// Ensure the reverse node is a branch (turnout)
			if (reverse_node->type == NODE_BRANCH) {
				// log_info("MERGE_PASS: Found merge node %s with reverse turnout %s",
				// 	 current_path_node->node->name ? current_path_node->node->name : "unnamed",
				// 	 reverse_node->name ? reverse_node->name : "unnamed");

				// Find any block containing this turnout that is owned by this train
				track_block_t *turnout_block = conductor_find_block_containing_turnout_owned_by_train(
					reverse_node, train_id, data);

				if (turnout_block) {
					// Determine the correct direction by looking at the next node in the path
					track_direction reverse_dir = DIR_STRAIGHT;
					
					// Check if we have a next node to determine direction
					if (node->next != &path->nodes) {
						path_node_t *next_path_node = dlist_entry(node->next, path_node_t, list);
						const track_node *next_node_reverse = next_path_node->node->reverse;

						// If the curved edge of the reverse turnout leads to the next node's reverse,
						// then we need to set the turnout to curved
						if (next_node_reverse && reverse_node->edge[DIR_CURVED].dest == next_node_reverse) {
							reverse_dir = DIR_CURVED;
						}
					}

					// log_info("MERGE_PASS: Setting turnout %s to %s in block %d for train %d (merge node processing)",
					// 	 reverse_node->name ? reverse_node->name : "unnamed",
					// 	 reverse_dir == DIR_STRAIGHT ? "STRAIGHT" : "CURVED",
					// 	 turnout_block->block_id, train_id);

					marklin_error_t switch_result =
						conductor_set_turnout_in_block(turnout_block, reverse_node, reverse_dir, train_id);

					if (switch_result != MARKLIN_ERROR_OK) {
						log_warn("MERGE_PASS: Failed to set turnout %s in block %d: error %d",
							 reverse_node->name ? reverse_node->name : "unnamed",
							 turnout_block->block_id, switch_result);
						merge_switches_failed++;
					} else {
						// log_info("MERGE_PASS: Successfully set turnout %s in block %d (from merge node)",
						// 	 reverse_node->name ? reverse_node->name : "unnamed",
						// 	 turnout_block->block_id);
						merge_switches_set++;
					}
				} else {
					// log_warn("MERGE_PASS: Turnout %s (reverse of merge node %s) not found in any block owned by train %d",
					// 	 reverse_node->name ? reverse_node->name : "unnamed",
					// 	 current_path_node->node->name ? current_path_node->node->name : "unnamed",
					// 	 train_id);
					merge_switches_failed++;
				}
			}
		}

		// Safe iteration: validate next pointer and advance
		if (!node->next || (u64)node->next < 0x1000) {
			log_error("MERGE_PASS: Corrupted next node pointer at iteration %u", iterations);
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}

		node = node->next;
		iterations++;
	}

	// Check for iteration limit in merge pass
	if (iterations >= MAX_SAFE_PATH_NODES) {
		log_error("MERGE_PASS: Hit iteration limit for train %d", train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	log_info("SWITCH_PASS: Completed for train %d - branch switches: %u set, %u failed; merge switches: %u set, %u failed", 
		 train_id, switches_set, switches_failed, merge_switches_set, merge_switches_failed);

	// Update totals for overall success calculation
	switches_set += merge_switches_set;
	switches_failed += merge_switches_failed;

	// Return success if we set at least one switch, or if there were no switches to set
	return (switches_failed == 0) ? MARKLIN_ERROR_OK : MARKLIN_ERROR_UNKNOWN;
}

static marklin_error_t handle_activate_path_request(conductor_task_data_t *data,
						    const marklin_conductor_request_t *request)
{
	if (!data || !request || !request->activate_path.path || !request->activate_path.result) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	path_result_t *path = request->activate_path.path;
	u8 train_id = request->activate_path.train_id;
	kinematic_distance_t max_distance_to_reserve = request->activate_path.max_distance_to_reserve;
	const track_node *current_sensor = request->activate_path.current_sensor;
	kinematic_distance_t current_offset_mm = request->activate_path.current_offset_mm;
	marklin_path_activation_result_t *result = request->activate_path.result;

	if (train_id == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Count path nodes for logging
	u32 path_node_count = 0;
	struct dlist_node *count_node;
	if (path == NULL || (u64)path < 0x1000) {
		Panic("Path is NULL");
	}

	// Validate dlist structure integrity before iteration
	if (path->nodes.next == NULL || path->nodes.prev == NULL) {
		log_error("PATH ACTIVATION ERROR: Train %d path %p has corrupted node list (next=%p, prev=%p)",
			  train_id, path, path->nodes.next, path->nodes.prev);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Additional safety check: verify the list is not completely corrupted
	if ((u64)path->nodes.next < 0x1000 || (u64)path->nodes.prev < 0x1000) {
		log_error("PATH ACTIVATION ERROR: Train %d path %p has invalid node pointers (next=%p, prev=%p)",
			  train_id, path, path->nodes.next, path->nodes.prev);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Safe iteration with bounds checking to prevent infinite loops
	const u32 MAX_SAFE_PATH_NODES = 100; // Reasonable upper bound for path length
	count_node = path->nodes.next;
	while (count_node != &path->nodes && path_node_count < MAX_SAFE_PATH_NODES) {
		path_node_count++;

		// Validate each node pointer before dereferencing
		if (count_node->next == NULL || (u64)count_node->next < 0x1000) {
			log_error("PATH ACTIVATION ERROR: Train %d corrupted node detected at position %u (next=%p)",
				  train_id, path_node_count, count_node->next);
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}

		count_node = count_node->next;
	}

	// Check if we hit the safety limit
	if (path_node_count >= MAX_SAFE_PATH_NODES) {
		log_error("PATH ACTIVATION ERROR: Train %d path too long or circular (%u+ nodes)", train_id,
			  path_node_count);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Initialize result structure
	result->next_expected_sensor = NULL;
	result->segment_distance = 0;
	result->next_expected_distance = 0;
	result->reversal_safety_distance = 0;
	result->blocks_reserved = 0;
	result->blocks_available_in_path = 0;
	result->furthest_activated_node = NULL;
	// Initialize reserved block nodes array
	for (u32 i = 0; i < MAX_TRACK_BLOCKS; i++) {
		result->reserved_block_nodes[i] = NULL;
	}
	result->stop_reason = PATH_ACTIVATION_STOP_END_OF_PATH;

	kinematic_distance_t reserved_distance = 0;
	// Removed reversal tracking variables - no longer needed with simplified logic
	const track_node *furthest_reachable_node =
		NULL; // Furthest point train can reach (first reserved in backward iteration)
	u32 blocks_reserved_count = 0;

	// Track blocks reserved during this activation session to prevent conflicts
	bool reserved_in_this_session[MAX_TRACK_BLOCKS] = { false };

	// First pass: Count total blocks available in the path (safe iteration)
	track_block_t *last_block = NULL;
	struct dlist_node *count_blocks_node = path->nodes.next;
	u32 count_blocks_iterations = 0;

	while (count_blocks_node != &path->nodes && count_blocks_iterations < MAX_SAFE_PATH_NODES) {
		// Validate node pointer before dereferencing
		if (count_blocks_node->next == NULL || (u64)count_blocks_node->next < 0x1000) {
			log_error("PATH ACTIVATION ERROR: Train %d corrupted node in blocks counting at iteration %u",
				  train_id, count_blocks_iterations);
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}

		path_node_t *count_path_node = dlist_entry(count_blocks_node, path_node_t, list);
		track_block_t *node_block =
			conductor_find_block_containing_node(count_path_node->node, data, false, true, true, true);

		if (node_block && node_block != last_block) {
			bool is_boundary = conductor_is_boundary_sensor(count_path_node->node, node_block);
			if (is_boundary || !last_block) {
				result->blocks_available_in_path++;
				last_block = node_block;
			}
		}

		count_blocks_node = count_blocks_node->next;
		count_blocks_iterations++;
	}

	// Check for iteration limit
	if (count_blocks_iterations >= MAX_SAFE_PATH_NODES) {
		log_error("PATH ACTIVATION ERROR: Train %d blocks counting hit iteration limit", train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Progressive activation: Reserve blocks one by one from END to START until hitting a limit or obstacle (safe iteration)
	// If current_position is provided, start from that position instead of the end
	struct dlist_node *node = path->nodes.prev;

	if (current_sensor != NULL) {
		// Find the current position in the path and start activation from there
		struct dlist_node *search_node = path->nodes.next;
		u32 search_iterations = 0;
		bool found_position = false;

		log_info(
			"PATH ACTIVATION: Train %d looking for current position %s (offset %lldmm) in path for continuation",
			train_id, current_sensor->name, current_offset_mm);

		while (search_node != &path->nodes && search_iterations < MAX_SAFE_PATH_NODES) {
			path_node_t *search_path_node = dlist_entry(search_node, path_node_t, list);

			if (search_path_node->node == current_sensor) {
				// Found current position, start activation from here (going backward from this point)
				node = search_node;
				found_position = true;
				log_info(
					"PATH ACTIVATION: Train %d found current position %s in path, starting activation from here",
					train_id, current_sensor->name);
				break;
			}

			search_node = search_node->next;
			search_iterations++;
		}

		if (!found_position) {
			// log_warn("PATH ACTIVATION: Train %d current position %s not found in path, using end of path",
			//  train_id, current_sensor->name);
			// Fall back to starting from end if current position not found
			node = path->nodes.prev;
		}
	}
	path_node_t *current_path_node = NULL;
	track_block_t *current_block = NULL;
	u32 main_loop_iterations = 0;

	while (node != &path->nodes && main_loop_iterations < MAX_SAFE_PATH_NODES) {
		current_path_node = dlist_entry(node, path_node_t, list);

		// With simplified logic, reversals only occur at start and are handled by train
		// We should not encounter reverse_here nodes during activation
		if (current_path_node->reverse_here) {
			log_warn(
				"PATH ACTIVATION: Unexpected reverse_here during path activation - ignoring for train %d",
				train_id);
			// Continue activation normally - train handles start reversals
		}

		// Find which block contains this node
		track_block_t *node_block =
			conductor_find_block_containing_node(current_path_node->node, data, true, false, true, true);

		// If we're entering a new block, try to reserve it
		if (node_block && node_block != current_block) {
			bool is_boundary = conductor_is_boundary_sensor(current_path_node->node, node_block);

			if (is_boundary || !current_block) {
				// Check distance limits before trying to reserve
				bool at_global_limit = max_distance_to_reserve > 0 &&
						       reserved_distance >= max_distance_to_reserve;

				if (at_global_limit) {
					log_info(
						"PATH ACTIVATION STOPPED: Reached max_distance_to_reserve limit (%lldmm/%lldmm) at block %d (reserved_distance=%lldmm)",
						reserved_distance, max_distance_to_reserve, node_block->block_id,
						reserved_distance);
					result->stop_reason = PATH_ACTIVATION_STOP_MAX_BLOCKS_REACHED;
					break;
				}

				// Check if this block was already reserved in this activation session
				if (node_block->block_id < MAX_TRACK_BLOCKS &&
				    reserved_in_this_session[node_block->block_id]) {
					log_info(
						"PATH ACTIVATION STOPPED: Block %d already reserved in this activation session (train %d)",
						node_block->block_id, train_id);
					result->stop_reason = PATH_ACTIVATION_STOP_PREVIOUSLY_RESERVED;
					break;
				}

				// Try to reserve the new block
				if (!conductor_is_block_available(node_block, train_id)) {
					log_info(
						"PATH ACTIVATION STOPPED: Block %d not available for train %d (reserved by another train?)",
						node_block->block_id, train_id);
					result->stop_reason = PATH_ACTIVATION_STOP_BLOCK_UNAVAILABLE;
					break;
				}

				marklin_error_t reserve_result = conductor_reserve_block(node_block, train_id);
				if (reserve_result != MARKLIN_ERROR_OK) {
					log_error(
						"PATH ACTIVATION FAILED: Could not reserve block %d for train %d (error=%d)",
						node_block->block_id, train_id, reserve_result);
					result->stop_reason = PATH_ACTIVATION_STOP_ERROR;
					break;
				}

				// Mark this block as reserved in this activation session
				if (node_block->block_id < MAX_TRACK_BLOCKS) {
					reserved_in_this_session[node_block->block_id] = true;
				}

				current_block = node_block;

				// Add this block to the reserved blocks array using the current path node as representative
				if (blocks_reserved_count < MAX_TRACK_BLOCKS) {
					result->reserved_block_nodes[blocks_reserved_count] = current_path_node->node;
				}

				blocks_reserved_count++;

				if (node->prev != &path->nodes) {
					path_node_t *prev_path_node = dlist_entry(node->prev, path_node_t, list);
					int num_edges = marklin_get_node_edge_count(current_path_node->node);
					for (int i = 0; i < num_edges; i++) {
						if (current_path_node->node->edge[i].dest == prev_path_node->node) {
							kinematic_distance_t edge_distance =
								prev_path_node->node->edge[i].dist;

							// Adjust distance for position offset if this is the first edge from current position
							if (current_sensor != NULL && blocks_reserved_count == 1 &&
							    current_path_node->node == current_sensor) {
								// Train is at current sensor with offset, adjust the first edge distance
								if (current_offset_mm > 0) {
									// Train has progressed past sensor, reduce remaining distance in first segment
									kinematic_distance_t remaining_distance =
										edge_distance - current_offset_mm;
									if (remaining_distance > 0) {
										reserved_distance += remaining_distance;
										log_info(
											"PATH ACTIVATION: Train %d adjusted first edge distance from %lldmm to %lldmm (offset: %lldmm)",
											train_id, edge_distance,
											remaining_distance,
											current_offset_mm);
									} else {
										log_info(
											"PATH ACTIVATION: Train %d has passed first edge completely (offset: %lldmm > edge: %lldmm)",
											train_id, current_offset_mm,
											edge_distance);
									}
								} else if (current_offset_mm < 0) {
									// Train is before sensor, add the offset to distance needed
									kinematic_distance_t total_distance =
										edge_distance + (-current_offset_mm);
									reserved_distance += total_distance;
									log_info(
										"PATH ACTIVATION: Train %d adjusted first edge distance from %lldmm to %lldmm (before sensor: %lldmm)",
										train_id, edge_distance, total_distance,
										-current_offset_mm);
								} else {
									// Offset is exactly 0, use normal distance
									reserved_distance += edge_distance;
								}
							} else {
								// Normal distance calculation for other edges
								reserved_distance += edge_distance;
							}
							break;
						}
					}
				}

				// Track furthest reachable point (last successful reservation in backward iteration)
				// This represents the furthest point from train's position that was successfully reserved
				furthest_reachable_node = current_path_node->node;
				log_info(
					"PATH ACTIVATION: Updated furthest reachable node to %s (reserved distance: %lldmm, blocks reserved: %u)",
					furthest_reachable_node->name ? furthest_reachable_node->name : "unnamed",
					reserved_distance, blocks_reserved_count);

				// Note: Safety blocks now counted in forward traversal calculation
			}
		}

		// Note: Switch setting moved to second pass after all blocks are reserved

		// Handle merge nodes that require reverse turnout setting
		// if (current_block && current_path_node->node->type == NODE_MERGE && current_path_node->node->reverse) {
		// 	const track_node *reverse_node = current_path_node->node->reverse;
		// 	if (reverse_node->type == NODE_BRANCH) {
		// 		// Check if reverse turnout is in same block
		// 		bool reverse_in_block = false;
		// 		for (u32 i = 0; i < current_block->turnout_count; i++) {
		// 			if (current_block->turnouts[i] == reverse_node) {
		// 				reverse_in_block = true;
		// 				break;
		// 			}
		// 		}

		// 		if (reverse_in_block) {
		// 			track_direction reverse_dir = DIR_STRAIGHT;
		// 			path_node_t *next_node =
		// 				dlist_entry(current_path_node->list.next, path_node_t, list);
		// 			const track_node *next_node_reverse = next_node->node->reverse;

		// 			if (reverse_node->edge[DIR_CURVED].dest == next_node_reverse) {
		// 				reverse_dir = DIR_CURVED;
		// 			}
		// 			conductor_set_turnout_in_block(current_block, reverse_node, reverse_dir,
		// 						       train_id);
		// 		}
		// 	}
		// }

		// Simplified logic - no mid-path reversal handling needed

		// Note: Safety distance termination now handled in forward traversal calculation

		// Safe iteration: validate prev pointer and advance (going backward through path)
		if (node->prev == NULL || (u64)node->prev < 0x1000) {
			log_error("PATH ACTIVATION ERROR: Train %d corrupted node in main loop at iteration %u",
				  train_id, main_loop_iterations);
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}

		node = node->prev;
		main_loop_iterations++;
	}

	// Check for iteration limit
	if (main_loop_iterations >= MAX_SAFE_PATH_NODES) {
		log_error("PATH ACTIVATION ERROR: Train %d main loop hit iteration limit", train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Set the furthest activated node (where path activation actually stopped)
	result->furthest_activated_node = furthest_reachable_node;

	// Calculate next sensors from the furthest reachable node
	if (request->activate_path.current_sensor) {
		conductor_calculate_next_sensors(request->activate_path.current_sensor, TRAIN_DIRECTION_FORWARD,
						 &result->next_expected_sensor, &result->next_expected_distance);
	} else {
		path_node_t *prev_path_node = dlist_entry(path->nodes.prev, path_node_t, list);
		conductor_calculate_next_sensors(prev_path_node->node, TRAIN_DIRECTION_FORWARD,
						 &result->next_expected_sensor, &result->next_expected_distance);
	}

	// Update final result
	result->segment_distance = reserved_distance;
	result->blocks_reserved = blocks_reserved_count;

	// Reversal fields no longer used with simplified logic

	log_info(
		"PATH ACTIVATION: Train %d path activation completed - reserved %u blocks, distance %lldmm, next sensor %s, furthest node %s, stop reason %d",
		train_id, blocks_reserved_count, reserved_distance,
		result->next_expected_sensor ? result->next_expected_sensor->name : "none",
		furthest_reachable_node ? furthest_reachable_node->name : "none", result->stop_reason);

	// If we successfully reserved blocks, perform second pass to set switches
	if (blocks_reserved_count > 0) {
		marklin_error_t switch_result = conductor_set_switches_in_path(path, train_id, data);

		if (switch_result != MARKLIN_ERROR_OK) {
			// log_warn("PATH ACTIVATION: Switch setting completed with some failures: error %d",
			//  switch_result);
			// Note: We don't fail the entire path activation if only switch setting has issues
		}

		return MARKLIN_ERROR_OK;
	} else {
		return MARKLIN_ERROR_ALREADY_RESERVED;
	}
}

// ############################################################################
// # Track Distance Calculation
// ############################################################################

static marklin_error_t conductor_calculate_track_distance(const track_node *from, const track_node *to, u8 train_id,
							  kinematic_distance_t *raw_distance,
							  kinematic_distance_t *effective_distance)
{
	UNUSED(train_id); // Train ID not needed for current switch state traversal

	if (!from || !to || !raw_distance || !effective_distance) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// If same node, distance is 0
	if (from == to) {
		*raw_distance = 0;
		*effective_distance = 0;
		return MARKLIN_ERROR_OK;
	}

	// Traverse track following current switch settings
	const track_node *current = from;
	kinematic_distance_t total_raw_distance = 0;
	kinematic_distance_t total_effective_distance = 0;
	int max_hops = 100; // Prevent infinite loops

	while (current && current != to && max_hops-- > 0) {
		// Determine which edge to follow based on node type
		const track_node *next_node = NULL;
		const track_edge *edge = NULL;

		switch (current->type) {
		case NODE_BRANCH: {
			// For branch nodes, use current switch direction
			switch_lookup_entry_t *switch_entry = conductor_get_switch_lookup_entry(current->num);
			track_direction switch_dir = switch_entry ? switch_entry->state.direction : DIR_STRAIGHT;

			if (current->edge[switch_dir].dest) {
				next_node = current->edge[switch_dir].dest;
				edge = &current->edge[switch_dir];
			}
			break;
		}
		case NODE_SENSOR:
		case NODE_MERGE:
		case NODE_ENTER:
		default:
			// For other nodes, follow the straight path
			if (current->edge[DIR_AHEAD].dest) {
				next_node = current->edge[DIR_AHEAD].dest;
				edge = &current->edge[DIR_AHEAD];
			}
			break;
		}

		// If we can't continue, path not found
		if (!next_node || !edge) {
			return MARKLIN_ERROR_NOT_FOUND;
		}

		// Add raw distance
		kinematic_distance_t edge_raw_distance = edge->dist;
		total_raw_distance += edge_raw_distance;

		// Calculate effective distance with resistance coefficient
		u32 resistance_coeff = edge->resistance_coefficient;
		if (resistance_coeff == 0) {
			resistance_coeff = RESISTANCE_DEFAULT; // Use default if not set
		}
		kinematic_distance_t edge_effective_distance =
			kinematic_apply_resistance_to_distance(edge_raw_distance, resistance_coeff);
		total_effective_distance += edge_effective_distance;

		current = next_node;
	}

	// Check if we found the destination
	if (current != to) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	*raw_distance = total_raw_distance;
	*effective_distance = total_effective_distance;
	return MARKLIN_ERROR_OK;
}

// ############################################################################
// # Main Server Implementation
// ############################################################################

void __noreturn marklin_conductor_task(void)
{
	conductor_task_data_t conductor_data;

	conductor_init_task_data(&conductor_data);
	g_conductor_data = &conductor_data;

	RegisterAs(MARKLIN_CONDUCTOR_SERVER_NAME);

	conductor_data.clock_server_tid = WhoIs(CLOCK_SERVER_NAME);
	if (conductor_data.clock_server_tid < 0) {
		Panic("Clock server not found");
	}

	conductor_data.command_server_tid = WhoIs(MARKLIN_CMD_SERVER_NAME);
	if (conductor_data.command_server_tid < 0) {
		Panic("Command server not found");
	}

	Create(MARKLIN_SENSOR_TASK_PRIORITY, sensor_timer_task);

	conductor_main_loop(&conductor_data);

	UNREACHABLE();
}

static void conductor_main_loop(conductor_task_data_t *data)
{
	marklin_conductor_request_t request;
	int sender_tid;

	for (;;) {
		int result = Receive(&sender_tid, (char *)&request, sizeof(request));
		if (result > 0) {
			conductor_process_request(data, sender_tid, &request);
		} else {
			continue;
		}
	}
}

static void conductor_process_request(conductor_task_data_t *data, int sender_tid, marklin_conductor_request_t *request)
{
	UNUSED(data);

	marklin_conductor_reply_t reply;
	reply.error = MARKLIN_ERROR_OK;

	switch (request->type) {
	case MARKLIN_CONDUCTOR_REQ_ON_SENSOR_DATA: {
		conductor_consume_sensor_data(request->sensor_data.sensor_data, request->sensor_data.tick);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_SET_SWITCH: {
		reply.error = switch_set_direction(request->set_switch.switch_id, request->set_switch.direction,
						   request->set_switch.disengage_solenoid, request->set_switch.force);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_GET_SENSOR_STATES: {
		u32 found_count =
			sensor_get_states(request->get_sensor_states.sensors, request->get_sensor_states.count);
		if (found_count == 0) {
			reply.error = MARKLIN_ERROR_NOT_FOUND;
		} else if (found_count < request->get_sensor_states.count) {
			reply.error = MARKLIN_ERROR_OK;
		}
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_FIND_PATH: {
		// Allocate a path pool for this path finding operation
		path_node_pool_t *pool = path_pool_alloc(&data->free_path_pools, request->find_path.train_id);
		if (!pool) {
			log_error("Failed to allocate path pool for train %d", request->find_path.train_id);
			reply.error = MARKLIN_ERROR_QUEUE_FULL; // Resource exhaustion
		} else {
			// Check if excluded blocks were already provided (deadlock resolution in progress)
			const track_block_t **final_excluded_blocks = request->find_path.excluded_blocks;
			u32 final_excluded_count = request->find_path.excluded_count;

			// If no exclusions provided, check for deadlock and potentially add exclusions
			const track_block_t **deadlock_excluded_blocks = NULL;
			u32 deadlock_excluded_count = 0;
			bool used_deadlock_resolution = false;

			if (!final_excluded_blocks || final_excluded_count == 0) {
				// Detect deadlock and get exclusions if needed
				if (conductor_detect_deadlock(data)) {
					data->deadlock_context.resolution_in_progress = true;
					if (conductor_resolve_deadlock(data, request->find_path.train_id,
								       &deadlock_excluded_blocks,
								       &deadlock_excluded_count)) {
						final_excluded_blocks = deadlock_excluded_blocks;
						final_excluded_count = deadlock_excluded_count;
						used_deadlock_resolution = true;
						log_info("Using deadlock resolution for train %d: excluding %d blocks",
							 request->find_path.train_id, deadlock_excluded_count);
					}
				}
			}

			// Attempt path finding with final exclusion list
			reply.error = path_find(request->find_path.from, request->find_path.to,
						request->find_path.train_id, request->find_path.allow_reversal,
						request->find_path.use_block_exit_start, final_excluded_blocks,
						final_excluded_count, pool, request->find_path.result);

			// Handle path finding result
			if (reply.error != MARKLIN_ERROR_OK) {
				// Path finding failed - record failure for deadlock detection
				if (reply.error == MARKLIN_ERROR_NO_PATH && !used_deadlock_resolution) {
					// Find which blocks would be needed (simplified - use reserved blocks as blocking)
					track_block_t *blocking_blocks[MAX_BLOCKING_BLOCKS];
					u32 blocking_count = 0;

					// Collect currently reserved blocks as potential blockers
					for (int i = 0;
					     i < data->track_block_count && blocking_count < MAX_BLOCKING_BLOCKS; i++) {
						if (data->track_blocks[i].owner_train_id != 0 &&
						    data->track_blocks[i].owner_train_id !=
							    request->find_path.train_id) {
							blocking_blocks[blocking_count++] = &data->track_blocks[i];
						}
					}

					conductor_record_path_failure(data, request->find_path.train_id,
								      request->find_path.from, request->find_path.to,
								      blocking_blocks, blocking_count);
				}

				path_pool_free(pool, &data->free_path_pools);
			} else if (used_deadlock_resolution) {
				// Success with deadlock resolution - clear the deadlock context
				data->deadlock_context.resolution_in_progress = false;
				log_info("Deadlock resolved successfully for train %d", request->find_path.train_id);
			}
		}
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_GET_NEXT_SENSORS: {
		reply.error = conductor_calculate_next_sensors(request->get_next_sensors.current_location,
							       request->get_next_sensors.direction,
							       request->get_next_sensors.expected_sensor,
							       request->get_next_sensors.expected_distance);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_GET_NEXT_TWO_SENSORS: {
		reply.error = conductor_calculate_next_two_sensors(request->get_next_two_sensors.current_location,
								   request->get_next_two_sensors.direction,
								   request->get_next_two_sensors.sensors,
								   request->get_next_two_sensors.distances,
								   request->get_next_two_sensors.count);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_CALCULATE_DISTANCE: {
		reply.error = conductor_calculate_track_distance(request->calculate_distance.from,
								 request->calculate_distance.to,
								 request->calculate_distance.train_id,
								 &reply.calculate_distance.raw_distance,
								 &reply.calculate_distance.effective_distance);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_ACTIVATE_PATH: {
		reply.error = handle_activate_path_request(data, request);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_RELEASE_TRAIN_BLOCKS: {
		reply.error = handle_release_train_blocks_request(data, request);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_RELEASE_SPECIFIC_BLOCK: {
		reply.error = handle_release_specific_block_request(data, request);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_RESERVE_SPECIFIC_BLOCK: {
		reply.error = handle_reserve_specific_block_request(data, request);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_CHECK_BLOCK_OWNERSHIP: {
		reply.error = handle_check_block_ownership_request(data, request, &reply);
		break;
	}
	case MARKLIN_CONDUCTOR_REQ_FREE_PATH: {
		reply.error = handle_free_path_request(data, request);
		break;
	}
	default:
		reply.error = MARKLIN_ERROR_INVALID_ARGUMENT;
		break;
	}

	Reply(sender_tid, (const char *)&reply, sizeof(reply));
}

// ############################################################################
// # Helper Functions for Other Modules
// ############################################################################

switch_lookup_entry_t *conductor_get_switch_lookup_entry(u8 switch_id)
{
	if (!g_conductor_data) {
		return NULL;
	}

	for (int i = 0; i < g_conductor_data->switch_count; i++) {
		if (g_conductor_data->switch_lookup[i].switch_node &&
		    g_conductor_data->switch_lookup[i].state.switch_id == switch_id) {
			return &g_conductor_data->switch_lookup[i];
		}
	}

	return NULL;
}

// ############################################################################
// # Block Release Implementation Functions
// ############################################################################

static marklin_error_t handle_release_train_blocks_request(conductor_task_data_t *data,
							   const marklin_conductor_request_t *request)
{
	if (!data || !request) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	u8 train_id = request->release_train_blocks.train_id;
	const track_node *keep_block_node = request->release_train_blocks.keep_block_node;

	if (train_id == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Find the block to keep (if specified)
	track_block_t *keep_block = NULL;
	if (keep_block_node) {
		keep_block = conductor_find_block_containing_node(keep_block_node, data, true, false, true, true);
		log_info("Train %d: Keeping block node %s, block id %d", train_id,
			 keep_block_node->name ? keep_block_node->name : "unnamed",
			 keep_block ? keep_block->block_id : 99);
		if (!keep_block) {
			log_warn("Train %d: Keep block node %s not found in any block", train_id,
				 keep_block_node->name);
		} else if (keep_block->owner_train_id != train_id && keep_block->owner_train_id != 0) {
			log_warn("Train %d: Keep block %d not owned by this train (owned by %d)", train_id,
				 keep_block->block_id, keep_block->owner_train_id);
			keep_block = NULL; // Don't keep a block we don't own
		}
	}

	u32 blocks_released = 0;
	marklin_error_t first_error = MARKLIN_ERROR_OK;

	// Iterate through all blocks and release those owned by this train (except the keep block)
	for (int i = 0; i < data->track_block_count; i++) {
		track_block_t *block = &data->track_blocks[i];
		if (block->owner_train_id == train_id && block != keep_block) {
			marklin_error_t release_result = conductor_release_block(block, train_id);
			if (release_result == MARKLIN_ERROR_OK) {
				blocks_released++;
				log_debug("Train %d: Released block %d", train_id, block->block_id);
			} else if (first_error == MARKLIN_ERROR_OK) {
				first_error = release_result;
			}
		}
	}

	if (blocks_released == 0 && first_error == MARKLIN_ERROR_OK) {
		return MARKLIN_ERROR_NOT_FOUND; // No blocks owned by this train (or only the keep block)
	}

	if (keep_block) {
		log_info("Train %d: Released %d blocks, kept block %d", train_id, blocks_released,
			 keep_block->block_id);
	} else {
		log_info("Train %d: Released %d blocks", train_id, blocks_released);
	}

	return first_error;
}

static marklin_error_t handle_release_specific_block_request(conductor_task_data_t *data,
							     const marklin_conductor_request_t *request)
{
	if (!data || !request || !request->release_specific_block.block_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	u8 train_id = request->release_specific_block.train_id;
	const track_node *block_node = request->release_specific_block.block_node;
	const track_node *current_block_node = request->release_specific_block.current_block_node;

	if (train_id == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Find the block containing this node
	track_block_t *block = conductor_find_block_containing_node(block_node, data, false, true, true, true);
	if (!block) {
		log_error("Train %d: No block found containing node %s", train_id, block_node->name);
		return MARKLIN_ERROR_NOT_FOUND;
	}

	// Check if train owns this block
	if (block->owner_train_id != train_id) {
		log_error("Train %d: Cannot release block %d (owned by train %d)", train_id, block->block_id,
			  block->owner_train_id);
		return MARKLIN_ERROR_NOT_OWNER;
	}

	// If current_block_node is specified, ensure it remains reserved atomically
	track_block_t *current_block = NULL;
	if (current_block_node) {
		current_block = conductor_find_block_containing_node(current_block_node, data, true, false, true, true);
		if (!current_block) {
			log_error("Train %d: No current block found containing node %s", train_id,
				  current_block_node->name);
			return MARKLIN_ERROR_NOT_FOUND;
		}

		// Check if the train should own the current block
		if (current_block->owner_train_id != train_id && current_block->owner_train_id != 0) {
			log_error("Train %d: Current block %d not owned by train (owned by train %d)", train_id,
				  current_block->block_id, current_block->owner_train_id);
			return MARKLIN_ERROR_NOT_OWNER;
		}

		// Verify we're not trying to release the same block as current block
		if (block == current_block) {
			log_error("Train %d: Cannot release current block %d atomically", train_id, block->block_id);
			return MARKLIN_ERROR_INVALID_ARGUMENT;
		}

		conductor_reserve_block(current_block, train_id); // Ensure current block remains reserved
	}

	marklin_error_t result = conductor_release_block(block, train_id);
	if (result == MARKLIN_ERROR_OK) {
		log_info("Train %d: Released block %d containing node %s", train_id, block->block_id, block_node->name);

		// If current_block_node was specified, log that it remains reserved
		if (current_block) {
			log_info("Train %d: Current block %d containing node %s remains reserved", train_id,
				 current_block->block_id, current_block_node->name);
		}
	}

	return result;
}

static marklin_error_t handle_reserve_specific_block_request(conductor_task_data_t *data,
							     const marklin_conductor_request_t *request)
{
	if (!data || !request || !request->reserve_specific_block.block_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	u8 train_id = request->reserve_specific_block.train_id;
	const track_node *block_node = request->reserve_specific_block.block_node;

	if (train_id == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Find the block containing this node
	track_block_t *block = conductor_find_block_containing_node(block_node, data, true, false, true, true);
	if (!block) {
		log_error("Train %d: No block found containing node %s", train_id, block_node->name);
		return MARKLIN_ERROR_NOT_FOUND;
	}

	// Check if block is already reserved by another train
	if (block->owner_train_id != 0 && block->owner_train_id != train_id) {
		// log_info("Train %d: Cannot reserve block %d (already owned by train %d)", train_id, block->block_id,
		//  block->owner_train_id);
		return MARKLIN_ERROR_ALREADY_RESERVED;
	}

	// If already owned by this train, that's fine
	if (block->owner_train_id == train_id) {
		log_debug("Train %d: Block %d already owned by this train", train_id, block->block_id);
		return MARKLIN_ERROR_OK;
	}

	// Try to reserve the block
	marklin_error_t result = conductor_reserve_block(block, train_id);
	if (result == MARKLIN_ERROR_OK) {
		log_info("Train %d: Reserved block %d containing node %s", train_id, block->block_id, block_node->name);
	} else {
		log_error("Train %d: Failed to reserve block %d containing node %s (error: %d)", train_id,
			  block->block_id, block_node->name, result);
	}

	return result;
}

static marklin_error_t handle_check_block_ownership_request(conductor_task_data_t *data,
							    const marklin_conductor_request_t *request,
							    marklin_conductor_reply_t *reply)
{
	if (!data || !request || !reply || !request->check_block_ownership.block_node) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	u8 train_id = request->check_block_ownership.train_id;
	const track_node *block_node = request->check_block_ownership.block_node;

	if (train_id == 0) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Find the block containing this node
	track_block_t *block = conductor_find_block_containing_node(block_node, data, true, false, true, true);
	if (!block) {
		log_debug("Train %d: No block found containing node %s for ownership check", train_id,
			  block_node->name);
		return MARKLIN_ERROR_NOT_FOUND;
	}

	// Set ownership information in reply
	reply->check_block_ownership.owner_train_id = block->owner_train_id;
	reply->check_block_ownership.owns_block = (block->owner_train_id == train_id);

	log_debug("Train %d: Block %d ownership check - owner: %d, owns_block: %s", train_id, block->block_id,
		  block->owner_train_id, reply->check_block_ownership.owns_block ? "true" : "false");

	return MARKLIN_ERROR_OK;
}

static marklin_error_t handle_free_path_request(conductor_task_data_t *data, const marklin_conductor_request_t *request)
{
	if (!data || !request || !request->free_path.path) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	path_result_t *path = request->free_path.path;

	// Validate the path has a pool assigned
	if (!path->pool) {
		log_debug("Path cleanup: path has no pool assigned, already cleaned up");
		return MARKLIN_ERROR_OK;
	}

	// Save the train_id before cleanup (path_cleanup will set pool to NULL)
	u8 train_id = path->pool->owner_train_id;
	path_node_pool_t *pool = path->pool;

	// Clean up the path nodes
	path_cleanup(path);

	// Free the pool back to the conductor's free list
	path_pool_free(pool, &data->free_path_pools);

	log_debug("Path cleanup: freed path pool for train %d", train_id);

	return MARKLIN_ERROR_OK;
}
