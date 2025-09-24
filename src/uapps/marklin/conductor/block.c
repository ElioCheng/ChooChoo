#include "marklin/conductor/block.h"
#include "marklin/common/track_node.h"
#include "marklin/conductor/block_definitions.h"
#include "marklin/conductor/conductor.h"
#include "marklin/topology/track.h"
#include "marklin/msgqueue/api.h"
#include "string.h"
// #include "klog.h"
#include "syscall.h"
#include "clock.h"

#define LOG_MODULE "block"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

// External reference to conductor data
extern conductor_task_data_t *g_conductor_data;

// ============================================================================
// Helper Functions
// ============================================================================

bool conductor_add_entry_sensor(track_block_t *block, const track_node *sensor)
{
	if (block->entry_sensor_count >= MAX_BOUNDARY_SENSORS_PER_BLOCK) {
		return false;
	}
	for (u32 i = 0; i < block->entry_sensor_count; i++) {
		if (block->entry_sensors[i] == sensor) {
			return true;
		}
	}

	block->entry_sensors[block->entry_sensor_count] = sensor;
	block->entry_sensor_count++;
	return true;
}

bool conductor_add_exit_sensor(track_block_t *block, const track_node *sensor)
{
	if (block->exit_sensor_count >= MAX_BOUNDARY_SENSORS_PER_BLOCK) {
		return false;
	}

	for (u32 i = 0; i < block->exit_sensor_count; i++) {
		if (block->exit_sensors[i] == sensor) {
			return true;
		}
	}

	block->exit_sensors[block->exit_sensor_count++] = sensor;
	return true;
}

bool conductor_add_internal_sensor(track_block_t *block, const track_node *sensor)
{
	if (block->internal_sensor_count >= MAX_INTERNAL_SENSORS_PER_BLOCK) {
		return false;
	}

	for (u32 i = 0; i < block->internal_sensor_count; i++) {
		if (block->internal_sensors[i] == sensor) {
			return true;
		}
	}

	block->internal_sensors[block->internal_sensor_count++] = sensor;
	return true;
}

bool conductor_add_turnout(track_block_t *block, const track_node *turnout)
{
	if (block->turnout_count >= MAX_TURNOUTS_PER_BLOCK) {
		return false;
	}

	for (u32 i = 0; i < block->turnout_count; i++) {
		if (block->turnouts[i] == turnout) {
			return true;
		}
	}

	block->turnouts[block->turnout_count++] = turnout;
	return true;
}

// ============================================================================
// Block Initialization (Hardcoded Definitions)
// ============================================================================

void conductor_init_blocks(conductor_task_data_t *data)
{
	if (!data) {
		return;
	}

	conductor_init_hardcoded_blocks(data, data->track_type);
}

// ============================================================================
// Block Query Functions
// ============================================================================

track_block_t *conductor_find_block_containing_node(const track_node *node, conductor_task_data_t *data,
						    bool search_entry_node, bool search_exit_node,
						    bool search_internal_node, bool search_turnouts)
{
	if (!node || !data) {
		return NULL;
	}

	for (int i = 0; i < data->track_block_count; i++) {
		track_block_t *block = &data->track_blocks[i];

		if (search_entry_node) {
			for (u32 j = 0; j < block->entry_sensor_count; j++) {
				if (block->entry_sensors[j] == node) {
					return block;
				}
			}
		}

		if (search_exit_node) {
			for (u32 j = 0; j < block->exit_sensor_count; j++) {
				if (block->exit_sensors[j] == node) {
					return block;
				}
			}
		}
		if (search_internal_node) {
			for (u32 j = 0; j < block->internal_sensor_count; j++) {
				if (block->internal_sensors[j] == node) {
					return block;
				}
			}
		}
		if (search_turnouts) {
			for (u32 j = 0; j < block->turnout_count; j++) {
				if (block->turnouts[j] == node) {
					return block;
				}
			}
		}
	}
	return NULL;
}

track_block_t *conductor_find_block_by_entry_node(const track_node *entry_node, conductor_task_data_t *data)
{
	if (!entry_node || !data || !marklin_is_boundary_node(entry_node)) {
		return NULL;
	}

	return conductor_find_block_containing_node(entry_node, data, true, false, false, false);
}

track_block_t *conductor_find_block_by_exit_node(const track_node *exit_node, conductor_task_data_t *data)
{
	if (!exit_node || !data || !marklin_is_boundary_node(exit_node)) {
		return NULL;
	}

	return conductor_find_block_containing_node(exit_node, data, false, true, false, false);
}

bool conductor_is_entry_node(const track_node *node, const track_block_t *block)
{
	if (!node || !block) {
		return false;
	}

	for (u32 i = 0; i < block->entry_sensor_count; i++) {
		if (block->entry_sensors[i] == node) {
			return true;
		}
	}

	return false;
}

bool conductor_is_exit_node(const track_node *node, const track_block_t *block)
{
	if (!node || !block) {
		return false;
	}

	for (u32 i = 0; i < block->exit_sensor_count; i++) {
		if (block->exit_sensors[i] == node) {
			return true;
		}
	}

	return false;
}

bool conductor_is_boundary_sensor(const track_node *node, const track_block_t *block)
{
	if (!node || !block) {
		return false;
	}

	return conductor_is_entry_node(node, block) || conductor_is_exit_node(node, block);
}

track_block_t *conductor_get_adjacent_block(const track_node *exit_node, track_direction direction,
					    conductor_task_data_t *data)
{
	if (!exit_node || !data || !marklin_is_boundary_node(exit_node)) {
		return NULL;
	}

	if (!exit_node->edge[direction].dest) {
		return NULL;
	}

	return conductor_find_block_by_entry_node(exit_node, data);
}

// ============================================================================
// Block Reservation Management
// ============================================================================

marklin_error_t conductor_reserve_block(track_block_t *block, u8 train_id)
{
	if (!block || train_id == 0) {
		log_warn("Reserve block failed: invalid arguments (block=%p, train_id=%d)", block, train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	log_debug("RESERVATION ATTEMPT: Train %d trying to reserve block %d (currently owned by train %d, occupied=%d)",
		  train_id, block->block_id, block->owner_train_id, block->occupied);

	if (block->owner_train_id != 0 && block->owner_train_id != train_id) {
		log_info("RESERVATION FAILED: Block %d already reserved by train %d (train %d attempted)",
			 block->block_id, block->owner_train_id, train_id);
		return MARKLIN_ERROR_ALREADY_RESERVED;
	}

	// Check if this is a re-reservation by the same train
	bool re_reservation = (block->owner_train_id == train_id);

	block->owner_train_id = train_id;
	u32 current_time = Time(g_conductor_data->clock_server_tid);
	block->reservation_time = current_time;

	if (re_reservation) {
		log_debug("RESERVATION REFRESHED: Train %d refreshed reservation of block %d at tick %u", train_id,
			  block->block_id, current_time);
	} else {
		log_info("BLOCK RESERVED: Train %d reserved block %d at tick %u (occupied=%d)", train_id,
			 block->block_id, current_time, block->occupied);
	}

	// Publish block reservation update to message queue
	marklin_block_reservation_data_t reservation_data;
	reservation_data.block_id = block->block_id;
	reservation_data.owner_train_id = train_id;
	reservation_data.status = block->occupied ? BLOCK_STATUS_OCCUPIED : BLOCK_STATUS_RESERVED;
	reservation_data.timestamp = current_time;

	// Copy entry sensor name if available
	if (block->current_entry_sensor && block->current_entry_sensor->name) {
		strncpy(reservation_data.entry_sensor_name, block->current_entry_sensor->name, 15);
		reservation_data.entry_sensor_name[15] = '\0';
	} else {
		reservation_data.entry_sensor_name[0] = '\0';
	}

	marklin_error_t publish_result =
		Marklin_MsgQueue_PublishTyped(MARKLIN_MSGQUEUE_EVENT_TYPE_BLOCK_RESERVATION, &reservation_data);
	if (publish_result != MARKLIN_ERROR_OK) {
		log_warn("Failed to publish block reservation update for block %d: error %d", block->block_id,
			 publish_result);
	}

	return MARKLIN_ERROR_OK;
}

marklin_error_t conductor_release_block(track_block_t *block, u8 train_id)
{
	if (!block || train_id == 0) {
		log_warn("Release block failed: invalid arguments (block=%p, train_id=%d)", block, train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	log_info("RELEASE ATTEMPT: Train %d trying to release block %d (currently owned by train %d, occupied=%d)",
		 train_id, block->block_id, block->owner_train_id, block->occupied);

	if (block->owner_train_id != train_id) {
		if (block->owner_train_id == 0) {
			log_warn("RELEASE FAILED: Train %d tried to release block %d which is already free", train_id,
				 block->block_id);
		} else {
			log_warn("RELEASE FAILED: Train %d tried to release block %d owned by train %d", train_id,
				 block->block_id, block->owner_train_id);
		}
		return MARKLIN_ERROR_NOT_OWNER;
	}

	u32 current_time = Time(g_conductor_data->clock_server_tid);
	u32 reservation_duration = current_time - block->reservation_time;

	block->owner_train_id = 0;
	block->reservation_time = 0;

	log_info("BLOCK RELEASED: Train %d released block %d at tick %u (held for %u ticks, still occupied=%d)",
		 train_id, block->block_id, current_time, reservation_duration, block->occupied);

	// Publish block release update to message queue
	marklin_block_reservation_data_t reservation_data;
	reservation_data.block_id = block->block_id;
	reservation_data.owner_train_id = 0; // Released, so no owner
	reservation_data.status = block->occupied ? BLOCK_STATUS_OCCUPIED : BLOCK_STATUS_FREE;
	reservation_data.timestamp = current_time;

	// Copy entry sensor name if available
	if (block->current_entry_sensor && block->current_entry_sensor->name) {
		strncpy(reservation_data.entry_sensor_name, block->current_entry_sensor->name, 15);
		reservation_data.entry_sensor_name[15] = '\0';
	} else {
		reservation_data.entry_sensor_name[0] = '\0';
	}

	marklin_error_t publish_result =
		Marklin_MsgQueue_PublishTyped(MARKLIN_MSGQUEUE_EVENT_TYPE_BLOCK_RESERVATION, &reservation_data);
	if (publish_result != MARKLIN_ERROR_OK) {
		log_warn("Failed to publish block release update for block %d: error %d", block->block_id,
			 publish_result);
	}

	return MARKLIN_ERROR_OK;
}

bool conductor_is_block_available(const track_block_t *block, u8 train_id)
{
	if (!block) {
		log_debug("Block availability check failed: null block");
		return false;
	}

	bool available = (block->owner_train_id == 0 || block->owner_train_id == train_id);

	log_info("AVAILABILITY CHECK: Block %d for train %d: %s (owner=%d, occupied=%d)", block->block_id, train_id,
		 available ? "AVAILABLE" : "UNAVAILABLE", block->owner_train_id, block->occupied);

	return available;
}

// ============================================================================
// Block Occupancy Tracking
// ============================================================================

void conductor_mark_block_occupied(track_block_t *block, const track_node *entry_sensor)
{
	if (!block) {
		return;
	}

	block->occupied = true;
	block->current_entry_sensor = entry_sensor;
	u32 current_time = Time(g_conductor_data->clock_server_tid);
	block->occupancy_time = current_time;

	// Publish block occupancy update to message queue
	marklin_block_reservation_data_t reservation_data;
	reservation_data.block_id = block->block_id;
	reservation_data.owner_train_id = block->owner_train_id;
	reservation_data.status = BLOCK_STATUS_OCCUPIED;
	reservation_data.timestamp = current_time;

	// Copy entry sensor name if available
	if (entry_sensor && entry_sensor->name) {
		strncpy(reservation_data.entry_sensor_name, entry_sensor->name, 15);
		reservation_data.entry_sensor_name[15] = '\0';
	} else {
		reservation_data.entry_sensor_name[0] = '\0';
	}

	marklin_error_t publish_result =
		Marklin_MsgQueue_PublishTyped(MARKLIN_MSGQUEUE_EVENT_TYPE_BLOCK_RESERVATION, &reservation_data);
	if (publish_result != MARKLIN_ERROR_OK) {
		log_warn("Failed to publish block occupancy update for block %d: error %d", block->block_id,
			 publish_result);
	}
}

void conductor_mark_block_clear(track_block_t *block)
{
	if (!block) {
		return;
	}

	block->occupied = false;
	block->current_entry_sensor = NULL;
	block->occupancy_time = 0;

	// Publish block clear update to message queue
	marklin_block_reservation_data_t reservation_data;
	reservation_data.block_id = block->block_id;
	reservation_data.owner_train_id = block->owner_train_id;
	reservation_data.status = (block->owner_train_id != 0) ? BLOCK_STATUS_RESERVED : BLOCK_STATUS_FREE;
	reservation_data.timestamp = Time(g_conductor_data->clock_server_tid);
	reservation_data.entry_sensor_name[0] = '\0'; // No entry sensor when cleared

	marklin_error_t publish_result =
		Marklin_MsgQueue_PublishTyped(MARKLIN_MSGQUEUE_EVENT_TYPE_BLOCK_RESERVATION, &reservation_data);
	if (publish_result != MARKLIN_ERROR_OK) {
		log_warn("Failed to publish block clear update for block %d: error %d", block->block_id,
			 publish_result);
	}
}

void conductor_update_block_occupancy(const track_node *sensor, conductor_task_data_t *data)
{
	if (!sensor || !data || !marklin_is_boundary_node(sensor)) {
		return;
	}

	for (int i = 0; i < data->track_block_count; i++) {
		track_block_t *block = &data->track_blocks[i];

		if (conductor_is_boundary_sensor(sensor, block)) {
			if (!block->occupied) {
				conductor_mark_block_occupied(block, sensor);
			} else if (block->current_entry_sensor && block->current_entry_sensor != sensor) {
				conductor_mark_block_clear(block);
			}
		}
	}
}

// ============================================================================
// Turnout Management Within Blocks
// ============================================================================

track_block_t *conductor_find_block_containing_turnout_owned_by_train(const track_node *turnout, u8 train_id,
								      conductor_task_data_t *data)
{
	if (!turnout || train_id == 0 || !data) {
		return NULL;
	}

	// Search through all blocks to find one that contains this turnout and is owned by the train
	for (int i = 0; i < data->track_block_count; i++) {
		track_block_t *block = &data->track_blocks[i];

		// Skip blocks not owned by this train
		if (block->owner_train_id != train_id) {
			continue;
		}

		// Check if this block contains the turnout
		for (u32 j = 0; j < block->turnout_count; j++) {
			if (block->turnouts[j] == turnout) {
				log_debug("TURNOUT_SEARCH: Found turnout %s in block %d owned by train %d",
					  turnout->name ? turnout->name : "unnamed", block->block_id, train_id);
				return block;
			}
		}
	}

	log_debug("TURNOUT_SEARCH: Turnout %s not found in any block owned by train %d",
		  turnout->name ? turnout->name : "unnamed", train_id);
	return NULL;
}

marklin_error_t conductor_set_turnout_in_block(track_block_t *block, const track_node *turnout,
					       track_direction direction, u8 train_id)
{
	log_debug("TURNOUT_SET: Attempting to set turnout %s (num=%d) to direction %d in block %d for train %d",
		  turnout ? (turnout->name ? turnout->name : "unnamed") : "NULL", turnout ? turnout->num : 99,
		  direction, block ? block->block_id : 99, train_id);

	if (!block || !turnout || train_id == 0) {
		log_warn("TURNOUT_SET: Invalid arguments - block=%p, turnout=%p, train_id=%d", block, turnout,
			 train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (block->owner_train_id != train_id) {
		log_warn("TURNOUT_SET: Block %d not owned by train %d (actually owned by train %d)", block->block_id,
			 train_id, block->owner_train_id);
		return MARKLIN_ERROR_NOT_OWNER;
	}

	bool found = false;
	for (u32 i = 0; i < block->turnout_count; i++) {
		if (block->turnouts[i] == turnout) {
			found = true;
			log_debug("TURNOUT_SET: Found turnout %s at index %d in block %d",
				  turnout->name ? turnout->name : "unnamed", i, block->block_id);
			break;
		}
	}

	if (!found) {
		log_warn("TURNOUT_SET: Turnout %s not found in block %d", turnout->name ? turnout->name : "unnamed",
			 block->block_id);
		return MARKLIN_ERROR_NOT_FOUND;
	}

	// log_info("TURNOUT_SET: Calling switch_set_direction for turnout %s (num=%d) direction=%d",
	//  turnout->name ? turnout->name : "unnamed", turnout->num, direction);

	marklin_error_t result = switch_set_direction(turnout->num, direction, 1, false);

	if (result != MARKLIN_ERROR_OK) {
		log_error("TURNOUT_SET: switch_set_direction failed for turnout %s: error %d",
			  turnout->name ? turnout->name : "unnamed", result);
	} else {
		// log_info("TURNOUT_SET: Successfully set turnout %s to %s", turnout->name ? turnout->name : "unnamed",
		// 	 direction == DIR_STRAIGHT ? "STRAIGHT" : "CURVED");
	}

	return result;
}

u32 conductor_get_block_turnouts(const track_block_t *block, const track_node **turnouts, u32 max_turnouts)
{
	if (!block || !turnouts || max_turnouts == 0) {
		return 0;
	}

	u32 count = block->turnout_count < max_turnouts ? block->turnout_count : max_turnouts;

	for (u32 i = 0; i < count; i++) {
		turnouts[i] = block->turnouts[i];
	}

	return count;
}

void conductor_print_all_blocks_info(conductor_task_data_t *data)
{
	if (!data) {
		log_error("conductor_print_all_blocks_info: Invalid data pointer");
		return;
	}

	log_info("=== TRACK BLOCKS INFORMATION ===");
	log_info("Total blocks discovered: %d", data->track_block_count);
	log_info("");

	for (int i = 0; i < data->track_block_count; i++) {
		track_block_t *block = &data->track_blocks[i];

		log_info("--- Block %d ---", block->block_id);

		// Entry sensors
		log_info("  Entry sensors (%d):", block->entry_sensor_count);
		for (u32 j = 0; j < block->entry_sensor_count; j++) {
			const track_node *entry = block->entry_sensors[j];
			log_info("    [%d] %s (type: %s)", j, entry->name ? entry->name : "unnamed",
				 entry->type == NODE_SENSOR ? "SENSOR" : "EXIT");
		}

		// Exit sensors
		log_info("  Exit sensors (%d):", block->exit_sensor_count);
		for (u32 j = 0; j < block->exit_sensor_count; j++) {
			const track_node *exit = block->exit_sensors[j];
			log_info("    [%d] %s (type: %s)", j, exit->name ? exit->name : "unnamed",
				 exit->type == NODE_SENSOR ? "SENSOR" : "EXIT");
		}

		// Internal sensors
		if (block->internal_sensor_count > 0) {
			log_info("  Internal sensors (%d):", block->internal_sensor_count);
			for (u32 j = 0; j < block->internal_sensor_count; j++) {
				const track_node *internal = block->internal_sensors[j];
				log_info("    [%d] %s", j, internal->name ? internal->name : "unnamed");
			}
		} else {
			log_info("  Internal sensors: none");
		}

		// Turnouts
		if (block->turnout_count > 0) {
			log_info("  Turnouts (%d):", block->turnout_count);
			for (u32 j = 0; j < block->turnout_count; j++) {
				const track_node *turnout = block->turnouts[j];
				log_info("    [%d] %s (num: %d)", j, turnout->name ? turnout->name : "unnamed",
					 turnout->num);
			}
		} else {
			log_info("  Turnouts: none");
		}

		// Connected blocks
		if (block->connected_block_count > 0) {
			log_info("  Connected blocks (%d):", block->connected_block_count);
			for (u32 j = 0; j < block->connected_block_count; j++) {
				track_block_t *connected = block->connected_blocks[j];
				log_info("    [%d] Block %d", j, connected->block_id);
			}
		} else {
			log_info("  Connected blocks: none");
		}

		// Reservation and occupancy state
		if (block->owner_train_id != 0) {
			log_info("  Reservation: RESERVED by train %d (time: %llu)", block->owner_train_id,
				 block->reservation_time);
		} else {
			log_info("  Reservation: FREE");
		}

		if (block->occupied) {
			log_info("  Occupancy: OCCUPIED (entry: %s, time: %llu)",
				 block->current_entry_sensor ? block->current_entry_sensor->name : "unknown",
				 block->occupancy_time);
		} else {
			log_info("  Occupancy: CLEAR");
		}

		log_info("");
	}

	log_info("=== END TRACK BLOCKS INFORMATION ===");
}

// ============================================================================
// Path and Block Integration
// ============================================================================

u32 conductor_find_blocks_on_path(const track_node *from, const track_node *to, track_block_t **blocks, u32 max_blocks,
				  conductor_task_data_t *data)
{
	if (!from || !to || !blocks || max_blocks == 0 || !data) {
		return 0;
	}

	u32 block_count = 0;

	// Find starting and destination blocks
	track_block_t *start_block = conductor_find_block_by_entry_node(from, data);
	track_block_t *dest_block = conductor_find_block_by_exit_node(to, data);

	if (!start_block || !dest_block) {
		return 0;
	}

	// If same block, just return it
	if (start_block == dest_block) {
		blocks[0] = start_block;
		return 1;
	}

	// Use BFS to find path through connected blocks
	track_block_t *queue[MAX_TRACK_BLOCKS];
	track_block_t *parent[MAX_TRACK_BLOCKS];
	bool visited[MAX_TRACK_BLOCKS];

	for (int i = 0; i < MAX_TRACK_BLOCKS; i++) {
		visited[i] = false;
		parent[i] = NULL;
	}

	int queue_head = 0, queue_tail = 0;
	queue[queue_tail++] = start_block;
	visited[start_block->block_id] = true;

	bool found_path = false;

	// BFS to find path
	while (queue_head < queue_tail && !found_path) {
		track_block_t *current = queue[queue_head++];

		if (current == dest_block) {
			found_path = true;
			break;
		}

		// Explore connected blocks
		for (u32 i = 0; i < current->connected_block_count; i++) {
			track_block_t *neighbor = current->connected_blocks[i];

			if (!visited[neighbor->block_id] && queue_tail < MAX_TRACK_BLOCKS) {
				visited[neighbor->block_id] = true;
				parent[neighbor->block_id] = current;
				queue[queue_tail++] = neighbor;
			}
		}
	}

	if (!found_path) {
		// No path found, just return start and destination blocks
		blocks[block_count++] = start_block;
		if (block_count < max_blocks) {
			blocks[block_count++] = dest_block;
		}
		return block_count;
	}

	// Reconstruct path from destination to start
	track_block_t *path_blocks[MAX_TRACK_BLOCKS];
	u32 path_length = 0;
	track_block_t *current = dest_block;

	while (current && path_length < MAX_TRACK_BLOCKS) {
		path_blocks[path_length++] = current;
		current = parent[current->block_id];
	}

	// Reverse path and copy to output
	for (int i = path_length - 1; i >= 0 && block_count < max_blocks; i--) {
		blocks[block_count++] = path_blocks[i];
	}

	return block_count;
}

marklin_error_t conductor_reserve_blocks(track_block_t **blocks, u32 block_count, u8 train_id)
{
	if (!blocks || block_count == 0 || train_id == 0) {
		log_warn("Multi-block reservation failed: invalid arguments (blocks=%p, count=%u, train_id=%d)", blocks,
			 block_count, train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	log_info("MULTI-BLOCK RESERVATION START: Train %d attempting to reserve %u blocks atomically", train_id,
		 block_count);

	// Log all blocks being requested for reservation
	for (u32 i = 0; i < block_count; i++) {
		if (blocks[i]) {
			log_debug("  - Block %d (owner=%d, occupied=%d)", blocks[i]->block_id,
				  blocks[i]->owner_train_id, blocks[i]->occupied);
		}
	}

	// First check if all blocks are available
	for (u32 i = 0; i < block_count; i++) {
		if (!conductor_is_block_available(blocks[i], train_id)) {
			log_info("MULTI-BLOCK RESERVATION FAILED: Block %d unavailable (owned by train %d)",
				 blocks[i] ? blocks[i]->block_id : 0, blocks[i] ? blocks[i]->owner_train_id : 0);
			return MARKLIN_ERROR_ALREADY_RESERVED;
		}
	}

	// Reserve all blocks
	u32 reserved_count = 0;
	for (u32 i = 0; i < block_count; i++) {
		marklin_error_t result = conductor_reserve_block(blocks[i], train_id);
		if (result != MARKLIN_ERROR_OK) {
			log_error("MULTI-BLOCK RESERVATION ROLLBACK: Failed at block %d, rolling back %u reservations",
				  blocks[i] ? blocks[i]->block_id : 0, reserved_count);
			// Rollback on failure
			for (u32 j = 0; j < i; j++) {
				conductor_release_block(blocks[j], train_id);
			}
			return result;
		}
		reserved_count++;
	}

	log_info("MULTI-BLOCK RESERVATION SUCCESS: Train %d reserved %u blocks atomically", train_id, block_count);

	return MARKLIN_ERROR_OK;
}
