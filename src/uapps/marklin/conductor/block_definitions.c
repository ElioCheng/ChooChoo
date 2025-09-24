#include "marklin/conductor/block_definitions.h"
#include "marklin/conductor/conductor.h"
#include "marklin/topology/track.h"
#include "string.h"
#include "klog.h"

#define LOG_MODULE "block_def"
#define LOG_LEVEL LOG_LEVEL_ERROR
#include "log.h"

#define COUNT_ARGS_CHAR(...) (sizeof((const char *[]){ __VA_ARGS__ }) / sizeof(const char *))
#define COUNT_ARGS_INT(...) (sizeof((int[]){ __VA_ARGS__ }) / sizeof(int))
#define BLOCK_DEF(id, entries, exits, internals, turnouts, connected) \
	{ .block_id = id,                                             \
	  .entry_sensor_names = { entries },                          \
	  .entry_sensor_count = COUNT_ARGS_CHAR(entries),             \
	  .exit_sensor_names = { exits },                             \
	  .exit_sensor_count = COUNT_ARGS_CHAR(exits),                \
	  .internal_sensor_names = { internals },                     \
	  .internal_sensor_count = COUNT_ARGS_CHAR(internals),        \
	  .turnout_names = { turnouts },                              \
	  .turnout_count = COUNT_ARGS_CHAR(turnouts),                 \
	  .connected_block_ids = { connected },                       \
	  .connected_block_count = COUNT_ARGS_INT(connected) }

#define ENTRY(...) __VA_ARGS__
#define EXIT(...) __VA_ARGS__
#define INTERNAL(...) __VA_ARGS__
#define TURNOUTS(...) __VA_ARGS__
#define CONNECTED(...) __VA_ARGS__
#define NONE

// ============================================================================
// Hardcoded Block Definitions for Track Layout A
// ============================================================================
#define TRACK_A_BLOCK_COUNT 23
static const hardcoded_block_def_t *get_track_a_blocks(void)
{
	static hardcoded_block_def_t track_a_blocks[TRACK_A_BLOCK_COUNT];
	static bool initialized = false;

	if (!initialized) {
		track_a_blocks[0] = (hardcoded_block_def_t)BLOCK_DEF(0, ENTRY("EN5", "A2"), EXIT("EX5", "A1"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1));
		track_a_blocks[1] = (hardcoded_block_def_t)BLOCK_DEF(
			1, ENTRY("A1", "C14", "C12", "A3", "A16", "A13"), EXIT("A2", "C13", "C11", "A4", "A15", "A14"),
			INTERNAL(NONE), TURNOUTS("BR12", "BR4", "BR11", "BR14"), CONNECTED(0, 2, 3, 4, 5, 6));
		track_a_blocks[2] = (hardcoded_block_def_t)BLOCK_DEF(2, ENTRY("C13", "E8"), EXIT("C14", "E7"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1, 7));
		track_a_blocks[3] = (hardcoded_block_def_t)BLOCK_DEF(3, ENTRY("EN4", "A14"), EXIT("EX4", "A13"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1));
		track_a_blocks[4] = (hardcoded_block_def_t)BLOCK_DEF(4, ENTRY("EN6", "A15"), EXIT("A16", "EX6"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1));
		track_a_blocks[5] = (hardcoded_block_def_t)BLOCK_DEF(5, ENTRY("A4", "B15"), EXIT("A3", "B16"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1, 9));
		track_a_blocks[6] = (hardcoded_block_def_t)BLOCK_DEF(6, ENTRY("C11", "E2", "D4"),
								     EXIT("C12", "D3", "E1"),
								     INTERNAL("E15", "E16", "B5", "B6"),
								     TURNOUTS("BR13"), CONNECTED(1, 18, 19));
		track_a_blocks[7] = (hardcoded_block_def_t)BLOCK_DEF(7, ENTRY("E7", "D8"), EXIT("E8", "D7"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(2, 12));
		track_a_blocks[8] = (hardcoded_block_def_t)BLOCK_DEF(
			8, ENTRY("A10", "A8", "A5", "C8", "EN8"), EXIT("A9", "A7", "A6", "C7", "EX8"),
			INTERNAL("A11", "A12"), TURNOUTS("BR1", "BR2", "BR3"), CONNECTED(13, 14, 15, 16));
		track_a_blocks[9] = (hardcoded_block_def_t)BLOCK_DEF(9, ENTRY("B16", "C9", "C6"),
								     EXIT("B15", "C10", "C5"), INTERNAL(NONE),
								     TURNOUTS("BR15"), CONNECTED(5, 16, 17));

		// track_a_blocks[10] = (hardcoded_block_def_t)BLOCK_DEF(10, ENTRY("C15", "D11"), EXIT("C16", "D12"),
		// 						      INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(16));
		track_a_blocks[11] = (hardcoded_block_def_t)BLOCK_DEF(11, ENTRY("C3", "EN3"), EXIT("C4", "EX3"),
								      INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(16));
		track_a_blocks[12] = (hardcoded_block_def_t)BLOCK_DEF(12, ENTRY("D6", "D7", "E9", "D10"),
								      EXIT("D5", "D8", "E10", "D9"), INTERNAL(NONE),
								      TURNOUTS("BR9", "BR8"), CONNECTED(7, 18, 21, 22));
		track_a_blocks[13] = (hardcoded_block_def_t)BLOCK_DEF(
			13, ENTRY("EN7", "A9"), EXIT("EX7", "A10"), INTERNAL("B7", "B8"), TURNOUTS(NONE), CONNECTED(8));
		track_a_blocks[14] = (hardcoded_block_def_t)BLOCK_DEF(14, ENTRY("EN10", "A7"), EXIT("EX10", "A8"),
								      INTERNAL("B11", "B12"), TURNOUTS(NONE),
								      CONNECTED(8));
		track_a_blocks[15] = (hardcoded_block_def_t)BLOCK_DEF(
			15, ENTRY("EN9", "A6"), EXIT("EX9", "A5"), INTERNAL("B9", "B10"), TURNOUTS(NONE), CONNECTED(8));

		track_a_blocks[16] = (hardcoded_block_def_t)BLOCK_DEF(16, ENTRY("C5", "C7", "C4", "E12"),
								      EXIT("C6", "C8", "C3", "E11"),
								      INTERNAL("C15", "C16", "D11", "D12"),
								      TURNOUTS("BR6", "BR18", "BR5", "BR7"),
								      CONNECTED(8, 9, 22, 11));
		track_a_blocks[17] = (hardcoded_block_def_t)BLOCK_DEF(17, ENTRY("C10", "B2", "C1"),
								      EXIT("C9", "C2", "B1"), INTERNAL("B3", "B4"),
								      TURNOUTS("BR16"), CONNECTED(9, 19, 20));
		track_a_blocks[18] = (hardcoded_block_def_t)BLOCK_DEF(18, ENTRY("D3", "D2", "D5"),
								      EXIT("D4", "D6", "D1"),
								      INTERNAL("E5", "E6", "E3", "E4"),
								      TURNOUTS("BR10"), CONNECTED(6, 19, 12));
		track_a_blocks[19] = (hardcoded_block_def_t)BLOCK_DEF(
			19, ENTRY("E1", "EN2", "D1", "B13", "EN1", "C2"), EXIT("E2", "EX2", "D2", "B14", "EX1", "C1"),
			INTERNAL(NONE), TURNOUTS("BR153", "BR154", "BR155", "BR156"), CONNECTED(6, 18, 17, 10));
		track_a_blocks[20] = (hardcoded_block_def_t)BLOCK_DEF(
			20, ENTRY("B1", "D13"), EXIT("B2", "D14"), INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(17, 10));
		track_a_blocks[21] = (hardcoded_block_def_t)BLOCK_DEF(
			21, ENTRY("E14", "E10"), EXIT("E13", "E9"), INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(12, 10));
		track_a_blocks[22] = (hardcoded_block_def_t)BLOCK_DEF(
			22, ENTRY("D9", "E11"), EXIT("D10", "E12"), INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(12, 16));

		track_a_blocks[10] = (hardcoded_block_def_t)BLOCK_DEF(10, ENTRY("B14", "D14", "E13"),
								      EXIT("B13", "D13", "E14"), INTERNAL("D15", "D16"),
								      TURNOUTS("BR17"), CONNECTED(21, 20, 19));
	}

	return track_a_blocks;
}

static u32 get_track_a_block_count(void)
{
	return TRACK_A_BLOCK_COUNT;
}

// ============================================================================
// Hardcoded Block Definitions for Track Layout B
// ============================================================================
#define TRACK_B_BLOCK_COUNT 23
static const hardcoded_block_def_t *get_track_b_blocks(void)
{
	static hardcoded_block_def_t track_b_blocks[TRACK_B_BLOCK_COUNT];
	static bool initialized = false;

	if (!initialized) {
		track_b_blocks[0] = (hardcoded_block_def_t)BLOCK_DEF(0, ENTRY("EN5", "A2"), EXIT("EX5", "A1"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1));
		track_b_blocks[1] = (hardcoded_block_def_t)BLOCK_DEF(
			1, ENTRY("A1", "C14", "C12", "A3", "A16", "A13"), EXIT("A2", "C13", "C11", "A4", "A15", "A14"),
			INTERNAL(NONE), TURNOUTS("BR12", "BR4", "BR11", "BR14"), CONNECTED(0, 2, 3, 4, 5, 6));
		track_b_blocks[2] = (hardcoded_block_def_t)BLOCK_DEF(2, ENTRY("C13", "E8"), EXIT("C14", "E7"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1, 7));
		track_b_blocks[3] = (hardcoded_block_def_t)BLOCK_DEF(3, ENTRY("EN4", "A14"), EXIT("EX4", "A13"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1));
		track_b_blocks[4] = (hardcoded_block_def_t)BLOCK_DEF(4, ENTRY("A15", "A12"), EXIT("A16", "A11"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1, 8));
		track_b_blocks[5] = (hardcoded_block_def_t)BLOCK_DEF(5, ENTRY("A4", "B15"), EXIT("A3", "B16"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(1, 9));
		track_b_blocks[6] = (hardcoded_block_def_t)BLOCK_DEF(6, ENTRY("C11", "E2", "D4"),
								     EXIT("C12", "D3", "E1"),
								     INTERNAL("E15", "E16", "B5", "B6"),
								     TURNOUTS("BR13"), CONNECTED(1, 18, 19));
		track_b_blocks[7] = (hardcoded_block_def_t)BLOCK_DEF(7, ENTRY("E7", "D8"), EXIT("E8", "D7"),
								     INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(2, 12));
		track_b_blocks[8] = (hardcoded_block_def_t)BLOCK_DEF(8, ENTRY("A11", "A10", "A8", "A5", "C8"),
								     EXIT("A12", "A9", "A7", "A6", "C7"),
								     INTERNAL(NONE), TURNOUTS("BR1", "BR2", "BR3"),
								     CONNECTED(4, 13, 14, 15, 16));
		track_b_blocks[9] = (hardcoded_block_def_t)BLOCK_DEF(9, ENTRY("B16", "C9", "C6"),
								     EXIT("B15", "C10", "C5"), INTERNAL(NONE),
								     TURNOUTS("BR15"), CONNECTED(5, 16, 17));

		// track_b_blocks[10] = (hardcoded_block_def_t)BLOCK_DEF(10, ENTRY("C15", "D11"), EXIT("C16", "D12"),
		// 						      INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(16));
		track_b_blocks[11] = (hardcoded_block_def_t)BLOCK_DEF(11, ENTRY("C3", "EN3"), EXIT("C4", "EX3"),
								      INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(16));
		track_b_blocks[12] = (hardcoded_block_def_t)BLOCK_DEF(12, ENTRY("D6", "D7", "E9", "D10"),
								      EXIT("D5", "D8", "E10", "D9"), INTERNAL(NONE),
								      TURNOUTS("BR9", "BR8"), CONNECTED(7, 18, 21, 22));
		track_b_blocks[13] = (hardcoded_block_def_t)BLOCK_DEF(
			13, ENTRY("EN7", "A9"), EXIT("EX7", "A10"), INTERNAL("B7", "B8"), TURNOUTS(NONE), CONNECTED(8));
		track_b_blocks[14] = (hardcoded_block_def_t)BLOCK_DEF(14, ENTRY("EN10", "A7"), EXIT("EX10", "A8"),
								      INTERNAL("B11", "B12"), TURNOUTS(NONE),
								      CONNECTED(8));
		track_b_blocks[15] = (hardcoded_block_def_t)BLOCK_DEF(
			15, ENTRY("EN9", "A6"), EXIT("EX9", "A5"), INTERNAL("B9", "B10"), TURNOUTS(NONE), CONNECTED(8));
		track_b_blocks[16] = (hardcoded_block_def_t)BLOCK_DEF(16, ENTRY("C5", "C7", "C4", "E12"),
								      EXIT("C6", "C8", "C3", "E11"),
								      INTERNAL("C15", "C16", "D11", "D12"),
								      TURNOUTS("BR6", "BR18", "BR5", "BR7"),
								      CONNECTED(8, 9, 22, 11));
		track_b_blocks[17] = (hardcoded_block_def_t)BLOCK_DEF(17, ENTRY("C10", "B2", "C1"),
								      EXIT("C9", "C2", "B1"), INTERNAL("B3", "B4"),
								      TURNOUTS("BR16"), CONNECTED(9, 19, 20));
		track_b_blocks[18] = (hardcoded_block_def_t)BLOCK_DEF(18, ENTRY("D3", "D2", "D5"),
								      EXIT("D4", "D6", "D1"),
								      INTERNAL("E5", "E6", "E3", "E4"),
								      TURNOUTS("BR10"), CONNECTED(6, 19, 12));
		track_b_blocks[19] = (hardcoded_block_def_t)BLOCK_DEF(
			19, ENTRY("E1", "EN2", "D1", "B13", "EN1", "C2"), EXIT("E2", "EX2", "D2", "B14", "EX1", "C1"),
			INTERNAL(NONE), TURNOUTS("BR153", "BR154", "BR155", "BR156"), CONNECTED(6, 18, 17, 10));
		track_b_blocks[20] = (hardcoded_block_def_t)BLOCK_DEF(
			20, ENTRY("B1", "D13"), EXIT("B2", "D14"), INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(17, 10));
		track_b_blocks[21] = (hardcoded_block_def_t)BLOCK_DEF(
			21, ENTRY("E14", "E10"), EXIT("E13", "E9"), INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(12, 10));
		track_b_blocks[22] = (hardcoded_block_def_t)BLOCK_DEF(
			22, ENTRY("D9", "E11"), EXIT("D10", "E12"), INTERNAL(NONE), TURNOUTS(NONE), CONNECTED(12, 16));

		track_b_blocks[10] = (hardcoded_block_def_t)BLOCK_DEF(10, ENTRY("B14", "D14", "E13"),
								      EXIT("B13", "D13", "E14"), INTERNAL("D15", "D16"),
								      TURNOUTS("BR17"), CONNECTED(21, 20, 19));

		initialized = true;
	}

	return track_b_blocks;
}

static u32 get_track_b_block_count(void)
{
	return TRACK_B_BLOCK_COUNT;
}

// ============================================================================
// Implementation Functions
// ============================================================================

const hardcoded_block_def_t *conductor_get_block_definitions(marklin_track_type_t layout, u32 *block_count)
{
	if (!block_count) {
		return NULL;
	}

	switch (layout) {
	case MARKLIN_TRACK_TYPE_A:
		*block_count = get_track_a_block_count();
		return get_track_a_blocks();

	case MARKLIN_TRACK_TYPE_B: {
		*block_count = get_track_b_block_count();
		return get_track_b_blocks();
	}

	default:
		*block_count = 0;
		return NULL;
	}
}

const track_node *conductor_resolve_sensor_name(const char *sensor_name, conductor_task_data_t *data)
{
	if (!sensor_name || !data || !data->track_nodes) {
		return NULL;
	}

	return marklin_find_node_by_name(data->track_nodes, data->track_size, sensor_name);
}

const track_node *conductor_resolve_turnout_name(const char *turnout_name, conductor_task_data_t *data)
{
	if (!turnout_name || !data || !data->track_nodes) {
		return NULL;
	}

	const track_node *node = marklin_find_node_by_name(data->track_nodes, data->track_size, turnout_name);

	if (node && node->type == NODE_BRANCH) {
		return node;
	}

	return NULL;
}

void conductor_init_hardcoded_blocks(conductor_task_data_t *data, marklin_track_type_t layout)
{
	if (!data) {
		klog_error("Invalid data pointer");
		return;
	}

	u32 def_count;
	const hardcoded_block_def_t *block_defs = conductor_get_block_definitions(layout, &def_count);

	if (!block_defs || def_count == 0) {
		klog_error("No block definitions found for layout %d", layout);
		return;
	}

	data->track_block_count = 0;

	for (u32 i = 0; i < def_count && i < MAX_TRACK_BLOCKS; i++) {
		const hardcoded_block_def_t *def = &block_defs[i];
		track_block_t *block = &data->track_blocks[data->track_block_count];

		memset(block, 0, sizeof(track_block_t));
		block->block_id = def->block_id;

		for (u32 j = 0; j < def->entry_sensor_count && j < MAX_BOUNDARY_SENSORS_PER_BLOCK; j++) {
			const track_node *sensor = conductor_resolve_sensor_name(def->entry_sensor_names[j], data);
			if (sensor) {
				conductor_add_entry_sensor(block, sensor);
			} else {
				klog_warn("Failed to resolve entry sensor: %s", def->entry_sensor_names[j]);
			}
		}

		for (u32 j = 0; j < def->exit_sensor_count && j < MAX_BOUNDARY_SENSORS_PER_BLOCK; j++) {
			const track_node *sensor = conductor_resolve_sensor_name(def->exit_sensor_names[j], data);
			if (sensor) {
				conductor_add_exit_sensor(block, sensor);
			} else {
				klog_warn("Failed to resolve exit sensor: %s", def->exit_sensor_names[j]);
			}
		}

		for (u32 j = 0; j < def->internal_sensor_count && j < MAX_INTERNAL_SENSORS_PER_BLOCK; j++) {
			const track_node *sensor = conductor_resolve_sensor_name(def->internal_sensor_names[j], data);
			if (sensor) {
				conductor_add_internal_sensor(block, sensor);
			} else {
				klog_warn("Failed to resolve internal sensor: %s", def->internal_sensor_names[j]);
			}
		}

		for (u32 j = 0; j < def->turnout_count && j < MAX_TURNOUTS_PER_BLOCK; j++) {
			const track_node *turnout = conductor_resolve_turnout_name(def->turnout_names[j], data);
			if (turnout) {
				conductor_add_turnout(block, turnout);
			} else {
				klog_warn("Failed to resolve turnout: %s", def->turnout_names[j]);
			}
		}

		block->owner_train_id = 0;
		block->reservation_time = 0;
		block->occupied = false;
		block->current_entry_sensor = NULL;
		block->occupancy_time = 0;
		block->connected_block_count = 0;

		data->track_block_count++;
	}

	for (u32 i = 0; i < def_count && i < MAX_TRACK_BLOCKS; i++) {
		const hardcoded_block_def_t *def = &block_defs[i];
		track_block_t *block = &data->track_blocks[i];

		block->connected_block_count = 0;
		for (u32 j = 0; j < def->connected_block_count && j < MAX_CONNECTED_BLOCKS_PER_BLOCK; j++) {
			u32 connected_id = def->connected_block_ids[j];

			track_block_t *connected_block = NULL;
			for (int k = 0; k < data->track_block_count; k++) {
				if (data->track_blocks[k].block_id == connected_id) {
					connected_block = &data->track_blocks[k];
					break;
				}
			}

			if (connected_block) {
				block->connected_blocks[block->connected_block_count++] = connected_block;
			} else {
				klog_warn("Failed to resolve connected block ID %d for block %d", connected_id,
					  block->block_id);
			}
		}
	}

	if (!conductor_validate_initialized_blocks(data)) {
		klog_error("Block validation failed - blocks may be misconfigured");
		Panic("Invalid block configuration detected");
	}

	// conductor_print_all_blocks_info(data);
}

// ============================================================================
// Validation Helper Functions
// ============================================================================

// Check if a sensor exists in any initialized block
static bool is_sensor_in_blocks(const track_node *sensor, conductor_task_data_t *data)
{
	for (int i = 0; i < data->track_block_count; i++) {
		const track_block_t *block = &data->track_blocks[i];

		// Check entry sensors
		for (u32 j = 0; j < block->entry_sensor_count; j++) {
			if (block->entry_sensors[j] == sensor) {
				return true;
			}
		}

		// Check exit sensors
		for (u32 j = 0; j < block->exit_sensor_count; j++) {
			if (block->exit_sensors[j] == sensor) {
				return true;
			}
		}

		// Check internal sensors
		for (u32 j = 0; j < block->internal_sensor_count; j++) {
			if (block->internal_sensors[j] == sensor) {
				return true;
			}
		}
	}

	return false;
}

// Check if all track sensors are present in initialized blocks
static bool validate_all_sensors_present(conductor_task_data_t *data)
{
	bool all_present = true;

	for (int i = 0; i < data->track_size; i++) {
		const track_node *node = &data->track_nodes[i];

		if (marklin_is_boundary_node(node) && node->name) {
			if (!is_sensor_in_blocks(node, data)) {
				klog_error("Sensor %s (type %d) not found in any initialized block", node->name,
					   node->type);
				all_present = false;
			}
		}
	}

	return all_present;
}

// Check if a switch exists in any initialized block
static bool is_switch_in_blocks(const track_node *switch_node, conductor_task_data_t *data)
{
	for (int i = 0; i < data->track_block_count; i++) {
		const track_block_t *block = &data->track_blocks[i];

		for (u32 j = 0; j < block->turnout_count; j++) {
			if (block->turnouts[j] == switch_node) {
				return true;
			}
		}
	}

	return false;
}

// Check if all track switches are present in initialized blocks
static bool validate_all_switches_present(conductor_task_data_t *data)
{
	bool all_present = true;

	for (int i = 0; i < data->track_size; i++) {
		const track_node *node = &data->track_nodes[i];

		if (node->type == NODE_BRANCH && node->name) {
			if (!is_switch_in_blocks(node, data)) {
				klog_error("Switch %s not found in any initialized block", node->name);
				all_present = false;
			}
		}
	}

	return all_present;
}

// Check if all blocks are connected (no isolated blocks)
static bool validate_block_connectivity(conductor_task_data_t *data)
{
	if (data->track_block_count == 0)
		return true;
	if (data->track_block_count == 1)
		return true;

	// Use BFS to check if all blocks are reachable from block 0
	bool visited[MAX_TRACK_BLOCKS] = { false };
	u32 queue[MAX_TRACK_BLOCKS];
	u32 queue_head = 0, queue_tail = 0;

	queue[queue_tail++] = 0;
	visited[0] = true;
	u32 visited_count = 1;

	while (queue_head < queue_tail) {
		u32 current_idx = queue[queue_head++];
		const track_block_t *current = &data->track_blocks[current_idx];

		for (u32 i = 0; i < current->connected_block_count; i++) {
			track_block_t *connected = current->connected_blocks[i];

			for (int j = 0; j < data->track_block_count; j++) {
				if (&data->track_blocks[j] == connected && !visited[j]) {
					visited[j] = true;
					visited_count++;
					queue[queue_tail++] = j;
					break;
				}
			}
		}
	}

	if (visited_count < (u32)data->track_block_count) {
		klog_error("Block connectivity check failed: only %d of %d blocks are connected", visited_count,
			   data->track_block_count);
		for (int i = 0; i < data->track_block_count; i++) {
			if (!visited[i]) {
				klog_error("  Block %d is not connected to the main network",
					   data->track_blocks[i].block_id);
			}
		}
		return false;
	}

	return true;
}

// Check entry/exit sensor pairing (entry's reverse should be exit in same block)
static bool validate_entry_exit_pairing(conductor_task_data_t *data)
{
	bool all_valid = true;

	for (int i = 0; i < data->track_block_count; i++) {
		const track_block_t *block = &data->track_blocks[i];

		for (u32 j = 0; j < block->entry_sensor_count; j++) {
			const track_node *entry = block->entry_sensors[j];
			if (!entry)
				continue;

			if (entry->reverse) {
				const track_node *reverse = entry->reverse;

				bool found = false;
				for (u32 k = 0; k < block->exit_sensor_count; k++) {
					if (block->exit_sensors[k] == reverse) {
						found = true;
						break;
					}
				}

				if (!found) {
					klog_error(
						"Block %d: Entry sensor %s has reverse %s which is not in exit sensors",
						block->block_id, entry->name ? entry->name : "unnamed",
						reverse->name ? reverse->name : "unnamed");
					all_valid = false;
				}
			}
		}
	}

	return all_valid;
}

// Check for duplicate switches across blocks
static bool validate_no_duplicate_switches(conductor_task_data_t *data)
{
	bool no_duplicates = true;

	// Check each pair of blocks
	for (int i = 0; i < data->track_block_count; i++) {
		for (int j = i + 1; j < data->track_block_count; j++) {
			const track_block_t *block1 = &data->track_blocks[i];
			const track_block_t *block2 = &data->track_blocks[j];

			for (u32 k = 0; k < block1->turnout_count; k++) {
				for (u32 l = 0; l < block2->turnout_count; l++) {
					if (block1->turnouts[k] == block2->turnouts[l]) {
						klog_error("Switch %s appears in both block %d and block %d",
							   block1->turnouts[k]->name ? block1->turnouts[k]->name :
										       "unnamed",
							   block1->block_id, block2->block_id);
						no_duplicates = false;
					}
				}
			}
		}
	}

	return no_duplicates;
}

// Check for duplicate internal sensors across blocks
static bool validate_no_duplicate_internals(conductor_task_data_t *data)
{
	bool no_duplicates = true;

	// Check each pair of blocks
	for (int i = 0; i < data->track_block_count; i++) {
		for (int j = i + 1; j < data->track_block_count; j++) {
			const track_block_t *block1 = &data->track_blocks[i];
			const track_block_t *block2 = &data->track_blocks[j];

			for (u32 k = 0; k < block1->internal_sensor_count; k++) {
				for (u32 l = 0; l < block2->internal_sensor_count; l++) {
					if (block1->internal_sensors[k] == block2->internal_sensors[l]) {
						klog_error("Internal sensor %s appears in both block %d and block %d",
							   block1->internal_sensors[k]->name ?
								   block1->internal_sensors[k]->name :
								   "unnamed",
							   block1->block_id, block2->block_id);
						no_duplicates = false;
					}
				}
			}
		}
	}

	return no_duplicates;
}

// Check block IDs match array indices
static bool validate_block_ids(conductor_task_data_t *data)
{
	bool all_valid = true;

	for (int i = 0; i < data->track_block_count; i++) {
		if (data->track_blocks[i].block_id != (u32)i) {
			klog_error("Block at index %d has ID %d - ID must match array index", i,
				   data->track_blocks[i].block_id);
			all_valid = false;
		}
	}

	return all_valid;
}

// Check bidirectional connectivity
static bool validate_bidirectional_connectivity(conductor_task_data_t *data)
{
	bool all_valid = true;

	for (int i = 0; i < data->track_block_count; i++) {
		const track_block_t *block = &data->track_blocks[i];

		// For each connected block, verify it connects back
		for (u32 j = 0; j < block->connected_block_count; j++) {
			track_block_t *connected_block = block->connected_blocks[j];

			if (!connected_block) {
				klog_error("Block %d has NULL connected block at index %d", block->block_id, j);
				all_valid = false;
				continue;
			}

			bool connects_back = false;
			for (u32 k = 0; k < connected_block->connected_block_count; k++) {
				if (connected_block->connected_blocks[k] == block) {
					connects_back = true;
					break;
				}
			}

			if (!connects_back) {
				klog_error("Block %d connects to block %d, but block %d does not connect back",
					   block->block_id, connected_block->block_id, connected_block->block_id);
				all_valid = false;
			}
		}
	}

	return all_valid;
}

// ============================================================================
// Main Validation Function
// ============================================================================

bool conductor_validate_initialized_blocks(conductor_task_data_t *data)
{
	if (!data) {
		klog_error("Invalid data pointer for validation");
		return false;
	}

	if (data->track_block_count == 0) {
		klog_error("No initialized blocks found");
		return false;
	}

	klog_info("Validating %d initialized blocks...", data->track_block_count);

	bool valid = true;

	// 1. Check all sensors are present
	klog_info("Checking all sensors are in blocks...");
	if (!validate_all_sensors_present(data)) {
		valid = false;
	}

	// 2. Check all switches are present
	klog_info("Checking all switches are in blocks...");
	if (!validate_all_switches_present(data)) {
		valid = false;
	}

	// 3. Check block connectivity
	klog_info("Checking block connectivity...");
	if (!validate_block_connectivity(data)) {
		valid = false;
	}

	// 4. Check entry/exit sensor pairing
	klog_info("Checking entry/exit sensor pairing...");
	if (!validate_entry_exit_pairing(data)) {
		valid = false;
	}

	// 5. Check no duplicate switches
	klog_info("Checking for duplicate switches...");
	if (!validate_no_duplicate_switches(data)) {
		valid = false;
	}

	// 6. Check no duplicate internal sensors
	klog_info("Checking for duplicate internal sensors...");
	if (!validate_no_duplicate_internals(data)) {
		valid = false;
	}

	// 7. Check block IDs match indices
	klog_info("Checking block IDs match array indices...");
	if (!validate_block_ids(data)) {
		valid = false;
	}

	// 8. Check bidirectional connectivity
	klog_info("Checking bidirectional connectivity...");
	if (!validate_bidirectional_connectivity(data)) {
		valid = false;
	}

	if (valid) {
		klog_info("Block validation PASSED");
	} else {
		klog_error("Block validation FAILED");
	}

	return valid;
}
