#ifndef MARKLIN_CONDUCTOR_BLOCK_H
#define MARKLIN_CONDUCTOR_BLOCK_H

#include "types.h"
#include "marklin/common/track_node.h"
#include "marklin/error.h"

typedef struct conductor_task_data_struct conductor_task_data_t;

// Maximum limits for block components
#define MAX_BOUNDARY_SENSORS_PER_BLOCK 8
#define MAX_INTERNAL_SENSORS_PER_BLOCK 8
#define MAX_TURNOUTS_PER_BLOCK 8
#define MAX_CONNECTED_BLOCKS_PER_BLOCK 8
#define MAX_TRACK_BLOCKS 32

/**
 * Track Block System
 *
 * A block is a contiguous, self-contained stretch of track bounded exclusively
 * by a designated set of entry-and-exit sensors. Every legal route between any
 * two boundary sensors remains entirely within the block
 *
 * Key Properties:
 * - Multiple boundary sensors (entry/exit points)
 * - May contain internal sensors (not boundaries)
 * - May contain turnouts (switches)
 * - Exclusive ownership by one train at a time
 * - Physical occupancy detection via sensors
 */

typedef struct track_block_struct track_block_t;

struct track_block_struct {
	u32 block_id;

	const track_node *entry_sensors[MAX_BOUNDARY_SENSORS_PER_BLOCK];
	u32 entry_sensor_count;
	const track_node *exit_sensors[MAX_BOUNDARY_SENSORS_PER_BLOCK];
	u32 exit_sensor_count;

	// Internal components
	const track_node *internal_sensors[MAX_INTERNAL_SENSORS_PER_BLOCK];
	u32 internal_sensor_count;
	const track_node *turnouts[MAX_TURNOUTS_PER_BLOCK];
	u32 turnout_count;

	track_block_t *connected_blocks[MAX_CONNECTED_BLOCKS_PER_BLOCK];
	u32 connected_block_count;

	// Reservation and occupancy state
	u8 owner_train_id; // 0 = free, otherwise train ID that reserved it
	u64 reservation_time;
	bool occupied; // Physical occupancy detected by sensor
	const track_node *current_entry_sensor; // Which entry sensor the train entered from
	u64 occupancy_time;
};

// ============================================================================
// Block Discovery and Initialization
// ============================================================================

void conductor_init_blocks(conductor_task_data_t *data);

// ============================================================================
// Block Query Functions
// ============================================================================

/**
 * Find the block that contains a given track node
 * Returns NULL if the node is not within any block
 */
track_block_t *conductor_find_block_containing_node(const track_node *node, conductor_task_data_t *data,
						    bool search_entry_node, bool search_exit_node,
						    bool search_internal_node, bool search_turnouts);

/**
 * Find a block by one of its boundary sensors
 * Returns NULL if the sensor is not a boundary of any block
 */
track_block_t *conductor_find_block_by_entry_node(const track_node *entry_node, conductor_task_data_t *data);
track_block_t *conductor_find_block_by_exit_node(const track_node *exit_node, conductor_task_data_t *data);

bool conductor_is_entry_node(const track_node *node, const track_block_t *block);
bool conductor_is_exit_node(const track_node *node, const track_block_t *block);
bool conductor_is_boundary_sensor(const track_node *node, const track_block_t *block);

/**
 * Get the next block in a given direction from an exit node
 */
track_block_t *conductor_get_adjacent_block(const track_node *exit_node, track_direction direction,
					    conductor_task_data_t *data);

void conductor_print_all_blocks_info(conductor_task_data_t *data);

// ============================================================================
// Block Helper Functions (used by hardcoded definitions)
// ============================================================================

/**
 * Helper functions for adding components to blocks during initialization
 */
bool conductor_add_entry_sensor(track_block_t *block, const track_node *sensor);
bool conductor_add_exit_sensor(track_block_t *block, const track_node *sensor);
bool conductor_add_internal_sensor(track_block_t *block, const track_node *sensor);
bool conductor_add_turnout(track_block_t *block, const track_node *turnout);

// ============================================================================
// Block Reservation Management
// ============================================================================

/**
 * Reserve a block for a train
 * Returns MARKLIN_ERROR_ALREADY_RESERVED if block is owned by another train
 */
marklin_error_t conductor_reserve_block(track_block_t *block, u8 train_id);

/**
 * Release a block reservation
 * Returns MARKLIN_ERROR_NOT_OWNER if the train doesn't own the block
 */
marklin_error_t conductor_release_block(track_block_t *block, u8 train_id);

/**
 * Check if a block is available for reservation by a train
 */
bool conductor_is_block_available(const track_block_t *block, u8 train_id);

// ============================================================================
// Block Occupancy Tracking
// ============================================================================

/**
 * Mark a block as physically occupied when a train enters
 * entry_sensor: The boundary sensor through which the train entered
 */
void conductor_mark_block_occupied(track_block_t *block, const track_node *entry_sensor);

/**
 * Mark a block as clear when the train exits
 */
void conductor_mark_block_clear(track_block_t *block);

/**
 * Update block occupancy based on sensor triggers
 * Called when any sensor is triggered
 */
void conductor_update_block_occupancy(const track_node *sensor, conductor_task_data_t *data);

// ============================================================================
// Turnout Management Within Blocks
// ============================================================================

track_block_t *conductor_find_block_containing_turnout_owned_by_train(const track_node *turnout, u8 train_id,
								      conductor_task_data_t *data);

/**
 * Set a turnout within a block
 * Only the train that owns the block can set its turnouts
 */
marklin_error_t conductor_set_turnout_in_block(track_block_t *block, const track_node *turnout,
					       track_direction direction, u8 train_id);

/**
 * Get all turnouts within a block
 */
u32 conductor_get_block_turnouts(const track_block_t *block, const track_node **turnouts, u32 max_turnouts);

// ============================================================================
// Path and Block Integration
// ============================================================================

/**
 * Find all blocks along a path
 * Returns the number of blocks found
 */
u32 conductor_find_blocks_on_path(const track_node *from, const track_node *to, track_block_t **blocks, u32 max_blocks,
				  conductor_task_data_t *data);

/**
 * Reserve multiple blocks atomically
 * Either all blocks are reserved or none are
 */
marklin_error_t conductor_reserve_blocks(track_block_t **blocks, u32 block_count, u8 train_id);

#endif /* MARKLIN_CONDUCTOR_BLOCK_H */
