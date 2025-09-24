#ifndef MARKLIN_CONDUCTOR_API_H
#define MARKLIN_CONDUCTOR_API_H

#include "marklin/common/track_node.h"
#include "marklin/train2/api.h"
#include "marklin/train/kinematics.h"
#include "marklin/conductor/block.h"

#include "marklin/error.h"
#include "types.h"

#define MARKLIN_CONDUCTOR_SERVER_NAME "conductor"

// Sensor state structure
typedef struct {
	u8 bank; // Sensor bank (0-4 for A-E)
	u8 sensor_id; // Sensor number within bank (1-16)
	u8 triggered; // Current sensor state (0 or 1)
	u64 last_triggered_tick; // Timestamp of last trigger
} marklin_sensor_state_t;

// Switch state structure
typedef struct {
	u8 switch_id; // Switch ID
	track_direction direction; // Current switch direction (DIR_STRAIGHT or DIR_CURVED)
	u64 last_changed_tick; // Timestamp of last direction change
} marklin_switch_state_t;

// Block reservation status enumeration
typedef enum { BLOCK_STATUS_FREE = 0, BLOCK_STATUS_RESERVED, BLOCK_STATUS_OCCUPIED } block_reservation_status_t;

// Block reservation data structure for message queue
typedef struct {
	u32 block_id; // Block identifier (0-29 for 30 blocks)
	u8 owner_train_id; // Train ID that owns the block (0 = free)
	block_reservation_status_t status; // Current reservation/occupancy status
	u64 timestamp; // When this status change occurred
	char entry_sensor_name[16]; // Name of entry sensor if occupied (empty string if not applicable)
} marklin_block_reservation_data_t;

marklin_error_t Marklin_SetSwitch(u8 switch_id, track_direction direction, u8 disengage_solenoid, bool force);

// Query the current state of multiple sensors
// sensors: array of sensor states to query (bank and sensor_id must be filled in)
// count: number of sensors to query
// Returns MARKLIN_ERROR_OK if at least one sensor was found
// Returns MARKLIN_ERROR_NOT_FOUND if no sensors were found
// For individual sensor states: triggered == 0xFF indicates sensor not found
marklin_error_t Marklin_GetSensorStates(marklin_sensor_state_t *sensors, u32 count);

// Path finding result structure (forward declaration)
typedef struct path_result path_result_t;

// Find a path between two nodes
// from: Starting track node
// to: Destination track node
// train_id: ID of the train requesting the path (for reservation checking)
// allow_reversal: Whether the train can reverse direction during the path
// use_block_exit_start: If true and 'from' is a block entry, use appropriate block exit as start
// excluded_blocks: Array of blocks to exclude from pathfinding (NULL for none)
// excluded_count: Number of blocks in excluded_blocks array
// result: Pointer to store the path result (must call Marklin_FreePath when done)
marklin_error_t Marklin_FindPath(const track_node *from, const track_node *to, u8 train_id, bool allow_reversal,
				 bool use_block_exit_start, const track_block_t **excluded_blocks, u32 excluded_count,
				 path_result_t *result);

// Free resources allocated by Marklin_FindPath
void Marklin_FreePath(path_result_t *result);

// Calculate next sensors a train should expect to hit
// current_location: Current position of the train
// direction: Current direction of the train
// expected_sensor: Pointer to store the primary expected sensor
// expected_distance: Pointer to store the expected distance to the next sensor
// Returns MARKLIN_ERROR_OK on success
marklin_error_t Marklin_GetNextSensors(const track_node *current_location, train_direction_t direction,
				       const track_node **expected_sensor, kinematic_distance_t *expected_distance);

// Calculate next TWO sensors for robust position tracking
// current_location: Current position of the train
// direction: Current direction of the train
// sensors: Array to store up to 2 expected sensors [primary, secondary]
// distances: Array to store distances to each sensor
// count: Pointer to store actual number of sensors found (1 or 2)
// Returns MARKLIN_ERROR_OK on success
marklin_error_t Marklin_GetNextTwoSensors(const track_node *current_location, train_direction_t direction,
					  const track_node **sensors, kinematic_distance_t *distances, u8 *count);

// Calculate actual distance between two track nodes following current switch states
// from: Starting track node
// to: Destination track node
// train_id: Train ID for path resolution (can be 0 for general calculation)
// raw_distance: Pointer to store raw calculated distance in mm
// effective_distance: Pointer to store resistance-compensated effective distance in mm
// Returns MARKLIN_ERROR_OK on success, MARKLIN_ERROR_NOT_FOUND if no path exists
marklin_error_t Marklin_CalculateTrackDistance(const track_node *from, const track_node *to, u8 train_id,
					       kinematic_distance_t *raw_distance,
					       kinematic_distance_t *effective_distance);

// Path activation stop reasons
typedef enum {
	PATH_ACTIVATION_STOP_END_OF_PATH = 0, // Reached end of path successfully
	PATH_ACTIVATION_STOP_BLOCK_UNAVAILABLE, // Hit an unavailable block
	PATH_ACTIVATION_STOP_MAX_BLOCKS_REACHED, // Reached max_blocks_to_reserve limit
	PATH_ACTIVATION_STOP_REVERSAL_POINT, // Stopped at reversal point
	PATH_ACTIVATION_STOP_PREVIOUSLY_RESERVED, // Hit a block already reserved in this activation session
	PATH_ACTIVATION_STOP_ERROR // Error during activation
} path_activation_stop_reason_t;

// Path activation result structure
typedef struct {
	const track_node *next_expected_sensor; // Next sensor the train should hit
	kinematic_distance_t next_expected_distance; // Distance to the next sensor
	u32 segment_distance; // Total distance of activated segment in mm
	// Block reservation information
	u32 blocks_reserved; // Number of blocks actually reserved
	u32 blocks_available_in_path; // Total blocks that could be reserved in full path
	const track_node *reserved_block_nodes[MAX_TRACK_BLOCKS]; // Array of reserved block nodes
	path_activation_stop_reason_t stop_reason; // Why activation stopped
	// Path extent information
	const track_node *furthest_activated_node; // Furthest node where path activation actually stopped
	kinematic_distance_t reversal_safety_distance; // Safety distance reserved past reversal
} marklin_path_activation_result_t;

// Activate a computed path for a train with configurable distance limits
// path: Path result from Marklin_FindPath
// train_id: ID of the train activating the path
// max_distance_to_reserve: Maximum distance to reserve in mm (0 = no limit)
// max_distance_past_reversal: Maximum distance to reserve past reversal point in mm (0 = no limit)
// current_sensor: Current sensor position of train (NULL for initial activation, sensor for continuation)
// current_offset_mm: Offset from current sensor in mm (ignored if current_sensor is NULL)
// result: Structure to store activation results
// Sets switches along the path and calculates next expected sensors
// When current_sensor is NULL, activates from end of path (initial activation)
// When current_sensor is provided, continues activation from that position with offset (path continuation)
// Returns MARKLIN_ERROR_OK on success, MARKLIN_ERROR_ALREADY_RESERVED if no blocks could be reserved
marklin_error_t Marklin_ActivatePath(path_result_t *path, u8 train_id, kinematic_distance_t max_distance_to_reserve,
				     const track_node *current_sensor, kinematic_distance_t current_offset_mm,
				     marklin_path_activation_result_t *result);

// ============================================================================
// Block Management Functions
// ============================================================================

// Reserve a specific block for a train
// train_id: ID of the train requesting the reservation
// block_node: A node within the block to be reserved (entry, exit, or internal)
// Returns MARKLIN_ERROR_OK on success, MARKLIN_ERROR_ALREADY_RESERVED if block unavailable
marklin_error_t Marklin_ReserveSpecificBlock(u8 train_id, const track_node *block_node);

// Check if a train owns a specific block
// train_id: ID of the train to check ownership for
// block_node: A node within the block to check (entry, exit, or internal)
// owns_block: Output parameter - true if train owns the block, false otherwise
// owner_train_id: Output parameter - ID of the train that owns the block (0 if free)
// Returns MARKLIN_ERROR_OK on success, MARKLIN_ERROR_NOT_FOUND if block not found
marklin_error_t Marklin_CheckBlockOwnership(u8 train_id, const track_node *block_node, bool *owns_block,
					    u8 *owner_train_id);

// ============================================================================
// Block Release Functions
// ============================================================================

// Release all blocks owned by a train, optionally keeping one
// train_id: ID of the train releasing blocks
// keep_block_node: A node within the block to keep reserved (NULL to release all blocks)
// Returns MARKLIN_ERROR_OK on success, MARKLIN_ERROR_NOT_FOUND if train owns no blocks
marklin_error_t Marklin_ReleaseTrainBlocks(u8 train_id, const track_node *keep_block_node);

// Release a specific block owned by a train
// train_id: ID of the train releasing the block
// block_node: A node within the block to be released (entry, exit, or internal)
// current_block_node: Block to ensure remains reserved (NULL to ignore for normal operation)
// Returns MARKLIN_ERROR_OK on success, MARKLIN_ERROR_NOT_OWNER if train doesn't own the block
marklin_error_t Marklin_ReleaseSpecificBlock(u8 train_id, const track_node *block_node,
					     const track_node *current_block_node);

#endif /* MARKLIN_CONDUCTOR_API_H */
