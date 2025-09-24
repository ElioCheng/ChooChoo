#ifndef MARKLIN_CONDUCTOR_H
#define MARKLIN_CONDUCTOR_H

#include "types.h"
#include "dlist.h"
#include "marklin/common/track_node.h"
#include "marklin/conductor/sensor.h"
#include "marklin/conductor/switch.h"
#include "marklin/conductor/api.h"
#include "marklin/conductor/path.h"
#include "marklin/train/kinematics.h"
#include "marklin/conductor/sensor.h"
#include "marklin/conductor/block.h"
#include "marklin/error.h"
#include "marklin/topology/api.h"

#define MARKLIN_CONDUCTOR_TASK_PRIORITY 4

// ############################################################################
// # Request/Reply Types
// ############################################################################

// Conductor request types
typedef enum {
	MARKLIN_CONDUCTOR_REQ_ON_SENSOR_DATA,
	MARKLIN_CONDUCTOR_REQ_SET_SWITCH,
	MARKLIN_CONDUCTOR_REQ_GET_SENSOR_STATES,
	MARKLIN_CONDUCTOR_REQ_FIND_PATH,
	MARKLIN_CONDUCTOR_REQ_GET_NEXT_SENSORS,
	MARKLIN_CONDUCTOR_REQ_GET_NEXT_TWO_SENSORS,
	MARKLIN_CONDUCTOR_REQ_CALCULATE_DISTANCE,
	MARKLIN_CONDUCTOR_REQ_ACTIVATE_PATH,
	MARKLIN_CONDUCTOR_REQ_RELEASE_TRAIN_BLOCKS,
	MARKLIN_CONDUCTOR_REQ_RELEASE_SPECIFIC_BLOCK,
	MARKLIN_CONDUCTOR_REQ_RESERVE_SPECIFIC_BLOCK,
	MARKLIN_CONDUCTOR_REQ_CHECK_BLOCK_OWNERSHIP,
	MARKLIN_CONDUCTOR_REQ_FREE_PATH,
} marklin_conductor_request_type_t;

// Conductor request structure
typedef struct {
	marklin_conductor_request_type_t type;
	union {
		struct {
			u16 *sensor_data;
			u32 tick;
		} sensor_data;
		struct {
			u8 switch_id;
			track_direction direction;
			u8 disengage_solenoid;
			bool force;
		} set_switch;
		struct {
			marklin_sensor_state_t *sensors;
			u32 count;
		} get_sensor_states;
		struct {
			const track_node *from;
			const track_node *to;
			u8 train_id;
			bool allow_reversal;
			bool use_block_exit_start;
			const track_block_t **excluded_blocks;
			u32 excluded_count;
			path_result_t *result;
		} find_path;
		struct {
			const track_node *current_location;
			train_direction_t direction;
			const track_node **expected_sensor;
			kinematic_distance_t *expected_distance;
		} get_next_sensors;
		struct {
			const track_node *current_location;
			train_direction_t direction;
			const track_node **sensors;
			kinematic_distance_t *distances;
			u8 *count;
		} get_next_two_sensors;
		struct {
			const track_node *from;
			const track_node *to;
			u8 train_id;
		} calculate_distance;
		struct {
			path_result_t *path;
			u8 train_id;
			kinematic_distance_t max_distance_to_reserve;
			const track_node *current_sensor;
			kinematic_distance_t current_offset_mm;
			marklin_path_activation_result_t *result;
		} activate_path;
		struct {
			u8 train_id;
			const track_node *keep_block_node; // NULL to release all blocks
		} release_train_blocks;
		struct {
			u8 train_id;
			const track_node *block_node;
			const track_node *current_block_node; // Block to ensure remains reserved (NULL to ignore)
		} release_specific_block;
		struct {
			u8 train_id;
			const track_node *block_node;
		} reserve_specific_block;
		struct {
			u8 train_id;
			const track_node *block_node;
		} check_block_ownership;
		struct {
			path_result_t *path;
		} free_path;
	};
} marklin_conductor_request_t;

// Conductor reply structure
typedef struct {
	marklin_error_t error;
	union {
		struct {
			kinematic_distance_t raw_distance;
			kinematic_distance_t effective_distance;
		} calculate_distance;
		struct {
			bool owns_block;
			u8 owner_train_id; // ID of the train that owns the block (0 if free)
		} check_block_ownership;
	};
} marklin_conductor_reply_t;

// ############################################################################
// # Deadlock Detection Types
// ############################################################################

#define MAX_TRAINS 8
#define MAX_BLOCKING_BLOCKS 8
#define DEADLOCK_DETECTION_WINDOW_MS 5000  // 5 second window for detecting patterns

// Structure to track failed path requests for deadlock detection
typedef struct {
	u8 train_id;
	const track_node *requested_from;
	const track_node *requested_to;
	track_block_t *blocking_blocks[MAX_BLOCKING_BLOCKS];
	u32 blocking_block_count;
	u64 failure_time;
	bool active;
} failed_path_request_t;

// Structure to manage deadlock resolution
typedef struct {
	u8 deadlocked_trains[MAX_TRAINS];
	u32 deadlocked_count;
	u64 detection_time;
	u8 resolution_priority_train;
	bool resolution_in_progress;
} deadlock_context_t;

// ############################################################################
// # Lookup Table Types
// ############################################################################

// Sensor lookup entry
typedef struct {
	const track_node *sensor_node; // Pointer to the sensor track node
	marklin_sensor_state_t state; // Current sensor state
} sensor_lookup_entry_t;

// Switch lookup entry
typedef struct {
	const track_node *switch_node; // Pointer to the switch track node (NODE_BRANCH)
	marklin_switch_state_t state; // Current switch state
} switch_lookup_entry_t;

// ############################################################################
// # Conductor Task Data
// ############################################################################

// Conductor task state (stack allocated)
typedef struct conductor_task_data_struct {
	int clock_server_tid;
	int command_server_tid;

	// Track state
	const track_node *track_nodes;
	int track_size;
	marklin_track_type_t track_type;

	// Sensor lookup table
	sensor_lookup_entry_t sensor_lookup[MARKLIN_SENSOR_BANK_COUNT * 16];
	int sensor_count;

	// Switch lookup table
	switch_lookup_entry_t switch_lookup[MARKLIN_SWITCH_MAX_COUNT];
	int switch_count;

	// Track blocks for reservation system
	track_block_t track_blocks[MAX_TRACK_BLOCKS];
	int track_block_count;

	bool sensor_blacklist_cache[5][16];

	// Path memory pools
	path_node_pool_t path_pools[MAX_CONCURRENT_PATHS];
	struct dlist_node free_path_pools;

	// Deadlock detection data
	failed_path_request_t recent_failures[MAX_TRAINS];
	u32 failure_count;
	deadlock_context_t deadlock_context;

} conductor_task_data_t;

// ############################################################################
// # Public Function Declarations
// ############################################################################

// Main conductor task entry point
void marklin_conductor_task(void);

// Initialize conductor (placeholder for future use)
void marklin_conductor_init(void);

// ############################################################################
// # Deadlock Detection Functions
// ############################################################################

// Initialize deadlock detection data structures
void conductor_init_deadlock_detection(conductor_task_data_t *data);

// Record a failed path request for deadlock analysis
void conductor_record_path_failure(conductor_task_data_t *data, u8 train_id, const track_node *from, 
				   const track_node *to, track_block_t **blocking_blocks, u32 blocking_count);

// Check if a deadlock condition exists and identify involved trains
bool conductor_detect_deadlock(conductor_task_data_t *data);

// Attempt to resolve a detected deadlock by excluding blocks for lower priority train
bool conductor_resolve_deadlock(conductor_task_data_t *data, u8 requesting_train_id,
			       const track_block_t ***excluded_blocks, u32 *excluded_count);

// Clean up expired failure records
void conductor_cleanup_old_failures(conductor_task_data_t *data);

// ############################################################################
// # Helper Functions for Internal Use
// ############################################################################

// Initialize lookup tables
void conductor_init_sensor_lookup(conductor_task_data_t *data);
void conductor_init_switch_lookup(conductor_task_data_t *data);

// Switch lookup helper
switch_lookup_entry_t *conductor_get_switch_lookup_entry(u8 switch_id);

#endif /* MARKLIN_CONDUCTOR_H */
