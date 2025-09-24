#ifndef MARKLIN_TRAIN_H
#define MARKLIN_TRAIN_H

#include "types.h"
#include "marklin/train2/api.h"
#include "marklin/common/track_node.h"
#include "marklin/conductor/path.h"
#include "marklin/conductor/block.h"
#include "marklin/conductor/api.h"
#include "marklin/msgqueue/api.h"
#include "marklin/train/kinematics.h"
#include "marklin/error.h"

// ############################################################################
// # Train Kinematic Model Data Structures
// ############################################################################

// Per-speed-level kinematic parameters
typedef struct {
	kinematic_velocity_t velocity; // Steady-state velocity (mm/tick)
	kinematic_accel_t acceleration; // Acceleration to this speed (mm/tick²)
	kinematic_accel_t deceleration; // Deceleration from this speed (mm/tick²)
	kinematic_distance_t stop_distance; // Distance to stop from this speed
	kinematic_time_t stop_time; // Time to stop from this speed

	// Last update information
	kinematic_time_t last_velocity_update; // When velocity was last measured
	kinematic_time_t last_acceleration_update; // When acceleration was last measured
	kinematic_time_t last_stop_update; // When stop distance was last measured
} kinematic_speed_params_t;

// Complete kinematic model for a single train
typedef struct {
	u8 train_id; // Train identifier

	// Per-speed-level parameters (28 levels total)
	kinematic_speed_params_t speeds[KINEMATIC_TOTAL_SPEED_LEVELS];
} train_kinematic_model_t;

// Model collection for all trains
typedef struct {
	train_kinematic_model_t models[KINEMATIC_MAX_TRAINS];
	bool model_initialized[KINEMATIC_MAX_TRAINS];
	u32 active_model_count;
	kinematic_time_t last_global_update;
} kinematic_model_collection_t;

#define MARKLIN_TRAIN_CMD_DELAY_MS 150
#define MARKLIN_TRAIN_CMD_DELAY_TICKS (MARKLIN_TRAIN_CMD_DELAY_MS / 10)

// Path retry constants for deadlock/blocking scenarios
#define TRAIN_PATH_RETRY_INITIAL_DELAY_MS 1000   // 1 second initial delay
#define TRAIN_PATH_RETRY_MAX_DELAY_MS 8000       // 8 second maximum delay
#define TRAIN_PATH_RETRY_MAX_ATTEMPTS 5          // Maximum retry attempts before giving up
#define TRAIN_PATH_RETRY_BACKOFF_MULTIPLIER 2    // Exponential backoff multiplier
#define MARKLIN_TRAIN_MAX_SPEED 14
#define MARKLIN_REVERSE_CMD 15
#define MARKLIN_HEADLIGHT_ON_CMD 16

// ############################################################################
// # State Machine Framework
// ############################################################################

// Primary train states - main state machine
typedef enum {
	TRAIN_STATE_IDLE = 0, // Train is stopped and not active
	TRAIN_STATE_MOVING = 1, // Train is in motion (includes kinematic stopping)
	TRAIN_STATE_STOPPING = 2, // Emergency stop only (normal stops use kinematic in MOVING)
	TRAIN_STATE_REVERSING = 3, // Train is executing reversal sequence
	TRAIN_STATE_ERROR = 4 // Train has encountered an error
} train_state_t;

// Movement sub-states
typedef enum {
	MOVEMENT_STATE_STATIONARY = 0, // Train is not moving
	MOVEMENT_STATE_ACCELERATING = 1, // Train is accelerating
	MOVEMENT_STATE_CRUISING = 2, // Train is at constant speed
	MOVEMENT_STATE_DECELERATING = 3 // Train is decelerating
} train_movement_state_t;

// Path management sub-states
typedef enum {
	PATH_STATE_NONE = 0, // No active path
	PATH_STATE_REQUESTING = 1, // Currently requesting a new path
	PATH_STATE_ACTIVE = 2, // Path is active and being followed
	PATH_STATE_REACHED = 3, // Destination reached
	PATH_STATE_CONTINUATION_NEEDED = 4, // Partial path needs continuation
	PATH_STATE_AT_REVERSAL = 5, // Train reached reversal point, needs to reverse
	PATH_STATE_REVERSING = 6 // Train is in the process of reversing
} train_path_state_t;

// Reversal sequence sub-states
typedef enum {
	REV_STATE_NONE = 0, // Not in reversal
	REV_STATE_STOPPING = 1, // Stopping before reversal
	REV_STATE_COMMAND = 2, // Executing reversal command
	REV_STATE_CLEARING = 3, // Moving past reversal point
	REV_STATE_RESUMING = 4 // Resuming normal operation
} train_reversal_state_t;

// State machine events
typedef enum {
	TRAIN_EVENT_NONE = 0,
	TRAIN_EVENT_START_MOVING = 1,
	TRAIN_EVENT_STOP_REQUESTED = 2,
	TRAIN_EVENT_EMERGENCY_STOP = 3,
	TRAIN_EVENT_SENSOR_TRIGGERED = 4,
	TRAIN_EVENT_DESTINATION_REACHED = 5,
	TRAIN_EVENT_PATH_END_REACHED = 6,
	TRAIN_EVENT_REVERSAL_NEEDED = 7,
	TRAIN_EVENT_REVERSAL_COMPLETE = 8,
	TRAIN_EVENT_ERROR_DETECTED = 9,
	TRAIN_EVENT_SPEED_CHANGED = 10,
	TRAIN_EVENT_PATH_CONTINUATION_NEEDED = 11
} train_event_t;

// State machine transition result
typedef enum {
	TRANSITION_HANDLED = 0, // Event was handled and state may have changed
	TRANSITION_IGNORED = 1, // Event was ignored (not valid for current state)
	TRANSITION_DEFERRED = 2 // Event should be processed later
} train_transition_result_t;

// Forward declaration for state machine context
struct train_task_data;

// State machine function pointer types
typedef train_transition_result_t (*train_state_handler_t)(struct train_task_data *data, train_event_t event);
typedef void (*train_state_entry_t)(struct train_task_data *data);
typedef void (*train_state_exit_t)(struct train_task_data *data);

// State machine state descriptor
typedef struct {
	train_state_t state;
	const char *name;
	train_state_handler_t handler;
	train_state_entry_t entry_action;
	train_state_exit_t exit_action;
} train_state_descriptor_t;

// Unified stop action enumeration (moved here for use in state machine)
typedef enum {
	TRAIN_CONTINUE = 0, // Continue normal operation
	TRAIN_STOP_DESTINATION, // Stop for destination arrival
	TRAIN_STOP_PATH_END, // Stop at end of activated path segment
	TRAIN_STOP_REVERSAL, // Stop at reversal point
	TRAIN_STOP_LOW_SPEED_TIMER, // Stop due to low speed mode timer expiration
	TRAIN_EMERGENCY_STOP // Emergency stop (safety critical)
} train_stop_action_t;

// State machine context
typedef struct {
	train_state_t current_state;
	train_state_t previous_state;
	train_movement_state_t movement_state;
	train_path_state_t path_state;
	train_reversal_state_t reversal_state;
	train_event_t pending_event;
	bool event_pending;
	u32 state_entry_time_tick;
	u32 transition_count;

	// Kinematic stopping tracking
	bool kinematic_stopping_active;
	u32 kinematic_stop_start_time_tick;
	u32 kinematic_stop_duration_ms;
	train_stop_action_t kinematic_stop_reason;

	// State transition tracking for sensor validation
	u64 last_moving_exit_time_tick; // Timestamp when train last exited MOVING state
} train_state_machine_t;

#define TRAIN_LENGTH_MM 200

// Train position representation (sensor + offset)
typedef struct {
	const track_node *sensor;
	kinematic_distance_t offset_mm; // Positive = past sensor, negative = before sensor
} train_position_t;

// Unified train motion state - single source of truth for all speed-related data
//
// SPEED SEMANTICS:
// - commanded_speed: Immediate speed command sent to train hardware (0-14)
// - requested_speed: Desired speed from user/autonomous control (0-14)
// - actual_speed: Physical speed accounting for acceleration/deceleration (0-14)
// - current_velocity: Kinematic velocity in mm/tick for physics calculations
// - current_acceleration: Current acceleration in mm/tick² for motion prediction
//
// RELATIONSHIPS:
// commanded_speed = requested_speed (with block-based safety constraints)
// actual_speed tracks commanded_speed with realistic acceleration curves
// current_velocity = kinematic_model_get_velocity(actual_speed, direction_info)
//
typedef struct {
	// Command layer speeds (what the system wants)
	u8 commanded_speed; // Speed currently sent to train hardware (immediate)
	u8 requested_speed; // Speed requested by user/autonomous systems

	// Speed transition state
	bool is_accelerating; // True if speed is changing
	bool commanded_speed_from_higher; // Direction of commanded speed change
	kinematic_time_t speed_change_time; // When current speed change started
	kinematic_time_t last_update_time; // When motion state was last updated

	// Location and navigation
	const track_node *expected_sensors[2];
	kinematic_distance_t expected_distances[2];
	kinematic_time_t expected_arrival_times[2]; // Expected arrival times for each sensor
	kinematic_time_t sensor_timeout_deadlines[2]; // Timeout deadlines for each sensor
	bool sensor_timeout_logged[2]; // Track which sensor timeouts have been logged
	u8 expected_sensor_count;
	train_direction_t direction; // Current travel direction

	// Continuous position tracking (single source of truth)
	train_position_t current_position; // Real-time position (sensor + offset)
	kinematic_time_t last_position_update; // When position was last updated

	// Centralized stopping calculation (computed once per loop)
	kinematic_distance_t current_stop_distance; // Current stopping distance in mm
	kinematic_time_t last_stop_distance_update; // When stopping distance was last calculated
} train_motion_state_t;

struct train_task_data {
	u8 train_id;
	const track_node *destination;
	char destination_name[16];
	kinematic_distance_t destination_offset_mm;
	kinematic_distance_t train_length_mm;
	// DEPRECATED: status field replaced by state_machine.current_state
	// Use train_get_external_status(data) for external API compatibility
	train_operating_mode_t operating_mode;

	// Unified motion state - single source of truth for all speed/position/location data
	train_motion_state_t motion;

	// Server connections
	int clock_server_tid;
	int controller_tid;
	int conductor_tid;
	int command_server_tid;

	// Misc control
	marklin_train_headlight_t headlight;

	// Timing
	u64 last_path_request_tick;
	u64 last_position_report_tick;
	u64 last_path_continuation_tick;

	// Sensor tracking
	marklin_msgqueue_subscription_t sensor_subscription;
	bool sensor_subscription_active;
	u64 last_sensor_trigger_tick;

	// Path management state machine (now in state_machine context)
	path_result_t current_path;
	bool has_active_path;
	bool path_ends_at_reversal;

	// Progressive path activation tracking
	marklin_path_activation_result_t last_activation_result;
	path_activation_stop_reason_t activation_stop_reason;
	const track_node *activation_end_point;
	bool needs_path_continuation;
	bool at_reversal_point;
	const track_node *reversal_node;
	const track_node *reversal_next_node;

	// Reservation management (all modes)
	u32 segments_needed_to_stop;
	u8 last_reservation_speed;

	// Block tracking for automatic release
	const track_node *reserved_block_nodes[MAX_TRACK_BLOCKS];
	u32 reserved_block_count;

	// Path re-activation tracking
	kinematic_distance_t last_activation_distance;

	// Kinematic model integration
	bool kinematic_model_enabled;

	train_kinematic_model_t *kinematic_model;

	bool sensor_blacklist_cache[5][16];

	// Random destination functionality
	bool random_destination_enabled;
	u64 last_random_destination_time;
	u64 destination_arrival_time; // When train arrived at destination (for pause timing)

	// Path retry tracking for deadlock/blocking scenarios
	u64 last_path_failure_time;
	u32 consecutive_path_failures;
	u64 next_retry_time;
	bool in_retry_backoff;

	// Low speed mode for short distance navigation
	bool low_speed_mode_enabled; // Flag to enable/disable low speed mode functionality
	bool low_speed_mode_active;
	u64 low_speed_start_time;
	u64 low_speed_expected_duration_ms;
	kinematic_distance_t low_speed_target_distance;

	// State machine context
	train_state_machine_t state_machine;
};

typedef struct train_task_data train_task_data_t;

void marklin_train_task(void);

// Train movement control
marklin_error_t train_set_speed(train_task_data_t *data, u8 speed);
marklin_error_t train_set_headlight(train_task_data_t *data, marklin_train_headlight_t headlight);
marklin_error_t train_toggle_headlight(train_task_data_t *data);
marklin_error_t train_reverse(train_task_data_t *data);
marklin_error_t train_stop(train_task_data_t *data);
marklin_error_t train_emergency_stop(train_task_data_t *data);
marklin_error_t train_reverse_and_continue(train_task_data_t *data);
marklin_error_t train_set_speed_and_headlight(train_task_data_t *data, u8 speed, marklin_train_headlight_t headlight);

// Centralized path management functions
marklin_error_t train_path_request_to_destination(train_task_data_t *data, const track_node *destination,
						  bool allow_reverse);
marklin_error_t train_path_activate_result(train_task_data_t *data, path_result_t *path_result);
void train_path_update_state_machine(train_task_data_t *data);
bool train_path_should_resume(train_task_data_t *data);

// Path retry management functions
void train_init_retry_state(train_task_data_t *data);
bool train_should_retry_path(train_task_data_t *data);
void train_record_path_failure(train_task_data_t *data);
void train_reset_retry_state(train_task_data_t *data);
u64 train_calculate_retry_delay(train_task_data_t *data);

// Progressive path activation functions
marklin_error_t train_handle_path_continuation(train_task_data_t *data);
marklin_error_t train_handle_reversal_sequence(train_task_data_t *data);
// Forward declaration for destination command
struct marklin_destination_cmd {
	const track_node *destination;
	char destination_name[16];
	kinematic_distance_t offset_mm;
};
marklin_error_t train_set_destination(train_task_data_t *data, const struct marklin_destination_cmd *dest_cmd);
marklin_error_t train_clear_destination(train_task_data_t *data);
marklin_error_t train_navigate_to_destination(train_task_data_t *data, const char *destination_name,
					      u8 requested_speed);

void train_switch_to_mode(train_task_data_t *data, train_operating_mode_t new_mode);

// Position utilities
marklin_error_t train_position_validate(const train_position_t *position);
kinematic_distance_t train_position_distance_between(const train_position_t *from, const train_position_t *to,
						     bool use_effective_distance);
bool train_position_is_at_destination(const train_position_t *current, const train_position_t *destination,
				      kinematic_distance_t tolerance_mm);

// Train length compensation
kinematic_distance_t train_calculate_stopping_offset(const train_task_data_t *data, kinematic_distance_t target_offset,
						     train_direction_t direction);

// Continuous position tracking
void train_update_current_position(train_task_data_t *data);

// Unified kinematic stopping system
void train_update_stop_distance(train_task_data_t *data);
train_stop_action_t train_check_unified_stop_conditions(train_task_data_t *data);
void train_execute_stop_action(train_task_data_t *data, train_stop_action_t action);
bool train_is_sensor_blacklisted(train_task_data_t *data, const track_node *sensor_node);

// Block management functions
void train_add_reserved_block(train_task_data_t *data, const track_node *block_node);
void train_remove_reserved_block(train_task_data_t *data, const track_node *block_node);
void train_clear_all_reserved_blocks(train_task_data_t *data);
marklin_error_t train_release_all_blocks(train_task_data_t *data, bool keep_current_block);
marklin_error_t train_release_specific_block(train_task_data_t *data, const track_node *block_node);
marklin_error_t train_release_exited_block(train_task_data_t *data, const track_node *sensor_node);

// State machine functions
void train_state_machine_init(train_task_data_t *data);
train_transition_result_t train_state_machine_process_event(train_task_data_t *data, train_event_t event);
void train_state_machine_update(train_task_data_t *data);
const char *train_state_name(train_state_t state);
const char *train_event_name(train_event_t event);

// State machine debugging and logging
void train_state_machine_debug_print_status(train_task_data_t *data);
const char *train_movement_state_name(train_movement_state_t state);
const char *train_path_state_name(train_path_state_t state);
const char *train_reversal_state_name(train_reversal_state_t state);
const char *train_transition_result_name(train_transition_result_t result);

// State machine to external status mapping
train_status_t train_get_external_status(const train_task_data_t *data);

// Kinematic stopping detection
bool train_check_kinematic_stop_complete(train_task_data_t *data);

// State handlers
train_transition_result_t train_state_idle_handler(train_task_data_t *data, train_event_t event);
train_transition_result_t train_state_moving_handler(train_task_data_t *data, train_event_t event);
train_transition_result_t train_state_stopping_handler(train_task_data_t *data, train_event_t event);
train_transition_result_t train_state_reversing_handler(train_task_data_t *data, train_event_t event);
train_transition_result_t train_state_error_handler(train_task_data_t *data, train_event_t event);

// State entry/exit actions
void train_state_idle_entry(train_task_data_t *data);
void train_state_moving_entry(train_task_data_t *data);
void train_state_stopping_entry(train_task_data_t *data);
void train_state_reversing_entry(train_task_data_t *data);
void train_state_error_entry(train_task_data_t *data);

void train_state_idle_exit(train_task_data_t *data);
void train_state_moving_exit(train_task_data_t *data);
void train_state_stopping_exit(train_task_data_t *data);
void train_state_reversing_exit(train_task_data_t *data);
void train_state_error_exit(train_task_data_t *data);

// Reversal sub-state handlers
void train_update_reversal_state_machine(train_task_data_t *data);

// Low speed mode functions
void train_init_low_speed_mode(train_task_data_t *data);
bool train_should_use_low_speed_mode(train_task_data_t *data, kinematic_distance_t distance);
void train_start_low_speed_mode(train_task_data_t *data, kinematic_distance_t distance);
void train_stop_low_speed_mode(train_task_data_t *data);
bool train_check_low_speed_timer(train_task_data_t *data);
void train_enable_low_speed_mode(train_task_data_t *data, bool enable);

// Online calibration system functions are declared in calibration.h

#endif /* MARKLIN_TRAIN_H */
