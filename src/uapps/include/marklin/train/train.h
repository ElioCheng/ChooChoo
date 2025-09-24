#ifndef MARKLIN_TRAIN_H
#define MARKLIN_TRAIN_H

#include "types.h"
#include "marklin/train/api.h"
#include "marklin/common/track_node.h"
#include "marklin/conductor/path.h"
#include "marklin/conductor/block.h"
#include "marklin/conductor/api.h"
#include "marklin/msgqueue/api.h"
#include "marklin/train/kinematics.h"
#include "marklin/error.h"

// Calibration types (moved from calibration.h to avoid circular dependency)
typedef enum {
	CALIBRATION_VELOCITY = 1, // Measure velocity at constant speed
	CALIBRATION_ACCELERATION = 2, // Measure acceleration between speeds
	CALIBRATION_DECELERATION = 3, // Measure deceleration between speeds
	CALIBRATION_STOPPING = 4, // Measure stopping distance and time
	CALIBRATION_SHORT_MOVE = 5 // Measure motion without reaching target speed
} kinematic_calibration_type_t;

typedef enum {
	CALIBRATION_STATE_IDLE = 0,
	CALIBRATION_STATE_PREPARING = 1, // Setting up experiment
	CALIBRATION_STATE_WAITING_START = 2, // Waiting for start sensor
	CALIBRATION_STATE_MEASURING = 3, // Experiment in progress
	CALIBRATION_STATE_STOPPING = 4, // Stopping train after measurement
	CALIBRATION_STATE_COMPLETE = 5, // Experiment completed
	CALIBRATION_STATE_ERROR = 6 // Experiment failed
} kinematic_calibration_state_t;

// Calibration support types (moved from model.h to avoid circular dependency)
typedef struct {
	kinematic_time_t timestamp; // When measurement was taken
	kinematic_distance_t distance; // Distance measured
	kinematic_time_t travel_time; // Time taken to travel distance
	kinematic_velocity_t velocity; // Calculated velocity
	u8 speed_level; // Speed setting during measurement
	bool from_higher_speed; // Speed transition direction
	const track_node *start_sensor; // Starting sensor
	const track_node *end_sensor; // Ending sensor
} kinematic_measurement_t;

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
#define MARKLIN_TRAIN_MAX_SPEED 14
#define MARKLIN_REVERSE_CMD 15
#define MARKLIN_HEADLIGHT_ON_CMD 16

// Path management state machine
typedef enum {
	PATH_STATE_NONE = 0, // No active path
	PATH_STATE_REQUESTING = 1, // Currently requesting a new path
	PATH_STATE_ACTIVE = 2, // Path is active and being followed (may be partial, use needs_path_continuation flag)
	PATH_STATE_REACHED = 3, // Destination reached
	PATH_STATE_AT_REVERSAL = 4, // Train reached reversal point, needs to reverse
	PATH_STATE_REVERSING = 5 // Train is in the process of reversing
} train_path_state_t;


typedef enum {
	OFFLINE_VELOCITY_LOOP_STATE_NAVIGATING_TO_B5 = 0,
	OFFLINE_VELOCITY_LOOP_STATE_AT_B5_PREPARING = 1,
	OFFLINE_VELOCITY_LOOP_STATE_SETTING_PATH = 2,
	OFFLINE_VELOCITY_LOOP_STATE_DRY_RUN = 3,
	OFFLINE_VELOCITY_LOOP_STATE_WAITING_FOR_B5 = 4,
	OFFLINE_VELOCITY_LOOP_STATE_READY_TO_MEASURE = 5,
	OFFLINE_VELOCITY_LOOP_STATE_MEASURING = 6,
	OFFLINE_VELOCITY_LOOP_STATE_SPEED_COMPLETE = 7,
	OFFLINE_VELOCITY_LOOP_STATE_TRANSITION_COMPLETE = 8,
	OFFLINE_VELOCITY_LOOP_STATE_DONE = 9,
} offline_velocity_loop_state_t;

// Offline experiment types
typedef enum {
	OFFLINE_EXPERIMENT_VELOCITY_LOOP = 1, // Loop-based velocity measurement
	OFFLINE_EXPERIMENT_ACCELERATION = 2, // Acceleration measurement between speed levels
	OFFLINE_EXPERIMENT_STOP_DISTANCE = 3, // Stop distance measurement using binary search
} offline_experiment_type_t;

typedef struct {
	offline_velocity_loop_state_t state;
	i32 offline_b5_cycle_count;
	kinematic_distance_t offline_loop_distance; // Calculated B5->C11->B5 distance
	u8 offline_current_speed_index;
	u8 offline_speed_levels[KINEMATIC_TOTAL_SPEED_LEVELS]; // Kinematic levels 0-27
	u8 offline_speed_count;
	kinematic_time_t offline_measurement_start_time;
	kinematic_time_t offline_speed_start_time;
} offline_velocity_loop_context_t;

// Acceleration experiment states
typedef enum {
	OFFLINE_ACCELERATION_STATE_NAVIGATING_TO_C13 = 0,
	OFFLINE_ACCELERATION_STATE_AT_C13_PREPARING = 1,
	OFFLINE_ACCELERATION_STATE_SETTING_PATH = 2,
	OFFLINE_ACCELERATION_STATE_READY_TO_MEASURE = 3,
	OFFLINE_ACCELERATION_STATE_MEASURING = 4,
	OFFLINE_ACCELERATION_STATE_SPEED_PAIR_COMPLETE = 5,
	OFFLINE_ACCELERATION_STATE_DONE = 6,
} offline_acceleration_state_t;

#define MAX_ACCELERATION_SPEED_PAIRS 16
#define MAX_ACCELERATION_MEASUREMENTS 10

typedef struct {
	offline_acceleration_state_t state;
	u8 from_speed_levels[MAX_ACCELERATION_SPEED_PAIRS]; // Starting speed levels to test
	u8 to_speed_levels[MAX_ACCELERATION_SPEED_PAIRS]; // Target speed levels to test
	u8 speed_pair_count; // Number of speed pairs to test
	u8 current_pair_index; // Current speed pair being tested
	kinematic_distance_t c13_to_e7_distance; // Distance from C13 to E7 sensors
	kinematic_time_t measurement_start_time; // When measurement started
	kinematic_time_t speed_change_time; // When speed change occurred at C13
	u32 measurements_per_pair; // Number of measurements per speed pair
	kinematic_accel_t measured_accelerations[MAX_ACCELERATION_MEASUREMENTS]; // Raw measurements
	u32 measurement_count; // Number of measurements taken for current pair
} offline_acceleration_context_t;

// Stop distance experiment states
typedef enum {
	OFFLINE_STOP_DISTANCE_STATE_NAVIGATING_TO_A3 = 0,
	OFFLINE_STOP_DISTANCE_STATE_AT_A3_PREPARING = 1,
	OFFLINE_STOP_DISTANCE_STATE_SETTING_PATH = 2,
	OFFLINE_STOP_DISTANCE_STATE_ACCELERATING_TO_SPEED = 3,
	OFFLINE_STOP_DISTANCE_STATE_WAITING_FOR_A3 = 4,
	OFFLINE_STOP_DISTANCE_STATE_BINARY_SEARCH = 5,
	OFFLINE_STOP_DISTANCE_STATE_SPEED_COMPLETE = 6,
	OFFLINE_STOP_DISTANCE_STATE_DONE = 7,
} offline_stop_distance_state_t;

#define MAX_STOP_DISTANCE_SPEEDS 27
#define MAX_STOP_DISTANCE_MEASUREMENTS 2

typedef struct {
	offline_stop_distance_state_t state;
	u8 speed_levels[MAX_STOP_DISTANCE_SPEEDS]; // Speed levels 1-14 to test
	u8 speed_count; // Number of speeds to test
	u8 current_speed_index; // Current speed being tested
	kinematic_distance_t a3_to_e7_distance; // Distance from A3 to E7 sensors

	// Binary search parameters
	kinematic_time_t delay_min; // Minimum delay (too early)
	kinematic_time_t delay_max; // Maximum delay (too late)
	kinematic_time_t delay_current; // Current delay being tested
	kinematic_time_t delay_optimal; // Optimal delay found

	// Measurement tracking
	kinematic_time_t a3_trigger_time; // When A3 was hit
	kinematic_time_t stop_command_time; // When stop command issued
	bool e7_triggered; // Did E7 trigger during this test?
	bool e7_final_state; // E7 final state (on/off)
	kinematic_time_t search_timeout; // Timeout for current search iteration
	kinematic_time_t max_search_time; // Maximum time for binary search

	// Results storage
	kinematic_time_t measured_stop_delays[MAX_STOP_DISTANCE_SPEEDS]; // Optimal delays per speed
	kinematic_distance_t measured_stop_distances[MAX_STOP_DISTANCE_SPEEDS]; // Calculated stop distances
	u32 measurements_per_speed; // Number of measurements per speed level
	u32 current_measurement; // Current measurement for this speed
	u32 binary_search_iterations; // Current search iteration count
	u32 max_binary_search_iterations; // Maximum iterations allowed
} offline_stop_distance_context_t;

// Offline experiment configuration
typedef struct {
	offline_experiment_type_t type; // Type of experiment
	union {
		offline_velocity_loop_context_t velocity_loop;
		offline_acceleration_context_t acceleration;
		offline_stop_distance_context_t stop_distance;
	};
} offline_experiment_context_t;

// ############################################################################
// # Online Calibration System Data Structures
// ############################################################################

// Online calibration state and context
typedef struct {
	bool enabled; // Whether online calibration is active

	// Current measurement tracking
	const track_node *last_sensor; // Previous sensor for distance calculation
	kinematic_time_t last_sensor_time; // Time when last sensor was triggered
	u8 measuring_speed; // Speed level being measured
	bool from_higher_speed; // Direction of speed transition
	bool measuring_acceleration; // True if currently measuring acceleration
	u8 accel_from_speed; // Starting speed for acceleration measurement
	u8 accel_to_speed; // Target speed for acceleration measurement
	kinematic_time_t speed_change_start_time; // When speed change began
} online_calibration_context_t;

#define TRAIN_LENGTH_MM 200

// Unified stop action enumeration
typedef enum {
	TRAIN_CONTINUE = 0, // Continue normal operation
	TRAIN_STOP_DESTINATION, // Stop for destination arrival
	TRAIN_STOP_PATH_END, // Stop at end of activated path segment
	TRAIN_STOP_REVERSAL, // Stop at reversal point
	TRAIN_EMERGENCY_STOP // Emergency stop (safety critical)
} train_stop_action_t;

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

	// Physical layer speeds (what the train is actually doing)
	u8 actual_speed; // Real-time physical speed (gradual acceleration/deceleration)
	kinematic_velocity_t current_velocity; // Current velocity in kinematic units
	kinematic_accel_t current_acceleration; // Current acceleration

	// Speed transition state
	bool is_accelerating; // True if speed is changing
	bool commanded_speed_from_higher; // Direction of commanded speed change
	bool actual_speed_from_higher; // Direction of actual speed change
	kinematic_time_t speed_change_time; // When current speed change started
	kinematic_time_t last_update_time; // When motion state was last updated

	// Location and navigation
	const track_node *expected_sensors[2];
	kinematic_distance_t expected_distances[2];
	kinematic_time_t expected_arrival_times[2]; // Expected arrival times for each sensor
	kinematic_time_t sensor_timeout_deadlines[2]; // Timeout deadlines for each sensor
	u8 expected_sensor_count;
	train_direction_t direction; // Current travel direction

	// Continuous position tracking (single source of truth)
	train_position_t current_position; // Real-time position (sensor + offset)
	kinematic_time_t last_position_update; // When position was last updated

	// Centralized stopping calculation (computed once per loop)
	kinematic_distance_t current_stop_distance; // Current stopping distance in mm
	kinematic_time_t last_stop_distance_update; // When stopping distance was last calculated
} train_motion_state_t;

typedef struct {
	u8 train_id;
	const track_node *destination;
	char destination_name[16];
	kinematic_distance_t destination_offset_mm;
	kinematic_distance_t train_length_mm;
	train_status_t status;
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

	// Path management state machine
	train_path_state_t path_state;
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

	// Offline experiment integration
	bool offline_experiment_active;
	offline_experiment_context_t offline_experiment_context;

	// Model reference (pointer to entry in global collection)
	train_kinematic_model_t *kinematic_model;

	// Online calibration integration
	online_calibration_context_t online_calibration;

	bool sensor_blacklist_cache[5][16];

	// Random destination functionality
	bool random_destination_enabled;
	u64 last_random_destination_time;
	u64 destination_arrival_time; // When train arrived at destination (for pause timing)

} train_task_data_t;

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
// Online calibration system functions are declared in calibration.h

#endif /* MARKLIN_TRAIN_H */
