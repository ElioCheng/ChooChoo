#ifndef MARKLIN_TRAIN_API_H
#define MARKLIN_TRAIN_API_H

#include "marklin/common/track_node.h"
#include "marklin/error.h"
#include "marklin/train/kinematics.h"

// Forward declaration to avoid circular dependency
typedef enum {
	TRAIN_STATUS_IDLE = 0,
	TRAIN_STATUS_REQUESTING_PATH = 1,
	TRAIN_STATUS_MOVING = 2,
	TRAIN_STATUS_STOPPING = 3
} train_status_t;

typedef enum {
	TRAIN_MODE_MANUAL = 0, // Manual control only
	TRAIN_MODE_WAYPOINT = 1 // Path-based navigation with destination
} train_operating_mode_t;

typedef enum {
	MARKLIN_TRAIN_HEADLIGHT_OFF = 0,
	MARKLIN_TRAIN_HEADLIGHT_ON = 1,
	MARKLIN_TRAIN_HEADLIGHT_AUTO = 2
} marklin_train_headlight_t;

typedef enum { TRAIN_DIRECTION_FORWARD = 0, TRAIN_DIRECTION_REVERSE = 1 } train_direction_t;

typedef struct {
	u8 train_id;
	const track_node *current_location;
	train_direction_t direction;
	marklin_train_headlight_t headlight;
	u8 current_speed;
	const track_node *destination;
	char destination_name[16];
	train_operating_mode_t mode;
	kinematic_distance_t location_offset_mm;
	kinematic_distance_t destination_offset_mm;
	train_status_t status;
	const track_node *next_sensor_1;
	const track_node *next_sensor_2;
} marklin_train_position_data_t;

typedef enum {
	MARKLIN_TRAIN_CMD_SET_MODE = 1,

	// Manual mode commands (direct control)
	MARKLIN_TRAIN_CMD_MANUAL_SET_EFFECTIVE_SPEED = 2,
	MARKLIN_TRAIN_CMD_MANUAL_REVERSE = 3,
	MARKLIN_TRAIN_CMD_MANUAL_TOGGLE_HEADLIGHT = 4,
	MARKLIN_TRAIN_CMD_MANUAL_STOP = 5,

	// Waypoint mode commands (signal-controlled)
	MARKLIN_TRAIN_CMD_SET_REQUESTED_SPEED = 6,
	MARKLIN_TRAIN_CMD_SET_DESTINATION = 7,

	// Emergency commands (all modes)
	MARKLIN_TRAIN_CMD_EMERGENCY_STOP = 8,

	// High-level navigation command
	MARKLIN_TRAIN_CMD_NAVIGATE_TO_DESTINATION = 9,

	// Offline experiment commands
	MARKLIN_TRAIN_CMD_START_OFFLINE_EXPERIMENT = 10,
	MARKLIN_TRAIN_CMD_STOP_OFFLINE_EXPERIMENT = 11,
	MARKLIN_TRAIN_CMD_GET_KINEMATIC_MODEL = 12,

	// Random destination mode command
	MARKLIN_TRAIN_CMD_SET_RANDOM_DESTINATION_MODE = 13,

	// Debug command
	MARKLIN_TRAIN_CMD_DEBUG_INFO = 14,
} marklin_train_command_type_t;

// Train command structure
typedef struct {
	marklin_train_command_type_t command_type;
	union {
		// Mode management
		struct {
			train_operating_mode_t mode;
		} set_mode;

		// Manual mode commands
		struct {
			u8 effective_speed;
			marklin_train_headlight_t headlight;
		} manual_set_effective_speed;

		// Waypoint mode commands
		struct {
			u8 requested_speed;
		} set_requested_speed;

		struct {
			const track_node *destination;
			char destination_name[16];
			kinematic_distance_t offset_mm;
		} set_destination;

		// High-level navigation command
		struct {
			char destination_name[16];
			bool allow_reverse;
			u8 requested_speed;
		} navigate_to_destination;

		// Offline experiment commands
		struct {
			u8 experiment_type; // 1=velocity, 2=acceleration, 3=stop_distance
			u8 speed_levels[28]; // Speed levels to test (kinematic levels 0-27)
			u8 speed_count; // Number of speed levels
			// For acceleration experiment, speeds are paired: [from1, to1, from2, to2, ...]
		} start_offline_experiment;

		// Random destination mode command
		struct {
			bool enabled;
		} set_random_destination_mode;
	};
} marklin_train_command_t;

marklin_error_t Marklin_TrainSetSpeed(u8 train_tid, u8 speed, marklin_train_headlight_t headlight);
marklin_error_t Marklin_TrainReverse(u8 train_tid, u8 speed);

// High-level navigation function
marklin_error_t Marklin_TrainNavigateToDestination(u8 train_id, const char *destination_name, bool allow_reverse,
						   u8 requested_speed);

#endif /* MARKLIN_TRAIN_API_H */
