#ifndef MARKLIN_H
#define MARKLIN_H

#include "types.h"
#include "dlist.h"

#include "marklin/controller/api.h"
#include "marklin/error.h"
#include "marklin/topology/api.h"
#include "marklin/train2/api.h"

#define MARKLIN_CONTROLLER_PRIORITY 5
#define MARKLIN_MAX_SPAWNED_TRAINS 16

typedef struct {
	marklin_request_type_t type;
	union {
		struct {
			marklin_track_type_t track_type;
		} init;

		marklin_train_spawn_info_t spawn_train;

		marklin_train_spawn_by_sensor_info_t spawn_train_by_sensor;

		struct {
			u8 train_id;
			marklin_train_command_t command;
		} train_command;

		struct {
			track_direction direction;
		} set_all_switches;

		struct {
			marklin_track_type_t track_type;
		} system_reset;
	};
} marklin_request_t;

typedef struct {
	marklin_error_t error;
	union {
		track_direction switch_direction;

		struct {
			int train_task_tid;
		} spawn_train;

		marklin_train_spawn_info_t train_info;

		marklin_system_snapshot_t system_snapshot;
	};
} marklin_reply_t;

typedef struct {
	struct dlist_node list;
	const track_node *node;
} switch_list_entry_t;

typedef struct {
	struct dlist_node list;
	const track_node *node;
} sensor_list_entry_t;

// Spawned train task tracking
typedef struct {
	struct dlist_node list;
	u8 train_id;
	int task_tid;
	const track_node *current_location;
	u8 current_speed;
	train_direction_t direction;
	marklin_train_headlight_t headlight;
	const track_node *destination;
	char destination_name[16];
	train_operating_mode_t mode;
	kinematic_distance_t location_offset_mm;
	kinematic_distance_t destination_offset_mm;
	train_status_t status;
	const track_node *next_sensor_1;
	const track_node *next_sensor_2;
} spawned_train_entry_t;

// Switch state tracking
typedef struct {
	struct dlist_node list;
	u8 switch_id;
	track_direction direction;
	u64 last_changed_tick;
} controller_switch_entry_t;

typedef struct {
	const track_node *track;
	int track_size;
	marklin_track_type_t track_type;

	// Track spawned train tasks
	struct dlist_node spawned_trains;
	int spawned_train_count;

	// Track switch states
	struct dlist_node tracked_switches;
	int tracked_switch_count;
} marklin_system_t;

extern marklin_system_t marklin_system;

void marklin_init(marklin_track_type_t track_type);
void marklin_controller_task(void);

#endif /* MARKLIN_H */
