#ifndef MARKLIN_CONTROLLER_API_H
#define MARKLIN_CONTROLLER_API_H

#include "marklin/common/track_node.h"
#include "marklin/error.h"
#include "marklin/topology/api.h"
#include "marklin/train2/api.h"
#include "marklin/train2/train.h"
#include "types.h"

#define MARKLIN_CONTROLLER_SERVER_NAME "marklin_controller"
#define MARKLIN_MAX_TRAINS_IN_SNAPSHOT 16
#define MARKLIN_MAX_SWITCHES_IN_SNAPSHOT 32

typedef enum {
	MARKLIN_REQ_SPAWN_TRAIN,
	MARKLIN_REQ_SPAWN_TRAIN_BY_SENSOR,
	MARKLIN_REQ_TRAIN_COMMAND,
	MARKLIN_REQ_SWITCH_COMMAND,
	MARKLIN_REQ_GET_SELF_TRAIN_INFO,
	MARKLIN_REQ_GET_SYSTEM_SNAPSHOT,
	MARKLIN_REQ_STOP_ALL_TRAINS,
	MARKLIN_REQ_SET_ALL_SWITCHES,
	MARKLIN_REQ_START_DEMO,
	MARKLIN_REQ_SYSTEM_RESET,
} marklin_request_type_t;

typedef struct {
	u8 train_id;
	const track_node *init_location;
} marklin_train_spawn_info_t;

typedef struct {
	u8 train_id;
	char sensor_name[16]; // Sensor name like "A1", "B5", etc.
} marklin_train_spawn_by_sensor_info_t;

typedef struct {
	u8 train_id;
	u8 speed;
	train_direction_t direction;
	marklin_train_headlight_t headlight;
	const track_node *current_location;
	const track_node *destination;
	char destination_name[16];
	train_operating_mode_t mode;
	kinematic_distance_t location_offset_mm;
	kinematic_distance_t destination_offset_mm;
	train_status_t status;
	const track_node *next_sensor_1;
	const track_node *next_sensor_2;
} marklin_train_snapshot_t;

typedef struct {
	u8 switch_id;
	track_direction direction;
	u64 last_changed_tick;
} marklin_switch_snapshot_t;

typedef struct {
	u8 active_train_count;
	marklin_train_snapshot_t trains[MARKLIN_MAX_TRAINS_IN_SNAPSHOT];
	u8 active_switch_count;
	marklin_switch_snapshot_t switches[MARKLIN_MAX_SWITCHES_IN_SNAPSHOT];
} marklin_system_snapshot_t;

marklin_error_t Marklin_ControllerInitTrack(marklin_track_type_t track_type);
marklin_error_t Marklin_ControllerSpawnTrain(u8 train_id, const track_node *init_location, int *train_task_tid);
marklin_error_t Marklin_ControllerSpawnTrainBySensor(u8 train_id, const char *sensor_name, int *train_task_tid);
marklin_error_t Marklin_ControllerSwitchCommand(u8 switch_id, track_direction direction);
marklin_error_t Marklin_ControllerGetSelfTrainInfo(marklin_train_spawn_info_t *info);
marklin_error_t Marklin_ControllerTrainCommand(u8 train_id, const marklin_train_command_t *command);
marklin_error_t Marklin_ControllerGetSystemSnapshot(marklin_system_snapshot_t *snapshot);
marklin_error_t Marklin_ControllerStopAllTrains(void);
marklin_error_t Marklin_ControllerSetAllSwitches(track_direction direction);
marklin_error_t Marklin_ControllerStartDemo(void);
marklin_error_t Marklin_ControllerSystemReset(marklin_track_type_t track_type);

#endif /* MARKLIN_CONTROLLER_API_H */
