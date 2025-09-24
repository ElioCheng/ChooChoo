#include "compiler.h"
#include "marklin/command/api.h"
#include "marklin/common/track_node.h"
#include "marklin/controller/api.h"
#include "marklin/error.h"
#include "marklin/controller/marklin.h"
#include "marklin/topology/api.h"
#include "marklin/topology/topology.h"
#include "marklin/topology/track.h"
#include "marklin/command/command.h"
#include "marklin/train/api.h"
#include "stdbool.h"

// Define the global train constants
const int all_possible_trains[7] = { 13, 14, 15, 16, 17, 18, 55 };
#include "marklin/train2/train.h"
#include "marklin/msgqueue/msgqueue.h"
#include "marklin/conductor/conductor.h"
#include "marklin/tui/tui.h"
#include "marklin/common/constants.h"
#include "io.h"
#include "klog.h"
#include "clock_server.h"
#include "name.h"
#include "types.h"
#include "clock.h"
#include "syscall.h"

// ############################################################################
// # Controller Public API
// ############################################################################

int marklin_controller_tid = -1;

static inline int get_marklin_controller_tid(void)
{
	if (marklin_controller_tid < 0) {
		marklin_controller_tid = WhoIs(MARKLIN_CONTROLLER_SERVER_NAME);
	}
	return marklin_controller_tid;
}

marklin_error_t Marklin_ControllerSpawnTrain(u8 train_id, const track_node *init_location, int *train_task_tid)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_SPAWN_TRAIN;
	request.spawn_train.train_id = train_id;
	request.spawn_train.init_location = init_location;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	*train_task_tid = reply.spawn_train.train_task_tid;

	return reply.error;
}

marklin_error_t Marklin_ControllerSpawnTrainBySensor(u8 train_id, const char *sensor_name, int *train_task_tid)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_SPAWN_TRAIN_BY_SENSOR;
	request.spawn_train_by_sensor.train_id = train_id;

	// Copy sensor name safely
	strncpy(request.spawn_train_by_sensor.sensor_name, sensor_name, 15);
	request.spawn_train_by_sensor.sensor_name[15] = '\0';

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	if (train_task_tid) {
		*train_task_tid = reply.spawn_train.train_task_tid;
	}

	return reply.error;
}

marklin_error_t Marklin_ControllerGetSelfTrainInfo(marklin_train_spawn_info_t *info)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_GET_SELF_TRAIN_INFO;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	*info = reply.train_info;

	return MARKLIN_ERROR_OK;
}

marklin_error_t Marklin_ControllerTrainCommand(u8 train_id, const marklin_train_command_t *command)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_TRAIN_COMMAND;
	request.train_command.train_id = train_id;
	request.train_command.command = *command;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_ControllerGetSystemSnapshot(marklin_system_snapshot_t *snapshot)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_GET_SYSTEM_SNAPSHOT;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	if (reply.error == MARKLIN_ERROR_OK) {
		*snapshot = reply.system_snapshot;
	}

	return reply.error;
}

marklin_error_t Marklin_ControllerStopAllTrains(void)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_STOP_ALL_TRAINS;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_ControllerSetAllSwitches(track_direction direction)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_SET_ALL_SWITCHES;
	request.set_all_switches.direction = direction;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_ControllerStartDemo(void)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_START_DEMO;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

marklin_error_t Marklin_ControllerSystemReset(marklin_track_type_t track_type)
{
	int server_tid = get_marklin_controller_tid();
	if (server_tid <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_request_t request;
	marklin_reply_t reply;

	request.type = MARKLIN_REQ_SYSTEM_RESET;
	request.system_reset.track_type = track_type;

	int result = Send(server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return reply.error;
}

// ############################################################################
// # Controller Server
// ############################################################################

marklin_system_t marklin_system;

static spawned_train_entry_t spawned_train_entries[MARKLIN_MAX_SPAWNED_TRAINS];
static controller_switch_entry_t controller_switch_entries[MARKLIN_MAX_SWITCHES_IN_SNAPSHOT];

int clock_server_tid = -1;
static int conductor_task_tid = -1;
static marklin_msgqueue_subscription_t switch_subscription;
static int switch_subscription_active = 0;
static marklin_msgqueue_subscription_t position_subscription;
static int position_subscription_active = 0;

#define LOG_MODULE "MARKLIN_CONTROLLER"
#define LOG_LEVEL LOG_LEVEL_NONE
#include "log.h"

// Forward declarations
static controller_switch_entry_t *__marklin_add_switch(u8 switch_id, track_direction direction, u64 tick);
static marklin_error_t __marklin_set_all_switches(track_direction direction);
static marklin_error_t __marklin_system_reset(marklin_track_type_t track_type);

static void __marklin_populate_switch_list(void)
{
	u64 current_tick = Time(clock_server_tid);
	for (int i = 0; i < marklin_system.track_size; i++) {
		const track_node *node = &marklin_system.track[i];
		if (node->type == NODE_BRANCH) {
			__marklin_add_switch(node->num, DIR_STRAIGHT, current_tick);
		}
	}
}

static void __marklin_stop_all_possible_trains(void)
{
	for (size_t i = 0; i < ALL_POSSIBLE_TRAINS_COUNT; i++) {
		u8 cmd = 0 + 16;
		u8 param = all_possible_trains[i];
		Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_WITH_PARAM, cmd, param,
						    MARKLIN_TRAIN_CMD_DELAY_TICKS, MARKLIN_CMD_PRIORITY_MEDIUM, param);
	}
}

void add_blocklisted_sensors(marklin_track_type_t track_type)
{
#define BROKEN_SENSOR(track, sensor_name)                                      \
	if (track_type == track) {                                             \
		u8 bank = marklin_parse_sensor_bank_from_name(sensor_name);    \
		u8 sensor_id = marklin_parse_sensor_id_from_name(sensor_name); \
		if (bank == 0xff || sensor_id == 0xff) {                       \
			Panic("Invalid sensor name: %s", sensor_name);         \
		}                                                              \
		Marklin_AddBlacklistedSensor(bank, sensor_id);                 \
	}

#include "marklin/common/blacklisted_sensors_list.h"
#undef BROKEN_SENSOR
}

void __marklin_init(marklin_track_type_t track_type)
{
	log_info("Initializing track type %d", track_type);

	// Create topology server for track initialization
	Create(MARKLIN_TOPOLOGY_SERVER_TASK_PRIORITY, marklin_topology_server_task);

	Delay(clock_server_tid, 100); // Wait for topology server to be ready

	if (Marklin_InitTrack(track_type) != MARKLIN_ERROR_OK) {
		Panic("Failed to initialize track");
		Exit();
	}

	int track_size = Marklin_GetTrackNodes(&marklin_system.track, &marklin_system.track_type);
	if (track_size <= 0) {
		Panic("Failed to get track nodes");
		Exit();
	}
	marklin_system.track_size = track_size;

	add_blocklisted_sensors(track_type);

	__marklin_populate_switch_list();

	// Create conductor task
	conductor_task_tid = Create(MARKLIN_CONDUCTOR_TASK_PRIORITY, marklin_conductor_task);

	Delay(clock_server_tid, 100); // Wait for conductor to be ready

	__marklin_stop_all_possible_trains();

	__marklin_set_all_switches(DIR_STRAIGHT);

	marklin_error_t switch_subscription_result =
		Marklin_MsgQueue_Subscribe(MARKLIN_MSGQUEUE_EVENT_TYPE_SWITCH_STATE, &switch_subscription);
	if (switch_subscription_result == MARKLIN_ERROR_OK) {
		switch_subscription_active = 1;
		log_info("Controller: Subscribed to switch updates");
	} else {
		switch_subscription_active = 0;
		log_error("Controller: Failed to subscribe to switch updates: %d", switch_subscription_result);
	}

	marklin_error_t position_subscription_result =
		Marklin_MsgQueue_Subscribe(MARKLIN_MSGQUEUE_EVENT_TYPE_TRAIN_POSITION, &position_subscription);
	if (position_subscription_result == MARKLIN_ERROR_OK) {
		position_subscription_active = 1;
		log_info("Controller: Subscribed to train position updates");
	} else {
		position_subscription_active = 0;
		log_error("Controller: Failed to subscribe to train position updates: %d",
			  position_subscription_result);
	}

	log_info("Marklin controller initialized");
}

static spawned_train_entry_t *__marklin_find_spawned_train(u8 train_id)
{
	struct dlist_node *node;
	dlist_for_each(node, &marklin_system.spawned_trains)
	{
		spawned_train_entry_t *entry = dlist_entry(node, spawned_train_entry_t, list);
		if (entry->train_id == train_id) {
			return entry;
		}
	}
	return NULL;
}

static spawned_train_entry_t *__marklin_find_spawned_train_by_tid(int task_tid)
{
	struct dlist_node *node;
	dlist_for_each(node, &marklin_system.spawned_trains)
	{
		spawned_train_entry_t *entry = dlist_entry(node, spawned_train_entry_t, list);
		if (entry->task_tid == task_tid) {
			return entry;
		}
	}
	return NULL;
}

static controller_switch_entry_t *__marklin_find_switch(u8 switch_id)
{
	struct dlist_node *node;
	dlist_for_each(node, &marklin_system.tracked_switches)
	{
		controller_switch_entry_t *entry = dlist_entry(node, controller_switch_entry_t, list);
		if (entry->switch_id == switch_id) {
			return entry;
		}
	}
	return NULL;
}

static controller_switch_entry_t *__marklin_add_switch(u8 switch_id, track_direction direction, u64 tick)
{
	if (marklin_system.tracked_switch_count >= MARKLIN_MAX_SWITCHES_IN_SNAPSHOT) {
		log_error("Cannot track more switches: limit reached");
		return NULL;
	}

	controller_switch_entry_t *entry = &controller_switch_entries[marklin_system.tracked_switch_count];
	entry->switch_id = switch_id;
	entry->direction = direction;
	entry->last_changed_tick = tick;

	dlist_insert_tail(&marklin_system.tracked_switches, &entry->list);
	marklin_system.tracked_switch_count++;

	return entry;
}

static void __marklin_process_switch_update(const marklin_msgqueue_message_t *message)
{
	if (!message)
		return;

	marklin_switch_state_t *switch_update = MARKLIN_MSGQUEUE_CAST_TO(marklin_switch_state_t, message);
	if (!switch_update) {
		log_error("Controller: Invalid switch update message format");
		return;
	}

	controller_switch_entry_t *entry = __marklin_find_switch(switch_update->switch_id);
	if (!entry) {
		entry = __marklin_add_switch(switch_update->switch_id, switch_update->direction,
					     switch_update->last_changed_tick);
	} else {
		entry->direction = switch_update->direction;
		entry->last_changed_tick = switch_update->last_changed_tick;
		log_debug("Updated switch %d to direction %d", switch_update->switch_id, switch_update->direction);
	}
}

static void __marklin_process_position_update(const marklin_msgqueue_message_t *message)
{
	if (!message)
		return;

	marklin_train_position_data_t *position_update =
		MARKLIN_MSGQUEUE_CAST_TO(marklin_train_position_data_t, message);
	if (!position_update) {
		log_error("Controller: Invalid position update message format");
		return;
	}

	spawned_train_entry_t *entry = __marklin_find_spawned_train(position_update->train_id);
	if (!entry) {
		log_warn("Controller: Received position update for unknown train %d", position_update->train_id);
		return;
	}

	// Update train state from position report
	entry->current_location = position_update->current_location;
	entry->direction = position_update->direction;
	entry->headlight = position_update->headlight;
	entry->current_speed = position_update->current_speed;
	entry->destination = position_update->destination;
	strncpy(entry->destination_name, position_update->destination_name, 15);
	entry->destination_name[15] = '\0';
	entry->mode = position_update->mode;
	entry->location_offset_mm = position_update->location_offset_mm;
	entry->destination_offset_mm = position_update->destination_offset_mm;
	entry->status = position_update->status;
	entry->next_sensor_1 = position_update->next_sensor_1;
	entry->next_sensor_2 = position_update->next_sensor_2;

	log_debug("Controller: Updated train %d position to %p, speed %d, direction %d, headlight %d, destination %s",
		  position_update->train_id, position_update->current_location, position_update->current_speed,
		  position_update->direction, position_update->headlight,
		  position_update->destination_name[0] ? position_update->destination_name : "None");
}

marklin_error_t __marklin_spawn_train(u8 train_id, const track_node *init_location, int *train_task_tid)
{
	if (marklin_system.spawned_train_count >= MARKLIN_MAX_SPAWNED_TRAINS) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	if (__marklin_find_spawned_train(train_id) != NULL) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int tid = Create(4, marklin_train_task);
	if (tid < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	spawned_train_entry_t *entry = &spawned_train_entries[marklin_system.spawned_train_count];
	entry->train_id = train_id;
	entry->task_tid = tid;
	entry->current_location = init_location;
	entry->current_speed = 0;
	entry->direction = TRAIN_DIRECTION_FORWARD;
	entry->headlight = MARKLIN_TRAIN_HEADLIGHT_ON;
	entry->destination = NULL;
	entry->destination_name[0] = '\0';
	entry->mode = TRAIN_MODE_MANUAL;
	entry->location_offset_mm = 0;
	entry->destination_offset_mm = 0;
	entry->status = TRAIN_STATUS_IDLE;

	dlist_insert_tail(&marklin_system.spawned_trains, &entry->list);
	marklin_system.spawned_train_count++;

	if (train_task_tid) {
		*train_task_tid = tid;
	}

	log_info("Controller: successfully spawned train %d as task %d", train_id, tid);
	return MARKLIN_ERROR_OK;
}

static const track_node *__marklin_find_sensor_by_name(const char *sensor_name)
{
	if (!sensor_name || strlen(sensor_name) == 0) {
		return NULL;
	}

	for (int i = 0; i < marklin_system.track_size; i++) {
		if (marklin_system.track[i].type == NODE_SENSOR && marklin_system.track[i].name &&
		    strcmp(marklin_system.track[i].name, sensor_name) == 0) {
			return &marklin_system.track[i];
		}
	}

	return NULL;
}

marklin_error_t __marklin_spawn_train_by_sensor(u8 train_id, const char *sensor_name, int *train_task_tid)
{
	const track_node *sensor_node = __marklin_find_sensor_by_name(sensor_name);
	if (sensor_node == NULL) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	return __marklin_spawn_train(train_id, sensor_node, train_task_tid);
}

marklin_error_t __marklin_get_self_train_info(marklin_train_spawn_info_t *info, int sender_tid)
{
	spawned_train_entry_t *entry = __marklin_find_spawned_train_by_tid(sender_tid);
	if (entry == NULL) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	info->train_id = entry->train_id;
	info->init_location = entry->current_location;

	return MARKLIN_ERROR_OK;
}

marklin_error_t __marklin_get_system_snapshot(marklin_system_snapshot_t *snapshot)
{
	snapshot->active_train_count = 0;
	snapshot->active_switch_count = 0;

	// Populate train data
	struct dlist_node *node;
	memset(snapshot, 0, sizeof(marklin_system_snapshot_t));
	dlist_for_each(node, &marklin_system.spawned_trains)
	{
		if (snapshot->active_train_count >= MARKLIN_MAX_TRAINS_IN_SNAPSHOT) {
			break;
		}

		spawned_train_entry_t *entry = dlist_entry(node, spawned_train_entry_t, list);
		marklin_train_snapshot_t *train_snapshot = &snapshot->trains[snapshot->active_train_count];

		train_snapshot->train_id = entry->train_id;
		train_snapshot->current_location = entry->current_location;

		train_snapshot->speed = entry->current_speed;
		train_snapshot->direction = entry->direction;
		train_snapshot->headlight = entry->headlight;
		train_snapshot->destination = entry->destination;
		strncpy(train_snapshot->destination_name, entry->destination_name, 15);
		train_snapshot->destination_name[15] = '\0';
		train_snapshot->mode = entry->mode;
		train_snapshot->location_offset_mm = entry->location_offset_mm;
		train_snapshot->destination_offset_mm = entry->destination_offset_mm;
		train_snapshot->status = entry->status;
		train_snapshot->next_sensor_1 = entry->next_sensor_1;
		train_snapshot->next_sensor_2 = entry->next_sensor_2;

		snapshot->active_train_count++;
	}

	dlist_for_each(node, &marklin_system.tracked_switches)
	{
		if (snapshot->active_switch_count >= MARKLIN_MAX_SWITCHES_IN_SNAPSHOT) {
			break;
		}

		controller_switch_entry_t *entry = dlist_entry(node, controller_switch_entry_t, list);
		marklin_switch_snapshot_t *switch_snapshot = &snapshot->switches[snapshot->active_switch_count];

		switch_snapshot->switch_id = entry->switch_id;
		switch_snapshot->direction = entry->direction;
		switch_snapshot->last_changed_tick = entry->last_changed_tick;

		snapshot->active_switch_count++;
	}

	return MARKLIN_ERROR_OK;
}

marklin_error_t __marklin_send_train_command(u8 train_id, const marklin_train_command_t *command)
{
	spawned_train_entry_t *entry = __marklin_find_spawned_train(train_id);
	if (entry == NULL) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	marklin_error_t train_result;
	int result = Send(entry->task_tid, (const char *)command, sizeof(marklin_train_command_t),
			  (char *)&train_result, sizeof(train_result));
	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return train_result;
}

marklin_error_t __marklin_stop_all_trains(void)
{
	struct dlist_node *node;
	marklin_train_command_t stop_command;
	stop_command.command_type = MARKLIN_TRAIN_CMD_EMERGENCY_STOP;

	dlist_for_each(node, &marklin_system.spawned_trains)
	{
		spawned_train_entry_t *entry = dlist_entry(node, spawned_train_entry_t, list);

		marklin_error_t train_result;
		int result = Send(entry->task_tid, (const char *)&stop_command, sizeof(marklin_train_command_t),
				  (char *)&train_result, sizeof(train_result));
		if (result < 0) {
			log_error("Failed to send stop command to train %d", entry->train_id);
		}
	}

	return MARKLIN_ERROR_OK;
}

marklin_error_t __marklin_set_all_switches(track_direction direction)
{
	struct dlist_node *node;
	marklin_error_t last_error = MARKLIN_ERROR_OK;
	u64 current_tick = Time(clock_server_tid);

	dlist_for_each(node, &marklin_system.tracked_switches)
	{
		controller_switch_entry_t *entry = dlist_entry(node, controller_switch_entry_t, list);
		marklin_error_t result =
			Marklin_SetSwitch(entry->switch_id, direction, node->next == NULL ? 1 : 0, true);
		if (result != MARKLIN_ERROR_OK) {
			log_error("Failed to set switch %d to direction %d", entry->switch_id, direction);
			last_error = result;
		} else {
			entry->direction = direction;
			entry->last_changed_tick = current_tick;
		}
	}

	return last_error;
}

marklin_error_t __marklin_start_demo(void)
{
	log_info("Demo functionality not yet implemented");
	return MARKLIN_ERROR_UNKNOWN;
}

marklin_error_t __marklin_system_reset(marklin_track_type_t track_type)
{
	log_info("Controller: System reset requested for track type %d", track_type);

	// Step 1: Kill all spawned train tasks
	struct dlist_node *node, *tmp;
	dlist_for_each_safe(node, tmp, &marklin_system.spawned_trains)
	{
		spawned_train_entry_t *entry = dlist_entry(node, spawned_train_entry_t, list);
		log_info("Controller: Killing train task %d (train_id %d)", entry->task_tid, entry->train_id);
		Kill(entry->task_tid, 1);
		dlist_del(node);
	}
	marklin_system.spawned_train_count = 0;

	// Step 2: Kill conductor task and all its children
	if (conductor_task_tid > 0) {
		log_info("Controller: Killing conductor task %d and its children", conductor_task_tid);
		Kill(conductor_task_tid, 1);
		conductor_task_tid = -1;
	}

	// Step 3: Reset all system state
	dlist_init(&marklin_system.spawned_trains);
	dlist_init(&marklin_system.tracked_switches);
	marklin_system.tracked_switch_count = 0;
	memset(controller_switch_entries, 0, sizeof(controller_switch_entries));
	memset(spawned_train_entries, 0, sizeof(spawned_train_entries));

	// Step 4: Unsubscribe from message queues
	if (switch_subscription_active) {
		Marklin_MsgQueue_Unsubscribe(&switch_subscription);
		switch_subscription_active = 0;
	}
	if (position_subscription_active) {
		Marklin_MsgQueue_Unsubscribe(&position_subscription);
		position_subscription_active = 0;
	}

	// Step 5: Re-initialize the system
	log_info("Controller: Re-initializing system");
	__marklin_init(track_type);

	log_info("Controller: System reset completed");
	return MARKLIN_ERROR_OK;
}

static void __process_request(int sender_tid, marklin_request_t *request)
{
	marklin_reply_t reply;
	reply.error = MARKLIN_ERROR_OK;

	switch (request->type) {
	case MARKLIN_REQ_SPAWN_TRAIN:
		reply.error = __marklin_spawn_train(request->spawn_train.train_id, request->spawn_train.init_location,
						    &reply.spawn_train.train_task_tid);
		break;

	case MARKLIN_REQ_SPAWN_TRAIN_BY_SENSOR:
		reply.error = __marklin_spawn_train_by_sensor(request->spawn_train_by_sensor.train_id,
							      request->spawn_train_by_sensor.sensor_name,
							      &reply.spawn_train.train_task_tid);
		break;

	case MARKLIN_REQ_TRAIN_COMMAND:
		reply.error =
			__marklin_send_train_command(request->train_command.train_id, &request->train_command.command);
		break;

	case MARKLIN_REQ_GET_SELF_TRAIN_INFO:
		reply.error = __marklin_get_self_train_info(&reply.train_info, sender_tid);
		break;

	case MARKLIN_REQ_GET_SYSTEM_SNAPSHOT:
		reply.error = __marklin_get_system_snapshot(&reply.system_snapshot);
		break;

	case MARKLIN_REQ_STOP_ALL_TRAINS:
		reply.error = __marklin_stop_all_trains();
		break;

	case MARKLIN_REQ_SET_ALL_SWITCHES:
		reply.error = __marklin_set_all_switches(request->set_all_switches.direction);
		break;

	case MARKLIN_REQ_START_DEMO:
		reply.error = __marklin_start_demo();
		break;

	case MARKLIN_REQ_SYSTEM_RESET:
		reply.error = __marklin_system_reset(request->system_reset.track_type);
		break;

	default:
		reply.error = MARKLIN_ERROR_INVALID_ARGUMENT;
		break;
	}

	Reply(sender_tid, (const char *)&reply, sizeof(reply));
}

void __marklin_process_msgqueue_message()
{
	if (switch_subscription_active || position_subscription_active) {
		marklin_msgqueue_message_t message;
		marklin_error_t msg_result = Marklin_MsgQueue_ReceiveNonBlock(&message);

		while (msg_result == MARKLIN_ERROR_OK) {
			if (message.event_type == MARKLIN_MSGQUEUE_EVENT_TYPE_SWITCH_STATE) {
				__marklin_process_switch_update(&message);
			} else if (message.event_type == MARKLIN_MSGQUEUE_EVENT_TYPE_TRAIN_POSITION) {
				__marklin_process_position_update(&message);
			}

			msg_result = Marklin_MsgQueue_ReceiveNonBlock(&message);
		}
	}
}

void __noreturn marklin_demo_task(void)
{
#define TRAIN_ID 14
	int tid;
	klog_info("Demo task started [tid: %d]", MyTid());
	Marklin_ControllerSpawnTrainBySensor(TRAIN_ID, "A1", &tid);

	clock_server_tid = WhoIs(CLOCK_SERVER_NAME);

	Delay(clock_server_tid, 1000);

	marklin_train_command_t command;
	command.command_type = MARKLIN_TRAIN_CMD_NAVIGATE_TO_DESTINATION;
	strncpy(command.navigate_to_destination.destination_name, "E7", 3);
	command.navigate_to_destination.destination_name[3] = '\0';
	command.navigate_to_destination.allow_reverse = false;
	command.navigate_to_destination.requested_speed = 10;
	Marklin_ControllerTrainCommand(TRAIN_ID, &command);

	// command.command_type = MARKLIN_TRAIN_CMD_MANUAL_REVERSE;
	// Marklin_ControllerTrainCommand(TRAIN_ID, &command);

	Delay(clock_server_tid, 2000);

	// for (;;) {
	command.command_type = MARKLIN_TRAIN_CMD_MANUAL_REVERSE;
	Marklin_ControllerTrainCommand(TRAIN_ID, &command);

	Delay(clock_server_tid, 500);

	command.command_type = MARKLIN_TRAIN_CMD_NAVIGATE_TO_DESTINATION;
	strncpy(command.navigate_to_destination.destination_name, "C12", 15);
	command.navigate_to_destination.destination_name[15] = '\0';
	command.navigate_to_destination.allow_reverse = false;
	command.navigate_to_destination.requested_speed = 10;
	Marklin_ControllerTrainCommand(TRAIN_ID, &command);

	Delay(clock_server_tid, 3000);

	command.command_type = MARKLIN_TRAIN_CMD_MANUAL_REVERSE;
	Marklin_ControllerTrainCommand(TRAIN_ID, &command);

	Delay(clock_server_tid, 500);

	command.command_type = MARKLIN_TRAIN_CMD_NAVIGATE_TO_DESTINATION;
	strncpy(command.navigate_to_destination.destination_name, "E7", 15);
	command.navigate_to_destination.destination_name[15] = '\0';
	command.navigate_to_destination.allow_reverse = false;
	command.navigate_to_destination.requested_speed = 10;
	Marklin_ControllerTrainCommand(TRAIN_ID, &command);

	Delay(clock_server_tid, 3000);
	// }

	Exit();
	UNREACHABLE();
}
void marklin_controller_task(void)
{
	marklin_request_t request;
	int sender_tid;

	klog_info("Controller server task started [tid: %d]", MyTid());

	RegisterAs(MARKLIN_CONTROLLER_SERVER_NAME);

	// Initialize basic system state and create core server tasks
	clock_server_tid = WhoIs(CLOCK_SERVER_NAME);
	conductor_task_tid = -1;
	switch_subscription_active = 0;
	position_subscription_active = 0;

	dlist_init(&marklin_system.spawned_trains);
	marklin_system.spawned_train_count = 0;
	dlist_init(&marklin_system.tracked_switches);
	marklin_system.tracked_switch_count = 0;
	memset(controller_switch_entries, 0, sizeof(controller_switch_entries));
	memset(spawned_train_entries, 0, sizeof(spawned_train_entries));

	// Create core server tasks that survive reset
	Create(MARKLIN_MSGQUEUE_SERVER_TASK_PRIORITY, marklin_msgqueue_server_task);
	Create(MARKLIN_CMD_SERVER_TASK_PRIORITY, marklin_cmd_server_task);

	Create(MARKLIN_TUI_SERVER_TASK_PRIORITY, marklin_tui_server_task);
	__marklin_system_reset(MARKLIN_TRACK_TYPE_A);

	Delay(clock_server_tid, 100);

	// Create(10, marklin_demo_task);

	for (;;) {
		int result = Receive(&sender_tid, (char *)&request, sizeof(request));
		__marklin_process_msgqueue_message();

		if (result < 0) {
			continue;
		}

		__process_request(sender_tid, &request);
	}
}
