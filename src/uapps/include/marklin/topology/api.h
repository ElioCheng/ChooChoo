#ifndef MARKLIN_TOPOLOGY_API_H
#define MARKLIN_TOPOLOGY_API_H

#include "marklin/common/track_node.h"
#include "marklin/error.h"

#define MARKLIN_TOPOLOGY_SERVER_NAME "marklin_topology_server"

typedef enum {
	MARKLIN_TOPOLOGY_REQ_GET_TRACK_NODES,
	MARKLIN_TOPOLOGY_REQ_INIT_TRACK,
	MARKLIN_TOPOLOGY_REQ_GET_SENSOR_BLACKLIST,
	MARKLIN_TOPOLOGY_REQ_ADD_BLACKLISTED_SENSOR,
	MARKLIN_TOPOLOGY_REQ_REMOVE_BLACKLISTED_SENSOR,
	MARKLIN_TOPOLOGY_REQ_IS_SENSOR_BLACKLISTED,
} marklin_topology_request_type_t;

typedef enum {
	MARKLIN_TRACK_TYPE_A,
	MARKLIN_TRACK_TYPE_B,
} marklin_track_type_t;

#define MAX_BLACKLISTED_SENSORS_PER_TRACK 60

typedef struct {
	u8 bank; // Sensor bank (0-4 for A-E)
	u8 sensor_id; // Sensor ID (1-16)
} blacklisted_sensor_t;

typedef struct {
	blacklisted_sensor_t sensors[MAX_BLACKLISTED_SENSORS_PER_TRACK];
	u8 count;
} sensor_blacklist_t;

typedef struct {
	marklin_topology_request_type_t type;
	union {
		marklin_track_type_t track_type;
		struct {
			u8 bank;
			u8 sensor_id;
		} sensor_blacklist_op;
	};
} marklin_topology_request_t;

typedef struct {
	marklin_error_t error;
	const track_node *track_nodes;
	int track_nodes_size;
	marklin_track_type_t track_type;
	union {
		sensor_blacklist_t blacklist;
		bool is_blacklisted;
	};
} marklin_topology_reply_t;

int Marklin_GetTrackNodes(const track_node **nodes, marklin_track_type_t *track_type);
marklin_error_t Marklin_InitTrack(marklin_track_type_t track_type);

// Sensor blacklist management
marklin_error_t Marklin_GetSensorBlacklist(sensor_blacklist_t *blacklist);
marklin_error_t Marklin_AddBlacklistedSensor(u8 bank, u8 sensor_id);
marklin_error_t Marklin_RemoveBlacklistedSensor(u8 bank, u8 sensor_id);
marklin_error_t Marklin_IsSensorBlacklisted(u8 bank, u8 sensor_id, bool *is_blacklisted);

#endif /* MARKLIN_TOPOLOGY_API_H */
