#ifndef MARKLIN_TOPOLOGY_H
#define MARKLIN_TOPOLOGY_H

#include "marklin/topology/api.h"

#define MARKLIN_TOPOLOGY_SERVER_TASK_PRIORITY 5

typedef struct {
	int topology_server_tid;
	marklin_track_type_t track_type;
	sensor_blacklist_t sensor_blacklist;
} topology_server_state_t;

void marklin_topology_server_task(void);

#endif /* MARKLIN_TOPOLOGY_H */
