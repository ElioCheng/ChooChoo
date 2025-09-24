#include "marklin/topology/topology.h"
#include "compiler.h"
#include "klog.h"
#include "marklin/error.h"
#include "marklin/topology/api.h"
#include "marklin/train/kinematics.h"
#include "marklin/topology/track.h"
#include "name.h"
#include "syscall.h"
#include "string.h"

int topology_server_tid = -1;

// #########################################################
// # Topology Server Public API
// #########################################################

marklin_error_t Marklin_InitTrack(marklin_track_type_t track_type)
{
	if (topology_server_tid < 0) {
		topology_server_tid = WhoIs(MARKLIN_TOPOLOGY_SERVER_NAME);
		if (topology_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_topology_request_t request;
	marklin_topology_reply_t reply;

	request.type = MARKLIN_TOPOLOGY_REQ_INIT_TRACK;
	request.track_type = track_type;

	int result = Send(topology_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	return reply.error;
}

marklin_error_t Marklin_GetSensorBlacklist(sensor_blacklist_t *blacklist)
{
	if (!blacklist) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (topology_server_tid < 0) {
		topology_server_tid = WhoIs(MARKLIN_TOPOLOGY_SERVER_NAME);
		if (topology_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_topology_request_t request;
	marklin_topology_reply_t reply;

	request.type = MARKLIN_TOPOLOGY_REQ_GET_SENSOR_BLACKLIST;

	int result = Send(topology_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	if (reply.error == MARKLIN_ERROR_OK) {
		*blacklist = reply.blacklist;
	}

	return reply.error;
}

marklin_error_t Marklin_AddBlacklistedSensor(u8 bank, u8 sensor_id)
{
	if (topology_server_tid < 0) {
		topology_server_tid = WhoIs(MARKLIN_TOPOLOGY_SERVER_NAME);
		if (topology_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_topology_request_t request;
	marklin_topology_reply_t reply;

	request.type = MARKLIN_TOPOLOGY_REQ_ADD_BLACKLISTED_SENSOR;
	request.sensor_blacklist_op.bank = bank;
	request.sensor_blacklist_op.sensor_id = sensor_id;

	int result = Send(topology_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	return reply.error;
}

marklin_error_t Marklin_RemoveBlacklistedSensor(u8 bank, u8 sensor_id)
{
	if (topology_server_tid < 0) {
		topology_server_tid = WhoIs(MARKLIN_TOPOLOGY_SERVER_NAME);
		if (topology_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_topology_request_t request;
	marklin_topology_reply_t reply;

	request.type = MARKLIN_TOPOLOGY_REQ_REMOVE_BLACKLISTED_SENSOR;
	request.sensor_blacklist_op.bank = bank;
	request.sensor_blacklist_op.sensor_id = sensor_id;

	int result = Send(topology_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	return reply.error;
}

marklin_error_t Marklin_IsSensorBlacklisted(u8 bank, u8 sensor_id, bool *is_blacklisted)
{
	if (!is_blacklisted) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (topology_server_tid < 0) {
		topology_server_tid = WhoIs(MARKLIN_TOPOLOGY_SERVER_NAME);
		if (topology_server_tid < 0) {
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}

	marklin_topology_request_t request;
	marklin_topology_reply_t reply;

	request.type = MARKLIN_TOPOLOGY_REQ_IS_SENSOR_BLACKLISTED;
	request.sensor_blacklist_op.bank = bank;
	request.sensor_blacklist_op.sensor_id = sensor_id;

	int result = Send(topology_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	if (reply.error == MARKLIN_ERROR_OK) {
		*is_blacklisted = reply.is_blacklisted;
	}

	return reply.error;
}

int Marklin_GetTrackNodes(const track_node **nodes, marklin_track_type_t *track_type)
{
	if (topology_server_tid <= 0) {
		topology_server_tid = WhoIs(MARKLIN_TOPOLOGY_SERVER_NAME);
		if (topology_server_tid <= 0) {
			return -1;
		}
	}

	marklin_topology_request_t request;
	marklin_topology_reply_t reply;

	request.type = MARKLIN_TOPOLOGY_REQ_GET_TRACK_NODES;

	int result = Send(topology_server_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return -1;
	}

	if (reply.error != MARKLIN_ERROR_OK) {
		return -1;
	}

	*nodes = reply.track_nodes;
	*track_type = reply.track_type;
	return reply.track_nodes_size;
}

// #########################################################
// # Topology Server Task
// #########################################################

extern void init_tracka(const track_node *track);
extern void init_trackb(const track_node *track);
#define TRACK_A_SIZE 144
#define TRACK_B_SIZE 140
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define TRACK_MAX_SIZE MAX(TRACK_A_SIZE, TRACK_B_SIZE)
static marklin_track_type_t track_type_g;

// Apply resistance coefficients to track edges based on configuration
static void apply_track_resistance_coefficients(track_node *track_nodes, int track_size, marklin_track_type_t track_type)
{
	// First, set all edges to default resistance
	for (int i = 0; i < track_size; i++) {
		track_node *node = &track_nodes[i];
		for (int edge_idx = 0; edge_idx < 2; edge_idx++) {
			// Cast away const to modify resistance coefficient during initialization
			track_edge *edge = (track_edge *)&node->edge[edge_idx];
			edge->resistance_coefficient = RESISTANCE_DEFAULT;
		}
	}
	
	// Apply custom resistance configurations from edge_resistance_list.h
#define TRACK_RESISTANCE(track, from_node, to_node, coefficient) \
	if (track_type == track) { \
		const track_node *from = marklin_find_node_by_name(track_nodes, track_size, from_node); \
		const track_node *to = marklin_find_node_by_name(track_nodes, track_size, to_node); \
		if (from && to) { \
			for (int edge_idx = 0; edge_idx < 2; edge_idx++) { \
				if (from->edge[edge_idx].dest == to) { \
					track_edge *edge = (track_edge *)&from->edge[edge_idx]; \
					edge->resistance_coefficient = coefficient; \
					break; \
				} \
			} \
		} \
	}

#include "marklin/common/edge_resistance_list.h"
#undef TRACK_RESISTANCE
}

int init_track(marklin_track_type_t track_type, const track_node *track_nodes)
{
	int track_size = 0;
	if (track_type == MARKLIN_TRACK_TYPE_A) {
		init_tracka(track_nodes);
		track_type_g = MARKLIN_TRACK_TYPE_A;
		track_size = TRACK_A_SIZE;
	} else if (track_type == MARKLIN_TRACK_TYPE_B) {
		init_trackb(track_nodes);
		track_type_g = MARKLIN_TRACK_TYPE_B;
		track_size = TRACK_B_SIZE;
	} else {
		return -1;
	}
	
	// Apply resistance coefficients after track initialization
	apply_track_resistance_coefficients((track_node *)track_nodes, track_size, track_type);
	
	return track_size;
}

void __noreturn marklin_topology_server_task(void)
{
	int sender_tid;
	marklin_topology_request_t request;
	marklin_topology_reply_t reply;

	RegisterAs(MARKLIN_TOPOLOGY_SERVER_NAME);

	topology_server_tid = MyTid();

	topology_server_state_t topology_server_state;
	memset(&topology_server_state, 0, sizeof(topology_server_state));

	topology_server_state.sensor_blacklist.count = 0;

	track_node track_nodes[TRACK_MAX_SIZE];
	int track_nodes_size = 0;

	for (;;) {
		int result = Receive(&sender_tid, (char *)&request, sizeof(request));

		if (result < 0) {
			continue;
		}

		reply.error = MARKLIN_ERROR_OK;

		switch (request.type) {
		case MARKLIN_TOPOLOGY_REQ_GET_TRACK_NODES:
			reply.track_nodes = track_nodes;
			reply.track_nodes_size = track_nodes_size;
			reply.track_type = track_type_g;
			break;

		case MARKLIN_TOPOLOGY_REQ_INIT_TRACK:
			track_nodes_size = init_track(request.track_type, track_nodes);
			if (track_nodes_size < 0) {
				reply.error = MARKLIN_ERROR_INVALID_ARGUMENT;
			}
			break;

		case MARKLIN_TOPOLOGY_REQ_GET_SENSOR_BLACKLIST: {
			reply.blacklist = topology_server_state.sensor_blacklist;
			break;
		}

		case MARKLIN_TOPOLOGY_REQ_ADD_BLACKLISTED_SENSOR: {
			sensor_blacklist_t *blacklist = &topology_server_state.sensor_blacklist;

			bool already_exists = false;
			for (u8 i = 0; i < blacklist->count; i++) {
				if (blacklist->sensors[i].bank == request.sensor_blacklist_op.bank &&
				    blacklist->sensors[i].sensor_id == request.sensor_blacklist_op.sensor_id) {
					already_exists = true;
					break;
				}
			}

			if (!already_exists && blacklist->count < MAX_BLACKLISTED_SENSORS_PER_TRACK) {
				blacklist->sensors[blacklist->count].bank = request.sensor_blacklist_op.bank;
				blacklist->sensors[blacklist->count].sensor_id = request.sensor_blacklist_op.sensor_id;
				blacklist->count++;
			} else if (blacklist->count >= MAX_BLACKLISTED_SENSORS_PER_TRACK) {
				reply.error = MARKLIN_ERROR_INVALID_ARGUMENT;
			}
			break;
		}

		case MARKLIN_TOPOLOGY_REQ_REMOVE_BLACKLISTED_SENSOR: {
			sensor_blacklist_t *blacklist = &topology_server_state.sensor_blacklist;

			bool found = false;
			for (u8 i = 0; i < blacklist->count; i++) {
				if (blacklist->sensors[i].bank == request.sensor_blacklist_op.bank &&
				    blacklist->sensors[i].sensor_id == request.sensor_blacklist_op.sensor_id) {
					for (u8 j = i; j < blacklist->count - 1; j++) {
						blacklist->sensors[j] = blacklist->sensors[j + 1];
					}
					blacklist->count--;
					found = true;
					break;
				}
			}

			if (!found) {
				reply.error = MARKLIN_ERROR_NOT_FOUND;
			}
			break;
		}

		case MARKLIN_TOPOLOGY_REQ_IS_SENSOR_BLACKLISTED: {
			sensor_blacklist_t *blacklist = &topology_server_state.sensor_blacklist;

			reply.is_blacklisted = false;
			for (u8 i = 0; i < blacklist->count; i++) {
				if (blacklist->sensors[i].bank == request.sensor_blacklist_op.bank &&
				    blacklist->sensors[i].sensor_id == request.sensor_blacklist_op.sensor_id) {
					reply.is_blacklisted = true;
					break;
				}
			}
			break;
		}

		default:
			reply.error = MARKLIN_ERROR_INVALID_ARGUMENT;
			break;
		}

		Reply(sender_tid, (const char *)&reply, sizeof(reply));
	}

	UNREACHABLE();
}
