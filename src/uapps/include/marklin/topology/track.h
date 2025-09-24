#ifndef MARKLIN_CONTROLLER_TRACK_H
#define MARKLIN_CONTROLLER_TRACK_H

#include "marklin/common/track_node.h"
#include "marklin/topology/api.h"
#include "types.h"
#include "string.h"

static inline const track_node *marklin_find_node_by_name(const track_node *nodes, int size, const char *name)
{
	if (name == NULL) {
		return NULL;
	}

	for (int i = 0; i < size; i++) {
		if (nodes[i].name != NULL && strcmp(nodes[i].name, name) == 0) {
			return &nodes[i];
		}
	}

	return NULL;
}

static inline const track_node *marklin_get_next_node(const track_node *current, track_direction dir)
{
	if (current == NULL || dir > 1) {
		return NULL;
	}

	return current->edge[dir].dest;
}

static inline const track_node *marklin_get_reverse_node(const track_node *node)
{
	if (node == NULL) {
		return NULL;
	}

	return node->reverse;
}

static u8 marklin_parse_sensor_bank_from_name(const char *name)
{
	if (!name || name[0] == '\0')
		return 0;

	switch (name[0]) {
	case 'A':
		return 0;
	case 'B':
		return 1;
	case 'C':
		return 2;
	case 'D':
		return 3;
	case 'E':
		return 4;
	default:
		return 0xFF;
	}
}

static u8 marklin_parse_sensor_id_from_name(const char *name)
{
	if (!name || name[0] == '\0')
		return 0xFF;

	// Skip the bank letter and parse the number
	const char *num_str = name + 1;
	u8 sensor_id = 0;

	while (*num_str >= '0' && *num_str <= '9') {
		sensor_id = sensor_id * 10 + (*num_str - '0');
		num_str++;
	}

	return sensor_id == 0 ? 0xFF : sensor_id;
}

// Helper function to extract sensor bank and ID from sensor node
static inline void marklin_get_sensor_info_from_node(const track_node *sensor_node, u8 *bank, u8 *sensor_id)
{
	if (!sensor_node || sensor_node->type != NODE_SENSOR || !sensor_node->name) {
		*bank = 0xFF;
		*sensor_id = 0xFF;
		return;
	}

	*bank = marklin_parse_sensor_bank_from_name(sensor_node->name);
	*sensor_id = marklin_parse_sensor_id_from_name(sensor_node->name);
}

// Helper function to find sensor node by bank and sensor ID
static inline const track_node *marklin_find_sensor_node_by_bank_id(const track_node *nodes, int size, u8 bank,
								    u8 sensor_id)
{
	if (!nodes || size <= 0) {
		return NULL;
	}

	for (int i = 0; i < size; i++) {
		const track_node *node = &nodes[i];
		if (node->type == NODE_SENSOR && node->name) {
			// Parse bank letter (A=0, B=1, etc.)
			char bank_char = node->name[0];
			u8 node_bank = (bank_char >= 'A' && bank_char <= 'E') ? (bank_char - 'A') : 0xFF;

			// Parse sensor number
			u8 node_sensor_id = 0;
			const char *num_str = node->name + 1;
			while (*num_str >= '0' && *num_str <= '9') {
				node_sensor_id = node_sensor_id * 10 + (*num_str - '0');
				num_str++;
			}

			if (node_bank == bank && node_sensor_id == sensor_id) {
				return node;
			}
		}
	}

	return NULL;
}

static inline int marklin_get_node_edge_count(const track_node *node)
{
	if (!node) {
		return 0;
	}

	switch (node->type) {
	case NODE_SENSOR:
	case NODE_MERGE:
	case NODE_ENTER:
		return 1; // DIR_AHEAD
	case NODE_BRANCH:
		return 2; // DIR_STRAIGHT / DIR_CURVED
	case NODE_EXIT:
		return 0; // No outgoing edges
	default:
		return 0;
	}
}

// Check if a node can be a block boundary (sensors, enter nodes, and exit nodes)
static inline bool marklin_is_boundary_node(const track_node *node)
{
	return node && (node->type == NODE_SENSOR || node->type == NODE_ENTER || node->type == NODE_EXIT);
}

// Check if a node is a turnout (branch)
static inline bool marklin_is_turnout_node(const track_node *node)
{
	return node && node->type == NODE_BRANCH;
}

#endif /* MARKLIN_CONTROLLER_TRACK_H */
