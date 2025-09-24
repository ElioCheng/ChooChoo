#ifndef MARKLIN_CONTROLLER_TRACK_NODE_H
#define MARKLIN_CONTROLLER_TRACK_NODE_H

#include "types.h"

#define TRACK_MAX 144 // Maximum number of track nodes
#define MARKLIN_EXIT_NODE_MAX_COUNT 16

// Node type enums
typedef enum {
	NODE_NONE = 0,
	NODE_SENSOR = 1,
	NODE_BRANCH = 2,
	NODE_MERGE = 3,
	NODE_ENTER = 4,
	NODE_EXIT = 5
} node_type;

typedef enum { DIR_AHEAD = 0, DIR_STRAIGHT = 0, DIR_CURVED = 1 } track_direction;

// Forward declaration
struct track_node;

// Edge structure
typedef struct track_edge {
	const struct track_node *src;
	const struct track_node *dest;
	int dist; // Distance in millimeters
	u32 resistance_coefficient; // Resistance coefficient (fixed-point, scale 1000, default 1000 = 1.0)
	const struct track_edge *reverse;
} track_edge;

typedef struct track_node {
	const char *name;
	node_type type;
	int num;
	const struct track_node *reverse;
	track_edge edge[2];
} track_node;

#endif // MARKLIN_CONTROLLER_TRACK_NODE_H
