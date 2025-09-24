#include "marklin/conductor/path.h"
#include "marklin/conductor/conductor.h"
#include "marklin/conductor/block.h"
#include "marklin/topology/api.h"
#include "marklin/topology/track.h"
#include "marklin/conductor/reversal_blacklist.h"
#include "priority_queue.h"
#include "string.h"
#include "syscall.h"
#include "io.h"
#include "printf.h"
#include <limits.h>

#define LOG_MODULE "path"
#define LOG_LEVEL LOG_LEVEL_WARN
#include "log.h"

extern conductor_task_data_t *g_conductor_data;

// Priority queue for Dijkstra's algorithm
#define MAX_PATH_QUEUE_SIZE 512

// Node state for pathfinding
typedef struct path_state {
	const track_node *node; // The actual node (original or reverse)
	int distance; // Distance from start
	struct path_state *prev; // Previous state in optimal path
	const track_edge *edge_used; // Edge used to reach this state
	bool visited;
} path_state_t;

typedef struct {
	path_state_t *state;
	int priority; // Lower is better (distance)
} pq_entry_t;

// Structure to hold traced path through a block
#define MAX_TRACED_PATH_NODES 20
typedef struct {
	const track_node *nodes[MAX_TRACED_PATH_NODES];
	track_direction switch_dirs[MAX_TRACED_PATH_NODES];
	int count;
	const track_node *exit_sensor;
} traced_path_t;

static int pq_compare_path(const pq_entry_t *a, const pq_entry_t *b)
{
	return (a->priority < b->priority) ? -1 : ((a->priority > b->priority) ? 1 : 0);
}

// Helper function to check if a block is in the excluded blocks list
static bool is_block_excluded(const track_block_t *block, const track_block_t **excluded_blocks, u32 excluded_count)
{
	if (!block || !excluded_blocks || excluded_count == 0) {
		return false;
	}

	for (u32 i = 0; i < excluded_count; i++) {
		if (excluded_blocks[i] == block) {
			return true;
		}
	}
	return false;
}

// Helper function to check if setting prev would create a cycle
static bool would_create_cycle(path_state_t *node, path_state_t *new_prev)
{
	if (!new_prev) {
		return false;
	}

	path_state_t *current = new_prev;
	int depth = 0;
	while (current) {
		if (current == node) {
			return true; // Found a cycle
		}
		current = current->prev;
		depth++;
		if (depth > TRACK_MAX * 2) {
			// Too deep, likely already has a cycle
			return true;
		}
	}
	return false;
}

// Format function for debug printing priority queue entries
static char pq_entry_format_buffer[256];
static __maybe_unused const char *pq_format_entry(const pq_entry_t *entry)
{
	memset(pq_entry_format_buffer, 0, sizeof(pq_entry_format_buffer));
	if (!entry || !entry->state) {
		return "(null)";
	}
	snprintf(pq_entry_format_buffer, sizeof(pq_entry_format_buffer), "%s dist=%d (nodeptr=%p, stateptr=%p)",
		 entry->state->node->name, entry->priority, entry->state->node, entry->state);
	return pq_entry_format_buffer;
}

static path_state_t *find_state(path_state_t *states, int state_count, const track_node *node)
{
	for (int i = 0; i < state_count; i++) {
		if (states[i].node == node) {
			return &states[i];
		}
	}
	return NULL;
}

static int init_path_states(path_state_t *states, const track_node *nodes, int node_count)
{
	int state_count = 0;

	for (int i = 0; i < node_count; i++) {
		// Add the forward node
		states[state_count].node = &nodes[i];
		states[state_count].distance = INT_MAX;
		states[state_count].prev = NULL;
		states[state_count].edge_used = NULL;
		states[state_count].visited = false;
		state_count++;

		// Add the reverse node if it exists
		if (nodes[i].reverse) {
			states[state_count].node = nodes[i].reverse;
			states[state_count].distance = INT_MAX;
			states[state_count].prev = NULL;
			states[state_count].edge_used = NULL;
			states[state_count].visited = false;
			state_count++;
		}
	}

	return state_count;
}

int path_get_edge_cost(const track_edge *edge, u8 train_id, bool is_reversal)
{
	UNUSED(train_id);

	if (!edge) {
		return is_reversal ? PATH_COST_REVERSAL : INT_MAX;
	}

	int cost = edge->dist;

	// Add penalty for reversals
	if (is_reversal) {
		cost += PATH_COST_REVERSAL;
	}

	return cost;
}

static void build_path_result(path_state_t *end_state, path_result_t *result, traced_path_t *prefix_path)
{
	if (!result->pool) {
		log_error("Path build failed: no pool assigned to result");
		result->total_distance = INT_MAX;
		return;
	}

	dlist_init(&result->nodes);
	result->total_distance = end_state->distance;
	result->num_reversals = 0;

	// Calculate the length of the main path (from pathfinding)
	int main_path_length = 0;
	path_state_t *current = end_state;
	while (current) {
		main_path_length++;
		if (main_path_length > TRACK_MAX * 2) {
			log_error("Path: Path too long (length=%d), likely has a cycle, aborting", main_path_length);
			result->total_distance = INT_MAX;
			return;
		}
		current = current->prev;
	}

	// Calculate prefix path length (excluding the exit sensor which is already in main path)
	int prefix_length = 0;
	if (prefix_path && prefix_path->count > 0) {
		prefix_length = prefix_path->count - 1; // Don't double-count the exit sensor
	}

	int total_path_length = prefix_length + main_path_length;

	if (total_path_length > MAX_PATH_NODES_PER_PATH) {
		log_error("Path too long: %d nodes (prefix: %d, main: %d), max is %d", total_path_length, prefix_length,
			  main_path_length, MAX_PATH_NODES_PER_PATH);
		result->total_distance = INT_MAX;
		return;
	}

	// Allocate path nodes from the pool and build the path in reverse order
	path_node_t *allocated_nodes[MAX_PATH_NODES_PER_PATH];
	int allocated_count = 0;

	// First, allocate all nodes we need
	for (int i = 0; i < total_path_length; i++) {
		path_node_t *pnode = path_node_alloc(result->pool);
		memset(pnode, 0, sizeof(path_node_t));
		if (!pnode) {
			log_error("Path build failed: could not allocate node %d/%d", i + 1, total_path_length);
			// Free any nodes we've allocated so far
			for (int j = 0; j < allocated_count; j++) {
				path_node_free(result->pool, allocated_nodes[j]);
			}
			result->total_distance = INT_MAX;
			return;
		}
		allocated_nodes[allocated_count++] = pnode;
	}

	// Now populate the main path nodes in reverse order (end to start)
	current = end_state;
	int idx = total_path_length - 1;

	while (current && idx >= prefix_length) {
		path_node_t *pnode = allocated_nodes[idx];
		pnode->node = current->node;

		if (current->prev) {
			// A reversal occurs when we transition from a node to its reverse counterpart
			// Check if current->prev->node and current->node are reverse pairs
			const track_node *prev_node = current->prev->node;
			const track_node *curr_node = current->node;

			if ((prev_node->reverse == curr_node) || (curr_node->reverse == prev_node)) {
				log_debug("Path: Reversal detected at node %s", pnode->node->name);
				if (idx > 0) {
					allocated_nodes[idx - 1]->reverse_here = true;
				}
				result->num_reversals++;
			}
		}

		if (current->prev && current->prev->node->type == NODE_BRANCH) {
			const track_node *branch_node = current->prev->node;
			if (branch_node && idx - 1 >= 0) {
				const track_node *target_node = current->node;

				if (branch_node->edge[DIR_STRAIGHT].dest == target_node) {
					allocated_nodes[idx - 1]->switch_dir = DIR_STRAIGHT;
				} else if (branch_node->edge[DIR_CURVED].dest == target_node) {
					allocated_nodes[idx - 1]->switch_dir = DIR_CURVED;
				}
			}
		}

		dlist_insert_tail(&result->nodes, &pnode->list);

		current = current->prev;
		idx--;
	}

	if (prefix_path && prefix_length > 0) {
		// to exclude the first node to avoid duplication
		for (int i = prefix_length - 1; i >= 1; i--) {
			path_node_t *pnode = allocated_nodes[idx];
			pnode->node = prefix_path->nodes[i];

			// Set switch direction for branch nodes
			if (pnode->node->type == NODE_BRANCH) {
				pnode->switch_dir = prefix_path->switch_dirs[i];
			}

			dlist_insert_tail(&result->nodes, &pnode->list);
			idx--;
		}
	}
}

/**
 * Trace through a block from the entry sensor to determine which exit sensor
 * the train would reach based on current switch positions.
 * Records the path taken through the block.
 * Returns true if successful, false if tracing fails.
 */
static bool trace_path_through_block(const track_block_t *block, const track_node *entry_sensor,
				     traced_path_t *traced_path)
{
	if (!block || !entry_sensor || !traced_path) {
		return false;
	}

	// Initialize traced path
	traced_path->count = 0;
	traced_path->exit_sensor = NULL;
	memset(traced_path->nodes, 0, sizeof(traced_path->nodes));
	memset(traced_path->switch_dirs, 0, sizeof(traced_path->switch_dirs));

	bool is_entry = false;
	for (u32 i = 0; i < block->entry_sensor_count; i++) {
		if (block->entry_sensors[i] == entry_sensor) {
			is_entry = true;
			break;
		}
	}
	if (!is_entry) {
		return false;
	}

	const track_node *current = entry_sensor;
	int max_hops = 20;
	int hops = 0;

	// Add the entry sensor as the first node
	if (traced_path->count < MAX_TRACED_PATH_NODES) {
		traced_path->nodes[traced_path->count] = current;
		traced_path->switch_dirs[traced_path->count] = DIR_STRAIGHT;
		traced_path->count++;
	}

	while (current && hops < max_hops) {
		hops++;

		for (u32 i = 0; i < block->exit_sensor_count; i++) {
			if (block->exit_sensors[i] == current) {
				traced_path->exit_sensor = current;
				return true; // Found the exit sensor
			}
		}

		const track_node *next = NULL;
		track_direction next_dir = DIR_STRAIGHT;

		switch (current->type) {
		case NODE_SENSOR:
		case NODE_MERGE:
			if (current->edge[DIR_STRAIGHT].dest) {
				next = current->edge[DIR_STRAIGHT].dest;
				next_dir = DIR_STRAIGHT;
			} else {
				return false;
			}
			break;

		case NODE_BRANCH: {
			switch_lookup_entry_t *switch_entry = conductor_get_switch_lookup_entry(current->num);
			if (switch_entry) {
				track_direction switch_dir = switch_entry->state.direction;
				if (current->edge[switch_dir].dest) {
					next = current->edge[switch_dir].dest;
					next_dir = switch_dir;
				} else {
					return false;
				}
			} else {
				if (current->edge[DIR_STRAIGHT].dest) {
					next = current->edge[DIR_STRAIGHT].dest;
					next_dir = DIR_STRAIGHT;
				} else {
					return false;
				}
			}
			break;
		}

		case NODE_EXIT:
		case NODE_ENTER:
		default:
			return false;
		}

		// Add the next node to the traced path
		if (next && traced_path->count < MAX_TRACED_PATH_NODES) {
			traced_path->nodes[traced_path->count] = next;
			// Store the switch direction for the previous node if it was a branch
			if (traced_path->count > 0 && traced_path->nodes[traced_path->count - 1]->type == NODE_BRANCH) {
				traced_path->switch_dirs[traced_path->count - 1] = next_dir;
			}
			traced_path->count++;
		}

		current = next;
	}

	return false;
}

marklin_error_t path_find(const track_node *from, const track_node *to, u8 train_id, bool allow_reversal,
			  bool use_block_exit_start, const track_block_t **excluded_blocks, u32 excluded_count,
			  path_node_pool_t *pool, path_result_t *result)
{
	if (!from || !to || !result || !pool) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (!pool->allocated || pool->owner_train_id != train_id) {
		log_error("Path find failed: invalid pool ownership (allocated=%d, owner=%d, train=%d)",
			  pool->allocated, pool->owner_train_id, train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Initialize result
	dlist_init(&result->nodes);
	result->total_distance = 0;
	result->num_reversals = 0;
	result->pool = pool;

	log_info("Path: Finding path from %s to %s (train %d, allow reversal: %d, use block exit start: %d)",
		 from->name, to->name, train_id, allow_reversal, use_block_exit_start);

	const track_node *actual_start = from;
	traced_path_t traced_path;
	bool has_traced_path = false;

	if (use_block_exit_start) {
		track_block_t *current_block = conductor_find_block_by_entry_node(from, g_conductor_data);
		if (current_block) {
			if (current_block->exit_sensor_count == 1) {
				actual_start = current_block->exit_sensors[0];
				log_debug("Path: Using block exit %s for pathfinding (train %d at entry %s)",
					  actual_start->name, train_id, from->name);
			} else if (current_block->exit_sensor_count > 1) {
				if (trace_path_through_block(current_block, from, &traced_path)) {
					actual_start = traced_path.exit_sensor;
					has_traced_path = true;
					log_debug("Path: Using traced exit %s for pathfinding (train %d at entry %s)",
						  actual_start->name, train_id, from->name);
				} else {
					actual_start = current_block->exit_sensors[0];
					log_debug(
						"Path: Tracing failed, using first exit %s of %u exits for pathfinding (train %d at entry %s)",
						actual_start->name, current_block->exit_sensor_count, train_id,
						from->name);
				}
			}
		}
	}

	const track_node *track_nodes = g_conductor_data->track_nodes;
	int track_size = g_conductor_data->track_size;
	if (track_size <= 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	static path_state_t states[TRACK_MAX * 2];
	static pq_entry_t pq_entries[MAX_PATH_QUEUE_SIZE];
	if (track_size * 2 > TRACK_MAX * 2) {
		return MARKLIN_ERROR_UNKNOWN;
	}

	int state_count = init_path_states(states, track_nodes, track_size);

	DECLARE_PRIORITY_QUEUE(pq, pq_entry_t, MAX_PATH_QUEUE_SIZE);
	pq_init(&pq, pq_compare_path);

	path_state_t *start_state = find_state(states, state_count, actual_start);
	if (!start_state) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	start_state->distance = 0;
	int pq_entry_index = 0;
	pq_entries[pq_entry_index] = (pq_entry_t){ .state = start_state, .priority = 0 };
	pq_push(&pq, &pq_entries[pq_entry_index]);
	pq_entry_index++;

	bool found = false;
	path_state_t *end_state = NULL;

	int __maybe_unused iteration = 0;
	while (!pq_is_empty(&pq)) {
		iteration++;
		pq_entry_t *entry = pq_pop(&pq);
		path_state_t *current = entry->state;

		if (current->visited) {
			continue;
		}
		current->visited = true;

		// Check if we reached the destination
		if (current->node == to) {
			found = true;
			end_state = current;
			break;
		}

		// The node is already the actual node we want to use
		const track_node *curr_node = current->node;
		if (!curr_node) {
			continue;
		}

		// Explore edges
		int num_edges = marklin_get_node_edge_count(curr_node);
		for (int i = 0; i < num_edges; i++) {
			const track_edge *edge = &curr_node->edge[i];
			if (!edge->dest) {
				continue;
			}

			// Find destination state
			path_state_t *next_state = find_state(states, state_count, edge->dest);
			if (!next_state) {
				continue;
			}

			// Check if destination node's block is excluded from pathfinding
			if (excluded_blocks && excluded_count > 0) {
				track_block_t *dest_block = conductor_find_block_containing_node(edge->dest, g_conductor_data, 
					true, true, true, false);
				if (is_block_excluded(dest_block, excluded_blocks, excluded_count)) {
					// Skip this edge as the destination block is excluded
					continue;
				}
			}

			// Calculate cost
			int edge_cost = path_get_edge_cost(edge, train_id, false);
			int new_distance = current->distance + edge_cost;

			// Update if better path found
			if (new_distance < next_state->distance) {
				// Check if updating prev would create a cycle
				if (would_create_cycle(next_state, current)) {
					continue;
				}

				next_state->distance = new_distance;
				next_state->prev = current;
				next_state->edge_used = edge;

				if (pq_entry_index < MAX_PATH_QUEUE_SIZE) {
					pq_entries[pq_entry_index] =
						(pq_entry_t){ .state = next_state, .priority = new_distance };
					pq_push(&pq, &pq_entries[pq_entry_index]);
					pq_entry_index++;
				}
			}
		}

		// Only allow reversals at the start of the path to simplify logic
		if (allow_reversal && current == start_state && current->node->reverse &&
		    !path_is_reversal_blacklisted(current->node)) {
			// Find the reverse node - if we're at the original, find reverse; if at reverse, find original
			const track_node *reverse_node = current->node->reverse;
			path_state_t *reverse_state = find_state(states, state_count, reverse_node);
			if (reverse_state && !reverse_state->visited) {
				int reversal_cost = path_get_edge_cost(NULL, train_id, true);
				int new_distance = current->distance + reversal_cost;

				if (new_distance < reverse_state->distance) {
					// Check if updating prev would create a cycle
					if (would_create_cycle(reverse_state, current)) {
						continue;
					}

					reverse_state->distance = new_distance;
					reverse_state->prev = current;
					reverse_state->edge_used = NULL;

					if (pq_entry_index < MAX_PATH_QUEUE_SIZE) {
						pq_entries[pq_entry_index] = (pq_entry_t){ .state = reverse_state,
											   .priority = new_distance };
						pq_push(&pq, &pq_entries[pq_entry_index]);
						pq_entry_index++;
					}
				}
			}
		}
	}

	marklin_error_t error = MARKLIN_ERROR_OK;

	if (found && end_state) {
		build_path_result(end_state, result, has_traced_path ? &traced_path : NULL);
		path_print(result);
	} else {
		// No path found
		dlist_init(&result->nodes);
		result->total_distance = -1;
		result->num_reversals = 0;
		error = MARKLIN_ERROR_NO_PATH;
	}

	return error;
}

void path_cleanup(path_result_t *result)
{
	if (!result) {
		return;
	}

	// Free all nodes back to the pool if we have one
	if (result->pool) {
		struct dlist_node *node_entry;
		struct dlist_node *next_entry;
		dlist_for_each_safe(node_entry, next_entry, &result->nodes)
		{
			path_node_t *node = dlist_entry(node_entry, path_node_t, list);
			dlist_del(&node->list);
			path_node_free(result->pool, node);
		}
		result->pool = NULL;
	}

	dlist_init(&result->nodes);
	result->total_distance = 0;
	result->num_reversals = 0;
}

void path_print(path_result_t *result)
{
	if (!result) {
		console_printf("Path: Invalid result\r\n");
		return;
	}

	if (dlist_is_empty(&result->nodes)) {
		console_printf("Path: No path found\r\n");
		return;
	}

	console_printf("\r\n=== Path Diagram <%p> ===\r\n", result);

	// Print the path nodes
	console_printf("Path: ");

	path_node_t *node;
	bool first = true;

	dlist_for_each_entry(node, &result->nodes, path_node_t, list)
	{
		if (!first) {
			console_printf(" -> ");
		}
		first = false;

		// Print node name
		console_printf("%s", node->node->name);

		// Add reversal indicator
		if (node->reverse_here) {
			console_printf("[R]");
		}

		// Add switch direction for branch nodes
		if (node->node->type == NODE_BRANCH) {
			if (node->switch_dir == DIR_STRAIGHT) {
				console_printf("(S)");
			} else if (node->switch_dir == DIR_CURVED) {
				console_printf("(C)");
			}
		}
	}

	console_printf("\n");

	// Print summary
	console_printf("\r\nSummary:\r\n");
	console_printf("  Total Distance: %d mm\r\n", result->total_distance);
	console_printf("  Reversals: %d\r\n", result->num_reversals);

	// Print legend
	// console_printf("\r\nLegend:\r\n");
	// console_printf("  [R] = Reversal required\r\n");
	// console_printf("  (S) = Switch set to Straight\r\n");
	// console_printf("  (C) = Switch set to Curved\r\n");
	// console_printf("===================\r\n\r\n");
}

// ############################################################################
// # Path Pool Management Functions
// ############################################################################

void path_pools_init(path_node_pool_t pools[MAX_CONCURRENT_PATHS], struct dlist_node *free_pools)
{
	if (!pools || !free_pools) {
		return;
	}

	// Initialize the free pools list
	dlist_init(free_pools);

	// Initialize each pool
	for (int i = 0; i < MAX_CONCURRENT_PATHS; i++) {
		path_node_pool_t *pool = &pools[i];

		// Initialize pool metadata
		pool->allocated = false;
		pool->owner_train_id = 0;
		pool->nodes_in_use = 0;

		// Initialize the free nodes list for this pool
		dlist_init(&pool->free_nodes);

		// Add all nodes to the free list
		for (int j = 0; j < MAX_PATH_NODES_PER_PATH; j++) {
			path_node_t *node = &pool->nodes[j];
			dlist_init_node(&node->list);
			dlist_insert_tail(&pool->free_nodes, &node->list);
		}

		// Add this pool to the free pools list
		dlist_init_node(&pool->pool_list_node);
		dlist_insert_tail(free_pools, &pool->pool_list_node);
	}

	log_info("Path pools initialized: %d pools with %d nodes each", MAX_CONCURRENT_PATHS, MAX_PATH_NODES_PER_PATH);
}

path_node_pool_t *path_pool_alloc(struct dlist_node *free_pools, u8 train_id)
{
	if (!free_pools || dlist_is_empty(free_pools) || train_id == 0) {
		log_error("Path pool allocation failed: no free pools or invalid train_id");
		return NULL;
	}

	// Get the first free pool
	struct dlist_node *node = dlist_first(free_pools);
	dlist_del(node);

	path_node_pool_t *pool = dlist_entry(node, path_node_pool_t, pool_list_node);
	pool->allocated = true;
	pool->owner_train_id = train_id;
	pool->nodes_in_use = 0;

	log_debug("Allocated path pool for train %d", train_id);
	return pool;
}

void path_pool_free(path_node_pool_t *pool, struct dlist_node *free_pools)
{
	if (!pool || !free_pools) {
		return;
	}

	// Reset all nodes to the free list
	dlist_init(&pool->free_nodes);
	for (int i = 0; i < MAX_PATH_NODES_PER_PATH; i++) {
		path_node_t *node = &pool->nodes[i];
		dlist_init_node(&node->list);
		dlist_insert_tail(&pool->free_nodes, &node->list);
	}

	// Reset pool metadata
	log_debug("Freeing path pool from train %d (had %d nodes in use)", pool->owner_train_id, pool->nodes_in_use);

	pool->allocated = false;
	pool->owner_train_id = 0;
	pool->nodes_in_use = 0;

	// Add back to free pools list
	dlist_insert_tail(free_pools, &pool->pool_list_node);
}

path_node_t *path_node_alloc(path_node_pool_t *pool)
{
	if (!pool || !pool->allocated || dlist_is_empty(&pool->free_nodes)) {
		if (pool && pool->allocated) {
			log_error("Path node allocation failed: pool exhausted for train %d (%d/%d nodes used)",
				  pool->owner_train_id, pool->nodes_in_use, MAX_PATH_NODES_PER_PATH);
		}
		return NULL;
	}

	struct dlist_node *node = dlist_first(&pool->free_nodes);
	dlist_del(node);

	path_node_t *path_node = dlist_entry(node, path_node_t, list);
	pool->nodes_in_use++;

	// Initialize the node
	path_node->node = NULL;
	path_node->switch_dir = DIR_STRAIGHT;
	path_node->reverse_here = false;
	dlist_init_node(&path_node->list);

	return path_node;
}

void path_node_free(path_node_pool_t *pool, path_node_t *node)
{
	if (!pool || !node) {
		return;
	}

	// Clear node data
	node->node = NULL;
	node->switch_dir = DIR_STRAIGHT;
	node->reverse_here = false;

	// Return to free list
	dlist_insert_tail(&pool->free_nodes, &node->list);
	pool->nodes_in_use--;
}

// ############################################################################
// # Reversal Blacklist Management Functions
// ############################################################################

// Static blacklist storage
static const track_node *reversal_blacklist[MAX_BLACKLISTED_REVERSAL_NODES];
static u32 reversal_blacklist_count = 0;

// External reference to conductor data for node lookup
extern conductor_task_data_t *g_conductor_data;

// Helper function to populate blacklist from hardcoded list
static void path_populate_hardcoded_blacklist(void)
{
	if (!g_conductor_data || !g_conductor_data->track_nodes) {
		log_error("Cannot populate reversal blacklist: conductor data not available");
		return;
	}

	const track_node *track_nodes = g_conductor_data->track_nodes;
	int track_size = g_conductor_data->track_size;
	UNUSED(track_nodes);
	UNUSED(track_size);

#undef REVERSAL_BLACKLIST_NODE
#define REVERSAL_BLACKLIST_NODE(name)                                                               \
	do {                                                                                        \
		const track_node *node = marklin_find_node_by_name(track_nodes, track_size, #name); \
		if (node) {                                                                         \
			if (reversal_blacklist_count < MAX_BLACKLISTED_REVERSAL_NODES) {            \
				reversal_blacklist[reversal_blacklist_count] = node;                \
				reversal_blacklist_count++;                                         \
				log_debug("Added %s to reversal blacklist", #name);                 \
			} else {                                                                    \
				log_error("Reversal blacklist full, cannot add %s", #name);         \
				break;                                                              \
			}                                                                           \
		} else {                                                                            \
			log_warn("Node %s not found in track layout, skipping", #name);             \
		}                                                                                   \
	} while (0);

#include "marklin/conductor/reversal_blacklist.h"

#undef REVERSAL_BLACKLIST_NODE
}

void path_init_reversal_blacklist(void)
{
	reversal_blacklist_count = 0;
	for (u32 i = 0; i < MAX_BLACKLISTED_REVERSAL_NODES; i++) {
		reversal_blacklist[i] = NULL;
	}

	// Populate with hardcoded blacklist from header file
	path_populate_hardcoded_blacklist();

	log_info("Reversal blacklist initialized with %d/%d nodes", reversal_blacklist_count,
		 MAX_BLACKLISTED_REVERSAL_NODES);

	// Debug: Print all blacklisted nodes
	if (reversal_blacklist_count > 0) {
		log_debug("Blacklisted reversal nodes:");
		for (u32 i = 0; i < reversal_blacklist_count; i++) {
			log_debug("  [%d]: %s", i, reversal_blacklist[i]->name);
		}
	}
}

bool path_is_reversal_blacklisted(const track_node *node)
{
	if (!node) {
		return false;
	}

	for (u32 i = 0; i < reversal_blacklist_count; i++) {
		if (reversal_blacklist[i] == node) {
			return true;
		}
	}
	return false;
}

void path_clear_reversal_blacklist(void)
{
	reversal_blacklist_count = 0;
	for (u32 i = 0; i < MAX_BLACKLISTED_REVERSAL_NODES; i++) {
		reversal_blacklist[i] = NULL;
	}
	log_info("Reversal blacklist cleared");
}

u32 path_get_reversal_blacklist_count(void)
{
	return reversal_blacklist_count;
}
