#ifndef MARKLIN_CONDUCTOR_PATH_H
#define MARKLIN_CONDUCTOR_PATH_H

#include "types.h"
#include "dlist.h"
#include "marklin/common/track_node.h"
#include "marklin/error.h"

// Cost penalties for pathfinding
#define PATH_COST_REVERSAL 1000

#define PATH_MIN_REVERSAL_LENGTH 500

// Reversal blacklist configuration
#define MAX_BLACKLISTED_REVERSAL_NODES 32

// Path pool configuration
#define MAX_PATH_NODES_PER_PATH 256
#define MAX_CONCURRENT_PATHS 8

typedef struct path_node {
	const track_node *node;
	track_direction switch_dir; // Required switch setting (if branch node)
	bool reverse_here; // Train should reverse at this node
	struct dlist_node list;
} path_node_t;

// Forward declarations
typedef struct path_node_pool path_node_pool_t;
typedef struct track_block_struct track_block_t;

typedef struct path_result {
	struct dlist_node nodes;
	int total_distance;
	int num_reversals;
	path_node_pool_t *pool;
} path_result_t;

// Path node pool for dynamic allocation
struct path_node_pool {
	path_node_t nodes[MAX_PATH_NODES_PER_PATH];
	struct dlist_node free_nodes;
	struct dlist_node pool_list_node;
	bool allocated;
	u8 owner_train_id;
	u32 nodes_in_use;
};

marklin_error_t path_find(const track_node *from, const track_node *to, u8 train_id, bool allow_reversal,
			  bool use_block_exit_start, const track_block_t **excluded_blocks, u32 excluded_count,
			  path_node_pool_t *pool, path_result_t *result);

void path_cleanup(path_result_t *result);

// Pool management functions
void path_pools_init(path_node_pool_t pools[MAX_CONCURRENT_PATHS], struct dlist_node *free_pools);
path_node_pool_t *path_pool_alloc(struct dlist_node *free_pools, u8 train_id);
void path_pool_free(path_node_pool_t *pool, struct dlist_node *free_pools);
path_node_t *path_node_alloc(path_node_pool_t *pool);
void path_node_free(path_node_pool_t *pool, path_node_t *node);

// Helper functions
int path_get_edge_cost(const track_edge *edge, u8 train_id, bool is_reversal);

void path_print(path_result_t *result);

// Reversal blacklist management functions
void path_init_reversal_blacklist(void);
bool path_is_reversal_blacklisted(const track_node *node);
u32 path_get_reversal_blacklist_count(void);

#endif /* MARKLIN_CONDUCTOR_PATH_H */
