#ifndef MARKLIN_CONDUCTOR_BLOCK_DEFINITIONS_H
#define MARKLIN_CONDUCTOR_BLOCK_DEFINITIONS_H

#include "marklin/topology/api.h"
#include "types.h"
#include "marklin/conductor/block.h"

#define MAX_BLOCK_DEF_SENSORS 8
#define MAX_BLOCK_DEF_TURNOUTS 4

typedef struct {
	u32 block_id;

	const char *entry_sensor_names[MAX_BLOCK_DEF_SENSORS];
	u32 entry_sensor_count;

	const char *exit_sensor_names[MAX_BLOCK_DEF_SENSORS];
	u32 exit_sensor_count;

	const char *internal_sensor_names[MAX_BLOCK_DEF_SENSORS];
	u32 internal_sensor_count;

	const char *turnout_names[MAX_BLOCK_DEF_TURNOUTS];
	u32 turnout_count;

	u32 connected_block_ids[MAX_BLOCK_DEF_SENSORS];
	u32 connected_block_count;
} hardcoded_block_def_t;

void conductor_init_hardcoded_blocks(conductor_task_data_t *data, marklin_track_type_t layout);

const hardcoded_block_def_t *conductor_get_block_definitions(marklin_track_type_t layout, u32 *block_count);

const track_node *conductor_resolve_sensor_name(const char *sensor_name, conductor_task_data_t *data);

const track_node *conductor_resolve_turnout_name(const char *turnout_name, conductor_task_data_t *data);

bool conductor_validate_initialized_blocks(conductor_task_data_t *data);

#endif /* MARKLIN_CONDUCTOR_BLOCK_DEFINITIONS_H */