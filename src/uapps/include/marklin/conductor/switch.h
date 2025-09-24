#ifndef MARKLIN_SWITCH_H
#define MARKLIN_SWITCH_H

#include "marklin/error.h"
#include "marklin/common/track_node.h"

#define MARKLIN_SWITCH_MAX_COUNT 32

#define MARKLIN_SWITCH_CMD_DELAY_MS 150
#define MARKLIN_SOLENOID_DEACTIVATE_MS 250

// Switch commands
#define MARKLIN_CMD_SWITCH_STRAIGHT 0x21
#define MARKLIN_CMD_SWITCH_CURVE 0x22
#define MARKLIN_CMD_SOLENOID_OFF 0x20

// Switch state management
void conductor_consume_switch_update(u8 switch_id, track_direction direction, u32 tick);

marklin_error_t switch_set_direction(u8 switch_id, track_direction direction, u8 disengage_solenoid, bool force_update);

#endif /* MARKLIN_SWITCH_H */
