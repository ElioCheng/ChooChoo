#ifndef MARKLIN_SENSOR_H
#define MARKLIN_SENSOR_H

#include "marklin/conductor/api.h"
#include "marklin/error.h"

#define MARKLIN_SENSOR_TASK_PRIORITY 3
#define MARKLIN_SENSOR_QUERY_INTERVAL_MS 250

#define MARKLIN_SENSOR_BANK_COUNT 5
#define MARKLIN_SENSOR_MAX_COUNT (MARKLIN_SENSOR_BANK_COUNT * 16)

// Sensor commands
#define MARKLIN_CMD_SENSOR_REPORT_ALL 0x85 // Read all banks up to and including 5th bank
#define MARKLIN_CMD_SENSOR_BANK_BASE 0xC0 // Base command for reading individual banks
#define MARKLIN_CMD_SENSOR_RESET_ON 0xC0 // Turn sensor reset mode on
#define MARKLIN_CMD_SENSOR_RESET_OFF 0x80 // Turn sensor reset mode off

void conductor_consume_sensor_data(u16 *sensor_data, u32 tick);
u32 sensor_get_states(marklin_sensor_state_t *sensors, u32 count);

marklin_error_t sensor_set_reset_mode(u8 reset_on);

void sensor_timer_task(void);

#endif /* MARKLIN_SENSOR_H */
