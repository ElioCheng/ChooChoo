#include "marklin/conductor/sensor.h"
#include "marklin/conductor/conductor.h"
#include "marklin/error.h"
#include "marklin/msgqueue/api.h"
#include "marklin/command/api.h"
#include "marklin/command/command.h"
#include "clock.h"
#include "name.h"
#include "compiler.h"
#include "io.h"
#include "syscall.h"

// ############################################################################
// # Global Data
// ############################################################################

extern conductor_task_data_t *g_conductor_data;

// Sensor timer task globals
static int clock_server_tid = -1;
static int conductor_tid = -1;

// ############################################################################
// # Forward Declarations
// ############################################################################

// Sensor processing helpers
static u8 sensor_reverse_bits(u8 byte);
static sensor_lookup_entry_t *sensor_get_lookup_entry(u8 bank, u8 sensor_id);
static void sensor_publish_update(sensor_lookup_entry_t *entry, u8 sensor_triggered);
static bool sensor_is_blacklisted(u8 bank, u8 sensor_id);

// Sensor timer task helpers
static void sensor_consume_response(int conductor_tid);

// ############################################################################
// # Sensor Processing Functions
// ############################################################################

static u8 sensor_reverse_bits(u8 byte)
{
	u8 result = 0;
	for (int i = 0; i < 8; i++) {
		result = (result << 1) | (byte & 1);
		byte >>= 1;
	}
	return result;
}

static bool sensor_is_blacklisted(u8 bank, u8 sensor_id)
{
	if (!g_conductor_data) {
		return false;
	}

	if (bank >= 5 || sensor_id < 1 || sensor_id > 16) {
		return false;
	}

	return g_conductor_data->sensor_blacklist_cache[bank][sensor_id - 1];
}

static sensor_lookup_entry_t *sensor_get_lookup_entry(u8 bank, u8 sensor_id)
{
	for (int i = 0; i < g_conductor_data->sensor_count; i++) {
		sensor_lookup_entry_t *entry = &g_conductor_data->sensor_lookup[i];
		if (entry->state.bank == bank && entry->state.sensor_id == sensor_id) {
			return entry;
		}
	}
	return NULL;
}

static void sensor_publish_update(sensor_lookup_entry_t *entry, u8 sensor_triggered)
{
	marklin_sensor_state_t update_data = { .bank = entry->state.bank,
					       .sensor_id = entry->state.sensor_id,
					       .triggered = sensor_triggered,
					       .last_triggered_tick = entry->state.last_triggered_tick };

	Marklin_MsgQueue_PublishTyped(MARKLIN_MSGQUEUE_EVENT_TYPE_SENSOR_UPDATE, &update_data);
}

// ############################################################################
// # Public API Implementation
// ############################################################################

u32 sensor_get_states(marklin_sensor_state_t *sensors, u32 count)
{
	u32 found_count = 0;
	for (u32 i = 0; i < count; i++) {
		marklin_sensor_state_t *query_sensor = &sensors[i];
		bool found = false;

		for (int j = 0; j < g_conductor_data->sensor_count; j++) {
			sensor_lookup_entry_t *entry = &g_conductor_data->sensor_lookup[j];
			if (entry->state.bank == query_sensor->bank &&
			    entry->state.sensor_id == query_sensor->sensor_id) {
				*query_sensor = entry->state;
				found = true;
				found_count++;
				break;
			}
		}

		if (!found) {
			query_sensor->triggered = 0xFF;
		}
	}

	return found_count;
}

void conductor_consume_sensor_data(u16 *sensor_data, u32 tick)
{
	if (!g_conductor_data)
		return;

	for (u8 bank = 0; bank < MARKLIN_SENSOR_BANK_COUNT; bank++) {
		u16 bank_data = sensor_data[bank];

		for (u8 sensor_bit = 0; sensor_bit < 16; sensor_bit++) {
			u8 sensor_triggered = (bank_data >> sensor_bit) & 0x01;
			u8 sensor_number = sensor_bit + 1; // Sensors are numbered 1-16

			sensor_lookup_entry_t *entry = sensor_get_lookup_entry(bank, sensor_number);
			if (!entry) {
				continue;
			}

			if (entry->state.triggered != sensor_triggered) {
				entry->state.triggered = sensor_triggered;
				if (sensor_triggered) {
					entry->state.last_triggered_tick = tick;
				}

				if (sensor_is_blacklisted(bank, sensor_number)) {
					continue;
				}

				sensor_publish_update(entry, sensor_triggered);
			}
		}
	}
}

marklin_error_t sensor_set_reset_mode(u8 reset_on)
{
	return Marklin_ScheduleCommandWithPriority(
		MARKLIN_CMD_TYPE_SINGLE, reset_on ? MARKLIN_CMD_SENSOR_RESET_ON : MARKLIN_CMD_SENSOR_RESET_OFF, 0, 0,
		MARKLIN_CMD_PRIORITY_CRITICAL, 0);
}

// ############################################################################
// # Sensor Timer Task Implementation
// ############################################################################

marklin_error_t Conductor_OnSensorData(int conductor_tid, u16 *sensor_data, u32 tick)
{
	if (conductor_tid < 0) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	if (!sensor_data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	marklin_conductor_request_t request;
	request.type = MARKLIN_CONDUCTOR_REQ_ON_SENSOR_DATA;
	request.sensor_data.sensor_data = sensor_data;
	request.sensor_data.tick = tick;

	marklin_conductor_reply_t reply;

	int result = Send(conductor_tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return MARKLIN_ERROR_COMMUNICATION;
	}

	return MARKLIN_ERROR_OK;
}

static __maybe_unused void sensor_consume_response(int conductor_tid)
{
	u16 sensor_data[MARKLIN_SENSOR_BANK_COUNT];

	for (u8 bank = 0; bank < MARKLIN_SENSOR_BANK_COUNT; bank++) {
		int bank_result[2] = { 0, 0 };

		// Read first byte of the bank
		bank_result[0] = marklin_getc();
		if (bank_result[0] == IO_NO_DATA) {
			return;
		}

		// Read second byte of the bank
		bank_result[1] = marklin_getc();
		if (bank_result[1] == IO_NO_DATA) {
			return;
		}

		u8 byte0_reversed = sensor_reverse_bits((u8)bank_result[0]);
		u8 byte1_reversed = sensor_reverse_bits((u8)bank_result[1]);
		sensor_data[bank] = (u16)byte0_reversed | ((u16)byte1_reversed << 8);
	}

	Conductor_OnSensorData(conductor_tid, sensor_data, Time(clock_server_tid));
}

void __noreturn sensor_timer_task(void)
{
	clock_server_tid = WhoIs(CLOCK_SERVER_NAME);

	conductor_tid = WhoIs(MARKLIN_CONDUCTOR_SERVER_NAME);

	if (clock_server_tid < 0 || conductor_tid < 0) {
		Panic("Sensor timer task failed to get server tids [clock_server_tid: %d, conductor_tid: %d]",
		      clock_server_tid, conductor_tid);
	}

	while (marklin_trygetc() != IO_NO_DATA) {
	}

	sensor_set_reset_mode(1);

	for (;;) {
		u32 current_tick = Time(clock_server_tid);
		u32 next_tick = current_tick + MS_TO_TICK(MARKLIN_SENSOR_QUERY_INTERVAL_MS);
		Marklin_ScheduleCommandWithPriority(MARKLIN_CMD_TYPE_SINGLE, MARKLIN_CMD_SENSOR_REPORT_ALL, 0, 0,
						    MARKLIN_CMD_PRIORITY_LOW, 0);
		sensor_consume_response(conductor_tid);
		DelayUntil(clock_server_tid, next_tick);
	}

	UNREACHABLE();
}
