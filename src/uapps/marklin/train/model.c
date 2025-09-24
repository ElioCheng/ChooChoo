#include "marklin/train/model.h"
#include "marklin/train/kinematics.h"
#include "marklin/train/model_defaults.h"
#include "marklin/common/constants.h"
#include "marklin/error.h"
#include "syscall.h"
#include "clock.h"
#include "clock_server.h"
#include "string.h"
#include "name.h"
#define LOG_MODULE "MODEL"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"
// #include "klog.h"

// ############################################################################
// # Global Model Storage
// ############################################################################

static kinematic_model_collection_t g_model_collection;
static bool g_model_system_initialized = false;
static int g_clock_server_tid = -1;

// NOTE: Default parameters are now loaded from model_defaults.h
// This provides train-specific calibrated values instead of generic defaults

// ############################################################################
// # Internal Helper Functions
// ############################################################################

static marklin_error_t model_init_clock_server(void)
{
	if (g_clock_server_tid < 0) {
		g_clock_server_tid = WhoIs(CLOCK_SERVER_NAME);
		if (g_clock_server_tid < 0) {
			log_error("Failed to find clock server for model system");
			return MARKLIN_ERROR_NOT_FOUND;
		}
	}
	return MARKLIN_ERROR_OK;
}

static kinematic_time_t model_current_time(void)
{
	if (g_clock_server_tid < 0) {
		if (model_init_clock_server() != MARKLIN_ERROR_OK) {
			return 0;
		}
	}
	return Time(g_clock_server_tid);
}

// NOTE: model_init_speed_params function removed - now using predefined defaults from model_defaults.h

static bool model_is_valid_train_id(u8 train_id)
{
	for (int i = 0; i < ALL_POSSIBLE_TRAINS_COUNT; i++) {
		if (train_id == all_possible_trains[i]) {
			return true;
		}
	}
	return false;
}

static int model_train_id_to_index(u8 train_id)
{
	for (int i = 0; i < ALL_POSSIBLE_TRAINS_COUNT; i++) {
		if (train_id == all_possible_trains[i]) {
			return i;
		}
	}
	return -1;
}

static const train_model_defaults_t *model_find_train_defaults(u8 train_id)
{
	return get_train_model_defaults(train_id);
}

// ############################################################################
// # Model Management API Implementation
// ############################################################################

marklin_error_t kinematic_model_init(void)
{
	if (g_model_system_initialized) {
		return MARKLIN_ERROR_OK;
	}

	// Initialize model collection
	memset(&g_model_collection, 0, sizeof(kinematic_model_collection_t));

	// Initialize clock server connection
	marklin_error_t result = model_init_clock_server();
	if (result != MARKLIN_ERROR_OK) {
		return result;
	}

	kinematic_time_t current_time = model_current_time();
	g_model_collection.last_global_update = current_time;

	g_model_system_initialized = true;
	log_debug("Kinematic model system initialized");

	return MARKLIN_ERROR_OK;
}

train_kinematic_model_t *kinematic_model_get(train_task_data_t *train_data)
{
	if (!g_model_system_initialized) {
		log_error("Model system not initialized");
		return NULL;
	}

	if (!train_data) {
		log_error("Invalid train_data pointer");
		return NULL;
	}

	// Return cached model pointer if available
	if (train_data->kinematic_model) {
		return train_data->kinematic_model;
	}

	// Initialize model pointer from global collection
	if (!model_is_valid_train_id(train_data->train_id)) {
		log_error("Invalid train ID: %d", train_data->train_id);
		return NULL;
	}

	int model_index = model_train_id_to_index(train_data->train_id);
	if (model_index < 0) {
		log_error("Failed to map train ID %d to model index", train_data->train_id);
		return NULL;
	}

	// Create model if it doesn't exist
	if (!g_model_collection.model_initialized[model_index]) {
		if (kinematic_model_create_default(train_data) != MARKLIN_ERROR_OK) {
			log_error("Failed to create default model for train %d", train_data->train_id);
			return NULL;
		}
	}

	// Cache the model pointer in train_data for future use
	train_data->kinematic_model = &g_model_collection.models[model_index];
	return train_data->kinematic_model;
}

marklin_error_t kinematic_model_create_default(train_task_data_t *train_data)
{
	if (!g_model_system_initialized) {
		return MARKLIN_ERROR_NOT_INITIALIZED;
	}

	if (!model_is_valid_train_id(train_data->train_id)) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	int model_index = model_train_id_to_index(train_data->train_id);
	if (model_index < 0) {
		log_error("Failed to map train ID %d to model index", train_data->train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Find predefined defaults for this train
	const train_model_defaults_t *defaults = model_find_train_defaults(train_data->train_id);
	if (!defaults) {
		log_error("No predefined defaults found for train %d", train_data->train_id);
		return MARKLIN_ERROR_NOT_FOUND;
	}

	train_kinematic_model_t *model = &g_model_collection.models[model_index];
	kinematic_time_t current_time = model_current_time();

	// Initialize model structure
	memset(model, 0, sizeof(train_kinematic_model_t));
	model->train_id = train_data->train_id;

	// Copy predefined speed parameters
	for (u8 i = 0; i < KINEMATIC_TOTAL_SPEED_LEVELS; i++) {
		memcpy(&model->speeds[i], &defaults->default_speeds[i], sizeof(kinematic_speed_params_t));

		// Update timestamps to current time
		model->speeds[i].last_velocity_update = current_time;
		model->speeds[i].last_acceleration_update = current_time;
		model->speeds[i].last_stop_update = current_time;
	}

	log_debug("Speed parameters copied for train %d", train_data->train_id);

	// Mark as initialized
	g_model_collection.model_initialized[model_index] = true;
	g_model_collection.active_model_count++;

	log_debug("Created default kinematic model for train %d using predefined defaults", train_data->train_id);

	return MARKLIN_ERROR_OK;
}

marklin_error_t kinematic_model_update_velocity(train_task_data_t *train_data, u8 speed_level, bool from_higher_speed,
						kinematic_measurement_t *measurement)
{
	if (!measurement) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	train_kinematic_model_t *model = kinematic_model_get(train_data);
	if (!model) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	u8 speed_index = kinematic_speed_to_index(speed_level, from_higher_speed);
	if (speed_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	kinematic_speed_params_t *params = &model->speeds[speed_index];
	kinematic_time_t current_time = model_current_time();

	// Calculate velocity from measurement
	kinematic_velocity_t measured_velocity = kinematic_velocity(measurement->distance, measurement->travel_time);

	// Update velocity using EWMA
	if (params->velocity == 0) {
		// First measurement - use directly
		params->velocity = measured_velocity;
	} else {
		// Update using EWMA
		params->velocity = kinematic_ewma_update(params->velocity, measured_velocity);
	}

	params->last_velocity_update = current_time;

	// Split velocity for precise display
	i64 velocity_int, velocity_frac;
	kinematic_velocity_split(params->velocity, &velocity_int, &velocity_frac);
	log_debug("Updated velocity for train %d speed %d: %lld.%06lld mm/tick", train_data->train_id, speed_level,
		  velocity_int, velocity_frac);

	return MARKLIN_ERROR_OK;
}

marklin_error_t kinematic_model_update_acceleration(train_task_data_t *train_data, u8 from_speed_index,
						    u8 to_speed_index, kinematic_accel_t acceleration)
{
	train_kinematic_model_t *model = kinematic_model_get(train_data);
	if (!model) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	// Update acceleration for the target speed
	if (to_speed_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	kinematic_speed_params_t *params = &model->speeds[from_speed_index];
	kinematic_time_t current_time = model_current_time();

	// Update acceleration using EWMA
	if (acceleration > 0) {
		params->acceleration = kinematic_ewma_update(params->acceleration, acceleration);
	} else {
		params->deceleration = kinematic_ewma_update(params->deceleration, acceleration);
	}

	params->last_acceleration_update = current_time;

	// Split acceleration for precise display
	i64 accel_int, accel_frac;
	kinematic_accel_split(acceleration, &accel_int, &accel_frac);
	log_debug("Updated acceleration for train %d from speed %d to %d: %lld.%06lld mm/tick²", train_data->train_id,
		  from_speed_index, to_speed_index, accel_int, accel_frac);

	return MARKLIN_ERROR_OK;
}

marklin_error_t kinematic_model_update_stopping(train_task_data_t *train_data, u8 speed_level_index,
						kinematic_distance_t stop_distance, kinematic_time_t stop_time)
{
	train_kinematic_model_t *model = kinematic_model_get(train_data);
	if (!model) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	if (speed_level_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	kinematic_speed_params_t *params = &model->speeds[speed_level_index];
	kinematic_time_t current_time = model_current_time();

	// Update stopping parameters using EWMA
	if (params->stop_distance == 0) {
		params->stop_distance = stop_distance;
		params->stop_time = stop_time;
	} else {
		params->stop_distance = kinematic_ewma_update(params->stop_distance, stop_distance);
		params->stop_time = kinematic_ewma_update(params->stop_time, stop_time);
	}

	params->last_stop_update = current_time;

	log_debug("Updated stopping for train %d speed %d: distance %lld mm, time %lld ticks", train_data->train_id,
		  speed_level_index, stop_distance, stop_time);

	return MARKLIN_ERROR_OK;
}

// ############################################################################
// # Model Query API Implementation
// ############################################################################

kinematic_velocity_t kinematic_model_get_velocity(train_task_data_t *train_data, u8 speed_level, bool from_higher_speed)
{
	train_kinematic_model_t *model = kinematic_model_get(train_data);
	if (!model) {
		return 0;
	}

	u8 speed_index = kinematic_speed_to_index(speed_level, from_higher_speed);
	if (speed_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return 0;
	}

	return model->speeds[speed_index].velocity;
}

kinematic_accel_t kinematic_model_get_acceleration(train_task_data_t *train_data, u8 from_speed, bool from_higher_speed,
						   u8 to_speed)
{
	train_kinematic_model_t *model = kinematic_model_get(train_data);
	if (!model) {
		return 0;
	}

	u8 from_speed_index = kinematic_speed_to_index(from_speed, from_higher_speed);
	if (from_speed_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return 0;
	}

	kinematic_speed_params_t *params = &model->speeds[from_speed_index];
	return (from_speed < to_speed) ? params->acceleration : -params->deceleration;
}

kinematic_distance_t kinematic_model_get_stop_distance(train_task_data_t *train_data, u8 speed_level,
						       bool from_higher_speed)
{
	train_kinematic_model_t *model = kinematic_model_get(train_data);
	if (!model) {
		return 0;
	}

	u8 speed_index = kinematic_speed_to_index(speed_level, from_higher_speed);
	if (speed_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return 0;
	}

	return model->speeds[speed_index].stop_distance;
}

kinematic_time_t kinematic_model_get_stop_time(train_task_data_t *train_data, u8 speed_level, bool from_higher_speed)
{
	train_kinematic_model_t *model = kinematic_model_get(train_data);
	if (!model) {
		return 0;
	}

	u8 speed_index = kinematic_speed_to_index(speed_level, from_higher_speed);
	if (speed_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return 0;
	}

	return model->speeds[speed_index].stop_time;
}

// ############################################################################
// # Model Persistence API Implementation
// ############################################################################

marklin_error_t kinematic_model_print_defaults(train_task_data_t *train_data)
{
	train_kinematic_model_t *model = kinematic_model_get(train_data);
	if (!model) {
		return MARKLIN_ERROR_NOT_FOUND;
	}

	log_info("========================================");
	log_info("MODEL DEFAULTS FORMAT FOR TRAIN %d", train_data->train_id);
	log_info("========================================");
	log_info("// Copy the following lines to model_defaults.c:");

	log_info("static kinematic_speed_params_t train_%d_speeds[KINEMATIC_TOTAL_SPEED_LEVELS] = {",
		 train_data->train_id);
	for (u8 i = 0; i < KINEMATIC_TOTAL_SPEED_LEVELS; i++) {
		kinematic_speed_params_t *params = &model->speeds[i];

		log_info("{%lld,%lld,%lld,%lld,%lld,0,0,0},", params->velocity, params->acceleration,
			 params->deceleration, params->stop_distance, params->stop_time);
	}

	log_info("};");
	log_info("");
	log_info("========================================");

	return MARKLIN_ERROR_OK;
}
