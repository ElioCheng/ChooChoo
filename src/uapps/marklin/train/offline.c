#include "marklin/train/offline.h"
#include "marklin/train/api.h"
#include "marklin/train/calibration.h"
#include "marklin/train/model.h"
#include "marklin/train/kinematics.h"
#include "marklin/controller/api.h"
#include "marklin/conductor/api.h"
#include "marklin/topology/track.h"
#include "marklin/error.h"
#include "syscall.h"
#include "clock.h"
#include "clock_server.h"
#include "string.h"
#include "name.h"
#include "compiler.h"
#define LOG_MODULE "OFFLINE"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"
#include "klog.h"

// ############################################################################
// # Utility Functions Implementation
// ############################################################################

marklin_error_t offline_create_default_config(offline_experiment_type_t type, offline_experiment_context_t *context)
{
	if (!context) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	context->type = type;

	if (type == OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
		context->velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_NAVIGATING_TO_B5;
		context->velocity_loop.offline_b5_cycle_count = 0;
		context->velocity_loop.offline_loop_distance = 0;
		context->velocity_loop.offline_current_speed_index = 0;
		context->velocity_loop.offline_speed_levels[0] = 1;
		context->velocity_loop.offline_speed_count = 1;
		context->velocity_loop.offline_measurement_start_time = 0;
		context->velocity_loop.offline_speed_start_time = 0;
	} else if (type == OFFLINE_EXPERIMENT_ACCELERATION) {
		context->acceleration.state = OFFLINE_ACCELERATION_STATE_NAVIGATING_TO_C13;
		context->acceleration.speed_pair_count = 0;
		context->acceleration.current_pair_index = 0;
		context->acceleration.c13_to_e7_distance = 0;
		context->acceleration.measurement_start_time = 0;
		context->acceleration.speed_change_time = 0;
		context->acceleration.measurements_per_pair = 3;
		context->acceleration.measurement_count = 0;
	} else if (type == OFFLINE_EXPERIMENT_STOP_DISTANCE) {
		context->stop_distance.state = OFFLINE_STOP_DISTANCE_STATE_NAVIGATING_TO_A3;
		context->stop_distance.speed_count = 0;
		context->stop_distance.current_speed_index = 0;
		context->stop_distance.a3_to_e7_distance = 0;
		context->stop_distance.delay_min = 0;
		context->stop_distance.delay_max = 0;
		context->stop_distance.delay_current = 0;
		context->stop_distance.delay_optimal = 0;
		context->stop_distance.a3_trigger_time = 0;
		context->stop_distance.stop_command_time = 0;
		context->stop_distance.e7_triggered = false;
		context->stop_distance.e7_final_state = false;
		context->stop_distance.search_timeout = 0;
		context->stop_distance.max_search_time = 0;
		context->stop_distance.measurements_per_speed = 3;
		context->stop_distance.current_measurement = 0;
		context->stop_distance.binary_search_iterations = 0;
		context->stop_distance.max_binary_search_iterations = 10;
	}

	return MARKLIN_ERROR_OK;
}

// ############################################################################
// # Offline Experiment Implementation Functions
// ############################################################################

marklin_error_t train_start_offline_experiment(train_task_data_t *data)
{
	if (!data) {
		klog_error("Train %d: Invalid arguments for offline experiment %p", data->train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (data->offline_experiment_active) {
		klog_error("Train %d: Already running an experiment", data->train_id);
		return MARKLIN_ERROR_UNKNOWN; // Already running an experiment
	}

	// Initialize experiment state
	data->offline_experiment_active = true;
	data->online_calibration.enabled = false;

	// Switch to manual mode for experiment
	data->operating_mode = TRAIN_MODE_MANUAL;

	// Set up experiments based on type
	if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
		train_start_offline_velocity_experiment(data);
	} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_ACCELERATION) {
		train_start_offline_acceleration_experiment(data);
	} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_STOP_DISTANCE) {
		train_start_offline_stop_distance_experiment(data);
	}

	klog_info("Train %d: Offline experiment setup completed", data->train_id);

	return MARKLIN_ERROR_OK;
}

void train_update_offline_experiment(train_task_data_t *data)
{
	if (!data || !data->offline_experiment_active) {
		return;
	}

	// Handle different experiment types
	if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
		train_update_offline_velocity_experiment(data);
	} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_ACCELERATION) {
		train_update_offline_acceleration_experiment(data);
	} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_STOP_DISTANCE) {
		train_update_offline_stop_distance_experiment(data);
	}
}

void train_process_offline_sensor_update(train_task_data_t *data, const track_node *sensor_node)
{
	if (!data || !data->offline_experiment_active || !sensor_node) {
		return;
	}

	// Handle different experiment types
	if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
		train_process_offline_velocity_sensor_update(data, sensor_node);
	} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_ACCELERATION) {
		train_process_offline_acceleration_sensor_update(data, sensor_node);
	} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_STOP_DISTANCE) {
		train_process_offline_stop_distance_sensor_update(data, sensor_node);
	}
}

kinematic_distance_t train_offline_calculate_path_distance(train_task_data_t *data, const track_node *from,
							   const track_node *to)
{
	if (!data || !from || !to) {
		return 0;
	}

	path_result_t path_result;
	marklin_error_t path_error = Marklin_FindPath(from, to, data->train_id, false, false, NULL, 0, &path_result);

	if (path_error != MARKLIN_ERROR_OK) {
		klog_error("Train %d: Failed to find path from %s to %s for distance calculation", data->train_id,
			   from->name, to->name);
		return 0;
	}

	kinematic_distance_t distance = (kinematic_distance_t)path_result.total_distance;
	Marklin_FreePath(&path_result);

	klog_debug("Train %d: Path distance from %s to %s: %lld mm", data->train_id, from->name, to->name, distance);

	return distance;
}

marklin_error_t train_offline_cleanup_experiment(train_task_data_t *data)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	if (!data->offline_experiment_active) {
		return MARKLIN_ERROR_OK;
	}

	// Stop the train
	train_stop(data);

	// Print final summary based on experiment type
	klog_info("========== Offline Experiment Complete ==========");
	if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
		klog_info("Train %d - Velocity Loop Test Summary", data->train_id);
		klog_info("Loop Distance: %lld mm (B5->C11->B5)",
			  data->offline_experiment_context.velocity_loop.offline_loop_distance);
		klog_info("Speed Levels Tested: %d levels",
			  data->offline_experiment_context.velocity_loop.offline_speed_count);
		for (u8 i = 0; i < data->offline_experiment_context.velocity_loop.offline_speed_count; i++) {
			klog_info("  Speed Level %d: %d", i + 1,
				  data->offline_experiment_context.velocity_loop.offline_speed_levels[i]);
		}
	} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_ACCELERATION) {
		klog_info("Train %d - Acceleration Test Summary", data->train_id);
		klog_info("Test Distance: %lld mm (C13->E7)",
			  data->offline_experiment_context.acceleration.c13_to_e7_distance);
		klog_info("Speed Pairs Tested: %d pairs",
			  data->offline_experiment_context.acceleration.speed_pair_count);
		for (u8 i = 0; i < data->offline_experiment_context.acceleration.speed_pair_count; i++) {
			klog_info("  Pair %d: %d -> %d", i + 1,
				  data->offline_experiment_context.acceleration.from_speed_levels[i],
				  data->offline_experiment_context.acceleration.to_speed_levels[i]);
		}
	} else if (data->offline_experiment_context.type == OFFLINE_EXPERIMENT_STOP_DISTANCE) {
		klog_info("Train %d - Stop Distance Test Summary", data->train_id);
		klog_info("Test Distance: %lld mm (A3->E7)",
			  data->offline_experiment_context.stop_distance.a3_to_e7_distance);
		klog_info("Speed Levels Tested: %d levels", data->offline_experiment_context.stop_distance.speed_count);
		for (u8 i = 0; i < data->offline_experiment_context.stop_distance.speed_count; i++) {
			klog_info("  Speed %d: stop distance %lld mm (delay: %lld ticks)",
				  data->offline_experiment_context.stop_distance.speed_levels[i],
				  data->offline_experiment_context.stop_distance.measured_stop_distances[i],
				  data->offline_experiment_context.stop_distance.measured_stop_delays[i]);
		}
	}
	klog_info("=================================================");

	kinematic_model_print_defaults(data);

	// Reset experiment state
	data->offline_experiment_active = false;
	memset(&data->offline_experiment_context, 0, sizeof(offline_experiment_context_t));
	data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_NAVIGATING_TO_B5;
	data->offline_experiment_context.velocity_loop.offline_b5_cycle_count = 0;
	data->offline_experiment_context.velocity_loop.offline_loop_distance = 0;
	data->offline_experiment_context.velocity_loop.offline_current_speed_index = 0;
	data->offline_experiment_context.velocity_loop.offline_speed_levels[0] = 1;
	data->offline_experiment_context.velocity_loop.offline_speed_count = 1;
	data->offline_experiment_context.velocity_loop.offline_measurement_start_time = 0;
	data->offline_experiment_context.velocity_loop.offline_speed_start_time = 0;

	klog_info("Train %d: Offline experiment cleanup completed", data->train_id);

	return MARKLIN_ERROR_OK;
}
