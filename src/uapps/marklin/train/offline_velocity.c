#include "clock.h"
#include "marklin/conductor/api.h"
#include "marklin/error.h"
#include "marklin/topology/track.h"
#include "marklin/train/api.h"
#include "marklin/train/offline.h"
#include "marklin/train/train.h"

#define LOG_MODULE "offline_velocity"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

extern const track_node *track_nodes;
extern int track_nodes_size;

static void __offline_velocity_setup_speed_level(train_task_data_t *data, u8 speed_level, bool from_higher_speed)
{
	if (speed_level == 0 || speed_level >= 14) {
		train_set_speed(data, speed_level);
	} else {
		if (from_higher_speed) {
			train_set_speed_and_headlight(data, speed_level + 1, MARKLIN_TRAIN_HEADLIGHT_ON);
			train_set_speed_and_headlight(data, speed_level, MARKLIN_TRAIN_HEADLIGHT_ON);
		} else {
			train_set_speed_and_headlight(data, speed_level - 1, MARKLIN_TRAIN_HEADLIGHT_ON);
			train_set_speed_and_headlight(data, speed_level, MARKLIN_TRAIN_HEADLIGHT_ON);
		}
	}
}

marklin_error_t train_start_offline_velocity_experiment(train_task_data_t *data)
{
	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_NAVIGATING_TO_B5;
	data->offline_experiment_context.velocity_loop.offline_current_speed_index = 0;

	if (data->offline_experiment_context.velocity_loop.offline_speed_count == 0) {
		log_error("Train %d: No speed levels configured for velocity experiment", data->train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	log_info("Train %d: Starting offline velocity loop experiment, navigating to B5", data->train_id);
	train_navigate_to_destination(data, "B5", 10);

	return MARKLIN_ERROR_OK;
}

void train_update_offline_velocity_experiment(train_task_data_t *data)
{
	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
		return;
	}

	const track_node *b5_node = NULL;
	const track_node *c11_node = NULL;
	path_result_t path_result;
	marklin_path_activation_result_t activation_result;

	switch (data->offline_experiment_context.velocity_loop.state) {
	case OFFLINE_VELOCITY_LOOP_STATE_NAVIGATING_TO_B5:
		return;

	case OFFLINE_VELOCITY_LOOP_STATE_AT_B5_PREPARING:
		log_info("Train %d: At B5, preparing for experiment", data->train_id);

		// Find B5 and C11 nodes
		b5_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "B5");
		c11_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "C11");

		if (!b5_node || !c11_node) {
			data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_DONE;
			log_error("Train %d: Cannot find B5 or C11 sensors", data->train_id);
			return;
		}

		// Calculate loop distance: B5->C11 + C11->B5
		kinematic_distance_t b5_to_c11 = train_offline_calculate_path_distance(data, b5_node, c11_node);
		kinematic_distance_t c11_to_b5 = train_offline_calculate_path_distance(data, c11_node, b5_node);
		data->offline_experiment_context.velocity_loop.offline_loop_distance = b5_to_c11 + c11_to_b5;

		log_info("Train %d: Loop distance calculated: B5->C11=%lld mm, C11->B5=%lld mm, Total=%lld mm",
			 data->train_id, b5_to_c11, c11_to_b5,
			 data->offline_experiment_context.velocity_loop.offline_loop_distance);

		data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_SETTING_PATH;
		break;

	case OFFLINE_VELOCITY_LOOP_STATE_SETTING_PATH:
		// Set up the B5->C11->B5 loop path
		b5_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "B5");
		c11_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "C11");
		log_info("Train %d: Setting up path", data->train_id);
		// First path: B5->C11
		if (Marklin_FindPath(b5_node, c11_node, data->train_id, false, false, &path_result) !=
		    MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to find B5->C11 path", data->train_id);
			data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_DONE;
			return;
		}

		if (Marklin_ActivatePath(&path_result, data->train_id, 0, 0, NULL, 0, &activation_result) != MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to activate B5->C11 path", data->train_id);
			Marklin_FreePath(&path_result);
			data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_DONE;
			return;
		}
		Marklin_FreePath(&path_result);

		// Second path: C11->B5 (this sets up switches for the return path)
		if (Marklin_FindPath(c11_node, b5_node, data->train_id, false, false, &path_result) !=
		    MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to find C11->B5 path", data->train_id);
			data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_DONE;
			return;
		}

		if (Marklin_ActivatePath(&path_result, data->train_id, 0, 0, NULL, 0, &activation_result) != MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to activate C11->B5 path", data->train_id);
			Marklin_FreePath(&path_result);
			data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_DONE;
			return;
		}
		Marklin_FreePath(&path_result);

		log_info("Train %d: B5->C11->B5 loop path activated", data->train_id);
		train_switch_to_mode(data, TRAIN_MODE_MANUAL);

		bool from_higher_speed = false;
		int speed_level = kinematic_index_to_speed(
			data->offline_experiment_context.velocity_loop.offline_speed_levels[0], &from_higher_speed);

		// Start dry run at first speed
		data->offline_experiment_context.velocity_loop.offline_current_speed_index = 0;
		__offline_velocity_setup_speed_level(data, speed_level, from_higher_speed);
		log_info("Train %d: Starting dry run at speed %d", data->train_id, speed_level);
		data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_DRY_RUN;
		break;

	case OFFLINE_VELOCITY_LOOP_STATE_DRY_RUN:
		// Waiting for dry run to complete (one full loop)
		break;

	case OFFLINE_VELOCITY_LOOP_STATE_WAITING_FOR_B5:
		// Waiting to arrive back at B5 to start measurement
		break;

	case OFFLINE_VELOCITY_LOOP_STATE_READY_TO_MEASURE:
		// Start measurement for current speed
		data->offline_experiment_context.velocity_loop.offline_b5_cycle_count = 0;
		data->offline_experiment_context.velocity_loop.offline_speed_start_time = Time(data->clock_server_tid);
		data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_MEASURING;
		log_info("Train %d: Starting measurement for speed %d", data->train_id,
			 data->offline_experiment_context.velocity_loop.offline_speed_levels
				 [data->offline_experiment_context.velocity_loop.offline_current_speed_index]);
		break;

	case OFFLINE_VELOCITY_LOOP_STATE_MEASURING:
		// Measurement in progress
		break;

	case OFFLINE_VELOCITY_LOOP_STATE_SPEED_COMPLETE:
		// Current speed measurement complete, move to next speed
		data->offline_experiment_context.velocity_loop.offline_current_speed_index++;
		if (data->offline_experiment_context.velocity_loop.offline_current_speed_index <
		    data->offline_experiment_context.velocity_loop.offline_speed_count) {
			bool from_higher_speed;
			int speed_level = kinematic_index_to_speed(
				data->offline_experiment_context.velocity_loop.offline_speed_levels
					[data->offline_experiment_context.velocity_loop.offline_current_speed_index],
				&from_higher_speed);
			// Set next speed and wait for B5
			__offline_velocity_setup_speed_level(data, speed_level, from_higher_speed);
			data->offline_experiment_context.velocity_loop.state =
				OFFLINE_VELOCITY_LOOP_STATE_WAITING_FOR_B5;
			log_info("Train %d: Moving to next speed level: %d", data->train_id,
				 data->offline_experiment_context.velocity_loop.offline_speed_levels
					 [data->offline_experiment_context.velocity_loop.offline_current_speed_index]);
		} else {
			// All speeds complete
			data->offline_experiment_context.velocity_loop.state = OFFLINE_VELOCITY_LOOP_STATE_DONE;
			train_offline_cleanup_experiment(data);
			train_navigate_to_destination(data, "A6", 10);
		}
		break;

	case OFFLINE_VELOCITY_LOOP_STATE_TRANSITION_COMPLETE:
		// Transition complete, continue with next state
		break;

	case OFFLINE_VELOCITY_LOOP_STATE_DONE:
		train_offline_cleanup_experiment(data);
		train_navigate_to_destination(data, "A6", 10);
		return;
	}
}

void train_process_offline_velocity_sensor_update(train_task_data_t *data, const track_node *sensor_node)
{
	if (!data || !data->offline_experiment_active || !sensor_node) {
		return;
	}

	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_VELOCITY_LOOP) {
		return;
	}

	// Handle velocity loop experiment
	// Check sensor name
	if (strcmp(sensor_node->name, "B5") == 0) {
		switch (data->offline_experiment_context.velocity_loop.state) {
		case OFFLINE_VELOCITY_LOOP_STATE_NAVIGATING_TO_B5:
			// Arrived at B5
			data->offline_experiment_context.velocity_loop.state =
				OFFLINE_VELOCITY_LOOP_STATE_AT_B5_PREPARING;
			train_stop(data);
			log_info("Train %d: Arrived at B5 for offline experiment", data->train_id);
			break;

		case OFFLINE_VELOCITY_LOOP_STATE_DRY_RUN:
			// Completed dry run, ready to start measurement
			data->offline_experiment_context.velocity_loop.state =
				OFFLINE_VELOCITY_LOOP_STATE_READY_TO_MEASURE;
			log_info("Train %d: Dry run complete, ready to measure", data->train_id);
			break;

		case OFFLINE_VELOCITY_LOOP_STATE_WAITING_FOR_B5:
			// Ready to start next speed measurement
			data->offline_experiment_context.velocity_loop.state =
				OFFLINE_VELOCITY_LOOP_STATE_READY_TO_MEASURE;
			break;

		case OFFLINE_VELOCITY_LOOP_STATE_MEASURING:
			data->offline_experiment_context.velocity_loop.offline_b5_cycle_count++;
			// Check if we've completed enough cycles
			const u32 cycles_per_speed = 5;
			bool from_higher_speed = false;
			int speed_level = kinematic_index_to_speed(
				data->offline_experiment_context.velocity_loop.offline_speed_levels
					[data->offline_experiment_context.velocity_loop.offline_current_speed_index],
				&from_higher_speed);

			if (data->offline_experiment_context.velocity_loop.offline_b5_cycle_count >=
			    (i32)cycles_per_speed) {
				// Calculate measurement for this speed
				kinematic_time_t measurement_end = Time(data->clock_server_tid);
				kinematic_time_t total_time =
					measurement_end -
					data->offline_experiment_context.velocity_loop.offline_speed_start_time;

				// Use calculated loop distance
				kinematic_distance_t total_distance =
					data->offline_experiment_context.velocity_loop.offline_loop_distance *
					data->offline_experiment_context.velocity_loop.offline_b5_cycle_count;
				kinematic_velocity_t measured_velocity = kinematic_velocity(total_distance, total_time);

				// Update kinematic model
				kinematic_measurement_t measurement = { .timestamp = measurement_end,
									.distance = total_distance,
									.travel_time = total_time,
									.velocity = measured_velocity,
									.speed_level = speed_level,
									.from_higher_speed = from_higher_speed,
									.start_sensor = sensor_node,
									.end_sensor = sensor_node };

				marklin_error_t update_result = kinematic_model_update_velocity(
					data, speed_level, from_higher_speed, &measurement);

				if (update_result == MARKLIN_ERROR_OK) {
					// Split velocity into integer and fractional parts for display
					i64 velocity_int, velocity_frac;
					kinematic_velocity_split(measured_velocity, &velocity_int, &velocity_frac);

					log_info(
						"Train %d: Speed %d measurement: %d cycles, %lld mm total, %lld ticks, velocity=%lld.%06lld mm/tick",
						data->train_id,
						data->offline_experiment_context.velocity_loop.offline_speed_levels
							[data->offline_experiment_context.velocity_loop
								 .offline_current_speed_index],
						data->offline_experiment_context.velocity_loop.offline_b5_cycle_count,
						total_distance, total_time, velocity_int, velocity_frac);

					// Print result to console
					log_info("==== Train %d - Speed Level %d Results ====", data->train_id,
						 data->offline_experiment_context.velocity_loop.offline_speed_levels
							 [data->offline_experiment_context.velocity_loop
								  .offline_current_speed_index]);
					log_info("  Cycles: %d",
						 data->offline_experiment_context.velocity_loop.offline_b5_cycle_count);
					log_info("  Loop Distance: %lld mm",
						 data->offline_experiment_context.velocity_loop.offline_loop_distance);
					log_info("  Total Distance: %lld mm", total_distance);
					log_info("  Total Time: %lld ticks (%u ms)", total_time,
						 kinematic_ticks_to_ms(total_time));
					log_info("  Average Velocity: %lld.%06lld mm/tick", velocity_int,
						 velocity_frac);
					// Calculate mm/s using the new conversion function
					i64 velocity_mm_s = kinematic_velocity_to_mm_per_second(measured_velocity);
					log_info("  Average Speed: %lld mm/s", velocity_mm_s);
				} else {
					log_error("Train %d: Failed to update kinematic model for speed %d",
						  data->train_id,
						  data->offline_experiment_context.velocity_loop.offline_speed_levels
							  [data->offline_experiment_context.velocity_loop
								   .offline_current_speed_index]);
				}

				// Move to next speed
				data->offline_experiment_context.velocity_loop.state =
					OFFLINE_VELOCITY_LOOP_STATE_SPEED_COMPLETE;
			}
			break;

		default:
			break;
		}
	}
}
