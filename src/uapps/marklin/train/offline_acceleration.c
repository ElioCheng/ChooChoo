#include "clock.h"
#include "marklin/conductor/api.h"
#include "marklin/error.h"
#include "marklin/topology/track.h"
#include "marklin/train/api.h"
#include "marklin/train/kinematics.h"
#include "marklin/train/offline.h"
#include "marklin/train/train.h"
#include "marklin/train/model.h"

#define LOG_MODULE "offline_acceleration"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

extern const track_node *track_nodes;
extern int track_nodes_size;

// Helper function to set up speed level with proper transition
static void __offline_acceleration_setup_speed_level(train_task_data_t *data, u8 speed_level, bool from_higher_speed)
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

// Calculate acceleration using the two-scenario algorithm
static kinematic_accel_t __calculate_acceleration(kinematic_velocity_t v1, kinematic_velocity_t v2,
						  kinematic_distance_t distance, kinematic_time_t time)
{
	if (time == 0) {
		return 0;
	}

	// Calculate average velocity during acceleration: va = (v1 + v2) / 2
	kinematic_velocity_t va = kinematic_average_velocity(v1, v2);

	// Calculate observed velocity: d/t
	kinematic_velocity_t observed_velocity = kinematic_velocity(distance, time);

	// Determine which scenario we're in
	if (observed_velocity > va) {
		// Scenario 1: Acceleration complete before sensor hit (d/t > va)
		// Split distance into acceleration (d1) and constant velocity (d2) segments

		// From equations:
		// va = d1 / t1  and  v2 = d2 / t2  and  d = d1 + d2  and  t = t1 + t2
		// We can solve for t1:
		// t1 = (2 * d * SCALE_FACTOR - v2 * t) / (v2 - v1)

		// elio: t1 = (2 * d * SCALE_FACTOR - 2 * v2 * t) / (v1 - v2)
		// Keep everything in scaled units to maintain precision

		// Calculate numerator: 2 * d * SCALE_FACTOR - v2 * t
		// Note: v2 is already scaled, so v2 * t gives us scaled_velocity * time
		i64 scaled_2d = kinematic_safe_multiply(2 * distance, KINEMATIC_VELOCITY_SCALE_FACTOR);
		i64 v2_times_2t = kinematic_safe_multiply(2 * v2, time);
		i64 numerator = scaled_2d - v2_times_2t;

		// Calculate denominator: v2 - v1 (both already scaled)
		i64 denominator = v1 - v2;

		// Calculate t1 = numerator / denominator
		kinematic_time_t t1 = (denominator != 0) ? (numerator / denominator) : 0;

		// Calculate acceleration: (v2 - v1) / t1
		return kinematic_acceleration(v1, v2, t1);
	} else {
		// Scenario 2: Acceleration not complete before sensor hit (d/t <= va)
		// Calculate velocity at sensor hit: vr = vs + (vs - v1)
		// where vs = observed_velocity = d/t

		kinematic_velocity_t vr = observed_velocity + (observed_velocity - v1);

		// Calculate acceleration: (vr - v1) / t
		return kinematic_acceleration(v1, vr, time);
	}
}

marklin_error_t train_start_offline_acceleration_experiment(train_task_data_t *data)
{
	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_ACCELERATION) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_NAVIGATING_TO_C13;
	data->offline_experiment_context.acceleration.current_pair_index = 0;
	data->offline_experiment_context.acceleration.measurement_count = 0;

	// Speed pairs are already set by train_handle_offline_experiment_command
	// Just validate we have at least one speed pair
	if (data->offline_experiment_context.acceleration.speed_pair_count == 0) {
		log_error("Train %d: No speed pairs configured for acceleration experiment", data->train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}
	data->offline_experiment_context.acceleration.measurements_per_pair = 3;

	log_info("Train %d: Starting acceleration experiment with %d speed pairs", data->train_id,
		 data->offline_experiment_context.acceleration.speed_pair_count);

	// Navigate to C13 to start the experiment
	train_navigate_to_destination(data, "E7", 10);

	return MARKLIN_ERROR_OK;
}

void train_update_offline_acceleration_experiment(train_task_data_t *data)
{
	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_ACCELERATION) {
		return;
	}

	const track_node *c13_node = NULL;
	const track_node *e7_node = NULL;
	const track_node *c16_node = NULL;
	path_result_t path_result;
	marklin_path_activation_result_t activation_result;

	switch (data->offline_experiment_context.acceleration.state) {
	case OFFLINE_ACCELERATION_STATE_NAVIGATING_TO_C13:
		return;

	case OFFLINE_ACCELERATION_STATE_AT_C13_PREPARING:
		log_info("Train %d: At C13, preparing for acceleration experiment", data->train_id);

		// Find C13 and E7 nodes
		c13_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "C13");
		e7_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "E7");

		if (!c13_node || !e7_node) {
			data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_DONE;
			log_error("Train %d: Cannot find C13 or E7 sensors", data->train_id);
			return;
		}

		// Calculate distance from C13 to E7
		data->offline_experiment_context.acceleration.c13_to_e7_distance =
			train_offline_calculate_path_distance(data, c13_node, e7_node);

		log_info("Train %d: C13->E7 distance calculated: %lld mm", data->train_id,
			 data->offline_experiment_context.acceleration.c13_to_e7_distance);

		data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_SETTING_PATH;
		break;

	case OFFLINE_ACCELERATION_STATE_SETTING_PATH:
		// Set up the B5->D3 path
		c13_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "C13");
		c16_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "C16");
		e7_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "E7");

		log_info("Train %d: Setting up E7->C16 path", data->train_id);

		if (Marklin_FindPath(e7_node, c16_node, data->train_id, false, false, &path_result) !=
		    MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to find E7->C16 path", data->train_id);
			data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_DONE;
			return;
		}

		if (Marklin_ActivatePath(&path_result, data->train_id, 0, 0, NULL, 0, &activation_result) != MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to activate E7->C16 path", data->train_id);
			Marklin_FreePath(&path_result);
			data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_DONE;
			return;
		}
		Marklin_FreePath(&path_result);

		if (Marklin_FindPath(c16_node, e7_node, data->train_id, false, false, &path_result) !=
		    MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to find C16->E7 path", data->train_id);
			data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_DONE;
			return;
		}

		if (Marklin_ActivatePath(&path_result, data->train_id, 0, 0, NULL, 0, &activation_result) != MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to activate C16->E7 path", data->train_id);
			Marklin_FreePath(&path_result);
			data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_DONE;
			return;
		}
		Marklin_FreePath(&path_result);

		train_switch_to_mode(data, TRAIN_MODE_MANUAL);
		data->destination = NULL;
		data->destination_offset_mm = 0;

		// Start with first speed pair
		data->offline_experiment_context.acceleration.current_pair_index = 0;
		data->offline_experiment_context.acceleration.measurement_count = 0;
		u8 current_pair = data->offline_experiment_context.acceleration.current_pair_index;
		bool from_higher_speed = false;
		u8 from_speed = kinematic_index_to_speed(
			data->offline_experiment_context.acceleration.from_speed_levels[current_pair],
			&from_higher_speed);

		log_info("Train %d: Setting initial speed %d for pair %d", data->train_id, from_speed, current_pair);

		// Set to initial speed
		__offline_acceleration_setup_speed_level(data, from_speed, from_higher_speed);
		data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_READY_TO_MEASURE;
		break;

	case OFFLINE_ACCELERATION_STATE_READY_TO_MEASURE:
		// Set initial speed and prepare for measurement
		{
		}
		break;

	case OFFLINE_ACCELERATION_STATE_MEASURING:
		// Measurement in progress - wait for sensor updates
		break;

	case OFFLINE_ACCELERATION_STATE_SPEED_PAIR_COMPLETE:
		// Current speed pair complete, move to next pair or finish
		data->offline_experiment_context.acceleration.current_pair_index++;

		if (data->offline_experiment_context.acceleration.current_pair_index <
		    data->offline_experiment_context.acceleration.speed_pair_count) {
			// Move to next speed pair
			data->offline_experiment_context.acceleration.measurement_count = 0;
			data->offline_experiment_context.acceleration.state =
				OFFLINE_ACCELERATION_STATE_READY_TO_MEASURE;

			log_info("Train %d: Moving to next speed pair (%d/%d)", data->train_id,
				 data->offline_experiment_context.acceleration.current_pair_index + 1,
				 data->offline_experiment_context.acceleration.speed_pair_count);
		} else {
			// All speed pairs complete
			data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_DONE;
			train_offline_cleanup_experiment(data);
			log_info("Train %d: Done with acceleration experiment, navigating back to A7", data->train_id);
			train_navigate_to_destination(data, "A7", 10);
		}
		break;

	case OFFLINE_ACCELERATION_STATE_DONE:
		train_offline_cleanup_experiment(data);

		return;
	}
}

void train_process_offline_acceleration_sensor_update(train_task_data_t *data, const track_node *sensor_node)
{
	if (!data || !data->offline_experiment_active || !sensor_node) {
		return;
	}

	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_ACCELERATION) {
		return;
	}

	// Handle B5 sensor - start of acceleration measurement
	if (strcmp(sensor_node->name, "C13") == 0) {
		switch (data->offline_experiment_context.acceleration.state) {
		case OFFLINE_ACCELERATION_STATE_NAVIGATING_TO_C13:
			// Arrived at B5
			data->offline_experiment_context.acceleration.state =
				OFFLINE_ACCELERATION_STATE_AT_C13_PREPARING;
			train_stop(data);
			log_info("Train %d: Arrived at C13 for acceleration experiment", data->train_id);
			break;

		case OFFLINE_ACCELERATION_STATE_MEASURING: {
			// Change speed and start timing
			u8 current_pair = data->offline_experiment_context.acceleration.current_pair_index;
			bool to_higher_speed = false;
			u8 to_speed = kinematic_index_to_speed(
				data->offline_experiment_context.acceleration.to_speed_levels[current_pair],
				&to_higher_speed);

			// Record speed change time and change to target speed
			data->offline_experiment_context.acceleration.speed_change_time = Time(data->clock_server_tid);
			__offline_acceleration_setup_speed_level(data, to_speed, to_higher_speed);

			log_info("Train %d: Speed changed from %d to %d at C13, measuring acceleration", data->train_id,
				 data->offline_experiment_context.acceleration.from_speed_levels[current_pair],
				 to_speed);
		} break;

		default:
			break;
		}
	}

	// Handle D3 sensor - end of acceleration measurement
	else if (strcmp(sensor_node->name, "E7") == 0) {
		if (data->offline_experiment_context.acceleration.state == OFFLINE_ACCELERATION_STATE_MEASURING) {
			// Calculate acceleration measurement
			kinematic_time_t measurement_end = Time(data->clock_server_tid);
			kinematic_time_t travel_time =
				measurement_end - data->offline_experiment_context.acceleration.speed_change_time;

			u8 current_pair = data->offline_experiment_context.acceleration.current_pair_index;
			u8 from_speed = data->offline_experiment_context.acceleration.from_speed_levels[current_pair];
			bool from_higher_speed = false;
			int from_speed_level = kinematic_index_to_speed(
				data->offline_experiment_context.acceleration.from_speed_levels
					[data->offline_experiment_context.acceleration.current_pair_index],
				&from_higher_speed);

			__offline_acceleration_setup_speed_level(data, from_speed_level, from_higher_speed);

			// Get velocities from kinematic model
			kinematic_velocity_t v1 =
				kinematic_model_get_velocity(data, from_speed_level, from_higher_speed);

			u8 to_speed = data->offline_experiment_context.acceleration.to_speed_levels[current_pair];
			int to_speed_level = kinematic_index_to_speed(
				data->offline_experiment_context.acceleration.to_speed_levels[current_pair],
				&from_higher_speed);
			kinematic_velocity_t v2 = kinematic_model_get_velocity(data, to_speed_level, from_higher_speed);

			if (v1 == 0 || v2 == 0) {
				log_warn("Train %d: Missing velocity data for speed %d->%d, skipping measurement",
					 data->train_id, from_speed, to_speed);
			} else {
				// Calculate acceleration using our algorithm
				kinematic_accel_t measured_acceleration = __calculate_acceleration(
					v1, v2, data->offline_experiment_context.acceleration.c13_to_e7_distance,
					travel_time);

				// Store measurement
				if (data->offline_experiment_context.acceleration.measurement_count <
				    MAX_ACCELERATION_MEASUREMENTS) {
					data->offline_experiment_context.acceleration.measured_accelerations
						[data->offline_experiment_context.acceleration.measurement_count] =
						measured_acceleration;
					data->offline_experiment_context.acceleration.measurement_count++;
				}

				// Update kinematic model
				marklin_error_t update_result = kinematic_model_update_acceleration(
					data, from_speed, to_speed, measured_acceleration);

				// Log results
				i64 accel_int, accel_frac;
				kinematic_accel_split(measured_acceleration, &accel_int, &accel_frac);

				if (update_result == MARKLIN_ERROR_OK) {
					log_info(
						"Train %d: Acceleration %d->%d: %lld.%08lld mm/tick² (time: %lld ticks, distance: %lld mm)",
						data->train_id, from_speed, to_speed, accel_int, accel_frac,
						travel_time,
						data->offline_experiment_context.acceleration.c13_to_e7_distance);
				} else {
					log_error("Train %d: Failed to update acceleration model for %d->%d",
						  data->train_id, from_speed, to_speed);
				}
			}

			// Check if we need more measurements for this pair
			if (data->offline_experiment_context.acceleration.measurement_count >=
			    data->offline_experiment_context.acceleration.measurements_per_pair) {
				// Done with this speed pair
				data->offline_experiment_context.acceleration.state =
					OFFLINE_ACCELERATION_STATE_SPEED_PAIR_COMPLETE;
			} else {
				// Need more measurements - go back to C13
				log_info("Train %d: Need %d more measurements for speed pair %d->%d", data->train_id,
					 data->offline_experiment_context.acceleration.measurements_per_pair -
						 data->offline_experiment_context.acceleration.measurement_count,
					 from_speed, to_speed);
				// Continue measuring (will loop back via path system)
			}
		}
	} else if (strcmp(sensor_node->name, "B15") == 0) {
		if (data->offline_experiment_context.acceleration.state ==
		    OFFLINE_ACCELERATION_STATE_READY_TO_MEASURE) {
			data->offline_experiment_context.acceleration.state = OFFLINE_ACCELERATION_STATE_MEASURING;
		}
	}
}
