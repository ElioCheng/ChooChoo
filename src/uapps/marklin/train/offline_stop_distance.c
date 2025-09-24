#include "clock.h"
#include "marklin/conductor/api.h"
#include "marklin/error.h"
#include "marklin/topology/track.h"
#include "marklin/train/api.h"
#include "marklin/train/kinematics.h"
#include "marklin/train/offline.h"
#include "marklin/train/train.h"
#include "marklin/train/model.h"
#include "stdbool.h"

#define LOG_MODULE "offline_stop_distance"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

extern const track_node *track_nodes;
extern int track_nodes_size;

// Binary search precision threshold (in ticks)
#define STOP_DISTANCE_SEARCH_PRECISION 1 // 10ms precision
#define STOP_DISTANCE_MAX_ITERATIONS 15 // Maximum binary search iterations
#define STOP_DISTANCE_TIMEOUT_TICKS 600 // 6 second timeout per iteration

// Helper function to set up speed level with proper transition
static void __offline_stop_distance_setup_speed_level(train_task_data_t *data, u8 speed_level, bool from_higher_speed)
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

// Calculate stop distance from optimal delay
static kinematic_distance_t __calculate_stop_distance(train_task_data_t *data, kinematic_time_t optimal_delay)
{
	bool from_higher_speed = false;
	u8 current_speed_level = kinematic_index_to_speed(
		data->offline_experiment_context.stop_distance
			.speed_levels[data->offline_experiment_context.stop_distance.current_speed_index],
		&from_higher_speed);

	// Get velocity from kinematic model
	kinematic_velocity_t velocity = kinematic_model_get_velocity(data, current_speed_level, from_higher_speed);
	if (velocity == 0) {
		log_warn("Train %d: No velocity data for speed %d", data->train_id, current_speed_level);
		return 0;
	}

	// Calculate distance traveled during delay before stop command
	// Use scaled arithmetic: distance = (velocity * optimal_delay) / SCALE_FACTOR
	// Keep intermediate result scaled for maximum precision
	i64 scaled_distance_before_stop = kinematic_safe_multiply(velocity, optimal_delay);
	kinematic_distance_t distance_before_stop = scaled_distance_before_stop / KINEMATIC_VELOCITY_SCALE_FACTOR;

	// Stop distance = total path distance - distance traveled before stop
	kinematic_distance_t total_distance = data->offline_experiment_context.stop_distance.a3_to_e7_distance;
	kinematic_distance_t stop_distance = total_distance - distance_before_stop;

	return stop_distance;
}

// Initialize binary search parameters for a new speed level
static void __init_binary_search(train_task_data_t *data)
{
	bool from_higher_speed = false;
	u8 current_speed_level = kinematic_index_to_speed(
		data->offline_experiment_context.stop_distance
			.speed_levels[data->offline_experiment_context.stop_distance.current_speed_index],
		&from_higher_speed);

	// Estimate travel time from A3 to E7 at current speed
	kinematic_velocity_t velocity = kinematic_model_get_velocity(data, current_speed_level, from_higher_speed);
	kinematic_time_t estimated_travel_time = 0;

	if (velocity > 0) {
		// Use scaled arithmetic: time = (distance * SCALE_FACTOR) / velocity
		// Keep distance scaled to maintain precision in division
		i64 scaled_distance =
			kinematic_safe_multiply(data->offline_experiment_context.stop_distance.a3_to_e7_distance,
						KINEMATIC_VELOCITY_SCALE_FACTOR);
		estimated_travel_time = scaled_distance / velocity;
	} else {
		// Fallback estimate: assume ~3mm/tick velocity
		estimated_travel_time = data->offline_experiment_context.stop_distance.a3_to_e7_distance / 3;
	}

	// Set search bounds
	data->offline_experiment_context.stop_distance.delay_min = 0; // Too early
	data->offline_experiment_context.stop_distance.delay_max = estimated_travel_time; // Too late
	data->offline_experiment_context.stop_distance.delay_current =
		(data->offline_experiment_context.stop_distance.delay_min +
		 data->offline_experiment_context.stop_distance.delay_max) /
		2;

	data->offline_experiment_context.stop_distance.binary_search_iterations = 0;
	data->offline_experiment_context.stop_distance.max_binary_search_iterations = STOP_DISTANCE_MAX_ITERATIONS;

	log_info("Train %d: Initialized binary search for speed %d - delay range [%lld, %lld, %lld] ticks",
		 data->train_id, current_speed_level, data->offline_experiment_context.stop_distance.delay_min,
		 data->offline_experiment_context.stop_distance.delay_current,
		 data->offline_experiment_context.stop_distance.delay_max);
}

// Process binary search result and update search bounds
static bool __update_binary_search(train_task_data_t *data, bool stop_too_early, bool stop_perfect)
{
	if (stop_perfect) {
		// Found optimal delay
		log_info("Train %d: Binary search converged to optimal delay", data->train_id);
		data->offline_experiment_context.stop_distance.delay_optimal =
			data->offline_experiment_context.stop_distance.delay_current;
		return true; // Search complete
	}

	// Update search bounds
	if (stop_too_early) {
		data->offline_experiment_context.stop_distance.delay_min =
			data->offline_experiment_context.stop_distance.delay_current;
	} else {
		// stop_too_late
		data->offline_experiment_context.stop_distance.delay_max =
			data->offline_experiment_context.stop_distance.delay_current;
	}

	// Calculate new delay
	kinematic_time_t new_delay = (data->offline_experiment_context.stop_distance.delay_min +
				      data->offline_experiment_context.stop_distance.delay_max) /
				     2;

	// Check if we've reached precision limit
	kinematic_time_t search_range = data->offline_experiment_context.stop_distance.delay_max -
					data->offline_experiment_context.stop_distance.delay_min;

	if (search_range <= STOP_DISTANCE_SEARCH_PRECISION) {
		// Use current best estimate as optimal
		data->offline_experiment_context.stop_distance.delay_optimal = new_delay;
		log_info("Train %d: Binary search converged within precision limit", data->train_id);
		return true; // Search complete
	}

	// Check iteration limit
	data->offline_experiment_context.stop_distance.binary_search_iterations++;
	if (data->offline_experiment_context.stop_distance.binary_search_iterations >=
	    data->offline_experiment_context.stop_distance.max_binary_search_iterations) {
		data->offline_experiment_context.stop_distance.delay_optimal = new_delay;
		log_warn("Train %d: Binary search reached iteration limit", data->train_id);
		return true; // Search complete
	}

	data->offline_experiment_context.stop_distance.delay_current = new_delay;

	log_info("Train %d: Binary search iteration %d - delay range [%lld, %lld, %lld] ticks", data->train_id,
		 data->offline_experiment_context.stop_distance.binary_search_iterations,
		 data->offline_experiment_context.stop_distance.delay_min, new_delay,
		 data->offline_experiment_context.stop_distance.delay_max);

	return false; // Continue search_d
}

marklin_error_t train_start_offline_stop_distance_experiment(train_task_data_t *data)
{
	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_STOP_DISTANCE) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	data->offline_experiment_context.stop_distance.state = OFFLINE_STOP_DISTANCE_STATE_NAVIGATING_TO_A3;
	data->offline_experiment_context.stop_distance.current_speed_index = 0;
	data->offline_experiment_context.stop_distance.current_measurement = 0;

	if (data->offline_experiment_context.stop_distance.speed_count == 0) {
		log_error("Train %d: No speed levels configured for stop distance experiment", data->train_id);
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	data->offline_experiment_context.stop_distance.measurements_per_speed = MAX_STOP_DISTANCE_MEASUREMENTS;

	// Initialize measurement arrays
	for (u8 i = 0; i < data->offline_experiment_context.stop_distance.speed_count; i++) {
		data->offline_experiment_context.stop_distance.measured_stop_delays[i] = 0;
		data->offline_experiment_context.stop_distance.measured_stop_distances[i] = 0;
	}

	log_info("Train %d: Starting stop distance experiment with %d speed levels", data->train_id,
		 data->offline_experiment_context.stop_distance.speed_count);

	// Navigate to A3 to start the experiment
	train_navigate_to_destination(data, "C13", 10);

	return MARKLIN_ERROR_OK;
}

void train_update_offline_stop_distance_experiment(train_task_data_t *data)
{
	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_STOP_DISTANCE) {
		return;
	}

	const track_node *a3_node = NULL;
	const track_node *e7_node = NULL;
	path_result_t path_result;
	marklin_path_activation_result_t activation_result;

	switch (data->offline_experiment_context.stop_distance.state) {
	case OFFLINE_STOP_DISTANCE_STATE_NAVIGATING_TO_A3:
		return;

	case OFFLINE_STOP_DISTANCE_STATE_AT_A3_PREPARING:
		log_info("Train %d: At A3, preparing for stop distance experiment", data->train_id);

		// Find A3 and E7 nodes
		a3_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "A3");
		e7_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "E7");

		if (!a3_node || !e7_node) {
			data->offline_experiment_context.stop_distance.state = OFFLINE_STOP_DISTANCE_STATE_DONE;
			log_error("Train %d: Cannot find A3 or E7 sensors", data->train_id);
			return;
		}

		// Calculate distance from A3 to E7
		data->offline_experiment_context.stop_distance.a3_to_e7_distance =
			train_offline_calculate_path_distance(data, a3_node, e7_node);

		log_info("Train %d: A3->E7 distance calculated: %lld mm", data->train_id,
			 data->offline_experiment_context.stop_distance.a3_to_e7_distance);

		data->offline_experiment_context.stop_distance.state = OFFLINE_STOP_DISTANCE_STATE_SETTING_PATH;
		break;

	case OFFLINE_STOP_DISTANCE_STATE_SETTING_PATH:
		// Set up the A3->E7 path
		a3_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "A3");
		e7_node = marklin_find_node_by_name(track_nodes, track_nodes_size, "E7");

		log_info("Train %d: Setting up E7->A3 path", data->train_id);

		if (Marklin_FindPath(e7_node, a3_node, data->train_id, false, false, &path_result) !=
		    MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to find E7->A3 path", data->train_id);
			data->offline_experiment_context.stop_distance.state = OFFLINE_STOP_DISTANCE_STATE_DONE;
			return;
		}

		if (Marklin_ActivatePath(&path_result, data->train_id, 0, 0, NULL, 0, &activation_result) != MARKLIN_ERROR_OK) {
			log_error("Train %d: Failed to activate E7->A3 path", data->train_id);
			Marklin_FreePath(&path_result);
			data->offline_experiment_context.stop_distance.state = OFFLINE_STOP_DISTANCE_STATE_DONE;
			return;
		}
		Marklin_FreePath(&path_result);

		log_info("Train %d: E7->A3 path activated", data->train_id);
		train_switch_to_mode(data, TRAIN_MODE_MANUAL);

		// Start with first speed level
		data->offline_experiment_context.stop_distance.current_speed_index = 0;
		data->offline_experiment_context.stop_distance.current_measurement = 0;
		data->offline_experiment_context.stop_distance.state =
			OFFLINE_STOP_DISTANCE_STATE_ACCELERATING_TO_SPEED;
		break;

	case OFFLINE_STOP_DISTANCE_STATE_ACCELERATING_TO_SPEED:
		// Set target speed and prepare for binary search
		{
			bool from_higher_speed = false;
			u8 current_speed_level = kinematic_index_to_speed(
				data->offline_experiment_context.stop_distance
					.speed_levels[data->offline_experiment_context.stop_distance.current_speed_index],
				&from_higher_speed);

			log_info("Train %d: Setting speed to %d for stop distance measurement %d/%d", data->train_id,
				 current_speed_level,
				 data->offline_experiment_context.stop_distance.current_measurement + 1,
				 data->offline_experiment_context.stop_distance.measurements_per_speed);

			// Set to target speed
			__offline_stop_distance_setup_speed_level(data, current_speed_level, from_higher_speed);

			// Initialize binary search parameters
			__init_binary_search(data);

			// Wait for next A3 trigger to start measurement
			data->offline_experiment_context.stop_distance.state =
				OFFLINE_STOP_DISTANCE_STATE_WAITING_FOR_A3;
		}
		break;

	case OFFLINE_STOP_DISTANCE_STATE_WAITING_FOR_A3:
		// Wait for sensor processing to handle A3 trigger
		break;

	case OFFLINE_STOP_DISTANCE_STATE_BINARY_SEARCH:
		// Check if it's time to issue stop command
		{
			kinematic_time_t current_time = Time(data->clock_server_tid);
			kinematic_time_t elapsed_time =
				current_time - data->offline_experiment_context.stop_distance.a3_trigger_time;

			// Check if we should issue stop command
			if (data->offline_experiment_context.stop_distance.stop_command_time == 0 &&
			    elapsed_time >= data->offline_experiment_context.stop_distance.delay_current) {
				// Issue stop command
				data->offline_experiment_context.stop_distance.stop_command_time = current_time;
				train_stop(data);

				u8 current_speed = data->offline_experiment_context.stop_distance
							   .speed_levels[data->offline_experiment_context.stop_distance
										 .current_speed_index];
				log_info("Train %d: Stop command issued for speed %d after %lld tick delay",
					 data->train_id, current_speed, elapsed_time);
			}

			// Check for timeout
			if (current_time - data->offline_experiment_context.stop_distance.search_timeout >
			    STOP_DISTANCE_TIMEOUT_TICKS) {
				marklin_sensor_state_t sensor_states;
				sensor_states.bank = marklin_parse_sensor_bank_from_name("E7");
				sensor_states.sensor_id = marklin_parse_sensor_id_from_name("E7");
				Marklin_GetSensorStates(&sensor_states, 1);
				data->offline_experiment_context.stop_distance.e7_final_state = sensor_states.triggered;

				// Analyze result and update binary search
				bool stop_too_early = !data->offline_experiment_context.stop_distance.e7_triggered;
				bool stop_perfect = data->offline_experiment_context.stop_distance.e7_triggered &&
						    data->offline_experiment_context.stop_distance.e7_final_state;

				log_debug(
					"Train %d: Binary search iteration complete - E7 triggered: %s, final state: %s",
					data->train_id,
					data->offline_experiment_context.stop_distance.e7_triggered ? "yes" : "no",
					data->offline_experiment_context.stop_distance.e7_final_state ? "on" : "off");

				bool search_complete = __update_binary_search(data, stop_too_early, stop_perfect);

				if (search_complete) {
					// Store measurement result
					kinematic_distance_t stop_distance = __calculate_stop_distance(
						data, data->offline_experiment_context.stop_distance.delay_optimal);

					u8 speed_index =
						data->offline_experiment_context.stop_distance.current_speed_index;
					data->offline_experiment_context.stop_distance
						.measured_stop_delays[speed_index] =
						data->offline_experiment_context.stop_distance.delay_optimal;
					data->offline_experiment_context.stop_distance
						.measured_stop_distances[speed_index] = stop_distance;

					// Update kinematic model with both stop distance and stop time
					bool from_higher_speed = false;
					u8 current_speed_level = kinematic_index_to_speed(
						data->offline_experiment_context.stop_distance.speed_levels[speed_index],
						&from_higher_speed);

					// Calculate stop time from delay and kinematic model
					kinematic_velocity_t velocity = kinematic_model_get_velocity(
						data, current_speed_level, from_higher_speed);
					kinematic_time_t stop_time = 0;
					if (velocity > 0) {
						// Estimate stop time: stop_distance / average_velocity
						// Use scaled arithmetic: stop_time = (stop_distance * SCALE_FACTOR * 2) / velocity
						// This avoids early division by 2 and maintains full precision
						i64 scaled_stop_distance = kinematic_safe_multiply(
							stop_distance, KINEMATIC_VELOCITY_SCALE_FACTOR);
						i64 scaled_stop_distance_doubled =
							kinematic_safe_multiply(scaled_stop_distance, 2);
						stop_time = scaled_stop_distance_doubled / velocity;
					}

					marklin_error_t update_result = kinematic_model_update_stopping(
						data,
						data->offline_experiment_context.stop_distance.speed_levels[speed_index],
						stop_distance, stop_time);

					// Log results
					if (update_result == MARKLIN_ERROR_OK) {
						log_info(
							"Train %d: Stop distance for speed %d: %lld mm (delay: %lld ticks)",
							data->train_id,
							data->offline_experiment_context.stop_distance
								.speed_levels[speed_index],
							stop_distance,
							data->offline_experiment_context.stop_distance.delay_optimal);
					} else {
						log_error("Train %d: Failed to update stop distance model for speed %d",
							  data->train_id,
							  data->offline_experiment_context.stop_distance
								  .speed_levels[speed_index]);
					}

					data->offline_experiment_context.stop_distance.state =
						OFFLINE_STOP_DISTANCE_STATE_SPEED_COMPLETE;
				} else {
					// Continue with next iteration - reset for next round
					bool from_higher_speed = false;
					u8 current_speed_level = kinematic_index_to_speed(
						data->offline_experiment_context.stop_distance
							.speed_levels[data->offline_experiment_context.stop_distance
									      .current_speed_index],
						&from_higher_speed);
					__offline_stop_distance_setup_speed_level(data, current_speed_level,
										  from_higher_speed);
					data->offline_experiment_context.stop_distance.stop_command_time = 0;
					data->offline_experiment_context.stop_distance.state =
						OFFLINE_STOP_DISTANCE_STATE_WAITING_FOR_A3;
				}
			}
		}
		break;

	case OFFLINE_STOP_DISTANCE_STATE_SPEED_COMPLETE:
		// Current speed measurement complete, move to next measurement or next speed
		data->offline_experiment_context.stop_distance.current_measurement++;

		if (data->offline_experiment_context.stop_distance.current_measurement <
		    data->offline_experiment_context.stop_distance.measurements_per_speed) {
			// More measurements needed for this speed
			data->offline_experiment_context.stop_distance.state =
				OFFLINE_STOP_DISTANCE_STATE_ACCELERATING_TO_SPEED;
		} else {
			// Move to next speed level
			data->offline_experiment_context.stop_distance.current_speed_index++;
			data->offline_experiment_context.stop_distance.current_measurement = 0;

			if (data->offline_experiment_context.stop_distance.current_speed_index <
			    data->offline_experiment_context.stop_distance.speed_count) {
				// More speeds to test
				data->offline_experiment_context.stop_distance.state =
					OFFLINE_STOP_DISTANCE_STATE_ACCELERATING_TO_SPEED;

				log_info("Train %d: Moving to next speed level (%d/%d)", data->train_id,
					 data->offline_experiment_context.stop_distance.current_speed_index + 1,
					 data->offline_experiment_context.stop_distance.speed_count);
			} else {
				// All speeds complete
				data->offline_experiment_context.stop_distance.state = OFFLINE_STOP_DISTANCE_STATE_DONE;
				train_offline_cleanup_experiment(data);
				train_navigate_to_destination(data, "A6", 10);
			}
		}
		break;

	case OFFLINE_STOP_DISTANCE_STATE_DONE:
		train_offline_cleanup_experiment(data);
		train_navigate_to_destination(data, "A6", 10);
		return;
	}
}

void train_process_offline_stop_distance_sensor_update(train_task_data_t *data, const track_node *sensor_node)
{
	if (!data || !data->offline_experiment_active || !sensor_node) {
		return;
	}

	if (data->offline_experiment_context.type != OFFLINE_EXPERIMENT_STOP_DISTANCE) {
		return;
	}

	// Handle A3 sensor - start of stop distance measurement
	if (strcmp(sensor_node->name, "A3") == 0) {
		switch (data->offline_experiment_context.stop_distance.state) {
		case OFFLINE_STOP_DISTANCE_STATE_NAVIGATING_TO_A3:
			// Arrived at A3
			data->offline_experiment_context.stop_distance.state =
				OFFLINE_STOP_DISTANCE_STATE_AT_A3_PREPARING;
			train_stop(data);
			log_info("Train %d: Arrived at A3 for stop distance experiment", data->train_id);
			break;

		case OFFLINE_STOP_DISTANCE_STATE_WAITING_FOR_A3:
			// Start binary search iteration
			data->offline_experiment_context.stop_distance.a3_trigger_time = Time(data->clock_server_tid);
			data->offline_experiment_context.stop_distance.search_timeout =
				data->offline_experiment_context.stop_distance.a3_trigger_time +
				STOP_DISTANCE_TIMEOUT_TICKS;
			data->offline_experiment_context.stop_distance.e7_triggered = false;
			data->offline_experiment_context.stop_distance.e7_final_state = false;

			u8 current_speed =
				data->offline_experiment_context.stop_distance
					.speed_levels[data->offline_experiment_context.stop_distance.current_speed_index];

			log_debug("Train %d: A3 triggered for speed %d, will delay %lld ticks before stop",
				  data->train_id, current_speed,
				  data->offline_experiment_context.stop_distance.delay_current);

			data->offline_experiment_context.stop_distance.state =
				OFFLINE_STOP_DISTANCE_STATE_BINARY_SEARCH;
			break;

		default:
			break;
		}
	}

	// Handle E7 sensor - end of stop distance measurement
	else if (strcmp(sensor_node->name, "E7") == 0) {
		if (data->offline_experiment_context.stop_distance.state == OFFLINE_STOP_DISTANCE_STATE_BINARY_SEARCH) {
			data->offline_experiment_context.stop_distance.e7_triggered = true;

			data->offline_experiment_context.stop_distance.e7_final_state = false;

			log_debug("Train %d: E7 sensor update during binary search", data->train_id);
		}
	}
}
