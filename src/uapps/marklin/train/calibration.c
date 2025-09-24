#include "marklin/train/calibration.h"
#include "compiler.h"
#include "marklin/train/train.h"
#include "marklin/train/model.h"
#include "marklin/conductor/api.h"
#include "marklin/topology/track.h"
#include "string.h"
#define LOG_MODULE "TRAIN_CALIBRATION"
#define LOG_LEVEL LOG_LEVEL_ERROR
#include "log.h"

// ############################################################################
// # Online Calibration System Implementation
// ############################################################################

marklin_error_t train_online_calibration_init(train_task_data_t *data)
{
	if (!data) {
		return MARKLIN_ERROR_INVALID_ARGUMENT;
	}

	// Initialize calibration context
	memset(&data->online_calibration, 0, sizeof(online_calibration_context_t));

	// Enable calibration by default
	data->online_calibration.enabled = true;

	// Initialize tracking state
	data->online_calibration.last_sensor = NULL;
	data->online_calibration.last_sensor_time = 0;
	data->online_calibration.measuring_speed = 0;
	data->online_calibration.from_higher_speed = false;
	data->online_calibration.measuring_acceleration = false;
	data->online_calibration.accel_from_speed = 0;
	data->online_calibration.accel_to_speed = 0;
	data->online_calibration.speed_change_start_time = 0;

	log_info("Train %d: Online calibration system initialized", data->train_id);
	return MARKLIN_ERROR_OK;
}

void train_online_calibration_update(train_task_data_t *data, const track_node *triggered_sensor,
				     kinematic_time_t trigger_time)
{
	if (!data || !triggered_sensor || !data->online_calibration.enabled) {
		return;
	}

	// Skip blacklisted sensors
	if (train_is_sensor_blacklisted(data, triggered_sensor)) {
		log_debug("Train %d: Skipping blacklisted sensor %s for calibration", data->train_id,
			  triggered_sensor->name);
		return;
	}

	// If we have a previous sensor, calculate distance and time
	if (data->online_calibration.last_sensor && data->online_calibration.last_sensor_time > 0) {
		kinematic_distance_t raw_distance, effective_distance;
		marklin_error_t result = Marklin_CalculateTrackDistance(data->online_calibration.last_sensor,
									triggered_sensor, data->train_id, &raw_distance,
									&effective_distance);

		if (result == MARKLIN_ERROR_OK && effective_distance > 0) {
			kinematic_time_t travel_time = trigger_time - data->online_calibration.last_sensor_time;

			if (travel_time > 0) {
				// Calculate velocity using effective distance for physics-based calibration
				kinematic_velocity_t measured_velocity =
					kinematic_velocity(effective_distance, travel_time);

				// Process velocity measurement if we're at steady speed
				if (!data->online_calibration.measuring_acceleration && data->motion.actual_speed > 0) {
					u8 speed_index = kinematic_speed_to_index(
						data->motion.actual_speed, data->online_calibration.from_higher_speed);
					train_online_calibration_process_velocity_measurement(
						data, measured_velocity, data->motion.actual_speed,
						data->online_calibration.from_higher_speed);

					log_debug(
						"Train %d: Velocity measurement - sensor %s to %s: %lld mm in %lld ticks (speed %d, index %d)",
						data->train_id, data->online_calibration.last_sensor->name,
						triggered_sensor->name, effective_distance, travel_time,
						data->motion.actual_speed, speed_index);
				}

				// Process acceleration measurement if we're in transition
				if (data->online_calibration.measuring_acceleration &&
				    data->online_calibration.speed_change_start_time > 0) {
					kinematic_time_t accel_time =
						trigger_time - data->online_calibration.speed_change_start_time;
					if (accel_time > 0) {
						// Calculate acceleration based on speed change and time
						kinematic_velocity_t from_velocity = kinematic_model_get_velocity(
							data, data->online_calibration.accel_from_speed, false);
						kinematic_velocity_t to_velocity = kinematic_model_get_velocity(
							data, data->online_calibration.accel_to_speed, false);

						if (from_velocity > 0 && to_velocity > 0) {
							// TODO: Follow offline_acceleration calculation to get the acceleration

							// kinematic_accel_t measured_acceleration =
							// 	kinematic_acceleration(from_velocity, to_velocity,
							// 			       accel_time);
							// train_online_calibration_process_acceleration_measurement(
							// 	data, measured_acceleration,
							// 	data->online_calibration.accel_from_speed,
							// 	data->online_calibration.accel_to_speed);

							// log_debug(
							// 	"Train %d: Acceleration measurement - %d to %d: %lld ticks",
							// 	data->train_id,
							// 	data->online_calibration.accel_from_speed,
							// 	data->online_calibration.accel_to_speed, accel_time);
						}
					}
				}
			}
		}
	}

	data->online_calibration.last_sensor = triggered_sensor;
	data->online_calibration.last_sensor_time = trigger_time;
}

void train_online_calibration_process_velocity_measurement(train_task_data_t *data, kinematic_velocity_t velocity,
							   u8 speed_level, bool from_higher)
{
	if (!data || !train_online_calibration_validate_velocity(velocity)) {
		return;
	}

	u8 speed_index = kinematic_speed_to_index(speed_level, from_higher);
	if (speed_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return;
	}

	kinematic_measurement_t measurement = { .timestamp = data->online_calibration.last_sensor_time,
						.velocity = velocity,
						.speed_level = speed_level,
						.from_higher_speed = from_higher,
						.start_sensor = data->online_calibration.last_sensor,
						.end_sensor = data->online_calibration.last_sensor };

	kinematic_model_update_velocity(data, speed_level, from_higher, &measurement);

	log_info("Train %d: Updated velocity model for speed %d (from_higher=%d): %lld", data->train_id, speed_level,
		 from_higher, velocity);
}

void train_online_calibration_process_acceleration_measurement(train_task_data_t *data, kinematic_accel_t acceleration,
							       u8 from_speed, u8 to_speed)
{
	if (!data || !train_online_calibration_validate_acceleration(acceleration)) {
		return;
	}

	u8 from_index = kinematic_speed_to_index(from_speed, false);
	u8 to_index = kinematic_speed_to_index(to_speed, false);

	if (from_index >= KINEMATIC_TOTAL_SPEED_LEVELS || to_index >= KINEMATIC_TOTAL_SPEED_LEVELS) {
		return;
	}

	kinematic_model_update_acceleration(data, from_index, to_index, acceleration);

	log_info("Train %d: Updated acceleration model from speed %d to %d: %lld", data->train_id, from_speed, to_speed,
		 acceleration);

	data->online_calibration.measuring_acceleration = false;
}

bool train_online_calibration_validate_velocity(kinematic_velocity_t velocity)
{
	// Sanity check: velocity should be positive and reasonable
	const kinematic_velocity_t max_reasonable_velocity = 7 * KINEMATIC_VELOCITY_SCALE_FACTOR;
	const kinematic_velocity_t min_reasonable_velocity = 1 * KINEMATIC_VELOCITY_SCALE_FACTOR;

	return velocity > min_reasonable_velocity && velocity < max_reasonable_velocity;
}

bool train_online_calibration_validate_acceleration(kinematic_accel_t acceleration)
{
	UNUSED(acceleration);
	return false;
}

void train_online_calibration_start_speed_measurement(train_task_data_t *data, u8 speed_level, bool from_higher)
{
	if (!data || !data->online_calibration.enabled || speed_level == data->motion.actual_speed) {
		return;
	}

	if (data->online_calibration.measuring_speed != speed_level ||
	    data->online_calibration.from_higher_speed != from_higher) {
		train_online_calibration_stop_measurement(data);
		data->online_calibration.measuring_speed = speed_level;
		data->online_calibration.from_higher_speed = from_higher;
		data->online_calibration.measuring_acceleration = false;
	}

	log_debug("Train %d: Started velocity measurement for speed %d (from_higher=%d)", data->train_id, speed_level,
		  from_higher);
}

void train_online_calibration_start_acceleration_measurement(train_task_data_t *data, u8 from_speed, u8 to_speed)
{
	if (!data || !data->online_calibration.enabled) {
		return;
	}

	data->online_calibration.measuring_acceleration = true;
	data->online_calibration.accel_from_speed = from_speed;
	data->online_calibration.accel_to_speed = to_speed;
	data->online_calibration.speed_change_start_time = data->online_calibration.last_sensor_time;

	log_debug("Train %d: Started acceleration measurement from speed %d to %d", data->train_id, from_speed,
		  to_speed);
}

void train_online_calibration_stop_measurement(train_task_data_t *data)
{
	if (!data || !data->online_calibration.enabled) {
		return;
	}

	data->online_calibration.measuring_speed = 0;
	data->online_calibration.measuring_acceleration = false;
	data->online_calibration.accel_from_speed = 0;
	data->online_calibration.accel_to_speed = 0;
	data->online_calibration.speed_change_start_time = 0;

	data->online_calibration.last_sensor_time = 0;
	data->online_calibration.last_sensor = NULL;
}
