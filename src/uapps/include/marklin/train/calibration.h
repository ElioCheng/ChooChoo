#ifndef MARKLIN_TRAIN_CALIBRATION_H
#define MARKLIN_TRAIN_CALIBRATION_H

#include "marklin/train/kinematics.h"
#include "marklin/common/track_node.h"
#include "marklin/error.h"
#include "marklin/train/train.h"

// ############################################################################
// # Online Calibration System API
// ############################################################################

// Initialize online calibration system for a train
marklin_error_t train_online_calibration_init(train_task_data_t *data);

// Process a sensor trigger for calibration purposes
void train_online_calibration_update(train_task_data_t *data, const track_node *triggered_sensor,
				     kinematic_time_t trigger_time);

// Start calibration measurements
void train_online_calibration_start_speed_measurement(train_task_data_t *data, u8 speed_level, bool from_higher);
void train_online_calibration_start_acceleration_measurement(train_task_data_t *data, u8 from_speed, u8 to_speed);

// Stop calibration measurements
void train_online_calibration_stop_measurement(train_task_data_t *data);

// Internal calibration functions
void train_online_calibration_process_velocity_measurement(train_task_data_t *data, kinematic_velocity_t velocity,
							   u8 speed_level, bool from_higher);
void train_online_calibration_process_acceleration_measurement(train_task_data_t *data, kinematic_accel_t acceleration,
							       u8 from_speed, u8 to_speed);
bool train_online_calibration_validate_velocity(kinematic_velocity_t velocity);
bool train_online_calibration_validate_acceleration(kinematic_accel_t acceleration);

#endif /* MARKLIN_TRAIN_CALIBRATION_H */
