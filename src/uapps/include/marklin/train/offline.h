#ifndef MARKLIN_TRAIN_OFFLINE_H
#define MARKLIN_TRAIN_OFFLINE_H

#include "marklin/train/calibration.h"
#include "marklin/train/model.h"
#include "marklin/train/kinematics.h"
#include "marklin/error.h"
#include "marklin/train/train.h"

// ############################################################################
// # Offline Experiment Control API
// ############################################################################
// Offline experiment integration functions
marklin_error_t train_start_offline_experiment(train_task_data_t *data);
void train_update_offline_experiment(train_task_data_t *data);
void train_process_offline_sensor_update(train_task_data_t *data, const track_node *sensor_node);

marklin_error_t train_offline_cleanup_experiment(train_task_data_t *data);
kinematic_distance_t train_offline_calculate_path_distance(train_task_data_t *data, const track_node *from,
							   const track_node *to);

// Loop-based velocity measurement between B5 and C11 sensors
marklin_error_t offline_experiment_velocity_loop(const offline_experiment_context_t *context);
void train_update_offline_velocity_experiment(train_task_data_t *data);
void train_process_offline_velocity_sensor_update(train_task_data_t *data, const track_node *sensor_node);
marklin_error_t train_start_offline_velocity_experiment(train_task_data_t *data);

// Acceleration measurement between B5 and D3 sensors
marklin_error_t train_start_offline_acceleration_experiment(train_task_data_t *data);
void train_update_offline_acceleration_experiment(train_task_data_t *data);
void train_process_offline_acceleration_sensor_update(train_task_data_t *data, const track_node *sensor_node);

// Stop distance measurement between C13 and E7 sensors using binary search
marklin_error_t train_start_offline_stop_distance_experiment(train_task_data_t *data);
void train_update_offline_stop_distance_experiment(train_task_data_t *data);
void train_process_offline_stop_distance_sensor_update(train_task_data_t *data, const track_node *sensor_node);

// ############################################################################
// # Utility Functions
// ############################################################################

// Create default experiment configuration
marklin_error_t offline_create_default_config(offline_experiment_type_t type, offline_experiment_context_t *context);

#endif /* MARKLIN_TRAIN_OFFLINE_H */
