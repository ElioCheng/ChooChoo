#ifndef MARKLIN_TRAIN_MODEL_H
#define MARKLIN_TRAIN_MODEL_H

#include "marklin/train/kinematics.h"
#include "marklin/common/track_node.h"
#include "marklin/error.h"
#include "marklin/train2/train.h"

// ############################################################################
// # Model Management API
// ############################################################################

// Initialize the kinematic model system
marklin_error_t kinematic_model_init(void);

// Get or create model for a specific train
train_kinematic_model_t *kinematic_model_get(train_task_data_t *train_data);

// Initialize a new train model with default values
marklin_error_t kinematic_model_create_default(train_task_data_t *train_data);

// Update acceleration parameters
marklin_error_t kinematic_model_update_acceleration(train_task_data_t *train_data, u8 from_speed_index,
						    u8 to_speed_index, kinematic_accel_t acceleration);

// Update stopping parameters
marklin_error_t kinematic_model_update_stopping(train_task_data_t *train_data, u8 speed_level_index,
						kinematic_distance_t stop_distance, kinematic_time_t stop_time);

// Get velocity for specific speed level and transition direction
kinematic_velocity_t kinematic_model_get_velocity(train_task_data_t *train_data, u8 speed_level,
						  bool from_higher_speed);

// Get acceleration between two speed levels
kinematic_accel_t kinematic_model_get_acceleration(train_task_data_t *train_data, u8 from_speed, bool from_higher_speed,
						   u8 to_speed);

// Get stopping distance for current speed
kinematic_distance_t kinematic_model_get_stop_distance(train_task_data_t *train_data, u8 speed_level,
						       bool from_higher_speed);

// Get stopping time for current speed
kinematic_time_t kinematic_model_get_stop_time(train_task_data_t *train_data, u8 speed_level, bool from_higher_speed);

// ############################################################################
// # Model Persistence API
// ############################################################################

// Export model data to console (for offline analysis)
marklin_error_t kinematic_model_export_console(train_task_data_t *train_data);

// Export all models to console
marklin_error_t kinematic_model_export_all_console(void);

// Print model data in model_defaults.c format for easy copy-paste
marklin_error_t kinematic_model_print_defaults(train_task_data_t *train_data);

#endif /* MARKLIN_TRAIN_MODEL_H */
