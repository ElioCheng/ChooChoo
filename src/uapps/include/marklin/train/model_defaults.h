#ifndef MARKLIN_TRAIN_MODEL_DEFAULTS_H
#define MARKLIN_TRAIN_MODEL_DEFAULTS_H

#include "marklin/train/kinematics.h"
#include "marklin/train/model.h"

// ############################################################################
// # Train Model Defaults Interface
// # Provides access to predefined kinematic parameters for all trains
// ############################################################################

// Complete train model initialization data
typedef struct {
	u8 train_id;
	const kinematic_speed_params_t *default_speeds;
} train_model_defaults_t;

// Get default model parameters for a specific train
const train_model_defaults_t *get_train_model_defaults(u8 train_id);

#endif /* MARKLIN_TRAIN_MODEL_DEFAULTS_H */
// clang-format on
